"""Persistent fair search over certified surviving strict installation supports.

Each positive-margin box is an INNER domain, never a proof of whole-support
infeasibility. A diagonal schedule revisits old boxes and introduces margins
tending to zero. Numerical failures never delete a support or subjob.
"""
import copy
import itertools
import json
import math
import time
import fcntl
import shutil
from strict_layout_direct import search as search_direct
from pathlib import Path

from strict_layout_structure import screen, strict_installation_domain
from strict_layout_topology import solve_decomposed, evaluate_candidate, physical_matrix

VERSION='strict_support_rotation_v1'
BASELINE=[.2,0,0,math.pi/4,.6,-.2,math.pi/2,math.pi/4,.6,.2,3*math.pi/2,math.pi/4]


def diagonal_level(visit):
    row=(math.isqrt(8*visit+1)-1)//2
    return visit-row*(row+1)//2


def inner_domain(config, support, level, signs):
    lo=list(config['lower']);hi=list(config['upper'])
    fraction=math.ldexp(1e-4,-min(level,1020))
    for j,b in enumerate(support):
        if not b:
            lo[j]=hi[j]=0
        else:
            delta=fraction*max(abs(lo[j]),abs(hi[j]),1e-12)
            if delta==0: raise ValueError('Margin underflow; exact boundary work is still unresolved.')
            if signs.get(j,1)>0: lo[j]=max(lo[j],delta)
            else: hi[j]=min(hi[j],-delta)
    if any(a>b for a,b in zip(lo,hi)): return None
    return dict(config,lower=lo,upper=hi,strict_support=list(support),order_modules=False,
                strict_collision_distance=fraction*config['x_face'])


def sign_variants(config,support):
    choices=[]
    for j,b in enumerate(support):
        if b:
            signs=([1] if config['upper'][j]>0 else [])+([-1] if config['lower'][j]<0 else [])
            choices.append((j,signs))
    variants=[dict(zip((j for j,_ in choices),s)) for s in itertools.product(*(s for _,s in choices))]
    baseline=canonical_seeds([BASELINE],support)
    variants.sort(key=lambda signs:not any(all(sign*x[j]>0 for j,sign in signs.items()) for x in baseline))
    return variants


def choose_job(entry, variants):
    """Spend three visits deepening existing boxes; one broadens base signs.
    Only introduce a smaller-margin layer after base signs received two new
    visits. Existing old margin layers remain in the fair deepening pool.
    """
    jobs=entry['jobs'];tick=entry.get('adaptive_visits',0)
    if jobs and tick%4!=3:
        key=min(jobs,key=lambda k:(jobs[k].get('adaptive_visits',0),jobs[k]['level'],jobs[k]['sign_index']))
        return key,jobs[key]['level'],jobs[key]['sign_index']
    base=[f'0:{i}' for i in range(len(variants))]
    missing=next((k for k in base if k not in jobs),None)
    if missing: return missing,0,int(missing.split(':')[1])
    if any(jobs[k].get('adaptive_visits',0)<2 for k in base):
        key=min(base,key=lambda k:jobs[k].get('adaptive_visits',0))
        return key,0,jobs[key]['sign_index']
    level=1
    while all(f'{level}:{i}' in jobs for i in range(len(variants))):level+=1
    si=next(i for i in range(len(variants)) if f'{level}:{i}' not in jobs)
    return f'{level}:{si}',level,si


def planned_budget(config,job):
    return min(float(config.get('support_max_slice_seconds',300)),
               float(config.get('support_slice_seconds',120))*2**min(job.get('adaptive_visits',0),3))


def save(path,state,config,current=''):
    data=dict(search_state=state)
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False));tmp.replace(path)
    if config.get('progress_path'):
        p=Path(config['progress_path']);t=p.with_suffix('.tmp')
        t.write_text(json.dumps(dict(stage='strict_support_rotation',coverage=state['best_coverage'],
            attempt=state['visits'],support_total=len(state['entries']),
            support_seen=sum(e['visits']>0 for e in state['entries']),support_current=current,
            support_visits=state['visits'],search_metrics=state.get('metrics',{}),
            search_phase=state.get('phase','ready'),checkpoint_file=str(path.resolve()))))
        t.replace(p)


def canonical_seeds(seeds, support):
    result=[]
    for x in seeds:
        if len(x)!=12 or not all(math.isfinite(v) for v in x): continue
        modules=[list(x[j:j+4]) for j in range(0,12,4)]
        for order in itertools.permutations(modules):
            candidate=sum(order,[])
            if [int(v!=0) for v in candidate]==list(support) and candidate not in result:
                result.append(candidate)
    return result


def run(config):
    folder=Path(config.get('output_dir','.'));folder.mkdir(parents=True,exist_ok=True)
    with (folder/'.support_search.lock').open('a') as lock:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('Another support search is using this output folder; its queue was not modified.') from exc
        return _run(config)


def _run(config):
    start=time.monotonic()
    if not all(0<=config['lower'][4*m+3]<=config['upper'][4*m+3]<=math.pi/2 for m in range(3)):
        raise ValueError('Strict support search currently requires beta in [0,pi/2].')
    report=screen(config)
    signature=report['scope_signature']+'_'+VERSION
    path=Path(config.get('output_dir','.'))/('Optim_support_queue_'+report['scope_signature'][:16]+'.json')
    # Reload the native JSON: MATLAB normalizes dictionary keys such as 0:1.
    state=json.loads(path.read_text())['search_state'] if path.exists() else None
    resumed=state is not None
    if state is not None and (state.get('version')!=VERSION or state.get('signature')!=signature):
        raise ValueError('Support queue version/domain mismatch; use a separate output directory.')
    survivors=[r for r in report['patterns'] if r['status']=='unresolved']
    if state is None:
        # The validated 152/156 candidate belongs to this orbit; prioritize it.
        survivors.sort(key=lambda r:(r['id']!='100111111111',sum(r['support']),r['id']))
        state=dict(version=VERSION,signature=signature,visits=0,best_coverage=0,best_x=[],
                   found_x=[],found_support=[],entries=[dict(id=r['id'],support=list(r['support']),visits=0,jobs={}) for r in survivors])
    expected={r['id']:list(r['support']) for r in survivors}
    if len(state['entries'])!=len(expected) or {e['id']:e['support'] for e in state['entries']}!=expected:
        raise ValueError('Support queue membership was changed or corrupted.')
    if state.get('active_job') and path.with_suffix('.active.json').exists():
        eid,key=state['active_job']
        entry=next(e for e in state['entries'] if e['id']==eid)
        entry['jobs'][key]['state']=json.loads(path.with_suffix('.active.json').read_text())['search_state']
    state.pop('active_job',None)
    if state.get('policy_version')!=2:
        backup=path.with_suffix('.pre_adaptive_v1.json')
        if path.exists() and not backup.exists(): shutil.copy2(path,backup)
        state['policy_version']=2
        state['metrics']={'legacy_visits':state['visits'],'nodes_since_upgrade':0,
                          'direct_calls':0,'direct_evaluations':0,'candidate_checks':0,
                          'geometry_calls_since_upgrade':0,'new_conflicts_since_upgrade':0}
    state.setdefault('metrics',{})
    rejected=state.setdefault('rejected_points',[])
    if config.get('rejected_support_x'):
        point=config['rejected_support_x']
        if point and isinstance(point[0],list): point=sum(point,[])
        if point not in rejected: rejected.append(point)
        if state['found_x']==point: state['found_x']=[]
    result=dict(status='timelimit',has_solution=False,x=[],groups=[],candidates=[],
                method=VERSION,search_state=state,checkpoint_file=str(path.resolve()),
                resume_note='resumed_supports' if resumed else 'new_supports',free_mask=[],nodes=0,solution_count=0)
    if state['found_x']:
        check=evaluate_candidate(state['found_x'],dict(config,strict_support=state['found_support']))
        if check['has_solution']:
            result.update(has_solution=True,status='candidate',x=state['found_x'])
        else: state['found_x']=[]
    save(path,state,config)
    seed_input=config.get('search_seeds',[])
    if seed_input and isinstance(seed_input[0],(int,float)): seed_input=[seed_input]
    seeds=[BASELINE]+list(seed_input)
    stop_visits=state['visits']+int(config.get('support_visit_limit',2147483647))
    while not result['has_solution'] and time.monotonic()-start<float(config['max_time']) and state['entries'] and state['visits']<stop_visits:
        entry=min(state['entries'],key=lambda e:e.get('adaptive_visits',0))
        variants=sign_variants(config,entry['support'])
        key,level,si=choose_job(entry,variants)
        job=entry['jobs'].setdefault(key,dict(level=level,sign_index=si,visits=0,state=None,status='pending'))
        child=inner_domain(config,entry['support'],level,variants[si])
        remaining=float(config['max_time'])-(time.monotonic()-start)
        if remaining<=0: break
        save(path,state,config,entry['id'])
        if child is not None:
            budget=min(remaining,planned_budget(config,job));slice_started=time.monotonic()
            child.update(max_time=budget,master_time=budget,geometry_time=budget,
                         checkpoint_path=str(path.with_suffix('.active.json')),
                         progress_path='',search_state=job['state'])
            child['search_seeds']=[x for x in canonical_seeds(seeds,entry['support']) if strict_installation_domain(x,child,entry['support'])]
            direct=job.setdefault('direct_state',{})
            before_checks=direct.get('candidate_checks',0)
            state['phase']='direct_geometry'
            save(path,state,config,entry['id'])
            direct_budget=min(float(config.get('support_direct_seconds',45)),budget*.25)
            last_save=[0.0]
            def progress():
                if time.monotonic()-last_save[0]>2:
                    save(path,state,config,entry['id']);last_save[0]=time.monotonic()
            direct_trial=search_direct(child,direct,direct_budget,child['search_seeds'],progress) if direct_budget>0 else dict(candidates=[],evaluations=0,has_solution=False)
            metrics=state['metrics']
            metrics['direct_calls']+=int(direct_budget>0)
            metrics['direct_evaluations']+=direct_trial['evaluations']
            metrics['candidate_checks']+=direct.get('candidate_checks',0)-before_checks
            candidates=direct_trial['candidates']
            # Persist actual geometry effort and seeds BEFORE entering the master.
            save(path,state,config,entry['id'])
            remaining=min(budget-(time.monotonic()-slice_started),float(config['max_time'])-(time.monotonic()-start))
            if remaining>.1 and not direct_trial['has_solution']:
                old=job['state'] or {}
                old_geometry=old.get('processed',0);old_conflicts=len(old.get('conflicts',[]))
                if old and direct.get('best_x') and direct['best_x']!=job.get('last_direct_seed'):
                    old['seeded']=False
                    job['last_direct_seed']=direct['best_x']
                child.update(max_time=remaining,master_time=remaining,geometry_time=remaining)
                child['search_seeds']+=candidates
                state['active_job']=[entry['id'],key]
                active=Path(child['checkpoint_path'])
                if active.exists(): active.unlink()
                state['phase']='group_master_or_geometry'
                save(path,state,config,entry['id'])
                trial=solve_decomposed(child)
                job['state']=trial['search_state'];job['status']=trial['status']
                state.pop('active_job',None)
                metrics['nodes_since_upgrade']+=trial.get('nodes',0)
                metrics['geometry_calls_since_upgrade']+=max(0,job['state'].get('processed',0)-old_geometry)
                metrics['new_conflicts_since_upgrade']+=max(0,len(job['state'].get('conflicts',[]))-old_conflicts)
                candidates+=trial.get('candidates',[])+([trial['x']] if trial.get('has_solution') else [])
            else: job['status']='direct_search_finished'
            job['last_budget_seconds']=budget
            job['last_elapsed_seconds']=time.monotonic()-slice_started
            for x in candidates:
                if not strict_installation_domain(x,config,entry['support']): continue
                check=evaluate_candidate(x,dict(config,strict_support=entry['support']))
                if check['coverage']>state['best_coverage']:
                    state['best_coverage']=check['coverage'];state['best_x']=x
                if check['has_solution'] and x not in rejected:
                    state['found_x']=x;state['found_support']=entry['support']
                    result.update(status='candidate',has_solution=True,x=x)
                    break
        else: job['status']='empty_inner_box_not_support_infeasibility'
        job['visits']+=1;entry['visits']+=1;state['visits']+=1
        job['adaptive_visits']=job.get('adaptive_visits',0)+1
        entry['adaptive_visits']=entry.get('adaptive_visits',0)+1
        save(path,state,config,entry['id'])
    state['phase']='saved'
    result['search_metrics']=state['metrics']
    result['nodes']=state['metrics']['nodes_since_upgrade']
    result.update(candidates=[state['best_x']] if state['best_x'] else [],
                  solution_count=int(result['has_solution']),
                  support_total=len(state['entries']),support_seen=sum(e['visits']>0 for e in state['entries']),
                  support_visits=state['visits'],best_coverage=state['best_coverage'],solve_time=time.monotonic()-start)
    save(path,state,config)
    return result
