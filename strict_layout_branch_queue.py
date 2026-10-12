"""Durable finite Boolean partition for the complete wrench relaxation.

Leaves partition the healthy even-group Cartesian product exactly. Only
independently checked rational certificates close leaves. Z3 UNSAT proofs
without an independent checker are retained in a separate review queue.
An unknown non-leaf splits into BOTH children; an interrupted active leaf
remains pending on disk. Progress counts certified excluded assignments,
not elapsed time, solver visits, physical volume, or solution probability.
"""
import copy
import hashlib
import itertools
import json
import math
import time
from fractions import Fraction as F
from functools import lru_cache
from pathlib import Path

from strict_layout_diagnosis import (atomic_json,build_z3,cases,coefficient_bounds,
    rational,scope,signature,verify_witness,witness_from_z3)
from strict_layout_topology import mask_rows,span_weights,verify_conflict

VERSION='certified_boolean_partition_v1'
CASE_ORDER=tuple(12*f+label for label in (0,1,6,7,2,3,4,5,8,9,10,11) for f in range(3))
COORDINATES=tuple((case,i) for case in CASE_ORDER for i in range(12) if i!=cases('wrench')[case][0])


def normalize_conflict(proof):
    proof=copy.deepcopy(proof)
    if proof['assignments'] and not isinstance(proof['assignments'][0],(list,tuple)):
        proof['assignments']=[proof['assignments']]
    if isinstance(proof['basis_cases'],(int,float)): proof['basis_cases']=[int(proof['basis_cases'])]
    if isinstance(proof['weights'],str): proof['weights']=[proof['weights']]
    proof['assignments']=sorted([[int(c),int(m)] for c,m in proof['assignments']])
    return proof


def conflict_key(proof):
    return hashlib.sha256(json.dumps(proof['assignments'],sort_keys=True).encode()).hexdigest()


def import_conflicts(config):
    root=Path(__file__).resolve().parent
    paths=list(root.glob('Optim_search_*.conflicts.json'))
    paths+=list((root/'analysis').glob('topology*checkpoint*.json'))
    paths+=list(root.glob('Optim_diagnosis_*/branch_queue.json'))
    extra=config.get('diagnostic_conflict_files',[])
    if isinstance(extra,str): extra=[extra]
    paths+=list(map(Path,extra))
    result={}; sources=[]; rejected=0
    for path in sorted(set(paths)):
        try:
            record=json.loads(path.read_text())
            proofs=record.get('conflicts',record.get('search_state',{}).get('conflicts',[]))
            if isinstance(proofs,dict): proofs=[proofs]
            accepted=0
            for raw in proofs:
                try:
                    proof=normalize_conflict(raw)
                    if not verify_conflict(proof,config): rejected+=1;continue
                    result[conflict_key(proof)]=proof;accepted+=1
                except (KeyError,TypeError,ValueError): rejected+=1
            if accepted: sources.append(dict(file=str(path.resolve()),accepted=accepted))
        except (OSError,ValueError,TypeError): rejected+=1
    return list(result.values()),dict(sources=sources,rejected=rejected)


class Domain:
    def __init__(self,config):
        self.config=config;self.maximum=int(config.get('max_group_size',6))
        self.universe={}
        for fault in (0,4,8):
            ids=[i for i in range(12) if i!=fault]
            self.universe[fault]=tuple(sum(1<<i for i in group) for n in range(2,self.maximum+1,2)
                for group in itertools.combinations(ids,n))
        self.total=len(self.universe[0])**36
        self.bounds=coefficient_bounds(config,'wrench')
        self.required=rational(config['min_axis_capacity'])-rational(config.get('pure_tolerance',1e-9))
        self.good={}
        rows={mask:mask_rows(mask) for masks in self.universe.values() for mask in masks}
        for case,(fault,label) in enumerate(cases('wrench')):
            row=label//2;sign=1 if label%2 else -1; good=set()
            for mask in self.universe[fault]:
                weights=[sign*int(v) for v in rows[mask][row]]
                upper=sum((w*self.bounds[m][row][1 if w>=0 else 0] for m,w in enumerate(weights)),F(0))
                if upper>=self.required: good.add(mask)
            self.good[case]=good

    @lru_cache(maxsize=8192)
    def candidates(self,case,bits):
        fault=cases('wrench')[case][0];ids=[i for i in range(12) if i!=fault]
        on=sum(1<<i for i,b in zip(ids,bits) if b=='1');known=sum(1<<i for i in ids[:len(bits)])
        return tuple(m for m in self.universe[fault] if m&known==on)

    def chunks(self,prefix):
        return {case:prefix[11*j:11*(j+1)] for j,case in enumerate(CASE_ORDER)}

    def domains(self,prefix):
        return {c:self.candidates(c,b) for c,b in self.chunks(prefix).items()}

    def mass(self,prefix):
        if not isinstance(prefix,str) or len(prefix)>396 or set(prefix)-{'0','1'}:
            raise ValueError('Invalid Boolean branch prefix.')
        return math.prod(len(v) for v in self.domains(prefix).values())

    def cheap_certificate(self,prefix,conflicts):
        domains=self.domains(prefix)
        for case,values in domains.items():
            if not self.good[case].intersection(values):
                return dict(kind='target_upper_bound',case=case)
        fixed={case:values[0] for case,values in domains.items() if len(values)==1}
        for key,proof in conflicts.items():
            if all(fixed.get(case)==mask for case,mask in proof['assignments']):
                return dict(kind='rational_conflict',conflict=key)
        return None

    def verify_certificate(self,prefix,certificate,conflicts):
        domains=self.domains(prefix)
        if certificate.get('kind')=='target_upper_bound':
            case=certificate.get('case')
            return case in domains and not self.good[case].intersection(domains[case])
        if certificate.get('kind')=='rational_conflict':
            proof=conflicts.get(certificate.get('conflict'))
            return proof is not None and all(domains[case]==(mask,) for case,mask in proof['assignments'])
        return False

    def derive_conflicts(self,prefix):
        fixed={case:masks[0] for case,masks in self.domains(prefix).items() if len(masks)==1}
        eps=rational(self.config.get('pure_tolerance',1e-9));result=[]
        rows={j:mask_rows(m) for j,m in fixed.items()}
        for axis in range(6):
            basis=[];ids=[]
            for case in sorted(fixed):
                if cases('wrench')[case][1]//2!=axis and span_weights(basis,rows[case][axis]) is None:
                    basis.append(rows[case][axis].tolist());ids.append(case)
            for case in sorted(fixed):
                if cases('wrench')[case][1]//2!=axis: continue
                weights=span_weights(basis,rows[case][axis])
                if weights is None or eps*sum(map(abs,weights))>=self.required: continue
                used=[(i,w) for i,w in enumerate(weights) if w]
                proof=dict(kind='rational_span_bound',axis=axis,target_case=case,
                    basis_cases=[ids[i] for i,w in used],weights=[str(w) for i,w in used],
                    assignments=[[ids[i],fixed[ids[i]]] for i,w in used]+[[case,fixed[case]]],
                    capacity_upper=str(eps*sum(map(abs,weights))))
                proof=normalize_conflict(proof)
                if not verify_conflict(proof,self.config): raise ValueError('Derived conflict failed exact verification.')
                result.append(proof)
        return result


def prepare_queue(config,previous=None):
    domain=Domain(config); note='new_branch_queue'
    if previous and previous.get('method')==VERSION and previous.get('scope_signature')==signature(config):
        state=copy.deepcopy(previous);note='resumed_branch_queue'
    else:
        state=dict(method=VERSION,scope=scope(config),scope_signature=signature(config),conflicts=[],
            leaves=[dict(prefix='',status='pending',trials=0)],calls=0,splits=0,total_solver_seconds=0,history=[])
        if previous: note='constraints_or_method_changed_branch_queue_reset'
    conflicts={}
    for raw in state['conflicts']:
        proof=normalize_conflict(raw)
        if not verify_conflict(proof,config): raise ValueError('Saved branch conflict failed independent verification.')
        conflicts[conflict_key(proof)]=proof
    leaves=state['leaves']; prefixes=sorted(n['prefix'] for n in leaves)
    if len(set(prefixes))!=len(prefixes) or any(b.startswith(a) for a,b in zip(prefixes,prefixes[1:])):
        raise ValueError('Overlapping saved branch regions.')
    if sum(domain.mass(p) for p in prefixes)!=domain.total or any(domain.mass(p)==0 for p in prefixes):
        raise ValueError('Saved branches do not cover the original finite domain.')
    for node in leaves:
        if node['status'] not in ('pending','review','closed') or not isinstance(node['trials'],int) or node['trials']<0:
            raise ValueError('Invalid branch state.')
        if node['status']=='closed' and not domain.verify_certificate(node['prefix'],node.get('certificate',{}),conflicts):
            raise ValueError('Saved exclusion failed independent verification.')
    if state.get('solution') and not verify_witness(state['solution'],config,'wrench')['valid']:
        raise ValueError('Saved complete witness failed independent verification.')
    return state,domain,conflicts,note


def branch_progress(state,domain):
    masses={kind:sum(domain.mass(n['prefix']) for n in state['leaves'] if n['status']==kind) for kind in ('closed','review','pending')}
    fraction=F(masses['closed'],domain.total)
    return dict(total_assignments=str(domain.total),verified_excluded_assignments=str(masses['closed']),
        reviewed_later_assignments=str(masses['review']),remaining_assignments=str(masses['pending']),
        certified_fraction=str(fraction),certified_percent=float(100*fraction),
        pending_branches=sum(n['status']=='pending' for n in state['leaves']),
        review_branches=sum(n['status']=='review' for n in state['leaves']),
        closed_branches=sum(n['status']=='closed' for n in state['leaves']),
        conflicts=len(state['conflicts']),solver_calls=state['calls'],splits=state['splits'])


def screen_first_case(state,domain):
    """Partition one requirement using exact capacity bounds before SMT.

    This small tree has at most 2^11 leaves. Stop at homogeneous subtrees;
    every closure carries a replayable bound certificate. Do not expand
    all 36 cases into their astronomically large Cartesian product.
    """
    todo=[n for n in state['leaves'] if n['status']=='pending' and len(n['prefix'])<=11]
    for node in todo:
        prefix=node['prefix'];values=domain.candidates(CASE_ORDER[0],prefix)
        good=domain.good[CASE_ORDER[0]].intersection(values)
        if not good:
            node.update(status='closed',certificate=dict(kind='target_upper_bound',case=CASE_ORDER[0]))
        elif len(good)<len(values):
            state['leaves'].remove(node)
            for bit in '01':
                if domain.candidates(CASE_ORDER[0],prefix+bit):
                    child=dict(prefix=prefix+bit,status='pending',trials=0)
                    state['leaves'].append(child);todo.append(child)
            state['splits']+=1


def add_conflict(solver,groups,proof):
    import z3
    solver.add(z3.Or([z3.Not(b) if mask>>i&1 else b for case,mask in proof['assignments']
        for i,b in enumerate(groups[case]) if i!=cases('wrench')[case][0]]))


def check_branch(solver,coefficients,groups,scale,config,prefix,seconds):
    import z3
    literals=[groups[c][i] if value=='1' else z3.Not(groups[c][i]) for (c,i),value in zip(COORDINATES,prefix)]
    solver.set(timeout=max(1,int(seconds*1000)))
    if config.get('diagnostic_rlimit') is not None: solver.set(rlimit=int(config['diagnostic_rlimit']))
    status=solver.check(*literals)
    if status==z3.sat:
        witness=witness_from_z3(solver.model(),coefficients,groups,scale,config,'wrench')
        checked=verify_witness(witness,config,'wrench')
        if not checked['valid']: raise ValueError('SMT branch witness failed independent full-fault verification.')
        return dict(status='sat_verified',witness=witness,verification=checked)
    if status==z3.unsat:
        export=z3.SolverFor('QF_LRA',ctx=solver.ctx);export.add(*solver.assertions(),*literals)
        return dict(status='unsat_reported',formula='(set-logic QF_LRA)\n'+export.to_smt2(),proof=solver.proof().sexpr())
    return dict(status='unknown',reason=solver.reason_unknown())


def solve_persistent(config,seconds,artifact_dir=None):
    started=time.monotonic();previous=None;folder=Path(artifact_dir) if artifact_dir else None
    base=float(config.get('diagnostic_branch_time',30))
    if not math.isfinite(base) or base<=0: raise ValueError('Branch time must be positive and finite.')
    if config.get('resume_file'):
        report=config.get('_diagnostic_previous_report')
        if report is None: report=json.loads(Path(config['resume_file']).read_text())
        path=report.get('branch_queue_file')
        if path: previous=json.loads(Path(path).read_text())
        elif report.get('queue_state'): previous=report['queue_state']
    state,domain,conflicts,note=prepare_queue(config,previous)
    imported,source_info=import_conflicts(config)
    for p in imported: conflicts[conflict_key(p)]=p
    state['conflicts']=list(conflicts.values())
    if config.get('diagnostic_initial_screen',True): screen_first_case(state,domain)
    output=dict(engine='z3_persistent',status='unknown',resume_note=note,imported_conflicts=len(imported),
        conflict_sources=source_info,independent_unsat_proof_verified=False)
    def persist():
        state['conflicts']=list(conflicts.values());progress=branch_progress(state,domain)
        output['branch_progress']=progress
        if folder:
            folder.mkdir(parents=True,exist_ok=True);path=folder/'branch_queue.json'
            atomic_json(path,state);output['branch_queue_file']=str(path.resolve())
        else: output['queue_state']=state
        if config.get('_diagnostic_queue_callback'): config['_diagnostic_queue_callback'](output)
        if config.get('progress_path'):
            atomic_json(config['progress_path'],dict(stage='diagnostic_branches',coverage=0,attempt=state['calls'],branch_progress=progress))
    persist()
    if state.get('solution'):
        output.update(status='sat_verified',witness=state['solution'],verification=verify_witness(state['solution'],config,'wrench'))
        output['elapsed_seconds']=time.monotonic()-started;return output
    solver=None
    while time.monotonic()-started<seconds:
        pending=[n for n in state['leaves'] if n['status']=='pending']
        if not pending: break
        node=min(pending,key=lambda n:(n['trials'],len(n['prefix']),n['prefix']))
        prefix=node['prefix']
        for proof in domain.derive_conflicts(prefix):
            key=conflict_key(proof)
            if key not in conflicts:
                conflicts[key]=proof
                if solver is not None: add_conflict(solver,groups,proof)
        certificate=domain.cheap_certificate(prefix,conflicts)
        if certificate:
            node.update(status='closed',certificate=certificate);persist();continue
        if solver is None:
            solver,c,groups,scale=build_z3(config,'wrench')
            solver.set(random_seed=int(config.get('seed',18))%2147483647)
            for proof in conflicts.values(): add_conflict(solver,groups,proof)
        remaining=seconds-(time.monotonic()-started)
        if remaining<=0: break
        budget=min(remaining,base*2**min(node['trials'],6))
        # The active node stays pending on disk until its outcome is committed.
        persist();tick=time.monotonic();trial=check_branch(solver,c,groups,scale,config,prefix,budget)
        state['total_solver_seconds']+=time.monotonic()-tick;state['calls']+=1;node['trials']+=1
        state['history'].append(dict(prefix=prefix,status=trial['status'],budget=budget))
        state['history']=state['history'][-128:]
        if trial['status']=='sat_verified':
            state['solution']=trial['witness'];output.update(trial);persist();break
        if trial['status']=='unsat_reported':
            node.update(status='review',reason='SMT proof awaits independent checking')
            if folder:
                stem='branch_'+hashlib.sha256(prefix.encode()).hexdigest()[:20]
                for key,suffix in (('formula','.smt2'),('proof','.z3proof')):
                    path=folder/(stem+suffix);path.write_text(trial[key]);node[key+'_file']=str(path.resolve())
                    node[key+'_sha256']=hashlib.sha256(trial[key].encode()).hexdigest()
            else: node.update(formula=trial['formula'],proof=trial['proof'])
        elif len(prefix)<len(COORDINATES):
            state['leaves'].remove(node)
            for bit in '01':
                child=prefix+bit
                if domain.mass(child)>0: state['leaves'].append(dict(prefix=child,status='pending',trials=0))
            state['splits']+=1
        persist()
    progress=branch_progress(state,domain)
    if output['status']!='sat_verified' and progress['pending_branches']==0:
        if progress['review_branches']:
            output['status']='review_required'
        else:
            # prepare_queue independently re-verifies all closures and exact coverage.
            prepare_queue(config,state)
            output.update(status='unsat_verified',independent_unsat_proof_verified=True)
    persist();output['elapsed_seconds']=time.monotonic()-started
    return output


def verify_queue_file(path):
    state=json.loads(Path(path).read_text())
    checked,domain,_,note=prepare_queue(state['scope'],state)
    if note!='resumed_branch_queue': raise ValueError('Saved queue method or scope signature is inconsistent.')
    return dict(valid=True,progress=branch_progress(checked,domain),
        whole_domain_unsat=all(n['status']=='closed' for n in checked['leaves']))
