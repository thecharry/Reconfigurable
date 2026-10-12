"""Necessary-condition diagnostics; a SAT result is never a physical layout.

Both layers are outer relaxations of the tolerant 12-thruster model. Z3 uses
exact rational linear arithmetic and Boolean choices. The optional SCIP run
uses the SAME relaxation, but numerical infeasibility is never a proof.
Witness checking and the elementary zero-row certificate use only stdlib.
"""
import argparse
import hashlib
import itertools
import json
import math
import time
from fractions import Fraction as F
from pathlib import Path

VERSION = 'layered_rational_diagnosis_v1'
SIGNS = ((-1,-1,1,1),(1,-1,1,-1),(1,-1,1,-1),
         (1,1,1,1),(-1,1,1,-1),(1,-1,-1,1))
NAMES = ('c','p','q','h','g','k')
LAYERS = ('force','wrench')


def rational(value):
    return F(str(value))


def dimension(layer):
    if layer not in LAYERS:
        raise ValueError('Layer must be force or wrench.')
    return 3 if layer == 'force' else 6


def cases(layer):
    return tuple((f,k) for f in (0,4,8) for k in range(2*dimension(layer)))


def scope(config):
    lo,hi = [rational(v) for v in config['lower']],[rational(v) for v in config['upper']]
    if len(lo)!=12 or len(hi)!=12 or any(not math.isfinite(float(v)) for v in lo+hi) or any(a>b for a,b in zip(lo,hi)):
        raise ValueError('Twelve finite, ordered bounds are required.')
    a=rational(config['x_face']); eps=rational(config.get('pure_tolerance',1e-9))
    rho=rational(config['min_axis_capacity']); maximum=config.get('max_group_size',6)
    if a<=0 or eps<=0 or rho<=eps or maximum not in (2,4,6):
        raise ValueError('Invalid physical scale, capacity, tolerance, or group size.')
    # Store the exact decimal interpretation used at the JSON interface.
    return dict(lower=[str(rational(v)) for v in lo],upper=[str(rational(v)) for v in hi],
                x_face=str(a),pure_tolerance=str(eps),min_axis_capacity=str(rho),
                max_group_size=int(maximum),
                min_install_distance=str(rational(config.get('min_install_distance',0))),
                min_install_angle_deg=str(rational(config.get('min_install_angle_deg',0))))


def signature(config):
    return hashlib.sha256(json.dumps(scope(config),sort_keys=True).encode()).hexdigest()


def coefficient_bounds(config,layer):
    """Rational OUTER bounds, without rounded sin/cos constants or divisions by c.

    General angular sectors, unit norm and installation distances/angles are
    deliberately omitted. Only exact alpha=0/beta=0 identities are retained.
    Thus an UNSAT result remains necessary for arbitrary input angular boxes.
    """
    s=scope(config); a=F(s['x_face']); result=[]
    for m in range(3):
        lo=list(map(F,s['lower'][4*m:4*m+4])); hi=list(map(F,s['upper'][4*m:4*m+4]))
        ey,ez=max(abs(lo[0]),abs(hi[0])),max(abs(lo[1]),abs(hi[1]))
        hemisphere=0<=lo[3]<=hi[3]<=F(str(math.pi/2))
        bounds=[(F(0) if hemisphere else F(-1),F(1)),(F(-1),F(1)),(F(-1),F(1)),
                (-(ey+ez)/a,(ey+ez)/a),(-1-ez/a,1+ez/a),(-1-ey/a,1+ey/a)]
        if lo[3]==hi[3]==0:
            bounds[:3]=[(F(1),F(1)),(F(0),F(0)),(F(0),F(0))]
        elif lo[2]==hi[2]==0:
            bounds[2]=(F(0),F(0))
            if hemisphere: bounds[1]=(F(0),F(1))
        result.append(bounds[:dimension(layer)])
    return result


def mask_rows(mask,dim):
    return [[sum(SIGNS[r][j]*((mask>>(4*m+j))&1) for j in range(4)) for m in range(3)] for r in range(dim)]


def healthy(mask,fault,maximum):
    return (isinstance(mask,int) and not isinstance(mask,bool) and 0<mask<4096 and
            bin(mask).count('1') in range(2,maximum+1,2) and not ((mask>>fault)&1))


def effects(coefficients,mask):
    dim=len(coefficients[0]); rows=mask_rows(mask,dim)
    return [sum((rows[r][m]*coefficients[m][r] for m in range(3)),F(0)) for r in range(dim)]


def pure(value,label,eps,required):
    axis=label//2; direction=1 if label%2 else -1
    return direction*value[axis]>=required and all(abs(v)<=eps for i,v in enumerate(value) if i!=axis)


def verify_witness(witness,config,layer):
    """Independent exact check, including ALL 13 fault conditions (not only 3)."""
    try:
        dim=dimension(layer); s=scope(config); bounds=coefficient_bounds(config,layer)
        if witness['scope_signature']!=signature(config) or witness['layer']!=layer:
            return dict(valid=False,reason='scope_mismatch')
        c=[list(map(F,row)) for row in witness['coefficients']]
        if len(c)!=3 or any(len(row)!=dim for row in c):
            return dict(valid=False,reason='coefficient_shape')
        if any(not bounds[m][r][0]<=c[m][r]<=bounds[m][r][1] for m in range(3) for r in range(dim)):
            return dict(valid=False,reason='coefficient_bounds')
        masks=witness['masks']; expected=cases(layer)
        if len(masks)!=len(expected): return dict(valid=False,reason='group_count')
        eps=F(s['pure_tolerance']); required=F(s['min_axis_capacity'])-eps
        for mask,(fault,label) in zip(masks,expected):
            if not healthy(mask,fault,s['max_group_size']) or not pure(effects(c,mask),label,eps,required):
                return dict(valid=False,reason='representative_group',fault=fault+1,label=label+1)
        # Do not rely on a symmetry argument in the final certificate checker.
        qualified=[[] for _ in range(2*dim)]
        for count in range(2,s['max_group_size']+1,2):
            for members in itertools.combinations(range(12),count):
                mask=sum(1<<i for i in members); value=effects(c,mask)
                for label in range(2*dim):
                    if pure(value,label,eps,required): qualified[label].append(mask)
        full=[]
        for fault in range(13):
            for label in range(2*dim):
                mask=next((mask for mask in qualified[label] if fault==0 or not (mask>>(fault-1))&1),None)
                if mask is None: return dict(valid=False,reason='full_fault_check',fault=fault,label=label+1)
                value=effects(c,mask)
                full.append(dict(fault=fault,label=label+1,thrusters=[i+1 for i in range(12) if mask>>i&1],
                                 target=str((1 if label%2 else -1)*value[label//2]),
                                 max_off_axis=str(max(abs(v) for i,v in enumerate(value) if i!=label//2))))
        return dict(valid=True,arithmetic='fractions',checked_requirements=len(full),full_fault_groups=full,
                    physical_layout_verified=False)
    except (KeyError,TypeError,ValueError,ZeroDivisionError,IndexError):
        return dict(valid=False,reason='malformed_witness')


def zero_axis_certificate(config,layer):
    bounds=coefficient_bounds(config,layer)
    for row in range(dimension(layer)):
        if all(bounds[m][row]==(0,0) for m in range(3)):
            return dict(kind='identically_zero_coefficient_row',scope_signature=signature(config),
                        layer=layer,row=row,required=str(rational(config['min_axis_capacity'])-rational(config.get('pure_tolerance',1e-9))))
    return None


def verify_zero_certificate(proof,config,layer):
    # Recompute from the original scope, not from bounds asserted by the proof.
    return proof is not None and proof==zero_axis_certificate(config,layer)


def build_z3(config,layer,fixed_masks=None):
    import z3
    dim=dimension(layer); bounds=coefficient_bounds(config,layer)
    required=rational(config['min_axis_capacity'])-rational(config.get('pure_tolerance',1e-9))
    eps=rational(config.get('pure_tolerance',1e-9))/required
    ctx=z3.Context(proof=True); solver=z3.SolverFor('QF_LRA',ctx=ctx)
    def Q(v): return z3.RealVal(str(v),ctx=ctx)
    coefficients=[[z3.Real(f'{NAMES[r]}_{m}',ctx=ctx) for r in range(dim)] for m in range(3)]
    for m in range(3):
        for r in range(dim):
            lo,hi=bounds[m][r]; solver.add(coefficients[m][r]>=Q(lo/required),coefficients[m][r]<=Q(hi/required))
    groups=[]
    if fixed_masks is not None and (len(fixed_masks)!=len(cases(layer)) or
            any(not healthy(mask,f,config.get('max_group_size',6)) for mask,(f,_) in zip(fixed_masks,cases(layer)))):
        raise ValueError('Invalid fixed groups.')
    for case,(fault,label) in enumerate(cases(layer)):
        if fixed_masks is None:
            group=[z3.Bool(f'b_{case}_{i}',ctx=ctx) if i!=fault else z3.BoolVal(False,ctx=ctx) for i in range(12)]
            count=z3.Sum([z3.If(b,Q(1),Q(0)) for b in group])
            solver.add(z3.Or([count==n for n in range(2,config.get('max_group_size',6)+1,2)]))
        else:
            group=[z3.BoolVal(bool(fixed_masks[case]>>i&1),ctx=ctx) for i in range(12)]
        groups.append(group)
        for row in range(dim):
            value=z3.Sum([z3.If(group[i],SIGNS[row][i%4]*coefficients[i//4][row],Q(0)) for i in range(12) if i!=fault])
            if row==label//2: solver.add((1 if label%2 else -1)*value>=1)
            else: solver.add(value>=Q(-eps),value<=Q(eps))
    return solver,coefficients,groups,required


def witness_from_z3(model,coefficients,groups,scale,config,layer):
    import z3
    values=[]
    for row in coefficients:
        result=[]
        for v in row:
            x=model.eval(v,model_completion=True)
            result.append(str(F(x.numerator_as_long(),x.denominator_as_long())*scale))
        values.append(result)
    return dict(layer=layer,scope_signature=signature(config),coefficients=values,
                masks=[sum(1<<i for i,b in enumerate(group) if z3.is_true(model.eval(b,model_completion=True))) for group in groups])


def solve_z3(config,layer,seconds,artifact_dir=None,fixed_masks=None):
    import z3
    started=time.monotonic(); solver,c,groups,scale=build_z3(config,layer,fixed_masks)
    result=dict(engine='z3',version=z3.get_version_string(),status='unknown',budget_seconds=seconds,
                exact_arithmetic=True,independent_unsat_proof_verified=False)
    formula='(set-logic QF_LRA)\n'+solver.to_smt2()
    result['formula_sha256']=hashlib.sha256(formula.encode()).hexdigest()
    if artifact_dir:
        file=Path(artifact_dir)/(layer+('_fixed' if fixed_masks is not None else '')+'.smt2')
        file.write_text(formula); result['formula_file']=str(file.resolve())
    remaining=seconds-(time.monotonic()-started)
    if remaining<=0:
        result.update(reason='budget_exhausted_during_model_build',elapsed_seconds=time.monotonic()-started)
        return result
    solver.set(timeout=max(1,int(remaining*1000)),random_seed=int(config.get('seed',18))%2147483647)
    if config.get('diagnostic_rlimit') is not None: solver.set(rlimit=int(config['diagnostic_rlimit']))
    status=solver.check(); result['solver_status']=str(status)
    if status==z3.sat:
        witness=witness_from_z3(solver.model(),c,groups,scale,config,layer)
        verification=verify_witness(witness,config,layer)
        result.update(status='sat_verified' if verification['valid'] else 'witness_rejected',witness=witness,verification=verification)
    elif status==z3.unsat:
        # Z3's proof is retained for scrutiny. Merely printing/replaying it is
        # not an independent proof checker, so this status cannot certify UNSAT.
        result['status']='unsat_reported'
        proof=solver.proof().sexpr(); result['proof_sha256']=hashlib.sha256(proof.encode()).hexdigest()
        if artifact_dir:
            path=Path(artifact_dir)/(layer+('_fixed' if fixed_masks is not None else '')+'.z3proof')
            path.write_text(proof); result['proof_file']=str(path.resolve())
    else: result['reason']=solver.reason_unknown()
    result['elapsed_seconds']=time.monotonic()-started
    return result


def solve_scip(config,layer,seconds,artifact_dir=None):
    """Numerical comparator of exactly the same bounded necessary conditions."""
    from pyscipopt import Model,quicksum,SCIP_PARAMEMPHASIS
    started=time.monotonic(); dim=dimension(layer); bounds=coefficient_bounds(config,layer)
    scale=rational(config['min_axis_capacity'])-rational(config.get('pure_tolerance',1e-9))
    eps=float(rational(config.get('pure_tolerance',1e-9))/scale)
    model=Model('necessary_'+layer); model.hideOutput()
    model.setEmphasis(SCIP_PARAMEMPHASIS.FEASIBILITY,quiet=True)
    model.setRealParam('numerics/feastol',1e-9); model.setIntParam('limits/solutions',1)
    model.setIntParam('randomization/randomseedshift',int(config.get('seed',18))%2147483647)
    c=[[model.addVar(f'{NAMES[r]}_{m}',lb=float(bounds[m][r][0]/scale),ub=float(bounds[m][r][1]/scale)) for r in range(dim)] for m in range(3)]
    groups=[]
    for case,(fault,label) in enumerate(cases(layer)):
        group=[model.addVar(f'b_{case}_{i}',vtype='B',ub=0 if i==fault else 1) for i in range(12)];groups.append(group)
        n=model.addVar(f'pairs_{case}',vtype='I',lb=1,ub=config.get('max_group_size',6)//2)
        model.addCons(quicksum(group)==2*n)
        for row in range(dim):
            terms=[]
            for i,b in enumerate(group):
                if i==fault: continue
                m,j=divmod(i,4); lo,hi=[float(v/scale) for v in bounds[m][row]]; x=c[m][row]
                if lo==hi: product=lo*b
                else:
                    product=model.addVar(lb=min(0,lo),ub=max(0,hi))
                    model.addCons(product>=lo*b);model.addCons(product<=hi*b)
                    model.addCons(product>=x-hi*(1-b));model.addCons(product<=x-lo*(1-b))
                terms.append(SIGNS[row][j]*product)
            value=quicksum(terms)
            if row==label//2: model.addCons((1 if label%2 else -1)*value>=1)
            else: model.addCons(value>=-eps);model.addCons(value<=eps)
    model.setObjective(0)
    remaining=seconds-(time.monotonic()-started)
    result=dict(engine='scip',status='unknown',budget_seconds=seconds,exact_arithmetic=False,
                exact_mode_available=any(k.startswith('exact/') for k in model.getParams()))
    if remaining>0:
        model.setRealParam('limits/time',remaining);model.optimize()
        result.update(solver_status=str(model.getStatus()),nodes=model.getNNodes())
        if model.getNSols():
            sol=model.getBestSol();masks=[sum(1<<i for i,b in enumerate(g) if model.getSolVal(sol,b)>.5) for g in groups]
            result.update(status='numerical_candidate',masks=masks)
            # Re-solve only its fixed choices in rational linear arithmetic.
            repair_budget=min(5,max(0,seconds-(time.monotonic()-started)))
            if repair_budget>.02:
                repair=solve_z3(config,layer,repair_budget,artifact_dir,masks)
                result['rational_repair']=repair
                if repair['status']=='sat_verified':
                    result.update(status='sat_verified',witness=repair['witness'],verification=repair['verification'])
        elif str(model.getStatus())=='infeasible': result['status']='numerically_infeasible'
    else: result['reason']='budget_exhausted_during_model_build'
    result['elapsed_seconds']=time.monotonic()-started
    return result


def atomic_json(path,value):
    path=Path(path);temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False));temp.replace(path)


def geometry_checkpoint(config,witness,path):
    """Only a verified FULL wrench witness may be dispatched for geometry."""
    if witness.get('layer')!='wrench' or not verify_witness(witness,config,'wrench')['valid']:
        raise ValueError('Only a fully checked wrench witness can seed geometry.')
    from strict_layout_topology import prepare_state
    geometry_config=dict(config,search_state={})
    geometry_config.setdefault('min_install_distance',0)
    geometry_config.setdefault('min_install_angle_deg',0)
    state,_=prepare_state(geometry_config)
    state.update(seeded=True,next_stage='geometry',pending=[dict(id=0,masks=witness['masks'],trials=0,last_status='unresolved',
        regions=[dict(lower=config['lower'],upper=config['upper'],depth=0)])])
    atomic_json(path,dict(search_state=state,search_seeds=[],necessary_witness=witness))
    return str(Path(path).resolve())


def diagnose(config):
    started=time.monotonic(); scope(config)
    maximum=float(config.get('max_time',240)); per_engine=float(config.get('diagnostic_stage_time',60))
    if not math.isfinite(maximum) or maximum<=0 or not math.isfinite(per_engine) or per_engine<=0:
        raise ValueError('Diagnostic time budgets must be positive finite numbers.')
    output=dict(method=VERSION,status='diagnostic_unknown',has_solution=False,x=[],groups=[],candidates=[],
                search_goal='necessary_conditions_only',nodes=0,solution_count=0,layers=[],
                scope=scope(config),scope_signature=signature(config),physical_layout_verified=False,
                original_problem_infeasibility_verified=False,resume_note='new_diagnosis')
    reused={};previous={}
    if config.get('resume_file'):
        path=Path(config['resume_file'])
        if path.suffix.lower()!='.json':
            raise ValueError('Resume diagnostics with diagnosis.json; MAT records are not SMT checkpoints.')
        previous=json.loads(path.read_text())
        if previous.get('method')!=VERSION or previous.get('scope_signature')!=signature(config):
            output['resume_note']='method_or_constraints_changed_diagnosis_reset'
        else:
            for key in ('branch_queue_file','branch_progress','queue_state'):
                if key in previous: output[key]=previous[key]
            for old in previous.get('layers',[]):
                if old.get('status')=='sat_verified':
                    check=verify_witness(old['witness'],config,old['layer'])
                    if not check['valid']: raise ValueError('Saved necessary-condition witness failed independent verification.')
                    reused[old['layer']]=(old['witness'],check)
            output['resume_note']='resumed_verified_layers' if reused else 'no_verified_layer_to_reuse'
    folder=Path(config['diagnostic_artifact_dir']).resolve() if config.get('diagnostic_artifact_dir') else None
    if folder: folder.mkdir(parents=True,exist_ok=True)
    def persist(stage):
        output['solve_time']=time.monotonic()-started
        if folder:
            output['diagnostic_report_file']=str(folder/'diagnosis.json');atomic_json(folder/'diagnosis.json',output)
            if config.get('diagnostic_latest_path'):
                atomic_json(config['diagnostic_latest_path'],dict(diagnostic_report_file=output['diagnostic_report_file']))
        if config.get('progress_path'):
            progress=dict(stage='diagnostic_'+stage,coverage=0,attempt=len(output['layers']))
            if 'branch_progress' in output: progress['branch_progress']=output['branch_progress']
            atomic_json(config['progress_path'],progress)
    def receive_queue(run):
        for key in ('branch_queue_file','branch_progress','queue_state'):
            if key in run: output[key]=run[key]
        persist('branches')
    for layer in LAYERS:
        item=dict(layer=layer,status='unknown',real_variables=3*dimension(layer),representative_requirements=len(cases(layer)),
                  bounds=[[[str(a),str(b)] for a,b in row] for row in coefficient_bounds(config,layer)],runs=[])
        output['layers'].append(item); persist(layer)
        certificate=zero_axis_certificate(config,layer)
        if certificate and verify_zero_certificate(certificate,config,layer):
            item.update(status='unsat_verified',certificate=certificate,independent_certificate_verified=True)
            output.update(status='diagnostic_unsat_verified',original_problem_infeasibility_verified=True)
            break
        if layer in reused:
            witness,check=reused[layer]
            item.update(status='sat_verified',witness=witness,verification=check,resumed_from=str(Path(config['resume_file']).resolve()))
            engines=[]
        else:
            engines=[solve_z3]+([solve_scip] if config.get('diagnostic_compare',True) else [])
        if layer=='wrench' and layer not in reused and config.get('diagnostic_persistent',True):
            from strict_layout_branch_queue import solve_persistent
            left=maximum-(time.monotonic()-started)
            if left>0:
                branch_config=dict(config,_diagnostic_queue_callback=receive_queue,_diagnostic_previous_report=previous)
                if output['resume_note']=='method_or_constraints_changed_diagnosis_reset':
                    branch_config.pop('resume_file',None)
                run=solve_persistent(branch_config,left,folder)
                item['runs'].append(run)
                item['status']=run['status']
                if run['status']=='sat_verified':
                    item.update(witness=run['witness'],verification=run['verification'])
                elif run['status']=='unsat_verified':
                    item.update(certificate=dict(kind='finite_branch_partition',queue_file=run['branch_queue_file'])
                                if 'branch_queue_file' in run else dict(kind='finite_branch_partition',queue_state=run['queue_state']))
                    output.update(status='diagnostic_unsat_verified',original_problem_infeasibility_verified=True)
                elif run['status']=='review_required': output['status']='diagnostic_review_required'
                receive_queue(run)
            engines=[]
        for solve in engines:
            left=maximum-(time.monotonic()-started)
            if left<=0: break
            # Reserve a fair share for all unstarted engines/layers.
            engines_left=len(engines)-len(item['runs'])+(len(engines) if layer=='force' else 0)
            budget=min(per_engine,left/max(1,engines_left))
            try: run=solve(config,layer,budget,folder)
            except ImportError as error:
                run=dict(engine='z3' if solve==solve_z3 else 'scip',status='engine_unavailable',reason=str(error))
            item['runs'].append(run);output['nodes']+=run.get('nodes',0)
            if run['status']=='sat_verified':
                item.update(status='sat_verified',witness=run['witness'],verification=run['verification'])
            elif run['status']=='unsat_reported':
                item['status']='solver_disagreement' if item['status']=='sat_verified' else 'unsat_reported'
            if item['status']=='sat_verified' and any(r['status']=='unsat_reported' for r in item['runs']):
                item['status']='solver_disagreement'
            persist(layer)
        if item['status']=='solver_disagreement':
            output['status']='diagnostic_solver_disagreement';break
        if layer=='wrench' and item['status']=='sat_verified':
            if layer in reused and 'branch_queue_file' in output:
                from strict_layout_branch_queue import prepare_queue,branch_progress
                state=json.loads(Path(output['branch_queue_file']).read_text())
                checked,domain,_,note=prepare_queue(config,state)
                if note!='resumed_branch_queue': raise ValueError('Saved queue does not match the verified witness scope.')
                output['branch_progress']=branch_progress(checked,domain)
            output['status']='necessary_conditions_sat'
            supported=all(0<=config['lower'][4*m+3]<=config['upper'][4*m+3]<=math.pi/2 for m in range(3))
            output['geometry_handoff_supported']=supported
            if folder and supported:
                output['geometry_checkpoint_file']=geometry_checkpoint(config,item['witness'],folder/'geometry_queue.json')
            elif not supported:
                output['geometry_handoff_note']='Beta domain requires the original trigonometric joint solver; no decomposition checkpoint created.'
        elif item['status']=='unsat_reported':
            output['status']='diagnostic_unsat_reported'
            # Keep the scope distinction: exact solver output is retained, but
            # has not passed an independent general UNSAT proof checker.
            break
    persist('complete')
    return output


def verify_report(path):
    report=json.loads(Path(path).read_text());config=report['scope']
    # Restore numeric geometry bounds for the canonical signature serializer.
    # scope() accepts Fraction strings, so no binary float conversion is needed.
    if report['scope_signature']!=signature(config): return dict(valid=False,reason='scope_mismatch')
    results=[]
    for layer in report['layers']:
        if layer['status']=='sat_verified': result=verify_witness(layer['witness'],config,layer['layer'])
        elif layer['status']=='unsat_verified':
            cert=layer['certificate']
            if cert.get('kind')=='finite_branch_partition':
                from strict_layout_branch_queue import prepare_queue
                state=json.loads(Path(cert['queue_file']).read_text()) if 'queue_file' in cert else cert['queue_state']
                # A reset for a mismatched scope must never count as verification.
                checked,_,_,note=prepare_queue(config,state)
                result=dict(valid=note=='resumed_branch_queue' and all(n['status']=='closed' for n in checked['leaves']))
            else: result=dict(valid=verify_zero_certificate(cert,config,layer['layer']))
        else: result=dict(valid=None,reason='no_independently_checkable_conclusion')
        results.append(dict(layer=layer['layer'],**result))
    proven_unsat=any(item['status']=='unsat_verified' and check['valid'] is True
                     for item,check in zip(report['layers'],results))
    valid=bool(results) and all(r['valid'] is not False for r in results) and (
        proven_unsat or all(r['valid'] is True for r in results))
    return dict(valid=valid,layers=results,original_problem_infeasibility_verified=proven_unsat)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('input',type=Path,nargs='?');parser.add_argument('output',type=Path,nargs='?')
    parser.add_argument('--verify',type=Path);args=parser.parse_args()
    if args.verify:
        result=verify_report(args.verify);print(json.dumps(result,ensure_ascii=False));return
    if not args.input or not args.output: parser.error('input and output, or --verify, are required')
    config=json.loads(args.input.read_text());result=diagnose(config);atomic_json(args.output,result)
    print(json.dumps({k:result[k] for k in ('status','solve_time','original_problem_infeasibility_verified')},ensure_ascii=False))


if __name__=='__main__': main()
