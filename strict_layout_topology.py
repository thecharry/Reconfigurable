"""Topology/geometry decomposition with independently checkable linear cuts.

Only rational linear-dependence certificates permanently exclude topologies.
Dispatch exclusions merely move a topology to the persisted geometry queue;
timeouts AND floating-point infeasibility leave that queue intact.
"""
import copy
import hashlib
import heapq
import itertools
import json
import math
import time
from fractions import Fraction
from pathlib import Path

import numpy as np
from pyscipopt import Model, quicksum, SCIP_PARAMEMPHASIS

SIGNS = np.array([[-1,-1,1,1], [1,-1,1,-1], [1,-1,1,-1],
                  [1,1,1,1], [-1,1,1,-1], [1,-1,-1,1]], dtype=int)
CASES = tuple((f,label) for f in (0,4,8) for label in range(12))
VERSION = 'topology_linear_certificates_v1'


def mask_rows(mask):
    bits = np.array([(int(mask)>>i)&1 for i in range(12)])
    return np.array([SIGNS @ bits[4*m:4*m+4] for m in range(3)]).T


def valid_masks(masks, maximum):
    return len(masks)==36 and all(isinstance(v,(int,np.integer)) and 0<v<4096 and
        bin(int(v)).count('1') in range(2,maximum+1,2) and not (int(v)>>CASES[j][0])&1
        for j,v in enumerate(masks))


def span_weights(basis, target):
    """Solve basis.T*lambda=target exactly, including a rank-zero basis."""
    n=len(basis)
    matrix=[[Fraction(basis[j][i]) for j in range(n)]+[Fraction(target[i])] for i in range(3)]
    pivots=[]; r=0
    for c in range(n):
        pivot=next((i for i in range(r,3) if matrix[i][c]),None)
        if pivot is None:
            continue
        matrix[r],matrix[pivot]=matrix[pivot],matrix[r]
        d=matrix[r][c]; matrix[r]=[v/d for v in matrix[r]]
        for i in range(3):
            if i!=r:
                d=matrix[i][c]; matrix[i]=[v-d*w for v,w in zip(matrix[i],matrix[r])]
        pivots.append(c); r+=1
    if any(not any(line[:n]) and line[n] for line in matrix):
        return None
    weights=[Fraction(0)]*n
    for i,c in enumerate(pivots):
        weights[c]=matrix[i][n]
    return weights


def linear_conflicts(masks, config):
    """Each returned cut remains valid for the actual epsilon acceptance model."""
    rows=[mask_rows(m) for m in masks]; result=[]
    eps=Fraction(str(config.get('pure_tolerance',1e-9)))
    required=Fraction(str(config['min_axis_capacity']))-eps
    for axis in range(6):
        basis=[]; cases=[]
        for j,(_,label) in enumerate(CASES):
            if label//2!=axis and span_weights(basis,rows[j][axis]) is None:
                basis.append(rows[j][axis].tolist()); cases.append(j)
        for j,(_,label) in enumerate(CASES):
            if label//2!=axis:
                continue
            weights=span_weights(basis,rows[j][axis])
            if weights is None or eps*sum(map(abs,weights))>=required:
                continue
            used=[(i,w) for i,w in enumerate(weights) if w]
            ids=[cases[i] for i,w in used]+[j]
            result.append(dict(kind='rational_span_bound',axis=axis,target_case=j,
                basis_cases=[cases[i] for i,w in used],weights=[str(w) for i,w in used],
                assignments=[[i,int(masks[i])] for i in ids],
                capacity_upper=str(eps*sum(map(abs,weights)))))
    return result


def verify_conflict(proof,config):
    """Recheck saved evidence before it is allowed to constrain a new master."""
    try:
        if proof['kind']!='rational_span_bound': return False
        entries=proof['assignments']
        if entries and not isinstance(entries[0],(list,tuple)): entries=[entries]
        assignments={int(i):int(m) for i,m in entries}
        axis=int(proof['axis']); target=int(proof['target_case'])
        raw=proof['basis_cases']; basis=[int(raw)] if isinstance(raw,(int,float)) else list(raw)
        raw=proof['weights']; weights=[Fraction(w) for w in ([raw] if isinstance(raw,str) else raw)]
        if axis not in range(6) or target not in range(36) or CASES[target][1]//2!=axis:
            return False
        if len(basis)!=len(weights) or set(assignments)!=set(basis+[target]): return False
        for j,m in assignments.items():
            if j not in range(36) or not 0<m<4096 or bin(m).count('1') not in range(2,int(config.get('max_group_size',6))+1,2) or (m>>CASES[j][0])&1:
                return False
        if any(CASES[j][1]//2==axis for j in basis): return False
        actual=[sum(w*int(mask_rows(assignments[j])[axis,k]) for j,w in zip(basis,weights)) for k in range(3)]
        if actual!=list(mask_rows(assignments[target])[axis]): return False
        eps=Fraction(str(config.get('pure_tolerance',1e-9)))
        return eps*sum(map(abs,weights)) < Fraction(str(config['min_axis_capacity']))-eps
    except (KeyError,TypeError,ValueError,IndexError,ZeroDivisionError):
        return False


def add_exclusion(model,groups,assignments):
    # This clause excludes exactly the stated joint group assignments.
    terms=[]
    for case,mask in assignments:
        fault,_=CASES[case]
        terms.extend(1-v if (mask>>i)&1 else v for i,v in enumerate(groups[case]) if i!=fault)
    model.addCons(quicksum(terms)>=1)


def add_sector(model,p,q,low,high,lateral,name):
    """Closed direction arc; its zero vector is allowed when beta=0."""
    if high-low>=2*math.pi: return
    edges=[(low,high)] if high-low<=math.pi else [(low,(low+high)/2),((low+high)/2,high)]
    active=[model.addVar(f'{name}_{i}',vtype='B') for i in range(2)] if len(edges)==2 else [None]
    if len(edges)==2: model.addCons(quicksum(active)==1)
    for (left,right),chosen in zip(edges,active):
        slack=0 if chosen is None else 4*lateral*(1-chosen)
        model.addCons(math.cos(left)*q-math.sin(left)*p>=-slack)
        model.addCons(math.sin(right)*p-math.cos(right)*q>=-slack)
        middle=(left+right)/2
        model.addCons(math.cos(middle)*p+math.sin(middle)*q>=-slack)


def product_envelope(model,x,y,xb,yb,name):
    lo,hi=xb; left,right=yb
    corners=[lo*left,lo*right,hi*left,hi*right]
    v=model.addVar(name,lb=min(corners),ub=max(corners))
    model.addCons(v>=lo*y+left*x-lo*left)
    model.addCons(v>=hi*y+right*x-hi*right)
    model.addCons(v<=hi*y+left*x-hi*left)
    model.addCons(v<=lo*y+right*x-lo*right)
    return v


def build_coefficients(config,relaxed=False,fixed_masks=None):
    model=Model('topology_linear_master' if relaxed else 'physical_geometry')
    if not config.get('verbose',False): model.hideOutput()
    model.setRealParam('numerics/feastol',min(1e-9,max(1e-10,float(config.get('pure_tolerance',1e-9))/10)))
    model.setRealParam('limits/time',max(.001,float(config.get('max_time',60))))
    model.setIntParam('limits/solutions',1)
    model.setEmphasis(SCIP_PARAMEMPHASIS.FEASIBILITY,quiet=True)
    model.setIntParam('randomization/randomseedshift',int(config.get('seed',18))%2147483647)
    lower,upper,a=config['lower'],config['upper'],float(config['x_face'])
    positions=[]; coefficients=[]; bounds=[]
    for m in range(3):
        lo,hi=lower[4*m:4*m+4],upper[4*m:4*m+4]
        if not 0<=lo[3]<=hi[3]<=math.pi/2: raise ValueError('Algebraic model requires beta in [0,pi/2].')
        y=model.addVar(f'y_{m}',lb=lo[0],ub=hi[0]); z=model.addVar(f'z_{m}',lb=lo[1],ub=hi[1])
        ey,ez,lateral=max(abs(lo[0]),abs(hi[0])),max(abs(lo[1]),abs(hi[1])),math.sin(hi[3])
        limits=[(max(0,math.cos(hi[3])),math.cos(lo[3])),(-lateral,lateral),(-lateral,lateral),
                (-(ey+ez)*lateral/a,(ey+ez)*lateral/a),(-lateral-ez/a,lateral+ez/a),(-lateral-ey/a,lateral+ey/a)]
        row=[model.addVar(f'{name}_{m}',lb=l,ub=u) for name,(l,u) in zip(('c','p','q','h','g','k'),limits)]
        c,p,q,h,g,k=row
        add_sector(model,p,q,lo[2],hi[2],lateral,f'sector_{m}')
        if relaxed and config.get('strong_master_relaxation',False):
            # A unit direction has at least one component of magnitude >=1/sqrt(3).
            # The disjunction avoids the all-near-zero solution of a mere box relaxation.
            choices=[]; floor=1/math.sqrt(3)-1e-12
            for j,sign in ((0,1),(1,-1),(1,1),(2,-1),(2,1)):
                chosen=model.addVar(f'norm_branch_{m}_{j}_{sign}',vtype='B');choices.append(chosen)
                minimum=limits[j][0] if sign>0 else -limits[j][1]
                model.addCons(sign*row[j]>=floor-(floor-minimum)*(1-chosen))
            model.addCons(quicksum(choices)>=1)
            squares=[]
            for j,v in enumerate(row[:3]):
                l,u=limits[j]
                square=model.addVar(f'square_{m}_{j}',lb=0,ub=max(l*l,u*u))
                model.addCons(square<=(l+u)*v-l*u)
                for t in np.linspace(l,u,5): model.addCons(square>=2*float(t)*v-float(t*t))
                squares.append(square)
            model.addCons(quicksum(squares)==1)
            yp=product_envelope(model,y,q,(lo[0],hi[0]),limits[2],f'yq_{m}')
            zp=product_envelope(model,z,p,(lo[1],hi[1]),limits[1],f'zp_{m}')
            zc=product_envelope(model,z,c,(lo[1],hi[1]),limits[0],f'zc_{m}')
            yc=product_envelope(model,y,c,(lo[0],hi[0]),limits[0],f'yc_{m}')
            model.addCons(a*h==yp-zp); model.addCons(a*(g-q)==zc); model.addCons(a*(k-p)==yc)
        elif not relaxed:
            model.addCons(c*c+p*p+q*q==1)
            model.addCons(a*h==y*q-z*p); model.addCons(a*(g-q)==z*c); model.addCons(a*(k-p)==y*c)
        positions.append((y,z)); coefficients.append(row); bounds.append(limits)
    # Ordering is invalid for fixed labelled topologies or unequal module boxes.
    same_bounds=all(lower[4*m:4*m+4]==lower[:4] and upper[4*m:4*m+4]==upper[:4] for m in (1,2))
    if fixed_masks is None and config.get('order_modules',True) and same_bounds:
        model.addCons(positions[0][0]<=positions[1][0]); model.addCons(positions[1][0]<=positions[2][0])
    if not relaxed:
        r=[];d=[]
        for m in range(3):
            y,z=positions[m];c,p,q=coefficients[m][:3]
            for sx,st in ((1,1),(1,-1),(-1,1),(-1,-1)):
                r.append([sx*a,st*y,st*z]);d.append([-sx*c,st*p,st*q])
        for i,j in itertools.combinations(range(12),2):
            separation=max(config.get('min_install_distance',0),config.get('strict_collision_distance',0))
            if separation>0:
                model.addCons(quicksum((r[i][k]-r[j][k])**2 for k in range(3))>=separation**2)
            if config.get('min_install_angle_deg',0)>0:
                model.addCons(quicksum(d[i][k]*d[j][k] for k in range(3))<=math.cos(math.radians(config['min_install_angle_deg'])))
    groups=[]; pairs={}; eps=float(config.get('pure_tolerance',1e-9)); rho=float(config['min_axis_capacity'])
    for case,(fault,label) in enumerate(CASES):
        if fixed_masks is None:
            group=[model.addVar(f'b_{case}_{i}',vtype='B',ub=0 if i==fault else 1) for i in range(12)]
            count=model.addVar(f'pairs_{case}',vtype='I',lb=1,ub=int(config.get('max_group_size',6))//2)
            model.addCons(quicksum(group)==2*count); pairs[fault,label]=count
        else:
            group=[(fixed_masks[case]>>i)&1 for i in range(12)]
        groups.append(group)
        for axis in range(6):
            terms=[]
            for i,chosen in enumerate(group):
                if i==fault: continue
                m,j=divmod(i,4); value=coefficients[m][axis];l,u=bounds[m][axis]
                if fixed_masks is not None: v=chosen*value
                elif l==u: v=l*chosen
                else: v=product_envelope(model,value,chosen,(l,u),(0,1),f'prod_{case}_{axis}_{i}')
                terms.append(int(SIGNS[axis,j])*v)
            effect=quicksum(terms)
            if axis==label//2: model.addCons((-1 if label%2==0 else 1)*effect>=max(0,rho-eps))
            else:
                model.addCons(effect<=eps); model.addCons(effect>=-eps)
    seeds=np.array(config.get('search_seeds',[]))
    if seeds.size:
        for seed in np.atleast_2d(seeds)[:4]:
            modules=seed.reshape(3,4).tolist()
            if fixed_masks is None and config.get('order_modules',True) and same_bounds: modules.sort()
            flat=np.array(modules).reshape(-1)
            if np.any(flat<lower) or np.any(flat>upper): continue
            sol=model.createPartialSol()
            for m,(y,z,alpha,beta) in enumerate(modules):
                c,p,q=math.cos(beta),math.sin(beta)*math.cos(alpha),math.sin(beta)*math.sin(alpha)
                values=(y,z,c,p,q,(y*q-z*p)/a,q+z*c/a,p+y*c/a)
                for variable,value in zip((*positions[m],*coefficients[m]),values): model.setSolVal(sol,variable,value)
            model.addSol(sol)
    model.setObjective(0)
    return model,dict(positions=positions,coefficients=coefficients,bounds=bounds),groups,pairs


def solution_x(model,data,config):
    sol=model.getBestSol(); result=[]
    for m in range(3):
        y,z=[model.getSolVal(sol,v) for v in data['positions'][m]]
        c,p,q=[model.getSolVal(sol,v) for v in data['coefficients'][m][:3]]
        lo,hi=config['lower'][4*m+2],config['upper'][4*m+2]
        alpha=lo if math.hypot(p,q)<1e-14 else math.atan2(q,p)
        alpha+=2*math.pi*math.ceil((lo-alpha-1e-12)/(2*math.pi))
        if alpha>hi and alpha-hi<1e-9: alpha=hi
        if alpha<lo and lo-alpha<1e-9: alpha=lo
        result.extend([y,z,alpha,math.atan2(math.hypot(p,q),c)])
    return result


def physical_matrix(x,a):
    columns=[]; positions=[]
    for y,z,alpha,beta in np.array(x).reshape(3,4):
        c,p,q=math.cos(beta),math.sin(beta)*math.cos(alpha),math.sin(beta)*math.sin(alpha)
        for sx,st in ((1,1),(1,-1),(-1,1),(-1,-1)):
            r=np.array([sx*a,st*y,st*z]);d=np.array([-sx*c,st*p,st*q])
            positions.append(r);columns.append(np.r_[d,np.cross(r,d)/a])
    return np.array(columns).T,np.array(positions)


def group_library(maximum):
    masks=[m for m in range(1,4096) if bin(m).count('1') in range(2,maximum+1,2)]
    return masks,np.array([[(m>>i)&1 for i in range(12)] for m in masks]).T


def repair_seed_topologies(seed,config,library,deadline):
    """Conflict-directed warm start; the full master still covers every other group.

    Search alternatives for the conflicting cases instead of fixing the closest
    group forever. This bounded heuristic never supplies an infeasibility claim.
    """
    masks,bits=library; A,_=physical_matrix(seed,float(config['x_face']))
    W=np.einsum('ai,ig->ag',A,bits); rho=float(config['min_axis_capacity'])
    options=[]; costs=[]
    for fault,label in CASES:
        axis=label//2; sign=-1 if label%2==0 else 1
        cost=np.sum(np.delete(W,axis,axis=0)**2,axis=0)+np.maximum(0,rho-sign*W[axis])**2
        possible=np.any(np.array([SIGNS[axis]@bits[4*m:4*m+4] for m in range(3)])!=0,axis=0)
        cost[(bits[fault]!=0)|~possible]=np.inf
        order=np.argsort(cost,kind='stable')[:16]
        options.append([masks[i] for i in order if np.isfinite(cost[i])]);costs.append({masks[i]:float(cost[i]) for i in order if np.isfinite(cost[i])})
    initial=tuple(o[0] for o in options)
    heap=[(sum(c[m] for c,m in zip(costs,initial)),0,initial)];seen={initial};proofs=[];candidates=[];count=0
    while heap and time.monotonic()<deadline and len(candidates)<4:
        _,_,topology=heapq.heappop(heap);conflicts=linear_conflicts(topology,config);count+=1
        if not conflicts:
            candidates.append(list(topology));continue
        proofs.extend(conflicts)
        # A valid short conflict tells exactly which group choices need changing.
        proof=min(conflicts,key=lambda p:len(p['assignments']))
        for case,_ in proof['assignments']:
            for mask in options[case]:
                child=list(topology);child[case]=mask;child=tuple(child)
                if child not in seen:
                    seen.add(child)
                    heapq.heappush(heap,(sum(c[m] for c,m in zip(costs,child)),len(seen),child))
    return candidates,proofs,count


def evaluate_candidate(x,config,library=None):
    masks,bits=library or group_library(int(config.get('max_group_size',6)))
    A,r=physical_matrix(x,float(config['x_face'])); W=np.einsum('ai,ig->ag',A,bits)
    eps=float(config.get('pure_tolerance',1e-9)); threshold=float(config['min_axis_capacity'])-eps
    coverage=0; selected=[]
    for fault in range(13):
        for label in range(12):
            axis=label//2; sign=-1 if label%2==0 else 1
            good=np.max(np.abs(np.delete(W,axis,axis=0)),axis=0)<=eps
            good&=sign*W[axis]>=threshold
            if fault: good&=bits[fault-1]==0
            coverage+=bool(np.any(good))
    for fault,label in CASES:
        axis=label//2;sign=-1 if label%2==0 else 1
        cost=np.sum(np.delete(W,axis,axis=0)**2,axis=0)+np.maximum(0,threshold-sign*W[axis])**2
        possible=np.any(np.array([SIGNS[axis]@bits[4*m:4*m+4] for m in range(3)])!=0,axis=0)
        cost[(bits[fault]!=0)|~possible]=np.inf
        selected.append(masks[int(np.argmin(cost))])
    installed=np.all(np.isfinite(x)) and np.all(np.array(x)>=np.array(config['lower'])-eps) and np.all(np.array(x)<=np.array(config['upper'])+eps)
    from strict_layout_structure import strict_installation_domain
    installed &= strict_installation_domain(x, config, config.get('strict_support'))
    for i,j in itertools.combinations(range(12),2):
        installed &= np.linalg.norm(r[i]-r[j])>=config.get('strict_collision_distance',0)
        installed &= np.linalg.norm(r[i]-r[j])+eps*float(config['x_face'])>=config.get('min_install_distance',0)
        if config.get('min_install_angle_deg',0)>0:
            installed &= np.dot(A[:3,i],A[:3,j])<=math.cos(math.radians(config['min_install_angle_deg']))+eps
    return dict(coverage=int(coverage),has_solution=bool(coverage==156 and installed),masks=selected)


def solve_geometry(config,work):
    node=work['regions'][0]
    child=dict(config,lower=node['lower'],upper=node['upper'],verbose=False,order_modules=False)
    started=time.monotonic();model,data,_,_=build_coefficients(child,fixed_masks=work['masks'])
    model.setRealParam('limits/time',max(.001,config['max_time']-(time.monotonic()-started)))
    model.optimize()
    result=dict(status=str(model.getStatus()),nodes=model.getNNodes(),has_solution=False)
    if model.getNSols():
        x=solution_x(model,data,child); check=evaluate_candidate(x,config)
        result.update(x=x,coverage=check['coverage'],has_solution=check['has_solution'])
    return result


def split_unresolved(work,config):
    node=work['regions'].pop(0)
    scale=np.maximum(np.array(config['upper'])-config['lower'],1e-15)
    axis=int(np.argmax((np.array(node['upper'])-node['lower'])/scale))
    middle=(node['lower'][axis]+node['upper'][axis])/2
    if node['lower'][axis]<middle<node['upper'][axis]:
        for side in (0,1):
            child=copy.deepcopy(node);child['depth']+=1
            child['upper' if side==0 else 'lower'][axis]=middle
            work['regions'].append(child)
    else:
        node['depth']+=1;work['regions'].append(node)


def _as_list(value):
    return [value] if isinstance(value,dict) else list(value or [])


def prepare_state(config):
    keys=('lower','upper','x_face','min_install_distance','min_install_angle_deg','min_axis_capacity')
    domain={k:config[k] for k in keys};domain.update(max_group_size=int(config.get('max_group_size',6)),pure_tolerance=float(config.get('pure_tolerance',1e-9)))
    if 'strict_support' in config:
        domain.update(strict_support=config['strict_support'],strict_collision_distance=config.get('strict_collision_distance',0))
    signature=hashlib.sha256(json.dumps(domain,sort_keys=True).encode()).hexdigest()
    state=copy.deepcopy(config.get('search_state') or {})
    note='resumed_topologies'
    if state.get('method')!=VERSION or state.get('signature')!=signature:
        note='new_topologies' if not state else 'method_or_constraints_changed_queue_reset'
        state=dict(version=3,method=VERSION,signature=signature,conflicts=[],pending=[],processed=0,
                   master_calls=0,master_numerically_exhausted=False,seeded=False,next_stage='master',
                   best_coverage=0,best_x=[],history=[])
    for key in ('conflicts','pending','history'): state[key]=_as_list(state.get(key))
    if any(not verify_conflict(p,config) for p in state['conflicts']):
        raise ValueError('Saved topology conflict failed independent rational verification.')
    for proof in state['conflicts']:
        if proof['assignments'] and not isinstance(proof['assignments'][0],(list,tuple)):
            proof['assignments']=[proof['assignments']]
        if isinstance(proof['basis_cases'],(int,float)): proof['basis_cases']=[int(proof['basis_cases'])]
        if isinstance(proof['weights'],str): proof['weights']=[proof['weights']]
    for work in state['pending']:
        work['regions']=_as_list(work['regions'])
        if not valid_masks(work['masks'],int(config.get('max_group_size',6))) or not work['regions']:
            raise ValueError('Invalid saved topology or empty unresolved region list.')
        for node in work['regions']:
            if len(node['lower'])!=12 or len(node['upper'])!=12 or not np.all(np.isfinite(node['lower']+node['upper'])) or np.any(np.array(node['lower'])<config['lower']) or np.any(np.array(node['upper'])>config['upper']) or np.any(np.array(node['lower'])>node['upper']):
                raise ValueError('Saved geometry region is outside the configured domain.')
    return state,note


def structural_obstruction(config):
    # Only exact zero identities are used here; narrow or nearly-zero angles
    # are left to the numerical model, not silently rounded into a proof.
    low,high=config['lower'],config['upper']
    no_z=all(high[4*m+3]==0 or low[4*m+2]==high[4*m+2]==0 for m in range(3))
    if no_z and config['min_axis_capacity']>config.get('pure_tolerance',1e-9):
        return dict(kind='identically_zero_force_axis',axis='Fz',reason='Every module has beta=0 or alpha=0; all q_m=0.')
    return None


def save_checkpoint(config,state,stage):
    if config.get('checkpoint_path'):
        path=Path(config['checkpoint_path']);tmp=path.with_suffix('.tmp')
        tmp.write_text(json.dumps(dict(search_state=state,search_seeds=config.get('search_seeds',[])),allow_nan=False))
        tmp.replace(path)
    if config.get('progress_path'):
        path=Path(config['progress_path']);tmp=path.with_suffix('.tmp')
        tmp.write_text(json.dumps(dict(stage=stage,coverage=state['best_coverage'],attempt=state['processed'],
            topologies=len(state['pending']),conflicts=len(state['conflicts']),master_calls=state['master_calls'])))
        tmp.replace(path)


def solve_decomposed(config):
    started=time.monotonic();state,note=prepare_state(config)
    maximum=int(config.get('max_group_size',6)); library=group_library(maximum)
    if maximum not in (2,4,6): raise ValueError('A single-axis group must contain at most six thrusters.')
    output=dict(status='timelimit',has_solution=False,x=[],groups=[],nodes=0,solution_count=0,
                method=VERSION,search_goal='first_feasible',resume_note=note,search_state=state,
                representative_faults=[1,5,9],candidates=[])
    obstruction=structural_obstruction(config)
    if obstruction:
        output.update(status='infeasible',infeasibility_proof=obstruction,solve_time=time.monotonic()-started,
                      conflict_count=len(state['conflicts']),topology_count=len(state['pending']),geometry_calls=state['processed'],master_calls=state['master_calls'])
        save_checkpoint(config,state,'topology_complete')
        return output
    seen={json.dumps(p['assignments'],sort_keys=True) for p in state['conflicts']}
    if not state['seeded']:
        seeds=np.array(config.get('search_seeds',[]))
        repair_deadline=started+min(float(config.get('topology_seed_time',3)),max(0,float(config['max_time'])/4))
        for seed in np.atleast_2d(seeds) if seeds.size else []:
            if time.monotonic()-started>=config['max_time']: break
            check=evaluate_candidate(seed,config,library)
            if check['coverage']>state['best_coverage']:
                state['best_coverage']=check['coverage'];state['best_x']=seed.tolist()
            candidates,proofs,count=repair_seed_topologies(seed,config,library,repair_deadline)
            state['seed_topologies_checked']=state.get('seed_topologies_checked',0)+count
            for masks in candidates:
                if not any(w['masks']==masks for w in state['pending']):
                    state['pending'].append(dict(id=len(state['pending']),masks=masks,trials=0,last_status='unresolved',
                        regions=[dict(lower=list(config['lower']),upper=list(config['upper']),depth=0)]))
            for proof in proofs+linear_conflicts(check['masks'],config):
                key=json.dumps(proof['assignments'],sort_keys=True)
                if key not in seen: state['conflicts'].append(proof);seen.add(key)
        state['seeded']=True
        if state['pending']: state['next_stage']='geometry'
    save_checkpoint(config,state,'topology_master')
    master=None; groups=None
    # A numerical exhausted flag is not a permanent certificate across runs.
    master_exhausted=False
    while time.monotonic()-started<float(config['max_time']):
        remaining=float(config['max_time'])-(time.monotonic()-started)
        geometry=bool(state['pending']) and (state['next_stage']=='geometry' or master_exhausted)
        if geometry:
            work=min(state['pending'],key=lambda w:(w['trials'],w['id']))
            budget=min(remaining,float(config.get('geometry_time',15))*2**min(work['regions'][0]['depth'],3))
            save_checkpoint(config,state,'topology_geometry')
            trial=solve_geometry(dict(config,max_time=max(.001,budget)),work)
            work['trials']+=1;work['last_status']=trial['status'];state['processed']+=1
            output['nodes']+=trial.get('nodes',0)
            state['history'].append(dict(topology_id=work['id'],status=trial['status'],budget=budget))
            state['history']=state['history'][-256:]
            # Even numerical 'infeasible' is retained. Only algebraic proof cuts prune.
            split_unresolved(work,config)
            if trial.get('coverage',-1)>state['best_coverage']:
                state['best_coverage']=trial['coverage'];state['best_x']=trial['x']
            if trial['has_solution']:
                output.update(status='candidate',has_solution=True,x=trial['x'],solution_count=1,
                    groups=[[i+1 for i in range(12) if (m>>i)&1] for m in work['masks']])
            state['next_stage']='master'
        else:
            if master is None:
                master,_,groups,_=build_coefficients(dict(config,verbose=False),relaxed=True)
                for proof in state['conflicts']: add_exclusion(master,groups,proof['assignments'])
                for work in state['pending']: add_exclusion(master,groups,list(enumerate(work['masks'])))
            remaining=float(config['max_time'])-(time.monotonic()-started)
            if remaining<=0: break
            # Continue the same transformed master on timeout, including its search tree.
            master.setRealParam('limits/time',master.getSolvingTime()+max(.001,min(remaining,float(config.get('master_time',30)))))
            before=master.getNNodes();master.optimize();state['master_calls']+=1
            output['nodes']+=max(0,master.getNNodes()-before)
            if master.getNSols():
                sol=master.getBestSol()
                masks=[sum(1<<i for i,v in enumerate(group) if master.getSolVal(sol,v)>.5) for group in groups]
                if not valid_masks(masks,maximum): raise RuntimeError('Master returned invalid healthy group membership.')
                proofs=linear_conflicts(masks,config)
                master.freeTransform()
                if proofs:
                    for proof in proofs:
                        key=json.dumps(proof['assignments'],sort_keys=True)
                        if key not in seen:
                            state['conflicts'].append(proof);seen.add(key);add_exclusion(master,groups,proof['assignments'])
                else:
                    work=dict(id=len(state['pending']),masks=masks,trials=0,last_status='unresolved',
                        regions=[dict(lower=list(config['lower']),upper=list(config['upper']),depth=0)])
                    state['pending'].append(work)
                    # Save unresolved work BEFORE excluding it from fresh dispatch.
                    save_checkpoint(config,state,'topology_master')
                    add_exclusion(master,groups,list(enumerate(masks)))
                state['next_stage']='geometry'
            elif str(master.getStatus())=='infeasible':
                master_exhausted=True;state['master_numerically_exhausted']=True
                if not state['pending']:
                    output['status']='numerically_infeasible';break
            else:
                state['next_stage']='geometry'
        save_checkpoint(config,state,'topology_geometry' if state['next_stage']=='geometry' else 'topology_master')
        if output['has_solution']: break
    if state['best_x']: output['candidates']=[state['best_x']]
    output.update(solve_time=time.monotonic()-started,conflict_count=len(state['conflicts']),
                  topology_count=len(state['pending']),geometry_calls=state['processed'],master_calls=state['master_calls'])
    if config.get('checkpoint_path'): output['checkpoint_file']=config['checkpoint_path']
    save_checkpoint(config,state,'topology_complete')
    return output
