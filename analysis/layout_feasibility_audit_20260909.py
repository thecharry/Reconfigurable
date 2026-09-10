"""Independent audit of the saved 152/156 layout; never changes the optimizer.

Run with the project's strict-solver-venv Python. Integer arithmetic checks
the exact layout A = S/(10*sqrt(2)); SCIP is used only for continuous LPs.
"""
import csv
import hashlib
import itertools
import json
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from pyscipopt import Model, quicksum

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from strict_layout_minlp import zero_free_patterns


def integer_matrix():
    signs = np.array([[-1,-1,1,1], [1,-1,1,-1], [1,-1,1,-1],
                      [1,1,1,1], [-1,1,1,-1], [1,-1,-1,1]])
    coefficients = ([10,10,0,0,0,11], [10,0,10,3,9,3],
                    [10,0,-10,-3,-9,3])
    return np.hstack([np.array(c)[:, None]*signs for c in coefficients])


def coverage(S, sizes):
    masks = np.array([[(bits >> i) & 1 for i in range(12)]
                      for bits in range(1,4096)
                      if bin(bits).count('1') in sizes], dtype=int).T
    W = S @ masks
    missing, certificates = [], []
    counts = []
    for fault in range(13):
        for axis in range(6):
            for sign in (-1,1):
                ok = np.all(np.delete(W,axis,axis=0)==0,axis=0) & (sign*W[axis]>0)
                if fault:
                    ok &= masks[fault-1]==0
                ids = np.flatnonzero(ok)
                counts.append(len(ids))
                entry = dict(fault=fault,axis=('+' if sign>0 else '-')+('Fx','Fy','Fz','Mx','My','Mz')[axis])
                if not len(ids):
                    missing.append(entry)
                else:
                    best = ids[np.argmax(sign*W[axis,ids])]
                    entry.update(group=(np.flatnonzero(masks[:,best])+1).tolist(),
                                 capacity=float(sign*W[axis,best]/(10*math.sqrt(2))))
                    certificates.append(entry)
    return dict(group_count=masks.shape[1],coverage=156-len(missing),missing=missing,
                smallest_positive_capacity=min(c['capacity'] for c in certificates),
                exact_integer_zero_test=True,certificates=certificates)


def parity_certificate():
    # Permit every cardinality and ignore all torques and installation positions.
    masks = np.array([[(bits >> i)&1 for i in range(12)]
                      for bits in range(4096) if not bits&1],dtype=int).T
    t = np.array([np.array([1,1,-1,-1])@masks[4*m:4*m+4] for m in range(3)])
    v = np.array([np.array([1,-1,1,-1])@masks[4*m:4*m+4] for m in range(3)])
    assert np.all((t-v)%2==0)
    positive = v[0]>0
    assert np.all(t[0,positive]==-1)
    pure_force = (t.sum(axis=0)==0)&(v[1]==v[2])
    assert not np.any(positive&pure_force)
    lattice_gap = np.maximum(np.abs(t.sum(axis=0)),np.abs(v[1]-v[2]))
    return dict(healthy_subsets_checked=masks.shape[1],positive_Fy_subsets=int(positive.sum()),
                pure_positive_Fy_subsets=int(np.sum(positive&pure_force)),
                minimum_integer_off_axis_force=int(lattice_gap[positive].min()),
                minimum_normalized_off_axis_force=float(lattice_gap[positive].min()/math.sqrt(2)),
                scope='Shared beta in (0,pi/2), alpha=(0,pi/2,3pi/2); all positions; all cardinalities',
                endpoint_obstruction='beta=0 has no Fy/Fz; beta=pi/2 has no Fx')


def continuous_relaxation(S):
    results=[]
    for fault in range(13):
        for axis in range(6):
            for sign in (-1,1):
                model=Model('continuous_allocation_relaxation'); model.hideOutput()
                model.setRealParam('numerics/feastol',1e-9)
                u=[model.addVar(lb=0,ub=0 if i==fault-1 else 1) for i in range(12)]
                capacity=model.addVar(lb=0,ub=120)
                for row in range(6):
                    model.addCons(quicksum(int(S[row,i])*u[i] for i in range(12)) ==
                                  (sign*capacity if row==axis else 0))
                # This LP still retains the relaxation of the six-thruster limit.
                model.addCons(quicksum(u)<=6)
                model.setObjective(capacity,'maximize'); model.optimize()
                assert str(model.getStatus())=='optimal'
                sol=model.getBestSol(); uv=np.array([model.getSolVal(sol,x) for x in u])
                target=np.zeros(6); target[axis]=sign*model.getSolVal(sol,capacity)
                results.append(dict(fault=fault,axis=axis,sign=sign,duties=uv.tolist(),
                                    capacity=float(abs(target[axis])/(10*math.sqrt(2))),
                                    residual=float(np.max(np.abs(S@uv-target))/(10*math.sqrt(2))),
                                    status=str(model.getStatus())))
    return dict(coverage=sum(r['capacity']>=1e-5 for r in results),
                minimum_capacity=min(r['capacity'] for r in results),
                maximum_residual=max(r['residual'] for r in results),
                definition='0<=u_i<=1, u_fault=0, sum(u)<=6; independent duties',results=results)


def pattern_audit():
    lower=[0,-.6,0,0]*3; upper=[.6,.6,2*math.pi,math.pi/2]*3
    patterns,roots=zero_free_patterns(lower,upper)
    zero_z=[]; algebraic=[]
    for p,r in zip(patterns,roots):
        mask=p['mask']
        if all(not(mask[4*m+2] and mask[4*m+3]) for m in range(3)):
            zero_z.append(p['id'])
        if all(r['lower'][4*m+2]<=0 and r['upper'][4*m+2]>=2*math.pi for m in range(3)):
            algebraic.append(p['id'])
    orbits={tuple(sorted(tuple(p['mask'][4*m:4*m+4]) for m in range(3))) for p in patterns}
    rows=list(csv.DictReader((ROOT/'recovery/zero_free_729_patterns.csv').open()))
    infeasible={int(r['PatternId']) for r in rows if r['Status']=='numerically_infeasible'}
    return dict(total=len(patterns),identical_module_permutation_classes=len(orbits),
                symbolically_zero_Fz_count=len(zero_z),symbolically_zero_Fz_ids=zero_z,
                recorded_infeasible_exactly_zero_Fz=(infeasible==set(zero_z)),
                recorded_status_counts=dict(Counter(r['Status'] for r in rows)),
                recorded_zero_Fz_seconds=sum(float(r['ElapsedSeconds']) for r in rows if int(r['PatternId']) in zero_z),
                algebraic_roots=len(algebraic),trigonometric_roots=len(patterns)-len(algebraic),
                full_free_pattern_id=next(p['id'] for p in patterns if all(p['mask'])))


def main():
    S=integer_matrix()
    result=dict(scope='Saved 152/156 candidate and current default 729-pattern model; no global infeasibility claim',
                code_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                             for name in ('Optim_Algorithm.m','strict_layout_minlp.py')},
                integer_matrix=S.tolist(),matrix_scale='1/(10*sqrt(2))',
                exact_strict_2_4_6=coverage(S,{2,4,6}),
                exact_strict_even_up_to_12=coverage(S,set(range(2,13,2))),
                exact_strict_any_up_to_12=coverage(S,set(range(1,13))),
                parity_certificate=parity_certificate(),patterns=pattern_audit(),
                continuous_relaxation=continuous_relaxation(S))
    path=ROOT/'analysis/layout_feasibility_audit_20260909.json'
    path.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    compact={k:{a:b for a,b in v.items() if a not in ('certificates','results','symbolically_zero_Fz_ids')}
             for k,v in result.items() if isinstance(v,dict)}
    print(json.dumps(compact,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
