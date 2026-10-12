"""Bounded multistart physical search with dynamic healthy 2/4/6 group choice.

This is a heuristic, never an infeasibility proof. All candidates go through
the independent full 156-demand checker; bounds fix every structural zero.
"""
import time
import itertools
import numpy as np
from scipy.optimize import least_squares
from strict_layout_topology import physical_matrix, group_library, evaluate_candidate, CASES


def search(config, state, seconds, seeds=(), progress=None):
    deadline=time.monotonic()+seconds
    lower=np.array(config['lower']);upper=np.array(config['upper']);free=upper>lower
    width=upper[free]-lower[free];library=group_library(6);_,bits=library
    healthy=[np.flatnonzero(bits[f]==0) for f in (0,4,8)]
    pairs=np.array(list(itertools.combinations(range(12),2)))
    eps=config['pure_tolerance'];mu=config['min_axis_capacity']-eps
    separation=max(config.get('min_install_distance',0),config.get('strict_collision_distance',0))
    rng=np.random.default_rng(int(config.get('seed',18))+state.get('calls',0)*7919)
    state['calls']=state.get('calls',0)+1
    evaluations=0;starts=0;candidates=[];found=False

    class Done(Exception): pass

    def unpack(u):
        x=lower.copy();x[free]=lower[free]+width*np.clip(u,0,1);return x

    def residual(u):
        nonlocal evaluations,found
        if time.monotonic()>=deadline: raise Done()
        x=unpack(u);A,r=physical_matrix(x,config['x_face'])
        W=np.einsum('ai,ig->ag',A,bits)
        output=[]
        for ids in healthy:
            values=W[:,ids]
            for label in range(12):
                axis=label//2;sign=-1 if label%2==0 else 1
                off=np.sign(values)*np.maximum(abs(values)-eps,0)
                off[axis]=np.maximum(mu-sign*values[axis],0)
                best=int(np.argmin(np.sum(off*off,axis=0)))
                output.extend(off[:,best]/mu)
        distances=np.linalg.norm(r[pairs[:,0]]-r[pairs[:,1]],axis=1)
        output.extend(np.maximum(separation-distances,0)/config['x_face'])
        if config.get('min_install_angle_deg',0)>0:
            cosines=np.sum(A[:3,pairs[:,0]]*A[:3,pairs[:,1]],axis=0)
            output.extend(np.maximum(cosines-np.cos(np.deg2rad(config['min_install_angle_deg'])),0))
        output=np.array(output);score=float(np.dot(output,output));evaluations+=1
        if score<state.get('best_score',float('inf')):
            state['best_score']=score;state['best_x']=x.tolist()
            check=evaluate_candidate(x,config,library)
            state['candidate_checks']=state.get('candidate_checks',0)+1
            if check['coverage']>state.get('best_coverage',-1):
                state['best_coverage']=check['coverage'];state['coverage_x']=x.tolist()
            if check['has_solution']:
                candidates.append(x.tolist());found=True;raise Done()
            if progress: progress()
        return output

    points=[]
    for x in [state.get('best_x',[]),*seeds]:
        if len(x)==12:
            points.append(np.clip((np.array(x)[free]-lower[free])/width,0,1))
    while time.monotonic()<deadline and not found:
        if starts<len(points): u=points[starts]
        elif points and starts%2:
            u=np.clip(points[0]+rng.normal(0,.08,free.sum()),0,1)
        else: u=rng.uniform(.02,.98,free.sum())
        starts+=1
        try:
            if free.any(): least_squares(residual,u,bounds=(0,1),max_nfev=60,ftol=1e-9,xtol=1e-10,gtol=1e-9)
            else: residual(u);break
        except Done: break
    state['evaluations']=state.get('evaluations',0)+evaluations
    state['starts']=state.get('starts',0)+starts
    for name in ('best_x','coverage_x'):
        if state.get(name) and state[name] not in candidates:candidates.append(state[name])
    return dict(candidates=candidates,has_solution=found,evaluations=evaluations,starts=starts)
