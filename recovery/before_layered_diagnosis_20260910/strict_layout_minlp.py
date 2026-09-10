"""SCIP backend for the 12-variable, even-group, single-fault layout model.

The MATLAB caller independently checks every returned layout. A time limit is
not an infeasibility certificate. No sign-pattern library is used here.
"""

import argparse
import json
import math
import platform
import traceback
import itertools
import random
import time
import hashlib
import copy
from pathlib import Path

import numpy as np
import pyscipopt
from pyscipopt import Model, cos, quicksum, sin, Eventhdlr, SCIP_EVENTTYPE, SCIP_PARAMEMPHASIS


# Stop phase-I search only after every representative requirement is satisfied.
class FullCoverageStop(Eventhdlr):
    def eventinit(self):
        self.model.catchEvent(SCIP_EVENTTYPE.BESTSOLFOUND, self)

    def eventexit(self):
        self.model.dropEvent(SCIP_EVENTTYPE.BESTSOLFOUND, self)

    def eventexec(self, event):
        if self.model.getSolObjVal(self.model.getBestSol()) >= 35.5:
            self.model.interruptSolve()


# For fixed directions, every group torque is affine in the six positions.
def fixed_direction_data(angles, a, max_group_size=6):
    masks = np.array([[(n >> i) & 1 for i in range(12)] for n in range(1, 4096)
                      if 0 < bin(n).count('1') <= max_group_size and bin(n).count('1') % 2 == 0], dtype=float).T
    force, offset, coefficients = np.zeros((3, 12)), np.zeros((3, 12)), np.zeros((3, 12, 6))
    sx, sy, sm = np.array([-1, -1, 1, 1]), np.array([1, -1, 1, -1]), np.array([-1, 1, 1, -1])
    for m, (alpha, beta) in enumerate(angles):
        c, p, q = math.cos(beta), math.sin(beta)*math.cos(alpha), math.sin(beta)*math.sin(alpha)
        indices = slice(4*m, 4*m+4)
        force[:, indices] = [sx*c, sy*p, sy*q]
        offset[1, indices], offset[2, indices] = sm*q, -sm*p
        coefficients[0, indices, 2*m], coefficients[0, indices, 2*m+1] = q/a, -p/a
        coefficients[1, indices, 2*m+1], coefficients[2, indices, 2*m] = sm*c/a, -sm*c/a
    return masks, force, np.einsum('ai,ig->ag', force, masks), np.einsum('ai,ig->ag', offset, masks), np.einsum('aip,ig->agp', coefficients, masks)


# Select healthy even groups jointly with positions; partial coverage is never a certified layout.
def solve_fixed_directions(config, angles, seconds):
    started = time.monotonic()
    masks, force, F, H, K = fixed_direction_data(angles, float(config['x_face']), int(config.get('max_group_size',6)))
    low = np.array(config['lower']).reshape(3, 4)[:, :2].reshape(-1)
    high = np.array(config['upper']).reshape(3, 4)[:, :2].reshape(-1)
    rho = float(config['min_axis_capacity'])
    minimum = H + np.minimum(K*low, K*high).sum(axis=2)
    maximum = H + np.maximum(K*low, K*high).sum(axis=2)
    cases = []
    for fault in (0, 4, 8):
        for label in range(12):
            axis, sign = label//2, (-1 if label % 2 == 0 else 1)
            allowed = masks[fault] == 0
            if axis < 3:
                allowed &= np.max(np.abs(F[[r for r in range(3) if r != axis]]), axis=0) <= 1e-12
                allowed &= sign*F[axis] >= rho
                zero_rows = list(range(3))
            else:
                allowed &= np.max(np.abs(F), axis=0) <= 1e-12
                allowed &= (maximum[axis-3] if sign > 0 else -minimum[axis-3]) >= rho
                zero_rows = [r for r in range(3) if r != axis-3]
            if axis < 3 and not np.any(allowed):
                return {'status': 'force_direction_infeasible', 'missing_case': [fault+1, label+1], 'has_solution': False}
            for row in zero_rows:
                allowed &= (minimum[row] <= 1e-12) & (maximum[row] >= -1e-12)
            ids = np.flatnonzero(allowed)
            if not len(ids):
                return {'status': 'direction_or_position_bounds_infeasible', 'missing_case': [fault+1, label+1], 'has_solution': False}
            cases.append((ids, zero_rows, sign))
    angle = float(config['min_install_angle_deg'])
    if angle > 0 and any(float(np.dot(force[:, i], force[:, j])) > math.cos(math.radians(angle))+1e-12 for i in range(12) for j in range(i+1, 12)):
        return {'status': 'installation_angle_infeasible', 'has_solution': False}
    model = Model('fixed_direction_group_feasibility')
    model.hideOutput()
    model.setRealParam('numerics/feastol', min(1e-9, float(config.get('pure_tolerance', 1e-9))))
    model.setEmphasis(SCIP_PARAMEMPHASIS.FEASIBILITY, quiet=True)
    model.setIntParam('randomization/randomseedshift', int(config.get('seed', 18)) % 2147483647)
    pos = [model.addVar(f'position_{i}', lb=float(low[i]), ub=float(high[i])) for i in range(6)]
    distance = float(config['min_install_distance'])
    if distance > 0:
        positions = [[sx*float(config['x_face']), st*pos[2*m], st*pos[2*m+1]]
                     for m in range(3) for sx, st in ((1, 1), (1, -1), (-1, 1), (-1, -1))]
        for i in range(12):
            for j in range(i+1, 12):
                model.addCons(quicksum((positions[i][r]-positions[j][r])**2 for r in range(3)) >= distance**2)
    active, choices = [], []
    for case_index, (ids, zero_rows, sign) in enumerate(cases):
        task = model.addVar(f'covered_{case_index}', vtype='B')
        active.append(task)
        choices.append([])
        for group in ids:
            chosen = model.addVar(f'group_{case_index}_{group}', vtype='B')
            choices[-1].append((int(group), chosen))
            for row in range(3):
                value = float(H[row, group])+quicksum(float(K[row, group, j])*pos[j] for j in range(6) if abs(K[row, group, j]) > 1e-15)
                if row in zero_rows:
                    model.addCons(value <= max(0, float(maximum[row, group]))*(1-chosen))
                    model.addCons(value >= min(0, float(minimum[row, group]))*(1-chosen))
                else:
                    bound = float(minimum[row, group] if sign > 0 else -maximum[row, group])
                    model.addCons(sign*value >= rho-max(0, rho-bound)*(1-chosen))
        model.addCons(quicksum(v for _, v in choices[-1]) == task)
    model.setObjective(quicksum(active), 'maximize')
    model.includeEventhdlr(FullCoverageStop(), 'complete', 'Stop at the first fully covered candidate')
    model.setRealParam('limits/time', max(.001, seconds-(time.monotonic()-started)))
    model.optimize()
    result = {'status': str(model.getStatus()), 'has_solution': False, 'coverage36': 0, 'nodes': model.getNNodes(), 'solve_time': time.monotonic()-started}
    if model.getNSols():
        sol = model.getBestSol()
        y = [model.getSolVal(sol, v) for v in pos]
        if model.getSolObjVal(sol) >= 35.5:
            rows, rhs = [], []
            for case_index, group in enumerate(choices):
                for g, chosen in group:
                    if model.getSolVal(sol, chosen) > .5:
                        for row in cases[case_index][1]:
                            rows.append(K[row, g]); rhs.append(-H[row, g])
            matrix = np.array(rows)
            correction = np.linalg.lstsq(matrix, np.array(rhs)-np.einsum('ij,j->i', matrix, y), rcond=1e-12)[0]
            polished = np.array(y)+correction
            if np.all(polished >= low-1e-10) and np.all(polished <= high+1e-10) and distance == 0:
                y = polished.tolist()
        result['x'] = [v for m in range(3) for v in (*y[2*m:2*m+2], *angles[m])]
        result['coverage36'] = int(round(model.getSolObjVal(sol)))
        W = np.vstack((F, H+np.einsum('agp,p->ag', K, y)))
        coverage, selected = 0, {}
        tolerance = float(config.get('pure_tolerance', 1e-9))
        for label in range(12):
            axis, sign = label//2, (-1 if label % 2 == 0 else 1)
            good = (np.max(np.abs(W[[r for r in range(6) if r != axis]]), axis=0) <= tolerance) & (sign*W[axis] >= rho-tolerance)
            for fault in range(13):
                eligible = good if fault == 0 else good & (masks[fault-1] == 0)
                indices = np.flatnonzero(eligible)
                if len(indices):
                    coverage += 1
                    if fault in (1, 5, 9):
                        selected[fault, label] = [int(i)+1 for i in np.flatnonzero(masks[:, indices[0]])]
        result['coverage'] = coverage
        result['has_solution'] = coverage == 156
        result['groups'] = [selected[key] for key in sorted(selected)]
    return result


# Preserve historical directions, then explore exact force-balance families and integer-sphere directions.
def direction_candidates(config):
    lower, upper = np.array(config['lower']).reshape(3, 4), np.array(config['upper']).reshape(3, 4)
    seeds = np.array(config.get('search_seeds', []))
    candidates = [x.reshape(3, 4)[:, 2:].tolist() for x in np.atleast_2d(seeds)] if seeds.size else []
    for beta in (math.atan(.1), math.atan(.2), math.atan(.3), math.pi/8, math.pi/4, math.pi/3):
        # Changing these signs is a new physical candidate, not an equivalence reduction.
        for ratio in (2, 3, 4):
            angle = math.acos(1/ratio)
            for first, second in ((0, angle), (math.pi, angle), (0, math.pi-angle), (math.pi, math.pi-angle)):
                candidates.append([[first, beta], [second, beta], [(-second) % (2*math.pi), beta]])
        for phase in (0, math.pi/2):
            candidates.append([[(phase+j*2*math.pi/3) % (2*math.pi), beta] for j in range(3)])
        for alpha in (math.pi/4, math.atan(.5), math.pi/6):
            candidates.append([[alpha, beta], [math.pi-alpha, beta], [math.pi+alpha, beta]])
    rng = random.Random(int(config.get('seed', 18)))
    for norm in (9, 49, 81, 121, 169, 289):
        limit = math.isqrt(norm)
        vectors = [(c, p, q) for c in range(1, limit+1) for p in range(-limit, limit+1) for q in range(-limit, limit+1) if c*c+p*p+q*q == norm]
        triples = list(itertools.combinations(vectors, 3))
        rng.shuffle(triples)
        for triple in triples[:100]:
            candidates.append([[math.atan2(q, p) % (2*math.pi), math.atan2(math.hypot(p, q), c)] for c, p, q in triple])
    seen = set()
    for angles in candidates:
        value = np.array(angles)
        key = tuple(np.round(value.reshape(-1), 12))
        if key not in seen and np.all(value >= lower[:, 2:]-1e-14) and np.all(value <= upper[:, 2:]+1e-14):
            seen.add(key)
            yield angles


def build_model(config):
    """Build the joint geometry/binary-group model without restricting topology."""
    if config.get('max_group_size',6) not in (2,4,6):
        raise ValueError('A single-axis strict group must contain at most six thrusters.')
    low, high = config['lower'], config['upper']
    if config.get('algebraic_model', True) and all(0 <= low[4*m+3] <= high[4*m+3] <= math.pi/2 for m in range(3)):
        return build_algebraic_model(config)
    model = Model("strict_satellite_layout")
    if not config.get("verbose", False):
        model.hideOutput()
    model.setRealParam("limits/time", float(config["max_time"]))
    model.setRealParam("numerics/feastol", 1e-9)
    model.setIntParam("limits/solutions", 1)
    model.setIntParam("randomization/randomseedshift", int(config.get("seed", 18)) % 2147483647)
    lower, upper = config["lower"], config["upper"]
    x = [model.addVar(f"x_{i}", lb=lo, ub=hi) for i, (lo, hi) in enumerate(zip(lower, upper))]
    a = float(config["x_face"])
    positions, directions, wrench = [], [], []
    for module in range(3):
        y, z, alpha, beta = x[4 * module:4 * module + 4]
        c, p, q = cos(beta), sin(beta) * cos(alpha), sin(beta) * sin(alpha)
        for sx, st in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            r = [sx * a, st * y, st * z]
            d = [-sx * c, st * p, st * q]
            moment = [(r[1] * d[2] - r[2] * d[1]) / a,
                      (r[2] * d[0] - r[0] * d[2]) / a,
                      (r[0] * d[1] - r[1] * d[0]) / a]
            positions.append(r)
            directions.append(d)
            wrench.append(d + moment)

    # Module permutation is a true symmetry because all three have identical
    # parameter bounds. No independent position/direction sign flips are made.
    if config.get('order_modules',True):
        model.addCons(x[0] <= x[4])
        model.addCons(x[4] <= x[8])
    distance = float(config["min_install_distance"])
    angle = float(config["min_install_angle_deg"])
    for i in range(12):
        for j in range(i + 1, 12):
            if distance > 0:
                model.addCons(quicksum((positions[i][k] - positions[j][k]) ** 2 for k in range(3)) >= distance ** 2)
            if angle > 0:
                model.addCons(quicksum(directions[i][k] * directions[j][k] for k in range(3)) <= math.cos(math.radians(angle)))

    # The threshold is fixed, not an objective to improve. Target capacities
    # can exceed it; they are not constrained to equal the threshold.
    rho = float(config["min_axis_capacity"])
    groups, pairs = {}, {}
    # Within every module, permutations (1 2)(3 4) and (1 3)(2 4)
    # change only the signs of physical wrench rows. Applying them to all
    # modules maps each failure to its module's first thruster. Both signs of
    # all six axes are retained, so checking faults 1,5,9 is exactly equivalent
    # to checking all 12 single failures. Nominal feasibility follows as well.
    for fault in (0, 4, 8):
        for label in range(12):
            axis, sign = label // 2, (-1 if label % 2 == 0 else 1)
            key = (fault, label)
            group = [model.addVar(f"z_{fault}_{label}_{i}", vtype="B", ub=0 if i == fault else 1) for i in range(12)]
            pairs[key] = model.addVar(f"pairs_{fault}_{label}", vtype="I", lb=1, ub=int(config.get('max_group_size',6))//2)
            model.addCons(quicksum(group) == 2 * pairs[key])
            for row in range(6):
                value = quicksum(wrench[i][row] * group[i] for i in range(12))
                if row == axis:
                    model.addCons(sign * value >= max(0,rho-float(config.get("pure_tolerance",1e-9))))
                else:
                    model.addCons(value <= float(config.get("pure_tolerance",1e-9)))
                    model.addCons(value >= -float(config.get("pure_tolerance",1e-9)))
            groups[key] = group
    model.setObjective(0.0, "minimize")
    return model, x, rho, groups, pairs


# Share physical coefficients; binary products use exact linear inequalities, not repeated trigonometric equations.
def build_algebraic_model(config):
    from strict_layout_topology import build_coefficients, CASES
    model, data, groups, pairs = build_coefficients(config)
    return model, data, float(config['min_axis_capacity']), dict(zip(CASES, groups)), pairs


# Enumerate all 729 labelled zero/free patterns; free intervals include their zero boundary.
def zero_free_patterns(lower,upper):
    pair_patterns = ((1,0),(0,1),(1,1))
    modules = [p+d for p in pair_patterns for d in pair_patterns]
    masks = sorted((sum(p,()) for p in itertools.product(modules,repeat=3)),key=lambda p:(sum(p),p))
    patterns, pending = [], []
    for index, mask in enumerate(masks):
        low, high = list(lower), list(upper)
        compatible = True
        for axis, free in enumerate(mask):
            if not free:
                compatible &= low[axis] <= 0 <= high[axis]
                low[axis] = high[axis] = 0
        patterns.append(dict(id=index+1,mask=list(mask),free_count=sum(mask),calls=0,
                             status='pending' if compatible else 'bounds_excluded',elapsed=0.0))
        if compatible:
            pending.append(dict(id=index,pattern_id=index+1,depth=0,round=0,lower=low,upper=high))
    return patterns, pending


# Sweep every pattern once before refinement; the original fair-region mode remains available.
def solve_fair(config):
    started = time.monotonic()
    fields = ('lower','upper','x_face','min_install_distance','min_install_angle_deg','min_axis_capacity')
    domain = {k: config[k] for k in fields}
    domain.update(max_group_size=int(config.get('max_group_size',6)), pure_tolerance=config.get('pure_tolerance',1e-9))
    patterned = config.get('pattern_search',False)
    if patterned:
        domain['enumeration'] = 'zero_free_729_v1'
    signature = hashlib.sha256(json.dumps(domain,sort_keys=True).encode()).hexdigest()
    state = copy.deepcopy(config.get('search_state') or {})
    if state.get('version') != (2 if patterned else 1) or state.get('signature') != signature:
        note = 'new_queue' if not state else 'constraints_changed_queue_reset'
        state = dict(version=1,signature=signature,processed=0,pruned=0,next_id=1,history=[],
                     pending=[dict(id=0,depth=0,round=0,lower=list(config['lower']),upper=list(config['upper']))])
        if patterned:
            state['patterns'],state['pending'] = zero_free_patterns(config['lower'],config['upper'])
            state['version'],state['next_id'] = 2,729
    else:
        note = 'resumed_queue'
    # MATLAB encodes a one-element struct array as an object, not a JSON array.
    for name in ('pending','history'):
        if isinstance(state[name],dict):
            state[name] = [state[name]]
    scale = np.maximum(np.array(config['upper'])-config['lower'],1e-15)
    seeds = np.array(config.get('search_seeds',[]))
    seeds = np.atleast_2d(seeds)[:16] if seeds.size else np.empty((0,12))
    output = dict(status='timelimit',has_solution=False,search_goal='first_feasible',x=[],groups=[],
                  solution_count=0,nodes=0,method='fair_spatial_joint_feasibility',resume_note=note)
    if patterned:
        output['method'] = 'zero_free_729_pattern_search'
    while state['pending'] and time.monotonic()-started < float(config['max_time']):
        if patterned:
            # At equal depth, rotate across classes before giving a class its next region.
            index = min(range(len(state['pending'])),key=lambda i:(state['pending'][i]['depth'],
                        state['patterns'][state['pending'][i]['pattern_id']-1]['calls'],state['pending'][i]['id']))
        elif state['processed'] % 2 == 0 or not len(seeds):
            index = min(range(len(state['pending'])),key=lambda i:(state['pending'][i]['depth'],state['pending'][i]['id']))
        else:
            def priority(node):
                gap = np.maximum(0,np.maximum(np.array(node['lower'])-seeds,seeds-np.array(node['upper'])))/scale
                return (float(np.min(np.sum(gap*gap,axis=1))),node['depth'],node['id'])
            index = min(range(len(state['pending'])),key=lambda i:priority(state['pending'][i]))
        node = state['pending'].pop(index)
        remaining = float(config['max_time'])-(time.monotonic()-started)
        budget = min(remaining,float(config.get('node_time',5 if patterned else 15))*2**min(node['depth'] if patterned else node['round'],4))
        child_config = dict(config,fair_search=False,pattern_search=False,fixed_direction_search=False,verbose=False,
                            order_modules=not patterned,lower=node['lower'],upper=node['upper'],max_time=max(.001,budget))
        node_started = time.monotonic()
        trial = solve(child_config)
        output['nodes'] += trial.get('nodes',0)
        state['processed'] += 1
        state['history'].append(dict(id=node['id'],depth=node['depth'],status=trial['status'],budget=budget,
                                     lower=node['lower'],upper=node['upper'],nodes=trial.get('nodes',0)))
        state['history'] = state['history'][-256:]
        if patterned:
            pattern = state['patterns'][node['pattern_id']-1]
            pattern['calls'] += 1
            pattern['elapsed'] += time.monotonic()-node_started
            pattern['status'] = 'unresolved'
            state['history'][-1]['pattern_id'] = node['pattern_id']
        if trial['has_solution']:
            output.update(trial)
            output['method'] = 'fair_spatial_joint_feasibility'
            if patterned:
                output['method'],output['pattern_id'] = 'zero_free_729_pattern_search',node['pattern_id']
                pattern['status'] = 'candidate_for_matlab_verification'
                output['free_mask'] = pattern['mask']
                output['actual_zero_mask'] = [int(v == 0) for v in trial['x']]
        # Split unproved regions even for a candidate: a rejected candidate must
        # not restart the exact same region indefinitely on the next run.
        if trial['status'] == 'infeasible' and not trial['has_solution']:
            state['pruned'] += 1
        else:
            widths = (np.array(node['upper'])-node['lower'])/scale
            axis = int(np.argmax(widths))
            middle = (node['lower'][axis]+node['upper'][axis])/2
            if node['lower'][axis] < middle < node['upper'][axis]:
                for side in (0,1):
                    child = dict(id=state['next_id'],depth=node['depth']+1,round=0,lower=list(node['lower']),upper=list(node['upper']))
                    child['upper' if side == 0 else 'lower'][axis] = middle
                    if patterned:
                        child['pattern_id'] = node['pattern_id']
                    state['pending'].append(child)
                    state['next_id'] += 1
            else:
                node['round'] += 1
                if patterned:
                    node['depth'] += 1
                state['pending'].append(node)
        if patterned and not any(n['pattern_id']==node['pattern_id'] for n in state['pending']):
            pattern['status'] = 'numerically_infeasible'
        if config.get('progress_path'):
            path = Path(config['progress_path']); temporary = path.with_suffix('.tmp')
            progress = dict(coverage=0,attempt=state['processed'],stage='fair_partition',pending=len(state['pending']))
            if patterned:
                progress.update(stage='zero_free_patterns',patterns_seen=sum(p['status']!='pending' for p in state['patterns']),
                                pattern_id=node['pattern_id'],free_count=pattern['free_count'])
            temporary.write_text(json.dumps(progress),encoding='utf-8')
            temporary.replace(path)
        if config.get('checkpoint_path'):
            path = Path(config['checkpoint_path']); temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(dict(search_state=state,search_seeds=config.get('search_seeds',[])),allow_nan=False),encoding='utf-8')
            temporary.replace(path)
        if config.get('verbose',False):
            label = f"Pattern {node['pattern_id']}/729, free={pattern['free_count']}, " if patterned else ''
            print(f"{label}region {node['id']}, depth {node['depth']}: {trial['status']}; pending {len(state['pending'])}",flush=True)
        if trial['has_solution']:
            break
    if not state['pending']:
        output['status'] = 'infeasible'
    output.update(search_state=state,solve_time=time.monotonic()-started)
    if config.get('checkpoint_path'):
        output['checkpoint_file'] = config['checkpoint_path']
    return output


def solve(config):
    """Return solver termination evidence and the incumbent, when one exists."""
    if config.get('decomposition_search',False):
        if all(0 <= config['lower'][4*m+3] <= config['upper'][4*m+3] <= math.pi/2 for m in range(3)):
            from strict_layout_topology import solve_decomposed
            return solve_decomposed(config)
        result = solve(dict(config,decomposition_search=False,fair_search=False,pattern_search=False))
        result['fallback_reason'] = 'Beta range outside algebraic hemisphere; original joint model retained.'
        return result
    if config.get('fair_search',False) or config.get('pattern_search',False):
        return solve_fair(config)
    if config.get('fixed_direction_search', False):
        started, attempts, candidates = time.monotonic(), [], []
        deadline = started+float(config['max_time'])*float(config.get('fixed_direction_fraction', .85))
        best_score = -1
        for angles in direction_candidates(config):
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                break
            trial = solve_fixed_directions(config, angles, min(remaining, float(config.get('fixed_direction_time', 8))))
            attempts.append(dict(angles=angles, **{k: v for k, v in trial.items() if k not in ('x', 'groups')}))
            if 'x' in trial and trial.get('coverage', 0) > best_score:
                best_score = trial.get('coverage', 0)
                candidates.append(trial['x'])
            if config.get('verbose', False):
                print(f"Fixed directions {len(attempts)}: {trial['status']}, verified coverage {trial.get('coverage', 0)}/156", flush=True)
            if config.get('progress_path'):
                path = Path(config['progress_path'])
                temporary = path.with_suffix('.tmp')
                temporary.write_text(json.dumps({'coverage': max(0, best_score), 'attempt': len(attempts), 'stage': 'fixed_direction'}), encoding='utf-8')
                temporary.replace(path)
            if trial['has_solution']:
                trial.update(search_goal='first_feasible', solution_count=1, capacity_threshold=config['min_axis_capacity'],
                             representative_faults=[1, 5, 9], candidates=candidates, fixed_direction_attempts=attempts,
                             method='fixed_direction_position_group_search', environment={'python': platform.python_version(), 'pyscipopt': pyscipopt.__version__})
                return trial
        remaining = float(config['max_time'])-(time.monotonic()-started)
        if remaining > 0:
            if config.get('progress_path'):
                path = Path(config['progress_path'])
                temporary = path.with_suffix('.tmp')
                temporary.write_text(json.dumps({'coverage': max(0,best_score), 'attempt': len(attempts), 'stage': 'joint_minlp'}), encoding='utf-8')
                temporary.replace(path)
            fallback = dict(config, fixed_direction_search=False, max_time=remaining)
            if candidates:
                old_seeds = np.array(config.get('search_seeds', []))
                old_seeds = np.atleast_2d(old_seeds).tolist() if old_seeds.size else []
                fallback['search_seeds'] = old_seeds[:1]+list(reversed(candidates[-2:]))+old_seeds[1:]
            output = solve(fallback)
        else:
            output = dict(status='timelimit', has_solution=False, search_goal='first_feasible', x=[], groups=[], solution_count=0)
        output.update(candidates=candidates, fixed_direction_attempts=attempts)
        return output
    started = time.monotonic()
    model, x, rho, groups, pairs = build_model(config)
    model.setRealParam('limits/time',max(.001,float(config['max_time'])-(time.monotonic()-started)))
    model.optimize()
    status = str(model.getStatus())
    has_solution = model.getNSols() > 0
    dual = model.getDualbound()
    output = {"status": status, "has_solution": has_solution, "search_goal": "first_feasible",
              "method": "shared_coefficient_joint_search" if isinstance(x,dict) else "trigonometric_joint_search",
              "solve_time": model.getSolvingTime(), "nodes": model.getNNodes(),
              "solution_count": model.getNSols(), "capacity_threshold": rho,
              "environment": {"python": platform.python_version(), "pyscipopt": pyscipopt.__version__,
                              "scip": f"{model.getMajorVersion()}.{model.getMinorVersion()}.{model.getTechVersion()}"},
              "dual_bound": dual if abs(dual) < 1e19 else None,
              "primal_bound": None, "relative_gap": None,
              "x": [], "representative_faults": [1, 5, 9], "groups": []}
    if has_solution:
        solution = model.getBestSol()
        if isinstance(x, dict):
            from strict_layout_topology import solution_x
            output['x'] = solution_x(model,x,config)
            output['method'] = 'shared_coefficient_joint_search'
        else:
            output["x"] = [model.getSolVal(solution, v) for v in x]
        # These bounds concern the constant feasibility objective, not the
        # satellite's maximum control capacity. MATLAB never labels it optimal.
        output["primal_bound"] = model.getSolObjVal(solution)
        gap = model.getGap()
        output["relative_gap"] = gap if math.isfinite(gap) else None
        output["groups"] = [[i + 1 for i, v in enumerate(group) if model.getSolVal(solution, v) > 0.5]
                            for group in groups.values()]
    return output


def main():
    """Exchange structured configuration/results with MATLAB through JSON."""
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    config = json.loads(args.input.read_text(encoding="utf-8"))
    try:
        result = solve(config)
    except Exception as error:
        result = {"status": "error", "has_solution": False,
                  "error": {"identifier": type(error).__name__, "message": str(error),
                            "report": traceback.format_exc()}}
    args.output.write_text(json.dumps(result, ensure_ascii=False, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
