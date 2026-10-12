"""Finite, proof-based screening of disjoint zero/nonzero parameter supports.

No numerical optimization, epsilon replacement of !=0, or layout feasibility
claim is made here. A survivor is unresolved under this version's rule set.
"""
import hashlib
import csv
import itertools
import json
import math
from pathlib import Path
from fractions import Fraction

VERSION = 'strict_support_analytic_v1'
PROOFS = {
    'bounds': 'A zero/nonzero requirement has empty intersection with its parameter interval.',
    'internal_collision': 'y=z=0 makes the two installations on each X face coincide.',
    'pair_distance_upper': 'An installation pair has a rational squared-distance upper bound equal to zero or smaller than the required minimum squared distance.',
    'zero_Fz': 'Every module has alpha=0 or beta=0; q=sin(beta)sin(alpha)=0, so Fz is identically zero.',
    'Mx_one_sign': 'h=(y*q-z*p)/a is identically zero except in at most one module. All four Mx entries of a module equal h. Every invocation has Mx=sum(n_m*h_m), n_m>=0; both nonzero signs are impossible.',
}


def h_possible(bits):
    y, z, alpha, beta = bits
    return bool(beta and (z or (y and alpha)))


def distance_obstruction(pattern, lower, upper, config):
    """Conservative interval upper bound; dependencies may only enlarge it."""
    a=Fraction(str(config['x_face']))
    minimum=Fraction(str(config.get('min_install_distance',0)))**2
    points=[]
    for m,bits in enumerate(pattern):
        intervals=[(Fraction(str(lower[4*m+j])),Fraction(str(upper[4*m+j])))
                   if bits[j] else (Fraction(0),Fraction(0)) for j in (0,1)]
        for sx,st in ((1,1),(1,-1),(-1,1),(-1,-1)):
            points.append([(sx*a,sx*a)]+[(l,h) if st==1 else (-h,-l) for l,h in intervals])
    for p,q in itertools.combinations(points,2):
        bound=sum(max(abs(l-h2),abs(h-l2))**2 for (l,h),(l2,h2) in zip(p,q))
        if bound==0 or bound<minimum:
            return True
    return False


def reasons(pattern, lower, upper, config):
    flat = sum(pattern, ())
    result = []
    if any((not b and not lo <= 0 <= hi) or (b and lo == hi == 0)
           for b, lo, hi in zip(flat, lower, upper)):
        result.append('bounds')
    if any(not (b[0] or b[1]) for b in pattern):
        result.append('internal_collision')
    if not result and distance_obstruction(pattern,lower,upper,config):
        result.append('pair_distance_upper')
    if all(not b[2] or not b[3] for b in pattern):
        result.append('zero_Fz')
    if sum(h_possible(b) for b in pattern) < 2:
        result.append('Mx_one_sign')
    return result


def strict_installation_domain(x, config, support=None):
    """Point-domain check; no epsilon can turn zero or coincidence into valid.

    This checks supplied floating-point values, not a certified enclosure of
    an unknown exact layout. Geometry certification remains a separate stage.
    """
    if len(x) != 12 or not all(math.isfinite(v) for v in x):
        return False
    if any(not lo <= v <= hi for v, lo, hi in zip(x, config['lower'], config['upper'])):
        return False
    if support is not None:
        if len(support)!=12 or any(b not in (0,1) for b in support):
            return False
        if any(bool(v != 0) != bool(b) for v, b in zip(x, support)):
            return False
    a = config['x_face']
    points = [(sx*a, st*x[4*m], st*x[4*m+1]) for m in range(3)
              for sx, st in ((1, 1), (1, -1), (-1, 1), (-1, -1))]
    minimum = config.get('min_install_distance', 0)
    return all(p != q and math.dist(p, q) >= minimum
               for p, q in itertools.combinations(points, 2))


def screen(config):
    lo, hi = list(config['lower']), list(config['upper'])
    if len(lo) != 12 or len(hi) != 12 or any(not math.isfinite(v) for v in lo+hi) or any(a>b for a,b in zip(lo,hi)):
        raise ValueError('Twelve finite ordered bounds required.')
    if config.get('max_group_size', 6) != 6:
        raise ValueError('Structural stage uses the requested 2/4/6 thruster contract.')
    if not config['x_face'] > 0 or not config['min_axis_capacity'] > config.get('pure_tolerance', 1e-9) > 0:
        raise ValueError('Positive face distance and nonzero target threshold required.')
    if not math.isfinite(config.get('min_install_distance',0)) or config.get('min_install_distance',0)<0:
        raise ValueError('Minimum installation distance must be finite and nonnegative.')
    # Module relabelling is valid only for identical module domains and the
    # uniform all-single-fault contract. Do not additionally order y values.
    symmetric = all(lo[4*m:4*m+4] == lo[:4] and hi[4*m:4*m+4] == hi[:4] for m in (1, 2))
    modules = list(itertools.product((0, 1), repeat=4))
    records = {}
    for pattern in itertools.product(modules, repeat=3):
        canonical = tuple(sorted(pattern)) if symmetric else pattern
        key = ''.join(str(b) for module in canonical for b in module)
        if key not in records:
            why = reasons(canonical, lo, hi, config)
            records[key] = dict(id=key, modules=canonical, support=sum(canonical, ()),
                                labelled_multiplicity=0, reasons=why,
                                status='excluded_proven' if why else 'unresolved')
        records[key]['labelled_multiplicity'] += 1
    rows = list(records.values())
    after = lambda forbidden: [r for r in rows if not set(r['reasons']).intersection(forbidden)]
    stages = []
    rules = []
    for rule in ('bounds', 'internal_collision', 'pair_distance_upper', 'zero_Fz', 'Mx_one_sign'):
        rules.append(rule)
        kept = after(rules)
        stages.append(dict(rule=rule, representative_remaining=len(kept),
                           labelled_remaining=sum(r['labelled_multiplicity'] for r in kept)))
    scope = {k: config.get(k) for k in ('lower','upper','x_face','min_axis_capacity','pure_tolerance','min_install_distance','min_install_angle_deg')}
    scope.update(max_group_size=6, strict_nonzero=True, forbid_coincidence=True, version=VERSION)
    signature = hashlib.sha256(json.dumps(scope, sort_keys=True).encode()).hexdigest()
    survivors = [r for r in rows if r['status'] == 'unresolved']
    result = dict(status='structural_screen_complete', has_solution=False, feasible=False,
                  version=VERSION, scope=scope, scope_signature=signature,
                  symmetry_reduced=symmetric, allowed_group_sizes=[2,4,6],
                  representative_total=len(rows), labelled_total=4096,
                  representative_remaining=len(survivors),
                  labelled_remaining=sum(r['labelled_multiplicity'] for r in survivors),
                  stages=stages, proofs=PROOFS, patterns=rows,
                  strict_constraints=['support bit 0: x=0; bit 1: x!=0',
                                      'v_m != 0; v_m != v_n; v_m != -v_n',
                                      'all 66 pair distances >= min_install_distance and > 0'],
                  limitation='Complete scan of the documented rules only; survivors are not certified feasible or immune to further analytic exclusions.')
    result['message'] = f"解析筛选完成：剩余{len(survivors)}种代表模式（{result['labelled_remaining']}种带编号模式）；允许2/4/6台。尚未证明可行，重复运行相同规则不会继续减少数量。"
    output = Path(config.get('output_dir', '.'))
    output.mkdir(parents=True, exist_ok=True)
    path = output / ('Optim_structure_' + signature[:16] + '.json')
    result['report_file'] = str(path.resolve())
    table = path.with_suffix('.csv')
    result['survivor_table_file'] = str(table.resolve())
    with table.open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['模式ID', '模块1_y_z_alpha_beta', '模块2_y_z_alpha_beta',
                         '模块3_y_z_alpha_beta', '带编号模式数', '状态'])
        for row in survivors:
            writer.writerow([row['id'], *(''.join(map(str, b)) for b in row['modules']),
                             row['labelled_multiplicity'], '尚未排除；不是可行性证明'])
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(path)
    return result
