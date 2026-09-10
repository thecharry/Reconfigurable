"""Exact rational polynomial encoding of the DEFAULT installation domain.

Requires z3-solver. Exports both ideal-zero and MATLAB-tolerance formulas.
This is a diagnostic experiment, not an integrated production optimizer.
No spacing/angle exclusions are imposed, matching the current zero defaults.
"""
import argparse
import json
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SIGNS=((-1,-1,1,1),(1,-1,1,-1),(1,-1,1,-1),
       (1,1,1,1),(-1,1,1,-1),(1,-1,-1,1))


def export_formula(tolerance=False):
    lines=['(set-logic QF_NRA)']
    def assertion(value):
        lines.append(f'(assert {value})')
    names=('y','z','c','p','q','h','g','k')
    bounds=(('0','0.6'),('(- 0.6)','0.6'),('0','1'),('(- 1)','1'),
            ('(- 1)','1'),('(- 0.6)','0.6'),('(- 1.3)','1.3'),('(- 1.3)','1.3'))
    for m in range(3):
        for name,(lo,hi) in zip(names,bounds):
            v=f'{name}_{m}'; lines.append(f'(declare-fun {v} () Real)')
            assertion(f'(and (<= {lo} {v}) (<= {v} {hi}))')
        y,z,c,p,q,h,g,k=[f'{name}_{m}' for name in names]
        assertion(f'(= (+ (* {c} {c}) (* {p} {p}) (* {q} {q})) 1)')
        assertion(f'(= (* 2 {h}) (- (* {y} {q}) (* {z} {p})))')
        assertion(f'(= (* 2 (- {g} {q})) (* {z} {c}))')
        assertion(f'(= (* 2 (- {k} {p})) (* {y} {c}))')
    assertion('(<= y_0 y_1)'); assertion('(<= y_1 y_2)')
    for fault in (0,4,8):
        for label in range(12):
            ids=[i for i in range(12) if i!=fault]
            bs={i:f'b_{fault}_{label}_{i}' for i in ids}
            for b in bs.values():
                lines.append(f'(declare-fun {b} () Bool)')
            count='(+ '+' '.join(f'(ite {b} 1.0 0.0)' for b in bs.values())+')'
            assertion('(or '+' '.join(f'(= {count} {n})' for n in (2,4,6))+')')
            for row in range(6):
                terms=[]
                for i in ids:
                    v=f'{names[row+2]}_{i//4}'
                    value=v if SIGNS[row][i%4]>0 else f'(- {v})'
                    terms.append(f'(ite {bs[i]} {value} 0.0)')
                wrench='(+ '+' '.join(terms)+')'
                if row==label//2:
                    signed=wrench if label%2 else f'(- {wrench})'
                    assertion(f'(>= {signed} '+('0.000009999' if tolerance else '0.00001')+')')
                elif tolerance:
                    assertion(f'(and (<= (- 0.000000001) {wrench}) (<= {wrench} 0.000000001))')
                else:
                    assertion(f'(= {wrench} 0)')
    lines.append('(check-sat)')
    return '\n'.join(lines)+'\n'


def build(tolerance=False):
    solver=z3.Tactic('qfnra-nlsat').solver()
    variables=[]
    for m in range(3):
        y,z,c,p,q,h,g,k=z3.Reals(' '.join(f'{v}_{m}' for v in ('y','z','c','p','q','h','g','k')))
        variables.append((y,z,c,p,q,h,g,k))
        solver.add(y>=0,y<=z3.Q(3,5),z>=-z3.Q(3,5),z<=z3.Q(3,5),c>=0,c<=1,
                   p>=-1,p<=1,q>=-1,q<=1,h>=-z3.Q(3,5),h<=z3.Q(3,5),
                   g>=-z3.Q(13,10),g<=z3.Q(13,10),k>=-z3.Q(13,10),k<=z3.Q(13,10))
        solver.add(c*c+p*p+q*q==1,2*h==y*q-z*p,2*(g-q)==z*c,2*(k-p)==y*c)
    # Safe only because the three modules have identical default bounds.
    solver.add(variables[0][0]<=variables[1][0],variables[1][0]<=variables[2][0])
    eps=z3.Q(1,10**9) if tolerance else z3.Q(0,1)
    rho=z3.Q(1,10**5)-eps
    groups={}
    for fault in (0,4,8):
        for label in range(12):
            group=[z3.Bool(f'b_{fault}_{label}_{i}') if i!=fault else z3.BoolVal(False) for i in range(12)]
            groups[fault,label]=group
            count=z3.Sum([z3.If(b,z3.RealVal(1),z3.RealVal(0)) for b in group])
            solver.add(z3.Or(count==2,count==4,count==6))
            for row in range(6):
                terms=[z3.If(group[i],SIGNS[row][i%4]*variables[i//4][row+2],z3.RealVal(0))
                       for i in range(12) if i!=fault]
                wrench=z3.Sum(terms)
                if row==label//2:
                    solver.add((-1 if label%2==0 else 1)*wrench>=rho)
                elif tolerance:
                    solver.add(wrench>=-eps,wrench<=eps)
                else:
                    solver.add(wrench==0)
    return solver,variables,groups


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--seconds',type=float,default=60)
    args=parser.parse_args()
    report=dict(scope='Default full domain, exact geometry, 36 representative requirements',
                continuous_variables=24,boolean_variables=396,nonlinear_physical_equalities=12)
    try:
        global z3
        import z3
        report['z3_version']=z3.get_version_string()
    except ImportError:
        z3=None
    for tolerance in (False,True):
        name='tolerant' if tolerance else 'ideal'
        path=ROOT/f'analysis/strict_layout_{name}_20260909.smt2'
        path.write_text(export_formula(tolerance))
        report[name+'_formula']=str(path)
        if tolerance or z3 is None:
            continue
        solver,variables,groups=build(tolerance)
        # Also parse the portable exported formula independently.
        parsed=z3.SolverFor('QF_NRA'); parsed.from_file(str(path))
        report[name+'_export_parsed']=True
        solver.set(timeout=round(args.seconds*1000))
        started=time.monotonic(); status=solver.check(); elapsed=time.monotonic()-started
        result=dict(status=str(status),seconds=elapsed,reason=solver.reason_unknown() if status==z3.unknown else '')
        if status==z3.sat:
            model=solver.model()
            result['variables']={str(v):str(model.eval(v,model_completion=True)) for row in variables for v in row}
            result['groups']={f'{f}_{label}':[i+1 for i,b in enumerate(group) if z3.is_true(model.eval(b))]
                              for (f,label),group in groups.items()}
            (ROOT/'analysis/strict_layout_z3_model_20260909.txt').write_text(str(model))
        report[name+'_run']=result
        print(json.dumps(result),flush=True)
    if z3 is None:
        report['ideal_run']=dict(status='not_run',reason='z3-solver not available; portable formulas exported')
    (ROOT/'analysis/layout_smt_probe_20260909.json').write_text(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
