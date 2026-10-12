"""Independent exact check of the counterfactual pulse model, not production acceptance.

Run from the repository root. No optimizer or production geometry is imported.
The baseline angles are exactly (0, pi/2, 3pi/2), beta=pi/4.
"""
import json
import math
from fractions import Fraction as F
from pathlib import Path

HERE = Path(__file__).resolve().parent
modules = [(F(1, 5), F(0), 10, 0),
           (F(3, 5), F(-1, 5), 0, 10),
           (F(3, 5), F(1, 5), 0, -10)]
# Integer S represents A=S/(10*sqrt(2)), with torque normalized by x_face=2.
columns, positions = [], []
for y, z, p, q in modules:
    for sx, st in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        r = [F(2*sx), st*y, st*z]
        d = [F(-10*sx), F(st*p), F(st*q)]
        torque = [(r[1]*d[2]-r[2]*d[1])/2,
                  (r[2]*d[0]-r[0]*d[2])/2,
                  (r[0]*d[1]-r[1]*d[0])/2]
        columns.append(d+torque)
        positions.append(r)
assert len({tuple(r) for r in positions}) == 12
certificate = json.loads((HERE/'independent_pulse_246_certificate.json').read_text())
seen, capacities = set(), []
for rec in certificate['records']:
    fault, label = rec['fault'], rec['label']
    assert (fault, label) not in seen
    seen.add((fault, label))
    u = [F(v) for v in rec['u']]
    assert len(u) == 12
    assert sum(v > 0 for v in u) in (2, 4, 6)
    assert all(v == 0 or F(1, 20) <= v <= 1 for v in u)
    assert not fault or u[fault-1] == 0
    w = [sum((columns[i][j]*u[i] for i in range(12)), F(0)) for j in range(6)]
    axis, sign = label//2, (-1 if label % 2 == 0 else 1)
    assert all(w[j] == 0 for j in range(6) if j != axis)
    # This exact rational lower bound implies normalized capacity > 1e-5.
    assert sign*w[axis] > F(1, 1000)
    assert w == [F(v) for v in rec['scaled_wrench']]
    capacities.append(float(sign*w[axis])/(10*math.sqrt(2)))
assert seen == {(f, k) for f in range(13) for k in range(12)}
result = dict(verified_demands=len(seen), arithmetic='exact rational',
              independent_geometry='r cross d, no production imports',
              unique_installation_points=len(positions),
              min_normalized_capacity=min(capacities),
              scope='Independent pulses, cycle-integrated purity only; not original equal-pulse model')
(HERE/'independent_verification.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
