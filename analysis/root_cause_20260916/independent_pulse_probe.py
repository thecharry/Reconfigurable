import sys,json,math,time
sys.path.insert(0,'.')
import numpy as np
from fractions import Fraction as F
from scipy.optimize import milp,Bounds,LinearConstraint
from strict_layout_support_search import BASELINE
from strict_layout_topology import physical_matrix
A,_=physical_matrix(BASELINE,2);S=np.rint(A*(10*math.sqrt(2))).astype(int)
assert np.max(abs(A-S/(10*math.sqrt(2))))<1e-14
start=time.monotonic();records=[]
for fault in range(13):
 for label in range(12):
  axis=label//2;sign=-1 if label%2==0 else 1
  # u[12], firing flags z[12], integer pair count k[1].
  rows=[];lo=[];hi=[]
  for j in range(6):
   if j==axis:continue
   rows.append(np.r_[S[j],np.zeros(13)]);lo.append(0);hi.append(0)
  for i in range(12):
   r=np.zeros(25);r[i]=1;r[12+i]=-1;rows.append(r);lo.append(-np.inf);hi.append(0)
   r=np.zeros(25);r[i]=1;r[12+i]=-.05;rows.append(r);lo.append(0);hi.append(np.inf)
  r=np.r_[np.zeros(12),np.ones(12),-2];rows.append(r);lo.append(0);hi.append(0)
  lower=np.zeros(25);upper=np.ones(25);lower[-1]=1;upper[-1]=3
  if fault:upper[fault-1]=upper[12+fault-1]=0
  obj=np.r_[-sign*S[axis],np.zeros(13)]
  sol=milp(obj,integrality=np.r_[np.zeros(12),np.ones(13)],bounds=Bounds(lower,upper),constraints=LinearConstraint(np.array(rows),lo,hi),options={'time_limit':3,'mip_rel_gap':0})
  rec={'fault':fault,'label':label,'solver_status':int(sol.status)}
  if sol.x is not None:
   u=[F(float(v)).limit_denominator(1000) for v in sol.x[:12]]
   w=[sum((F(int(S[j,i]))*u[i] for i in range(12)),F(0)) for j in range(6)]
   valid=(all(w[j]==0 for j in range(6) if j!=axis) and sign*w[axis]>F(1,1000) and sum(v>0 for v in u) in (2,4,6) and all(v==0 or F(1,20)<=v<=1 for v in u) and (not fault or u[fault-1]==0))
   rec.update(valid_exact=valid,u=[str(v) for v in u],scaled_wrench=[str(v) for v in w],capacity=float(sign*w[axis])/(10*math.sqrt(2)),fired=sum(v>0 for v in u))
  records.append(rec)
r={'seconds':time.monotonic()-start,'coverage':sum(r.get('valid_exact',False) for r in records),'minimum_capacity':min((r['capacity'] for r in records if r.get('valid_exact')),default=0),'conditions':'Independent duty cycles, each active t in [0.02,0.4] seconds, active count 2/4/6; cycle-integrated pure wrench, NOT equal-pulse instantaneous purity.','records':records}
open('analysis/root_cause_20260916/independent_pulse_246_certificate.json','w').write(json.dumps(r,indent=2));print({k:v for k,v in r.items() if k!='records'});print([r for r in records if r['fault']==1 and r['label']==3])
