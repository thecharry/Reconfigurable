"""Exact witnesses, conservative relaxation bounds and honest result semantics."""
import copy
import hashlib
import itertools
import json
import math
import tempfile
import unittest
from fractions import Fraction as F
from pathlib import Path
from unittest.mock import patch

from strict_layout_diagnosis import (SIGNS,build_z3,cases,coefficient_bounds,diagnose,effects,
    geometry_checkpoint,healthy,pure,scope,signature,solve_scip,solve_z3,
    verify_report,verify_witness,verify_zero_certificate,zero_axis_certificate)


def config(**updates):
    c=dict(lower=[0,-.6,0,0]*3,upper=[.6,.6,2*math.pi,math.pi/2]*3,x_face=2,
           min_install_distance=0,min_install_angle_deg=0,min_axis_capacity=1e-5,
           pure_tolerance=1e-9,max_group_size=6,max_time=2,diagnostic_stage_time=1,diagnostic_compare=False,
           diagnostic_persistent=False)
    c.update(updates);return c


def force_witness(c=None):
    c=c or config();coeff=[[F(1,2),F(1,2),F(s,2)] for s in (1,-1,-1)]
    library=[sum(1<<i for i in g) for n in (2,4,6) for g in itertools.combinations(range(12),n)]
    values={m:effects(coeff,m) for m in library}
    eps=F(str(c['pure_tolerance']));required=F(str(c['min_axis_capacity']))-eps
    masks=[next(m for m in library if healthy(m,f,6) and pure(values[m],k,eps,required)) for f,k in cases('force')]
    return dict(layer='force',scope_signature=signature(c),coefficients=[[str(v) for v in row] for row in coeff],masks=masks)


class LayeredDiagnosisTests(unittest.TestCase):
    def test_rational_scope_roundtrip_and_signature(self):
        c=config();self.assertEqual(signature(c),signature(scope(c)))
        self.assertEqual(signature(c),signature(dict(c,max_time=500)))
        self.assertNotEqual(signature(c),signature(config(pure_tolerance=2e-9)))
        with self.assertRaises(ValueError): scope(config(min_axis_capacity=1e-10))

    def test_relaxation_bounds_contain_physical_coefficients(self):
        # Also cover beta outside the algebraic hemisphere and unequal boxes.
        c=config(lower=[-.4,-.6,-2,-1]*3,upper=[.6,.8,7,3]*3)
        bounds=coefficient_bounds(c,'wrench')
        for y,z,alpha,beta in itertools.product((-.4,.2,.6),(-.6,.1,.8),(-2,.2,7),(-1,.3,3)):
            cv=math.cos(beta);p=math.sin(beta)*math.cos(alpha);q=math.sin(beta)*math.sin(alpha)
            vals=(cv,p,q,(y*q-z*p)/2,q+z*cv/2,p+y*cv/2)
            for v,(lo,hi) in zip(vals,bounds[0]): self.assertTrue(float(lo)<=v<=float(hi))
        self.assertEqual(len(coefficient_bounds(c,'force')[0]),3)

    def test_coefficient_signs_match_cross_product_model(self):
        from strict_layout_topology import SIGNS as production_signs
        self.assertEqual(tuple(map(tuple,production_signs.tolist())),SIGNS)

    def test_force_witness_checks_all_78_requirements_without_becoming_a_layout(self):
        c=config(); w=force_witness(c); result=verify_witness(w,c,'force')
        self.assertTrue(result['valid']);self.assertEqual(result['checked_requirements'],78)
        self.assertFalse(result['physical_layout_verified'])
        self.assertNotEqual(sum(F(v)**2 for v in w['coefficients'][0]),1)

    def test_tampered_groups_and_coefficients_are_rejected(self):
        c=config();w=force_witness(c)
        bad=copy.deepcopy(w);bad['masks'][0]|=1
        self.assertFalse(verify_witness(bad,c,'force')['valid'])
        bad=copy.deepcopy(w);bad['coefficients'][0][0]='3'
        self.assertFalse(verify_witness(bad,c,'force')['valid'])
        self.assertFalse(verify_witness(w,config(pure_tolerance=2e-9),'force')['valid'])

    def test_exact_zero_obstruction_is_not_inferred_from_nearly_zero_angles(self):
        c=config(lower=[0,-.6,0,0]*3,upper=[.6,.6,0,math.pi/2]*3)
        proof=zero_axis_certificate(c,'force')
        self.assertTrue(verify_zero_certificate(proof,c,'force'))
        changed=copy.deepcopy(proof);changed['row']=0
        self.assertFalse(verify_zero_certificate(changed,c,'force'))
        c['upper'][2]=1e-15
        self.assertIsNone(zero_axis_certificate(c,'force'))

    def test_z3_exact_fixed_group_sat_and_portable_formula(self):
        import z3
        c=config();w=force_witness(c)
        with tempfile.TemporaryDirectory() as folder:
            r=solve_z3(c,'force',5,folder,w['masks'])
            self.assertEqual(r['status'],'sat_verified')
            self.assertTrue(verify_witness(r['witness'],c,'force')['valid'])
            formula=Path(r['formula_file']).read_text()
            self.assertTrue(formula.startswith('(set-logic QF_LRA)'))
            self.assertEqual(hashlib.sha256(formula.encode()).hexdigest(),r['formula_sha256'])
            solver=z3.SolverFor('QF_LRA');solver.from_string(formula)
            self.assertEqual(solver.check(),z3.sat)

    def test_z3_unsat_proof_is_saved_but_not_mislabelled_independently_verified(self):
        import z3
        c=config();masks=force_witness(c)['masks'];masks[1]=masks[0]
        with tempfile.TemporaryDirectory() as folder:
            r=solve_z3(c,'force',5,folder,masks)
            self.assertEqual(r['status'],'unsat_reported')
            self.assertFalse(r['independent_unsat_proof_verified'])
            self.assertTrue(Path(r['proof_file']).is_file())
            solver=z3.SolverFor('QF_LRA');solver.from_file(r['formula_file'])
            self.assertEqual(solver.check(),z3.unsat)

    def test_unknown_and_numerical_infeasibility_never_certify_original_infeasibility(self):
        with patch('strict_layout_diagnosis.solve_z3',return_value=dict(engine='z3',status='unknown')), \
             patch('strict_layout_diagnosis.solve_scip',return_value=dict(engine='scip',status='numerically_infeasible')):
            r=diagnose(config(diagnostic_compare=True))
        self.assertEqual(len(r['layers']),2);self.assertEqual(r['status'],'diagnostic_unknown')
        self.assertFalse(r['has_solution']);self.assertFalse(r['original_problem_infeasibility_verified'])

    def test_real_z3_resource_limit_is_unknown(self):
        r=solve_z3(config(diagnostic_rlimit=1),'force',3)
        self.assertEqual(r['status'],'unknown')
        self.assertFalse(r['independent_unsat_proof_verified'])

    def test_verified_zero_certificate_persists_and_can_be_rechecked(self):
        with tempfile.TemporaryDirectory() as folder:
            c=config(upper=[.6,.6,0,math.pi/2]*3,diagnostic_artifact_dir=folder)
            r=diagnose(c)
            self.assertEqual(r['status'],'diagnostic_unsat_verified')
            self.assertTrue(r['original_problem_infeasibility_verified']);self.assertFalse(r['has_solution'])
            self.assertTrue(verify_report(r['diagnostic_report_file'])['valid'])

    def test_force_only_witness_cannot_enter_geometry_queue(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError,'wrench witness'):
                geometry_checkpoint(config(),force_witness(),Path(folder)/'queue.json')

    def test_resume_rechecks_force_witness_and_runs_only_unresolved_wrench_layer(self):
        c=config();w=force_witness(c)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'diagnosis.json'
            path.write_text(json.dumps(dict(method='layered_rational_diagnosis_v1',scope_signature=signature(c),
                layers=[dict(layer='force',status='sat_verified',witness=w)])))
            with patch('strict_layout_diagnosis.solve_z3',return_value=dict(engine='z3',status='unknown')) as engine:
                r=diagnose(dict(c,resume_file=str(path)))
            self.assertEqual(r['resume_note'],'resumed_verified_layers')
            self.assertEqual(r['layers'][0]['verification']['checked_requirements'],78)
            self.assertEqual(engine.call_count,1);self.assertEqual(engine.call_args.args[1],'wrench')
            self.assertFalse(r['has_solution']);self.assertEqual(r['status'],'diagnostic_unknown')

    def test_resume_rejects_a_corrupted_verified_witness(self):
        c=config();w=force_witness(c);w['masks'][0]|=1
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'diagnosis.json'
            path.write_text(json.dumps(dict(method='layered_rational_diagnosis_v1',scope_signature=signature(c),
                layers=[dict(layer='force',status='sat_verified',witness=w)])))
            with self.assertRaisesRegex(ValueError,'independent verification'):
                diagnose(dict(c,resume_file=str(path)))


if __name__=='__main__': unittest.main()
