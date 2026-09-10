"""Proof validity, domain preservation, and lossless unresolved-work dispatch."""
import copy
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from strict_layout_topology import (CASES,add_sector,build_coefficients,evaluate_candidate,
    group_library,linear_conflicts,mask_rows,prepare_state,save_checkpoint,solve_decomposed,
    solve_geometry,span_weights,split_unresolved,structural_obstruction,verify_conflict)
from strict_layout_minlp import build_model,zero_free_patterns


def config(**updates):
    c=dict(lower=[0,-.6,0,0]*3,upper=[.6,.6,2*math.pi,math.pi/2]*3,x_face=2,
        min_install_distance=0,min_install_angle_deg=0,min_axis_capacity=1e-5,
        pure_tolerance=1e-9,max_group_size=6,max_time=1,search_seeds=[])
    c.update(updates);return c


BASELINE=[.2,0,0,math.pi/4,.6,-.2,math.pi/2,math.pi/4,.6,.2,3*math.pi/2,math.pi/4]


class TopologyTests(unittest.TestCase):
    def test_historical_candidate_yields_exact_two_group_conflict(self):
        c=config(); check=evaluate_candidate(BASELINE,c)
        self.assertEqual(check['coverage'],152)
        proofs=linear_conflicts(check['masks'],c)
        self.assertTrue(proofs)
        self.assertTrue(all(verify_conflict(p,c) for p in proofs))
        proof=next(p for p in proofs if p['target_case']==3)
        self.assertEqual(proof['weights'],['2'])
        self.assertEqual(proof['capacity_upper'],'1/500000000')
        self.assertEqual(len(proof['assignments']),2)
        # A rank argument alone would be unsafe once epsilon permits the target.
        self.assertFalse(verify_conflict(proof,config(min_axis_capacity=2e-9)))

    def test_corrupted_or_unhealthy_proofs_are_never_loaded(self):
        c=config(); proof=linear_conflicts(evaluate_candidate(BASELINE,c)['masks'],c)[0]
        bad=copy.deepcopy(proof);bad['weights']=['3']
        self.assertFalse(verify_conflict(bad,c))
        state,_=prepare_state(c);state['conflicts']=[bad]
        with self.assertRaisesRegex(ValueError,'rational verification'):
            prepare_state(dict(c,search_state=state))
        bad=copy.deepcopy(proof);bad['assignments'][-1][1]|=1
        self.assertFalse(verify_conflict(bad,c))

    def test_matlab_singleton_shape_roundtrip(self):
        c=config(); proof=linear_conflicts(evaluate_candidate(BASELINE,c)['masks'],c)[0]
        state,_=prepare_state(c);state['conflicts']=copy.deepcopy(proof)
        state['conflicts']['basis_cases']=state['conflicts']['basis_cases'][0]
        state['conflicts']['weights']=state['conflicts']['weights'][0]
        restored,note=prepare_state(dict(c,search_state=state))
        self.assertEqual(note,'resumed_topologies')
        self.assertTrue(verify_conflict(restored['conflicts'][0],c))

    def test_exact_rational_elimination_including_rank_zero(self):
        self.assertEqual(span_weights([], [0,0,0]),[])
        self.assertIsNone(span_weights([], [1,0,0]))
        weights=span_weights([[2,0,1],[0,3,0]], [1,1,1/2])
        self.assertEqual([str(w) for w in weights],['1/2','1/3'])
        self.assertIsNone(span_weights([[2,0,1]], [1,0,0]))

    def test_all_216_zero_z_patterns_have_a_direct_obstruction(self):
        c=config();_,roots=zero_free_patterns(c['lower'],c['upper'])
        rejected=sum(structural_obstruction(dict(c,lower=r['lower'],upper=r['upper'])) is not None for r in roots)
        self.assertEqual(rejected,216)
        self.assertIsNone(structural_obstruction(c))

    def test_algebraic_model_preserves_narrow_and_wide_angle_arcs(self):
        # Check only the arc constraints here, including a wrap through 2*pi.
        from pyscipopt import Model
        for low,high,angle,expected in [(0,0,math.pi,False),(0,0,0,True),
            (.3,.7,.5,True),(.3,.7,2,False),(.2,5.9,3,True),(.2,5.9,0,False),
            (5.8,6.6,6.4,True),(5.8,6.6,3,False)]:
            m=Model();m.hideOutput()
            p=m.addVar(lb=math.cos(angle),ub=math.cos(angle));q=m.addVar(lb=math.sin(angle),ub=math.sin(angle))
            add_sector(m,p,q,low,high,1,'arc');m.optimize()
            self.assertEqual(m.getNSols()>0,expected,(low,high,angle))
        # No regression to trigonometric products when an alpha box is cut.
        c=config(lower=[0,-.6,.3,0]*3,upper=[.6,.6,.7,math.pi/2]*3)
        model,data,*_=build_model(c)
        self.assertIsInstance(data,dict)
        self.assertEqual(sum(v.getConshdlrName()=='nonlinear' for v in model.getConss()),12)

    def test_linear_master_contains_no_nonlinear_constraints(self):
        model,data,groups,_=build_coefficients(config(),relaxed=True)
        self.assertEqual(len(groups),36)
        self.assertFalse(any(v.getConshdlrName()=='nonlinear' for v in model.getConss()))
        for j,g in enumerate(groups): self.assertEqual(g[CASES[j][0]].getUbGlobal(),0)

    def test_geometry_keeps_labelled_topology_and_all_unresolved_boxes(self):
        c=config(); masks=evaluate_candidate(BASELINE,c)['masks']
        model,data,_,_=build_coefficients(c,fixed_masks=masks)
        self.assertEqual(model.getNBinVars(),0)
        work=dict(masks=masks,regions=[dict(lower=c['lower'][:],upper=c['upper'][:],depth=0)])
        volume=np.prod(np.array(c['upper'])-c['lower'])
        for _ in range(5): split_unresolved(work,c)
        self.assertAlmostEqual(sum(np.prod(np.array(n['upper'])-n['lower']) for n in work['regions']),volume)

    def test_timeout_and_numeric_infeasibility_both_preserve_topology(self):
        for status in ('timelimit','infeasible'):
            c=config(max_time=1);state,_=prepare_state(c)
            state['seeded']=True;state['next_stage']='geometry'
            state['pending']=[dict(id=0,masks=evaluate_candidate(BASELINE,c)['masks'],trials=0,last_status='unresolved',
                regions=[dict(lower=c['lower'],upper=c['upper'],depth=0)])]
            clock=[0]
            def slave(cfg,work):
                clock[0]+=2;return dict(status=status,has_solution=False,nodes=1)
            with patch('strict_layout_topology.time.monotonic',side_effect=lambda:clock[0]),patch('strict_layout_topology.solve_geometry',side_effect=slave):
                r=solve_decomposed(dict(c,search_state=state))
            self.assertFalse(r['has_solution']);self.assertEqual(r['status'],'timelimit')
            self.assertEqual(len(r['search_state']['pending']),1)
            self.assertEqual(len(r['search_state']['pending'][0]['regions']),2)
            self.assertEqual(r['search_state']['pending'][0]['last_status'],status)

    def test_changed_constraints_and_legacy_queue_reset_explicitly(self):
        c=config();state,_=prepare_state(c)
        restored,note=prepare_state(dict(c,search_state=state,min_axis_capacity=2e-5))
        self.assertEqual(note,'method_or_constraints_changed_queue_reset')
        self.assertFalse(restored['conflicts'])
        _,note=prepare_state(dict(c,search_state={'version':2,'pending':[]}))
        self.assertEqual(note,'method_or_constraints_changed_queue_reset')

    def test_checkpoint_contains_proofs_and_unresolved_work(self):
        c=config();state,_=prepare_state(c)
        state['conflicts']=linear_conflicts(evaluate_candidate(BASELINE,c)['masks'],c)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'state.json'
            save_checkpoint(dict(c,checkpoint_path=str(path)),state,'topology_master')
            restored,_=prepare_state(dict(c,search_state=json.loads(path.read_text())['search_state']))
            self.assertEqual(restored['conflicts'],state['conflicts'])

    def test_zero_capacity_backend_transport_fixture(self):
        # Explicitly NOT a valid design: MATLAB refuses this zero threshold.
        c=config(lower=[0,0,0,0]*3,upper=[0,0,0,0]*3,min_axis_capacity=0,max_time=3)
        masks=[6 if fault==0 else 5 for fault,label in CASES]
        work=dict(masks=masks,regions=[dict(lower=c['lower'],upper=c['upper'],depth=0)])
        r=solve_geometry(c,work)
        self.assertTrue(r['has_solution'])
        self.assertEqual(len(r['x']),12)


if __name__=='__main__': unittest.main()
