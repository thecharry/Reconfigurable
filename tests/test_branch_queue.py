"""Certified coverage, durable resume and unknown/UNSAT safety boundaries."""
import copy
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from test_layered_diagnosis import config as base_config,force_witness
from strict_layout_diagnosis import build_z3,diagnose,signature,verify_report
from strict_layout_branch_queue import (CASE_ORDER,Domain,branch_progress,check_branch,import_conflicts,
    prepare_queue,screen_first_case,solve_persistent,verify_queue_file)


def config(**kwargs):
    return base_config(**dict(dict(diagnostic_initial_screen=False),**kwargs))


def split_to(state,domain,target):
    prefix=''
    for bit in target:
        state['leaves'].remove(next(n for n in state['leaves'] if n['prefix']==prefix))
        for value in '01':
            child=prefix+value
            if domain.mass(child): state['leaves'].append(dict(prefix=child,status='pending',trials=0))
        prefix+=bit
    return next(n for n in state['leaves'] if n['prefix']==target)


class BranchQueueTests(unittest.TestCase):
    def test_real_unsat_assumptions_export_a_replayable_branch_formula(self):
        import z3
        c=config();solver,coefficients,groups,scale=build_z3(c,'wrench')
        result=check_branch(solver,coefficients,groups,scale,c,'01100000000',5)
        self.assertEqual(result['status'],'unsat_reported')
        self.assertTrue(result['proof'])
        replay=z3.SolverFor('QF_LRA');replay.from_string(result['formula'])
        self.assertEqual(replay.check(),z3.unsat)

    def test_full_partition_unsat_certifies_even_if_earlier_layer_was_unknown(self):
        # Deliberately impossible capacity fixture, not the user's default domain.
        with tempfile.TemporaryDirectory() as folder, \
             patch('strict_layout_diagnosis.solve_z3',return_value=dict(engine='z3',status='unknown')):
            c=config(min_axis_capacity=100,diagnostic_persistent=True,diagnostic_initial_screen=True,
                     diagnostic_artifact_dir=folder)
            result=diagnose(c)
            self.assertEqual(result['status'],'diagnostic_unsat_verified')
            self.assertTrue(result['original_problem_infeasibility_verified'])
            self.assertFalse(result['has_solution'])
            self.assertEqual(result['branch_progress']['certified_percent'],100)
            self.assertTrue(verify_report(result['diagnostic_report_file'])['valid'])

    def test_initial_bound_screen_has_exact_nonoverlapping_mass_and_is_idempotent(self):
        c=config();s,d,_,_=prepare_queue(c);screen_first_case(s,d)
        self.assertEqual(int(branch_progress(s,d)['verified_excluded_assignments']),(847-len(d.good[0]))*847**35)
        before=copy.deepcopy(s);screen_first_case(s,d);self.assertEqual(s,before)
        prepare_queue(c,s)

    def test_partition_counts_every_healthy_assignment_exactly_once(self):
        c=config();s,d,proofs,_=prepare_queue(c)
        self.assertEqual(d.total,847**36)
        self.assertEqual(d.mass('0')+d.mass('1'),d.total)
        node=split_to(s,d,'01100000000')  # negative Fx asked of two +Fx thrusters
        node.update(status='closed',certificate=d.cheap_certificate(node['prefix'],proofs))
        checked,_,_,note=prepare_queue(c,s)
        p=branch_progress(checked,d)
        self.assertEqual(note,'resumed_branch_queue')
        self.assertEqual(int(p['verified_excluded_assignments']),847**35)
        self.assertAlmostEqual(p['certified_percent'],100/847)
        bad=copy.deepcopy(s);bad['leaves'].append(dict(prefix='',status='pending',trials=0))
        with self.assertRaisesRegex(ValueError,'Overlapping'): prepare_queue(c,bad)
        bad=copy.deepcopy(s);bad['leaves'].pop()
        with self.assertRaisesRegex(ValueError,'cover'): prepare_queue(c,bad)

    def test_tampered_closed_certificate_fails_and_new_constraints_reset(self):
        c=config();s,d,_,_=prepare_queue(c)
        s['leaves'][0].update(status='closed',certificate=dict(kind='target_upper_bound',case=0))
        with self.assertRaisesRegex(ValueError,'exclusion'): prepare_queue(c,s)
        fresh,_,_,note=prepare_queue(config(min_axis_capacity=2e-5),s)
        self.assertIn('reset',note);self.assertEqual(fresh['leaves'][0]['status'],'pending')

    def test_existing_conflicts_are_rechecked_and_deduplicated(self):
        proofs,info=import_conflicts(config())
        self.assertGreater(len(proofs),0)
        from strict_layout_topology import verify_conflict
        self.assertTrue(all(verify_conflict(p,config()) for p in proofs))
        self.assertGreater(len(info['sources']),0)

    def test_unknown_splits_and_next_call_resumes_children(self):
        visited=[]
        def unknown(*args):
            visited.append(args[-2]);time.sleep(.03)
            return dict(status='unknown',reason='test timeout')
        with tempfile.TemporaryDirectory() as folder, \
             patch('strict_layout_branch_queue.import_conflicts',return_value=([],{})), \
             patch('strict_layout_branch_queue.build_z3',return_value=(DummySolver(),None,None,None)), \
             patch('strict_layout_branch_queue.check_branch',side_effect=unknown):
            c=config(diagnostic_branch_time=.01)
            first=solve_persistent(c,.45,folder)
            self.assertEqual(first['status'],'unknown')
            self.assertEqual(first['branch_progress']['certified_percent'],0)
            report=Path(folder)/'diagnosis.json';report.write_text(json.dumps(first))
            count=len(visited)
            second=solve_persistent(dict(c,resume_file=str(report)),.45,folder)
            self.assertGreater(second['branch_progress']['solver_calls'],first['branch_progress']['solver_calls'])
            self.assertNotIn('',visited[count:])
            self.assertEqual(second['resume_note'],'resumed_branch_queue')
            self.assertTrue(verify_queue_file(second['branch_queue_file'])['valid'])

    def test_unsat_report_is_reviewed_and_does_not_advance_certified_progress(self):
        with tempfile.TemporaryDirectory() as folder, \
             patch('strict_layout_branch_queue.import_conflicts',return_value=([],{})), \
             patch('strict_layout_branch_queue.build_z3',return_value=(DummySolver(),None,None,None)), \
             patch('strict_layout_branch_queue.check_branch',return_value=dict(status='unsat_reported',formula='test',proof='test')) as engine:
            first=solve_persistent(config(),2,folder)
            self.assertEqual(first['status'],'review_required')
            self.assertEqual(first['branch_progress']['certified_percent'],0)
            report=Path(folder)/'diagnosis.json';report.write_text(json.dumps(first))
            second=solve_persistent(dict(config(),resume_file=str(report)),2,folder)
            self.assertEqual(engine.call_count,1)
            self.assertEqual(second['branch_progress']['review_branches'],1)
            self.assertFalse(verify_queue_file(second['branch_queue_file'])['whole_domain_unsat'])

    def test_interrupted_active_branch_stays_pending_and_missing_queue_is_an_error(self):
        with tempfile.TemporaryDirectory() as folder, \
             patch('strict_layout_branch_queue.import_conflicts',return_value=([],{})), \
             patch('strict_layout_branch_queue.build_z3',return_value=(DummySolver(),None,None,None)), \
             patch('strict_layout_branch_queue.check_branch',side_effect=RuntimeError('interrupted')):
            with self.assertRaisesRegex(RuntimeError,'interrupted'): solve_persistent(config(),2,folder)
            queue=Path(folder)/'branch_queue.json'
            self.assertEqual(json.loads(queue.read_text())['leaves'],[dict(prefix='',status='pending',trials=0)])
            report=Path(folder)/'diagnosis.json';report.write_text(json.dumps(dict(branch_queue_file=str(queue))))
            queue.unlink()
            with self.assertRaises(FileNotFoundError): solve_persistent(dict(config(),resume_file=str(report)),2,folder)

    def test_verified_closed_leaf_is_not_sent_to_solver_again(self):
        c=config();s,d,proofs,_=prepare_queue(c);node=split_to(s,d,'01100000000')
        node.update(status='closed',certificate=d.cheap_certificate(node['prefix'],proofs))
        for n in s['leaves']:
            if n['status']=='pending': n['status']='review'
        with tempfile.TemporaryDirectory() as folder, patch('strict_layout_branch_queue.check_branch') as engine:
            report=Path(folder)/'diagnosis.json';report.write_text(json.dumps(dict(queue_state=s)))
            r=solve_persistent(dict(c,resume_file=str(report)),2,folder)
            self.assertEqual(engine.call_count,0);self.assertEqual(r['branch_progress']['closed_branches'],1)

    def test_force_witness_resume_keeps_queue_and_latest_pointer(self):
        # Exact witness replay is included in the wall budget. Leave room for
        # at least one branch check on a busy desktop (e.g. MATLAB startup).
        c=config(diagnostic_persistent=True,diagnostic_rlimit=1,max_time=10)
        with tempfile.TemporaryDirectory() as folder:
            report=Path(folder)/'diagnosis.json'
            report.write_text(json.dumps(dict(method='layered_rational_diagnosis_v1',scope_signature=signature(c),
                layers=[dict(layer='force',status='sat_verified',witness=force_witness(c))])))
            c.update(resume_file=str(report),diagnostic_artifact_dir=folder,diagnostic_latest_path=str(Path(folder)/'latest.json'))
            first=diagnose(c);second=diagnose(c)
            self.assertEqual(second['layers'][0]['verification']['checked_requirements'],78)
            self.assertGreater(second['branch_progress']['solver_calls'],first['branch_progress']['solver_calls'])
            self.assertEqual(json.loads(Path(c['diagnostic_latest_path']).read_text())['diagnostic_report_file'],str(report.resolve()))


class DummySolver:
    def set(self,**kwargs): pass


if __name__=='__main__': unittest.main()
