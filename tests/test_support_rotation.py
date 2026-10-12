import itertools
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from strict_layout_support_search import run, inner_domain, diagonal_level, sign_variants, BASELINE, choose_job, planned_budget
from strict_layout_structure import strict_installation_domain
from strict_layout_topology import evaluate_candidate, physical_matrix


class SupportRotationTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.c=dict(lower=[0,-.6,0,0]*3,upper=[.6,.6,2*math.pi,math.pi/2]*3,
                    x_face=2,min_axis_capacity=1e-5,pure_tolerance=1e-9,max_group_size=6,
                    min_install_distance=0,min_install_angle_deg=0,max_time=300,
                    output_dir=self.folder.name,support_visit_limit=1,support_direct_seconds=0)

    def fake(self,c):
        state=c.get('search_state') or {'token':0}
        state['token']+=1
        return dict(search_state=state,status='numerically_infeasible',has_solution=False,candidates=[])

    def test_round_robin_resume_and_no_numeric_pruning(self):
        with patch('strict_layout_support_search.solve_decomposed',side_effect=self.fake):
            first=run(dict(self.c,support_visit_limit=115))
            self.assertEqual(first['support_seen'],115)
            self.assertTrue(all(e['visits']==1 for e in first['search_state']['entries']))
            second=run(self.c)
        self.assertEqual(second['support_visits'],116)
        self.assertEqual(second['support_total'],115)
        self.assertEqual(second['resume_note'],'resumed_supports')
        self.assertFalse(second['has_solution'])

    def test_active_subjob_checkpoint_recovered_after_interruption(self):
        def interrupt(c):
            Path(c['checkpoint_path']).write_text(json.dumps({'search_state':{'token':41}}))
            raise RuntimeError('interrupted')
        with patch('strict_layout_support_search.solve_decomposed',side_effect=interrupt):
            with self.assertRaises(RuntimeError): run(self.c)
        with patch('strict_layout_support_search.solve_decomposed',side_effect=self.fake):
            r=run(self.c)
        self.assertEqual(r['search_state']['entries'][0]['jobs']['0:0']['state']['token'],42)

    def test_inner_boxes_preserve_zero_nonzero_and_expand(self):
        support=[int(v!=0) for v in BASELINE]
        signs=sign_variants(self.c,support)[0]
        a=inner_domain(self.c,support,0,signs);b=inner_domain(self.c,support,10,signs)
        for j,s in enumerate(support):
            if not s: self.assertEqual(a['lower'][j],a['upper'][j]);self.assertEqual(a['lower'][j],0)
            else: self.assertTrue(a['lower'][j]>0 or a['upper'][j]<0)
            self.assertLessEqual(b['lower'][j],a['lower'][j]);self.assertGreaterEqual(b['upper'][j],a['upper'][j])
        self.assertEqual([diagonal_level(i) for i in range(10)],[0,0,1,0,1,2,0,1,2,3])

    def test_historical_layout_is_surviving_but_not_feasible(self):
        r=evaluate_candidate(BASELINE,self.c)
        self.assertEqual(r['coverage'],152);self.assertFalse(r['has_solution'])
        self.assertTrue(strict_installation_domain(BASELINE,self.c))
        _,points=physical_matrix(BASELINE,2)
        self.assertAlmostEqual(min(math.dist(p,q) for p,q in itertools.combinations(points,2)),.4)
        with patch('strict_layout_support_search.solve_decomposed',side_effect=self.fake):
            out=run(self.c)
        self.assertEqual(out['search_state']['entries'][0]['id'],'100111111111')

    def test_fake_solver_success_cannot_bypass_full_verification(self):
        def false_success(c):
            return dict(search_state={'token':1},status='candidate',has_solution=True,x=BASELINE,candidates=[])
        with patch('strict_layout_support_search.solve_decomposed',side_effect=false_success):
            r=run(self.c)
        self.assertFalse(r['has_solution']);self.assertEqual(r['best_coverage'],152)

    def test_existing_jobs_deepen_and_budget_grows(self):
        job={'level':0,'sign_index':0,'adaptive_visits':0}
        e={'jobs':{'0:0':job},'adaptive_visits':0}
        self.assertEqual(choose_job(e,[{},{}]),('0:0',0,0))
        self.assertEqual(planned_budget({},job),120)
        job['adaptive_visits']=1;self.assertEqual(planned_budget({},job),240)
        job['adaptive_visits']=2;self.assertEqual(planned_budget({},job),300)
        e['adaptive_visits']=3
        self.assertEqual(choose_job(e,[{},{}]),('0:1',0,1))

    def test_v1_upgrade_backs_up_and_preserves_480_visits(self):
        with patch('strict_layout_support_search.solve_decomposed',side_effect=self.fake):
            r=run(self.c)
        path=Path(r['checkpoint_file']);saved=json.loads(path.read_text());s=saved['search_state']
        s.pop('policy_version');s.pop('metrics');s['visits']=480
        for i,e in enumerate(s['entries']):
            e['visits']=5 if i<20 else 4;e.pop('adaptive_visits',None)
        original=json.dumps(saved);path.write_text(original)
        with patch('strict_layout_support_search.solve_decomposed',side_effect=self.fake):
            out=run(self.c)
        self.assertEqual(out['support_visits'],481)
        self.assertEqual(out['search_metrics']['legacy_visits'],480)
        self.assertEqual(path.with_suffix('.pre_adaptive_v1.json').read_text(),original)

    def test_direct_geometry_runs_even_when_master_never_finds_groups(self):
        with patch('strict_layout_support_search.solve_decomposed',side_effect=self.fake):
            r=run(dict(self.c,support_direct_seconds=1))
        self.assertEqual(r['search_metrics']['direct_calls'],1)
        self.assertGreater(r['search_metrics']['direct_evaluations'],0)
        self.assertGreater(r['search_metrics']['candidate_checks'],0)
        self.assertFalse(r['has_solution'])


if __name__=='__main__': unittest.main()
