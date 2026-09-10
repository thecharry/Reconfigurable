"""Physical symmetry and joint-model regression checks, independent of MATLAB."""

import math
import json
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path

import numpy as np

from strict_layout_minlp import build_model, solve, solve_fair, zero_free_patterns, fixed_direction_data, solve_fixed_directions


def physical_matrix(x):
    columns = []
    for y, z, alpha, beta in np.array(x).reshape(3, 4):
        c, p, q = math.cos(beta), math.sin(beta) * math.cos(alpha), math.sin(beta) * math.sin(alpha)
        for sx, st in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            r = np.array([2 * sx, st * y, st * z])
            d = np.array([-sx * c, st * p, st * q])
            columns.append(np.r_[d, np.cross(r, d) / 2])
    return np.array(columns).T


class StrictLayoutTests(unittest.TestCase):
    def test_729_patterns_and_variable_counts(self):
        patterns,roots = zero_free_patterns([-1]*12,[1]*12)
        self.assertEqual(len(patterns),729)
        self.assertEqual(len(roots),729)
        self.assertEqual(len({tuple(p['mask']) for p in patterns}),729)
        counts = {n:sum(p['free_count']==n for p in patterns) for n in range(6,13)}
        self.assertEqual(counts,{6:64,7:192,8:240,9:160,10:60,11:12,12:1})
        for p,r in zip(patterns,roots):
            for m in range(3):
                self.assertTrue(any(p['mask'][4*m:4*m+2]))
                self.assertTrue(any(p['mask'][4*m+2:4*m+4]))
            for i,free in enumerate(p['mask']):
                self.assertEqual((r['lower'][i],r['upper'][i]),(-1,1) if free else (0,0))

    def test_pattern_bounds_exclude_only_incompatible_fixed_zeros(self):
        patterns,roots = zero_free_patterns([.1]*12,[1]*12)
        self.assertEqual(len(roots),1)
        self.assertEqual(sum(p['status']=='bounds_excluded' for p in patterns),728)

    def test_every_pattern_is_visited_before_any_refinement(self):
        config = dict(lower=[0]*12,upper=[1]*12,x_face=2,min_install_distance=0,
                      min_install_angle_deg=0,min_axis_capacity=1e-5,max_time=730,node_time=1,pattern_search=True)
        clock,visited = [0],[]
        def timeout(cfg):
            self.assertFalse(cfg['order_modules'])
            visited.append(tuple(cfg['upper']))
            clock[0] += 1
            return dict(status='timelimit',has_solution=False,nodes=0)
        with patch('strict_layout_minlp.solve',side_effect=timeout), patch('strict_layout_minlp.time.monotonic',side_effect=lambda:clock[0]):
            result = solve_fair(config)
        self.assertEqual(len(set(visited[:729])),729)
        self.assertTrue(all(p['calls']>=1 for p in result['search_state']['patterns']))
        self.assertEqual(sum(p['calls'] for p in result['search_state']['patterns']),730)
        self.assertEqual(result['search_state']['pruned'],0)

    def test_allocation_group_ceiling_is_six(self):
        masks, *_ = fixed_direction_data([(0,.4),(2,.4),(4,.4)],2)
        self.assertEqual(masks.shape,(12,1485))
        self.assertEqual(set(masks.sum(axis=0)),{2,4,6})
        config = dict(lower=[0,-.6,0,0]*3,upper=[.6,.6,2*math.pi,math.pi/2]*3,x_face=2,
                      min_install_distance=0,min_install_angle_deg=0,min_axis_capacity=1e-5,max_time=1)
        model,_,_,_,pairs = build_model(config)
        self.assertGreater(model.getNVars(),0)
        self.assertTrue(all(v.getUbGlobal()==3 for v in pairs.values()))
        with self.assertRaises(ValueError):
            build_model(dict(config,max_group_size=8))

    def test_fair_search_visits_siblings_and_resumes_without_restarting_root(self):
        config = dict(lower=[0]*12,upper=[1]*12,x_face=2,min_install_distance=0,
                      min_install_angle_deg=0,min_axis_capacity=1e-5,max_time=3,node_time=1)
        clock, visited = [0], []
        def timeout(cfg):
            visited.append((cfg['lower'],cfg['upper']))
            clock[0] += 1
            return dict(status='timelimit',has_solution=False,nodes=0)
        with patch('strict_layout_minlp.solve',side_effect=timeout), patch('strict_layout_minlp.time.monotonic',side_effect=lambda:clock[0]):
            result = solve_fair(config)
            ids = [v['id'] for v in result['search_state']['history']]
            self.assertEqual(ids,[0,1,2])
            config['search_state'] = result['search_state']
            resumed = solve_fair(config)
        self.assertEqual(resumed['resume_note'],'resumed_queue')
        self.assertEqual(resumed['search_state']['processed'],6)
        ids = [v['id'] for v in resumed['search_state']['history']]
        self.assertEqual(len(set(ids)),len(ids))
        self.assertEqual(resumed['search_state']['pruned'],0)
        volume = sum(np.prod(np.array(n['upper'])-n['lower']) for n in resumed['search_state']['pending'])
        self.assertEqual(volume,1)

    def test_changed_constraints_invalidate_saved_queue(self):
        config = dict(lower=[0]*12,upper=[1]*12,x_face=2,min_install_distance=0,
                      min_install_angle_deg=0,min_axis_capacity=1e-5,max_time=0)
        first = solve_fair(config)
        second = solve_fair(dict(config,search_state=first['search_state'],max_group_size=4))
        self.assertEqual(second['resume_note'],'constraints_changed_queue_reset')
        self.assertEqual(second['search_state']['pending'][0]['id'],0)

    def test_fixed_direction_affine_matrix_matches_cross_product(self):
        x = np.array([.2,-.3,.7,.4,.5,.1,2.2,.6,.35,-.55,5.1,.8]).reshape(3,4)
        masks, _, force, offset, coefficients = fixed_direction_data(x[:,2:],2)
        reconstructed = np.vstack((force, offset+np.einsum('agp,p->ag',coefficients,x[:,:2].reshape(-1))))
        expected = np.einsum('ai,ig->ag',physical_matrix(x.reshape(-1)),masks)
        np.testing.assert_allclose(reconstructed,expected,atol=1e-14)

    def test_old_152_direction_family_has_a_position_independent_missing_axis(self):
        config = dict(lower=[0,-.6,0,0]*3,upper=[.6,.6,2*math.pi,math.pi/2]*3,x_face=2,
                      min_install_distance=0,min_install_angle_deg=0,min_axis_capacity=1e-5)
        result = solve_fixed_directions(config,[(0,math.pi/8),(3*math.pi/2,math.pi/8),(math.pi/2,math.pi/8)],1)
        self.assertFalse(result['has_solution'])
        self.assertEqual(result['missing_case'],[1,4])

    def test_shared_coefficients_keep_only_twelve_physical_nonlinear_equalities(self):
        config = dict(lower=[0,-.6,0,0]*3,upper=[.6,.6,2*math.pi,math.pi/2]*3,x_face=2,
                      min_install_distance=0,min_install_angle_deg=0,min_axis_capacity=1e-5,max_time=1,
                      search_seeds=[[.2,0,0,.3,.2,.2,2,.4,.2,-.2,4,.5]])
        model, variables, _, _, _ = build_model(config)
        self.assertIsInstance(variables,dict)
        self.assertEqual(sum(c.getConshdlrName()=='nonlinear' for c in model.getConss()),12)

    def test_zero_target_symbolic_pruning_preserves_possible_groups(self):
        x = [.2, -.3, .7, .4, .5, .1, 2.2, .6, .35, -.55, 5.1, .8]
        masks = np.array([[(bits >> i) & 1 for i in range(12)] for bits in range(1, 4096)
                          if bin(bits).count('1') % 2 == 0]).T
        signs = np.array([[-1, -1, 1, 1], [1, -1, 1, -1], [1, -1, 1, -1],
                          [1, 1, 1, 1], [-1, 1, 1, -1], [1, -1, -1, 1]])
        for axis in range(6):
            coefficients = np.array([signs[axis] @ masks[4 * m:4 * m + 4] for m in range(3)])
            impossible = np.all(coefficients == 0, axis=0)
            wrench = np.sum(physical_matrix(x)[axis, :, None] * masks[:, impossible], axis=0)
            np.testing.assert_allclose(wrench, 0, atol=1e-14)

    def test_feasibility_objective_and_first_solution_limit(self):
        config = dict(lower=[.2, -.4, .1, .1] * 3, upper=[.5, .4, 6, 1.5] * 3,
                      x_face=2, min_install_distance=0, min_install_angle_deg=0,
                      min_axis_capacity=.1, max_time=10)
        model, _, threshold, _, _ = build_model(config)
        self.assertEqual(threshold, .1)
        self.assertEqual(model.getParam('limits/solutions'), 1)
        self.assertTrue(all(variable.getObj() == 0 for variable in model.getVars()))

    def test_incumbent_transport_with_zero_threshold_fixture(self):
        # Only a backend transport fixture: MATLAB rejects this zero threshold,
        # so this is not evidence of a valid satellite control layout.
        config = dict(lower=[0, 0, 0, 0] * 3, upper=[0, 0, 0, 0] * 3,
                      x_face=2, min_install_distance=0, min_install_angle_deg=0,
                      min_axis_capacity=0, max_time=10)
        result = solve(config)
        self.assertTrue(result['has_solution'])
        self.assertEqual(result['primal_bound'], 0)
        self.assertEqual(result['search_goal'], 'first_feasible')
        self.assertEqual(len(result['x']), 12)
        for index, group in enumerate(result['groups']):
            self.assertEqual(len(group) % 2, 0)
            self.assertNotIn((1, 5, 9)[index // 12], group)

    def test_cli_failure_preserves_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            input_path, output_path = Path(directory) / 'input.json', Path(directory) / 'output.json'
            input_path.write_text(json.dumps({'max_time': 1}), encoding='utf-8')
            backend = Path(__file__).resolve().parents[1] / 'strict_layout_minlp.py'
            subprocess.run([sys.executable, str(backend), str(input_path), str(output_path)], check=True)
            result = json.loads(output_path.read_text(encoding='utf-8'))
            self.assertEqual(result['status'], 'error')
            self.assertFalse(result['has_solution'])
            self.assertIn('KeyError', result['error']['report'])

    def test_only_physical_permutations_preserve_wrench_rows(self):
        x = [.2, -.3, .7, .4, .5, .1, 2.2, .6, .35, -.55, 5.1, .8]
        b = physical_matrix(x)
        for permutation, signs in [([1, 0, 3, 2], [1, -1, -1, 1, -1, -1]),
                                   ([2, 3, 0, 1], [-1, 1, 1, 1, -1, -1])]:
            indices = [4 * module + i for module in range(3) for i in permutation]
            np.testing.assert_allclose(b[:, indices], np.array(signs)[:, None] * b, atol=1e-14)

    def test_independent_position_flip_is_not_an_equivalence(self):
        original = physical_matrix([.4, .4, math.pi / 4, .5] * 3)
        flipped = physical_matrix([-.4, .4, math.pi / 4, .5] * 3)
        self.assertLess(np.max(np.abs(original[3])), 1e-14)
        self.assertGreater(np.min(np.abs(flipped[3])), .1)

    def test_impossible_fixed_direction_gets_infeasibility_status(self):
        config = dict(lower=[.2, -.4, 0, 0] * 3, upper=[.5, .4, 2 * math.pi, 0] * 3,
                      x_face=2, min_install_distance=0, min_install_angle_deg=0,
                      min_axis_capacity=.1, max_time=10)
        result = solve(config)
        self.assertEqual(result['status'], 'infeasible')
        self.assertFalse(result['has_solution'])
        self.assertEqual(result['x'], [])

    def test_tiny_time_limit_is_not_infeasibility(self):
        config = dict(lower=[.2, -.4, .1, .1] * 3, upper=[.5, .4, 6, 1.5] * 3,
                      x_face=2, min_install_distance=0, min_install_angle_deg=0,
                      min_axis_capacity=.1, max_time=.00001)
        result = solve(config)
        self.assertIn(result['status'], ('timelimit', 'unknown'))
        self.assertFalse(result['has_solution'])
        self.assertIsNone(result['primal_bound'])


if __name__ == '__main__':
    unittest.main()
