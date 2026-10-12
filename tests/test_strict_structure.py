import itertools
import math
import tempfile
import unittest
from strict_layout_structure import screen, strict_installation_domain, h_possible


class StrictStructureTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.c = dict(lower=[0,-.6,0,0]*3, upper=[.6,.6,2*math.pi,math.pi/2]*3,
                      x_face=2, min_axis_capacity=1e-5, pure_tolerance=1e-9,
                      max_group_size=6, output_dir=self.folder.name)

    def test_exhaustive_partition_and_rule_counts(self):
        r=screen(self.c)
        self.assertEqual([(s['representative_remaining'],s['labelled_remaining']) for s in r['stages']],
                         [(816,4096),(364,1728),(364,1728),(199,999),(115,558)])
        expanded=set()
        for row in r['patterns']:
            orbit=set(itertools.permutations(row['modules']))
            self.assertEqual(len(orbit),row['labelled_multiplicity'])
            self.assertTrue(expanded.isdisjoint(orbit))
            expanded.update(orbit)
        self.assertEqual(len(expanded),4096)
        self.assertFalse(r['has_solution'])

    def test_unequal_module_bounds_disable_symmetry(self):
        self.c['upper'][0]=.5
        r=screen(self.c)
        self.assertFalse(r['symmetry_reduced'])
        self.assertEqual(r['representative_remaining'],558)

    def test_collision_and_strict_support_without_epsilon(self):
        x=[0,.1,0,0, 0,.2,0,0, 0,.3,0,0]
        self.assertTrue(strict_installation_domain(x,self.c))
        support=[int(v!=0) for v in x]
        self.assertTrue(strict_installation_domain(x,self.c,support))
        support[2]=1
        self.assertFalse(strict_installation_domain(x,self.c,support))
        x[2]=1e-30
        self.assertTrue(strict_installation_domain(x,self.c,support))
        x[5]=-.1 # Antipodal collision, even though vectors differ.
        self.assertFalse(strict_installation_domain(x,self.c))
        x[5]=.1
        self.assertFalse(strict_installation_domain(x,self.c))
        x[5]=.2; x[1]=0
        self.assertFalse(strict_installation_domain(x,self.c))

    def test_h_zero_identity_and_one_sign_for_every_allowed_group(self):
        for bits in itertools.product((0,1),repeat=4):
            if not h_possible(bits):
                y,z,alpha,beta=[v*b for v,b in zip((.3,-.2,1.2,.7),bits)]
                h=(y*math.sin(beta)*math.sin(alpha)-z*math.sin(beta)*math.cos(alpha))/2
                self.assertEqual(h,0)
        for size in (2,4,6):
            for group in itertools.combinations(range(12),size):
                n=sum(i<4 for i in group)
                self.assertGreaterEqual(n*.2,0)
                self.assertLessEqual(n*(-.2),0)

    def test_two_only_is_not_silently_screened(self):
        self.c['max_group_size']=2
        with self.assertRaises(ValueError): screen(self.c)

    def test_impossible_positive_spacing_is_proven(self):
        self.c['min_install_distance']=10
        r=screen(self.c)
        self.assertEqual(r['representative_remaining'],0)
        self.assertTrue(any('pair_distance_upper' in p['reasons'] for p in r['patterns']))


if __name__=='__main__': unittest.main()
