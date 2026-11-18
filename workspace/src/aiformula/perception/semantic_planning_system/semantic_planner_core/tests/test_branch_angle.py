"""Current-lap branch-angle screening; no external road observations."""
import math
import unittest

import numpy as np
import test_route_fusion as fixtures
from semantic_planner_core.geometric_alignment import angle_in_range, branch_angles
from semantic_planner_core.rough_route import RouteReference
from semantic_planner_core.route_fusion import build_fused_route


class BranchAngleTests(unittest.TestCase):
    def test_angle_uses_official_inner_chord_and_odom_tangent(self):
        class Ref:
            def _interval_trusted(self, a, b):
                return True
            def point_at(self, s):
                return (s, 0., True)
        candidate = {'node_progress': {'a': 10.}, 'node_positions': {'a': np.array([10., 0.])},
                     'direction': 1,
                     'scale_map_to_odom': 1., 'rotation_map_to_odom_rad': 0.,
                     'translation_map_to_odom': (0., 0.)}
        angles=branch_angles(candidate, Ref(), {'a':[('p',(10., 5.)),('q',(15., 5.))]})
        self.assertEqual([item['inner_node_id'] for item in angles], ['p', 'q'])
        self.assertAlmostEqual(angles[0]['angle_deg'], 90.)
        self.assertAlmostEqual(angles[1]['angle_deg'], 135.)

    def test_inclusive_directed_80_to_95_degree_limits(self):
        for value in (80., 90., 95.):
            self.assertTrue(angle_in_range(value, 80., 95.))
        for value in (79.9, 95.1, 100.):
            self.assertFalse(angle_in_range(value, 80., 95.))

    def test_reverse_driving_normalizes_to_official_way_direction(self):
        class Ref:
            def __init__(self, sign): self.sign=sign
            def _interval_trusted(self, a, b): return True
            def point_at(self, s): return (self.sign*s, 0., True)
        common={'node_progress': {'a':10.}, 'node_positions': {'a':np.array([10.,0.])},
                'scale_map_to_odom':1., 'rotation_map_to_odom_rad':0., 'translation_map_to_odom':(0.,0.)}
        inner=(10.-5.*math.cos(math.radians(85.)),-5.*math.sin(math.radians(85.)))
        a=branch_angles(dict(common,direction=1), Ref(1), {'a':[('p',inner)]})
        b=branch_angles(dict(common,direction=-1), Ref(-1), {'a':[('p',inner)]})
        self.assertAlmostEqual(a[0]['angle_deg'],b[0]['angle_deg'])
        self.assertAlmostEqual(a[0]['angle_deg'],85.)

    def test_100_degree_direction_is_not_folded_to_80(self):
        class Ref:
            def _interval_trusted(self,a,b):return True
            def point_at(self,s):return (s,0.,True)
        c={'node_progress':{'a':10.},'node_positions':{'a':np.array([10.,0.])},'direction':1,
           'scale_map_to_odom':1.,'rotation_map_to_odom_rad':0.,'translation_map_to_odom':(0.,0.)}
        inner=(10.-5.*math.cos(math.radians(100.)),-5.*math.sin(math.radians(100.)))
        measured=branch_angles(c,Ref(),{'a':[('p',inner)]})[0]['angle_deg']
        self.assertAlmostEqual(measured,100.)
        self.assertFalse(angle_in_range(measured,80.,95.))

    def test_exempt_only_05_to_01_and_keep_other_branches(self):
        class Ref:
            def _interval_trusted(self, a, b): return True
            def point_at(self, s): return (s, 0., True)
        n1='10000000001'; n2='10000000002'; i5='10000000005'; i6='10000000006'
        c={'node_progress':{n1:10.,n2:20.},'node_positions':{n1:np.array([10.,0.]),n2:np.array([20.,0.])},
           'direction':1,'scale_map_to_odom':1.,'rotation_map_to_odom_rad':0.,'translation_map_to_odom':(0.,0.)}
        result=branch_angles(c,Ref(),{n1:[(i5,(10.,5.)),(i6,(10.,-5.))],n2:[(i5,(20.,5.))]})
        self.assertEqual([(x['outer_node_id'],x['inner_node_id']) for x in result],[(n1,i6),(n2,i5)])

    def test_any_untrusted_interior_of_window_blocks_angle(self):
        class Ref:
            def _interval_trusted(self, a, b):
                self.assertion=(a,b)
                return False
            def point_at(self, s):
                raise AssertionError('Must not use a window containing excluded odometry')
        ref=Ref()
        c={'node_progress': {'a':10.}}
        self.assertIsNone(branch_angles(c, ref, {'a':[('p',(10.,5.))]}))
        self.assertEqual(ref.assertion,(8.,12.))

    def test_missing_official_topology_does_not_guess_nearest_interior(self):
        points=[(0,0),(30,0),(30,7),(12,7),(12,25),(0,25)]
        result=build_fused_route(fixtures.record(points),fixtures.route(points),comparison_points=[(10,10)])
        self.assertFalse(result.status['available'])
        self.assertEqual(result.status['reason'],'branch_connections_missing')
        self.assertEqual(result.node_positions,{})
        self.assertGreater(result.reference.length_m,0.)

    def test_malformed_topology_returns_odom_only(self):
        points=[(0,0),(30,0),(30,7),(12,7),(12,25),(0,25)]
        result=build_fused_route(fixtures.record(points),fixtures.route(points),
                                 branch_connections={'unknown':[('inside',(1,2))]})
        self.assertFalse(result.status['available'])
        self.assertEqual(result.status['reason'],'branch_connections_invalid')


if __name__ == '__main__':
    unittest.main()
