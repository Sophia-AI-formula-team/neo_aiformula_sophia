"""Regression tests for discrete odometry crossing the first-lap start plane."""
import math
import unittest

import test_map_assist as fixtures


class ClosureCrossingTests(unittest.TestCase):
    def setUp(self):
        self.h = fixtures.MapAssistTests(methodName='runTest')
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.node = self.h.node
        self.addCleanup(self.node.close_fusion_worker)
        self.h.start_session()
        self.node.samples = [self.h.module.RouteSample(3.0 * math.sin(2 * math.pi * i / 120),
                                                      3.0 * (1 - math.cos(2 * math.pi * i / 120)),
                                                      2 * math.pi * i / 120)
                             for i in range(120)]
        self.node.left_start_area = True
        self.node.recorded_distance = 18.8
        self.node.p['lap_closure_radius_m'] = 0.1

    def test_segment_crossing_near_start_closes_even_if_next_sample_is_outside_radius(self):
        self.node._try_complete_lap((0.15, 0.0, 0.0), False, (-0.15, 0.0, 0.0))
        self.assertIsNotNone(self.node.reference)
        self.assertIsNotNone(self.node._fusion_future)

    def test_crossing_far_from_start_does_not_close(self):
        self.node._try_complete_lap((0.15, 0.5, 0.0), False, (-0.15, 0.5, 0.0))
        self.assertIsNone(self.node.reference)
        self.assertIsNone(self.node._fusion_future)

    def test_reverse_crossing_does_not_close(self):
        self.node._try_complete_lap((-0.01, 0.0, 0.0), False, (0.01, 0.0, 0.0))
        self.assertIsNone(self.node.reference)

    def test_subnanometre_rounding_at_start_does_not_prevent_closure(self):
        self.node._try_complete_lap((-3.4e-15, 0.0, 0.0), False, (-0.15, 0.0, 0.0))
        self.assertIsNotNone(self.node.reference)


if __name__ == '__main__':
    unittest.main()
