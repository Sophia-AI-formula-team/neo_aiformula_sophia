"""Regression checks for the closure seam, separate from OA exclusions."""
import math
import unittest

from semantic_planner_core.geometric_alignment import _branch_tangent
from semantic_planner_core.rough_route import RouteSample
from semantic_planner_core.route_fusion import build_odom_reference, _fit_trace


def lap(excluded_near_seam=False):
    # 40m square, deliberately leave the final 1m unobserved.
    points = []
    for x in range(11):
        points.append((float(x), 0.0))
    for y in range(1, 11):
        points.append((10.0, float(y)))
    for x in range(9, -1, -1):
        points.append((float(x), 10.0))
    for y in range(9, 0, -1):
        points.append((0.0, float(y)))
    return [RouteSample(x, y, 0.0, excluded=(excluded_near_seam and i == 0))
            for i, (x, y) in enumerate(points)]


class SeamTests(unittest.TestCase):
    def test_only_pure_seam_gets_separate_gap_limit(self):
        seam = build_odom_reference(lap(), 0.2, 0.0)
        self.assertEqual(_fit_trace(seam, 0.1, 3.0)[1], None)
        oa = build_odom_reference(lap(True), 0.2, 0.0)
        self.assertEqual(_fit_trace(oa, 0.1, 3.0)[1], 'excluded_gap_too_long')

    def test_one_sided_tangent_never_uses_oa(self):
        seam = build_odom_reference(lap(), 0.2, 0.0)
        tangent, source = _branch_tangent(seam, 1.0, 2.0)
        self.assertIsNotNone(tangent)
        self.assertEqual(source, 'seam_forward')
        oa = build_odom_reference(lap(True), 0.2, 0.0)
        self.assertEqual(_branch_tangent(oa, 1.0, 2.0), (None, None))


if __name__ == '__main__':
    unittest.main()
