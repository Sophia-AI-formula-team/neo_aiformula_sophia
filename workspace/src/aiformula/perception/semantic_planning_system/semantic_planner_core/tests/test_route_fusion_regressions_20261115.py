"""Regression checks for coordinate-invariant matching and excluded gaps."""

import math
import unittest
from unittest.mock import patch

from semantic_planner_core.rough_route import RouteSample
import test_route_fusion as fixtures
from test_route_fusion import build_fused_route


class FusionRegressionTests(unittest.TestCase):
    def test_equivalent_map_coordinates_keep_the_same_match(self):
        outline = fixtures.RouteFusionTests.outline
        measured = [(1.2 * x + 21.0, 1.2 * y - 37.0) for x, y in outline]
        samples = fixtures.record(measured)
        baseline = build_fused_route(samples, fixtures.route(outline))
        rebased = build_fused_route(
            samples, fixtures.route([(0.08 * x + 25.0, 0.08 * y - 19.0) for x, y in outline]))
        self.assertTrue(baseline.status["available"], baseline.status)
        self.assertTrue(rebased.status["available"], rebased.status)
        for identifier, point in baseline.node_positions.items():
            self.assertLess(math.dist(point, rebased.node_positions[identifier]), 0.01)

    @staticmethod
    def samples_with_excluded_detour():
        samples = []
        for sample in fixtures.record(fixtures.RouteFusionTests.outline, step=0.05):
            fraction = (sample.x - 13.0) / 1.9
            if abs(sample.y) < 1e-7 and 0.0 < fraction < 1.0:
                samples.append(RouteSample(sample.x, 0.35 * math.sin(math.pi * fraction),
                                           sample.yaw, excluded=True))
            else:
                samples.append(sample)
        return samples

    def assert_gap_is_straight_and_untrusted(self, reference):
        size = len(reference.points)
        first = next(index for index, (x, y) in enumerate(reference.points)
                     if 12.5 < x < 15.5 and abs(y) < 1.0 and not reference.trusted[index])
        while not reference.trusted[(first - 1) % size]:
            first = (first - 1) % size
        last = first
        while not reference.trusted[(last + 1) % size]:
            last = (last + 1) % size
        start = reference.points[(first - 1) % size]
        end = reference.points[(last + 1) % size]
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        self.assertGreater(length, 1.0)
        deviations = []
        index = first
        while True:
            self.assertFalse(reference.trusted[index])
            x, y = reference.points[index]
            deviations.append(abs(dx * (y - start[1]) - dy * (x - start[0])) / length)
            if index == last:
                break
            index = (index + 1) % size
        self.assertLess(max(deviations), 0.01)

    def test_excluded_detour_is_straight_in_fused_reference(self):
        result = build_fused_route(self.samples_with_excluded_detour(),
                                   fixtures.route(fixtures.RouteFusionTests.outline))
        self.assertTrue(result.status["available"], result.status)
        self.assert_gap_is_straight_and_untrusted(result.reference)

    def test_excluded_detour_is_straight_in_fallback_reference(self):
        with patch("semantic_planner_core.route_fusion._match_outer_nodes",
                   return_value=(None, {}, "matching_unavailable")):
            result = build_fused_route(self.samples_with_excluded_detour(),
                                       fixtures.route(fixtures.RouteFusionTests.outline))
        self.assertFalse(result.status["available"])
        self.assertEqual(result.status["reason"], "matching_unavailable")
        self.assert_gap_is_straight_and_untrusted(result.reference)


if __name__ == "__main__":
    unittest.main()
