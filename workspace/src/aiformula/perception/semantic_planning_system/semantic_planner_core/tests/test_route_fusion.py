"""Measured scale, turn correspondences and final-curve junction regressions."""
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from semantic_planner_core.rough_route import RouteReference, RouteSample
from semantic_planner_core.rough_topology import OfficialRoute
from semantic_planner_core.route_fusion import build_fused_route, _shape_displacements


def route(points):
    values = list(points)
    ids = [str(i) for i in range(len(values))]
    return OfficialRoute(ids + [ids[0]], values + [values[0]])


def record(points, *, step=0.12, reverse=False, shift=0, excluded=None):
    samples = []
    for first, second in zip(points, list(points[1:]) + [points[0]]):
        count = max(1, math.ceil(math.dist(first, second) / step))
        yaw = math.atan2(second[1] - first[1], second[0] - first[0])
        for i in range(count):
            x = first[0] + i / count * (second[0] - first[0])
            y = first[1] + i / count * (second[1] - first[1])
            samples.append(RouteSample(x, y, yaw, excluded=bool(excluded and excluded(x, y))))
    if reverse:
        samples = list(reversed(samples))
    return samples[shift:] + samples[:shift]


class RouteFusionTests(unittest.TestCase):
    outline = [(0.0, 0.0), (30.0, 0.0), (30.0, 7.0), (12.0, 7.0), (12.0, 25.0), (0.0, 25.0)]

    def assert_available(self, result):
        self.assertTrue(result.status["available"], result.status)
        self.assertEqual(result.status["mode"], "fused")
        self.assertEqual(result.status["metric_source"], "odometry")
        self.assertFalse(result.status["official_size_used"])

    def test_official_shape_changes_final_reference(self):
        samples = record(self.outline)
        original = build_fused_route(samples, route(self.outline))
        changed = list(self.outline)
        changed[1] = (31.5, -0.7)
        alternative = build_fused_route(samples, route(changed))
        self.assert_available(original)
        self.assert_available(alternative)
        difference = max(math.dist(original.reference.point_at(original.reference.length_m * i / 100)[:2],
                                   alternative.reference.point_at(alternative.reference.length_m * i / 100)[:2])
                         for i in range(100))
        self.assertGreater(difference, 0.04)
        self.assertLessEqual(alternative.status["max_node_displacement_m"], 0.75 + 1e-10)

    def test_official_uniform_scale_translation_cannot_resize_or_move_odom(self):
        measured = [(x * 1.2 + 21.0, y * 1.2 - 37.0) for x, y in self.outline]
        samples = record(measured)
        expected = build_fused_route(samples, route(self.outline))
        self.assert_available(expected)
        for scale, tx, ty in ((8.0, -421.0, 375.0), (0.08, 25.0, -19.0)):
            transformed = [(scale * x + tx, scale * y + ty) for x, y in self.outline]
            actual = build_fused_route(samples, route(transformed))
            self.assert_available(actual)
            self.assertEqual(len(actual.reference.points), len(expected.reference.points))
            self.assertLess(max(math.dist(a, b) for a, b in zip(expected.reference.points, actual.reference.points)), 1e-6)
            self.assertAlmostEqual(actual.reference.length_m, expected.reference.length_m, places=6)
            for identifier in expected.node_progress:
                self.assertAlmostEqual(actual.node_progress[identifier], expected.node_progress[identifier], places=5)
        odom = RouteReference(samples, smooth_window_m=0.0)
        self.assertAlmostEqual(expected.reference.length_m / odom.length_m, 1.0, delta=0.01)

    def test_individual_segment_length_mismatch_uses_measured_corners(self):
        changed = [(0.0, 0.0), (31.8, 0.0), (31.8, 8.5), (12.8, 8.5), (12.8, 25.0), (0.0, 25.0)]
        samples = record(self.outline, shift=55)
        result = build_fused_route(samples, route(changed))
        self.assert_available(result)
        # The official 8.5 m side must map to the measured 7 m corner spacing.
        for identifier, actual in result.node_positions.items():
            self.assertLess(math.dist(actual, self.outline[int(identifier)]), 0.35)
        measured_side = math.dist(result.node_positions["1"], result.node_positions["2"])
        self.assertAlmostEqual(measured_side, 7.0, delta=0.3)

    def test_unknown_start_rotation_and_reverse_direction(self):
        angle, c, s = 0.61, math.cos(0.61), math.sin(0.61)
        measured = [(c * x - s * y + 9, s * x + c * y - 5) for x, y in self.outline]
        for reverse in (False, True):
            result = build_fused_route(record(measured, reverse=reverse, shift=123), route(self.outline))
            self.assert_available(result)
            self.assertEqual(result.alignment["direction"], -1 if reverse else 1)
            self.assertAlmostEqual(result.alignment["rotation_rad"], -angle, delta=0.01)
            for identifier, actual in result.node_positions.items():
                self.assertLess(math.dist(actual, measured[int(identifier)]), 0.3)

    def test_symmetric_or_bad_shapes_do_not_produce_navigation_map(self):
        square = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]
        result = build_fused_route(record(square), route(square))
        self.assertFalse(result.status["available"])
        self.assertEqual(result.status["reason"], "ambiguous_correspondence")
        self.assertEqual(result.status["mode"], "odom_only")
        self.assertEqual(result.node_positions, {})
        self.assertEqual(result.node_progress, {})
        distorted = list(self.outline)
        distorted[1] = (45.0, -12.0)
        bad = build_fused_route(record(distorted), route(self.outline), max_rmse_m=0.1)
        self.assertFalse(bad.status["available"])
        self.assertEqual(bad.status["reason"], "rmse_exceeds_limit")

    def test_short_excluded_straight_section_never_becomes_trusted(self):
        samples = record(self.outline, excluded=lambda x, y: 14.0 < x < 14.5 and y < 0.1)
        result = build_fused_route(samples, route(self.outline))
        self.assert_available(result)
        self.assertTrue(any(not value for value in result.reference.trusted))
        matched = result.reference.match(14.25, 0.0, 14.25, 0.0, 1.0, 1.0)
        self.assertIsNotNone(matched)
        self.assertFalse(result.reference.point_at(matched)[2])
        for progress in result.node_progress.values():
            self.assertTrue(result.reference.point_at(progress)[2])

    def test_excluded_turn_cannot_be_used_as_anchor(self):
        samples = record(self.outline, excluded=lambda x, y: x > 29.8 and y < 0.4)
        result = build_fused_route(samples, route(self.outline))
        self.assertFalse(result.status["available"])
        self.assertEqual(result.status["reason"], "node_turn_unobserved_or_excluded")
        self.assertEqual(result.node_positions, {})

    def test_long_excluded_gap_cannot_dominate_shape_fit(self):
        samples = record(self.outline, excluded=lambda x, y: 10.0 < x < 18.0 and y < 0.1)
        result = build_fused_route(samples, route(self.outline))
        self.assertFalse(result.status["available"])
        self.assertEqual(result.status["reason"], "excluded_gap_too_long")
        self.assertIsNone(result.alignment)

    def test_node_progress_matches_final_smoothed_curve(self):
        changed = [(0.0, 0.0), (31.8, 0.0), (31.8, 8.5), (12.8, 8.5), (12.8, 25.0), (0.0, 25.0)]
        result = build_fused_route(record(self.outline, shift=17), route(changed), smooth_window_m=1.8)
        self.assert_available(result)
        for identifier, progress in result.node_progress.items():
            self.assertGreaterEqual(progress, 0.0)
            self.assertLess(progress, result.reference.length_m)
            self.assertEqual(result.node_positions[identifier], result.reference.point_at(progress)[:2])
            self.assertTrue(result.reference.point_at(progress)[2])
        # Final lengths/correspondences are not the preliminary equal-fraction
        # estimate. The unequal official side must not shift the road junction.
        official = route(changed)
        naive = ((official._cumulative_m[1] / official.chord_length_m
                  - result.alignment["phase_fraction"]) % 1.0) * result.reference.length_m
        self.assertGreater(abs(result.node_progress["1"] - naive), 0.3)
        self.assertLess(math.dist(result.node_positions["1"], self.outline[1]), 0.2)

    def test_actual_official_map_nodes_are_observable_in_both_directions(self):
        source = Path(__file__).parents[2] / "semantic_local_planner_ros" / "config" / "aiformula_topomap.osm"
        official = OfficialRoute.from_osm(source)
        for reverse in (False, True):
            result = build_fused_route(record(official.points[:-1], reverse=reverse, shift=99), official)
            self.assert_available(result)
            self.assertEqual(len(result.node_positions), 5)
            for identifier, expected in zip(official.node_ids[:-1], official.points[:-1]):
                self.assertLess(math.dist(result.node_positions[identifier], expected), 0.3)

    def test_actual_official_uniform_scale_is_not_a_quality_gate_or_route_scale(self):
        source = Path(__file__).parents[2] / "semantic_local_planner_ros" / "config" / "aiformula_topomap.osm"
        official = OfficialRoute.from_osm(source)
        samples = record(official.points[:-1], shift=135)
        expected = build_fused_route(samples, official)
        transformed = OfficialRoute(official.node_ids,
                                    [(x * 10.0 + 450.0, y * 10.0 - 275.0) for x, y in official.points])
        actual = build_fused_route(samples, transformed)
        self.assert_available(actual)
        self.assertGreater(actual.alignment["scale"], 9.0)
        self.assertLess(max(math.dist(a, b) for a, b in zip(expected.reference.points, actual.reference.points)), 1e-6)
        for identifier in expected.node_positions:
            self.assertLess(math.dist(expected.node_positions[identifier], actual.node_positions[identifier]), 1e-6)

    def test_degenerate_chord_is_rejected_and_solver_failure_falls_back(self):
        # Two distinct progress values can revisit exactly one physical point.
        # Such a loop must not cause a divide-by-zero in direction constraints.
        class RevisitingReference:
            spacing_m = 0.2

            @staticmethod
            def point_at(progress):
                return {0.0: (0.0, 0.0, True), 10.0: (0.0, 0.0, True),
                        20.0: (5.0, 8.0, True)}[progress]

        with self.assertRaisesRegex(ValueError, "degenerate_measured_node_chord"):
            _shape_displacements(RevisitingReference(), route([(0, 0), (4, 0), (5, 8)]),
                                 {"rotation_rad": 0.0}, {"0": 0.0, "1": 10.0, "2": 20.0}, 0.25, 0.75)
        with patch("semantic_planner_core.route_fusion.np.linalg.lstsq", side_effect=np.linalg.LinAlgError("failed")):
            failed = build_fused_route(record(self.outline), route(self.outline))
        self.assertFalse(failed.status["available"])
        self.assertEqual(failed.status["mode"], "odom_only")
        self.assertIn("shape_fusion_failed", failed.status["reason"])
        self.assertEqual(failed.node_positions, {})

    def test_long_final_smoothing_keeps_each_node_at_its_own_corner(self):
        result = build_fused_route(record(self.outline, shift=23), route(self.outline), smooth_window_m=3.0)
        self.assert_available(result)
        for identifier, position in result.node_positions.items():
            self.assertLess(math.dist(position, self.outline[int(identifier)]), 0.3)

    def test_invalid_parameters_are_rejected(self):
        cases = ({"shape_weight": 1}, {"shape_weight": -0.1}, {"shape_weight": True},
                 {"node_search_m": 0}, {"feature_window_m": math.nan}, {"max_displacement_m": -1},
                 {"max_rmse_m": -1}, {"ambiguity_margin_m": math.inf})
        for options in cases:
            with self.subTest(options=options), self.assertRaises(ValueError):
                build_fused_route(record(self.outline), route(self.outline), **options)


if __name__ == "__main__":
    unittest.main()
