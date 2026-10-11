"""Closed-loop phase/direction fitting tests; no ROS or numeric dependencies."""
import math
import unittest

from semantic_planner_core.topology_alignment import fit_loop_alignment, official_fraction_from_recorded
from semantic_planner_core.traffic_route import validated_alignment


class TopologyAlignmentTests(unittest.TestCase):
    official = [(0.0, 0.0), (30.0, 0.0), (30.0, 7.0), (12.0, 7.0), (12.0, 25.0), (0.0, 25.0)]

    def make_recording(self, direction=1, scale=1.3, angle=0.47, tx=12.0, ty=-8.0):
        points = self.official
        edge, edge_fraction = 2, 0.37
        first, second = points[edge], points[(edge + 1) % len(points)]
        start = (first[0] + edge_fraction * (second[0] - first[0]),
                 first[1] + edge_fraction * (second[1] - first[1]))
        lengths = [math.dist(point, points[(index + 1) % len(points)]) for index, point in enumerate(points)]
        phase = (sum(lengths[:edge]) + edge_fraction * lengths[edge]) / sum(lengths)
        if direction == 1:
            ordered = [start] + [points[(edge + 1 + index) % len(points)] for index in range(len(points))] + [start]
        else:
            ordered = [start] + [points[(edge - index) % len(points)] for index in range(len(points))] + [start]
        c, s = math.cos(angle), math.sin(angle)
        recorded = [((c * (x - tx) + s * (y - ty)) / scale,
                     (-s * (x - tx) + c * (y - ty)) / scale) for x, y in ordered]
        return recorded, phase

    def test_unknown_start_known_similarity_both_directions(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                recorded, phase = self.make_recording(direction=direction)
                before = list(recorded)
                result = fit_loop_alignment(recorded, self.official, sample_count=128)
                self.assertTrue(result["available"], result)
                self.assertEqual(result["direction"], direction)
                self.assertAlmostEqual(result["phase_fraction"], phase, delta=1e-5)
                self.assertAlmostEqual(result["scale"], 1.3, delta=1e-5)
                self.assertAlmostEqual(result["rotation_rad"], 0.47, delta=1e-5)
                self.assertAlmostEqual(result["translation_x"], 12.0, delta=1e-4)
                self.assertAlmostEqual(result["translation_y"], -8.0, delta=1e-4)
                self.assertLess(result["rmse_m"], 1e-4)
                self.assertTrue(result["automatic_estimate"])
                self.assertTrue(result["quality_gated"])
                self.assertTrue(result["field_validation_required"])
                self.assertEqual(recorded, before)
                inverse = validated_alignment(result, 3.0, 0.5, 2.0)
                recovered_start = inverse.point(*self.official[0])
                self.assertTrue(all(math.isfinite(value) for value in recovered_start))
                for fraction in (0.0, 0.2, 1.0, 2.3, -0.4):
                    mapped = official_fraction_from_recorded(result, fraction)
                    self.assertAlmostEqual(mapped, (phase + direction * fraction) % 1.0, delta=1e-5)

    def test_circle_and_square_reject_rotational_symmetry(self):
        circle = [(12.0 * math.cos(2 * math.pi * i / 128), 12.0 * math.sin(2 * math.pi * i / 128))
                  for i in range(128)]
        square = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]
        for points in (circle, square):
            with self.subTest(vertices=len(points)):
                result = fit_loop_alignment(points, points, sample_count=128)
                self.assertFalse(result["available"])
                self.assertEqual(result["reason"], "ambiguous_correspondence")
                self.assertLess(result["alternative_rmse_m"], 1e-6)
                with self.assertRaises(ValueError):
                    official_fraction_from_recorded(result, 0.2)

    def test_adjacent_phase_samples_are_not_ambiguity_competitors(self):
        recorded, _ = self.make_recording()
        result = fit_loop_alignment(recorded, self.official, sample_count=256)
        self.assertTrue(result["available"], result)
        if result["direction"] == result["alternative_direction"]:
            separation = abs((result["phase_fraction"] - result["alternative_phase_fraction"] + 0.5) % 1 - 0.5)
            self.assertGreaterEqual(separation, 0.125 - 1e-12)
        self.assertGreater(result["ambiguity_gap_m"], 0.3)

    def test_scale_outside_configured_range_is_rejected(self):
        recorded, _ = self.make_recording(scale=3.0)
        result = fit_loop_alignment(recorded, self.official, sample_count=128)
        self.assertFalse(result["available"])
        self.assertEqual(result["reason"], "scale_out_of_range")

    def test_shape_residual_and_reflection_are_rejected(self):
        distorted = list(self.official)
        distorted[1] = (45.0, -12.0)
        mirrored = [(-x, y) for x, y in self.official]
        for recorded in (distorted, mirrored):
            with self.subTest(recorded=recorded):
                result = fit_loop_alignment(recorded, self.official, sample_count=128, max_rmse_m=0.05)
                self.assertFalse(result["available"])
                self.assertEqual(result["reason"], "rmse_exceeds_limit")

    def test_invalid_points_are_rejected(self):
        cases = [None, [], [(0, 0)] * 4, [(0, 0), (1, 0), (math.nan, 1)],
                 [(0, 0), (1, 0), (0, math.inf)], [(0,), (1, 0), (0, 1)],
                 [(1e308, 0), (-1e308, 0), (0, 1e308)]]
        for points in cases:
            with self.subTest(points=points), self.assertRaises(ValueError):
                fit_loop_alignment(points, self.official)
            with self.subTest(official=points), self.assertRaises(ValueError):
                fit_loop_alignment(self.official, points)

    def test_invalid_configuration_is_rejected(self):
        cases = [dict(sample_count=True), dict(sample_count=64.5), dict(sample_count=8),
                 dict(sample_count=2048), dict(max_rmse_m=-1), dict(max_rmse_m=math.nan),
                 dict(min_scale=0), dict(min_scale=2, max_scale=1), dict(max_scale=math.inf),
                 dict(ambiguity_margin_m=-0.1), dict(ambiguity_margin_m=True)]
        for options in cases:
            with self.subTest(options=options), self.assertRaises(ValueError):
                fit_loop_alignment(self.official, self.official, **options)

    def test_progress_mapping_rejects_malformed_alignment(self):
        base = {"available": True, "phase_fraction": 0.2, "direction": -1,
                "transform_direction": "recorded_odom_to_official_xy"}
        for change in ({"available": False}, {"direction": True}, {"direction": 0},
                       {"phase_fraction": math.nan}, {"phase_fraction": 1.0},
                       {"transform_direction": "inverse"}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                official_fraction_from_recorded({**base, **change}, 0.4)
        with self.assertRaises(ValueError):
            official_fraction_from_recorded(base, math.inf)


if __name__ == "__main__":
    unittest.main()
