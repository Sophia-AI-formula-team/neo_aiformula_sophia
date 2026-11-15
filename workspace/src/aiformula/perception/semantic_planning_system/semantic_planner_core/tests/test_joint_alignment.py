"""Joint correspondence and odometry fallback regressions."""
import math
import unittest
from pathlib import Path

import numpy as np

import test_route_fusion as fixtures
from semantic_planner_core.rough_route import RouteReference, RouteSample
from semantic_planner_core.rough_topology import OfficialRoute
from semantic_planner_core.traffic_route import TopologyGraph
from semantic_planner_core.route_fusion import (
    _candidate_disagreement, _joint_project_nodes, _project_nodes,
    _projection_data, build_fused_route, build_odom_reference,
)


class JointAlignmentTests(unittest.TestCase):
    def test_joint_assignment_can_use_a_non_nearest_ordered_candidate(self):
        reference = RouteReference(fixtures.record([(0, 0), (10, 0), (10, 10), (0, 10)]),
                                   spacing_m=0.2, smooth_window_m=0.0, max_detour_gap_m=0.0)
        points = np.asarray([(3.0, 0.0), (2.8, 0.0), (10.0, 10.0), (0.0, 10.0)])
        data = _projection_data(reference)
        nearest = _project_nodes(reference, points, 1.0, 0.0, (0.0, 0.0), data)
        joint = _joint_project_nodes(reference, points, 1.0, 0.0, (0.0, 0.0), data, 1, 4.0, 2.0)
        self.assertGreater(nearest[1][0], nearest[1][1])
        self.assertIsNotNone(joint)
        self.assertLess(joint[1][0], joint[1][1])
        self.assertGreater(max(joint[2]), 0.5)
        alternative = _joint_project_nodes(reference, points, 1.0, 0.0, (0.0, 0.0),
                                            data, 1, 4.0, 2.0, forbidden_node=0,
                                            forbidden_progress=joint[1][0], forbidden_radius_m=0.5)
        self.assertIsNotNone(alternative)
        self.assertGreater(abs(alternative[1][0] - joint[1][0]), 0.5)

    def test_inner_node_disagreement_is_not_hidden_by_equal_outer_projections(self):
        positions = {"a": np.asarray((0.0, 0.0)), "b": np.asarray((10.0, 0.0))}
        first = {"node_ids": ["a", "b"], "node_positions": positions,
                 "scale_map_to_odom": 1.0, "rotation_map_to_odom_rad": 0.0,
                 "translation_map_to_odom": (0.0, 0.0)}
        second = dict(first, rotation_map_to_odom_rad=math.pi / 2.0)
        self.assertEqual(_candidate_disagreement(first, second, []), 0.0)
        self.assertGreater(_candidate_disagreement(first, second, [(8.0, 4.0)]), 1.0)

    def test_symmetric_loop_is_not_accepted_as_unique(self):
        outline = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]
        result = build_fused_route(fixtures.record(outline, step=0.25), fixtures.route(outline),
                                   comparison_points=[(5.0, 8.0)])
        self.assertFalse(result.status["available"], result.status)
        self.assertEqual(result.status["reason"], "ambiguous_correspondence")
        self.assertEqual(result.node_positions, {})
        self.assertIsNotNone(result.alignment["alternative_node_spread_m"])

    def test_official_nodes_and_synthetic_rounded_track_remain_ambiguous(self):
        source = Path(__file__).parents[2] / "semantic_local_planner_ros" / "config" / "aiformula_topomap.osm"
        official = OfficialRoute.from_osm(source)
        graph = TopologyGraph.from_osm(source)
        inner = [point for identifier, point in graph.nodes.items()
                 if identifier not in official.node_ids and graph.adjacency[identifier]]
        points = []
        for index in range(1200):
            angle = math.pi + 2.0 * math.pi * index / 1200.0
            c, s = math.cos(angle), math.sin(angle)
            points.append((41.0 + 41.0 * math.copysign(abs(c) ** 0.4, c),
                           -0.7 + 32.5 * math.copysign(abs(s) ** 0.4, s)))
        points = points[225:] + points[:225]
        samples = [RouteSample(x, y, math.atan2(points[(i + 1) % len(points)][1] - y,
                                                 points[(i + 1) % len(points)][0] - x))
                   for i, (x, y) in enumerate(points)]
        result = build_fused_route(samples, official, max_rmse_m=3.2, comparison_points=inner)
        self.assertFalse(result.status["available"], result.status)
        self.assertEqual(result.status["reason"], "ambiguous_correspondence")
        self.assertGreater(result.alignment["alternative_node_spread_m"], 10.0)

    def test_different_start_and_reverse_still_search_expected_direction(self):
        outline = [(0, 0), (80, 0), (90, 20), (42, 28), (18, 45), (0, 25)]
        angle = 0.57
        c, s = math.cos(angle), math.sin(angle)
        observed = [(c * x - s * y + 19, s * x + c * y - 7) for x, y in outline]
        outcomes = []
        for reverse, shift in ((False, 12), (False, 184), (True, 73)):
            with self.subTest(reverse=reverse, shift=shift):
                result = build_fused_route(fixtures.record(observed, step=0.35, reverse=reverse, shift=shift),
                                           fixtures.route(outline), comparison_points=[(20, 8), (12, 12)])
                self.assertIsNotNone(result.alignment, result.status)
                self.assertEqual(result.alignment["direction"], -1 if reverse else 1)
                self.assertAlmostEqual(result.alignment["rotation_rad"], -angle, delta=0.03)
                self.assertLess(result.alignment["rmse_m"], 0.5)
                self.assertIn(result.status["reason"], ("accepted", "ambiguous_correspondence"))
                outcomes.append(result.status["reason"])
        self.assertEqual(len(set(outcomes)), 1, outcomes)

    def test_long_avoidance_gap_is_drawn_as_a_line_but_never_trusted(self):
        samples = []
        for point in fixtures.record(fixtures.RouteFusionTests.outline, step=0.1):
            u = (point.x - 10.0) / 8.0
            if abs(point.y) < 1e-9 and 0.0 < u < 1.0:
                samples.append(RouteSample(point.x, 0.7 * math.sin(math.pi * u), point.yaw, True))
            else:
                samples.append(point)
        reference = build_odom_reference(samples)
        gap = [(point, trusted) for point, trusted in zip(reference.points, reference.trusted)
               if 10.5 < point[0] < 17.5 and abs(point[1]) < 1.0]
        self.assertGreater(len(gap), 10)
        self.assertTrue(all(not trusted for _, trusted in gap))
        self.assertLess(max(abs(point[1]) for point, _ in gap), 0.02)

    def test_invalid_gap_and_comparison_parameters_are_rejected(self):
        outline = fixtures.RouteFusionTests.outline
        samples = fixtures.record(outline)
        for options in ({"max_detour_gap_m": math.nan}, {"max_detour_gap_m": -1.0},
                        {"ambiguity_position_m": 0.0}, {"comparison_points": [1.0]}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                build_fused_route(samples, fixtures.route(outline), **options)

    def test_unobserved_large_closure_gap_is_not_trusted(self):
        samples = [RouteSample(0, 0, 0), RouteSample(10, 0, 0),
                   RouteSample(10, 10, 0), RouteSample(0, 10, 0),
                   RouteSample(0, 1, 0)]
        reference = build_odom_reference(samples, spacing_m=0.2, smooth_window_m=0.0)
        self.assertTrue(reference.point_at(5.0)[2])
        self.assertFalse(reference.point_at(reference.length_m - 0.3)[2])


if __name__ == "__main__":
    unittest.main()
