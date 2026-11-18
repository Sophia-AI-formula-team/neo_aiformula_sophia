"""End-to-end matching with the actual official graph and a synthetic lap.

The generated loop is not a survey, a replay, or a prior map. This checks
software behavior only and cannot confirm field junction positions.
"""
import math
from pathlib import Path
import unittest

from fusion_test_fixtures import hypothetical_smooth_lap, official_graph_branches
from semantic_planner_core.rough_route import RouteSample
from semantic_planner_core.rough_topology import OfficialRoute
from semantic_planner_core.route_fusion import build_fused_route
from semantic_planner_core.traffic_route import TopologyGraph


class OfficialGraphFusionTests(unittest.TestCase):
    def test_full_graph_branches_at_multiple_start_poses_and_directions(self):
        path = Path(__file__).parents[2] / 'semantic_local_planner_ros/config/aiformula_topomap.osm'
        official = OfficialRoute.from_osm(path)
        graph = TopologyGraph.from_osm(path)
        branches = official_graph_branches(official, graph)
        self.assertEqual(len(branches), 5)
        comparison = [graph.nodes[key] for key in graph.nodes
                      if key not in official.node_ids and graph.adjacency[key]]
        raw = hypothetical_smooth_lap(official, graph)
        angle, scale = 0.47, 1.12
        c, s = math.cos(angle), math.sin(angle)
        measured = [(scale * (c*x - s*y) + 11.0, scale * (s*x + c*y) - 7.0) for x, y in raw]
        for reverse, shift in ((False, 0), (False, 137), (True, 253)):
            with self.subTest(reverse=reverse, shift=shift):
                points = list(reversed(measured)) if reverse else list(measured)
                points = points[shift:] + points[:shift]
                samples = [RouteSample(x, y, math.atan2(points[(i+1) % len(points)][1] - y,
                                                        points[(i+1) % len(points)][0] - x))
                           for i, (x, y) in enumerate(points)]
                result = build_fused_route(samples, official, max_rmse_m=3.2,
                                           branch_connections=branches, comparison_points=comparison)
                self.assertTrue(result.status['available'], result.status)
                self.assertEqual(result.alignment['direction'], -1 if reverse else 1)
                self.assertFalse(result.alignment['candidate_gap_enforced'])
                self.assertEqual(len(result.alignment['branch_angles']), 4)
                self.assertTrue(all(80.0 <= angle['angle_deg'] <= 95.0
                                    for angle in result.alignment['branch_angles']))
                for identifier, (x, y) in zip(official.node_ids[:-1], official.points[:-1]):
                    expected = (scale * (c*x - s*y) + 11.0, scale * (s*x + c*y) - 7.0)
                    self.assertLess(math.dist(result.node_positions[identifier], expected), 0.3)


if __name__ == '__main__':
    unittest.main()
