"""Fake-ROS traffic integration; does not simulate vehicle dynamics or DDS."""
import math
import unittest

import test_map_assist as fixtures
from semantic_planner_core.rough_route import RouteReference, RouteSample


class RedApproachTests(unittest.TestCase):
    def setUp(self):
        self.h = fixtures.MapAssistTests(methodName="runTest")
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.addCleanup(lambda: self.h.node.close_fusion_worker())
        self.h.traffic_node()
        self.node = self.h.node
        nodes = {"00": (-20., 0.), "01": (0., 0.), "02": (20., 0.),
                 "03": (40., 0.), "04": (40., 20.), "06": (30., 10.)}
        outer = ["00", "01", "02", "03", "04", "00"]
        adjacency = {key: {} for key in nodes}
        for a, b in list(zip(outer, outer[1:])) + [("02", "06"), ("06", "03")]:
            adjacency[a][b] = adjacency[b][a] = math.dist(nodes[a], nodes[b])
        self.node.official = self.h.module.OfficialRoute(outer, [nodes[k] for k in outer])
        self.node.mapped_topology = self.h.module.TopologyGraph(nodes, adjacency, [outer])
        dense = self.h.dense_loop([nodes[k] for k in outer], spacing=0.4)[:-1]
        self.node.reference = RouteReference([RouteSample(x, y, 0.) for x, y in dense], 0.2, 0.0)
        self.node.fused_node_positions = {k: nodes[k] for k in outer[:-1]}
        self.node.fused_node_progress = {"00": 0., "01": 20., "02": 40., "03": 60., "04": 80.}
        self.node.topology_alignment = {"direction": 1}
        self.node.fusion_status = {"available": True, "reason": "synthetic_fixture"}
        self.node.progress = 24.0
        self.h.triplet()
        self.h.odom(4.0, 0.0, 0.0)
        self.assertTrue(self.node.matched)

    def activate_red(self):
        self.h.confirm_light("red")
        self.node._tick()
        self.assertEqual(self.node.blocked_edge, ["02", "03"])
        self.assertEqual(self.node.detour_node_ids, ["02", "06", "03"])
        self.assertEqual(self.node.traffic_mode, "approach")

    def test_far_red_plans_and_publishes_distinct_reference_points_then_enters_detour(self):
        self.activate_red()
        self.assertGreater(self.node.detour_entry_progress - self.node.progress, 15.0)
        points = [self.node.point_pubs[k].messages[-1] for k in "abc"]
        self.assertTrue(all(p.x > 0 and math.isfinite(p.x) and math.isfinite(p.y) for p in points))
        self.assertTrue(points[0].x < points[1].x < points[2].x)
        self.assertTrue(all(abs(p.y) < 1e-5 for p in points))
        self.node._on_controller_cmd(self.h.command())
        self.assertGreater(self.node.cmd_pub.messages[-1].linear.x, 0.0)
        for step in range(1, 40):
            self.h.advance(0.2)
            self.h.odom(4.0 + 0.4 * step, 0.0, 0.0)
            self.node._tick()
            if self.node.traffic_mode == "detour":
                break
        self.assertEqual(self.node.traffic_mode, "detour", self.node.traffic_reason)
        self.assertEqual(self.node.detour_node_ids, ["02", "06", "03"])
        self.assertTrue(self.node.detour_motion_ready, self.node.traffic_reason)
        points = [self.node.point_pubs[k].messages[-1] for k in "abc"]
        self.assertTrue(all(math.isfinite(p.x) and math.isfinite(p.y) and p.x > 0 for p in points))
        self.assertTrue(all(math.dist((a.x, a.y), (b.x, b.y)) > 1e-6 for a, b in zip(points, points[1:])))

        # Drive the geometry fixture through 06 and back to 03. The poses are
        # supplied by the test, not produced by a vehicle dynamics simulation.
        for step in range(33):
            t = step / 32.0
            yaw = math.pi / 4 if t < 0.92 else (0.0 if t < 0.96 else -math.pi / 4)
            self.h.advance(0.3)
            self.h.odom(20.0 + 10.0 * t, 10.0 * t, yaw)
            self.node._tick()
            self.assertTrue(self.node.detour_motion_ready, self.node.traffic_reason)
        for step in range(1, 33):
            t = step / 32.0
            self.h.advance(0.3)
            self.h.odom(30.0 + 10.0 * t, 10.0 - 10.0 * t, -math.pi / 4)
            self.node._tick()
            if self.node.traffic_mode == "normal":
                break
        self.assertEqual(self.node.traffic_mode, "normal", self.node.traffic_reason)
        self.assertEqual(self.node.traffic_reason, "detour_complete_resume_vision")

    def test_oa_silences_map_points_and_green_cancels_approach(self):
        self.activate_red()
        count = self.h.output_count()
        self.node._on_avoidance(fixtures.DataMessage(True))
        self.h.advance(0.1)
        self.h.odom(4.1, 0.0, 0.0)
        self.node._tick()
        self.assertEqual(self.h.output_count(), count)
        command = self.h.command(0.4, -0.2)
        self.node._on_controller_cmd(command)
        self.assertEqual(self.h.twist_values(self.node.cmd_pub.messages[-1]), self.h.twist_values(command))
        self.node._on_avoidance(fixtures.DataMessage(False))
        self.h.advance(0.25)
        self.h.odom(4.2, 0.0, 0.0)
        self.node._tick()
        self.assertGreater(self.h.output_count(), count)
        self.h.confirm_light("green")
        self.assertEqual(self.node.traffic_mode, "normal")

    def test_missing_fusion_stays_stopped(self):
        self.node.fusion_status = {"available": False, "reason": "unavailable"}
        self.h.confirm_light("red")
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, "wait")
        self.assertEqual(self.node.traffic_reason, "red_wait_map_fusion")
        self.h.assert_stopped()

    def test_avoidance_also_has_priority_after_entering_detour(self):
        self.activate_red()
        for step in range(1, 40):
            self.h.advance(0.2)
            self.h.odom(4.0 + 0.4 * step, 0.0, 0.0)
            self.node._tick()
            if self.node.traffic_mode == "detour":
                break
        self.assertEqual(self.node.traffic_mode, "detour")
        self.node._on_avoidance(fixtures.DataMessage(True))
        count = self.h.output_count()
        self.h.advance(0.1)
        self.h.odom(*self.node.odom)
        self.node._tick()
        self.assertEqual(self.h.output_count(), count)
        command = self.h.command(0.4, -0.2)
        self.node._on_controller_cmd(command)
        self.assertEqual(self.h.twist_values(self.node.cmd_pub.messages[-1]), self.h.twist_values(command))
        self.node._on_avoidance(fixtures.DataMessage(False))
        self.h.advance(0.25)
        self.h.odom(*self.node.odom)
        self.node._tick()
        self.assertGreater(self.h.output_count(), count)
        self.assertEqual(self.node.traffic_mode, "detour")

    def test_reverse_official_order_still_selects_forward_node(self):
        old = self.node.official
        identifiers = list(reversed(old.node_ids[:-1]))
        identifiers.append(identifiers[0])
        self.node.official = self.h.module.OfficialRoute(
            identifiers, [self.node.mapped_topology.nodes[k] for k in identifiers])
        self.node.topology_alignment["direction"] = -1
        self.activate_red()

    def test_official_graph_shortest_route_for_example(self):
        graph = self.node.topology
        outer = graph.closed_ways[0][:-1]
        ids = {identifier[-2:]: identifier for identifier in graph.nodes if identifier.startswith("1")}
        route = graph.alternative_route(ids["02"], ids["03"], True, outer, True)
        self.assertEqual([k[-2:] for k in route.node_ids], ["02", "06", "03"])


if __name__ == "__main__":
    unittest.main()
