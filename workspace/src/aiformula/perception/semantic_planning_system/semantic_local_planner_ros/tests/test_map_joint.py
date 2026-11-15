"""Safety and ownership checks for asynchronous joint-map integration."""
import math
import threading
import time
import unittest
from concurrent.futures import Future
from unittest.mock import patch

import test_map_assist as fixtures


class ControlledExecutor:
    def __init__(self):
        self.future = Future()
        self.future.set_running_or_notify_cancel()
        self.calls = []

    def submit(self, function, *args, **kwargs):
        self.calls.append((function, args, kwargs))
        return self.future

    def shutdown(self, wait=False):
        pass


class JointMapNodeTests(unittest.TestCase):
    def setUp(self):
        self.h = fixtures.MapAssistTests(methodName="runTest")
        self.h.setUp()
        from semantic_planner_core.route_fusion import FusionResult
        self.FusionResult = FusionResult
        self.addCleanup(self.h.doCleanups)
        self.addCleanup(lambda: self.h.node.close_fusion_worker())

    def prepare_circle_lap(self, traffic=False, controlled=True):
        if traffic:
            self.h.node.close_fusion_worker()
            self.h.parameter_overrides.update(enable_traffic_control=True, enable_velocity_gate=True)
            self.h.node = self.h.module.MapAssist()
        node = self.h.node
        self.h.start_session()
        if controlled:
            node.close_fusion_worker()
            node._fusion_executor = ControlledExecutor()
        count = 120
        samples = []
        for index in range(count):
            angle = 2.0 * math.pi * index / count
            samples.append(self.h.module.RouteSample(3.0 * math.sin(angle),
                                                     3.0 * (1.0 - math.cos(angle)), angle))
        node.samples = samples
        node.recorded_distance = sum(math.hypot(a.x - b.x, a.y - b.y)
                                     for a, b in zip(samples, samples[1:]))
        node.left_start_area = True
        node.odom = (0.0, 0.0, 0.0)
        node.odom_time = self.h.now
        node.odom_stamp = self.h.now
        node.odom_frame = "odom"
        node.child_frame = "base_link"
        node._try_complete_lap((0.0, 0.0, 0.0), False)
        self.assertIsNotNone(node.reference)
        return node

    def test_odom_reference_can_assist_while_fusion_is_pending(self):
        node = self.prepare_circle_lap()
        self.assertFalse(node.fusion_status["available"])
        self.assertEqual(node.fusion_status["reason"], "alignment_pending")
        for _ in range(5):
            self.h.conflicting_frame()
        self.assertTrue(node.matched)
        self.assertGreater(abs(node.last_correction), 0.0)
        node._publish_status()
        import json
        status = json.loads(node.status_pub.messages[-1].data)
        self.assertTrue(status["odom_reference_ready"])
        self.assertTrue(status["fusion_pending"])
        self.assertEqual(status["phase"], "odom_only")

    def test_avoidance_owns_points_while_fusion_is_pending(self):
        node = self.prepare_circle_lap()
        self.h.conflicting_frame()
        count = self.h.output_count()
        node._on_avoidance(fixtures.DataMessage(True))
        self.h.advance()
        self.h.triplet()
        self.h.odom()
        node._tick()
        self.assertEqual(self.h.output_count(), count)
        self.assertEqual(node.last_correction, 0.0)

    def test_odom_assist_continues_after_ambiguous_fusion(self):
        node = self.prepare_circle_lap()
        result = self.FusionResult(node.reference, {"reason": "ambiguous_correspondence"},
                                   {"available": False, "mode": "odom_only",
                                    "reason": "ambiguous_correspondence"}, {}, {})
        node._fusion_executor.future.set_result(result)
        node._tick()
        self.assertFalse(node.fusion_status["available"])
        self.assertEqual(node.fusion_status["reason"], "ambiguous_correspondence")
        for _ in range(5):
            self.h.conflicting_frame()
        self.assertTrue(node.matched)
        self.assertGreater(abs(node.last_correction), 0.0)

    def test_worker_exception_keeps_odom_reference(self):
        node = self.prepare_circle_lap()
        reference = node.reference
        node._fusion_executor.future.set_exception(ValueError("example failure"))
        node._tick()
        self.assertIs(node.reference, reference)
        self.assertFalse(node.fusion_status["available"])
        self.assertEqual(node.fusion_status["reason"], "alignment_worker_failed")

    def test_untrusted_odom_reference_never_corrects_visual_points(self):
        node = self.prepare_circle_lap()
        node.reference.trusted = [False] * len(node.reference.trusted)
        for _ in range(5):
            expected = self.h.conflicting_frame()
        self.assertTrue(node.matched)
        self.assertEqual(node.last_correction, 0.0)
        self.h.assert_last_points_equal(expected)

    def test_red_waits_without_a_confirmed_topology(self):
        node = self.prepare_circle_lap(traffic=True)
        self.h.confirm_light("red")
        node._tick()
        self.assertEqual(node.traffic_mode, "wait")
        self.assertEqual(node.traffic_reason, "red_wait_map_fusion")
        self.assertIsNone(node.detour_tracker)
        self.h.assert_stopped()

    def test_result_waits_until_avoidance_releases_and_pose_is_fresh(self):
        node = self.prepare_circle_lap()
        result = self.FusionResult(node.reference,
                              {"scale": 1.0, "rotation_rad": 0.0, "translation_x": 0.0,
                               "translation_y": 0.0, "rmse_m": 0.0},
                              {"available": True, "mode": "fused", "reason": "accepted"}, {}, {})
        node._on_avoidance(fixtures.DataMessage(True))
        node._fusion_executor.future.set_result(result)
        with patch.object(node, "_prepare_mapped_topology") as prepare:
            node._tick()
            self.assertFalse(node.fusion_status["available"])
            self.assertIs(node._pending_fusion, result)
            prepare.assert_not_called()
            node._on_avoidance(fixtures.DataMessage(False))
            self.h.advance(node.p["resume_delay_sec"] + 0.01)
            node._tick()
            self.assertFalse(node.fusion_status["available"], "A fresh matched odom is required")
            self.h.odom()
            node._tick()
            self.assertTrue(node.fusion_status["available"])
            prepare.assert_called_once()

    def test_old_worker_result_is_discarded_after_reset(self):
        node = self.prepare_circle_lap()
        future = node._fusion_executor.future
        result = self.FusionResult(node.reference, None,
                              {"available": True, "mode": "fused", "reason": "accepted"}, {}, {})
        node._on_reset(fixtures.DataMessage(True))
        future.set_result(result)
        node._tick()
        self.assertIsNone(node.reference)
        self.assertFalse(node.fusion_status["available"])
        self.assertIsNone(node._pending_fusion)

    def test_real_worker_does_not_block_lap_callback(self):
        started, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)

        def slow_alignment(*args, **kwargs):
            started.set()
            release.wait(5.0)
            return self.FusionResult(None, None,
                                     {"available": False, "mode": "odom_only", "reason": "test_finished"}, {}, {})

        with patch.object(self.h.module, "build_fused_route", slow_alignment):
            before = time.monotonic()
            node = self.prepare_circle_lap(controlled=False)
            elapsed = time.monotonic() - before
            self.assertTrue(started.wait(1.0))
            self.assertLess(elapsed, 1.0)
            self.h.conflicting_frame()
            self.assertIsNotNone(node.reference)
            self.assertEqual(node.fusion_status["reason"], "alignment_pending")
            release.set()
            node._fusion_future.result(timeout=2.0)

    def test_real_fusion_attaches_graph_and_red_uses_internal_route(self):
        self.h.node.close_fusion_worker()
        self.h.parameter_overrides.update(enable_traffic_control=True, enable_velocity_gate=True)
        self.h.node = self.h.module.MapAssist()
        node = self.h.node
        self.h.start_session()
        outline = [(0, 0), (25, -12), (80, 0), (50, 30), (20, 40)]
        identifiers = ["n0", "n1", "n2", "n3", "n4"]
        node.official = self.h.module.OfficialRoute(identifiers + [identifiers[0]], outline + [outline[0]])
        nodes = dict(zip(identifiers, outline))
        nodes.update({"i0": (20, 8), "i1": (12, 12)})
        adjacency = {key: {} for key in nodes}

        def connect(first, second):
            distance = math.dist(nodes[first], nodes[second])
            adjacency[first][second] = distance
            adjacency[second][first] = distance

        for first, second in zip(identifiers, identifiers[1:] + identifiers[:1]):
            connect(first, second)
        for first, second in (("n0", "i0"), ("n1", "i0"), ("i0", "i1"),
                              ("i1", "n3"), ("i1", "n4")):
            connect(first, second)
        node.topology = self.h.module.TopologyGraph(nodes, adjacency, [identifiers + [identifiers[0]]])
        dense = self.h.dense_loop(outline + [outline[0]], spacing=0.3)[:-1]
        dense = dense[20:] + dense[:20]
        node.samples = [self.h.module.RouteSample(x, y, 0.0) for x, y in dense]
        node.recorded_distance = sum(math.dist(first, second)
                                     for first, second in zip(dense, dense[1:] + dense[:1]))
        node.left_start_area = True
        node.odom = (dense[0][0], dense[0][1], 0.0)
        node.odom_time = self.h.now
        node.odom_stamp = self.h.now
        node.odom_frame = "odom"
        node.child_frame = "base_link"
        node._try_complete_lap(node.odom, False)
        result = node._fusion_future.result(timeout=20.0)
        self.assertTrue(result.status["available"], result.status)
        self.h.advance()
        self.h.odom(*node.odom)
        node._tick()
        self.assertTrue(node.fusion_status["available"], node.fusion_status)
        self.assertTrue(node.topology_mapping_status["available"], node.topology_mapping_status)
        start = identifiers[0]
        position = node.fused_node_positions[start]
        next_position = node.mapped_topology.nodes["i0"]
        yaw = math.atan2(next_position[1] - position[1], next_position[0] - position[0])
        node.odom = (position[0], position[1], yaw)
        node.odom_time = self.h.now
        node.odom_stamp = self.h.now
        node.progress = node.fused_node_progress[start]
        node.matched = True
        self.h.confirm_light("red")
        node._tick()
        self.assertEqual(node.traffic_mode, "detour", node.traffic_reason)
        self.assertIn("i0", node.detour_node_ids)
        count = self.h.output_count()
        node._on_avoidance(fixtures.DataMessage(True))
        self.h.advance()
        self.h.odom(*node.odom)
        node._tick()
        self.assertEqual(self.h.output_count(), count)


if __name__ == "__main__":
    unittest.main()
