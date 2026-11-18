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
        node._try_complete_lap((0.0, 0.0, 0.0), False, (-0.05, 0.0, 0.0))
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

    def test_geometric_rejections_preserve_odom_and_wait_on_red(self):
        import json
        for reason in ("global_transform_ambiguous", "node_position_uncertain",
                       "solver_not_converged", "solver_budget_exhausted"):
            with self.subTest(reason=reason):
                node = self.prepare_circle_lap(traffic=True)
                reference = node.reference
                result = self.FusionResult(reference, {"reason": reason, "available": False},
                                           {"available": False, "mode": "odom_only", "reason": reason}, {}, {})
                node._fusion_executor.future.set_result(result)
                node._tick()
                self.assertIs(node.reference, reference)
                self.assertEqual(node.fusion_status["reason"], reason)
                node._publish_status()
                status = json.loads(node.status_pub.messages[-1].data)
                self.assertEqual(status["map_fusion"]["reason"], reason)
                self.assertTrue(status["odom_reference_ready"])
                self.h.confirm_light("red", stop_flag=True)
                node._tick()
                self.assertEqual(node.traffic_mode, "wait")
                self.assertEqual(node.traffic_reason, "red_wait_map_fusion")
                self.assertFalse(node.fused_node_positions)

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
        self.h.confirm_light("red", stop_flag=True)
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
        # This exercises the real asynchronous fusion using the actual
        # supplied topology and a newly generated, hypothetical smooth loop.
        # No previously recorded course or known ground-truth map is loaded.
        self.h.learned_official_loop()
        node = self.h.node
        self.assertTrue(node.fusion_status["available"], node.fusion_status)
        self.assertTrue(node.topology_mapping_status["available"], node.topology_mapping_status)
        self.h.confirm_light("red", stop_flag=True)
        node._tick()
        self.assertEqual(node.traffic_mode, "detour", node.traffic_reason)
        self.assertTrue(set(node.detour_node_ids) - set(node.official.node_ids))
        self.assertGreater(len(node.detour_tracker.points), len(node.detour_node_ids))
        count = self.h.output_count()
        node._on_avoidance(fixtures.DataMessage(True))
        self.h.advance()
        self.h.odom(*node.odom)
        node._tick()
        self.assertEqual(self.h.output_count(), count)


if __name__ == "__main__":
    unittest.main()
