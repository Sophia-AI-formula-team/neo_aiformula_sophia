"""ROS callback integration using stubs; no DDS or vehicle dynamics."""
import json
import math
import unittest

import test_map_assist as fixtures
from semantic_planner_core.rough_route import RouteReference, RouteSample


class StopFlagAndUbsTests(unittest.TestCase):
    def setUp(self):
        self.h = fixtures.MapAssistTests(methodName='runTest')
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.h.traffic_node()
        self.node = self.h.node
        self.addCleanup(lambda: self.node.close_fusion_worker())
        self.h.ready_visual_motion()

    def set_fused_map(self):
        nodes = {'00': (-20., 0.), '01': (-18., 0.), '02': (0., 0.),
                 '03': (40., 30.), '04': (40., 50.), '06': (0., 30.)}
        outer = ['00', '01', '02', '03', '04', '00']
        adjacency = {key: {} for key in nodes}
        for a, b in list(zip(outer, outer[1:])) + [('02', '06'), ('06', '03')]:
            adjacency[a][b] = adjacency[b][a] = math.dist(nodes[a], nodes[b])
        self.node.official = self.h.module.OfficialRoute(outer, [nodes[k] for k in outer])
        self.node.mapped_topology = self.h.module.TopologyGraph(nodes, adjacency, [outer])
        self.node.reference = RouteReference([
            RouteSample(-20, 0, 0), RouteSample(40, 0, 0),
            RouteSample(40, 50, 0), RouteSample(-20, 50, 0),
        ], 0.2, 0.0)
        self.node.fused_node_positions = {k: nodes[k] for k in outer[:-1]}
        self.node.fused_node_progress = {'00': 0., '01': 2., '02': 20., '03': 90., '04': 110.}
        self.node.topology_alignment = {'direction': 1}
        self.node.fusion_status = {'available': True, 'reason': 'synthetic_fixture'}
        self.node.progress = 8.0
        self.h.advance(4.0)
        # New session avoids tripping odometry jump validation in the fixture.
        self.node.odom = None
        self.node.odom_stamp = None
        self.h.triplet()
        self.h.odom(-12., 0., 0.)
        self.assertTrue(self.node.matched)

    def red_far(self):
        self.node._on_traffic_stop_flag(fixtures.DataMessage(False))
        self.h.advance(0.02)
        self.h.confirm_light('red')
        self.node._tick()

    def start_detour(self):
        self.set_fused_map()
        self.red_far()
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.assertTrue(self.node.smooth_path, self.node.smooth_plan_reason)
        # Force the trigger margin, not the distance flag, for tests which
        # exercise an already committed detour rather than trigger timing.
        self.node.p['detour_trigger_margin_m'] = 100.0
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'detour', self.node.traffic_reason)

    def test_first_lap_stops_only_at_distance_flag_and_waits_for_green(self):
        self.red_far()
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.assertEqual(self.node.traffic_reason, 'red_prepare_first_lap')
        self.h.triplet()
        self.node._tick()
        moving = self.h.command()
        self.node._on_controller_cmd(moving)
        self.assertEqual(self.h.twist_values(self.node.cmd_pub.messages[-1]), self.h.twist_values(moving))
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_first_lap')
        self.h.assert_stopped()
        self.node._on_traffic_stop_flag(fixtures.DataMessage(False))
        self.assertEqual(self.node.traffic_mode, 'wait', 'Flag drop must not release a red stop')
        self.h.confirm_light('green')
        self.assertEqual(self.node.traffic_mode, 'normal')

    def test_first_lap_red_stop_does_not_exclude_valid_mapping_samples(self):
        self.red_far()
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.h.advance(0.2)
        self.h.triplet()
        self.h.odom(0.25, 0.0, 0.0)
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertGreaterEqual(len(self.node.samples), 2)
        self.assertFalse(self.node.samples[-1].excluded)

    def test_distance_true_alone_does_not_stop_before_red_confirmation(self):
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.assertEqual(self.node.traffic_mode, 'normal')
        self.h.confirm_light('red')
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_first_lap')

    def test_status_exposes_accepted_but_nonunique_correspondence(self):
        self.node.topology_alignment = {'available': True, 'global_transform_available': False,
                                        'candidate_gap_enforced': False}
        self.node._publish_status()
        status = json.loads(self.node.status_pub.messages[-1].data)
        self.assertTrue(status['fusion_ambiguity_warning'])
        self.node.topology_alignment = {'available': True, 'global_transform_available': True,
                                        'node_correspondence_available': False}
        self.node._publish_status()
        self.assertTrue(json.loads(self.node.status_pub.messages[-1].data)['fusion_ambiguity_warning'])
        self.node.topology_alignment = {'available': True, 'global_transform_available': True,
                                        'node_correspondence_available': True}
        self.node._publish_status()
        self.assertFalse(json.loads(self.node.status_pub.messages[-1].data)['fusion_ambiguity_warning'])
        self.node.topology_alignment = {'available': False, 'global_transform_available': False}
        self.node._publish_status()
        self.assertFalse(json.loads(self.node.status_pub.messages[-1].data)['fusion_ambiguity_warning'])

    def test_red_prepares_graph_and_ubs_without_taking_visual_control(self):
        self.set_fused_map()
        self.red_far()
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.assertEqual(self.node.blocked_edge, ['02', '03'])
        self.assertEqual(self.node.detour_node_ids, ['02', '06', '03'])
        self.assertTrue(self.node.smooth_path, self.node.smooth_plan_reason)
        self.assertLessEqual(self.node.smooth_path.max_curvature, 0.251)
        visual = self.h.triplet()
        self.node._tick()
        self.h.assert_last_points_equal(visual)
        self.assertTrue(self.node.detour_path_pub.messages[-1].poses)
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_stop_point_before_turn')
        self.node.p['detour_trigger_margin_m'] = 100.0
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'detour', self.node.traffic_reason)
        self.assertTrue(self.node.detour_motion_ready)
        for label in 'abc':
            point = self.node.point_pubs[label].messages[-1]
            self.assertGreater(point.x, 0.0)
            self.assertLess(abs(math.degrees(math.atan2(point.y, point.x))), 65.0)

    def test_no_distance_topic_and_infeasible_radius_hold_safely(self):
        self.h.confirm_light('red')
        self.assertEqual(self.node.traffic_reason, 'red_wait_flag_unavailable')
        self.h.assert_stopped()
        self.node._on_traffic_stop_flag(fixtures.DataMessage(False))
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.h.confirm_light('green')
        self.set_fused_map()
        self.node.p['detour_min_turn_radius_m'] = 100.0
        self.red_far()
        self.assertFalse(self.node.smooth_path)
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertIn('spline', self.node.traffic_reason)
        self.h.assert_stopped()

    def test_first_distance_message_true_keeps_first_lap_stopped(self):
        self.h.confirm_light('red')
        self.assertEqual(self.node.traffic_reason, 'red_wait_flag_unavailable')
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.node._tick()
        self.assertTrue(self.node.stop_committed)
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_first_lap')

    def test_oa_owns_points_and_commands_while_detouring(self):
        self.start_detour()
        count = self.h.output_count()
        commands = len(self.node.cmd_pub.messages)
        self.node._on_avoidance(fixtures.DataMessage(True))
        self.assertEqual(len(self.node.cmd_pub.messages), commands, 'OA start must not inject a stop')
        self.h.advance(0.1)
        self.h.odom(*self.node.odom)
        self.node._tick()
        self.assertEqual(self.h.output_count(), count)
        self.h.advance(0.6)
        self.node._tick()
        self.assertEqual(len(self.node.cmd_pub.messages), commands, 'OA owns commands even when the gate timer expires')
        invalid = self.h.command()
        invalid.angular.z = math.nan
        self.node._on_controller_cmd(invalid)
        self.assertEqual(len(self.node.cmd_pub.messages), commands, 'Invalid OA command is dropped, not replaced')
        moving = self.h.command(0.4, -0.2)
        self.node._on_controller_cmd(moving)
        self.assertEqual(self.h.twist_values(self.node.cmd_pub.messages[-1]), self.h.twist_values(moving))
        self.node._on_avoidance(fixtures.DataMessage(False))
        self.h.advance(0.25)
        self.h.odom(*self.node.odom)
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'detour')
        self.assertGreater(self.h.output_count(), count)

    def test_green_does_not_abandon_a_committed_turn(self):
        self.start_detour()
        tracker = self.node.detour_tracker
        self.h.confirm_light('green')
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'detour')
        self.assertIs(self.node.detour_tracker, tracker)

    def test_same_uncleared_red_event_does_not_restart_at_exit(self):
        self.start_detour()
        self.node._on_traffic_stop_flag(fixtures.DataMessage(False))
        self.h.confirm_light('red', stop_flag=True)
        self.assertFalse(self.node.detour_next_red_pending)
        self.h.emit_detour_completion()
        self.assertEqual(self.node.traffic_mode, 'normal')
        self.assertTrue(self.node.traffic_event_consumed)
        self.h.confirm_light('red', stop_flag=True)
        self.assertEqual(self.node.traffic_mode, 'normal')

    def test_new_red_event_during_detour_is_not_lost(self):
        self.start_detour()
        self.h.confirm_light('green')
        self.h.confirm_light('red')
        self.assertTrue(self.node.detour_next_red_pending)
        self.h.emit_detour_completion()
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.assertEqual(self.node.traffic_reason, 'red_prepare_next_event')
        self.assertFalse(self.node.traffic_event_consumed)

    def test_green_clears_a_new_red_event_during_detour(self):
        self.start_detour()
        self.h.confirm_light('green')
        self.h.confirm_light('red')
        self.assertTrue(self.node.detour_next_red_pending)
        self.h.confirm_light('green')
        self.assertFalse(self.node.detour_next_red_pending)
        self.assertEqual(self.node.traffic_mode, 'detour')
        self.h.emit_detour_completion()
        self.assertEqual(self.node.traffic_mode, 'normal')

    def test_distance_event_during_oa_is_deferred_until_oa_releases(self):
        self.set_fused_map()
        self.red_far()
        self.node.p['detour_trigger_margin_m'] = 100.0
        self.node._on_avoidance(fixtures.DataMessage(True))
        points = self.h.output_count()
        commands = len(self.node.cmd_pub.messages)
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.node._tick()
        self.assertEqual(self.h.output_count(), points)
        self.assertEqual(len(self.node.cmd_pub.messages), commands)
        moving = self.h.command(0.4, -0.2)
        self.node._on_controller_cmd(moving)
        self.assertEqual(self.h.twist_values(self.node.cmd_pub.messages[-1]), self.h.twist_values(moving))
        self.node._on_avoidance(fixtures.DataMessage(False))
        self.h.advance(0.25)
        self.h.odom(*self.node.odom)
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'detour', self.node.traffic_reason)

    def test_full_ideal_pose_replay_returns_to_main_line_and_visual_control(self):
        self.start_detour()
        path = self.node.smooth_path
        expected_progress = self.node.detour_exit_progress + path.trim_m + 2.0
        last_pose = self.node.odom
        for index in range(1, len(path.points) - 1):
            # Keep execution fast while obeying the node's odometry speed checks.
            if index % 3:
                continue
            p, q = path.points[index], path.points[index + 1]
            yaw = math.atan2(q[1] - p[1], q[0] - p[0])
            last_pose = (p[0], p[1], yaw)
            self.h.advance(0.2)
            self.h.odom(*last_pose)
            self.node._tick()
            if self.node.traffic_mode == 'normal':
                break
        self.assertEqual(self.node.traffic_mode, 'normal', self.node.traffic_reason)
        self.assertEqual(self.node.traffic_reason, 'detour_complete_resume_vision')
        self.assertAlmostEqual(self.node.progress, expected_progress)
        self.h.advance(0.25)
        self.h.triplet()
        self.h.odom(*last_pose)
        self.node._tick()
        self.assertTrue(self.node.matched, 'Rejoin seeds progress at the real main-line location')
        self.assertTrue(self.node.visual_motion_ready)


if __name__ == '__main__':
    unittest.main()
