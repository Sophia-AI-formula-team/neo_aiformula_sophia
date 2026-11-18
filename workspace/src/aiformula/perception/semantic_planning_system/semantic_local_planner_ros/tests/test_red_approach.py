"""Red pre-planning cases for the two-stage distance-flag policy."""
import unittest

import test_map_assist as fixtures
import test_ubs_map_assist as ubs


class RedApproachTests(unittest.TestCase):
    def setUp(self):
        self.case = ubs.StopFlagAndUbsTests(methodName='runTest')
        self.case.setUp()
        self.addCleanup(self.case.doCleanups)
        self.h = self.case.h
        self.node = self.case.node
        self.case.set_fused_map()

    def test_reverse_official_order_selects_the_same_forward_edge(self):
        old = self.node.official
        identifiers = list(reversed(old.node_ids[:-1]))
        identifiers.append(identifiers[0])
        self.node.official = self.h.module.OfficialRoute(
            identifiers, [self.node.mapped_topology.nodes[k] for k in identifiers])
        self.node.topology_alignment['direction'] = -1
        self.case.red_far()
        self.assertEqual(self.node.blocked_edge, ['02', '03'])
        self.assertEqual(self.node.detour_node_ids, ['02', '06', '03'])
        self.assertEqual(self.node.traffic_mode, 'prepare')

    def test_missing_fusion_stops_only_when_distance_flag_triggers(self):
        self.node.fusion_status = {'available': False, 'reason': 'not_ready'}
        self.case.red_far()
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_map_fusion')
        self.h.assert_stopped()

    def test_green_before_distance_flag_cancels_the_prepared_route(self):
        self.case.red_far()
        self.assertTrue(self.node.smooth_path)
        self.h.confirm_light('green')
        self.assertEqual(self.node.traffic_mode, 'normal')
        self.assertIsNone(self.node.smooth_path)
        self.assertEqual(self.node.detour_path_pub.messages[-1].poses, [])

    def test_avoidance_owns_control_before_stop_flag(self):
        self.case.red_far()
        self.node._on_avoidance(fixtures.DataMessage(True))
        points = self.h.output_count()
        commands = len(self.node.cmd_pub.messages)
        self.node._tick()
        self.assertEqual(self.h.output_count(), points)
        self.assertEqual(len(self.node.cmd_pub.messages), commands)


if __name__ == '__main__':
    unittest.main()
