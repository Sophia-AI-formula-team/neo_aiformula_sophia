"""Check the two 0.4.5 ROS status adapters without ROS or motor outputs."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

scripts = Path(__file__).resolve().parents[1] / "shellscript" / "mapassist_045"
with patch.object(sys, "path", [str(scripts)] + sys.path):
    from guarded_stop_wait import translate
    from stop_wait_preview import convert_status


class LightReleaseTests(unittest.TestCase):
    def setUp(self):
        self.fields = dict(traffic_planner_version="ubs_auto_turn_v1", use_sim_time=False,
                           traffic_mode="normal", traffic_reason="traffic_idle",
                           ros_time_sec=123.0, path_output_allowed=True,
                           avoidance_active=False, branch_active=False,
                           traffic_stop_committed=False, auto_stop_latched=False,
                           velocity_gate_reason="forwarding_controller")
        self.now = 123_000_000_000

    def test_diagnostic_resume_requires_green_observation_and_green_vote(self):
        for raw, vote, expected in (("unknown", "green", "wait"), ("green", "not_red", "wait"),
                                    ("off", "off", "wait"), ("green", "green", "normal")):
            with self.subTest(raw=raw, vote=vote):
                data = dict(self.fields, traffic_light_raw=raw, traffic_light=vote)
                self.assertEqual(convert_status(json.dumps(data), self.now)["traffic_mode"], expected)

    def test_formal_adapter_holds_old_non_green_releases(self):
        for reason in ("red_off_resume", "non_red_resume"):
            data = dict(self.fields, traffic_light_raw="unknown", traffic_light="not_red",
                        traffic_reason=reason)
            self.assertTrue(translate(json.dumps(data), self.now, True)["traffic_stop_required"])

    def test_authorized_turn_or_lane_following_can_continue_without_visible_light(self):
        for mode in ("normal", "detour", "wait"):
            with self.subTest(mode=mode):
                data = dict(self.fields, traffic_light_raw="unknown", traffic_light="unknown",
                            traffic_mode=mode, smooth_plan_available=True, route_matched=True)
                self.assertEqual(translate(json.dumps(data), self.now, True)["traffic_stop_required"],
                                 mode == "wait")
