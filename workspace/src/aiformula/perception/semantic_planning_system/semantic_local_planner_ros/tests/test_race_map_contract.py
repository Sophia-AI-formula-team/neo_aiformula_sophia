"""Static contract checks for the independently runnable race_map executable."""

import ast
from pathlib import Path
import unittest


class RaceMapContractTests(unittest.TestCase):
    def setUp(self):
        self.package = Path(__file__).resolve().parents[1]
        self.source = self.package / "semantic_local_planner_ros" / "race_map.py"
        self.setup_source = (self.package / "setup.py").read_text(encoding="utf-8")
        self.config = (self.package / "config" / "race_map.yaml").read_text(encoding="utf-8")

    def test_console_entrypoint_and_node_load_race_profile(self):
        self.assertIn(
            "race_map = semantic_local_planner_ros.race_map:main",
            self.setup_source,
        )
        tree = ast.parse(self.source.read_text(encoding="utf-8"), filename=str(self.source))
        self.assertTrue(any(
            isinstance(node, ast.ClassDef)
            and node.name == "RaceMap"
            and any(isinstance(base, ast.Name) and base.id == "MapAssist" for base in node.bases)
            for node in tree.body
        ))
        self.assertIn('super().__init__(node_name="race_map")',
                      self.source.read_text(encoding="utf-8"))
        self.assertIn('startup_args(args, "race_map.yaml")',
                      self.source.read_text(encoding="utf-8"))

    def test_vehicle_profile_matches_ros_interfaces_and_requested_thresholds(self):
        for expected in (
            "race_map:",
            "odom_topic: /aiformula_sensing/gyro_odometry_publisher/odom",
            "avoidance_flag_topic: /lane_stop_flag",
            "input_point_a_topic: /original_processed_point_a",
            "output_point_a_topic: /processed_point_a",
            "traffic_light_topic: /map_assist/traffic_light_input",
            "input_cmd_vel_topic: /map_assist/controller_cmd_vel",
            "output_cmd_vel_topic: /aiformula_control/game_pad/cmd_vel",
            "enable_traffic_control: true",
            "traffic_policy: first_lap_wait_then_detour",
            "traffic_vote_window_sec: 0.5",
            "traffic_vote_min_samples: 3",
            "enable_velocity_gate: true",
            "min_lap_distance_m: 0.0",
            "lap_closure_radius_m: 5.0",
            "alignment_max_rmse_m: 4.0",
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, self.config)


if __name__ == "__main__":
    unittest.main()