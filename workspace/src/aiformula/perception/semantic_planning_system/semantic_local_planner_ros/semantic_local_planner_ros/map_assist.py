"""Learn a lap, assist vision, and wait or replan on a red-light event.

The original avoidance node owns path-point output while its Bool flag is True.
This node only observes then: no corrected, original, or zero points are sent.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
from collections import deque
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from semantic_planner_core.rough_route import HeadingGuard, RouteSample
from semantic_planner_core.rough_topology import OfficialRoute
from semantic_planner_core.route_fusion import build_fused_route
from semantic_planner_core.anchored_topology import anchored_graph
from semantic_planner_core.traffic_route import DetourTracker, OfficialToOdom, TopologyGraph
from semantic_planner_core.traffic_light_filter import TrafficLightFilter
from semantic_planner_core.signal_approach import parse_signal_observation, stop_envelope


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def installed_config(filename: str) -> str:
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory("semantic_local_planner_ros"), "config", filename)


def startup_args(args=None) -> List[str]:
    """Bare ros2 run uses the vehicle profile; bag replay selects its own file."""
    argv = list(sys.argv[1:] if args is None else args)
    if "--params-file" not in argv:
        argv = ["--ros-args", "--params-file", installed_config("map_assist.yaml"), "--"] + argv
    return argv


class MapAssist(Node):
    """Learn in RAM, then gently constrain persistent visual heading outliers."""

    def __init__(self) -> None:
        super().__init__("map_assist")
        defaults = {
            "odom_topic": "/aiformula_sensing/gyro_odometry_publisher/odom",
            "avoidance_flag_topic": "/lane_stop_flag",
            "avoidance_initially_idle": False,
            "branch_active_topic": "/map_assist/branch_active",
            "reset_topic": "/map_assist/reset",
            "status_topic": "/map_assist/status",
            "reference_path_topic": "/map_assist/path",
            "topomap_path": "",
            "route_node_ids": "",
            "angle_tolerance_deg": 15.0,
            "progress_tolerance_m": 1.0,
            "persistence_frames": 5,
            "max_correction_deg": 8.0,
            "max_step_deg": 2.0,
            "sample_spacing_m": 0.2,
            "smooth_window_m": 1.0,
            "max_detour_gap_m": 2.0,
            "min_lap_distance_m": 40.0,
            "lap_closure_radius_m": 1.5,
            "lap_closure_heading_deg": 25.0,
            "match_window_m": 3.0,
            "max_match_distance_m": 1.5,
            "max_odom_speed_mps": 4.0,
            "max_odom_yaw_rate_radps": 3.0,
            "max_odom_position_std_m": 1.0,
            "max_odom_yaw_std_deg": 20.0,
            "max_odom_gap_sec": 2.0,
            "max_recorded_samples": 20000,
            "publish_rate_hz": 20.0,
            "point_timeout_sec": 0.5,
            "max_point_skew_sec": 0.15,
            "odom_timeout_sec": 0.5,
            "resume_delay_sec": 0.2,
            "enable_correction": True,
            "enable_traffic_control": False,
            "traffic_light_topic": "/map_assist/traffic_light_input",
            "traffic_vote_window_sec": 0.5,
            "traffic_vote_min_samples": 3,
            "enable_signal_approach": False,
            "signal_observation_topic": "/map_assist/signal_observation",
            "signal_observation_timeout_sec": 0.5,
            "target_stop_distance_m": 8.5,
            "approach_max_speed_mps": 0.8,
            "approach_deceleration_mps2": 1.0,
            "approach_latency_sec": 0.2,
            "approach_stop_tolerance_m": 0.1,
            "enable_velocity_gate": False,
            "input_cmd_vel_topic": "/map_assist/controller_cmd_vel",
            "output_cmd_vel_topic": "/aiformula_control/game_pad/cmd_vel",
            "cmd_vel_timeout_sec": 0.5,
            "alignment_max_rmse_m": 3.0,
            "alignment_ambiguity_margin_m": 0.3,
            "fusion_node_search_m": 4.0,
            "fusion_feature_window_m": 2.0,
            "fusion_shape_weight": 0.25,
            "fusion_max_displacement_m": 0.75,
            "detour_entry_radius_m": 3.0,
            "detour_waypoint_radius_m": 0.8,
            "detour_max_cross_track_m": 2.0,
            "detour_lookahead_m": 0.5,
            "detour_point_spacing_m": 0.5,
        }
        for label in "abc":
            defaults[f"input_point_{label}_topic"] = f"/original_processed_point_{label}"
            defaults[f"output_point_{label}_topic"] = f"/processed_point_{label}"
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if not self.p["topomap_path"]:
            self.p["topomap_path"] = installed_config("aiformula_topomap.osm")
        self._validate_parameters()
        ids = [item.strip() for item in self.p["route_node_ids"].split(",") if item.strip()]
        self.official = OfficialRoute.from_osm(self.p["topomap_path"], ids or None)
        self.topology = TopologyGraph.from_osm(self.p["topomap_path"])
        self.guard = HeadingGuard(
            angle_tolerance_rad=math.radians(self.p["angle_tolerance_deg"]),
            persistence_frames=self.p["persistence_frames"],
            max_correction_rad=math.radians(self.p["max_correction_deg"]),
            max_step_rad=math.radians(self.p["max_step_deg"]),
        )
        self.points = {label: None for label in "abc"}
        self.point_times = {label: -math.inf for label in "abc"}
        self.point_seq = {label: 0 for label in "abc"}
        self.used_seq = {label: 0 for label in "abc"}
        # Event state stays latched until an explicit opposite event. Unknown
        # startup state cannot accidentally compete with an active avoidance.
        self.avoidance_active = False if self.p["avoidance_initially_idle"] else None
        self.branch_active = False
        self.resume_after = 0.0
        self.odom = None
        self.odom_time = -math.inf
        self.odom_stamp = None
        self.odom_frame = "odom"
        self.child_frame = ""
        self.odom_covariance_available = False
        self.last_status_time = -math.inf
        self.odom_seq = 0
        self.light_filter = TrafficLightFilter(
            window_sec=self.p["traffic_vote_window_sec"],
            min_samples=self.p["traffic_vote_min_samples"],
        )
        self.traffic_light_raw = "unknown"
        self.traffic_light = "unknown"
        self.traffic_mode = "normal"
        self.traffic_reason = "traffic_idle"
        self.signal_observation = None
        self.signal_observation_time = -math.inf
        self.signal_observation_sequence = -1
        self.signal_range_reason = "signal_range_missing"
        self.signal_wait_latched = False
        self.odom_speed_mps = None
        self.odom_speed_time = -math.inf
        self.odom_speed_history = deque()
        self.topology_alignment = None
        self.official_to_odom = None
        self.fusion_status = {"available": False, "reason": "not_built"}
        self.fused_node_positions = {}
        self.fused_node_progress = {}
        self.mapped_topology = None
        self.topology_mapping_status = {"available": False, "reason": "not_built"}
        self.detour_tracker = None
        self.detour_node_ids = []
        self.blocked_edge = None
        self.detour_exit_progress = 0.0
        self.detour_used_odom_seq = -1
        self.detour_motion_ready = False
        self.visual_motion_ready = False
        self.last_cmd_time = -math.inf
        self.velocity_gate_reason = "waiting_controller_command"
        self._reset_learning("waiting_for_odom")

        self.point_pubs = {}
        for label in "abc":
            self.create_subscription(Pose2D, self.p[f"input_point_{label}_topic"], self._point_cb(label), 10)
            self.point_pubs[label] = self.create_publisher(Pose2D, self.p[f"output_point_{label}_topic"], 10)
        self.create_subscription(Odometry, self.p["odom_topic"], self._on_odom, 10)
        self.create_subscription(Bool, self.p["avoidance_flag_topic"], self._on_avoidance, 10)
        self.create_subscription(Bool, self.p["branch_active_topic"], self._on_branch, 10)
        self.create_subscription(Bool, self.p["reset_topic"], self._on_reset, 10)
        self.status_pub = self.create_publisher(String, self.p["status_topic"], 10)
        self.path_pub = self.create_publisher(
            Path, self.p["reference_path_topic"],
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.cmd_pub = None
        if self.p["enable_velocity_gate"]:
            self.cmd_pub = self.create_publisher(Twist, self.p["output_cmd_vel_topic"], 10)
            self.create_subscription(Twist, self.p["input_cmd_vel_topic"], self._on_controller_cmd, 10)
        if self.p["enable_traffic_control"]:
            self.create_subscription(String, self.p["traffic_light_topic"], self._on_light_state, 10)
            if self.p["enable_signal_approach"]:
                self.create_subscription(String, self.p["signal_observation_topic"], self._on_signal_observation, 10)
        self.create_timer(1.0 / self.p["publish_rate_hz"], self._tick)
        # Diagnostics must remain visible before /clock arrives and while a bag
        # is paused. Only this status timer uses steady time; control stays on
        # the node's ROS clock and cannot run from the diagnostic callback.
        self.status_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.status_timer = self.create_timer(0.2, self._status_tick, clock=self.status_clock)
        self._status_tick()
        self.get_logger().info(
            "Map assist: first lap learns in RAM; points use metres, x forward/y left. "
            f"Initial avoidance state: {self.avoidance_active}; unknown waits for a real False flag. "
            f"Official map: {self.p['topomap_path']}"
        )

    def _validate_parameters(self) -> None:
        for name in (
            "sample_spacing_m", "min_lap_distance_m",
            "lap_closure_radius_m", "match_window_m", "max_match_distance_m",
            "max_odom_speed_mps", "max_odom_yaw_rate_radps", "max_odom_position_std_m",
            "max_odom_yaw_std_deg", "max_odom_gap_sec", "publish_rate_hz",
            "point_timeout_sec", "max_point_skew_sec", "odom_timeout_sec",
            "cmd_vel_timeout_sec", "alignment_max_rmse_m",
            "fusion_node_search_m", "fusion_feature_window_m", "fusion_shape_weight",
            "fusion_max_displacement_m", "detour_entry_radius_m", "detour_waypoint_radius_m",
            "detour_max_cross_track_m", "detour_lookahead_m", "detour_point_spacing_m",
            "signal_observation_timeout_sec", "target_stop_distance_m",
            "approach_max_speed_mps", "approach_deceleration_mps2", "approach_stop_tolerance_m",
        ):
            if not math.isfinite(float(self.p[name])) or self.p[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("smooth_window_m", "max_detour_gap_m", "progress_tolerance_m", "resume_delay_sec", "approach_latency_sec"):
            if not math.isfinite(float(self.p[name])) or self.p[name] < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if (not math.isfinite(self.p["alignment_ambiguity_margin_m"])
                or self.p["alignment_ambiguity_margin_m"] <= 0):
            raise ValueError("Invalid topology alignment quality limits")
        if self.p["fusion_shape_weight"] >= 1.0:
            raise ValueError("fusion_shape_weight must be in (0, 1)")
        if self.p["enable_traffic_control"] and not self.p["enable_velocity_gate"]:
            raise ValueError("Traffic waiting requires the velocity gate; zero path points cannot stop LYA")
        if self.p["enable_signal_approach"] and not self.p["enable_traffic_control"]:
            raise ValueError("Signal approach requires traffic control and its velocity gate")
        if self.p["signal_observation_topic"] == self.p["traffic_light_topic"]:
            raise ValueError("Atomic observations and legacy color topics must differ")
        if self.p["input_cmd_vel_topic"] == self.p["output_cmd_vel_topic"]:
            raise ValueError("Velocity input/output must differ; feedback is not allowed")
        for name in ("angle_tolerance_deg", "max_correction_deg", "max_step_deg", "lap_closure_heading_deg"):
            if not math.isfinite(float(self.p[name])) or not 0 < self.p[name] < 90:
                raise ValueError(f"{name} must be between 0 and 90 degrees")
        if self.p["persistence_frames"] < 1 or self.p["max_recorded_samples"] < 30:
            raise ValueError("Invalid frame/sample count")
        inputs = {self.p[f"input_point_{label}_topic"] for label in "abc"}
        outputs = {self.p[f"output_point_{label}_topic"] for label in "abc"}
        if len(inputs) != 3 or len(outputs) != 3 or inputs & outputs:
            raise ValueError("Input/output point topics must be distinct; feedback is not allowed")

    def _reset_learning(self, reason: str) -> None:
        was_detouring = self.traffic_mode == "detour"
        was_approaching = self.traffic_mode == "approach"
        self.signal_observation = None
        self.signal_range_reason = "signal_range_reset"
        self.odom_speed_mps = None
        self.odom_speed_time = -math.inf
        self.odom_speed_history.clear()
        # Odom can reset the map before the next control tick. Never release a
        # committed detour using an old green from a detector that has stopped.
        self.traffic_light = self.light_filter.evaluate(time.monotonic())
        self.topology_alignment = None
        self.official_to_odom = None
        self.fusion_status = {"available": False, "reason": reason}
        self.fused_node_positions = {}
        self.fused_node_progress = {}
        self.mapped_topology = None
        self.topology_mapping_status = {"available": False, "reason": reason}
        self.detour_tracker = None
        self.detour_node_ids = []
        self.blocked_edge = None
        self.detour_motion_ready = False
        self.visual_motion_ready = False
        self.last_cmd_time = -math.inf
        if self.p["enable_traffic_control"]:
            if was_detouring and self.traffic_light == "green":
                self._resume_vision("map_reset_resume_vision")
            elif was_detouring or was_approaching or self.traffic_light == "red":
                self.traffic_mode = "wait"
                self.traffic_reason = "map_reset_wait"
                self.signal_wait_latched |= was_approaching
        self.samples = []
        self.reference = None
        self.recorded_distance = 0.0
        self.left_start_area = False
        self.excluded_since_sample = False
        self.progress = 0.0
        self.pending_distance = 0.0
        self.matched = False
        self.alignment = None
        self.reason = reason
        self.last_correction = 0.0
        self.guard.reset()

    def _on_reset(self, msg: Bool) -> None:
        if msg.data:
            self._reset_learning("reset_learning")
            self.odom = None
            self.odom_stamp = None
            self._publish_path()  # Clear the previous latched reference.

    def _on_avoidance(self, msg: Bool) -> None:
        active = bool(msg.data)
        previous = self.avoidance_active
        self.avoidance_active = active
        if active or previous is not False:
            self.excluded_since_sample = True
            self.guard.reset()
            self.last_correction = 0.0
        if previous is not active:
            # Require a whole new visual triplet after the real exit event.
            self.used_seq = dict(self.point_seq)
            self.visual_motion_ready = False
            self.detour_motion_ready = False
            self.last_cmd_time = -math.inf
        if not active and previous is not False:
            self.resume_after = time.monotonic() + self.p["resume_delay_sec"]
            self.reason = "resume_delay"
        elif active:
            self.reason = "avoidance_active"
        self._publish_velocity_stop_if_needed(time.monotonic())

    def _on_light_state(self, msg: String) -> None:
        # A legacy color has no corresponding metric observation. Never attach
        # it to a cached range from another detector frame.
        self.signal_observation = None
        self.signal_range_reason = "signal_range_missing"
        self._receive_light_state(msg.data)

    def _on_signal_observation(self, msg: String) -> None:
        """Atomic real detector frame; duplicate frames never refresh or vote."""
        now = time.monotonic()
        try:
            observation = parse_signal_observation(msg.data)
        except (ValueError, TypeError):
            self.signal_observation = None
            self.signal_range_reason = "signal_observation_invalid"
            self._update_light_vote(now)
            self._publish_velocity_stop_if_needed(now)
            return
        if observation.sequence <= self.signal_observation_sequence:
            return
        self.signal_observation_sequence = observation.sequence
        if observation.age_sec > self.p["signal_observation_timeout_sec"]:
            self.signal_observation = None
            self.signal_range_reason = "signal_observation_stale"
            self._update_light_vote(now)
            self._publish_velocity_stop_if_needed(now)
            return
        self.signal_observation = observation
        self.signal_observation_time = now
        self.signal_range_reason = observation.range_reason
        self._receive_light_state(observation.state)

    def _receive_light_state(self, data: str) -> None:
        state = str(data).strip().lower()
        self.traffic_light_raw = state if state in ("red", "green") else "unknown"
        if not self.p["enable_traffic_control"]:
            return
        now = time.monotonic()
        self.light_filter.add(self.traffic_light_raw, now)
        self._update_light_vote(now)
        self._publish_velocity_stop_if_needed(now)

    def _update_light_vote(self, now: float) -> None:
        if not self.p["enable_traffic_control"]:
            return
        self.traffic_light = self.light_filter.evaluate(now)
        if self.traffic_mode == "approach":
            profile = self._signal_approach_profile(now)
            # Keep confirmed red intent when older votes expire or tie. The
            # current real color and fresh range still guard every approach;
            # a historical unknown vote alone is not a new sensing failure.
            if self.traffic_light_raw == "unknown":
                self._wait_for_signal("signal_color_unknown", fault=True)
            elif not profile["valid"]:
                self._wait_for_signal(profile["reason"], fault=True)
            elif profile["remaining_m"] <= self.p["approach_stop_tolerance_m"]:
                self._wait_for_signal("signal_target_reached")
        if self.traffic_light == "red" and self.traffic_mode == "normal":
            self.traffic_mode = "wait"
            self.traffic_reason = "red_wait_first_lap" if self.reference is None else "red_planning"
            if self.p["enable_signal_approach"]:
                profile = self._signal_approach_profile(now)
                if profile["valid"] and self.traffic_light_raw == "red" and profile["remaining_m"] > self.p["approach_stop_tolerance_m"]:
                    self.traffic_mode = "approach"
                    self.traffic_reason = "red_approach_signal"
            self.guard.reset()
            self.last_correction = 0.0
            self.visual_motion_ready = False
            if self.avoidance_active is not True:
                self.last_cmd_time = -math.inf
        elif self.traffic_light == "green" and self.traffic_mode in ("wait", "approach") and (
            not self.p["enable_signal_approach"] or self.traffic_light_raw == "green"
        ):
            self._resume_vision("green_resume")
        # Unknown/missing detections never release an already confirmed red.
        # A committed detour finishes even if its light subsequently turns green.

    def _resume_vision(self, reason: str) -> None:
        self.traffic_mode = "normal"
        self.traffic_reason = reason
        self.signal_wait_latched = False
        self.detour_tracker = None
        self.detour_motion_ready = False
        self.visual_motion_ready = False
        self.used_seq = dict(self.point_seq)
        self.resume_after = time.monotonic() + self.p["resume_delay_sec"]
        if self.avoidance_active is not True:
            self.last_cmd_time = -math.inf
        self.guard.reset()
        self.last_correction = 0.0

    def _wait_for_signal(self, reason: str, fault: bool = False) -> None:
        self.traffic_mode = "wait"
        self.traffic_reason = reason
        self.signal_wait_latched |= fault
        self.visual_motion_ready = False
        self.last_cmd_time = -math.inf
        self.guard.reset()
        self.last_correction = 0.0

    def _signal_approach_profile(self, now: float) -> dict:
        observation = self.signal_observation
        if observation is None or observation.distance_m is None:
            return {"valid": False, "reason": self.signal_range_reason}
        age = observation.age_sec + max(0.0, now - self.signal_observation_time)
        if age > self.p["signal_observation_timeout_sec"]:
            return {"valid": False, "reason": "signal_observation_stale", "age_sec": age}
        if (self.odom_speed_mps is None or now - self.odom_speed_time > self.p["odom_timeout_sec"]
                or now - self.odom_time > self.p["odom_timeout_sec"]):
            return {"valid": False, "reason": "signal_approach_odom_stale"}
        # Retain one sample preceding capture to cover its first integration interval.
        capture_time = self.signal_observation_time - observation.age_sec
        history = list(self.odom_speed_history)
        before = [v for t, v in history if t <= capture_time]
        speeds = [v for t, v in history if t >= capture_time]
        speed_bound = max([0.0, self.odom_speed_mps] + speeds + before[-1:])
        profile = stop_envelope(
            observation.distance_m, observation.uncertainty_m, age,
            speed_bound, self.odom_speed_mps, self.p["target_stop_distance_m"],
            self.p["approach_max_speed_mps"], self.p["approach_deceleration_mps2"],
            self.p["approach_latency_sec"],
        )
        profile.update(valid=True, reason="ok", age_sec=age)
        return profile

    def _velocity_hold_reason(self, now: float) -> Optional[str]:
        # OA still owns the path and moving commands during its active phase.
        # Red waits remain latched and are reconsidered when OA releases control.
        if self.avoidance_active is True:
            return None
        blocked = self._output_block_reason(now)
        if blocked:
            return blocked
        if self.p["enable_traffic_control"] and self.traffic_mode == "wait":
            return self.traffic_reason
        if self.traffic_mode == "approach":
            profile = self._signal_approach_profile(now)
            if not profile["valid"]:
                return profile["reason"]
            if profile["braking_required"]:
                return "approach_braking"
        if self.traffic_mode == "detour":
            if not self.detour_motion_ready or now - self.odom_time > self.p["odom_timeout_sec"]:
                return "detour_waiting_valid_odom_and_points"
        elif not self.visual_motion_ready or not self._fresh_points(now):
            return "waiting_fresh_visual_triplet"
        return None

    def _on_controller_cmd(self, msg: Twist) -> None:
        now = time.monotonic()
        self._update_light_vote(now)
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.last_cmd_time = -math.inf
            self.velocity_gate_reason = "invalid_controller_command"
            self.cmd_pub.publish(Twist())
            return
        held = self._velocity_hold_reason(now)
        if held:
            self.last_cmd_time = -math.inf
            self.velocity_gate_reason = held
            self.cmd_pub.publish(Twist())
            return
        self.last_cmd_time = now
        if self.traffic_mode == "approach" and self.avoidance_active is not True:
            limit = self._signal_approach_profile(now)["speed_limit_mps"]
            # Preserve LYA curvature while slowing; never reverse or rotate in
            # place to approach a red light, and never mutate the upstream cmd.
            limited = Twist()
            if msg.linear.x > 0:
                scale = min(1.0, limit / msg.linear.x)
                limited.linear.x = msg.linear.x * scale
                limited.angular.z = msg.angular.z * scale
            self.velocity_gate_reason = "signal_approach_speed_limit"
            self.cmd_pub.publish(limited)
            return
        self.velocity_gate_reason = "forwarding_controller"
        self.cmd_pub.publish(msg)

    def _publish_velocity_stop_if_needed(self, now: float) -> None:
        if not self.p["enable_velocity_gate"] or self.cmd_pub is None:
            return
        held = self._velocity_hold_reason(now)
        if held or now - self.last_cmd_time > self.p["cmd_vel_timeout_sec"]:
            self.velocity_gate_reason = held or "controller_command_timeout"
            if held:
                self.last_cmd_time = -math.inf
            self.cmd_pub.publish(Twist())
        # Moving commands are only forwarded on receipt; never replay a cache.

    def _prepare_mapped_topology(self) -> None:
        """Attach the official graph to the already fused odometry-frame nodes.

        Fusion is a construction step independent of traffic control. Its
        inverse similarity supplies the initial internal-node estimates only;
        outer junctions use the final reference anchors exactly.
        """
        self.official_to_odom = None
        self.mapped_topology = None
        self.topology_mapping_status = {"available": False, "reason": "fusion_unavailable"}
        if not self.fusion_status.get("available") or not self.topology_alignment:
            return
        try:
            self.official_to_odom = OfficialToOdom(
                **{key: self.topology_alignment[key] for key in
                   ("scale", "rotation_rad", "translation_x", "translation_y", "rmse_m")}
            )
            self.mapped_topology = anchored_graph(
                self.topology, self.fused_node_positions, self.official_to_odom,
            )
            self.topology_mapping_status = {
                "available": True, "reason": "outer_nodes_anchored_to_fused_reference",
                "internal_positions": "estimated_from_official_graph_and_outer_anchors",
            }
        except (ValueError, KeyError) as exc:
            self.topology_mapping_status = {"available": False, "reason": str(exc)}
            self.official_to_odom = None

    def _node_progress(self, node_id: str) -> float:
        # Progress is measured on the final smoothed curve, after node matching
        # and deformation. A global official-length ratio is not valid here.
        return self.fused_node_progress[node_id]

    def _current_topology_edge(self):
        if not self.fusion_status.get("available") or not self.fused_node_progress:
            return None
        fraction_s = self.progress % self.reference.length_m
        ordered = sorted(self.fused_node_progress, key=self.fused_node_progress.get)
        before = [key for key in ordered if self.fused_node_progress[key] <= fraction_s]
        start = before[-1] if before else ordered[-1]
        return [start, ordered[(ordered.index(start) + 1) % len(ordered)]]

    def _update_traffic(self, now: float) -> None:
        if not self.p["enable_traffic_control"] or self.traffic_mode != "wait" or self.traffic_light != "red":
            return
        if self.signal_wait_latched:
            return  # Observation faults cannot be released by a mapped detour.
        if self.reference is None:
            if self.traffic_reason != "signal_target_reached":
                self.traffic_reason = "red_wait_first_lap"
            return
        if self._output_block_reason(now) or self.branch_active:
            self.traffic_reason = "red_wait_maneuver_release"
            return
        if not self.fusion_status.get("available"):
            self.traffic_reason = "red_wait_map_fusion"
            return
        if self.mapped_topology is None:
            self.traffic_reason = "red_wait_mapped_topology"
            return
        if self.odom is None or now - self.odom_time > self.p["odom_timeout_sec"] or not self.matched:
            self.traffic_reason = "red_wait_localization"
            return
        outer = self.official.node_ids[:-1]
        locations = self.fused_node_positions
        length = self.reference.length_m
        candidates = [key for key in outer if abs(
            (self._node_progress(key) - self.progress + length * 0.5) % length - length * 0.5
        ) <= self.p["detour_entry_radius_m"] + self.p["progress_tolerance_m"]]
        if not candidates:
            self.traffic_reason = "red_wait_near_junction"
            return
        start = min(candidates, key=lambda key: math.dist(self.odom[:2], locations[key]))
        if math.dist(self.odom[:2], locations[start]) > self.p["detour_entry_radius_m"]:
            self.traffic_reason = "red_wait_near_junction"
            return
        # Phase/direction identify the normal next edge; red removes that edge.
        direction = int(self.topology_alignment["direction"])
        target = outer[(outer.index(start) + direction) % len(outer)]
        route = self.mapped_topology.alternative_route(start, target, require_internal=True, outer_node_ids=outer)
        self.blocked_edge = [start, target]
        if route is None:
            self.traffic_reason = "red_no_alternative_route"
            return
        tracker = DetourTracker(
            route.points,
            waypoint_radius_m=self.p["detour_waypoint_radius_m"],
            max_cross_track_m=self.p["detour_max_cross_track_m"],
            lookahead_m=self.p["detour_lookahead_m"], spacing_m=self.p["detour_point_spacing_m"],
            entry_radius_m=self.p["detour_entry_radius_m"],
        )
        initial = tracker.update(*self.odom)
        if not initial.available:
            self.traffic_reason = "red_route_unusable_" + initial.reason
            return
        self.detour_tracker = tracker
        self.detour_node_ids = route.node_ids
        target_progress = self._node_progress(target)
        target_progress += math.floor(self.progress / self.reference.length_m) * self.reference.length_m
        if target_progress < self.progress:
            target_progress += self.reference.length_m
        self.detour_exit_progress = target_progress
        self.detour_used_odom_seq = -1
        self.detour_motion_ready = False
        self.visual_motion_ready = False
        self.last_cmd_time = -math.inf
        self.traffic_mode = "detour"
        self.traffic_reason = "following_alternative_route"
        self.guard.reset()
        self.last_correction = 0.0

    def _process_detour(self, now: float) -> None:
        if self._output_block_reason(now):
            self.detour_motion_ready = False
            return
        if self.odom is None or now - self.odom_time > self.p["odom_timeout_sec"]:
            self.detour_motion_ready = False
            self.reason = "detour_odom_stale"
            return
        if self.odom_seq == self.detour_used_odom_seq:
            return
        self.detour_used_odom_seq = self.odom_seq
        result = self.detour_tracker.update(*self.odom)
        self.detour_motion_ready = result.available
        self.reason = self.traffic_reason = "detour_" + result.reason
        if result.done:
            # Seed association near the known exit, then let fresh vision resume.
            # Never globally jump to a nearby crossing on the recorded outer lap.
            self.progress = self.detour_exit_progress
            self.pending_distance = 0.0
            self.matched = False
            # No lamp IDs or permanent handled-red latch: reconsider only the
            # current fresh vote, including a red that appeared on the detour.
            self._resume_vision("detour_complete_resume_vision")
            self.traffic_light = self.light_filter.evaluate(now)
            if self.traffic_light == "red":
                self.traffic_mode = "wait"
                self.traffic_reason = "red_after_detour"
            return
        if result.available:
            for label, (x, y) in zip("abc", result.points):
                self.point_pubs[label].publish(Pose2D(x=float(x), y=float(y), theta=0.0))

    def _on_branch(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active != self.branch_active or active:
            self.guard.reset()
            self.last_correction = 0.0
        self.branch_active = active
        if active:
            self.excluded_since_sample = True
            self.reason = "branch_active"

    def _point_cb(self, label: str):
        def callback(msg: Pose2D) -> None:
            self.points[label] = msg
            self.point_times[label] = time.monotonic()
            self.point_seq[label] += 1
            if not all(math.isfinite(value) for value in (msg.x, msg.y, msg.theta)):
                self.visual_motion_ready = False
                if self.avoidance_active is False and self.traffic_mode in ("normal", "approach"):
                    self.last_cmd_time = -math.inf
        return callback

    def _output_block_reason(self, now: float) -> Optional[str]:
        if self.avoidance_active is None:
            return "avoidance_state_unknown"
        if self.avoidance_active is True:
            return "avoidance_active"
        if now < self.resume_after:
            return "resume_delay"
        return None

    def _bypass_reason(self, now: float) -> Optional[str]:
        blocked = self._output_block_reason(now)
        if blocked:
            return blocked
        if self.branch_active:
            return "branch_active"
        if self.p["enable_traffic_control"] and self.traffic_mode not in ("normal", "approach"):
            return "traffic_" + self.traffic_mode
        return None

    def _fresh_points(self, now: float) -> bool:
        return all(self.points[k] is not None and now - self.point_times[k] <= self.p["point_timeout_sec"] for k in "abc")

    def _recording_points_valid(self, now: float) -> bool:
        if not self._fresh_points(now):
            return False
        if max(self.point_times.values()) - min(self.point_times.values()) > self.p["max_point_skew_sec"]:
            return False
        return all(
            point.x > 0.0 and all(math.isfinite(v) for v in (point.x, point.y, point.theta))
            for point in self.points.values()
        )

    def _on_odom(self, msg: Odometry) -> None:
        now = time.monotonic()
        pos, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (pos.x, pos.y, q.x, q.y, q.z, q.w)
        norm = math.hypot(*values[2:])
        if not all(math.isfinite(v) for v in values) or not math.isfinite(norm) or norm < 1e-6:
            self.matched = False
            self.odom_time = -math.inf
            self.excluded_since_sample = True
            self.guard.reset()
            self.reason = "invalid_odom"
            return
        covariance = getattr(msg.pose, "covariance", [])
        variances = [float(covariance[i]) for i in (0, 7, 35)] if len(covariance) == 36 else [0.0] * 3
        self.odom_covariance_available = all(v > 0.0 and math.isfinite(v) for v in variances)
        if (any(not math.isfinite(v) or v < 0.0 for v in variances)
                or max(variances[:2]) > self.p["max_odom_position_std_m"] ** 2
                or variances[2] > math.radians(self.p["max_odom_yaw_std_deg"]) ** 2):
            self.matched = False
            self.odom_time = -math.inf
            self.excluded_since_sample = True
            self.guard.reset()
            self.reason = "odom_uncertainty_high"
            return
        qx, qy, qz, qw = (v / norm for v in values[2:])
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        current = (float(pos.x), float(pos.y), yaw)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame = msg.header.frame_id or "odom"
        delta = 0.0
        if self.odom is not None:
            delta = math.hypot(current[0] - self.odom[0], current[1] - self.odom[1])
            dt = now - self.odom_time
            reset_reason = None
            if stamp > 0 and self.odom_stamp is not None and self.odom_stamp > 0:
                if stamp < self.odom_stamp - self.p["max_odom_gap_sec"]:
                    reset_reason = "odom_time_reset"
                elif stamp <= self.odom_stamp:
                    self.reason = "out_of_order_odom"
                    self.matched = False
                    self.excluded_since_sample = True
                    self.guard.reset()
                    return
                dt = stamp - self.odom_stamp
            if frame != self.odom_frame or msg.child_frame_id != self.child_frame:
                reset_reason = "odom_frame_changed"
            elif not math.isfinite(dt) or dt > self.p["max_odom_gap_sec"]:
                reset_reason = "odom_gap_reset"
            elif delta > self.p["max_odom_speed_mps"] * max(dt, 0.02) + 0.2:
                reset_reason = "odom_jump_reset"
            elif abs(wrap_angle(yaw - self.odom[2])) > self.p["max_odom_yaw_rate_radps"] * max(dt, 0.02) + 0.15:
                reset_reason = "odom_heading_jump_reset"
            if reset_reason:
                self._reset_learning(reset_reason)
                self._publish_path()
                delta = 0.0
        self.odom, self.odom_time, self.odom_stamp = current, now, stamp
        twist = getattr(getattr(msg, "twist", None), "twist", None)
        speed = getattr(getattr(twist, "linear", None), "x", None)
        if isinstance(speed, (int, float)) and not isinstance(speed, bool) and math.isfinite(speed):
            self.odom_speed_mps, self.odom_speed_time = float(speed), now
            self.odom_speed_history.append((now, float(speed)))
            oldest = now - self.p["signal_observation_timeout_sec"]
            while len(self.odom_speed_history) > 1 and self.odom_speed_history[1][0] < oldest:
                self.odom_speed_history.popleft()
        else:
            self.odom_speed_mps = None
        self.odom_seq += 1
        self.odom_frame, self.child_frame = frame, msg.child_frame_id
        excluded = self._bypass_reason(now) is not None or not self._recording_points_valid(now)
        self.excluded_since_sample |= excluded
        if self.reference is None:
            if not self.samples:
                self.samples.append(RouteSample(*current, excluded=excluded))
            else:
                start = self.samples[0]
                if math.hypot(current[0] - start.x, current[1] - start.y) > max(3.0, 2.0 * self.p["lap_closure_radius_m"]):
                    self.left_start_area = True
                previous = self.samples[-1]
                sampled_distance = math.hypot(current[0] - previous.x, current[1] - previous.y)
                if sampled_distance >= self.p["sample_spacing_m"]:
                    if len(self.samples) >= self.p["max_recorded_samples"]:
                        self.reason = "recording_limit_reached_reset_required"
                        return
                    self.samples.append(RouteSample(*current, excluded=self.excluded_since_sample))
                    self.recorded_distance += sampled_distance
                    self.excluded_since_sample = excluded
            self._try_complete_lap(current, excluded)
        elif self.traffic_mode != "detour":
            # Detour distance need not equal progress along the reference.
            # Keep the local search centred near the last match; repeated local
            # matching advances it as the car moves, without banking a long
            # unmatched detour as an artificial jump along the reference.
            self.pending_distance = min(self.pending_distance + delta, self.p["match_window_m"] * 0.5)
            result = self.reference.match(
                current[0], current[1], self.progress, self.pending_distance,
                self.p["match_window_m"], self.p["max_match_distance_m"],
            )
            self.matched = result is not None
            if result is not None:
                self.progress = result
                self.pending_distance = 0.0

    def _try_complete_lap(self, current, excluded: bool) -> None:
        # The official drawing does not define odometric metres or lap length.
        minimum = self.p["min_lap_distance_m"]
        if not self.left_start_area or excluded:
            return
        first = self.samples[0]
        if math.hypot(current[0] - first.x, current[1] - first.y) > self.p["lap_closure_radius_m"]:
            return
        if abs(wrap_angle(current[2] - first.yaw)) > math.radians(self.p["lap_closure_heading_deg"]):
            return
        if len(self.samples) < 30 or self.recorded_distance < minimum:
            self._reset_learning("returned_before_min_lap_distance")
            self.samples.append(RouteSample(*current, excluded=excluded))
            return
        candidate_samples = self.samples + [RouteSample(*current, excluded=excluded)]
        try:
            fusion = build_fused_route(
                candidate_samples, self.official, spacing_m=self.p["sample_spacing_m"],
                smooth_window_m=self.p["smooth_window_m"], max_detour_gap_m=self.p["max_detour_gap_m"],
                max_rmse_m=self.p["alignment_max_rmse_m"],
                ambiguity_margin_m=self.p["alignment_ambiguity_margin_m"],
                node_search_m=self.p["fusion_node_search_m"],
                feature_window_m=self.p["fusion_feature_window_m"],
                shape_weight=self.p["fusion_shape_weight"],
                max_displacement_m=self.p["fusion_max_displacement_m"],
            )
        except ValueError as exc:
            self.reason = "reference_build_failed"
            self.get_logger().warn(str(exc))
            return
        self.reference = fusion.reference
        self.fusion_status = dict(fusion.status)
        self.fused_node_positions = dict(fusion.node_positions)
        self.fused_node_progress = dict(fusion.node_progress)
        self.topology_alignment = fusion.alignment
        # Retain the old diagnostic field for readers; it is not the fusion result.
        self.alignment = {"diagnostic_only": True, "superseded_by": "map_fusion"}
        self.recorded_distance += math.hypot(current[0] - self.samples[-1].x, current[1] - self.samples[-1].y)
        self.progress = 0.0
        self.pending_distance = 0.0
        self.matched = False  # First post-closure odometry confirms association.
        self.reason = "first_lap_complete" if self.fusion_status.get("available") else "first_lap_fusion_unavailable"
        self._prepare_mapped_topology()
        self._publish_path()
        self.get_logger().info(
            f"First lap complete: {self.reference.length_m:.1f} odom m; "
            f"fusion={self.fusion_status.get('available')}, reason={self.fusion_status.get('reason')}."
        )

    def _tick(self) -> None:
        now = time.monotonic()
        self._update_light_vote(now)
        self._update_traffic(now)
        blocked = self._output_block_reason(now)
        if blocked:
            self.guard.reset()
            self.last_correction = 0.0
            self.reason = blocked
            self._publish_housekeeping(now)
            return
        if self.p["enable_traffic_control"] and self.traffic_mode == "wait":
            self.reason = self.traffic_reason
            self.guard.reset()
            self.last_correction = 0.0
            self._publish_housekeeping(now)
            return
        if self.traffic_mode == "detour":
            self._process_detour(now)
            self._publish_housekeeping(now)
            return
        bypass = self._bypass_reason(now)
        if bypass:
            self.guard.reset()
            self.last_correction = 0.0
            self.reason = bypass
        # Publish only a genuinely new, complete triplet. Pose2D has no stamp;
        # replaying cached points would defeat a downstream input watchdog.
        if not self._fresh_points(now):
            self.reason = "visual_points_stale"
            self.guard.reset()
            self.last_correction = 0.0
        elif all(self.point_seq[k] > self.used_seq[k] for k in "abc"):
            times = list(self.point_times.values())
            if max(times) - min(times) > self.p["max_point_skew_sec"]:
                self.reason = "visual_triplet_skew"
                self.guard.reset()
                self.visual_motion_ready = False
                self.last_cmd_time = -math.inf
            else:
                self.used_seq = dict(self.point_seq)
                self._process_triplet(now)
        self._publish_housekeeping(now)

    def _publish_housekeeping(self, now: float) -> None:
        self._publish_velocity_stop_if_needed(now)

    def _process_triplet(self, now: float) -> None:
        # Check again so direct callback entry cannot bypass avoidance silence.
        blocked = self._output_block_reason(now)
        if blocked:
            self.guard.reset()
            self.last_correction = 0.0
            self.reason = blocked
            return
        if self.p["enable_traffic_control"] and self.traffic_mode not in ("normal", "approach"):
            return
        points = [self.points[k] for k in "abc"]
        correction = 0.0
        guard_called = False
        if any(point is None for point in points) or not all(
            math.isfinite(v) for point in points for v in (point.x, point.y, point.theta)
        ):
            # Stop publishing; the vehicle controller must enforce its timeout.
            self.guard.reset()
            self.reason = "invalid_visual_points"
            self.last_correction = 0.0
            self.visual_motion_ready = False
            self.last_cmd_time = -math.inf
            return
        zero = all(point.x == 0.0 and point.y == 0.0 for point in points)
        if zero and self.p["enable_velocity_gate"]:
            self.visual_motion_ready = False
            self.last_cmd_time = -math.inf
            self.reason = "visual_zero_stop"
            self.guard.reset()
            self.last_correction = 0.0
            return
        bypass = self._bypass_reason(now)
        if zero:
            self.reason = "visual_stop_preserved"
        elif bypass:
            self.reason = bypass
        elif not all(point.x > 0.0 for point in points):
            self.reason = "visual_geometry_not_forward"
        elif self.reference is None:
            self.reason = "learning_first_lap"
        elif not self.fusion_status.get("available"):
            self.reason = "map_fusion_unavailable_passthrough"
        elif self.odom is None or now - self.odom_time > self.p["odom_timeout_sec"]:
            self.reason = "odom_stale_passthrough"
        elif not self.matched:
            self.reason = "route_match_unavailable"
        else:
            # Keep a/b/c order; choose the farthest point only for heading and
            # use a similar preview distance on the recorded route.
            target = max(points, key=lambda p: math.hypot(p.x, p.y))
            lookahead = math.hypot(target.x, target.y)
            visual_angle = math.atan2(target.y, target.x)
            candidates = self.reference.direction_candidates(
                *self.odom, self.progress, lookahead, self.p["progress_tolerance_m"],
            )
            guard_called = True
            correction = self.guard.update(visual_angle, candidates, enabled=bool(self.p["enable_correction"]))
            self.reason = self.guard.reason
        if not guard_called:
            self.guard.reset()
        c, s = math.cos(correction), math.sin(correction)
        rotated = [(c * point.x - s * point.y, s * point.x + c * point.y) for point in points]
        if correction and any(x <= 0 for x, _ in rotated):
            correction = 0.0
            rotated = [(point.x, point.y) for point in points]
            self.reason = "correction_would_point_backward"
            self.guard.reset()
        self.last_correction = correction
        self.visual_motion_ready = True
        for label, point, (x, y) in zip("abc", points, rotated):
            # theta is an upstream contract, not our visual-heading estimate.
            self.point_pubs[label].publish(Pose2D(x=float(x), y=float(y), theta=float(point.theta)))

    def _publish_path(self) -> None:
        msg = Path()
        msg.header.frame_id = self.odom_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.reference is not None:
            points = self.reference.points + [self.reference.points[0]]
            for x, y in points:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)
        self.path_pub.publish(msg)

    def _status_tick(self) -> None:
        # Read-only diagnostics: never publish path points or velocity here.
        self._publish_status_if_due(time.monotonic())

    def _publish_status_if_due(self, now: float) -> None:
        if now - self.last_status_time >= 0.2:
            self._publish_status()
            self.last_status_time = now

    def _publish_status(self) -> None:
        reference = self.reference
        now = time.monotonic()

        def age_seconds(received_at: float) -> Optional[float]:
            return max(0.0, now - received_at) if math.isfinite(received_at) else None

        payload = {
            "phase": ("guarding" if self.fusion_status.get("available") else "odom_only") if reference is not None else "learning",
            "reason": self.reason,
            "lap": 2 + max(0, int(self.progress / reference.length_m)) if reference is not None else 1,
            "recorded_samples": len(self.samples), "recorded_distance_m": self.recorded_distance,
            "reference_length_m": reference.length_m if reference is not None else None,
            "route_matched": self.matched, "progress_m": self.progress,
            "use_sim_time": bool(self.get_parameter("use_sim_time").value),
            "ros_time_sec": self.get_clock().now().nanoseconds * 1e-9,
            "odom_age_sec": age_seconds(self.odom_time),
            "point_age_sec": {label: age_seconds(self.point_times[label]) for label in "abc"},
            "recording_points_valid": self._recording_points_valid(now),
            "odom_covariance_available": self.odom_covariance_available,
            "correction_deg": math.degrees(self.last_correction),
            "violation_frames": self.guard.violation_count,
            "avoidance_active": self.avoidance_active, "branch_active": self.branch_active,
            "path_output_allowed": self._output_block_reason(now) is None and self.traffic_mode != "wait",
            "enable_correction": self.p["enable_correction"],
            "official_route_node_ids": self.official.node_ids,
            "approximate_official_edge": self._current_topology_edge(),
            "official_alignment_diagnostic_only": self.alignment,
            "map_fusion": self.fusion_status,
            "official_node_positions_odom": self.fused_node_positions,
            "official_node_progress_m": self.fused_node_progress,
            "topology_mapping": self.topology_mapping_status,
            "traffic_light_raw": self.traffic_light_raw,
            "traffic_light": self.light_filter.evaluate(now),
            "traffic_vote": self.light_filter.snapshot(now),
            "traffic_mode": self.traffic_mode,
            "traffic_reason": self.traffic_reason,
            "signal_approach": {
                "enabled": self.p["enable_signal_approach"],
                "target_stop_distance_m": self.p["target_stop_distance_m"],
                "observation_sequence": self.signal_observation_sequence,
                "wait_latched": self.signal_wait_latched,
                "distance_m": self.signal_observation.distance_m if self.signal_observation else None,
                "uncertainty_m": self.signal_observation.uncertainty_m if self.signal_observation else None,
                "profile": self._signal_approach_profile(now),
            },
            "topology_alignment": self.topology_alignment,
            "blocked_edge": self.blocked_edge,
            "detour_node_ids": self.detour_node_ids,
            "velocity_gate_reason": self.velocity_gate_reason if self.p["enable_velocity_gate"] else "disabled",
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))


def main(args=None) -> None:
    rclpy.init(args=startup_args(args))
    node = MapAssist()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
