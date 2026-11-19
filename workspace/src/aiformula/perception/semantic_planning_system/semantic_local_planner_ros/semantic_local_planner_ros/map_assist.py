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
from concurrent.futures import ThreadPoolExecutor
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
from semantic_planner_core.route_fusion import build_fused_route, build_odom_reference
from semantic_planner_core.anchored_topology import anchored_graph
from semantic_planner_core.traffic_route import OfficialToOdom, TopologyGraph
from semantic_planner_core.traffic_light_filter import NON_RED_STATES, TrafficLightFilter
from semantic_planner_core.smooth_detour import SmoothDetourTracker, build_smooth_detour


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
            "max_closure_gap_m": 4.0,
            "min_lap_distance_m": 40.0,
            "lap_closure_radius_m": 2.5,
            "lap_closure_heading_deg": 25.0,
            "match_window_m": 3.0,
            "max_match_distance_m": 2.5,
            "match_heading_limit_deg": 75.0,
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
            "traffic_stop_flag_topic": "/flag_stop",
            "traffic_stop_flag_timeout_sec": 0.0,
            "traffic_vote_window_sec": 0.5,
            "traffic_vote_min_samples": 3,
            "traffic_red_rearm_sec": 2.0,
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
            "fusion_candidate_tolerance_m": 1.0,
            "branch_angle_min_deg": 80.0,
            "branch_angle_max_deg": 95.0,
            "detour_entry_radius_m": 3.0,
            "detour_waypoint_radius_m": 0.8,
            "detour_max_cross_track_m": 2.0,
            "detour_lookahead_m": 0.5,
            "detour_point_spacing_m": 0.5,
            # Simulation starting value only: replace with the measured vehicle radius.
            "detour_min_turn_radius_m": 4.0,
            # Start before the candidate reaches the hard minimum radius. The
            # forecast is only a short, straight-ahead pose estimate.
            "detour_trigger_margin_m": 0.5,
            "detour_trigger_preview_m": 1.0,
            "detour_max_trim_m": 12.0,
            "detour_curve_spacing_m": 0.15,
            "detour_target_angle_deg": 65.0,
            "detour_plan_update_distance_m": 1.5,
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
        self.green_on_last_frame = False
        self.green_frames_since_red = 0
        self.traffic_light_raw = "unknown"
        self.traffic_light = "unknown"
        self.traffic_mode = "normal"
        self.traffic_reason = "traffic_idle"
        # None means no message has ever arrived: it is not the same as False.
        self.stop_flag = None
        self.stop_committed = False
        self.traffic_event_consumed = False
        self.red_preview_unconfirmed = False
        self.red_last_confirmed_at = -math.inf
        self.last_red_observed_at = -math.inf
        self.detour_green_seen = False
        self.detour_signal_rearmed = False
        self.detour_next_red_pending = False
        self.auto_trigger_reason = "not_evaluated"
        self.auto_current_radius_m = None
        self.auto_predicted_radius_m = None
        self.auto_stop_latched = False
        self.smooth_path = None
        self.smooth_plan_reason = "not_planned"
        self.smooth_plan_pose = None
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
        self.detour_entry_progress = None
        self.detour_exit_progress = 0.0
        self.detour_used_odom_seq = -1
        self.detour_motion_ready = False
        self.visual_motion_ready = False
        self.last_cmd_time = -math.inf
        self.velocity_gate_reason = "waiting_controller_command"
        self._fusion_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="map_fusion")
        self._fusion_future = None
        self._pending_fusion = None
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
        self.detour_path_pub = self.create_publisher(
            Path, "/map_assist/detour_path",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.cmd_pub = None
        if self.p["enable_velocity_gate"]:
            self.cmd_pub = self.create_publisher(Twist, self.p["output_cmd_vel_topic"], 10)
            self.create_subscription(Twist, self.p["input_cmd_vel_topic"], self._on_controller_cmd, 10)
        if self.p["enable_traffic_control"]:
            self.create_subscription(String, self.p["traffic_light_topic"], self._on_light_state, 10)
            self.create_subscription(Bool, self.p["traffic_stop_flag_topic"], self._on_traffic_stop_flag, 10)
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
        lower, upper = self.p["branch_angle_min_deg"], self.p["branch_angle_max_deg"]
        if (isinstance(lower, bool) or isinstance(upper, bool)
                or not math.isfinite(lower) or not math.isfinite(upper)
                or not 0.0 <= lower <= upper <= 180.0):
            raise ValueError("Branch-angle limits must satisfy 0 <= min <= max <= 180")
        for name in (
            "sample_spacing_m", "min_lap_distance_m",
            "lap_closure_radius_m", "match_window_m", "max_match_distance_m",
            "max_odom_speed_mps", "max_odom_yaw_rate_radps", "max_odom_position_std_m",
            "max_odom_yaw_std_deg", "max_odom_gap_sec", "publish_rate_hz",
            "point_timeout_sec", "max_point_skew_sec", "odom_timeout_sec",
            "cmd_vel_timeout_sec", "alignment_max_rmse_m",
            "fusion_node_search_m", "fusion_feature_window_m", "fusion_shape_weight",
            "fusion_max_displacement_m", "fusion_candidate_tolerance_m",
            "detour_entry_radius_m", "detour_waypoint_radius_m",
            "detour_max_cross_track_m", "detour_lookahead_m", "detour_point_spacing_m",
            "detour_min_turn_radius_m", "detour_trigger_preview_m", "detour_max_trim_m", "detour_curve_spacing_m",
            "detour_plan_update_distance_m",
            "traffic_red_rearm_sec",
        ):
            if not math.isfinite(float(self.p[name])) or self.p[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("smooth_window_m", "max_detour_gap_m", "max_closure_gap_m", "progress_tolerance_m", "resume_delay_sec", "detour_trigger_margin_m",
                     "traffic_stop_flag_timeout_sec"):
            if not math.isfinite(float(self.p[name])) or self.p[name] < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if (not math.isfinite(self.p["alignment_ambiguity_margin_m"])
                or self.p["alignment_ambiguity_margin_m"] <= 0):
            raise ValueError("Invalid topology alignment quality limits")
        if self.p["fusion_shape_weight"] >= 1.0:
            raise ValueError("fusion_shape_weight must be in (0, 1)")
        if self.p["enable_traffic_control"] and not self.p["enable_velocity_gate"]:
            raise ValueError("Traffic waiting requires the velocity gate; zero path points cannot stop LYA")
        if self.p["input_cmd_vel_topic"] == self.p["output_cmd_vel_topic"]:
            raise ValueError("Velocity input/output must differ; feedback is not allowed")
        if self.p["enable_traffic_control"] and self.p["traffic_stop_flag_topic"] in (
                self.p["avoidance_flag_topic"], self.p["branch_active_topic"]):
            raise ValueError("Traffic stop flag must be a separate topic from OA and branch flags")
        for name in ("angle_tolerance_deg", "max_correction_deg", "max_step_deg", "lap_closure_heading_deg", "match_heading_limit_deg", "detour_target_angle_deg"):
            if not math.isfinite(float(self.p[name])) or not 0 < self.p[name] < 90:
                raise ValueError(f"{name} must be between 0 and 90 degrees")
        if self.p["persistence_frames"] < 1 or self.p["max_recorded_samples"] < 30:
            raise ValueError("Invalid frame/sample count")
        inputs = {self.p[f"input_point_{label}_topic"] for label in "abc"}
        outputs = {self.p[f"output_point_{label}_topic"] for label in "abc"}
        if len(inputs) != 3 or len(outputs) != 3 or inputs & outputs:
            raise ValueError("Input/output point topics must be distinct; feedback is not allowed")

    def _reset_learning(self, reason: str) -> None:
        if self._fusion_future is not None:
            self._fusion_future.cancel()
        self._fusion_future = None
        self._pending_fusion = None
        was_detouring = self.traffic_mode in ("prepare", "detour")
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
        self.detour_graph_points = []
        self.smooth_path = None
        self.smooth_plan_pose = None
        self.smooth_plan_reason = "not_planned"
        # A completed bypass must not suppress a red event after map reset.
        self.traffic_event_consumed = False
        self.red_preview_unconfirmed = False
        self.red_last_confirmed_at = -math.inf
        self.last_red_observed_at = -math.inf
        self.detour_green_seen = False
        self.detour_signal_rearmed = False
        self.detour_next_red_pending = False
        self.auto_trigger_reason = "not_evaluated"
        self.auto_current_radius_m = None
        self.auto_predicted_radius_m = None
        self.auto_stop_latched = False
        if hasattr(self, "detour_path_pub"):
            self._publish_detour_path()
        self.blocked_edge = None
        self.detour_entry_progress = None
        self.detour_motion_ready = False
        self.visual_motion_ready = False
        self.last_cmd_time = -math.inf
        if self.p["enable_traffic_control"]:
            if was_detouring and self._confirmed_green():
                self._resume_vision("map_reset_resume_vision")
            elif was_detouring or (self.traffic_light == "red" and self.stop_flag is not False):
                self.traffic_mode = "wait"
                self.traffic_reason = "map_reset_wait"
                self.stop_committed = True
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
        state = str(msg.data).strip().lower()
        if state not in ("red", "unknown", *NON_RED_STATES):
            state = "unknown"
        self.traffic_light_raw = state
        if not self.p["enable_traffic_control"]:
            return
        now = time.monotonic()
        if self.traffic_light_raw == "red":
            # With no light ID, an extended period without red can re-arm a
            # later event. A short detector flicker cannot do so.
            rearmed = (math.isfinite(self.last_red_observed_at)
                       and now - self.last_red_observed_at >= self.p["traffic_red_rearm_sec"])
            if rearmed and self.traffic_mode == "detour":
                self.detour_signal_rearmed = True
            elif rearmed and self.traffic_mode == "normal":
                self.traffic_event_consumed = False
            self.last_red_observed_at = now
        # A single red observation may start a preview after the first lap,
        # but can never by itself commit a turn or release a stop.
        if (self.reference is not None and self.traffic_light_raw == "red"
                and self.traffic_mode == "normal" and not self.traffic_event_consumed):
            self.traffic_mode = "prepare"
            self.traffic_reason = "red_preview_unconfirmed"
            self.red_preview_unconfirmed = True
        self._update_light_vote(now, received_frame=True)
        self._publish_velocity_stop_if_needed(now)

    @property
    def stop_flag(self):
        timeout = self.p["traffic_stop_flag_timeout_sec"]
        if timeout and time.monotonic() - self._stop_flag_time > timeout:
            return None
        return self._stop_flag

    @stop_flag.setter
    def stop_flag(self, value):
        self._stop_flag = value
        self._stop_flag_time = time.monotonic()

    def _on_traffic_stop_flag(self, msg: Bool) -> None:
        self.stop_flag = bool(msg.data)
        # After a lap is recorded, the distance flag is only a fallback stop;
        # it never triggers a turn or re-arms a light event.
        if self.reference is None and not self.stop_flag:
            self.traffic_event_consumed = False
        now = time.monotonic()
        self._update_light_vote(now)
        if self.stop_flag and self.reference is not None and self.traffic_mode in ("prepare", "wait"):
            # Apply the fallback without waiting for the next control timer.
            # The same pose/route/light checks and OA priority still apply.
            self._update_traffic(now)
        self._publish_velocity_stop_if_needed(now)

    def _confirmed_green(self) -> bool:
        # A vote can change between frames as older observations expire.
        # Release requires a green majority on the incoming green frame itself.
        return self.green_on_last_frame and self.traffic_light == "green"

    def _clear_light_confirmation(self) -> None:
        self.light_filter.clear()
        self.traffic_light = "unknown"
        self.green_on_last_frame = False
        self.green_frames_since_red = 0
        self.red_last_confirmed_at = -math.inf

    def _update_light_vote(self, now: float, *, received_frame: bool = False) -> None:
        if not self.p["enable_traffic_control"]:
            return
        raw = self.traffic_light_raw
        watching_red = self.traffic_mode in ("prepare", "wait") or self.detour_next_red_pending
        if (watching_red and math.isfinite(self.red_last_confirmed_at)
                and (raw not in ("red", "green")
                     or now - self.red_last_confirmed_at > self.p["traffic_vote_window_sec"])):
            # A missing lamp also produces zero red pixels. Hold before the pixel
            # stop point, and require a new vote before committing a red detour.
            if self.traffic_mode != "detour":
                self._auto_red_hold("red_wait_signal_lost")
            self._clear_light_confirmation()
        if received_frame:
            self.traffic_light = self.light_filter.add(raw, now)
            self.green_on_last_frame = False
            if raw == "green":
                self.green_frames_since_red += 1
                if self.traffic_light == "green":
                    # Green frames preceding the latest red belong to the previous signal state.
                    count = self.light_filter.snapshot(now)["sample_count"]
                    self.green_on_last_frame = 2 * self.green_frames_since_red > count
            elif raw == "red":
                self.green_frames_since_red = 0
                if self.traffic_light == "red":
                    self.red_last_confirmed_at = now
        else:
            self.traffic_light = self.light_filter.evaluate(now)
        green = self._confirmed_green()
        red = self.traffic_light == "red" and self._red_recently_confirmed(now)
        if self.traffic_mode == "detour":
            if green:
                self.detour_green_seen = True
                self.detour_next_red_pending = False
            elif (self.detour_green_seen or self.detour_signal_rearmed) and red:
                self.detour_next_red_pending = True
        if green:
            self.traffic_event_consumed = False
        elif red and self.traffic_mode == "prepare":
            self.red_preview_unconfirmed = False
        if red and self.traffic_mode == "normal" and not self.traffic_event_consumed:
            self.traffic_mode = "prepare"
            self.traffic_reason = "red_prepare"
        elif self.traffic_mode in ("prepare", "wait") and green:
            self._resume_vision("green_resume")
        if self.reference is not None:
            return
        # On the first lap the distance flag marks the usual red stop position.
        if (self.traffic_mode == "wait" and self.traffic_reason == "red_wait_flag_unavailable"
                and not self.stop_committed and self.stop_flag is False
                and self._red_recently_confirmed(now)):
            self.traffic_mode = "prepare"
            self.traffic_reason = "red_prepare"
        if self.traffic_mode == "prepare" and self.stop_flag is None:
            self.traffic_mode = "wait"
            self.traffic_reason = "red_wait_flag_unavailable"
            self.visual_motion_ready = False
            if self.avoidance_active is not True:
                self.last_cmd_time = -math.inf
        elif self.traffic_mode in ("prepare", "wait") and self.stop_flag is True:
            self.traffic_mode = "wait"
            self.stop_committed = True
            self.traffic_reason = "red_wait_first_lap"
            self.visual_motion_ready = False
            if self.avoidance_active is not True:
                self.last_cmd_time = -math.inf

    def _resume_vision(self, reason: str) -> None:
        self.traffic_mode = "normal"
        self.traffic_reason = reason
        self.stop_committed = False
        self.red_preview_unconfirmed = False
        self.red_last_confirmed_at = -math.inf
        self.detour_green_seen = False
        self.detour_signal_rearmed = False
        self.detour_next_red_pending = False
        self.auto_trigger_reason = "not_evaluated"
        self.auto_current_radius_m = None
        self.auto_predicted_radius_m = None
        self.auto_stop_latched = False
        self.detour_tracker = None
        self.detour_node_ids = []
        self.detour_graph_points = []
        self.blocked_edge = None
        self.detour_entry_progress = None
        self.smooth_path = None
        self.smooth_plan_pose = None
        self.smooth_plan_reason = "not_planned"
        if hasattr(self, "detour_path_pub"):
            self._publish_detour_path()
        self.detour_motion_ready = False
        self.visual_motion_ready = False
        self.used_seq = dict(self.point_seq)
        self.resume_after = time.monotonic() + self.p["resume_delay_sec"]
        if self.avoidance_active is not True:
            self.last_cmd_time = -math.inf
        self.guard.reset()
        self.last_correction = 0.0

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
        if self.traffic_mode == "detour":
            if not self.detour_motion_ready or now - self.odom_time > self.p["odom_timeout_sec"]:
                return "route_waiting_valid_odom_and_points"
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
            # Reject invalid inputs, but do not insert commands while OA owns
            # the manoeuvre. OA/downstream must enforce their own watchdog.
            if self.avoidance_active is not True:
                self.cmd_pub.publish(Twist())
            return
        held = self._velocity_hold_reason(now)
        if held:
            self.last_cmd_time = -math.inf
            self.velocity_gate_reason = held
            self.cmd_pub.publish(Twist())
            return
        self.last_cmd_time = now
        self.velocity_gate_reason = "forwarding_controller"
        self.cmd_pub.publish(msg)

    def _publish_velocity_stop_if_needed(self, now: float) -> None:
        if not self.p["enable_velocity_gate"] or self.cmd_pub is None:
            return
        # Do not inject a timeout stop over OA while it owns the manoeuvre.
        # Fresh OA-derived controller commands still pass through the gate.
        if self.avoidance_active is True:
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

    def _select_red_route(self) -> bool:
        """Select the next forward outer junction once per red event."""
        outer = self.official.node_ids[:-1]
        length = self.reference.length_m
        candidates = []
        for key in outer:
            forward = (self._node_progress(key) - self.progress) % length
            if (forward > length - self.p["detour_waypoint_radius_m"]
                    and math.dist(self.odom[:2], self.fused_node_positions[key])
                    <= self.p["detour_waypoint_radius_m"]):
                forward = 0.0
            candidates.append((forward, key))
        if not candidates:
            self.smooth_plan_reason = "red_no_outer_junction"
            return False
        forward, start = min(candidates)
        entry_progress = self.progress + forward
        direction = int(self.topology_alignment["direction"])
        target = outer[(outer.index(start) + direction) % len(outer)]
        try:
            route = self.mapped_topology.alternative_route(
                start, target, require_internal=True, outer_node_ids=outer,
                require_first_internal=True)
        except (ValueError, KeyError) as exc:
            self.smooth_plan_reason = "red_graph_error_" + str(exc)
            return False
        if route is None:
            self.smooth_plan_reason = "red_no_alternative_route"
            return False
        self.blocked_edge = [start, target]
        self.detour_node_ids = list(route.node_ids)
        self.detour_graph_points = list(route.points)
        self.detour_entry_progress = entry_progress
        self.detour_exit_progress = self._node_progress(target)
        self.detour_exit_progress += math.floor(entry_progress / length) * length
        if self.detour_exit_progress < entry_progress:
            self.detour_exit_progress += length
        return True

    def _make_smooth_plan(self, force: bool = False) -> bool:
        """Recompute from a live pose, never execute an obsolete candidate."""
        if not self.detour_graph_points and not self._select_red_route():
            return False
        remaining = self.detour_entry_progress - self.progress
        if remaining < -0.5:
            self.smooth_path = None
            self.smooth_plan_reason = "junction_already_passed"
            self._publish_detour_path()
            return False
        if remaining > 0.05 and not self.reference._interval_trusted(self.progress, self.detour_entry_progress):
            self.smooth_path = None
            self.smooth_plan_reason = "untrusted_entry_reference"
            self._publish_detour_path()
            return False
        if self.smooth_plan_pose is not None:
            moved = math.dist(self.odom[:2], self.smooth_plan_pose[:2])
            turned = abs(wrap_angle(self.odom[2] - self.smooth_plan_pose[2]))
            threshold = 0.05 if force else self.p["detour_plan_update_distance_m"]
            if moved < threshold and turned < math.radians(3.0):
                return self.smooth_path is not None
        self.smooth_plan_pose = tuple(self.odom)
        self.smooth_path = None
        try:
            self.smooth_path = self._smooth_plan_at(self.odom)
        except ValueError as exc:
            self.smooth_plan_reason = str(exc)
            self._publish_detour_path()
            return False
        self.smooth_plan_reason = "ready"
        self._publish_detour_path()
        return True

    def _smooth_plan_at(self, pose):
        return build_smooth_detour(
            self.reference, self.detour_graph_points, self.detour_exit_progress,
            pose, self.p["detour_min_turn_radius_m"], self.p["detour_max_trim_m"],
            spacing_m=self.p["detour_curve_spacing_m"],
            post_exit_m=max(6.0, self.p["detour_lookahead_m"] +
                            2.0 * self.p["detour_point_spacing_m"] + 3.0))

    @staticmethod
    def _path_radius(path):
        curvature = path.max_curvature
        return 1.0 / curvature if curvature > 1e-9 else None

    def _auto_turn_due(self):
        """Check the current plan and one short straight-ahead prediction.

        The candidate is a geometric preview, not a vehicle-dynamics forecast.
        Never replace the current plan with the predicted one for tracking.
        """
        limit = self.p["detour_min_turn_radius_m"] + self.p["detour_trigger_margin_m"]
        self.auto_current_radius_m = self._path_radius(self.smooth_path)
        self.auto_predicted_radius_m = None
        if self.auto_current_radius_m is not None and self.auto_current_radius_m <= limit:
            self.auto_trigger_reason = "current_radius_near_limit"
            return True
        distance = self.p["detour_trigger_preview_m"]
        if self.detour_entry_progress - self.progress <= distance:
            self.auto_trigger_reason = "entry_within_preview"
            return True
        x, y, yaw = self.odom
        predicted_pose = (x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw)
        try:
            predicted_path = self._smooth_plan_at(predicted_pose)
        except ValueError:
            self.auto_trigger_reason = "predicted_plan_unavailable"
            return True
        self.auto_predicted_radius_m = self._path_radius(predicted_path)
        if self.auto_predicted_radius_m is not None and self.auto_predicted_radius_m <= limit:
            self.auto_trigger_reason = "predicted_radius_near_limit"
            return True
        self.auto_trigger_reason = "radius_has_margin"
        return False

    def _auto_red_hold(self, reason: str, latch: bool = True) -> None:
        self.traffic_mode = "wait"
        self.traffic_reason = reason
        self.auto_stop_latched = self.auto_stop_latched or latch
        self.stop_committed = self.auto_stop_latched
        self.visual_motion_ready = False
        if self.avoidance_active is not True:
            self.last_cmd_time = -math.inf

    def _clear_auto_measurements(self, reason: str) -> None:
        self.auto_trigger_reason = reason
        self.auto_current_radius_m = None
        self.auto_predicted_radius_m = None

    def _auto_unavailable(self, prepare_reason: str, wait_reason: str) -> None:
        """Search while approaching; fall back to the distance stop on red."""
        if self.red_preview_unconfirmed:
            self.traffic_reason = prepare_reason
            return
        if self.stop_flag is None:
            # Without a distance state the requested last stop cannot be located.
            self._auto_red_hold("red_wait_flag_unavailable", latch=False)
        elif self.stop_flag or self.auto_stop_latched:
            self._auto_red_hold(wait_reason)
        else:
            self.traffic_mode = "prepare"
            self.stop_committed = False
            self.traffic_reason = prepare_reason

    def _red_recently_confirmed(self, now: float) -> bool:
        # A full majority must have confirmed red first. Between detector
        # frames the sliding window may momentarily have fewer than three
        # samples; tolerate only a short gap while the latest observation is
        # still fresh and red. A green/unknown raw observation cannot start it.
        if self.traffic_light_raw != "red":
            return False
        last_age = self.light_filter.snapshot(now)["last_message_age_sec"]
        return (last_age is not None and last_age <= self.p["traffic_vote_window_sec"]
                and now - self.red_last_confirmed_at <= self.p["traffic_vote_window_sec"])

    def _update_traffic(self, now: float) -> None:
        if not self.p["enable_traffic_control"] or self.traffic_mode not in ("prepare", "wait"):
            return
        if self.traffic_mode == "wait" and not self._red_recently_confirmed(now):
            return
        if self.reference is None:
            # During the learning lap only, preserve the existing distance stop.
            if self.stop_flag is None:
                self.traffic_mode = "wait"
                self.traffic_reason = "red_wait_flag_unavailable"
                return
            if self.stop_committed:
                self.traffic_mode = "wait"
                self.traffic_reason = "red_wait_first_lap"
            else:
                self.traffic_mode = "prepare"
                self.traffic_reason = "red_prepare_first_lap"
            return
        if self._output_block_reason(now) or self.branch_active:
            self.smooth_plan_reason = "waiting_for_oa_or_branch_release"
            if self.stop_committed:
                self.traffic_reason = "red_wait_maneuver_release"
            return
        if not self.fusion_status.get("available") or self.mapped_topology is None:
            self.smooth_plan_reason = "waiting_for_map_fusion"
            self._clear_auto_measurements("waiting_for_map_fusion")
            self._auto_unavailable("red_prepare_map_fusion", "red_wait_map_fusion")
            return
        if self.odom is None or now - self.odom_time > self.p["odom_timeout_sec"] or not self.matched:
            self.smooth_plan_reason = "waiting_for_localization"
            self._clear_auto_measurements("waiting_for_localization")
            self._auto_unavailable("red_prepare_localization", "red_wait_localization")
            return
        if not self.reference.point_at(self.progress)[2]:
            self.smooth_plan_reason = "untrusted_current_reference"
            self._clear_auto_measurements("untrusted_current_reference")
            self._auto_unavailable("red_prepare_untrusted_reference", "red_wait_untrusted_reference")
            return
        # Recheck near the current pose rather than relying on an old preview.
        ready = self._make_smooth_plan(force=True)
        if not ready:
            self._clear_auto_measurements("current_plan_unavailable")
            self._auto_unavailable("red_prepare_spline_unavailable",
                                   "red_wait_spline_" + self.smooth_plan_reason)
            return
        due = self._auto_turn_due()
        if not due and self.stop_flag is True:
            # A valid detour at the stop position takes priority over the
            # straight-line red fallback after successful map matching.
            due = True
            self.auto_trigger_reason = "stop_point_with_feasible_detour"
        if self.red_preview_unconfirmed:
            self.traffic_reason = "red_preview_spline_ready"
            return
        if not self._red_recently_confirmed(now):
            if due or self.auto_stop_latched:
                self._auto_unavailable("red_prepare_light_confirmation", "red_wait_light_confirmation")
            else:
                self.traffic_reason = "red_prepare_spline_ready"
            return
        if not due:
            if self.auto_stop_latched:
                self._auto_red_hold("red_wait_stop_point_before_turn")
            else:
                self.traffic_mode = "prepare"
                self.stop_committed = False
                self.traffic_reason = "red_prepare_spline_ready"
            return
        # Only the plan recomputed from the present pose may be executed.
        tracker = SmoothDetourTracker(
            self.smooth_path, self.p["detour_max_cross_track_m"],
            self.p["detour_lookahead_m"], self.p["detour_point_spacing_m"],
            self.p["detour_target_angle_deg"])
        initial = tracker.update(*self.odom)
        if not initial.available:
            self._auto_unavailable("red_prepare_spline_" + initial.reason,
                                   "red_wait_spline_" + initial.reason)
            return
        self.detour_tracker = tracker
        self.auto_stop_latched = False
        self.stop_committed = False
        self.traffic_mode = "detour"
        self.detour_green_seen = False
        self.detour_signal_rearmed = False
        self.detour_next_red_pending = False
        self.traffic_reason = "detour_tracking"
        self.detour_used_odom_seq = -1
        self.detour_motion_ready = False
        self.visual_motion_ready = False
        self.last_cmd_time = -math.inf
        self.guard.reset()
        self.last_correction = 0.0
        self._process_detour(now)

    def _publish_detour_path(self) -> None:
        if not hasattr(self, "detour_path_pub"):
            return
        msg = Path()
        msg.header.frame_id = self.odom_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.smooth_path is not None:
            for x, y in self.smooth_path.points:
                point = PoseStamped()
                point.header = msg.header
                point.pose.position.x, point.pose.position.y = float(x), float(y)
                point.pose.orientation.w = 1.0
                msg.poses.append(point)
        self.detour_path_pub.publish(msg)

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
            # The smooth path reconnects after the exit node and tracks the
            # outer reference for another 2 m before handing back to vision.
            # Seed near that actual rejoin, not at the earlier junction node.
            self.progress = self.detour_exit_progress + self.smooth_path.trim_m + 2.0
            self.pending_distance = 0.0
            self.matched = False
            new_red_waiting = self.detour_next_red_pending
            green_at_exit = self._confirmed_green()
            red_at_exit = self._red_recently_confirmed(now)
            red_confirmed_at = self.red_last_confirmed_at
            self._resume_vision("detour_complete_resume_vision")
            # No second-lap distance flag is used to distinguish lights. A
            # green or long absence followed by red can re-arm a new event.
            if new_red_waiting and not green_at_exit:
                self.traffic_event_consumed = False
                if red_at_exit:
                    self.red_last_confirmed_at = red_confirmed_at
                    self.traffic_mode = "prepare"
                    self.traffic_reason = "red_prepare_next_event"
                else:
                    self._auto_red_hold("red_wait_signal_lost")
                    self._clear_light_confirmation()
            else:
                self.traffic_event_consumed = not green_at_exit
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
                if self.avoidance_active is False and self.traffic_mode == "normal":
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
        if self.p["enable_traffic_control"] and self.traffic_mode not in ("normal", "prepare"):
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
        previous_odom = self.odom
        self.odom, self.odom_time, self.odom_stamp = current, now, stamp
        self.odom_seq += 1
        self.odom_frame, self.child_frame = frame, msg.child_frame_id
        # A red wait holds velocity but valid visual/odom samples remain usable.
        # Otherwise the next lap cannot approach across its own red-stop segment.
        excluded = (self.avoidance_active is not False or self.branch_active
                    or (self.p["enable_traffic_control"] and self.traffic_mode == "detour")
                    or not self._recording_points_valid(now))
        self.excluded_since_sample |= excluded
        if self.reference is None:
            if not self.samples:
                # The start is an actual usable observation, not a pose seen
                # while the camera/OA state is still unavailable. Otherwise
                # the first lap seam would be inseparable from an exclusion.
                if excluded:
                    self.reason = "waiting_valid_mapping_inputs"
                    return
                self.samples.append(RouteSample(*current, excluded=False))
                self.excluded_since_sample = False
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
            self._try_complete_lap(current, excluded, previous_odom)
        elif self.traffic_mode != "detour":
            # Detour distance need not equal progress along the reference.
            # Keep the local search centred near the last match; repeated local
            # matching advances it as the car moves, without banking a long
            # unmatched detour as an artificial jump along the reference.
            self.pending_distance = min(self.pending_distance + delta, self.p["match_window_m"] * 0.5)
            result = self.reference.match(
                current[0], current[1], self.progress, self.pending_distance,
                self.p["match_window_m"], self.p["max_match_distance_m"],
                yaw=current[2], max_heading_error_rad=math.radians(self.p["match_heading_limit_deg"]),
            )
            self.matched = result is not None
            if result is not None:
                self.progress = result
                self.pending_distance = 0.0

    def _try_complete_lap(self, current, excluded: bool, previous_odom=None) -> None:
        # The official drawing does not define odometric metres or lap length.
        minimum = self.p["min_lap_distance_m"]
        if not self.left_start_area or excluded:
            return
        first = self.samples[0]
        if abs(wrap_angle(current[2] - first.yaw)) > math.radians(self.p["lap_closure_heading_deg"]):
            return
        # Cross the start-normal plane in the original travel direction. This
        # prevents the larger radius from closing as soon as the car enters it.
        # Check the interpolated crossing, not only the later odom sample: a
        # low sampling rate can put that sample outside the closure radius even
        # though the segment crossed the plane at the correct start position.
        if previous_odom is None:
            return
        forward_x, forward_y = math.cos(first.yaw), math.sin(first.yaw)
        before = (previous_odom[0] - first.x) * forward_x + (previous_odom[1] - first.y) * forward_y
        after = (current[0] - first.x) * forward_x + (current[1] - first.y) * forward_y
        epsilon = 1e-9
        if before > epsilon or after < -epsilon or after - before <= epsilon:
            return
        fraction = max(0.0, min(1.0, -before / (after - before)))
        crossing_x = previous_odom[0] + fraction * (current[0] - previous_odom[0])
        crossing_y = previous_odom[1] + fraction * (current[1] - previous_odom[1])
        if math.hypot(crossing_x - first.x, crossing_y - first.y) > self.p["lap_closure_radius_m"]:
            return
        if len(self.samples) < 30 or self.recorded_distance < minimum:
            self._reset_learning("returned_before_min_lap_distance")
            self.samples.append(RouteSample(*current, excluded=excluded))
            return
        candidate_samples = self.samples + [RouteSample(*current, excluded=excluded)]
        try:
            reference = build_odom_reference(candidate_samples, self.p["sample_spacing_m"],
                                             self.p["smooth_window_m"])
        except ValueError as exc:
            self.reason = "reference_build_failed"
            self.get_logger().warn(str(exc))
            return
        self.reference = reference
        # A first-lap stop may still be latched when the map closes.
        self.auto_stop_latched = self.stop_committed
        self.fusion_status = {"available": False, "mode": "odom_only", "reason": "alignment_pending"}
        self.alignment = {"diagnostic_only": True, "superseded_by": "map_fusion"}
        self.recorded_distance += math.hypot(current[0] - self.samples[-1].x, current[1] - self.samples[-1].y)
        self.progress = 0.0
        self.pending_distance = 0.0
        self.matched = False  # Fresh post-closure odometry confirms association.
        self.reason = "first_lap_odom_ready"
        self._publish_path()
        official = OfficialRoute(list(self.official.node_ids), list(self.official.points))
        outer = set(official.node_ids)
        comparison_points = [tuple(point) for identifier, point in self.topology.nodes.items()
                             if identifier not in outer and self.topology.adjacency.get(identifier)]
        branch_connections = {
            identifier: [(neighbour, tuple(self.topology.nodes[neighbour]))
                         for neighbour in self.topology.adjacency.get(identifier, {})
                         if neighbour not in outer]
            for identifier in official.node_ids[:-1]
        }
        branch_connections = {key: values for key, values in branch_connections.items() if values}
        try:
            self._fusion_future = self._fusion_executor.submit(
                build_fused_route, tuple(candidate_samples), official,
                spacing_m=self.p["sample_spacing_m"], smooth_window_m=self.p["smooth_window_m"],
                max_detour_gap_m=self.p["max_detour_gap_m"], max_rmse_m=self.p["alignment_max_rmse_m"],
                max_closure_gap_m=self.p["max_closure_gap_m"],
                ambiguity_margin_m=self.p["alignment_ambiguity_margin_m"],
                node_search_m=self.p["fusion_node_search_m"], feature_window_m=self.p["fusion_feature_window_m"],
                shape_weight=self.p["fusion_shape_weight"], max_displacement_m=self.p["fusion_max_displacement_m"],
                comparison_points=comparison_points, ambiguity_position_m=self.p["fusion_candidate_tolerance_m"],
                branch_connections=branch_connections,
                branch_angle_min_deg=self.p["branch_angle_min_deg"],
                branch_angle_max_deg=self.p["branch_angle_max_deg"],
            )
        except RuntimeError as exc:
            self.fusion_status = {"available": False, "mode": "odom_only", "reason": "alignment_submit_failed"}
            self.get_logger().warn(str(exc))
        self.get_logger().info(
            f"First lap complete: {self.reference.length_m:.1f} odom m; topology alignment is running."
        )

    def _poll_fusion(self, now: float) -> None:
        """Apply a worker result on the ROS thread, never from the worker."""
        if self._fusion_future is not None and self._fusion_future.done():
            future, self._fusion_future = self._fusion_future, None
            try:
                result = future.result()
            except Exception as exc:
                self.fusion_status = {"available": False, "mode": "odom_only", "reason": "alignment_worker_failed"}
                self.get_logger().warn("Topology alignment failed: " + str(exc))
                return
            self.topology_alignment = result.alignment
            if not result.status.get("available"):
                self.fusion_status = dict(result.status)
                return
            self._pending_fusion = result
            self.fusion_status = dict(result.status)
            self.fusion_status.update(available=False, mode="odom_only", reason="alignment_waiting_reference_match")
        result = self._pending_fusion
        if result is None or self.reference is None:
            return
        # OA retains ownership. A red wait may apply the map, because it needs
        # the result before it can decide whether an alternative route exists.
        if self._output_block_reason(now) or self.branch_active or self.traffic_mode == "detour":
            return
        if self.odom is None or now - self.odom_time > self.p["odom_timeout_sec"] or not self.matched:
            return
        progress = result.reference.match(
            self.odom[0], self.odom[1], self.progress, 0.0,
            self.p["match_window_m"], self.p["max_match_distance_m"],
            yaw=self.odom[2], max_heading_error_rad=math.radians(self.p["match_heading_limit_deg"]),
        )
        if progress is None or not result.reference.point_at(progress)[2]:
            return
        self.reference = result.reference
        self.progress = progress
        self.pending_distance = 0.0
        self.fusion_status = dict(result.status)
        self.fused_node_positions = dict(result.node_positions)
        self.fused_node_progress = dict(result.node_progress)
        self._pending_fusion = None
        self.guard.reset()
        self.last_correction = 0.0
        self._prepare_mapped_topology()
        self._publish_path()
        self.get_logger().info("Topology alignment accepted and attached to the odometry route.")

    def _tick(self) -> None:
        now = time.monotonic()
        self._poll_fusion(now)
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
        if self.p["enable_traffic_control"] and self.traffic_mode not in ("normal", "prepare"):
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
            "odom_reference_ready": reference is not None,
            "fusion_pending": self._fusion_future is not None or self._pending_fusion is not None,
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
            # Lowest-score selection is intentionally retained even when
            # alternatives exist. Expose that diagnostic separately so an
            # accepted fit is not mistaken for a unique correspondence.
            "fusion_ambiguity_warning": bool(self.topology_alignment and
                self.topology_alignment.get("available") is True and
                (self.topology_alignment.get("global_transform_available") is False or
                 self.topology_alignment.get("node_correspondence_available") is False)),
            "traffic_light_raw": self.traffic_light_raw,
            "traffic_light": self.light_filter.evaluate(now),
            "traffic_vote": self.light_filter.snapshot(now),
            "auto_red_signal_valid": self._red_recently_confirmed(now),
            "traffic_mode": self.traffic_mode,
            "traffic_reason": self.traffic_reason,
            "traffic_planner_version": "ubs_auto_turn_v1",
            "traffic_stop_flag": self.stop_flag,
            "traffic_stop_flag_role": "first_lap_stop" if self.reference is None else "red_fallback_only",
            "auto_stop_latched": self.auto_stop_latched,
            "traffic_stop_committed": self.stop_committed,
            "traffic_event_consumed": self.traffic_event_consumed,
            "detour_next_red_pending": self.detour_next_red_pending,
            "traffic_red_rearm_sec": self.p["traffic_red_rearm_sec"],
            "smooth_plan_reason": self.smooth_plan_reason,
            "smooth_plan_available": self.smooth_path is not None,
            "smooth_min_turn_radius_m": self.p["detour_min_turn_radius_m"],
            "smooth_max_curvature_per_m": self.smooth_path.max_curvature if self.smooth_path else None,
            "auto_trigger_reason": self.auto_trigger_reason,
            "auto_current_radius_m": self.auto_current_radius_m,
            "auto_predicted_radius_m": self.auto_predicted_radius_m,
            "auto_radius_trigger_m": self.p["detour_min_turn_radius_m"] + self.p["detour_trigger_margin_m"],
            "auto_trigger_preview_m": self.p["detour_trigger_preview_m"],
            "smooth_turn_trim_m": self.smooth_path.trim_m if self.smooth_path else None,
            "smooth_detour_progress_m": self.detour_tracker.progress_m if self.detour_tracker else None,
            "topology_alignment": self.topology_alignment,
            "blocked_edge": self.blocked_edge,
            "detour_node_ids": self.detour_node_ids,
            "detour_entry_progress_m": self.detour_entry_progress,
            "detour_approach_remaining_m": (None if self.detour_entry_progress is None
                                             else self.detour_entry_progress - self.progress),
            "velocity_gate_reason": self.velocity_gate_reason if self.p["enable_velocity_gate"] else "disabled",
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))

    def close_fusion_worker(self) -> None:
        if self._fusion_future is not None:
            self._fusion_future.cancel()
        self._fusion_future = None
        self._pending_fusion = None
        self._fusion_executor.shutdown(wait=False)


def main(args=None) -> None:
    rclpy.init(args=startup_args(args))
    node = MapAssist()
    try:
        rclpy.spin(node)
    finally:
        node.close_fusion_worker()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
