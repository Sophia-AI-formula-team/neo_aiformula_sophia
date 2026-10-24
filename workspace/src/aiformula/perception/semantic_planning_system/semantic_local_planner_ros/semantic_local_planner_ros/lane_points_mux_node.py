from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import json
import time

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from semantic_planner_core.constraints import SemanticConstraints


class LanePointsMuxNode(Node):
    """Mux original and semantic three-point Pose2D streams for withoutkalman."""

    def __init__(self) -> None:
        super().__init__("lane_points_mux_node")
        for name, value in (
            ("original_point_a_topic", "/original_processed_point_a"),
            ("original_point_b_topic", "/original_processed_point_b"),
            ("original_point_c_topic", "/original_processed_point_c"),
            ("semantic_point_a_topic", "/semantic_planner/processed_point_a"),
            ("semantic_point_b_topic", "/semantic_planner/processed_point_b"),
            ("semantic_point_c_topic", "/semantic_planner/processed_point_c"),
            ("output_point_a_topic", "/processed_point_a"),
            ("output_point_b_topic", "/processed_point_b"),
            ("output_point_c_topic", "/processed_point_c"),
            ("semantic_constraints_topic", "/semantic_constraints"),
            ("costmap_health_topic", "/local_semantic_costmap_health"),
            ("lane_recovery_state_topic", "/lane_recovery_state"),
            ("selected_primitive_topic", "/selected_primitive"),
            ("mux_state_topic", "/semantic_planner/lane_points_mux_state"),
            ("lane_stop_flag_topic", "/lane_stop_flag"),
            ("speed_limit_topic", "/semantic_planner/speed_limit"),
            ("publish_rate_hz", 10.0),
            ("stale_timeout_sec", 0.5),
            ("require_recoverable_after_diversion", True),
            ("publish_zero_when_blocked", True),
            ("publish_speed_limit", True),
            ("normal_speed_limit_mps", 0.45),
            ("search_speed_limit_mps", 0.18),
            ("stop_speed_limit_mps", 0.0),
            ("min_semantic_point_x_m", 0.05),
            ("max_semantic_point_x_m", 5.5),
            ("max_abs_semantic_point_y_m", 2.2),
        ):
            self.declare_parameter(name, value)

        self.original_points: Dict[str, Optional[Pose2D]] = {"a": None, "b": None, "c": None}
        self.semantic_points: Dict[str, Optional[Pose2D]] = {"a": None, "b": None, "c": None}
        self.original_times: Dict[str, float] = {"a": 0.0, "b": 0.0, "c": 0.0}
        self.semantic_times: Dict[str, float] = {"a": 0.0, "b": 0.0, "c": 0.0}
        self.constraints = SemanticConstraints()
        self.costmap_health = "stale"
        self.lane_recovery_state = "ignore_lane"
        self.selected_primitive = "STOP"
        self.had_diversion = False

        self.create_subscription(Pose2D, str(self.get_parameter("original_point_a_topic").value), self._original_cb("a"), 10)
        self.create_subscription(Pose2D, str(self.get_parameter("original_point_b_topic").value), self._original_cb("b"), 10)
        self.create_subscription(Pose2D, str(self.get_parameter("original_point_c_topic").value), self._original_cb("c"), 10)
        self.create_subscription(Pose2D, str(self.get_parameter("semantic_point_a_topic").value), self._semantic_cb("a"), 10)
        self.create_subscription(Pose2D, str(self.get_parameter("semantic_point_b_topic").value), self._semantic_cb("b"), 10)
        self.create_subscription(Pose2D, str(self.get_parameter("semantic_point_c_topic").value), self._semantic_cb("c"), 10)
        self.create_subscription(String, str(self.get_parameter("semantic_constraints_topic").value), self._on_constraints, 10)
        self.create_subscription(String, str(self.get_parameter("costmap_health_topic").value), self._on_costmap_health, 10)
        self.create_subscription(String, str(self.get_parameter("lane_recovery_state_topic").value), self._on_lane_recovery_state, 10)
        self.create_subscription(String, str(self.get_parameter("selected_primitive_topic").value), self._on_selected_primitive, 10)

        self.point_a_pub = self.create_publisher(Pose2D, str(self.get_parameter("output_point_a_topic").value), 10)
        self.point_b_pub = self.create_publisher(Pose2D, str(self.get_parameter("output_point_b_topic").value), 10)
        self.point_c_pub = self.create_publisher(Pose2D, str(self.get_parameter("output_point_c_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("mux_state_topic").value), 10)
        self.stop_flag_pub = self.create_publisher(Bool, str(self.get_parameter("lane_stop_flag_topic").value), 10)
        self.speed_limit_pub = self.create_publisher(Float32, str(self.get_parameter("speed_limit_topic").value), 10)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate_hz, 0.1), self._publish_once)

    def _original_cb(self, label: str):
        def callback(msg: Pose2D) -> None:
            self.original_points[label] = msg
            self.original_times[label] = time.monotonic()

        return callback

    def _semantic_cb(self, label: str):
        def callback(msg: Pose2D) -> None:
            self.semantic_points[label] = msg
            self.semantic_times[label] = time.monotonic()

        return callback

    def _on_constraints(self, msg: String) -> None:
        try:
            self.constraints = SemanticConstraints.from_json(msg.data)
        except (TypeError, json.JSONDecodeError, ValueError):
            self.get_logger().warn("Ignoring invalid semantic constraints JSON")
            return
        if self.constraints.diversion_active:
            self.had_diversion = True

    def _on_costmap_health(self, msg: String) -> None:
        payload = msg.data.strip()
        if not payload:
            self.costmap_health = "stale"
            return
        try:
            parsed = json.loads(payload)
        except json.JSONDecodeError:
            self.costmap_health = payload
            return
        self.costmap_health = str(parsed.get("status", "stale"))

    def _on_lane_recovery_state(self, msg: String) -> None:
        self.lane_recovery_state = msg.data.strip() or "ignore_lane"

    def _on_selected_primitive(self, msg: String) -> None:
        self.selected_primitive = msg.data.strip() or "STOP"

    def _publish_once(self) -> None:
        points, source, reason = self._select_points()
        if points is None:
            if not bool(self.get_parameter("publish_zero_when_blocked").value):
                self._publish_stop_and_speed("zero")
                self._publish_state("none", reason)
                return
            points = (Pose2D(), Pose2D(), Pose2D())
            source = "zero"

        self.point_a_pub.publish(points[0])
        self.point_b_pub.publish(points[1])
        self.point_c_pub.publish(points[2])
        self._publish_stop_and_speed(source)
        self._publish_state(source, reason)

    def _select_points(self) -> Tuple[Optional[Tuple[Pose2D, Pose2D, Pose2D]], str, str]:
        if self.constraints.diversion_active or self.constraints.red_forward_forbidden:
            if self.costmap_health != "fresh":
                return None, "zero", "costmap_stale_during_diversion"
            if self.selected_primitive == "STOP":
                return None, "zero", "semantic_stop_selected"
            semantic = self._fresh_triplet(self.semantic_points, self.semantic_times, validate_semantic=True)
            if semantic is not None:
                return semantic, "semantic", "red_search_path_found"
            return None, "zero", "semantic_missing_stale_or_invalid_during_diversion"

        if not self.constraints.allow_lane_follow:
            return None, "zero", "lane_follow_not_allowed"

        require_recoverable = bool(self.get_parameter("require_recoverable_after_diversion").value)
        if self.had_diversion and require_recoverable and self.lane_recovery_state != "recoverable":
            return None, "zero", "waiting_for_lane_recovery_gate"

        original = self._fresh_triplet(self.original_points, self.original_times)
        if original is not None:
            return original, "original", "lane_follow_allowed"

        return None, "zero", "original_missing_or_stale"

    def _fresh_triplet(
        self,
        points: Dict[str, Optional[Pose2D]],
        stamps: Dict[str, float],
        validate_semantic: bool = False,
    ) -> Optional[Tuple[Pose2D, Pose2D, Pose2D]]:
        timeout = float(self.get_parameter("stale_timeout_sec").value)
        now = time.monotonic()
        triplet: List[Pose2D] = []
        for label in ("a", "b", "c"):
            point = points[label]
            if point is None or now - stamps[label] > timeout:
                return None
            triplet.append(point)
        result = (triplet[0], triplet[1], triplet[2])
        if validate_semantic and not self._semantic_triplet_is_safe(result):
            return None
        return result

    def _semantic_triplet_is_safe(self, points: Tuple[Pose2D, Pose2D, Pose2D]) -> bool:
        min_x = float(self.get_parameter("min_semantic_point_x_m").value)
        max_x = float(self.get_parameter("max_semantic_point_x_m").value)
        max_abs_y = float(self.get_parameter("max_abs_semantic_point_y_m").value)
        for point in points:
            values = (float(point.x), float(point.y), float(point.theta))
            if not all(np_isfinite(value) for value in values):
                return False
            if point.x < min_x or point.x > max_x:
                return False
            if abs(point.y) > max_abs_y:
                return False
        return True

    def _publish_stop_and_speed(self, source: str) -> None:
        stop = source in ("zero", "none")
        self.stop_flag_pub.publish(Bool(data=stop))
        if not bool(self.get_parameter("publish_speed_limit").value):
            return
        if stop:
            speed = float(self.get_parameter("stop_speed_limit_mps").value)
        elif source == "semantic":
            speed = float(self.get_parameter("search_speed_limit_mps").value)
        else:
            speed = float(self.get_parameter("normal_speed_limit_mps").value)
        self.speed_limit_pub.publish(Float32(data=float(speed)))

    def _publish_state(self, source: str, reason: str) -> None:
        payload = {
            "source": source,
            "reason": reason,
            "selected_primitive": self.selected_primitive,
            "lane_recovery_state": self.lane_recovery_state,
            "diversion_active": self.constraints.diversion_active,
            "allow_lane_follow": self.constraints.allow_lane_follow,
            "costmap_health": self.costmap_health,
            "had_diversion": self.had_diversion,
            "original_fresh": self._fresh_triplet(self.original_points, self.original_times) is not None,
            "semantic_fresh": self._fresh_triplet(self.semantic_points, self.semantic_times, validate_semantic=True) is not None,
        }
        self.state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def np_isfinite(value: float) -> bool:
    return value == value and value not in (float("inf"), float("-inf"))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LanePointsMuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
