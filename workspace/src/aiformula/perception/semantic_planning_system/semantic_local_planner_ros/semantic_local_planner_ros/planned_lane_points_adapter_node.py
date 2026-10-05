from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from semantic_planner_core.constraints import SemanticConstraints


class PlannedLanePointsAdapterNode(Node):
    """Convert the selected semantic planner path into three vehicle-frame Pose2D points."""

    def __init__(self) -> None:
        super().__init__("planned_lane_points_adapter_node")
        for name, value in (
            ("selected_path_debug_topic", "/selected_path_debug"),
            ("selected_primitive_topic", "/selected_primitive"),
            ("semantic_constraints_topic", "/semantic_constraints"),
            ("semantic_point_a_topic", "/semantic_planner/processed_point_a"),
            ("semantic_point_b_topic", "/semantic_planner/processed_point_b"),
            ("semantic_point_c_topic", "/semantic_planner/processed_point_c"),
            ("publish_rate_hz", 10.0),
            ("stale_timeout_sec", 0.5),
            ("publish_only_when_diversion_active", False),
            ("publish_zero_when_stop", True),
            ("vehicle_forward_scale_m", 5.0),
            ("vehicle_lateral_scale_m", 3.0),
            ("point_a_path_fraction", 0.0),
            ("point_b_path_fraction", 0.5),
            ("point_c_path_fraction", 1.0),
            ("theta_mode", "zero"),
        ):
            self.declare_parameter(name, value)

        self.latest_path: Path | None = None
        self.latest_path_time = 0.0
        self.selected_primitive = "STOP"
        self.constraints = SemanticConstraints()

        self.create_subscription(
            Path,
            str(self.get_parameter("selected_path_debug_topic").value),
            self._on_path,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("selected_primitive_topic").value),
            self._on_primitive,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("semantic_constraints_topic").value),
            self._on_constraints,
            10,
        )

        self.point_a_pub = self.create_publisher(
            Pose2D,
            str(self.get_parameter("semantic_point_a_topic").value),
            10,
        )
        self.point_b_pub = self.create_publisher(
            Pose2D,
            str(self.get_parameter("semantic_point_b_topic").value),
            10,
        )
        self.point_c_pub = self.create_publisher(
            Pose2D,
            str(self.get_parameter("semantic_point_c_topic").value),
            10,
        )

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate_hz, 0.1), self._publish_once)

    def _on_path(self, msg: Path) -> None:
        self.latest_path = msg
        self.latest_path_time = time.monotonic()

    def _on_primitive(self, msg: String) -> None:
        self.selected_primitive = msg.data.strip() or "STOP"

    def _on_constraints(self, msg: String) -> None:
        try:
            self.constraints = SemanticConstraints.from_json(msg.data)
        except (TypeError, json.JSONDecodeError, ValueError):
            self.get_logger().warn("Ignoring invalid semantic constraints JSON")

    def _publish_once(self) -> None:
        if self._should_publish_zero():
            if bool(self.get_parameter("publish_zero_when_stop").value):
                self._publish_points((Pose2D(), Pose2D(), Pose2D()))
            return

        points = self._path_to_vehicle_points(self.latest_path)
        self._publish_points(points)

    def _should_publish_zero(self) -> bool:
        if bool(self.get_parameter("publish_only_when_diversion_active").value) and not self.constraints.diversion_active:
            return True
        if self.selected_primitive == "STOP":
            return True
        if self.latest_path is None or not self.latest_path.poses:
            return True
        timeout = float(self.get_parameter("stale_timeout_sec").value)
        return time.monotonic() - self.latest_path_time > timeout

    def _path_to_vehicle_points(self, path: Path | None) -> tuple[Pose2D, Pose2D, Pose2D]:
        if path is None or not path.poses:
            return Pose2D(), Pose2D(), Pose2D()

        normalized_points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in path.poses
        ]
        fractions = (
            float(self.get_parameter("point_a_path_fraction").value),
            float(self.get_parameter("point_b_path_fraction").value),
            float(self.get_parameter("point_c_path_fraction").value),
        )
        selected_indices = [self._fraction_to_index(fraction, len(normalized_points)) for fraction in fractions]
        vehicle_points = [self._normalized_to_vehicle(point) for point in normalized_points]

        output: list[Pose2D] = []
        for index in selected_indices:
            x_m, y_m = vehicle_points[index]
            heading_index = min(index + 1, len(vehicle_points) - 1)
            if heading_index == index and index > 0:
                heading_index = index - 1
            hx_m, hy_m = vehicle_points[heading_index]
            if str(self.get_parameter("theta_mode").value) == "path_heading":
                theta = math.atan2(hy_m - y_m, hx_m - x_m)
            else:
                theta = 0.0
            output.append(Pose2D(x=float(x_m), y=float(y_m), theta=float(theta)))

        return output[0], output[1], output[2]

    def _normalized_to_vehicle(self, point: tuple[float, float]) -> tuple[float, float]:
        x_ratio, y_ratio = point
        x_ratio = self._clamp01(x_ratio)
        y_ratio = self._clamp01(y_ratio)
        forward_scale = float(self.get_parameter("vehicle_forward_scale_m").value)
        lateral_scale = float(self.get_parameter("vehicle_lateral_scale_m").value)
        # Front-view convention: image bottom is near the robot; x<0.5 is left side.
        vehicle_x = max(0.0, 1.0 - y_ratio) * forward_scale
        vehicle_y = (0.5 - x_ratio) * lateral_scale
        return vehicle_x, vehicle_y

    @staticmethod
    def _fraction_to_index(fraction: float, length: int) -> int:
        if length <= 1:
            return 0
        fraction = min(1.0, max(0.0, fraction))
        return int(round(fraction * float(length - 1)))

    @staticmethod
    def _clamp01(value: float) -> float:
        return min(1.0, max(0.0, value))

    def _publish_points(self, points: tuple[Pose2D, Pose2D, Pose2D]) -> None:
        self.point_a_pub.publish(points[0])
        self.point_b_pub.publish(points[1])
        self.point_c_pub.publish(points[2])


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlannedLanePointsAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
