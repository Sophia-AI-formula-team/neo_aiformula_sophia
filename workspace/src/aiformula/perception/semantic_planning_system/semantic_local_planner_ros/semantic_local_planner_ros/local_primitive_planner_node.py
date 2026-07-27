from __future__ import annotations

import json

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from semantic_planner_core.constraints import SemanticConstraints
from semantic_planner_core.heading import yaw_from_quaternion
from semantic_planner_core.planner import LocalPrimitivePlanner, PlannerWeights
from semantic_planner_core.primitives import Primitive, default_primitives


class LocalPrimitivePlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("local_primitive_planner_node")
        for name, value in (
            ("planner_rate_hz", 10.0),
            ("primitive_duration_sec", 1.0),
            ("small_left_w", 0.30),
            ("medium_left_w", 0.55),
            ("strong_left_w", 0.85),
            ("slow_forward_v", 0.15),
            ("diversion_v", 0.08),
            ("heading_weight", 0.8),
            ("smoothness_weight", 0.4),
            ("curvature_weight", 0.3),
            ("semantic_cost_weight", 1.0),
            ("progress_weight", 0.25),
            ("local_costmap_topic", "/local_semantic_costmap"),
            ("semantic_constraints_topic", "/semantic_constraints"),
            ("theta_ref_topic", "/theta_ref"),
            ("odom_topic", "/aiformula_sensing/gyro_odometry_publisher/odom"),
            ("selected_primitive_topic", "/selected_primitive"),
            ("selected_path_debug_topic", "/selected_path_debug"),
            ("planner_cmd_vel_topic", "/planner_cmd_vel"),
            ("planner_score_debug_topic", "/planner_score_debug"),
            ("primitive_markers_topic", "/semantic_planner/primitive_markers"),
            ("marker_frame_id", "base_footprint"),
            ("marker_forward_scale_m", 5.0),
            ("marker_lateral_scale_m", 3.0),
        ):
            self.declare_parameter(name, value)

        self.bridge = CvBridge()
        self.costmap: np.ndarray | None = None
        self.constraints = SemanticConstraints()
        self.theta_ref = 0.0
        self.current_yaw = 0.0
        self.previous_primitive_name: str | None = None

        self.create_subscription(Image, str(self.get_parameter("local_costmap_topic").value), self._on_costmap, 10)
        self.create_subscription(String, str(self.get_parameter("semantic_constraints_topic").value), self._on_constraints, 10)
        self.create_subscription(Float32, str(self.get_parameter("theta_ref_topic").value), self._on_theta_ref, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 20)

        self.primitive_pub = self.create_publisher(String, str(self.get_parameter("selected_primitive_topic").value), 10)
        self.path_pub = self.create_publisher(Path, str(self.get_parameter("selected_path_debug_topic").value), 10)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("planner_cmd_vel_topic").value), 10)
        self.score_pub = self.create_publisher(String, str(self.get_parameter("planner_score_debug_topic").value), 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("primitive_markers_topic").value),
            10,
        )

        rate_hz = float(self.get_parameter("planner_rate_hz").value)
        self.create_timer(1.0 / max(rate_hz, 0.1), self._plan_once)

    def _planner(self) -> LocalPrimitivePlanner:
        weights = PlannerWeights(
            heading=float(self.get_parameter("heading_weight").value),
            smoothness=float(self.get_parameter("smoothness_weight").value),
            curvature=float(self.get_parameter("curvature_weight").value),
            semantic_cost=float(self.get_parameter("semantic_cost_weight").value),
            progress=float(self.get_parameter("progress_weight").value),
        )
        return LocalPrimitivePlanner(weights=weights, primitives=self._primitives())

    def _primitives(self) -> list[Primitive]:
        return default_primitives(
            duration=float(self.get_parameter("primitive_duration_sec").value),
            small_left_w=float(self.get_parameter("small_left_w").value),
            medium_left_w=float(self.get_parameter("medium_left_w").value),
            strong_left_w=float(self.get_parameter("strong_left_w").value),
            slow_forward_v=float(self.get_parameter("slow_forward_v").value),
            diversion_v=float(self.get_parameter("diversion_v").value),
        )

    def _on_costmap(self, msg: Image) -> None:
        self.costmap = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def _on_constraints(self, msg: String) -> None:
        self.constraints = SemanticConstraints.from_json(msg.data)

    def _on_theta_ref(self, msg: Float32) -> None:
        self.theta_ref = float(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _plan_once(self) -> None:
        if self.costmap is None:
            self._publish_stop("missing_costmap")
            return

        selected, details = self._planner().plan(
            self.costmap,
            self.constraints,
            current_yaw=self.current_yaw,
            theta_ref=self.theta_ref,
            previous_primitive_name=self.previous_primitive_name,
        )
        self.previous_primitive_name = selected.name
        self.primitive_pub.publish(String(data=selected.name))
        self.cmd_pub.publish(self._primitive_to_twist(selected))
        self.path_pub.publish(self._primitive_to_path(selected))
        self.score_pub.publish(String(data=json.dumps(details, sort_keys=True)))
        self.marker_pub.publish(self._primitive_markers(details, selected.name))

    def _publish_stop(self, reason: str) -> None:
        self.primitive_pub.publish(String(data="STOP"))
        self.cmd_pub.publish(Twist())
        self.score_pub.publish(String(data=json.dumps({"STOP": {"reason": reason, "valid": True}})))
        self.marker_pub.publish(self._delete_all_markers())

    def _primitive_to_twist(self, primitive: Primitive) -> Twist:
        msg = Twist()
        if primitive.name != "STOP":
            msg.linear.x = float(primitive.linear_velocity)
            msg.angular.z = float(primitive.angular_velocity)
        return msg

    def _primitive_to_path(self, primitive: Primitive) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "camera"
        for x_ratio, y_ratio in primitive.path_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x_ratio)
            pose.pose.position.y = float(y_ratio)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _primitive_markers(self, details: dict, selected_name: str) -> MarkerArray:
        markers = self._delete_all_markers()
        now = self.get_clock().now().to_msg()
        marker_id = 1
        for primitive in self._primitives():
            if primitive.is_stop or not primitive.path_points:
                continue

            info = details.get(primitive.name, {})
            selected = primitive.name == selected_name
            valid = bool(info.get("valid", False))
            line = Marker()
            line.header.stamp = now
            line.header.frame_id = str(self.get_parameter("marker_frame_id").value)
            line.ns = "semantic_primitives"
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08 if selected else 0.04
            line.color.r, line.color.g, line.color.b, line.color.a = self._marker_color(valid, selected)
            line.points = [self._normalized_to_marker_point(point) for point in primitive.path_points]
            markers.markers.append(line)

            label = Marker()
            label.header = line.header
            label.ns = "semantic_primitive_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = line.points[-1]
            label.pose.position.z += 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18 if selected else 0.13
            label.color.r, label.color.g, label.color.b, label.color.a = self._marker_color(valid, selected)
            reason = str(info.get("reason", "unknown"))
            label.text = f"{primitive.name}\n{reason}"
            markers.markers.append(label)

        return markers

    def _delete_all_markers(self) -> MarkerArray:
        marker = Marker()
        marker.action = Marker.DELETEALL
        return MarkerArray(markers=[marker])

    def _normalized_to_marker_point(self, point: tuple[float, float]) -> Point:
        x_ratio, y_ratio = point
        forward_scale = float(self.get_parameter("marker_forward_scale_m").value)
        lateral_scale = float(self.get_parameter("marker_lateral_scale_m").value)
        point_msg = Point()
        point_msg.x = max(0.0, 1.0 - float(y_ratio)) * forward_scale
        point_msg.y = (0.5 - float(x_ratio)) * lateral_scale
        point_msg.z = 0.05
        return point_msg

    @staticmethod
    def _marker_color(valid: bool, selected: bool) -> tuple[float, float, float, float]:
        if selected:
            return 0.0, 1.0, 0.0, 1.0
        if valid:
            return 0.0, 0.45, 1.0, 0.85
        return 1.0, 0.0, 0.0, 0.75


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LocalPrimitivePlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
