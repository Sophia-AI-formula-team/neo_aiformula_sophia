from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String

from semantic_planner_core.constraints import SemanticConstraints
from semantic_planner_core.heading import yaw_from_quaternion


class TrafficLightGateNode(Node):
    def __init__(self) -> None:
        super().__init__("traffic_light_gate_node")
        self.declare_parameter("red_stable_frames", 3)
        self.declare_parameter("green_stable_frames", 3)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("traffic_light_state_topic", "/traffic_light/state")
        self.declare_parameter("odom_topic", "/aiformula_sensing/gyro_odometry_publisher/odom")
        self.declare_parameter("semantic_constraints_topic", "/semantic_constraints")
        self.declare_parameter("theta_ref_topic", "/theta_ref")

        self.red_stable_frames = int(self.get_parameter("red_stable_frames").value)
        self.green_stable_frames = int(self.get_parameter("green_stable_frames").value)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.current_yaw = 0.0
        self.theta_ref = 0.0
        self.red_count = 0
        self.green_count = 0
        self.constraints = SemanticConstraints()

        self.create_subscription(String, str(self.get_parameter("traffic_light_state_topic").value), self._on_light_state, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 20)
        self.constraints_pub = self.create_publisher(String, str(self.get_parameter("semantic_constraints_topic").value), 10)
        self.theta_pub = self.create_publisher(Float32, str(self.get_parameter("theta_ref_topic").value), 10)
        self.create_timer(1.0 / max(0.1, rate_hz), self._publish)

    def _on_light_state(self, msg: String) -> None:
        state = msg.data.strip().lower()
        if state == "red":
            self.red_count += 1
            self.green_count = 0
        elif state == "green":
            self.green_count += 1
            self.red_count = 0
        else:
            self.red_count = 0
            self.green_count = 0

        if self.red_count == self.red_stable_frames:
            self.theta_ref = self.current_yaw

        if self.red_count >= self.red_stable_frames:
            self.constraints = SemanticConstraints(
                red_light_active=True,
                red_forward_forbidden=True,
                allow_lane_follow=False,
                diversion_active=True,
                mode="RED_FORWARD_GATE",
            )
        elif self.green_count >= self.green_stable_frames:
            self.constraints = SemanticConstraints(
                red_light_active=False,
                red_forward_forbidden=False,
                allow_lane_follow=True,
                diversion_active=False,
                mode="NORMAL",
            )

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _publish(self) -> None:
        self.constraints_pub.publish(String(data=self.constraints.to_json()))
        self.theta_pub.publish(Float32(data=float(self.theta_ref)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrafficLightGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
