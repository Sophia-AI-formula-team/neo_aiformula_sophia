from __future__ import annotations

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from semantic_planner_core.constraints import SemanticConstraints
from semantic_planner_core.primitives import PrimitiveName


class SafetySupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_supervisor_node")
        for name, value in (
            ("max_linear_velocity", 0.25),
            ("max_angular_velocity", 1.2),
            ("costmap_stale_timeout_sec", 0.5),
            ("allow_cmd_clamping", True),
            ("planner_cmd_vel_topic", "/planner_cmd_vel"),
            ("local_costmap_topic", "/local_semantic_costmap"),
            ("semantic_constraints_topic", "/semantic_constraints"),
            ("selected_primitive_topic", "/selected_primitive"),
            ("output_cmd_vel_topic", "/aiformula_control/game_pad/cmd_vel"),
            ("safety_state_topic", "/safety_state"),
        ):
            self.declare_parameter(name, value)

        self.bridge = CvBridge()
        self.cmd = Twist()
        self.costmap: np.ndarray | None = None
        self.costmap_time = 0.0
        self.constraints = SemanticConstraints()
        self.selected_primitive = "STOP"

        self.create_subscription(Twist, str(self.get_parameter("planner_cmd_vel_topic").value), self._on_cmd, 10)
        self.create_subscription(Image, str(self.get_parameter("local_costmap_topic").value), self._on_costmap, 10)
        self.create_subscription(String, str(self.get_parameter("semantic_constraints_topic").value), self._on_constraints, 10)
        self.create_subscription(String, str(self.get_parameter("selected_primitive_topic").value), self._on_selected, 10)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("output_cmd_vel_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("safety_state_topic").value), 10)
        self.create_timer(0.05, self._supervise)

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd = msg

    def _on_costmap(self, msg: Image) -> None:
        self.costmap = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.costmap_time = time.monotonic()

    def _on_constraints(self, msg: String) -> None:
        self.constraints = SemanticConstraints.from_json(msg.data)

    def _on_selected(self, msg: String) -> None:
        self.selected_primitive = msg.data.strip()

    def _supervise(self) -> None:
        safe, state, cmd = self._checked_command()
        self.cmd_pub.publish(cmd if safe else Twist())
        self.state_pub.publish(String(data=state))

    def _checked_command(self) -> tuple[bool, str, Twist]:
        if self.costmap is None or time.monotonic() - self.costmap_time > float(
            self.get_parameter("costmap_stale_timeout_sec").value
        ):
            return False, "STOP_COSTMAP_MISSING_OR_STALE", Twist()

        valid_names = {item.value for item in PrimitiveName}
        if self.selected_primitive not in valid_names:
            return False, "STOP_UNKNOWN_PRIMITIVE", Twist()

        if self.constraints.red_forward_forbidden and self.selected_primitive == PrimitiveName.SLOW_FORWARD.value:
            return False, "STOP_RED_FORWARD_GATE", Twist()

        max_v = float(self.get_parameter("max_linear_velocity").value)
        max_w = float(self.get_parameter("max_angular_velocity").value)
        if abs(self.cmd.linear.x) > max_v or abs(self.cmd.angular.z) > max_w:
            if not bool(self.get_parameter("allow_cmd_clamping").value):
                return False, "STOP_COMMAND_LIMIT", Twist()
            clamped = Twist()
            clamped.linear.x = max(-max_v, min(max_v, self.cmd.linear.x))
            clamped.angular.z = max(-max_w, min(max_w, self.cmd.angular.z))
            return True, "CLAMPED", clamped

        return True, "PASS", self.cmd


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
