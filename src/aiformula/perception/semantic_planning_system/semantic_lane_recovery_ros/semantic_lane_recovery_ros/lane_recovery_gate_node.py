from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from semantic_planner_core.constraints import SemanticConstraints
from semantic_planner_core.lane_recovery import LaneRecoveryGate


class LaneRecoveryGateNode(Node):
    def __init__(self) -> None:
        super().__init__("lane_recovery_gate_node")
        for name, value in (
            ("stable_required_frames", 5),
            ("min_confidence", 0.65),
            ("horizontal_angle_threshold_deg", 15.0),
            ("max_center_jump_ratio", 0.18),
            ("lane_result_topic", "/lane_result"),
            ("semantic_constraints_topic", "/semantic_constraints"),
            ("lane_recovery_state_topic", "/lane_recovery_state"),
        ):
            self.declare_parameter(name, value)

        self.constraints = SemanticConstraints()
        self.gate = self._make_gate()
        self.state_pub = self.create_publisher(String, str(self.get_parameter("lane_recovery_state_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("lane_result_topic").value), self._on_lane_result, 10)
        self.create_subscription(String, str(self.get_parameter("semantic_constraints_topic").value), self._on_constraints, 10)

    def _make_gate(self) -> LaneRecoveryGate:
        return LaneRecoveryGate(
            stable_required_frames=int(self.get_parameter("stable_required_frames").value),
            min_confidence=float(self.get_parameter("min_confidence").value),
            horizontal_angle_threshold_deg=float(self.get_parameter("horizontal_angle_threshold_deg").value),
            max_center_jump_ratio=float(self.get_parameter("max_center_jump_ratio").value),
        )

    def _on_constraints(self, msg: String) -> None:
        self.constraints = SemanticConstraints.from_json(msg.data)

    def _on_lane_result(self, msg: String) -> None:
        try:
            lane_result = json.loads(msg.data)
        except json.JSONDecodeError:
            lane_result = {}
        state = self.gate.update(lane_result, self.constraints)
        self.state_pub.publish(String(data=state))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LaneRecoveryGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
