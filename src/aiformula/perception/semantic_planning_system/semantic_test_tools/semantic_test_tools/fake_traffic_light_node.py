from __future__ import annotations

import itertools

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FakeTrafficLightNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_traffic_light_node")
        self.declare_parameter("state", "red")
        self.declare_parameter("cycle", False)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.pub = self.create_publisher(String, "/traffic_light/state", 10)
        self.states = itertools.cycle(["red", "red", "green", "green", "unknown"])
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate_hz, 0.1), self._publish)

    def _publish(self) -> None:
        if bool(self.get_parameter("cycle").value):
            state = next(self.states)
        else:
            state = str(self.get_parameter("state").value)
        self.pub.publish(String(data=state))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakeTrafficLightNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
