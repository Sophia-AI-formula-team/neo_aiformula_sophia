from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FakeLaneResultNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_lane_result_node")
        self.declare_parameter("mode", "valid")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.pub = self.create_publisher(String, "/lane_result", 10)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate_hz, 0.1), self._publish)

    def _publish(self) -> None:
        mode = str(self.get_parameter("mode").value)
        now = time.monotonic()
        if mode == "valid":
            result = {"stable": True, "angle_deg": 72.0, "center_x": 0.50, "confidence": 0.9}
        elif mode == "horizontal":
            result = {"stable": True, "angle_deg": 3.0, "center_x": 0.50, "confidence": 0.9}
        elif mode == "noisy":
            result = {
                "stable": int(now * 4) % 2 == 0,
                "angle_deg": 60.0 + 20.0 * math.sin(now * 3.0),
                "center_x": 0.50 + 0.25 * math.sin(now * 5.0),
                "confidence": 0.55 + 0.25 * abs(math.sin(now)),
            }
        else:
            result = {"stable": False, "angle_deg": 0.0, "center_x": 0.5, "confidence": 0.0}
        self.pub.publish(String(data=json.dumps(result)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakeLaneResultNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
