from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class RedPixelTrafficLightAdapterNode(Node):
    """Convert a red-pixel count into the semantic traffic-light state topic."""

    def __init__(self) -> None:
        super().__init__("red_pixel_traffic_light_adapter_node")
        for name, value in (
            ("red_pixels_topic", "/red_pixels_count"),
            ("traffic_light_state_topic", "/traffic_light/state"),
            ("red_pixel_threshold", 600),
            ("clear_pixel_threshold", 100),
            ("stale_timeout_sec", 0.5),
            ("publish_unknown_when_stale", True),
            ("publish_rate_hz", 10.0),
        ):
            self.declare_parameter(name, value)

        self.latest_count: int | None = None
        self.latest_time = 0.0
        self.state = "unknown"
        self.create_subscription(Int32, str(self.get_parameter("red_pixels_topic").value), self._on_count, 10)
        self.pub = self.create_publisher(String, str(self.get_parameter("traffic_light_state_topic").value), 10)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate_hz, 0.1), self._publish)

    def _on_count(self, msg: Int32) -> None:
        self.latest_count = int(msg.data)
        self.latest_time = time.monotonic()
        if self.latest_count >= int(self.get_parameter("red_pixel_threshold").value):
            self.state = "red"
        elif self.latest_count <= int(self.get_parameter("clear_pixel_threshold").value):
            self.state = "unknown"

    def _publish(self) -> None:
        stale = self.latest_count is None or time.monotonic() - self.latest_time > float(
            self.get_parameter("stale_timeout_sec").value
        )
        if stale and not bool(self.get_parameter("publish_unknown_when_stale").value):
            return
        if stale:
            self.state = "unknown"
        self.pub.publish(String(data=self.state))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RedPixelTrafficLightAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
