from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class FakeMaskPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_mask_publisher_node")
        self.declare_parameter("mode", "straight_road")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("height", 120)
        self.declare_parameter("width", 160)
        self.declare_parameter("road_mask_topic", "/aiformula_perception/road_detector/mask_image")
        self.declare_parameter("offroad_mask_topic", "/semantic_perception/offroad_mask")
        self.declare_parameter("lane_guidance_mask_topic", "/lane_guidance_mask")
        self.bridge = CvBridge()
        self.road_pub = self.create_publisher(Image, str(self.get_parameter("road_mask_topic").value), 10)
        self.offroad_pub = self.create_publisher(Image, str(self.get_parameter("offroad_mask_topic").value), 10)
        self.lane_pub = self.create_publisher(Image, str(self.get_parameter("lane_guidance_mask_topic").value), 10)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate_hz, 0.1), self._publish)

    def _publish(self) -> None:
        mode = str(self.get_parameter("mode").value)
        height = int(self.get_parameter("height").value)
        width = int(self.get_parameter("width").value)
        road, offroad, lane = make_masks(mode, height, width)
        stamp = self.get_clock().now().to_msg()
        for pub, mask in ((self.road_pub, road), (self.offroad_pub, offroad), (self.lane_pub, lane)):
            msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            msg.header.stamp = stamp
            msg.header.frame_id = "camera"
            pub.publish(msg)


def make_masks(mode: str, height: int, width: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    road = np.zeros((height, width), dtype=np.uint8)
    offroad = np.zeros_like(road)
    lane = np.zeros_like(road)

    road[int(height * 0.35) :, int(width * 0.25) : int(width * 0.75)] = 255
    offroad[:, : int(width * 0.18)] = 255
    offroad[:, int(width * 0.82) :] = 255
    lane[int(height * 0.35) :, int(width * 0.48) : int(width * 0.52)] = 255

    if mode == "red_gate_left_open":
        road[int(height * 0.45) :, int(width * 0.18) : int(width * 0.45)] = 255
    elif mode == "red_gate_left_blocked":
        offroad[int(height * 0.45) :, : int(width * 0.55)] = 255
    elif mode == "noisy_lane":
        rng = np.random.default_rng(4)
        noise = rng.random((height, width)) > 0.98
        lane[noise] = 255
    elif mode != "straight_road":
        raise ValueError(f"Unknown fake mask mode: {mode}")

    return road, offroad, lane


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakeMaskPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
