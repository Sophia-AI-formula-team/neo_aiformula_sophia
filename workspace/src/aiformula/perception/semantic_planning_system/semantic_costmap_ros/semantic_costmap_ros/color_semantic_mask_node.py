from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ColorSemanticMaskNode(Node):
    """Color-threshold adapter for black road and green forbidden regions.

    This node is intentionally simple and training-free. It does not modify the
    road detector or YOLOP. It reads a camera image and publishes two masks for
    the semantic planner:
    - road mask: dark/black pixels, limited to a lower image ROI by default.
    - offroad mask: green pixels, treated as lethal forbidden regions.
    """

    def __init__(self) -> None:
        super().__init__("color_semantic_mask_node")
        for name, value in (
            ("image_topic", "/aiformula_sensing/zed_node/left_image/undistorted"),
            ("road_mask_topic", "/semantic_perception/road_mask"),
            ("offroad_mask_topic", "/semantic_perception/offroad_mask"),
            ("debug_image_topic", "/semantic_perception/color_mask_debug"),
            ("road_roi_top_ratio", 0.35),
            ("offroad_roi_top_ratio", 0.10),
            ("road_value_max", 80),
            ("road_saturation_max", 120),
            ("green_h_min", 35),
            ("green_h_max", 90),
            ("green_s_min", 45),
            ("green_v_min", 35),
            ("morph_kernel_size", 5),
        ):
            self.declare_parameter(name, value)

        self.bridge = CvBridge()
        self.create_subscription(Image, str(self.get_parameter("image_topic").value), self._on_image, 10)
        self.road_pub = self.create_publisher(Image, str(self.get_parameter("road_mask_topic").value), 10)
        self.offroad_pub = self.create_publisher(Image, str(self.get_parameter("offroad_mask_topic").value), 10)
        self.debug_pub = self.create_publisher(Image, str(self.get_parameter("debug_image_topic").value), 10)

    def _on_image(self, msg: Image) -> None:
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        road_mask, offroad_mask = self._segment(bgr)

        road_msg = self.bridge.cv2_to_imgmsg(road_mask, encoding="mono8")
        offroad_msg = self.bridge.cv2_to_imgmsg(offroad_mask, encoding="mono8")
        debug_msg = self.bridge.cv2_to_imgmsg(self._debug_overlay(bgr, road_mask, offroad_mask), encoding="bgr8")
        for out in (road_msg, offroad_msg, debug_msg):
            out.header = msg.header
        self.road_pub.publish(road_msg)
        self.offroad_pub.publish(offroad_msg)
        self.debug_pub.publish(debug_msg)

    def _segment(self, bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        height, width = bgr.shape[:2]
        road_roi = np.zeros((height, width), dtype=np.uint8)
        offroad_roi = np.zeros((height, width), dtype=np.uint8)
        road_roi[int(height * float(self.get_parameter("road_roi_top_ratio").value)) :, :] = 255
        offroad_roi[int(height * float(self.get_parameter("offroad_roi_top_ratio").value)) :, :] = 255

        road_value_max = int(self.get_parameter("road_value_max").value)
        road_saturation_max = int(self.get_parameter("road_saturation_max").value)
        road_mask = cv2.inRange(
            hsv,
            np.array([0, 0, 0], dtype=np.uint8),
            np.array([179, road_saturation_max, road_value_max], dtype=np.uint8),
        )
        road_mask = cv2.bitwise_and(road_mask, road_roi)

        green_mask = cv2.inRange(
            hsv,
            np.array(
                [
                    int(self.get_parameter("green_h_min").value),
                    int(self.get_parameter("green_s_min").value),
                    int(self.get_parameter("green_v_min").value),
                ],
                dtype=np.uint8,
            ),
            np.array([int(self.get_parameter("green_h_max").value), 255, 255], dtype=np.uint8),
        )
        green_mask = cv2.bitwise_and(green_mask, offroad_roi)

        kernel_size = max(1, int(self.get_parameter("morph_kernel_size").value))
        if kernel_size > 1:
            kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
            road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_OPEN, kernel)
            road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)
            green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
            green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        return road_mask, green_mask

    @staticmethod
    def _debug_overlay(bgr: np.ndarray, road_mask: np.ndarray, offroad_mask: np.ndarray) -> np.ndarray:
        overlay = bgr.copy()
        road_color = np.zeros_like(overlay)
        road_color[:, :] = (255, 0, 0)
        offroad_color = np.zeros_like(overlay)
        offroad_color[:, :] = (0, 0, 255)
        overlay = np.where(road_mask[..., None] > 0, cv2.addWeighted(overlay, 0.45, road_color, 0.55, 0), overlay)
        overlay = np.where(
            offroad_mask[..., None] > 0,
            cv2.addWeighted(overlay, 0.45, offroad_color, 0.55, 0),
            overlay,
        )
        return overlay.astype(np.uint8)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ColorSemanticMaskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
