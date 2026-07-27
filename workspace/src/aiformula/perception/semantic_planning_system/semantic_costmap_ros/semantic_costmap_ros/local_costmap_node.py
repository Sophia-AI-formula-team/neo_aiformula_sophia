from __future__ import annotations

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from semantic_planner_core.constraints import SemanticConstraints
from semantic_planner_core.costmap import SemanticCostmapBuilder


class LocalCostmapNode(Node):
    def __init__(self) -> None:
        super().__init__("local_costmap_node")
        for name, value in (
            ("low_cost", 20),
            ("unknown_cost", 180),
            ("lethal_cost", 255),
            ("red_forward_center_x_ratio", 0.5),
            ("red_forward_width_ratio", 0.20),
            ("red_forward_top_y_ratio", 0.25),
            ("red_forward_bottom_y_ratio", 0.62),
            ("mask_stale_timeout_sec", 0.5),
            ("publish_rate_hz", 10.0),
            ("fallback_height", 120),
            ("fallback_width", 160),
            ("road_mask_topic", "/aiformula_perception/road_detector/mask_image"),
            ("offroad_mask_topic", "/semantic_perception/offroad_mask"),
            ("lane_guidance_mask_topic", "/lane_guidance_mask"),
            ("semantic_constraints_topic", "/semantic_constraints"),
            ("local_costmap_topic", "/local_semantic_costmap"),
            ("local_costmap_debug_topic", "/local_semantic_costmap_debug"),
        ):
            self.declare_parameter(name, value)

        self.bridge = CvBridge()
        self.road_mask: np.ndarray | None = None
        self.offroad_mask: np.ndarray | None = None
        self.lane_mask: np.ndarray | None = None
        self.road_time = 0.0
        self.offroad_time = 0.0
        self.constraints = SemanticConstraints()

        self.create_subscription(Image, str(self.get_parameter("road_mask_topic").value), self._on_road, 10)
        self.create_subscription(Image, str(self.get_parameter("offroad_mask_topic").value), self._on_offroad, 10)
        self.create_subscription(Image, str(self.get_parameter("lane_guidance_mask_topic").value), self._on_lane, 10)
        self.create_subscription(String, str(self.get_parameter("semantic_constraints_topic").value), self._on_constraints, 10)
        self.costmap_pub = self.create_publisher(Image, str(self.get_parameter("local_costmap_topic").value), 10)
        self.debug_pub = self.create_publisher(Image, str(self.get_parameter("local_costmap_debug_topic").value), 10)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate_hz, 0.1), self._publish_costmap)

    def _builder(self) -> SemanticCostmapBuilder:
        return SemanticCostmapBuilder(
            low_cost=int(self.get_parameter("low_cost").value),
            unknown_cost=int(self.get_parameter("unknown_cost").value),
            lethal_cost=int(self.get_parameter("lethal_cost").value),
            center_x_ratio=float(self.get_parameter("red_forward_center_x_ratio").value),
            width_ratio=float(self.get_parameter("red_forward_width_ratio").value),
            top_y_ratio=float(self.get_parameter("red_forward_top_y_ratio").value),
            bottom_y_ratio=float(self.get_parameter("red_forward_bottom_y_ratio").value),
        )

    def _on_road(self, msg: Image) -> None:
        self.road_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.road_time = time.monotonic()

    def _on_offroad(self, msg: Image) -> None:
        self.offroad_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.offroad_time = time.monotonic()

    def _on_lane(self, msg: Image) -> None:
        self.lane_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def _on_constraints(self, msg: String) -> None:
        self.constraints = SemanticConstraints.from_json(msg.data)

    def _publish_costmap(self) -> None:
        costmap = self._build_or_fallback()
        image = self.bridge.cv2_to_imgmsg(costmap, encoding="mono8")
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = "camera"
        self.costmap_pub.publish(image)
        self.debug_pub.publish(image)

    def _build_or_fallback(self) -> np.ndarray:
        now = time.monotonic()
        stale_timeout = float(self.get_parameter("mask_stale_timeout_sec").value)
        masks_ready = self.road_mask is not None and self.offroad_mask is not None
        masks_fresh = now - self.road_time <= stale_timeout and now - self.offroad_time <= stale_timeout
        if masks_ready and masks_fresh:
            return self._builder().build(
                self.road_mask,
                self.offroad_mask,
                self.constraints,
                lane_guidance_mask=self.lane_mask,
            )

        height = int(self.get_parameter("fallback_height").value)
        width = int(self.get_parameter("fallback_width").value)
        self.get_logger().warn("Input masks missing or stale; publishing conservative high-cost costmap.")
        return np.full((height, width), int(self.get_parameter("unknown_cost").value), dtype=np.uint8)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LocalCostmapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
