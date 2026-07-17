#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def adjust_contrast(img_bgr: np.ndarray, contrast: float = 1.0) -> np.ndarray:
    """Simple linear contrast adjustment around mid-gray."""
    img = img_bgr.astype(np.float32)
    img = (img - 127.5) * contrast + 127.5
    img = np.clip(img, 0, 255).astype(np.uint8)
    return img


def adjust_saturation(img_bgr: np.ndarray, saturation: float = 1.0) -> np.ndarray:
    """Scale saturation in HSV space."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    s = s.astype(np.float32)
    s *= saturation
    s = np.clip(s, 0, 255).astype(np.uint8)

    hsv = cv2.merge((h, s, v))
    out = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return out


def adjust_contrast_and_saturation(
    img_bgr: np.ndarray,
    contrast: float = 1.0,
    saturation: float = 1.0,
) -> np.ndarray:
    out = adjust_contrast(img_bgr, contrast=contrast)
    out = adjust_saturation(out, saturation=saturation)
    return out


class ImagePreprocessNode(Node):
    def __init__(self) -> None:
        super().__init__('image_preprocess_node')

        self.bridge = CvBridge()

        # Topics: where to read & where to publish
        self.declare_parameter(
            'input_topic',
            '/aiformula_sensing/zed_node/left_image/undistorted'
        )
        self.declare_parameter(
            'output_topic',
            '/aiformula_sensing/zed_node/left_image/normalized'
        )

        # Parameters: tunable from command line or ros2 param set
        self.declare_parameter('contrast', 1.9)
        self.declare_parameter('saturation', 1.9)

        self.input_topic = self.get_parameter(
            'input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(
            Image,
            self.output_topic,
            10
        )

        self.get_logger().info(
            f"ImagePreprocessNode started: {self.input_topic} -> {self.output_topic}"
        )

        # Allow live parameter updates
        self.add_on_set_parameters_callback(self.on_param_change)

    def on_param_change(self, params):
        from rclpy.parameter import SetParametersResult
        for p in params:
            if p.name == 'contrast':
                self.get_logger().info(f"Updated contrast: {p.value}")
            if p.name == 'saturation':
                self.get_logger().info(f"Updated saturation: {p.value}")
        return SetParametersResult(successful=True)

    def image_callback(self, msg: Image) -> None:
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}")
            return

        contrast = self.get_parameter(
            'contrast').get_parameter_value().double_value
        saturation = self.get_parameter(
            'saturation').get_parameter_value().double_value

        processed = adjust_contrast_and_saturation(
            img,
            contrast=contrast,
            saturation=saturation,
        )

        out_msg = self.bridge.cv2_to_imgmsg(processed, encoding='bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImagePreprocessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
