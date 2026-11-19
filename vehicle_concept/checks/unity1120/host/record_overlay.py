"""Save the vehicle camera node's actual B1.4 display image with its ROS time.

Run this optional recorder inside the isolated driver container with ROS sourced.
It consumes only the overlay and never publishes telemetry or control.
"""
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, folder):
        super().__init__("controller_comparison_overlay_recorder")
        self.folder = folder
        self.bridge = CvBridge()
        self.create_subscription(Image, "/concept/perception/overlay", self.record, qos_profile_sensor_data)

    def record(self, msg):
        stamp = msg.header.stamp
        name = self.folder / f"{stamp.sec}-{stamp.nanosec:09d}.png"
        if name.exists():
            raise RuntimeError(f"Duplicate overlay stamp {name.name}")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if not cv2.imwrite(str(name), image):
            raise IOError(name)


if __name__ == "__main__":
    output = Path("/evidence/overlays")
    output.mkdir(exist_ok=True)
    rclpy.init()
    node = Recorder(output)
    print("READY overlay recorder", flush=True)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
