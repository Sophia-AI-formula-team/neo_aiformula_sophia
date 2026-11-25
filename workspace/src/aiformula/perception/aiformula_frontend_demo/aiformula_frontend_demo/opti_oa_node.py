"""Publish a checked semantic band and its centerline for each camera observation."""
import json
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import NS, as_json
from .oa_stage import OaStage


class OptiOaNode(Node):
    def __init__(self):
        super().__init__("opti_oa")
        self.worker = OaStage(
            self.declare_parameter("vehicle_width_m", 1.75).value,
            self.declare_parameter("clearance_m", .10).value,
            self.declare_parameter("lookahead_m", 11.5).value,
            self.declare_parameter("max_lane_gap_m", 4.).value,
            self.declare_parameter("planning_origin_x_m", 3.).value,
            length_weight=self.declare_parameter("length_weight", 1.).value,
            curvature_weight=self.declare_parameter("curvature_weight", .25).value,
            solid_weight=self.declare_parameter("solid_weight", .70).value,
        )
        self.last_stamp = None
        self.route = self.create_publisher(Path, "/front_demo/opti_oa/path", 5)
        self.status = self.create_publisher(String, "/front_demo/opti_oa/status", 5)
        self.band = self.create_publisher(Image, "/front_demo/opti_oa/band", 5)
        self.create_subscription(String, "/front_demo/semantic/observation", self.on_data, 5)

    def on_data(self, msg):
        try:
            semantic = json.loads(msg.data)
            timestamp = int(semantic["stamp_ns"])
            if self.last_stamp is not None and 0 <= self.last_stamp-timestamp <= NS:
                return
            self.last_stamp = timestamp
            status, result = self.worker.step(semantic)
            message = Path()
            message.header.frame_id = status["body_frame"]
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(timestamp, NS)
            for x, y, yaw in status["path"]:
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                message.poses.append(pose)
            self.route.publish(message)
            self.status.publish(String(data=as_json(status)))
            image = Image()
            image.header.stamp = message.header.stamp
            image.header.frame_id = semantic["image_frame"]
            image.height = semantic["calibration"]["height"]
            image.width = semantic["calibration"]["width"]
            image.encoding = "mono8"
            image.step = image.width
            image.data = (result.band.tobytes() if result is not None
                          else bytes(image.height*image.width))
            self.band.publish(image)
        except (ValueError, KeyError, TypeError, RuntimeError) as error:
            self.get_logger().warning(f"opti_oa frame rejected: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = OptiOaNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
