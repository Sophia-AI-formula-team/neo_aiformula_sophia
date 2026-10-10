import json
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

from yolop_a3_runtime import PerceptionModel, ROS_TOPICS


class YolopA3Node(Node):
    def __init__(self):
        super().__init__("yolop_a3_perception")
        weights = Path(self.declare_parameter("weights", "weights/shared_best.pth").value).expanduser()
        device = str(self.declare_parameter("device", "auto").value)
        threshold = float(self.declare_parameter("score_threshold", .5).value)
        topic = str(self.declare_parameter("input_image_topic", ROS_TOPICS.input_image).value)
        self.model = PerceptionModel(weights, device=device, score_threshold=threshold)
        self.bridge = CvBridge()
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.images = {
            "lane_overlay": (self.create_publisher(Image, ROS_TOPICS.lane_overlay, 1), "bgr8"),
            "lane_mask": (self.create_publisher(Image, ROS_TOPICS.lane_mask, 1), "mono8"),
            "object_overlay": (self.create_publisher(Image, ROS_TOPICS.object_overlay, 1), "bgr8"),
            "obstacle_mask": (self.create_publisher(Image, ROS_TOPICS.obstacle_mask, 1), "mono8"),
        }
        self.red = self.create_publisher(Int32, ROS_TOPICS.red_pixels_count, 1)
        self.light = self.create_publisher(String, ROS_TOPICS.traffic_light_state, 1)
        self.detections = self.create_publisher(String, ROS_TOPICS.detections_json, 1)
        self.subscription = self.create_subscription(Image, topic, self.receive, qos)
        self.get_logger().info(f"A3 single-backbone weights={weights}; input={topic}")

    def receive(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            output = self.model.predict(image)
            for name, (publisher, encoding) in self.images.items():
                result = self.bridge.cv2_to_imgmsg(getattr(output, name), encoding=encoding)
                result.header = message.header
                publisher.publish(result)
            self.red.publish(Int32(data=output.red_pixels_count))
            self.light.publish(String(data=output.traffic_light_state))
            payload = output.detections_payload()
            payload["header"] = dict(stamp=dict(sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec),
                                     frame_id=message.header.frame_id)
            self.detections.publish(String(data=json.dumps(payload)))
        except Exception as error:
            self.get_logger().error(f"perception callback failed: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YolopA3Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
