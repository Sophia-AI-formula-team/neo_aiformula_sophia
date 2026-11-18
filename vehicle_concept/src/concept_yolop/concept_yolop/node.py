"""Standalone B1.4 sensor node. The image timestamp belongs to the camera."""
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .geometry import CALIBRATED, UNVERIFIED, CalibrationReceipt
from .frame_codec import encode_frame
from .inference import Predictor
from .weights import default_path

CAMERA = '/aiformula_sensing/zed_node/left_image/undistorted'
FRAME = '/concept/perception/frame'
OVERLAY = '/concept/perception/overlay'


def draw_overlay(image, road, lane, det):
    out = image.copy()
    out[road] = .57 * out[road] + .43 * np.array([218, 132, 42])
    out[lane] = (30, 143, 255)
    for box, label, score in zip(det['boxes'], det['labels'], det['scores']):
        if label == 91 and score >= .5:
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(out, (x1, y1), (x2, y2), (80, 40, 240), 2)
            cv2.putText(out, 'cone %.2f' % score, (x1, max(14, y1 - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (80, 40, 240), 1, cv2.LINE_AA)
    return out


class CameraNode(Node):
    def __init__(self):
        super().__init__('concept_yolop')
        for name, default in [('weights', str(default_path())), ('device', 'cuda'),
                              ('camera_info_topic', '/aiformula_sensing/zed_node/left/camera_info'),
                              ('calibration', ''), ('image_topic', CAMERA), ('frame_topic', FRAME), ('overlay_topic', OVERLAY)]:
            self.declare_parameter(name, default)
        self.predictor = Predictor(self.get_parameter('weights').value or default_path(), self.get_parameter('device').value)
        self.bridge = CvBridge()
        self.last_error_at = float('-inf')
        self.frames = self.create_publisher(String, self.get_parameter('frame_topic').value, 1)
        self.overlay = self.create_publisher(Image, self.get_parameter('overlay_topic').value, 1)
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.calibration = None
        self.info = None
        path = self.get_parameter('calibration').value
        if path:
            self.calibration = CalibrationReceipt(path)
            self.info_subscription = self.create_subscription(
                CameraInfo, self.get_parameter('camera_info_topic').value, self.on_info, qos)
        self.images = self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image, qos)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            bgr, road, lane, det = self.predictor.predict(bgr)
            stamp = msg.header.stamp
            verified = self.calibration and self.calibration.matches(self.images.topic_name, msg, self.info)
            wire = encode_frame(stamp.sec, stamp.nanosec, msg.header.frame_id, bgr, road, lane, det,
                                CALIBRATED if verified else UNVERIFIED)
            self.frames.publish(String(data=wire))
            if self.overlay.get_subscription_count():
                preview = self.bridge.cv2_to_imgmsg(draw_overlay(bgr, road, lane, det), encoding='bgr8')
                preview.header = msg.header
                self.overlay.publish(preview)
        except Exception as exc:
            # rclpy Foxy has no portable throttle_duration_sec keyword.
            if time.monotonic() - self.last_error_at >= 2.0:
                self.get_logger().error('Camera frame dropped: %s' % exc)
                self.last_error_at = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
