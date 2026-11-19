"""Standalone B1.4 sensor node. The image timestamp belongs to the camera."""
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from .geometry import HONDA, UNVERIFIED, load_projection
from .frame_codec import encode_frame
from .inference import Predictor
from .weights import default_path

CAMERA = '/aiformula_sensing/zed_node/left_image/undistorted'
FRAME = '/concept/perception/frame'
OVERLAY = '/concept/perception/overlay'
ROAD_BLEND = (.57 * np.arange(256)[:, None, None] + .43 * np.array([218, 132, 42])).astype(np.uint8)


def draw_overlay(image, road, lane, det):
    out = image.copy()
    # Every uint8 channel has only 256 possible blends. Keep the original
    # truncation while avoiding several float64 arrays on each camera frame.
    cv2.copyTo(cv2.LUT(image, ROAD_BLEND), road.view(np.uint8), out)
    out[lane] = (30, 143, 255)
    for box, label, score in zip(det['boxes'], det['labels'], det['scores']):
        if label == 91 and score >= .5:
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(out, (x1, y1), (x2, y2), (80, 40, 240), 2)
            cv2.putText(out, 'cone %.2f' % score, (x1, max(14, y1 - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (80, 40, 240), 1, cv2.LINE_AA)
    return out


class CameraNode(Node):
    def __init__(self, frame_consumer=None, image_group=None):
        super().__init__('concept_yolop')
        self.frame_consumer = frame_consumer
        for name, default in [('weights', str(default_path())), ('device', 'cuda'),
                              ('camera_info_topic', '/aiformula_sensing/zed_node/left/camera_info'),
                              ('calibration', ''), ('image_topic', CAMERA), ('frame_topic', FRAME), ('overlay_topic', OVERLAY)]:
            self.declare_parameter(name, default)
        self.predictor = Predictor(self.get_parameter('weights').value or default_path(), self.get_parameter('device').value)
        if self.predictor.device.type == 'cuda':
            # Initialize CUDA, cuDNN and detection kernels before subscribing to the
            # camera. predict() copies its outputs to CPU, completing the warm-up.
            self.predictor.predict(np.zeros((360, 640, 3), dtype=np.uint8))
        self.bridge = CvBridge()
        self.last_error_at = float('-inf')
        self.frames = self.create_publisher(String, self.get_parameter('frame_topic').value, 1)
        overlay_topic = self.get_parameter('overlay_topic').value
        self.overlay = self.create_publisher(Image, overlay_topic, 1)
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.compressed_overlay = self.create_publisher(CompressedImage, overlay_topic + '/compressed', qos)
        self.last_bag_error_at = float('-inf')
        self.calibration = None
        self.info = None
        path = self.get_parameter('calibration').value
        if path:
            self.calibration = load_projection(path)
            self.info_subscription = self.create_subscription(
                CameraInfo, self.get_parameter('camera_info_topic').value, self.on_info, qos, callback_group=image_group)
        self.images = self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image, qos, callback_group=image_group)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        try:
            received = self.get_clock().now().nanoseconds
            start = time.monotonic_ns()
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            matched = self.calibration and self.calibration.matches(self.images.topic_name, msg, self.info)
            if self.calibration and self.calibration.profile == HONDA and not matched:
                if time.monotonic() - self.last_error_at >= 5.:
                    self.get_logger().warning('Waiting for matching ZED CameraInfo: topic=%s image=%s info=%s'
                                              % (self.images.topic_name, msg.header.frame_id,
                                                 self.info.header.frame_id if self.info else 'missing'))
                    self.last_error_at = time.monotonic()
                return
            if matched:
                bgr = self.calibration.rectify(bgr)
            prepared = time.monotonic_ns()
            bgr, road, lane, det = self.predictor.predict(bgr)
            inferred = time.monotonic_ns()
            if matched:
                road, lane, det = self.calibration.restrict_predictions(road, lane, det)
            stamp = msg.header.stamp
            profile = self.calibration.profile if matched else UNVERIFIED
            if self.frame_consumer is not None:
                self.frame_timing_ms = {
                    'capture_to_image_callback': (received - stamp.sec * 10**9 - stamp.nanosec) / 1e6,
                    'decode_and_projection': (prepared - start) / 1e6,
                    'inference': (inferred - prepared) / 1e6,
                    'restrict_predictions': (time.monotonic_ns() - inferred) / 1e6,
                }
                self.frame_consumer(stamp, msg.header.frame_id, profile, bgr, road, lane, det)
            if self.frame_consumer is None or self.frames.get_subscription_count():
                wire = encode_frame(stamp.sec, stamp.nanosec, msg.header.frame_id, bgr, road, lane, det, profile)
                self.frames.publish(String(data=wire))
            display = self.overlay.get_subscription_count()
            record = self.compressed_overlay.get_subscription_count()
            if display or record:
                overlay = draw_overlay(bgr, road, lane, det)
                if display:
                    preview = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                    preview.header = msg.header
                    self.overlay.publish(preview)
                if record:
                    self.publish_compressed(overlay, msg.header)
        except Exception as exc:
            # rclpy Foxy has no portable throttle_duration_sec keyword.
            if time.monotonic() - self.last_error_at >= 2.0:
                self.get_logger().error('Camera frame dropped: %s' % exc)
                self.last_error_at = time.monotonic()

    def publish_compressed(self, overlay, header):
        try:
            ok, jpeg = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                raise ValueError('JPEG encoder returned no image')
            self.compressed_overlay.publish(CompressedImage(
                header=header, format='bgr8; jpeg compressed bgr8', data=jpeg.tobytes()))
        except (cv2.error, ValueError, RuntimeError) as exc:
            if time.monotonic() - self.last_bag_error_at >= 2.0:
                self.get_logger().warning('Bag camera frame dropped: %s' % exc)
                self.last_bag_error_at = time.monotonic()


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
