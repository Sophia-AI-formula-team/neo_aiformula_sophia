"""Join the stock detector and camera by source stamp. Publish only private OA 1B topics."""
import argparse
from collections import OrderedDict
import json
import time

from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from .traffic_signal import SignalTracker


RAW = "/aiformula_sensing/zed_node/left_image/undistorted"
DETECTIONS = "/aiformula_perception/yolop_a3/detections"
ODOMETRY = "/aiformula_sensing/gyro_odometry_publisher/odom"
STATUS = "/semantic_drive/map_assist_045/status"
FRAME = "zed_left_camera_optical_frame"
PRIVATE = "/oa_1b"
MAX_AGE_NS = 500_000_000


def stamp_ns(sec, nanosec):
    if type(sec) is not int or type(nanosec) is not int or sec < 0 or not 0 <= nanosec < 10**9:
        return 0
    return sec * 10**9 + nanosec


class Observer(Node):
    def __init__(self, threshold):
        super().__init__("oa_1b_observer")
        self.threshold = threshold
        self.tracker = SignalTracker()
        self.bridge = CvBridge()
        self.raw = OrderedDict()
        self.detections = OrderedDict()
        self.started = False
        self.detection_output = self.create_publisher(String, PRIVATE + "/detections", 10)
        self.light_output = self.create_publisher(String, PRIVATE + "/traffic_light/state", 10)
        self.stop_output = self.create_publisher(Bool, PRIVATE + "/flag_stop", 10)
        self.create_subscription(Image, RAW, self.receive_image, qos_profile_sensor_data)
        self.create_subscription(String, DETECTIONS, self.receive_detection, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOMETRY, self.receive_odom, qos_profile_sensor_data)
        self.create_subscription(String, STATUS, self.receive_status, 10)

    def fresh(self, stamp):
        return stamp > 0 and 0 <= self.get_clock().now().nanoseconds - stamp <= MAX_AGE_NS

    def receive_odom(self, message):
        stamp = message.header.stamp
        if self.fresh(stamp_ns(stamp.sec, stamp.nanosec)):
            position = message.pose.pose.position
            velocity = message.twist.twist.linear
            self.tracker.odometry(position.x, position.y, (velocity.x**2 + velocity.y**2)**.5, time.monotonic())

    def receive_status(self, message):
        try:
            data = json.loads(message.data)
            if isinstance(data, dict):
                self.tracker.status(data, time.monotonic())
        except ValueError:
            pass

    def receive_image(self, message):
        if message.header.frame_id == FRAME:
            stamp = message.header.stamp
            self.join(self.raw, stamp_ns(stamp.sec, stamp.nanosec), message)

    def receive_detection(self, message):
        try:
            data = json.loads(message.data)
            header = data["header"]
            source = header["stamp"]
            if (header["frame_id"] == FRAME and data["traffic_light_state"] in ("red", "green", "unknown")
                    and type(data["red_pixels_count"]) is int and data["red_pixels_count"] >= 0
                    and isinstance(data["detections"], list)):
                self.join(self.detections, stamp_ns(source["sec"], source["nanosec"]), data)
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn("Rejected malformed AV314 detection")

    def join(self, buffer, stamp, item):
        if not self.fresh(stamp):
            return
        buffer[stamp] = item
        if len(buffer) > 16:
            buffer.popitem(last=False)
        if stamp in self.raw and stamp in self.detections:
            self.publish(self.raw.pop(stamp), self.detections.pop(stamp), stamp)

    def publish(self, message, data, stamp):
        owners = [(endpoint.node_namespace, endpoint.node_name)
                  for endpoint in self.get_publishers_info_by_topic(DETECTIONS)]
        if owners != [("/aiformula_perception", "yolop_a3_perception")]:
            return
        if not self.tracker.accept_stamp(stamp, self.get_clock().now().nanoseconds):
            return
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        corrected = self.tracker.correct(image, data, stamp, time.monotonic())
        if not self.fresh(stamp):
            self.tracker.consecutive = 0
            return
        corrected["oa_1b"]["threshold"] = self.threshold
        self.detection_output.publish(String(data=json.dumps(corrected, allow_nan=False)))
        self.light_output.publish(String(data=corrected["traffic_light_state"]))
        self.stop_output.publish(Bool(data=corrected["red_pixels_count"] >= self.threshold))
        if not self.started:
            self.started = True
            print("OA1B_CAMERA_READY", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--threshold", type=int, default=50)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    if args.threshold < 1:
        parser.error("--threshold must be positive")
    if args.check:
        return
    rclpy.init(args=[])
    node = None
    try:
        node = Observer(args.threshold)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
