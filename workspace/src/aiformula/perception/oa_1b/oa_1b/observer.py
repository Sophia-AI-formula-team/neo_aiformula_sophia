"""Join the stock detector and camera by source stamp. Publish only private OA 1B topics."""
import argparse
from collections import OrderedDict
import hashlib
import inspect
import json
from pathlib import Path
import time

from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from yolop_a3_runtime import color_semantics, shared_runtime, traffic_light_policy

from .traffic_signal import SignalTracker


RAW = "/aiformula_sensing/zed_node/left_image/undistorted"
DETECTIONS = "/aiformula_perception/yolop_a3/detections"
ODOMETRY = "/aiformula_sensing/gyro_odometry_publisher/odom"
STATUS = "/semantic_drive/map_assist_045/status"
FRAME = "zed_left_camera_optical_frame"
PRIVATE = "/oa_1b"
MAX_AGE_NS = 500_000_000
WEIGHTS = "a411c983b661d6b26217019a968251c89a6b66c95400e4e7cb78f04a644ca10c"
PINNED = {
    shared_runtime: "d5d43a905b05c6a8f02721485025f9077cebf5c81b3fe3f8e69897ee3e53601d",
    color_semantics: "483da35887ae28931cf1e473843faba74e49201634e1493ccdf709781492c285",
    traffic_light_policy: "8eb8fa7f0871d94946e59d15cd9e1ddf182c5238b1719005d024684fb0e982d6",
}


def check_file(path, expected):
    with Path(path).open("rb") as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    if digest.hexdigest() != expected:
        raise RuntimeError(f"{path} SHA-256 {digest.hexdigest()} differs from expected {expected}")


def check_runtime(weights):
    check_file(weights, WEIGHTS)
    for module, digest in PINNED.items():
        check_file(inspect.getfile(module), digest)


def stamp_ns(sec, nanosec):
    if type(sec) is not int or type(nanosec) is not int or sec < 0 or not 0 <= nanosec < 10**9:
        return 0
    return sec * 10**9 + nanosec


class Candidates:
    def __init__(self, weights):
        self.detector = shared_runtime.SharedFPNPerception(weights, device="cuda", score_threshold=.50)
        self.detector.model.register_forward_hook(self.capture)
        self.boxes = []

    def capture(self, _module, _args, output):
        objects = output[1][0]
        lamps = objects["labels"] == 10
        self.boxes = list(zip(objects["boxes"][lamps].cpu().tolist(), objects["scores"][lamps].cpu().tolist()))

    def infer(self, image):
        self.boxes = []
        self.detector.predict(image)
        return self.boxes


class Observer(Node):
    def __init__(self, weights, threshold):
        super().__init__("oa_1b_observer")
        self.threshold = threshold
        self.candidates = Candidates(weights)
        self.tracker = SignalTracker()
        self.bridge = CvBridge()
        self.raw = OrderedDict()
        self.detections = OrderedDict()
        self.started = False
        self.warmed = False
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
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if not self.warmed:
            # Initialize the CUDA path before the first live decision. Cold first inference can expire the input.
            self.candidates.infer(image)
            self.warmed = True
        if not self.tracker.accept_stamp(stamp, self.get_clock().now().nanoseconds):
            return
        if data["traffic_light_state"] == "unknown":
            candidates = lambda: self.candidates.infer(image)
        else:
            candidates = [(box["box_xyxy"], box["score"]) for box in data["detections"] if box["class_id"] == 10]
        corrected = self.tracker.correct(image, data, candidates, stamp, time.monotonic(),
                                         color_semantics.classify_traffic_light)
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
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--threshold", type=int, default=50)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    if args.threshold < 1:
        parser.error("--threshold must be positive")
    check_runtime(args.weights)
    if args.check:
        return
    rclpy.init(args=[])
    node = None
    try:
        node = Observer(args.weights, args.threshold)
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
