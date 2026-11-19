"""Publish stop-position flags from fresh A3 red-pixel observations."""
import argparse
from collections import Counter
import json
import sys

INPUT = "/aiformula_perception/yolop_a3/detections"
OUTPUT = "/flag_stop"
EXPECTED_FRAME = "zed_left_camera_optical_frame"
MAX_AGE_NS = 500_000_000


def inspect_observation(text, now_ns):
    try:
        payload = json.loads(text)
        header = payload["header"]
        stamp = header["stamp"]
        sec, nanosec = stamp["sec"], stamp["nanosec"]
        count = payload["red_pixels_count"]
        if (
            header["frame_id"] != EXPECTED_FRAME
            or type(sec) is not int or not 0 <= sec <= 2147483647
            or type(nanosec) is not int or not 0 <= nanosec < 1_000_000_000
            or type(count) is not int or count < 0
        ):
            return None, "invalid_header_or_pixel_count"
        source = sec * 1_000_000_000 + nanosec
        if source > 0 and 0 <= now_ns - source <= MAX_AGE_NS:
            return (source, count), "ok"
        return None, "stale_or_future_image"
    except (KeyError, TypeError, ValueError):
        return None, "malformed_detections"


def observation(text, now_ns):
    return inspect_observation(text, now_ns)[0]


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool, String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("threshold", type=int)
    args = parser.parse_args()
    if args.threshold <= 0:
        parser.error("threshold must be positive")

    class PixelStop(Node):
        def __init__(self):
            super().__init__("a3_pixel_stop_position")
            self.output = self.create_publisher(Bool, OUTPUT, 1)
            self.create_subscription(String, INPUT, self.receive, qos_profile_sensor_data)
            self.last_source = 0
            self.counts = Counter()
            self.latest_pixels = None
            self.latest_source = None
            self.audit = self.create_publisher(String, "/semantic_drive/map_assist_045/pixel_stop_status", 1)
            self.create_timer(1.0, self.report)

        def report(self):
            age = None if self.latest_source is None else (
                self.get_clock().now().nanoseconds - self.latest_source) / 1e6
            data = dict(threshold=args.threshold, latest_pixels=self.latest_pixels, age_ms=age,
                        counts=self.counts, detections_publishers=self.count_publishers(INPUT),
                        flag_publishers=self.count_publishers(OUTPUT))
            self.audit.publish(String(data=json.dumps(data)))
            if self.latest_source is None or age > MAX_AGE_NS / 1e6:
                self.get_logger().warn("No fresh pixel-stop output: " + json.dumps(data))


        def receive(self, message):
            if self.count_publishers(INPUT) != 1 or self.count_publishers(OUTPUT) != 1:
                self.counts["publisher_count_mismatch"] += 1
                return
            item, reason = inspect_observation(message.data, self.get_clock().now().nanoseconds)
            if item is None:
                self.counts[reason] += 1
                return
            if item[0] <= self.last_source:
                self.counts["nonmonotonic_image"] += 1
                return
            self.last_source, count = item
            self.latest_pixels, self.latest_source = count, self.last_source
            at_stop = count >= args.threshold
            self.counts["at_stop" if at_stop else "below_threshold"] += 1
            self.output.publish(Bool(data=at_stop))

    rclpy.init(args=[sys.argv[0]])
    node = None
    try:
        node = PixelStop()
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
