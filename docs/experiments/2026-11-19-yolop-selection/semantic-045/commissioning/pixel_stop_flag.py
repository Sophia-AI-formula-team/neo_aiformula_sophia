"""Publish stop-position flags from fresh A3 red-pixel observations."""
import argparse
import json
import sys

INPUT = "/aiformula_perception/yolop_a3/detections"
OUTPUT = "/flag_stop"
EXPECTED_FRAME = "zed_left_camera_optical_frame"
MAX_AGE_NS = 500_000_000


def observation(text, now_ns):
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
            return None
        source = sec * 1_000_000_000 + nanosec
        if source > 0 and 0 <= now_ns - source <= MAX_AGE_NS:
            return source, count
    except (KeyError, TypeError, ValueError):
        return None
    return None


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
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
            self.create_subscription(String, INPUT, self.receive, 1)
            self.last_source = 0

        def receive(self, message):
            if self.count_publishers(INPUT) != 1 or self.count_publishers(OUTPUT) != 1:
                return
            item = observation(message.data, self.get_clock().now().nanoseconds)
            if item is None or item[0] <= self.last_source:
                return
            self.last_source, count = item
            self.output.publish(Bool(data=count >= args.threshold))

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
