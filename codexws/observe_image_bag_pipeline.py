#!/usr/bin/env python3
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2


class Observer(Node):
    def __init__(self):
        super().__init__("codex_image_bag_observer")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.counts = {"image": 0, "mask": 0, "vehicle_fit": 0}
        self.image_sizes = {}
        self.clouds = {
            lane: {"messages": 0, "max_points": 0, "max_data_bytes": 0}
            for lane in ("left", "center", "right")
        }
        self.subscriptions_ = [
            self.create_subscription(
                Image,
                "/aiformula_sensing/zed_node/left_image/undistorted",
                lambda msg: self.on_image("image", msg),
                qos,
            ),
            self.create_subscription(
                Image,
                "/aiformula_perception/road_detector/mask_image",
                lambda msg: self.on_image("mask", msg),
                qos,
            ),
            self.create_subscription(
                Image,
                "/aiformula_perception/lane_line_publisher/vehicle_fit_image",
                lambda msg: self.on_image("vehicle_fit", msg),
                qos,
            ),
        ]
        for lane in self.clouds:
            self.subscriptions_.append(
                self.create_subscription(
                    PointCloud2,
                    "/aiformula_perception/lane_line_publisher/lane_lines/" + lane,
                    lambda msg, name=lane: self.on_cloud(name, msg),
                    qos,
                )
            )

    def increment(self, name):
        self.counts[name] += 1

    def on_image(self, name, msg):
        self.increment(name)
        self.image_sizes[name] = {
            "width": int(msg.width),
            "height": int(msg.height),
            "encoding": msg.encoding,
            "data_bytes": len(msg.data),
        }

    def on_cloud(self, lane, msg):
        stats = self.clouds[lane]
        stats["messages"] += 1
        stats["max_points"] = max(stats["max_points"], int(msg.width) * int(msg.height))
        stats["max_data_bytes"] = max(stats["max_data_bytes"], len(msg.data))


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
    rclpy.init()
    node = Observer()
    started = time.monotonic()
    try:
        while time.monotonic() - started < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = {
            "duration_sec": round(time.monotonic() - started, 3),
            "counts": node.counts,
            "image_sizes": node.image_sizes,
            "clouds": node.clouds,
        }
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report["counts"]["image"] > 0 else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
