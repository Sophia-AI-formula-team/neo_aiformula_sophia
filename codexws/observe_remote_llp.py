#!/usr/bin/env python3
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2


TOPICS = {
    "camera": ("/aiformula_sensing/zed_node/left_image/undistorted", Image),
    "mask": ("/aiformula_perception/road_detector/mask_image", Image),
    "dynamic_roi": ("/aiformula_perception/lane_line_publisher/dynamic_roi_image", Image),
    "vehicle_fit": ("/aiformula_perception/lane_line_publisher/vehicle_fit_image", Image),
    "left": ("/aiformula_perception/lane_line_publisher/lane_lines/left", PointCloud2),
    "right": ("/aiformula_perception/lane_line_publisher/lane_lines/right", PointCloud2),
    "center": ("/aiformula_perception/lane_line_publisher/lane_lines/center", PointCloud2),
}


class Observer(Node):
    def __init__(self):
        super().__init__("codex_llp_observer")
        self.counts = {name: 0 for name in TOPICS}
        self.image_sizes = {name: [] for name in ("camera", "mask", "dynamic_roi", "vehicle_fit")}
        self.widths = {name: [] for name in ("left", "right", "center")}
        self._codex_subscriptions = []
        for name, (topic, msg_type) in TOPICS.items():
            callback = self._image_callback(name) if msg_type is Image else self._cloud_callback(name)
            self._codex_subscriptions.append(
                self.create_subscription(msg_type, topic, callback, qos_profile_sensor_data)
            )

    def _image_callback(self, name):
        def callback(msg):
            self.counts[name] += 1
            self.image_sizes[name].append([int(msg.width), int(msg.height)])
        return callback

    def _cloud_callback(self, name):
        def callback(msg):
            self.counts[name] += 1
            self.widths[name].append(int(msg.width))
        return callback


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=22.0)
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    args = parser.parse_args()
    rclpy.init()
    node = Observer()
    startup_deadline = time.monotonic() + args.startup_timeout
    while rclpy.ok() and node.counts["camera"] == 0 and time.monotonic() < startup_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    first_camera_received = node.counts["camera"] > 0
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    report = {
        "duration_seconds": args.duration,
        "first_camera_received": first_camera_received,
        "counts": node.counts,
        "image_sizes": {name: sorted({tuple(size) for size in sizes}) for name, sizes in node.image_sizes.items()},
        "cloud_widths": {
            name: {"min": min(widths) if widths else None, "max": max(widths) if widths else None,
                   "unique": sorted(set(widths))}
            for name, widths in node.widths.items()
        },
    }
    print(json.dumps(report, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
