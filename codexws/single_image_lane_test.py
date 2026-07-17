#!/usr/bin/env python3
import json
import math
import os
import struct
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2


IMAGE_TOPIC = "/aiformula_sensing/zed_node/left_image/undistorted"
MASK_TOPIC = "/aiformula_perception/road_detector/mask_image"
OUTPUT_ROOT = os.environ.get("CODEX_OUTPUT_ROOT", "/aiformula_perception/lane_line_publisher")
CLOUD_TOPICS = {
    "left": OUTPUT_ROOT + "/lane_lines/left",
    "center": OUTPUT_ROOT + "/lane_lines/center",
    "right": OUTPUT_ROOT + "/lane_lines/right",
}
DIAGNOSTIC_IMAGE_TOPICS = {
    "road_mask": MASK_TOPIC,
    "road_annotated": "/aiformula_visualization/road_detector/annotated_mask_image",
    "dynamic_roi": OUTPUT_ROOT + "/dynamic_roi_image",
    "linear_fit": OUTPUT_ROOT + "/linear_fit_image",
    "vehicle_fit": OUTPUT_ROOT + "/vehicle_fit_image",
}


class SingleImageLaneTest(Node):
    def __init__(self, image_path, duration_sec):
        super().__init__("codex_single_image_lane_test")
        self.duration_sec = duration_sec
        self.started = time.monotonic()
        self.bridge = CvBridge()
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("could not read image: " + image_path)
        self.image_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_message.header.frame_id = "zed_left_camera_optical_frame"

        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        reliable.durability = DurabilityPolicy.VOLATILE
        self.publisher = self.create_publisher(Image, IMAGE_TOPIC, reliable)
        self.diagnostic_images = {}
        self.image_stats = {}
        self.image_subscriptions = []
        for name, topic in DIAGNOSTIC_IMAGE_TOPICS.items():
            self.image_subscriptions.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, image_name=name: self.on_diagnostic_image(image_name, msg),
                    reliable,
                )
            )
        self.cloud_stats = {
            name: {
                "messages": 0,
                "max_declared_points": 0,
                "max_finite_xyz_points": 0,
                "max_nonzero_xyz_points": 0,
                "last_frame_id": "",
                "last_data_bytes": 0,
            }
            for name in CLOUD_TOPICS
        }
        self.cloud_subscriptions = []
        for name, topic in CLOUD_TOPICS.items():
            self.cloud_subscriptions.append(
                self.create_subscription(
                    PointCloud2,
                    topic,
                    lambda msg, lane=name: self.on_cloud(lane, msg),
                    reliable,
                )
            )
        self.mask_messages = 0
        self.last_mask = {}
        self.published_images = 0
        self.create_timer(0.2, self.publish_image)

    def publish_image(self):
        self.image_message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.image_message)
        self.published_images += 1

    def on_diagnostic_image(self, name, msg):
        if name == "road_mask":
            self.mask_messages += 1
            self.last_mask = {
                "width": msg.width,
                "height": msg.height,
                "encoding": msg.encoding,
                "data_bytes": len(msg.data),
                "frame_id": msg.header.frame_id,
            }
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as error:
            self.image_stats[name] = {"decode_error": str(error)}
            return
        self.diagnostic_images[name] = image.copy()
        flat = image.reshape(-1)
        unique, counts = __import__("numpy").unique(flat, return_counts=True)
        histogram = {
            str(int(value)): int(count)
            for value, count in zip(unique[:32], counts[:32])
        }
        self.image_stats[name] = {
            "width": msg.width,
            "height": msg.height,
            "encoding": msg.encoding,
            "data_bytes": len(msg.data),
            "frame_id": msg.header.frame_id,
            "nonzero_values": int(__import__("numpy").count_nonzero(image)),
            "min": int(image.min()) if image.size else None,
            "max": int(image.max()) if image.size else None,
            "histogram_first_32_values": histogram,
        }

    def on_cloud(self, lane, msg):
        stats = self.cloud_stats[lane]
        stats["messages"] += 1
        declared = int(msg.width) * int(msg.height)
        stats["max_declared_points"] = max(stats["max_declared_points"], declared)
        stats["last_frame_id"] = msg.header.frame_id
        stats["last_data_bytes"] = len(msg.data)

        field_offsets = {field.name: field.offset for field in msg.fields}
        if not all(axis in field_offsets for axis in ("x", "y", "z")) or msg.point_step <= 0:
            return
        byte_order = ">" if msg.is_bigendian else "<"
        finite = 0
        nonzero = 0
        available = min(declared, len(msg.data) // msg.point_step)
        for index in range(available):
            base = index * msg.point_step
            try:
                x = struct.unpack_from(byte_order + "f", msg.data, base + field_offsets["x"])[0]
                y = struct.unpack_from(byte_order + "f", msg.data, base + field_offsets["y"])[0]
                z = struct.unpack_from(byte_order + "f", msg.data, base + field_offsets["z"])[0]
            except (struct.error, IndexError):
                break
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                finite += 1
                if x != 0.0 or y != 0.0 or z != 0.0:
                    nonzero += 1
        stats["max_finite_xyz_points"] = max(stats["max_finite_xyz_points"], finite)
        stats["max_nonzero_xyz_points"] = max(stats["max_nonzero_xyz_points"], nonzero)

    def report(self, report_path):
        artifact_dir = os.path.dirname(report_path) or "."
        for name, image in self.diagnostic_images.items():
            cv2.imwrite(os.path.join(artifact_dir, "codex_" + name + ".png"), image)
        all_nonempty = all(
            stats["max_finite_xyz_points"] > 0 and stats["max_nonzero_xyz_points"] > 0
            for stats in self.cloud_stats.values()
        )
        return {
            "image_topic": IMAGE_TOPIC,
            "published_images": self.published_images,
            "mask_messages": self.mask_messages,
            "last_mask": self.last_mask,
            "diagnostic_images": self.image_stats,
            "clouds": self.cloud_stats,
            "all_three_clouds_nonempty": all_nonempty,
            "duration_sec": round(time.monotonic() - self.started, 3),
        }


def main():
    if len(sys.argv) < 2:
        print("usage: single_image_lane_test.py IMAGE [DURATION] [REPORT]", file=sys.stderr)
        return 64
    image_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
    report_path = sys.argv[3] if len(sys.argv) > 3 else "/tmp/codex_single_image_report.json"
    rclpy.init()
    node = SingleImageLaneTest(image_path, duration)
    try:
        while rclpy.ok() and time.monotonic() - node.started < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report(report_path)
        with open(report_path, "w", encoding="utf-8") as output:
            json.dump(report, output, indent=2, sort_keys=True)
            output.write("\n")
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report["all_three_clouds_nonempty"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
