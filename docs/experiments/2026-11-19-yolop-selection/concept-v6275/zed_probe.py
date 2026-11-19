"""Validate live ZED-to-Concept wiring with counters only."""
from collections import Counter
from datetime import datetime
import json
import os
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from concept_yolop.frame_codec import parse_frame_header

if os.environ.get("ROS_DOMAIN_ID") != "197" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise SystemExit("Use localhost-only domain 197")

rclpy.init()
node = rclpy.create_node("concept_zed_acceptance")
counts = Counter()
dimensions = {}
camera_headers, output_headers = set(), set()
geometry = set()
detections = []


def header(stamp, frame_id):
    return (stamp.sec * 10**9 + stamp.nanosec, frame_id)


def camera(message):
    counts["camera"] += 1
    dimensions["camera"] = [message.width, message.height, message.encoding]
    camera_headers.add(header(message.header.stamp, message.header.frame_id))


def info(message):
    counts["camera_info"] += 1
    dimensions["camera_info"] = [message.width, message.height]


def perception(message):
    payload, stamp, frame_id, geometry_profile = parse_frame_header(message.data)
    assert payload["road_png"] and payload["lane_png"]
    assert len(payload["boxes"]) == len(payload["scores"]) == len(payload["labels"])
    counts["perception"] += 1
    detections.append(len(payload["boxes"]))
    output_headers.add((stamp, frame_id))
    geometry.add(geometry_profile)
    dimensions["perception"] = [payload["width"], payload["height"]]


def jpeg(message):
    assert len(message.data) > 0 and "jpeg" in message.format
    counts["jpeg"] += 1


camera_topic = "/aiformula_sensing/zed_node/left_image/undistorted"
info_topic = "/aiformula_sensing/zed_node/left/camera_info"
node.create_subscription(Image, camera_topic, camera, qos_profile_sensor_data)
node.create_subscription(CameraInfo, info_topic, info, qos_profile_sensor_data)
node.create_subscription(String, "/concept/perception/frame", perception, qos_profile_sensor_data)
node.create_subscription(CompressedImage, "/concept/perception/overlay/compressed", jpeg, qos_profile_sensor_data)

try:
    deadline = time.monotonic() + 35
    while time.monotonic() < deadline and min(counts["camera"], counts["perception"], counts["jpeg"]) < 12:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = dict(
        time_jst=datetime.now().astimezone().isoformat(timespec="seconds"),
        counts=dict(counts), dimensions=dimensions, geometry_profiles=sorted(geometry),
        max_detections=max(detections, default=0), matched_headers=len(camera_headers & output_headers),
        camera_info_topics=[name for name, kinds in node.get_topic_names_and_types()
                            if "sensor_msgs/msg/CameraInfo" in kinds],
    )
    print(json.dumps(result), flush=True)
    assert min(counts["camera"], counts["perception"], counts["jpeg"]) >= 12, result
    assert result["matched_headers"] > 0, result
finally:
    node.destroy_node()
    rclpy.shutdown()
