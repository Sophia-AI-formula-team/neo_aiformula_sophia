#!/usr/bin/env python3
"""Observe live perception without publishing inputs or control commands.

Python 3.8 / ROS2 Foxy. Run in an environment providing rclpy and numpy.
--save-dir requires OpenCV and creates a new directory; existing paths are
rejected. This observer neither starts/stops other nodes nor reads any bag.
"""

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import sys
import time

import numpy as np


IMAGE_TOPICS = {
    "camera": "/aiformula_sensing/zed_node/left_image/undistorted",
    "lane_mask": "/aiformula_perception/road_detector/mask_image",
    "lane_overlay": "/aiformula_visualization/road_detector/annotated_mask_image",
    "obstacle_mask": "/cone_mask",
    "object_overlay": "/cone_bbox_image",
    "avoidance_debug": "/processed_image",
}
DETECTIONS_TOPIC = "/aiformula_perception/yolop_a2/detections"
SCALAR_TOPICS = {
    "avoidance_roi_count": "/cone_pixels_count",
    "avoidance_selected": "/point_flag",
}
POINT_TOPICS = {"point_" + key: "/oa_point_" + key for key in ("a", "b", "c")}
POINT_TOPICS["filtered_lane_pose"] = "/filtered_lane_pose"


def image_array(message):
    """Decode supported byte images while respecting per-row padding."""
    channels = {"mono8": 1, "8UC1": 1, "bgr8": 3, "rgb8": 3,
                "bgra8": 4, "rgba8": 4}.get(message.encoding)
    if channels is None:
        raise ValueError("unsupported encoding: " + message.encoding)
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or step < width * channels:
        raise ValueError("invalid width, height, or row step")
    data = np.frombuffer(message.data, dtype=np.uint8)
    if data.size < height * step:
        raise ValueError("image data shorter than height * step")
    array = data[:height * step].reshape(height, step)[:, :width * channels]
    shape = (height, width) if channels == 1 else (height, width, channels)
    return array.reshape(shape)


def header_record(header):
    return {
        "stamp_ns": int(header.stamp.sec) * 1000000000 + int(header.stamp.nanosec),
        "frame_id": header.frame_id,
    }


def new_stats(topic):
    return {"topic": topic, "messages": 0, "first": None, "last": None,
            "errors": 0, "error_samples": deque(maxlen=3)}


def observe(stats):
    now = time.monotonic()
    stats["messages"] += 1
    if stats["first"] is None:
        stats["first"] = now
    stats["last"] = now


def frequency_report(stats, duration):
    span = None if stats["first"] is None else stats["last"] - stats["first"]
    return {
        "topic": stats["topic"],
        "messages": stats["messages"],
        "receive_hz": round((stats["messages"] - 1) / span, 3) if span else None,
        "window_hz": round(stats["messages"] / duration, 3) if duration else None,
        "last_receive_age_seconds": round(time.monotonic() - stats["last"], 4)
        if stats["last"] is not None else None,
        "errors": stats["errors"],
        "error_samples": list(stats["error_samples"]),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--save-dir", type=Path, help="New directory for latest received PNGs")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be a positive finite number")

    # Refuse old directories before starting observation; PNG files use exclusive writes.
    cv2 = None
    if args.save_dir is not None:
        import cv2
        args.save_dir = args.save_dir.expanduser().absolute()
        try:
            args.save_dir.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            parser.error("--save-dir already exists; choose a new directory")

    import rclpy
    from geometry_msgs.msg import Pose2D
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, Int32, String

    rclpy.init(args=[])
    node = Node("live_perception_observer", enable_rosout=False, start_parameter_services=False)
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                     reliability=ReliabilityPolicy.BEST_EFFORT)
    images = {}
    for name, topic in IMAGE_TOPICS.items():
        stat = new_stats(topic)
        stat.update(formats=Counter(), headers=deque(maxlen=4096), ages=[],
                    latest=None, latest_array=None, latest_array_encoding=None,
                    nonzero=[], invalid_mask_frames=0,
                    decoded_frames=0, debug_cyan_pixels=[])
        images[name] = stat
    detections = new_stats(DETECTIONS_TOPIC)
    detections.update(labels={}, latest=[], latest_header=None, latest_timings_ms=None)
    scalars = {name: new_stats(topic) for name, topic in SCALAR_TOPICS.items()}
    points = {name: new_stats(topic) for name, topic in POINT_TOPICS.items()}
    for stat in list(scalars.values()) + list(points.values()):
        stat.update(latest=None, samples=deque(maxlen=8))
    for stat in scalars.values():
        stat.update(minimum=None, maximum=None, true_messages=0)

    def record_error(stat, error):
        stat["errors"] += 1
        stat["error_samples"].append(str(error))

    def on_image(name, message):
        stat = images[name]
        observe(stat)
        try:
            header = header_record(message.header)
            header.update(width=int(message.width), height=int(message.height))
            stat["headers"].append(header)
            stat["formats"][(int(message.width), int(message.height), message.encoding)] += 1
            stat["latest"] = dict(header, encoding=message.encoding, step=int(message.step))
            if header["stamp_ns"] != 0:
                stat["ages"].append((node.get_clock().now().nanoseconds - header["stamp_ns"]) / 1e9)
            array = image_array(message)
            stat["decoded_frames"] += 1
            if args.save_dir is not None:
                stat["latest_array"] = array.copy()
                stat["latest_array_encoding"] = message.encoding
            if name in ("lane_mask", "obstacle_mask"):
                if message.encoding != "mono8" or array.ndim != 2:
                    stat["invalid_mask_frames"] += 1
                elif np.any((array != 0) & (array != 255)):
                    stat["invalid_mask_frames"] += 1
                stat["nonzero"].append(int(np.count_nonzero(array)))
            if name == "avoidance_debug" and message.encoding == "bgr8":
                stat["debug_cyan_pixels"].append(int(np.count_nonzero(
                    np.all(array == np.array([255, 255, 0], dtype=np.uint8), axis=2))))
        except Exception as error:
            record_error(stat, error)

    def on_detections(message):
        observe(detections)
        try:
            payload = json.loads(message.data)
            latest = []
            for item in payload.get("detections", []):
                score = float(item["score"])
                if not math.isfinite(score):
                    raise ValueError("nonfinite detection score")
                label = str(item.get("semantic_label", item.get("class_name", "unknown")))
                summary = detections["labels"].setdefault(
                    label, {"observations": 0, "min_score": score, "max_score": score})
                summary["observations"] += 1
                summary["min_score"] = min(summary["min_score"], score)
                summary["max_score"] = max(summary["max_score"], score)
                latest.append({"label": label, "score": score,
                               "class_id": item.get("class_id"), "box_xyxy": item.get("box_xyxy")})
            detections["latest"] = latest[:30]
            detections["latest_header"] = payload.get("header")
            detections["latest_timings_ms"] = payload.get("timings_ms")
        except Exception as error:
            record_error(detections, error)

    def on_scalar(name, message):
        stat = scalars[name]
        observe(stat)
        stat["latest"] = message.data
        stat["samples"].append(message.data)
        stat["minimum"] = message.data if stat["minimum"] is None else min(stat["minimum"], message.data)
        stat["maximum"] = message.data if stat["maximum"] is None else max(stat["maximum"], message.data)
        if name == "avoidance_selected" and message.data:
            stat["true_messages"] += 1

    def on_point(name, message):
        stat = points[name]
        observe(stat)
        value = {"x": float(message.x), "y": float(message.y), "theta": float(message.theta)}
        if not all(math.isfinite(x) for x in value.values()):
            record_error(stat, "nonfinite Pose2D")
            return
        stat["latest"] = value
        stat["samples"].append(value)

    subscriptions = []
    for name, topic in IMAGE_TOPICS.items():
        subscriptions.append(node.create_subscription(
            Image, topic, lambda message, name=name: on_image(name, message), qos))
    subscriptions.append(node.create_subscription(String, DETECTIONS_TOPIC, on_detections, qos))
    for name, topic in SCALAR_TOPICS.items():
        message_type = Bool if name == "avoidance_selected" else Int32
        subscriptions.append(node.create_subscription(
            message_type, topic, lambda message, name=name: on_scalar(name, message), qos))
    for name, topic in POINT_TOPICS.items():
        subscriptions.append(node.create_subscription(
            Pose2D, topic, lambda message, name=name: on_point(name, message), qos))

    started = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, args.duration - (time.monotonic() - started))))
    except KeyboardInterrupt:
        interrupted = True
    duration = time.monotonic() - started
    report = {"observation_seconds": round(duration, 3), "interrupted": interrupted,
              "images": {}, "detections": {}, "avoidance": {},
              "notes": [
                  "Receive rates are observer rates, not guaranteed publisher throughput.",
                  "Header ages use the observer ROS clock; negative ages indicate clock disagreement.",
                  "Unmatched camera stamps can result from observer DDS loss or startup timing.",
                  "Cyan pixels are visual evidence only; boxes can share this color. They do not prove a spline or a Path message.",
                  "This tool publishes no application messages and does not run bag playback or start/stop other nodes.",
              ]}
    camera_headers = {record["stamp_ns"]: record for record in images["camera"]["headers"]}
    try:
        for name, stat in images.items():
            result = frequency_report(stat, duration)
            result["publishers"] = node.count_publishers(stat["topic"])
            result["formats"] = [dict(width=w, height=h, encoding=e, frames=count)
                                 for (w, h, e), count in stat["formats"].items()]
            result["latest"] = stat["latest"]
            result["header_samples"] = list(stat["headers"])[-3:]
            ages = stat["ages"]
            result["header_age_seconds"] = None if not ages else {
                "latest": round(ages[-1], 4), "min": round(min(ages), 4),
                "max": round(max(ages), 4), "mean": round(sum(ages) / len(ages), 4)}
            if name != "camera":
                matches = [(record, camera_headers[record["stamp_ns"]])
                           for record in stat["headers"] if record["stamp_ns"] in camera_headers]
                result["camera_header_comparison"] = {
                    "compared": len(matches), "unmatched": len(stat["headers"]) - len(matches),
                    "frame_id_mismatches": sum(a["frame_id"] != b["frame_id"] for a, b in matches),
                    "dimension_mismatches": sum((a["width"], a["height"]) != (b["width"], b["height"]) for a, b in matches)}
            if name in ("lane_mask", "obstacle_mask"):
                counts = stat["nonzero"]
                result["binary_mono8_all_observed_frames"] = (
                    stat["invalid_mask_frames"] == 0 and stat["errors"] == 0
                    if stat["decoded_frames"] else None)
                result["nonzero_pixels"] = None if not counts else {
                    "latest": counts[-1], "min": min(counts), "max": max(counts),
                    "nonempty_frames": sum(count > 0 for count in counts)}
            if name == "avoidance_debug":
                counts = stat["debug_cyan_pixels"]
                result["cyan_pixel_count"] = None if not counts else {"latest": counts[-1], "max": max(counts)}
            if args.save_dir is not None and stat["latest_array"] is not None:
                array = stat["latest_array"]
                encoding = stat["latest_array_encoding"]
                if encoding == "rgb8":
                    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
                elif encoding == "rgba8":
                    array = cv2.cvtColor(array, cv2.COLOR_RGBA2BGRA)
                success, encoded = cv2.imencode(".png", array)
                if not success:
                    raise RuntimeError("PNG encoding failed for " + name)
                destination = args.save_dir / (name + ".png")
                with destination.open("xb") as stream:
                    stream.write(encoded.tobytes())
                result["saved_image"] = str(destination)
            report["images"][name] = result
        report["detections"] = dict(frequency_report(detections, duration),
            publishers=node.count_publishers(DETECTIONS_TOPIC), labels=detections["labels"],
            latest=detections["latest"], latest_header=detections["latest_header"],
            latest_timings_ms=detections["latest_timings_ms"])
        for name, stat in list(scalars.items()) + list(points.items()):
            report["avoidance"][name] = dict(frequency_report(stat, duration),
                publishers=node.count_publishers(stat["topic"]), latest=stat["latest"],
                samples=list(stat["samples"]))
            if name in scalars:
                report["avoidance"][name].update(minimum=stat["minimum"], maximum=stat["maximum"])
            if name == "avoidance_selected":
                report["avoidance"][name]["true_messages"] = stat["true_messages"]
        print(json.dumps(report, ensure_ascii=False, separators=(",", ":"), allow_nan=False))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
