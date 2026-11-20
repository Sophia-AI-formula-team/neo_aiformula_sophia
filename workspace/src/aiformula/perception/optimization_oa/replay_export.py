"""Exercise the planning core on JSONL exported from the vehicle bag."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
from statistics import median
import struct
from time import perf_counter

import numpy as np

from optimization_oa.geometry import GroundCamera, rotation_matrix
from optimization_oa.inputs import (
    Cloud, align_clouds, boxes_to_mask, cloud_xy, decode_detections, matched_clouds,
)
from optimization_oa.planner import PlannerConfig, plan


def records(folder, topic):
    path = folder / "jsonl" / (topic_key(topic) + ".jsonl")
    with path.open(encoding="utf-8") as source:
        for line in source:
            yield json.loads(line)["data"]


def topic_key(topic):
    return topic.strip("/").replace("/", "__")


def csv_rows(folder, topic):
    with (folder / "csv" / (topic_key(topic) + ".csv")).open(encoding="utf-8", newline="") as source:
        yield from csv.DictReader(source)


def stamp_ns(header):
    stamp = header["stamp"]
    return stamp["sec"] * 10**9 + stamp["nanosec"]


def rigid_transform(record):
    transform = record["transform"]
    offset = transform["translation"]
    rotation = transform["rotation"]
    matrix = np.eye(4)
    matrix[:3, 3] = [offset[key] for key in "xyz"]
    matrix[:3, :3] = rotation_matrix([rotation[key] for key in "xyzw"])
    return matrix


def camera_model(folder):
    topic = "/aiformula_sensing/zed_node/left/camera_info"
    if (folder / "jsonl" / (topic_key(topic) + ".jsonl")).is_file():
        info = next(records(folder, topic))
    else:
        row = next(csv_rows(folder, topic))
        info = dict(width=int(row["width"]), height=int(row["height"]),
                    p=[float(row[f"p.{i}"]) for i in range(12)],
                    header={"frame_id": row["header.frame_id"]})
    tree = {
        item["child_frame_id"]: (item["header"]["frame_id"], rigid_transform(item))
        for record in records(folder, "/tf_static") for item in record["transforms"]
    }
    optical_frame = info["header"]["frame_id"]
    frame = optical_frame
    chain = np.eye(4)
    visited = set()
    while frame != "base_footprint":
        if frame in visited:
            raise ValueError("loop in static TF")
        visited.add(frame)
        frame, transform = tree[frame]
        chain = transform @ chain
    camera = GroundCamera(info["width"], info["height"], info["p"], chain[:3, 3], chain[:3, :3])
    return camera, optical_frame


def read_clouds(folder, side):
    topic = "/aiformula_perception/lane_line_publisher/lane_lines/" + side
    if not (folder / "jsonl" / (topic_key(topic) + ".jsonl")).is_file():
        return list(read_exported_clouds(folder, topic))
    clouds = []
    for data in records(folder, topic):
        if data["header"]["frame_id"] != "base_footprint":
            continue
        points = cloud_xy(bytes(data["data"]), {f["name"]: (f["offset"], f["datatype"])
                                              for f in data["fields"]},
                          data["width"], data["height"], data["row_step"],
                          data["point_step"], data["is_bigendian"])
        clouds.append(Cloud(stamp_ns(data["header"]), points))
    return clouds


def read_exported_clouds(folder, topic):
    stem = topic_key(topic)
    npz = folder / "npz" / stem
    raw = folder / "raw"
    with (npz / "timestamps.csv").open(encoding="utf-8", newline="") as frames, \
            (raw / (stem + "_index.csv")).open(encoding="utf-8", newline="") as index, \
            (raw / (stem + ".bin")).open("rb") as source:
        for frame, record in zip(csv.DictReader(frames), csv.DictReader(index)):
            if frame["timestamp_ns"] != record["timestamp_ns"]:
                raise ValueError("point cloud export timestamps disagree")
            source.seek(int(record["byte_offset"]))
            header = source.read(16)
            if header[:2] not in (b"\x00\x00", b"\x00\x01"):
                raise ValueError("unsupported point cloud CDR format")
            order = "<" if header[1] == 1 else ">"
            seconds, nanos, frame_size = struct.unpack_from(order + "iII", header, 4)
            if source.read(frame_size).rstrip(b"\0") != b"base_footprint":
                continue
            if int(frame["point_count"]):
                with np.load(npz / frame["filename"]) as data:
                    points = np.column_stack((data["x"], data["y"]))
            else:
                points = np.empty((0, 2))
            yield Cloud(seconds*10**9 + nanos, points)


def read_odom(folder):
    topic = "/aiformula_sensing/gyro_odometry_publisher/odom"
    odom = {}
    if not (folder / "jsonl" / (topic_key(topic) + ".jsonl")).is_file():
        for row in csv_rows(folder, topic):
            if row["child_frame_id"] != "base_footprint":
                continue
            stamp = int(row["header.stamp.sec"])*10**9 + int(row["header.stamp.nanosec"])
            rotation = rotation_matrix([float(row[f"pose.pose.orientation.{key}"]) for key in "xyzw"])
            odom[stamp] = (float(row["pose.pose.position.x"]), float(row["pose.pose.position.y"]),
                           math.atan2(rotation[1, 0], rotation[0, 0]))
        return odom
    for data in records(folder, topic):
        if data["child_frame_id"] != "base_footprint":
            continue
        pose = data["pose"]["pose"]
        rotation = rotation_matrix([pose["orientation"][key] for key in "xyzw"])
        odom[stamp_ns(data["header"])] = (pose["position"]["x"], pose["position"]["y"],
                                         math.atan2(rotation[1, 0], rotation[0, 0]))
    return odom


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("export", type=Path)
    parser.add_argument("--vehicle-width-m", type=float, required=True)
    args = parser.parse_args()
    config = PlannerConfig(args.vehicle_width_m)
    config.validate()
    camera, optical_frame = camera_model(args.export)
    clouds = [read_clouds(args.export, side) for side in ("center", "left", "right")]
    odom = read_odom(args.export)
    counts = Counter()
    times = []
    examples = {}
    checked_bands = 0
    for data in records(args.export, "/aiformula_perception/yolop_a3/detections"):
        frame = decode_detections(data["data"])
        if frame.frame_id != optical_frame:
            counts["waiting_for_matching_camera_info"] += 1
            continue
        group = matched_clouds(clouds, frame.stamp_ns, 220_000_000)
        if group is None:
            counts["waiting_for_synchronized_lane_clouds"] += 1
            continue
        if len(group[0].points) < 2 or any(len(cloud.points) < 2 for cloud in group[1:]):
            counts["missing_centerline" if len(group[0].points) < 2 else "missing_lane_boundary"] += 1
            continue
        aligned = align_clouds(group, odom, frame.stamp_ns, 700_000_000)
        if aligned is None:
            counts["waiting_for_odometry_alignment"] += 1
            continue
        start = perf_counter()
        try:
            obstacles = boxes_to_mask(frame.detections, camera.width, camera.height)
            result = plan(*aligned, obstacles, config, camera)
        except (ValueError, TypeError, KeyError, np.linalg.LinAlgError) as error:
            counts["invalid_planner_input"] += 1
            examples.setdefault("invalid_planner_input", str(error))
            continue
        times.append((perf_counter() - start) * 1000)
        if len(result.path):
            present = result.band > 0
            if not np.any(present) or np.any(present & (~result.allowed | result.obstacles)):
                raise RuntimeError("chosen band violates camera-space occupancy")
            checked_bands += 1
        counts[result.reason] += 1
        examples.setdefault(result.reason, dict(stamp_ns=frame.stamp_ns, **result.metrics))
    print(json.dumps(dict(frames=sum(counts.values()), decisions=counts, checked_bands=checked_bands,
                          examples=examples,
                          planning_ms_median=median(times) if times else None,
                          planning_ms_max=max(times) if times else None), indent=2))


if __name__ == "__main__":
    main()
