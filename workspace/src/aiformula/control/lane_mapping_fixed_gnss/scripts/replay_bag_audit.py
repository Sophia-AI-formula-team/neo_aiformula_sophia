#!/usr/bin/env python3
"""Read-only recorded CAN/raw-gyro audit, never a ROS/vehicle replay.

Original bag arrival and source times are preserved. GPS is decoded solely to
audit endpoint availability; neither baseline nor poison-GPS motion consumes
any VectorNav input. Optional teach probing is explicitly unarmed and cannot
produce a ready route without real valid endpoint anchors.
"""

import argparse
from collections import Counter, deque
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import struct
import sys
import time
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss.motion import CausalWheelGyroOdometry, decode_honda_rpm
from lane_mapping_fixed_gnss.anchors import decode_vectornav_gps
from lane_mapping_fixed_gnss.mapping import TeachMapEngine
from lane_mapping_lya_reference.mapping_core import GroundLookup, make_transform

CAN = "/aiformula_sensing/vehicle_info"
GYRO = "/aiformula_sensing/zed_node/imu/data_raw"
GPS = "/aiformula_sensing/vectornav/raw/gps"
MASK = "/aiformula_perception/pub_mask_image"
CAMERA = "/aiformula_sensing/zed_node/left/camera_info"
VN_PREFIX = "/aiformula_sensing/vectornav/"


def stamp(message):
    return int(message.header.stamp.sec) * 1000000000 + int(message.header.stamp.nanosec)


def json_write(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def percentiles(values):
    return dict(zip(("p50", "p95", "max"), map(float, np.percentile(values, [50, 95, 100])))) if values else None


def typestore(msg_directory):
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    store = get_typestore(Stores.ROS2_FOXY)
    definitions = {}
    for path in sorted(Path(msg_directory).glob("*.msg")):
        definitions.update(get_types_from_msg(path.read_text(encoding="utf-8"),
                                              "vectornav_msgs/msg/" + path.stem))
    definitions.update(get_types_from_msg(
        "std_msgs/Header header\nuint32 id\nbool is_rtr\nbool is_extended\nbool is_error\nuint8 dlc\nuint8[8] data\n",
        "can_msgs/msg/Frame"))
    store.register(definitions)
    return store


def transform(edges, child):
    pending, visited = [("base_footprint", np.eye(4))], set()
    while pending:
        frame, matrix = pending.pop()
        if frame == child:
            return matrix
        if frame in visited:
            continue
        visited.add(frame)
        for (parent, descendant), edge in edges.items():
            if parent == frame and descendant not in visited:
                pending.append((descendant, matrix @ edge))
            elif descendant == frame and parent not in visited:
                pending.append((parent, matrix @ np.linalg.inv(edge)))
    raise ValueError("no_arrived_static_transform_to_" + str(child))


class PoisonVectorNavInput:
    """A replacement for EVERY forbidden VectorNav payload, not a valid fix."""
    def __getattribute__(self, name):
        raise RuntimeError("forbidden GPS/INS field was read: " + name)


def discard_forbidden(topic, payload):
    # The counterfactual payload is deliberately not accessed/deserialized.
    if not topic.startswith(VN_PREFIX):
        raise ValueError("only VectorNav sources belong to the forbidden branch")
    del payload


def audit(bag, msg_directory, probe_teach=False, motion_start_offset_s=0.0):
    bag = Path(bag).resolve()
    databases = sorted(bag.glob("*.db3")) if bag.is_dir() else [bag]
    if len(databases) != 1:
        raise ValueError("audit requires a single sqlite3 bag segment")
    directory = ROOT / ".artifacts" / (datetime.now(timezone.utc).strftime("bag_audit_%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:8])
    directory.mkdir(parents=True, exist_ok=False)
    store = typestore(msg_directory)
    baseline, poisoned = CausalWheelGyroOdometry(), CausalWheelGyroOdometry()
    motion_hashes = [hashlib.sha256(), hashlib.sha256()]
    mask_pair_hashes = [hashlib.sha256(), hashlib.sha256()]
    forbidden_hashes = [hashlib.sha256(), hashlib.sha256()]
    counts, reasons, topic_counts, gps_fields, can_ids = Counter(), Counter(), Counter(), Counter(), Counter()
    gyro_ages, wheel_ages, gyro_gaps, mask_ages, pose_gaps = [], [], [], [], []
    wheel_source_intervals = []
    previous_wheel_source, first_continuity_failure = None, None
    pose_buffers = [deque(maxlen=4096), deque(maxlen=4096)]
    edges, camera, lookup, gyro_frame, gyro_rotation = {}, None, None, None, None
    latest_mount, mount_candidate, mount_stamps = None, None, set()
    first_record, first_rejection = None, None
    engine = TeachMapEngine() if probe_teach else None
    teach_counts = Counter()
    started = time.perf_counter()
    with sqlite3.connect(databases[0].as_uri() + "?mode=ro", uri=True) as conn, \
            (directory / "motion.jsonl").open("x", encoding="utf-8") as motion_log, \
            (directory / "mask_pairing.jsonl").open("x", encoding="utf-8") as mask_log:
        conn.execute("PRAGMA query_only=ON")
        all_topics = list(conn.execute("SELECT id,name,type FROM topics"))
        topics = {int(i): (name, kind) for i, name, kind in all_topics
                  if name in (CAN, GYRO, MASK, CAMERA, "/tf_static", "/tf") or name.startswith(VN_PREFIX)}
        found = {name for name, _ in topics.values()}
        missing = {CAN, GYRO, GPS, MASK, CAMERA, "/tf_static"} - found
        if missing:
            raise ValueError("required topics missing: " + str(sorted(missing)))
        query = ("SELECT id,topic_id,timestamp,data FROM messages WHERE topic_id IN ("
                 + ",".join("?" for _ in topics) + ") ORDER BY timestamp,id")
        for sequence, (message_id, topic_id, record_ns, data) in enumerate(conn.execute(query, tuple(topics))):
            topic, kind = topics[topic_id]
            record_ns = int(record_ns)
            if first_record is None:
                first_record = record_ns
            # Synthetic monotonic receipt coordinate preserves EVERY recorded
            # inter-arrival gap; it is not a measured network/physical latency.
            receipt = record_ns - first_record + 1000000000
            topic_counts[topic] += 1
            if topic.startswith(VN_PREFIX):
                discard_forbidden(topic, data)
                discard_forbidden(topic, PoisonVectorNavInput())
                forbidden_hashes[0].update(data)
                forbidden_hashes[1].update(b"ALL_VECTORNAV_INPUT_REPLACED_WITH_POISON")
                counts["gps_ins_inputs_replaced_and_excluded"] += 1
                if topic == GPS:
                    message = store.deserialize_cdr(data, kind)
                    gps_fields[str(int(message.group_fields))] += 1
                    try:
                        decode_vectornav_gps(message, receipt)
                        counts["raw_gps_decoder_accepted"] += 1
                    except ValueError as error:
                        reasons["gps:" + str(error)] += 1
                        counts["raw_gps_decoder_rejected"] += 1
                continue
            message = store.deserialize_cdr(data, kind)
            if topic == "/tf":
                # The only admitted /tf edge is the ZED factory IMU mounting,
                # never map/odom/body attitude or an INS-derived pose.
                for item in message.transforms:
                    if (str(item.header.frame_id), str(item.child_frame_id)) != ("zed_left_camera_frame", "zed_imu_link"):
                        continue
                    t, q = item.transform.translation, item.transform.rotation
                    candidate = (stamp(item), make_transform([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]))
                    if latest_mount is None or candidate[0] >= latest_mount[0]:
                        latest_mount = candidate
                    counts["factory_mount_messages_seen"] += 1
                continue
            if topic == "/tf_static":
                for item in message.transforms:
                    t, q = item.transform.translation, item.transform.rotation
                    key = (str(item.header.frame_id), str(item.child_frame_id))
                    matrix = make_transform([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
                    if key in edges and not np.allclose(edges[key], matrix, atol=1e-12, rtol=0):
                        raise ValueError("recorded static TF changed")
                    edges[key] = matrix
                continue
            if topic == CAMERA:
                projection = np.asarray(message.p, dtype=float).reshape(3, 4)[:, :3]
                matrix = projection if np.all(np.isfinite(projection)) and abs(np.linalg.det(projection)) > 1e-12 else np.asarray(message.k).reshape(3, 3)
                profile = {"width": int(message.width), "height": int(message.height),
                           "frame_id": str(message.header.frame_id), "matrix": matrix.tolist(),
                           "stamp_ns": stamp(message)}
                if camera is None:
                    camera = profile
                elif any(camera[k] != profile[k] for k in ("width", "height", "frame_id", "matrix")):
                    raise ValueError("recorded calibration changed")
                continue
            source = stamp(message)
            if topic == GYRO:
                counts["raw_gyro_seen"] += 1
                gyro_ages.append((record_ns - source) / 1e6)
                try:
                    if not 0 <= record_ns - source <= 200000000:
                        raise ValueError("raw_gyro_source_stale_or_future")
                    frame = str(message.header.frame_id)
                    try:
                        mounting = transform(edges, frame)
                        mounting_stamp = 0
                    except ValueError:
                        if frame != "zed_imu_link" or latest_mount is None:
                            raise ValueError("waiting_arrived_factory_imu_mount")
                        mounting_stamp, relative_mount = latest_mount
                        if mounting_stamp > source:
                            if gyro_rotation is None:
                                raise ValueError("waiting_causal_factory_imu_mount")
                            mounting = None
                        else:
                            mounting = transform(edges, "zed_left_camera_frame") @ relative_mount
                    if mounting is not None:
                        if mounting_stamp:
                            if mount_candidate is None:
                                mount_candidate = mounting.copy()
                            if not np.allclose(mounting, mount_candidate, atol=1e-7, rtol=0):
                                raise ValueError("factory_imu_mount_changed")
                            mount_stamps.add(mounting_stamp)
                            if len(mount_stamps) < 3:
                                raise ValueError("waiting_three_identical_mount_stamps")
                        if gyro_rotation is None:
                            gyro_rotation, gyro_frame = mounting[:3, :3].copy(), frame
                        elif not np.allclose(gyro_rotation, mounting[:3, :3], atol=1e-7, rtol=0):
                            raise ValueError("latched_gyro_rotation_changed")
                    if frame != gyro_frame:
                        raise ValueError("gyro_frame_changed")
                    rate = np.asarray([message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z])
                    if not np.all(np.isfinite(rate)):
                        raise ValueError("nonfinite_raw_gyro")
                    yaw_rate = float((gyro_rotation @ rate)[2])
                    errors = []
                    for motion in (baseline, poisoned):
                        try:
                            motion.accept_gyro(source, receipt, yaw_rate)
                            errors.append(None)
                        except ValueError as error:
                            errors.append(str(error))
                    if errors[0] != errors[1]:
                        raise RuntimeError("GPS/INS replacement altered gyro admission")
                    if errors[0]:
                        raise ValueError(errors[0])
                    counts["raw_gyro_accepted"] += 1
                except ValueError as error:
                    counts["raw_gyro_rejected"] += 1
                    reasons["gyro:" + str(error)] += 1
                    if first_rejection is None:
                        first_rejection = {"topic": topic, "record_ns": record_ns, "reason": str(error)}
                continue
            if topic == CAN:
                counts["can_seen"] += 1
                can_ids[str(int(message.id))] += 1
                if int(message.id) != 1809:
                    counts["can_other_ids_ignored"] += 1
                    continue
                counts["wheel_can_seen"] += 1
                wheel_ages.append((record_ns - source) / 1e6)
                if previous_wheel_source is not None:
                    wheel_source_intervals.append((source - previous_wheel_source) / 1e6)
                previous_wheel_source = source
                row = {"sequence": sequence, "record_ns": record_ns, "source_ns": source}
                if record_ns - first_record < motion_start_offset_s * 1e9:
                    counts["wheel_can_before_fixed_segment"] += 1
                    row.update(accepted=False, reason="before_predeclared_independent_motion_segment")
                    motion_log.write(json.dumps(row) + "\n")
                    continue
                try:
                    left, right = decode_honda_rpm(message.id, message.data, message.dlc,
                                                   message.is_rtr, message.is_extended, message.is_error)
                    outputs, errors = [], []
                    for motion in (baseline, poisoned):
                        try:
                            outputs.append(motion.accept_wheels(source, receipt, left, right, record_ns,
                                                                 now_steady_ns=receipt))
                            errors.append(None)
                        except ValueError as error:
                            outputs.append(None)
                            errors.append(str(error))
                    if errors[0] != errors[1] or outputs[0] != outputs[1]:
                        raise RuntimeError("GPS/INS replacement changed wheel/gyro poses")
                    if errors[0]:
                        raise ValueError(errors[0])
                    sample = outputs[0]
                    if sample.gyro_stamp_ns > sample.stamp_ns:
                        raise RuntimeError("future gyro consumed")
                    gyro_gaps.append((sample.stamp_ns - sample.gyro_stamp_ns) / 1e6)
                    for index, value in enumerate(outputs):
                        packed = struct.pack("<qqdddd", value.stamp_ns, value.gyro_stamp_ns,
                                             value.x, value.y, value.yaw, value.speed)
                        motion_hashes[index].update(packed)
                        pose_buffers[index].append((sequence, value))
                    counts["wheel_can_accepted"] += 1
                    row.update(accepted=True, x=sample.x, y=sample.y, yaw=sample.yaw,
                               speed=sample.speed, gyro_source_ns=sample.gyro_stamp_ns)
                except ValueError as error:
                    counts["wheel_can_rejected"] += 1
                    reasons["wheel:" + str(error)] += 1
                    # Match the runtime callback: no mask may reuse a pose
                    # buffer from before a rejected wheel measurement.
                    for buffer in pose_buffers:
                        buffer.clear()
                    row.update(accepted=False, reason=str(error))
                    if baseline.last_sample is not None:
                        row["source_dt_from_last_accepted_ms"] = (source - baseline.last_sample.stamp_ns) / 1e6
                    if str(error) == "wheel_dt_gap_or_restart" and first_continuity_failure is None:
                        first_continuity_failure = dict(row)
                    if first_rejection is None:
                        first_rejection = {"topic": topic, "record_ns": record_ns, "reason": str(error)}
                motion_log.write(json.dumps(row) + "\n")
                continue

            counts["mask_seen"] += 1
            mask_ages.append((record_ns - source) / 1e6)
            row = {"sequence": sequence, "record_ns": record_ns, "source_ns": source}
            if record_ns - first_record < motion_start_offset_s * 1e9:
                counts["mask_before_fixed_segment"] += 1
                row.update(reason="before_predeclared_independent_motion_segment")
                mask_log.write(json.dumps(row) + "\n")
                continue
            try:
                if not 0 <= record_ns - source <= 500000000:
                    raise ValueError("mask_source_stale_or_future")
                selected = []
                for index, buffer in enumerate(pose_buffers):
                    eligible = [(seq, item) for seq, item in buffer if seq < sequence and item.stamp_ns <= source]
                    chosen = max(eligible, key=lambda pair: pair[1].stamp_ns) if eligible else None
                    selected.append(chosen)
                    if chosen:
                        sample = chosen[1]
                        mask_pair_hashes[index].update(struct.pack("<qqddd", source, sample.stamp_ns,
                                                                 sample.x, sample.y, sample.yaw))
                if selected[0] != selected[1]:
                    raise RuntimeError("GPS/INS replacement changed mask pairing")
                if selected[0] is None:
                    raise ValueError("no_already_arrived_past_motion")
                sample = selected[0][1]
                gap_ms = (source - sample.stamp_ns) / 1e6
                if gap_ms > 150.0:
                    raise ValueError("motion_mask_gap")
                pose_gaps.append(gap_ms)
                counts["mask_causal_pairs"] += 1
                row.update(paired=True, pose_source_ns=sample.stamp_ns, pose_gap_ms=gap_ms)
                if engine is not None:
                    projection_started = time.perf_counter()
                    if camera is None or camera["stamp_ns"] > source:
                        raise ValueError("no_arrived_camera_calibration")
                    if str(message.header.frame_id) != camera["frame_id"]:
                        raise ValueError("mask_camera_frame_mismatch")
                    if lookup is None:
                        lookup = GroundLookup(camera["width"], camera["height"], camera["matrix"],
                                              transform(edges, camera["frame_id"]), .5)
                    if message.encoding not in ("mono8", "8UC1"):
                        raise ValueError("unsupported_mask_encoding")
                    raw = np.frombuffer(message.data, dtype=np.uint8)
                    mask = raw.reshape(int(message.height), int(message.step))[:, :int(message.width)]
                    local, sensitivity, _ = lookup.project_mask_with_sensitivity(mask, 127)
                    @contextmanager
                    def guard():
                        age = (record_ns - source) / 1e6 + (time.perf_counter() - projection_started) * 1000.
                        yield age <= 500.
                    outcome = engine.process(local, sensitivity, [sample.x, sample.y, sample.yaw], source,
                                             commit_guard=guard)
                    teach_counts[outcome["reason"]] += 1
                    row.update(teach_accepted=outcome["accepted"], teach_reason=outcome["reason"],
                               teach_confirmed_cells=outcome["confirmed_cells"],
                               teach_frontier_ns=outcome["matched_against_frontier_ns"])
            except ValueError as error:
                reasons["mask:" + str(error)] += 1
                row.update(reason=str(error))
            mask_log.write(json.dumps(row) + "\n")
    snapshot_summary = None
    if engine is not None:
        try:
            snapshot = engine.freeze()
            snapshot_summary = {"frozen": True, "trace_points": len(snapshot.trace),
                                "consensus_cells": len(snapshot.consensus_xy), "stats": dict(snapshot.stats)}
        except ValueError as error:
            snapshot_summary = {"frozen": False, "reason": str(error)}
    metadata = bag / "metadata.yaml" if bag.is_dir() else bag.parent / "metadata.yaml"
    report = {
        "experiment": "recorded_CAN_raw_gyro_input_audit_not_vehicle_validation",
        "output_directory": str(directory), "source_bag": str(bag),
        "source_read_only": True, "ros_topics_published": False, "source_timestamps_relabelled": False,
        "motion_segment_start_offset_s": motion_start_offset_s,
        "motion_segment_policy": "First wheel after a predeclared fixed offset starts an independent local diagnostic. No reset after any admitted pose; no claim of stationary/GNSS start or complete lap.",
        "source_metadata_sha256": hashlib.sha256(metadata.read_bytes()).hexdigest(),
        "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in (Path(__file__), ROOT / "lane_mapping_fixed_gnss" / "motion.py",
                                       ROOT / "lane_mapping_fixed_gnss" / "anchors.py",
                                       ROOT / "lane_mapping_fixed_gnss" / "mapping.py")},
        "topics_in_bag": len(all_topics), "selected_topic_counts": dict(topic_counts),
        "counts": dict(counts), "rejection_reasons": dict(reasons),
        "can_ids": dict(can_ids), "gps_group_fields": dict(gps_fields),
        "gyro_record_minus_header_ms": percentiles(gyro_ages),
        "wheel_record_minus_header_ms": percentiles(wheel_ages),
        "wheel_source_interarrival_ms": percentiles(wheel_source_intervals),
        "wheel_source_intervals_over_200ms": sum(value > 200. for value in wheel_source_intervals),
        "accepted_gyro_to_wheel_gap_ms": percentiles(gyro_gaps),
        "mask_record_minus_header_ms": percentiles(mask_ages),
        "accepted_pose_to_mask_gap_ms": percentiles(pose_gaps),
        "source_frontier_violations": 0,
        "gyro_sensor_frame": gyro_frame,
        "gyro_rotation_to_base": None if gyro_rotation is None else gyro_rotation.tolist(),
        "factory_mount_distinct_causal_stamps": len(mount_stamps),
        "factory_mount_policy": "Only camera->IMU rigid edge from arrived /tf, at/before raw gyro; freeze after three distinct identical matrices, tolerance1e-7. Other dynamic transforms excluded.",
        "motion_config": dict(baseline.config), "motion_state": baseline.snapshot(),
        "first_input_rejection": first_rejection,
        "first_wheel_continuity_failure": first_continuity_failure,
        "gps_ins_invariance": {
            "replacement": "Every VectorNav payload replaced with poison object raising on any attribute read",
            "baseline_motion_sha256": motion_hashes[0].hexdigest(), "poisoned_motion_sha256": motion_hashes[1].hexdigest(),
            "baseline_mask_pair_sha256": mask_pair_hashes[0].hexdigest(), "poisoned_mask_pair_sha256": mask_pair_hashes[1].hexdigest(),
            "forbidden_stream_original_sha256": forbidden_hashes[0].hexdigest(),
            "forbidden_stream_replaced_sha256": forbidden_hashes[1].hexdigest(),
            "identical_pose_outputs": motion_hashes[0].digest() == motion_hashes[1].digest(),
            "identical_causal_mask_pairs": mask_pair_hashes[0].digest() == mask_pair_hashes[1].digest(),
            "nonempty_pose_evidence": counts["wheel_can_accepted"] > 0,
            "nonempty_mask_pair_evidence": counts["mask_causal_pairs"] > 0,
            "orientation_or_INS_velocity_used": False},
        "teach_probe": {"enabled": probe_teach, "counts": dict(teach_counts), "snapshot": snapshot_summary,
                        "endpoint_anchor_permission_obtained": False, "ready_bundle_created": False,
                        "interpretation": "Unarmed pure-engine probe only, not an actual authorized runtime TEACH run."},
        "elapsed_wall_s": time.perf_counter() - started,
        "limitations": ["Record/header differences are not measured physical latency.",
                        "All counts continue after errors for diagnosis; runtime HOLD/arming is not simulated.",
                        "Invalid/missing raw GPS fields cannot be replaced with CommonGroup INS positions.",
                        "No independent trajectory truth or real endpoint windows; no claimed complete lap or closed-loop validation."],
    }
    json_write(directory / "report.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--vectornav-msg-dir", type=Path, required=True)
    parser.add_argument("--probe-teach", action="store_true")
    parser.add_argument("--motion-start-offset-seconds", type=float, default=0.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.motion_start_offset_seconds) or not 0.0 <= args.motion_start_offset_seconds <= 3600.0:
        parser.error("motion-start-offset-seconds must be finite and in 0..3600")
    report = audit(args.bag, args.vectornav_msg_dir, args.probe_teach, args.motion_start_offset_seconds)
    print(json.dumps({key: report[key] for key in
                      ("output_directory", "counts", "rejection_reasons", "gps_ins_invariance", "teach_probe")}))


if __name__ == "__main__":
    main()
