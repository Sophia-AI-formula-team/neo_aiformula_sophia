#!/usr/bin/env python3
"""Offline, arrival-ordered mask localization diagnostics; no ROS publication.

The recorded-data experiment freezes an anchor made from a time prefix, then
matches only later masks against that past map. The synthetic experiment builds
its anchor from rendered first-lap masks, then injects known second-lap drift.
Their metrics are deliberately separate: the real bag has no position truth.
"""

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys
import time
import uuid

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lane_mapping_lya_reference.mapping_core import (  # noqa: E402
    CausalSampleBuffer, GroundLookup, SparseConsensusMap, VectorNavLocalizer,
    make_transform, transform_local_points,
)


MASK_TOPIC = "/aiformula_perception/pub_mask_image"
VN_TOPIC = "/aiformula_sensing/vectornav/raw/common"
CAMERA_TOPIC = "/aiformula_sensing/zed_node/left/camera_info"
DEFAULT_CAMERA_FRAME = "zed_left_camera_optical_frame"


def stamp_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(clean_metrics(value), stream, indent=2, sort_keys=True, allow_nan=False,
                  default=serializable)
        stream.write("\n")


def clean_metrics(value):
    """Unbounded diagnostic errors are null in JSON, never nonstandard Infinity."""
    if isinstance(value, (np.ndarray,)):
        return clean_metrics(value.tolist())
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [clean_metrics(item) for item in value]
    if isinstance(value, dict):
        return {key: clean_metrics(item) for key, item in value.items()}
    return value


def new_output_directory():
    base = PACKAGE_ROOT / ".artifacts"
    base.mkdir(exist_ok=True)
    label = datetime.now(timezone.utc).strftime("mask_localization_%Y%m%dT%H%M%SZ_")
    destination = base / (label + uuid.uuid4().hex[:8])
    destination.mkdir(exist_ok=False)
    return destination


def percentiles(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(zip(("p50", "p95", "max"), [float(x) for x in
                                            np.percentile(values, [50, 95, 100])]))


def residual_summary(rows):
    """Keep matched/accepted subsets explicit; unavailable metrics stay null."""
    result = {}
    for group, subset in (("all_attempts", rows),
                          ("accepted", [row for row in rows if row.get("accepted")])):
        for output_key, alternatives in (
                ("before_rmse_m", ("before_rmse_m", "rmse_before_m", "residual_before_m")),
                ("after_rmse_m", ("after_rmse_m", "rmse_after_m", "residual_after_m"))):
            values = []
            for row in subset:
                metrics = row.get("metrics", {})
                for key in alternatives:
                    if key in metrics:
                        if metrics[key] is not None:
                            values.append(metrics[key])
                        break
            result[group + "_" + output_key] = percentiles(values)
    return result


def nearest_distances(points, reference):
    """Bounded-memory diagnostic distance; never used by localization itself."""
    points, reference = np.asarray(points), np.asarray(reference)
    if not len(points):
        return np.empty(0)
    if not len(reference):
        return np.full(len(points), np.inf)
    distances = []
    for first in range(0, len(points), 128):
        delta = points[first:first + 128, None, :] - reference[None, :, :]
        distances.append(np.sqrt(np.min(np.sum(delta * delta, axis=2), axis=1)))
    return np.concatenate(distances)


def map_quality(points, truth_lanes, tolerance_m=0.2):
    errors = nearest_distances(points, truth_lanes)
    coverage_errors = nearest_distances(truth_lanes, points)
    return {
        "point_count": len(points), "truth_sample_count": len(truth_lanes),
        "point_distance_to_truth_m": percentiles(errors),
        "points_within_20cm_fraction": float(np.mean(errors <= tolerance_m)) if len(errors) else None,
        "truth_within_20cm_coverage_fraction": float(np.mean(coverage_errors <= tolerance_m)),
        "coverage_scope": "entire synthetic closed pair of lane markings",
    }


def new_map():
    # Identical to the production recording defaults; no test-only relaxation.
    return SparseConsensusMap(0.1, 5, 1, max_candidate_cells=200000,
                              max_confirmed_cells=100000, candidate_ttl_s=30.0)


def fuse(mapper, local_xy, sensitivity, pose, source_stamp):
    world = transform_local_points(local_xy, *pose)
    candidate = mapper.points_to_unique_keys(world)
    reliable = mapper.points_to_unique_keys(world[sensitivity <= 0.1])
    mapper.update(candidate, reliable, stamp_ns=source_stamp)
    if mapper.capacity_rejected_cells or mapper.confirmation_capacity_rejections:
        raise RuntimeError("production anchor map capacity reached")


def resolve_static_transform(edges, parent, child):
    """Compose an already-arrived static tree only; never inspect future TF."""
    pending = [(parent, np.eye(4))]
    visited = set()
    while pending:
        frame, matrix = pending.pop()
        if frame == child:
            return matrix
        if frame in visited:
            continue
        visited.add(frame)
        for (upstream, downstream), edge in edges.items():
            if upstream == frame and downstream not in visited:
                pending.append((downstream, matrix @ edge))
            elif downstream == frame and upstream not in visited:
                pending.append((upstream, matrix @ np.linalg.inv(edge)))
    raise ValueError("past static TF has no camera-to-base chain")


def mask_array(message):
    if message.encoding not in ("mono8", "8UC1"):
        raise ValueError("nonbinary_mask_encoding")
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width < 2 or height < 2 or step < width:
        raise ValueError("invalid_mask_dimensions")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if len(raw) < height * step:
        raise ValueError("truncated_mask")
    return raw[:height * step].reshape(height, step)[:, :width]


def make_typestore(msg_directory):
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    store = get_typestore(Stores.ROS2_FOXY)
    definitions = {}
    for name in ("InsStatus", "CommonGroup"):
        path = Path(msg_directory) / (name + ".msg")
        definitions.update(get_types_from_msg(path.read_text(encoding="utf-8"),
                                              "vectornav_msgs/msg/" + name))
    store.register(definitions)
    return store


def iter_bag_messages(bag, store):
    """Read just four input streams through SQLite's read-only URI interface."""
    bag = Path(bag)
    databases = sorted(bag.glob("*.db3")) if bag.is_dir() else [bag]
    if len(databases) != 1 or databases[0].suffix != ".db3":
        raise ValueError("diagnostic currently requires exactly one db3 file")
    database = databases[0]
    with sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True) as conn:
        conn.execute("PRAGMA query_only=ON")
        topics = {int(i): (name, kind) for i, name, kind in
                  conn.execute("SELECT id,name,type FROM topics")
                  if name in (MASK_TOPIC, VN_TOPIC, CAMERA_TOPIC, "/tf_static")}
        found = {name for name, _ in topics.values()}
        required = {MASK_TOPIC, VN_TOPIC, CAMERA_TOPIC, "/tf_static"}
        if found != required:
            raise ValueError("required inputs absent: " + str(sorted(required - found)))
        query = ("SELECT id,topic_id,timestamp,data FROM messages WHERE topic_id IN ("
                 + ",".join("?" for _ in topics) + ") ORDER BY timestamp,id")
        for sequence, (_, topic_id, record_ns, data) in enumerate(conn.execute(query, tuple(topics))):
            name, kind = topics[topic_id]
            yield sequence, name, int(record_ns), store.deserialize_cdr(data, kind)


def result_row(result):
    return {"accepted": bool(result.accepted), "reason": str(result.reason),
            "pose_xyyaw": np.asarray(result.pose_xyyaw).tolist(),
            "correction_se2": np.asarray(result.correction_se2).tolist(),
            "metrics": result.metrics}


def replay_bag(bag, msg_directory, destination, anchor_seconds=60.0,
               maximum_match_age_ms=500.0):
    from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer

    store = make_typestore(msg_directory)
    map_builder, localizer = new_map(), VectorNavLocalizer()
    past_poses = CausalSampleBuffer(4096)
    camera, camera_signature, lookup = None, None, None
    edges = {}
    counts = Counter()
    first_mask_record = None
    anchor_frozen_ns = None
    matcher = None
    anchor_xy = np.empty((0, 2))
    last_mask_header = None
    latencies = []
    ages = []
    prior_track, corrected_track = [], []
    accepted_rows = []
    debug_snapshots = {}
    frame_index = -1
    source_bag = Path(bag).resolve()
    metadata = source_bag / "metadata.yaml" if source_bag.is_dir() else source_bag.parent / "metadata.yaml"
    with (destination / "recorded_frames.jsonl").open("x", encoding="utf-8") as frames, \
            (destination / "recorded_events.jsonl").open("x", encoding="utf-8") as events:
        def event(kind, **values):
            events.write(json.dumps(clean_metrics(dict(event=kind, **values)), default=serializable,
                                    allow_nan=False) + "\n")
            events.flush()

        for sequence, topic, record, message in iter_bag_messages(source_bag, store):
            if topic == VN_TOPIC:
                try:
                    sample = localizer.accept(message, record, sequence)
                    past_poses.append(sample)
                    counts["vectornav_accepted"] += 1
                except ValueError as error:
                    counts["vectornav_rejected_" + str(error)] += 1
                    event("vectornav_rejected", record_ns=record, reason=str(error))
                    if str(error) in ("vectornav_position_jump", "vectornav_heading_jump",
                                      "vectornav_nonincreasing_stamp"):
                        raise RuntimeError("production recorder would latch invalid: " + str(error))
                continue
            if topic == CAMERA_TOPIC:
                projection = np.asarray(message.p, dtype=float).reshape(3, 4)[:, :3]
                intrinsics = projection if np.all(np.isfinite(projection)) and abs(np.linalg.det(projection)) > 1e-12 else np.asarray(message.k, dtype=float).reshape(3, 3)
                if (not str(message.header.frame_id) or int(message.width) < 2 or int(message.height) < 2
                        or not np.all(np.isfinite(intrinsics)) or abs(np.linalg.det(intrinsics)) < 1e-12
                        or stamp_ns(message.header.stamp) < 0):
                    event("invalid_camera_info", record_ns=record)
                    continue
                signature = (int(message.width), int(message.height), str(message.header.frame_id),
                             tuple(intrinsics.ravel()), tuple(message.d), str(message.distortion_model))
                if camera_signature is not None and signature != camera_signature:
                    raise RuntimeError("camera calibration changed inside recorded lap")
                if camera is None:
                    camera = {"width": int(message.width), "height": int(message.height),
                              "frame_id": str(message.header.frame_id),
                              "stamp_ns": stamp_ns(message.header.stamp), "matrix": intrinsics,
                              "distortion": list(message.d), "distortion_model": str(message.distortion_model)}
                    camera_signature = signature
                continue
            if topic == "/tf_static":
                for transform in message.transforms:
                    translation, rotation = transform.transform.translation, transform.transform.rotation
                    matrix = make_transform([translation.x, translation.y, translation.z],
                                            [rotation.x, rotation.y, rotation.z, rotation.w])
                    key = (str(transform.header.frame_id), str(transform.child_frame_id))
                    if key in edges and not np.allclose(edges[key], matrix, atol=1e-12, rtol=0):
                        raise RuntimeError("static extrinsic changed inside recorded lap")
                    edges[key] = matrix
                continue

            frame_index += 1
            if not debug_snapshots:
                debug_snapshots["first_input"] = {"mask": mask_array(message).copy(),
                                                   "record_ns": record}
            source = stamp_ns(message.header.stamp)
            age = (record - source) / 1e6
            counts["mask_received"] += 1
            ages.append(age)
            if first_mask_record is None:
                first_mask_record = record
            prefix_phase = record - first_mask_record < int(anchor_seconds * 1e9)
            row = {"frame": frame_index, "arrival_sequence": sequence,
                   "record_ns": record, "mask_header_ns": source,
                   "mask_timestamp_difference_ms": age,
                   "phase": "anchor_prefix" if prefix_phase else "match_suffix"}
            started = time.perf_counter_ns()
            reason = None
            if source <= 0 or (last_mask_header is not None and source <= last_mask_header):
                raise RuntimeError("invalid or regressed mask header")
            last_mask_header = source
            if not -20.0 <= age <= (1000.0 if prefix_phase else maximum_match_age_ms):
                reason = "mask_timestamp_age"
            elif camera is None or camera["stamp_ns"] > source:
                reason = "camera_not_yet_available"
            elif (str(message.header.frame_id) != camera["frame_id"]
                  or (int(message.width), int(message.height)) != (camera["width"], camera["height"])):
                reason = "camera_mask_mismatch"
            pose, gap, pairing = past_poses.latest_not_after(source, sequence, 150000000)
            if reason is None and pose is None:
                reason = pairing
            if pose is not None:
                row.update(pose_header_ns=pose.stamp_ns, pose_arrival_sequence=pose.arrival_seq,
                           pose_to_mask_gap_ms=gap / 1e6)
                if pose.stamp_ns > source or pose.arrival_seq >= sequence:
                    raise RuntimeError("future VectorNav pose selected by replay")
            if reason is None:
                if lookup is None:
                    matrix = resolve_static_transform(edges, "base_footprint", camera["frame_id"])
                    lookup = GroundLookup(camera["width"], camera["height"],
                                          camera["matrix"], matrix, 0.5)
                local_xy, sensitivity, projection = lookup.project_mask_with_sensitivity(mask_array(message), 127)
                prior = np.asarray([pose.east_m, pose.north_m, pose.heading_rad])
                row.update(stable_pixels=projection.stable_pixels,
                           reliable_pixels=int(np.sum(sensitivity <= 0.1)), prior_xyyaw=prior.tolist())
                prior_track.append(prior)
                if prefix_phase:
                    if age + (time.perf_counter_ns() - started) / 1e6 > 1000.0:
                        reason = "anchor_commit_deadline"
                    else:
                        fuse(map_builder, local_xy, sensitivity, prior, source)
                        reason = "anchor_prefix_fused"
                else:
                    if matcher is None:
                        anchor_xy, _ = map_builder.confirmed_points_and_votes()
                        anchor_xy = anchor_xy.copy()
                        matcher = LaneMapLocalizer(anchor_xy)
                        anchor_frozen_ns = record
                        event("past_anchor_frozen", record_ns=record,
                              source_frontier_ns=map_builder.last_stamp_ns,
                              anchor_cells=len(anchor_xy), future_masks_consumed=0)
                        if map_builder.last_stamp_ns >= source:
                            raise RuntimeError("anchor contains current/future frame evidence")
                        debug_snapshots["first_suffix"] = {
                            "mask": mask_array(message).copy(), "record_ns": record,
                            "mask_header_ns": source, "anchor": anchor_xy.copy(),
                            "local_reliable": local_xy[sensitivity <= 0.1].copy(),
                            "prior_xyyaw": prior.copy(), "anchor_frontier_ns": map_builder.last_stamp_ns,
                        }
                    result = matcher.match(local_xy[sensitivity <= 0.1], prior, source,
                                           prior_stamp_ns=pose.stamp_ns, commit=False)
                    row.update(result_row(result))
                    candidate_elapsed_ms = (time.perf_counter_ns() - started) / 1e6
                    row["candidate_pose_xyyaw"] = row["pose_xyyaw"]
                    row["candidate_accepted"] = row["accepted"]
                    row["age_at_commit_check_ms"] = age + candidate_elapsed_ms
                    row["committed"] = False
                    if result.accepted and age + candidate_elapsed_ms <= maximum_match_age_ms:
                        row["committed"] = bool(matcher.commit(result))
                        if not row["committed"]:
                            row["accepted"] = False
                            row["reason"] = "commit_rejected"
                            row["pose_xyyaw"] = np.asarray(matcher.correct(prior)).tolist()
                    elif result.accepted:
                        row["accepted"] = False
                        row["reason"] = "mask_commit_deadline"
                        row["pose_xyyaw"] = np.asarray(matcher.correct(prior)).tolist()
                    accepted_stamp = matcher.last_accepted_stamp_ns
                    correction_age = None if accepted_stamp is None else (source - accepted_stamp) / 1e6
                    row["last_committed_correction_age_ms"] = correction_age
                    row["correction_fresh"] = (correction_age is not None
                                                and 0.0 <= correction_age <= maximum_match_age_ms)
                    row["trusted_pose_update"] = bool(row["committed"])
                    corrected_track.append(np.asarray(row["pose_xyyaw"]).copy())
                    accepted_rows.append(row)
                    reason = "match_" + str(row["reason"])
                    counts["matches_accepted" if row["accepted"] else "matches_rejected"] += 1
            elapsed = (time.perf_counter_ns() - started) / 1e6
            latencies.append(elapsed)
            row.update(status=reason, processing_ms=elapsed,
                       confirmed_anchor_cells=len(map_builder.confirmed_keys),
                       capacity_rejected_cells=map_builder.capacity_rejected_cells)
            counts[reason] += 1
            frames.write(json.dumps(clean_metrics(row), default=serializable, allow_nan=False) + "\n")
            frames.flush()

    refined, refinement_votes = (np.empty((0, 2)), np.empty(0)) if matcher is None else matcher.refined_points_and_votes()
    report = {
        "experiment": "recorded_single_lap_prefix_anchor_suffix_matching",
        "source_bag": str(source_bag), "source_bag_read_only": True,
        "metadata_sha256": hashlib.sha256(metadata.read_bytes()).hexdigest(),
        "anchor_duration_s": anchor_seconds, "anchor_frozen_record_ns": anchor_frozen_ns,
        "anchor_cells": len(anchor_xy), "refined_cells": len(refined),
        "input_topics": [MASK_TOPIC, VN_TOPIC, CAMERA_TOPIC, "/tf_static"],
        "camera_calibration": camera,
        "camera_to_base_transform": (resolve_static_transform(edges, "base_footprint", camera["frame_id"])
                                     if lookup is not None else None),
        "vectornav_origin_lla": localizer.origin_lla,
        "matcher_config": matcher.config if matcher is not None else None,
        "refinement_capacity": matcher.refinement_metrics() if matcher is not None else None,
        "counts": dict(counts), "processing_ms": percentiles(latencies),
        "match_attempt_processing_ms": percentiles([row["processing_ms"] for row in accepted_rows]),
        "match_accepted_processing_ms": percentiles([row["processing_ms"] for row in accepted_rows if row["accepted"]]),
        "matcher_internal_ms": percentiles([row["metrics"].get("elapsed_ms", np.nan)
                                              for row in accepted_rows]),
        "residuals": residual_summary(accepted_rows),
        "mask_record_minus_header_ms": percentiles(ages),
        "maximum_match_age_ms": maximum_match_age_ms,
        "causal_pose_gap_max_ms": 150.0, "vectornav_maximum_age_ms": 250.0,
        "future_samples_used": 0, "anchor_update_after_freeze": False,
        "refinement_reused_for_localization": False,
        "independent_ground_truth": False, "independent_second_lap": False,
        "physical_latency_measured": False, "closed_loop_vehicle_validation": False,
        "controller_freshness_state_machine_replayed": False,
        "returned_pose_policy": "Rejected results retain core diagnostic SE2 extrapolation only; not trusted updates or vehicle control. Figures show accepted commits only.",
        "commit_deadline_policy": "record-minus-header plus projection/matching callback elapsed <= mask age limit",
        "interpretation": "Residuals indicate consistency only; overlap rejection can reflect coverage or filtered non-line geometry. Inspect first_suffix_geometry, not the path alone.",
    }
    report["first_suffix_geometry"] = save_recorded_geometry(destination, debug_snapshots)
    write_json(destination / "recorded_report.json", report)
    plot_recorded(destination, np.asarray(prior_track), np.asarray(corrected_track),
                  anchor_xy, refined, accepted_rows)
    return report


def save_recorded_geometry(destination, snapshots):
    """Save original masks and past-only evidence AFTER timed replay completes."""
    plt = plotting()
    for name, payload in snapshots.items():
        plt.imsave(str(destination / (name + "_full_mask.png")), payload["mask"],
                   cmap="gray", vmin=0, vmax=255)
    sample = snapshots.get("first_suffix")
    if sample is None:
        return None
    np.savez_compressed(str(destination / "first_suffix_geometry.npz"), **sample)
    local = sample["local_reliable"]
    if len(local) > 600:
        local = local[np.linspace(0, len(local) - 1, 600).astype(int)]
    world = transform_local_points(local, *sample["prior_xyyaw"])
    errors = nearest_distances(world, sample["anchor"])
    fig, axis = plt.subplots(figsize=(7, 6))
    axis.scatter(sample["anchor"][:, 0], sample["anchor"][:, 1], s=2,
                 color="0.65", label="Frozen past confirmed anchor")
    axis.scatter(world[:, 0], world[:, 1], s=5, color="tab:red",
                 label="First suffix scan transformed by VN prior")
    axis.set(aspect="equal", xlabel="East (m)", ylabel="North (m)",
             title="Actual first suffix: coverage before normal/PCA filtering")
    axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(str(destination / "first_suffix_geometry.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)
    return {"diagnostic_scan_sample_points": len(world),
            "full_reliable_scan_points": len(sample["local_reliable"]),
            "mask_nonzero_fraction": float(np.mean(sample["mask"] > 127)),
            "mask_unique_values": np.unique(sample["mask"]).tolist(),
            "unfiltered_anchor_distance_m": percentiles(errors),
            "within_15cm_of_unfiltered_anchor": float(np.mean(errors <= .15)) if len(errors) else None,
            "within_65cm_of_unfiltered_anchor": float(np.mean(errors <= .65)) if len(errors) else None,
            "note": "Diagnostic nearest distance uses all confirmed anchor cells, BEFORE matcher line-normal filtering; this is not an accepted pose."}


def render_synthetic_mask(lane_truth, pose, rng=None):
    x, y, yaw = pose
    delta = lane_truth - np.asarray([x, y])
    forward = math.cos(yaw) * delta[:, 0] + math.sin(yaw) * delta[:, 1]
    left = -math.sin(yaw) * delta[:, 0] + math.cos(yaw) * delta[:, 1]
    visible = forward > 1.0
    forward, left = forward[visible], left[visible]
    u = 320.0 - 254.0 * left / forward
    v = 180.0 + 254.0 / forward
    if rng is not None:
        u += rng.normal(0.0, 0.25, len(u))
        v += rng.normal(0.0, 0.25, len(v))
    u, v = np.rint(u).astype(int), np.rint(v).astype(int)
    visible = (u >= 0) & (u < 640) & (v >= 0) & (v < 360)
    mask = np.zeros((360, 640), dtype=np.uint8)
    mask[v[visible], u[visible]] = 255
    if rng is not None:
        mask[rng.integers(0, 360, 30), rng.integers(0, 640, 30)] = 255
    return mask


def synthetic_experiment(destination, repeat_frames=160):
    from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer

    rng = np.random.default_rng(2718)
    radius = 12.0
    angle = np.linspace(-math.pi / 2, 3 * math.pi / 2, 5001, endpoint=False)
    truth_lanes = np.vstack([np.column_stack((r * np.cos(angle), radius + r * np.sin(angle)))
                             for r in (radius - 1.2, radius + 1.2)])
    lookup = GroundLookup(640, 360, [254., 0., 320., 0., 254., 180., 0., 0., 1.],
                          make_transform([0., 0., 1.], [-.5, .5, -.5, .5]), .5)
    mapper = new_map()
    for index, angle in enumerate(np.linspace(-math.pi / 2, 3 * math.pi / 2, 1001)):
        pose = np.array([radius * math.cos(angle), radius + radius * math.sin(angle), angle + math.pi / 2])
        local, sensitivity, _ = lookup.project_mask_with_sensitivity(render_synthetic_mask(truth_lanes, pose), 127)
        fuse(mapper, local, sensitivity, pose, 1000000000 + index * 50000000)
    anchor, _ = mapper.confirmed_points_and_votes()
    anchor = anchor.copy()
    matcher = LaneMapLocalizer(anchor)
    raw_repeat_map = SparseConsensusMap(0.1, 3, 1, max_candidate_cells=30000,
                                        max_confirmed_cells=30000)
    accepted_raw_repeat_map = SparseConsensusMap(0.1, 3, 1, max_candidate_cells=30000,
                                                 max_confirmed_cells=30000)
    rows, prior_errors, corrected_errors = [], [], []
    truth_track, prior_track, corrected_track = [], [], []
    counts = Counter()
    drift = np.asarray([0.20, -0.12, 0.03])
    for index, angle in enumerate(np.linspace(-math.pi / 2, 3 * math.pi / 2, repeat_frames, endpoint=False)):
        truth = np.array([radius * math.cos(angle), radius + radius * math.sin(angle), angle + math.pi / 2])
        prior = truth + drift
        mask = render_synthetic_mask(truth_lanes, truth, rng)
        started = time.perf_counter_ns()
        local, sensitivity, _ = lookup.project_mask_with_sensitivity(mask, 127)
        source_stamp = 60000000000 + index * 50000000
        result = matcher.match(local[sensitivity <= 0.1], prior, source_stamp,
                               prior_stamp_ns=source_stamp, commit=False)
        elapsed = (time.perf_counter_ns() - started) / 1e6
        result_fields = result_row(result)
        result_fields["committed"] = False
        if result.accepted and elapsed <= 500.0:
            result_fields["committed"] = bool(matcher.commit(result))
            if not result_fields["committed"]:
                result_fields.update(accepted=False, reason="commit_rejected")
        elif result.accepted:
            result_fields.update(accepted=False, reason="mask_commit_deadline")
        corrected = np.asarray(result.pose_xyyaw if result_fields["committed"] else matcher.correct(prior))
        result_fields["pose_xyyaw"] = corrected.tolist()
        raw_world = transform_local_points(local[sensitivity <= 0.1], *prior)
        raw_keys = raw_repeat_map.points_to_unique_keys(raw_world)
        raw_repeat_map.update(raw_keys, raw_keys, stamp_ns=source_stamp)
        if result_fields["accepted"]:
            accepted_raw_repeat_map.update(raw_keys, raw_keys, stamp_ns=source_stamp)
        before = float(np.linalg.norm(prior[:2] - truth[:2]))
        after = float(np.linalg.norm(corrected[:2] - truth[:2]))
        heading_error = abs(math.atan2(math.sin(corrected[2] - truth[2]),
                                      math.cos(corrected[2] - truth[2])))
        tangent = np.asarray([math.cos(truth[2]), math.sin(truth[2])])
        normal = np.asarray([-tangent[1], tangent[0]])
        before_vector, after_vector = prior[:2] - truth[:2], corrected[:2] - truth[:2]
        row = dict(frame=index, stamp_ns=source_stamp, truth_xyyaw=truth.tolist(),
                   prior_xyyaw=prior.tolist(), prior_position_error_m=before,
                   corrected_position_error_m=after, corrected_heading_error_rad=heading_error,
                   prior_lateral_error_m=float(before_vector @ normal),
                   corrected_lateral_error_m=float(after_vector @ normal),
                   prior_longitudinal_error_m=float(before_vector @ tangent),
                   corrected_longitudinal_error_m=float(after_vector @ tangent),
                   processing_ms=elapsed, **result_fields)
        rows.append(row)
        counts[row["reason"]] += 1
        truth_track.append(truth)
        prior_track.append(prior)
        corrected_track.append(corrected)
        prior_errors.append(before)
        corrected_errors.append(after)
    with (destination / "synthetic_frames.jsonl").open("x", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(clean_metrics(row), default=serializable, allow_nan=False) + "\n")
    accepted = [row for row in rows if row["accepted"]]
    raw_points, _ = raw_repeat_map.confirmed_points_and_votes()
    accepted_raw_points, _ = accepted_raw_repeat_map.confirmed_points_and_votes()
    refined_points, _ = matcher.refined_points_and_votes()
    error_components = {}
    for label, subset in (("all_frames", rows), ("accepted_frames", accepted)):
        for source in ("prior", "corrected"):
            for axis in ("lateral", "longitudinal"):
                key = source + "_" + axis + "_error_m"
                error_components[label + "_" + source + "_" + axis + "_rmse_m"] = (
                    float(np.sqrt(np.mean([row[key] ** 2 for row in subset]))) if subset else None)
    report = {
        "experiment": "synthetic_first_lap_rendered_anchor_then_known_drift_noisy_repeat",
        "anchor_frames": 1001, "repeat_frames": repeat_frames, "anchor_cells": len(anchor),
        "accepted_frames": len(accepted), "reason_counts": dict(counts),
        "injected_drift_xyyaw": drift.tolist(), "pixel_noise_sigma": 0.25,
        "salt_pixels_per_frame": 30, "seed": 2718,
        "matcher_config": matcher.config,
        "refinement_capacity": matcher.refinement_metrics(),
        "prior_position_rmse_m": float(np.sqrt(np.mean(np.square(prior_errors)))),
        "returned_position_rmse_m_all_frames": float(np.sqrt(np.mean(np.square(corrected_errors)))),
        "accepted_position_rmse_m": (float(np.sqrt(np.mean([row["corrected_position_error_m"] ** 2
                                                            for row in accepted]))) if accepted else None),
        "same_accepted_subset_prior_position_rmse_m": (float(np.sqrt(np.mean([
            row["prior_position_error_m"] ** 2 for row in accepted]))) if accepted else None),
        "accepted_heading_rmse_rad": (float(np.sqrt(np.mean([
            row["corrected_heading_error_rad"] ** 2 for row in accepted]))) if accepted else None),
        "same_accepted_subset_prior_heading_rmse_rad": abs(float(drift[2])) if accepted else None,
        "error_components": error_components,
        "map_quality": {
            "raw_prior_all_repeat_frames_3vote": map_quality(raw_points, truth_lanes),
            "raw_prior_same_accepted_frames_3vote": map_quality(accepted_raw_points, truth_lanes),
            "accepted_matched_refinement_3vote": map_quality(refined_points, truth_lanes),
            "fairness_note": "Refinement uses accepted matched inliers; raw baseline uses all low-J4 projections. Point counts and whole-track truth coverage expose filtering/coverage changes.",
        },
        "residuals": residual_summary(rows),
        "processing_ms": percentiles([row["processing_ms"] for row in rows]),
        "independent_ground_truth": True, "real_world_accuracy_claim": False,
        "synthetic_pose_used_only_to_render_and_score": True,
        "anchor_created_from_masks_not_truth_lane_coordinates": True,
        "anchor_update_after_freeze": False,
        "vectornav_message_admission_tested": False,
        "trajectory_note": "Controlled geometric drift/noise experiment; synthetic pose sequence does not claim vehicle-feasible speeds or test VectorNav message admission.",
    }
    write_json(destination / "synthetic_report.json", report)
    plot_synthetic(destination, np.asarray(truth_track), np.asarray(prior_track),
                   np.asarray(corrected_track), prior_errors, corrected_errors, rows)
    plot_map_quality(destination, raw_points, refined_points, truth_lanes)
    return report


def plotting():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def plot_recorded(destination, prior, corrected, anchor, refined, rows):
    plt = plotting()
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    if len(anchor):
        axes[0].scatter(anchor[:, 0], anchor[:, 1], s=1, c="0.6", label="Past frozen anchor")
    if len(refined):
        axes[0].scatter(refined[:, 0], refined[:, 1], s=2, c="green", label="Separate bounded refinement")
    if len(prior):
        axes[0].plot(prior[:, 0], prior[:, 1], c="tab:blue", label="VectorNav prior")
    accepted = [row for row in rows if row["accepted"]]
    if accepted:
        accepted_xy = np.asarray([row["pose_xyyaw"][:2] for row in accepted])
        axes[0].scatter(accepted_xy[:, 0], accepted_xy[:, 1], c="tab:orange", s=24,
                        label="Accepted mask pose updates only")
    axes[0].set(aspect="equal", xlabel="ENU east (m)", ylabel="ENU north (m)",
                title="Recorded lap: prefix anchor only, no future map")
    axes[0].legend(fontsize=8)
    if rows:
        axes[1].scatter([row["frame"] for row in accepted],
                        [np.linalg.norm(np.asarray(row["pose_xyyaw"])[:2] - np.asarray(row["prior_xyyaw"])[:2])
                         for row in accepted], c="green", s=20, label="Accepted update")
        rejected = [row["frame"] for row in rows if not row["accepted"]]
        axes[1].scatter(rejected, np.zeros(len(rejected)), c="red", marker="x", s=8,
                        label="Rejected: no trusted pose update")
        axes[1].legend(fontsize=8)
    axes[1].set(xlabel="Recorded mask index", ylabel="Accepted position correction (m)",
                title="Accepted commits only; NOT ground-truth error")
    fig.tight_layout()
    fig.savefig(str(destination / "recorded_diagnostics.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


def plot_synthetic(destination, truth, prior, corrected, before, after, rows):
    plt = plotting()
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    axes[0].plot(truth[:, 0], truth[:, 1], c="black", label="Known synthetic truth")
    axes[0].plot(prior[:, 0], prior[:, 1], c="tab:red", label="Injected-drift prior")
    axes[0].plot(corrected[:, 0], corrected[:, 1], c="tab:green", label="Matcher returned pose")
    axes[0].set(aspect="equal", xlabel="East (m)", ylabel="North (m)", title="Synthetic repeat only")
    axes[0].legend(fontsize=8)
    axes[1].plot(before, label="Prior position error", c="tab:red")
    axes[1].plot(after, label="Returned position error", c="tab:green")
    rejected = [index for index, row in enumerate(rows) if not row["accepted"]]
    axes[1].scatter(rejected, [after[index] for index in rejected], c="black", marker="x", s=12,
                    label="Rejected frame")
    axes[1].set(xlabel="Repeat frame", ylabel="Position error (m)",
                title="Known synthetic truth, not real-bag accuracy")
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(str(destination / "synthetic_diagnostics.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


def plot_map_quality(destination, raw, refined, truth):
    plt = plotting()
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for axis, points, title in zip(axes, (raw, refined),
                                    ("Raw prior: all repeat frames, 3 votes",
                                     "Accepted matched refinement: 3 votes")):
        axis.scatter(truth[:, 0], truth[:, 1], c="0.85", s=1)
        if len(points):
            errors = nearest_distances(points, truth)
            scatter = axis.scatter(points[:, 0], points[:, 1], c=errors, s=5,
                                   cmap="viridis", vmin=0, vmax=0.3)
            fig.colorbar(scatter, ax=axis, label="Distance to known synthetic lane (m)")
        axis.set(aspect="equal", title=title, xlabel="East (m)", ylabel="North (m)")
        axis.set_xlim(-15, 15)
        axis.set_ylim(-3, 27)
    fig.suptitle("Synthetic map quality: compare coverage and point count, not just cleaned appearance")
    fig.tight_layout()
    fig.savefig(str(destination / "synthetic_map_quality.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--vectornav-msg-dir", type=Path)
    parser.add_argument("--anchor-seconds", type=float, default=60.0)
    parser.add_argument("--maximum-match-age-ms", type=float, default=500.0)
    parser.add_argument("--synthetic-only", action="store_true")
    parser.add_argument("--repeat-frames", type=int, default=160)
    args = parser.parse_args(argv)
    if not args.synthetic_only and (args.bag is None or args.vectornav_msg_dir is None):
        parser.error("recorded diagnostic requires --bag and --vectornav-msg-dir")
    if not math.isfinite(args.anchor_seconds) or args.anchor_seconds <= 0:
        parser.error("anchor-seconds must be finite and positive")
    if not math.isfinite(args.maximum_match_age_ms) or args.maximum_match_age_ms <= 0:
        parser.error("maximum-match-age-ms must be finite and positive")
    if not 8 <= args.repeat_frames <= 5000:
        parser.error("repeat-frames must be 8..5000")
    destination = new_output_directory()
    report = {"output_directory": str(destination), "vehicle_topics_published": False,
              "source_bag_modified": False,
              "source_sha256": {str(path.relative_to(PACKAGE_ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in (Path(__file__),
                                              PACKAGE_ROOT / "lane_mapping_lya_reference" / "mapping_core.py",
                                              PACKAGE_ROOT / "lane_mapping_lya_reference" / "mask_localization.py")}}
    try:
        report["synthetic"] = synthetic_experiment(destination, args.repeat_frames)
        if not args.synthetic_only:
            report["recorded"] = replay_bag(args.bag, args.vectornav_msg_dir, destination,
                                             args.anchor_seconds, args.maximum_match_age_ms)
    except Exception as error:
        report["error"] = str(error)
        write_json(destination / "report.json", report)
        print(json.dumps({"output": str(destination), "error": str(error)}))
        raise
    write_json(destination / "report.json", report)
    print(json.dumps(report, default=serializable))


if __name__ == "__main__":
    main()
