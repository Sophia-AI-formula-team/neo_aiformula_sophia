#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TOPIC = "/aiformula_sensing/zed_node/left_image/undistorted"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Render a four-panel YOLOP_A2 video from a rosbag image topic."
    )
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--rosbagel-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--lane-weights",
        type=Path,
        default=PROJECT_ROOT / "weights" / "lane_epoch_282.pth",
    )
    parser.add_argument(
        "--object-weights",
        type=Path,
        default=PROJECT_ROOT / "weights" / "object_detector_usable.pth",
    )
    parser.add_argument("--object-score", type=float, default=0.50)
    parser.add_argument("--cone-score", type=float, default=0.50)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    _validate_args(args)
    sys.path.insert(0, str(PROJECT_ROOT))
    sys.path.insert(0, str(args.rosbagel_root.resolve()))

    from rosbagel.core.bag_reader import open_bag_reader
    from rosbagel.core.decoder import decode_sensor_image
    from yolop_a2_runtime import DualPerceptionModel

    timestamps = _read_timestamps(open_bag_reader, args.bag, args.topic)
    frame_count = len(timestamps)
    duration_sec = (timestamps[-1] - timestamps[0]) / 1e9
    fps = (frame_count - 1) / duration_sec

    model = DualPerceptionModel(
        args.lane_weights,
        args.object_weights,
        device=args.device,
        object_score_threshold=args.object_score,
        cone_score_threshold=args.cone_score,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    partial_output = args.output.with_name(f"{args.output.stem}.partial{args.output.suffix}")
    partial_output.unlink(missing_ok=True)
    writer = cv2.VideoWriter(
        str(partial_output),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (1280, 720),
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not open video writer: {partial_output}")

    started = time.perf_counter()
    stats = {"written": 0, "lane_nonempty": 0, "obstacle_nonempty": 0, "detections": 0}
    try:
        with open_bag_reader(args.bag, backend="rosbags") as reader:
            for record in reader.iter_messages([args.topic]):
                if record.decoded is None:
                    raise RuntimeError(
                        f"could not decode image at timestamp {record.timestamp_ns}"
                    )
                decoded = decode_sensor_image(record.decoded)
                source = _prepare_source(decoded.array)
                prediction = model.predict(source)
                elapsed_sec = (record.timestamp_ns - timestamps[0]) / 1e9
                writer.write(_compose_frame(source, prediction, elapsed_sec))
                _update_stats(stats, prediction)
                if stats["written"] % 100 == 0 or stats["written"] == frame_count:
                    wall_sec = time.perf_counter() - started
                    print(
                        f"frames={stats['written']}/{frame_count} "
                        f"wall_sec={wall_sec:.1f} "
                        f"effective_fps={stats['written'] / wall_sec:.2f}",
                        flush=True,
                    )
    finally:
        writer.release()

    if stats["written"] != frame_count:
        raise RuntimeError(
            f"video frame mismatch: wrote {stats['written']}, expected {frame_count}"
        )
    if not partial_output.is_file() or partial_output.stat().st_size == 0:
        raise RuntimeError(f"video was not written: {partial_output}")

    args.output.unlink(missing_ok=True)
    partial_output.replace(args.output)
    report = _build_report(args, timestamps, duration_sec, fps, stats, started)
    report_path = args.output.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)


def _validate_args(args: argparse.Namespace) -> None:
    if not args.bag.is_dir():
        raise ValueError(f"bag directory was not found: {args.bag}")
    if not (args.rosbagel_root / "rosbagel").is_dir():
        raise ValueError(f"ROSBagel package was not found: {args.rosbagel_root}")
    for name in ("lane_weights", "object_weights"):
        path = getattr(args, name)
        if not path.is_file():
            raise ValueError(f"{name} was not found: {path}")
    for name in ("object_score", "cone_score"):
        value = getattr(args, name)
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be between 0 and 1, got {value}")
    if args.output.suffix.lower() != ".mp4":
        raise ValueError(f"output must use the .mp4 extension: {args.output}")


def _read_timestamps(open_bag_reader: object, bag: Path, topic: str) -> list[int]:
    with open_bag_reader(bag, backend="rosbags") as reader:
        timestamps = [record.timestamp_ns for record in reader.iter_messages([topic])]
    if len(timestamps) < 2:
        raise RuntimeError(f"expected at least two frames on {topic}, got {len(timestamps)}")
    return timestamps


def _prepare_source(image: np.ndarray) -> np.ndarray:
    source = _ensure_bgr(image)
    if source.shape[:2] != (360, 640):
        source = cv2.resize(source, (640, 360), interpolation=cv2.INTER_AREA)
    return source


def _compose_frame(source: np.ndarray, prediction: object, elapsed_sec: float) -> np.ndarray:
    original = _label_panel(source, f"Original camera  t={elapsed_sec:07.3f}s")
    combined = _label_panel(
        prediction.combined_overlay,
        f"YOLOP_A2 lane + objects  detections={len(prediction.detections)}",
    )
    lane = _label_panel(_ensure_bgr(prediction.lane_mask), "Lane mask")
    obstacle = _label_panel(_ensure_bgr(prediction.obstacle_mask), "Obstacle mask")
    return np.vstack((np.hstack((original, combined)), np.hstack((lane, obstacle))))


def _label_panel(image: np.ndarray, title: str) -> np.ndarray:
    panel = image.copy()
    cv2.rectangle(panel, (0, 0), (panel.shape[1] - 1, 30), (18, 18, 18), -1)
    cv2.putText(
        panel,
        title,
        (10, 21),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.56,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return panel


def _ensure_bgr(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def _update_stats(stats: dict[str, int], prediction: object) -> None:
    stats["written"] += 1
    stats["lane_nonempty"] += int(np.any(prediction.lane_mask))
    stats["obstacle_nonempty"] += int(np.any(prediction.obstacle_mask))
    stats["detections"] += len(prediction.detections)


def _build_report(
    args: argparse.Namespace,
    timestamps: list[int],
    duration_sec: float,
    fps: float,
    stats: dict[str, int],
    started: float,
) -> dict[str, object]:
    return {
        "source_bag": str(args.bag.resolve()),
        "source_bag_read_only": True,
        "topic": args.topic,
        "frame_count": stats["written"],
        "first_timestamp_ns": timestamps[0],
        "last_timestamp_ns": timestamps[-1],
        "source_duration_sec": round(duration_sec, 9),
        "video_fps": round(fps, 9),
        "video_size": [1280, 720],
        "codec": "mp4v",
        "layout": {
            "top_left": "original camera",
            "top_right": "lane and object combined overlay",
            "bottom_left": "lane mask",
            "bottom_right": "obstacle mask",
        },
        "lane_nonempty_frames": stats["lane_nonempty"],
        "obstacle_nonempty_frames": stats["obstacle_nonempty"],
        "total_detections": stats["detections"],
        "generation_wall_sec": round(time.perf_counter() - started, 3),
        "output": str(args.output.resolve()),
    }


if __name__ == "__main__":
    main()
