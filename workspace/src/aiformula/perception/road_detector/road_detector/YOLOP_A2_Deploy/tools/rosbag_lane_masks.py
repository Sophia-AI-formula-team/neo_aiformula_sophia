#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np
import torch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TOPIC = "/aiformula_sensing/zed_node/left_image/undistorted"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export binary YOLOP_A2 lane masks from a rosbag image topic."
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
    return parser


def main() -> None:
    args = build_parser().parse_args()
    _validate_args(args)
    sys.path.insert(0, str(PROJECT_ROOT))
    sys.path.insert(0, str(args.rosbagel_root.resolve()))

    from rosbagel.core.bag_reader import open_bag_reader
    from rosbagel.core.decoder import decode_sensor_image
    from yolop_a2_runtime.lane_model import load_lane_checkpoint
    from yolop_a2_runtime.lane_processing import (
        LaneRuntimeConfig,
        lane_probability_from_logits,
        postprocess_lane_probability,
        preprocess_lane_image,
        restore_lane_map,
    )

    device = _resolve_device(args.device)
    model = load_lane_checkpoint(args.lane_weights, device=device)
    use_half = device.type == "cuda"
    if use_half:
        model.half()
    lane_config = LaneRuntimeConfig()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(
            prefix=f".{args.output.name}-staging-",
            dir=args.output.parent,
        )
    )
    started = time.perf_counter()
    frames: list[dict[str, int | str]] = []
    try:
        with open_bag_reader(args.bag, backend="rosbags") as reader:
            expected_count = reader.get_message_count(args.topic)
            for index, record in enumerate(reader.iter_messages([args.topic])):
                if record.decoded is None:
                    raise RuntimeError(
                        f"could not decode image at timestamp {record.timestamp_ns}"
                    )
                decoded = decode_sensor_image(record.decoded)
                image_bgr = _prepare_source(decoded.array)
                mask = _predict_lane_mask(
                    image_bgr,
                    model,
                    device,
                    use_half,
                    lane_config,
                    preprocess_lane_image,
                    lane_probability_from_logits,
                    restore_lane_map,
                    postprocess_lane_probability,
                )
                filename = f"{index:06d}_{record.timestamp_ns}.png"
                path = staging / filename
                if not cv2.imwrite(str(path), mask):
                    raise OSError(f"failed to write lane mask: {path}")
                frames.append(
                    {
                        "index": index,
                        "timestamp_ns": record.timestamp_ns,
                        "filename": filename,
                        "nonzero_pixels": int(np.count_nonzero(mask)),
                    }
                )
                if len(frames) % 100 == 0 or len(frames) == expected_count:
                    wall_sec = time.perf_counter() - started
                    print(
                        f"frames={len(frames)}/{expected_count} "
                        f"wall_sec={wall_sec:.1f} "
                        f"effective_fps={len(frames) / wall_sec:.2f}",
                        flush=True,
                    )

        if len(frames) != expected_count:
            raise RuntimeError(
                f"lane mask count mismatch: wrote {len(frames)}, expected {expected_count}"
            )
        manifest = _build_manifest(args, device, frames, started)
        (staging / "_manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n",
            encoding="utf-8",
        )
        if args.output.exists():
            raise FileExistsError(f"output already exists: {args.output}")
        staging.replace(args.output)
    except Exception:
        if staging.exists():
            shutil.rmtree(staging)
        raise

    print(json.dumps(manifest, indent=2), flush=True)


def _validate_args(args: argparse.Namespace) -> None:
    if not args.bag.is_dir():
        raise ValueError(f"bag directory was not found: {args.bag}")
    if not (args.rosbagel_root / "rosbagel").is_dir():
        raise ValueError(f"ROSBagel package was not found: {args.rosbagel_root}")
    if not args.lane_weights.is_file():
        raise ValueError(f"lane weights were not found: {args.lane_weights}")
    if args.output.exists():
        raise FileExistsError(f"output already exists: {args.output}")


def _resolve_device(raw_device: str) -> torch.device:
    if raw_device.lower() == "auto":
        return torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    device = torch.device(raw_device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(f"CUDA device {device} was requested but CUDA is unavailable")
    return device


def _prepare_source(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if image.shape[2] == 3:
        return image
    raise ValueError(f"unsupported decoded image shape: {image.shape}")


@torch.inference_mode()
def _predict_lane_mask(
    image_bgr: np.ndarray,
    model: torch.nn.Module,
    device: torch.device,
    use_half: bool,
    lane_config: object,
    preprocess_lane_image: object,
    lane_probability_from_logits: object,
    restore_lane_map: object,
    postprocess_lane_probability: object,
) -> np.ndarray:
    lane_input, shape = preprocess_lane_image(image_bgr, lane_config)
    lane_input = lane_input.to(device)
    lane_input = lane_input.half() if use_half else lane_input.float()
    lane_logits = model(lane_input)
    lane_probability = lane_probability_from_logits(lane_logits)[0]
    restored = restore_lane_map(lane_probability, shape).astype(np.float32)
    lane_mask = postprocess_lane_probability(restored, lane_config)
    return lane_mask.astype(np.uint8) * 255


def _build_manifest(
    args: argparse.Namespace,
    device: torch.device,
    frames: list[dict[str, int | str]],
    started: float,
) -> dict[str, object]:
    return {
        "source_bag": str(args.bag.resolve()),
        "source_bag_read_only": True,
        "topic": args.topic,
        "lane_weights": str(args.lane_weights.resolve()),
        "lane_weights_sha256": _sha256(args.lane_weights),
        "device": str(device),
        "frame_count": len(frames),
        "image_format": "binary uint8 PNG",
        "image_size": [640, 360],
        "nonempty_masks": sum(frame["nonzero_pixels"] > 0 for frame in frames),
        "generation_wall_sec": round(time.perf_counter() - started, 3),
        "frames": frames,
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


if __name__ == "__main__":
    main()
