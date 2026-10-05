#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
from argparse import ArgumentTypeError
from pathlib import Path
from typing import Any
from uuid import uuid4

import cv2

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from yolop_a2_runtime import DualPerceptionModel
from tools.demo_board import build_demo_board


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
OUTPUT_KINDS = (
    "lane_mask",
    "lane_overlay",
    "obstacle_mask",
    "object_overlay",
    "combined_overlay",
    "detections",
)
MANAGED_OUTPUT_NAMES = frozenset(
    (*OUTPUT_KINDS, "summary.json", "demo_board.jpg", "smoke_report.json")
)


class DemoInputError(ValueError):
    pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run YOLOP_A2 deployment inference.")
    parser.add_argument("--source", type=Path, required=True)
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
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "inference" / "demo",
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument("--object-score", type=_probability, default=0.50)
    parser.add_argument("--cone-score", type=_probability, default=0.50)
    parser.add_argument("--board-max-images", type=_positive_integer, default=8)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    try:
        run_demo(args)
    except DemoInputError as error:
        parser.error(str(error))


def run_demo(args: argparse.Namespace) -> dict[str, Any]:
    image_paths = _discover_images(args.source)
    if not image_paths:
        raise DemoInputError(f"no images found under {args.source}")
    _validate_source_output_boundary(args.source, args.output)
    _validate_checkpoint_path(args.lane_weights, "lane")
    _validate_checkpoint_path(args.object_weights, "object")
    _validate_output_target(args.output)

    model = DualPerceptionModel(
        args.lane_weights,
        args.object_weights,
        device=args.device,
        object_score_threshold=args.object_score,
        cone_score_threshold=args.cone_score,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    staging_root = Path(
        tempfile.mkdtemp(
            prefix=f".{args.output.name}-staging-",
            dir=args.output.parent,
        )
    )
    try:
        summary = _generate_outputs(staging_root, image_paths, model, args)
        build_demo_board(
            staging_root,
            summary["images"],
            max_images=args.board_max_images,
        )
        _publish_staged_output(staging_root, args.output)
    except Exception:
        if staging_root.exists():
            shutil.rmtree(staging_root)
        raise

    print(f"processed_images={len(summary['images'])}")
    print(f"output={args.output.resolve()}")
    print(f"demo_board={(args.output / 'demo_board.jpg').resolve()}")
    return summary


def _generate_outputs(
    staging_root: Path,
    image_paths: list[Path],
    model: DualPerceptionModel,
    args: argparse.Namespace,
) -> dict[str, Any]:
    output_directories = {name: staging_root / name for name in OUTPUT_KINDS}
    for directory in output_directories.values():
        directory.mkdir(parents=True)

    summaries: list[dict[str, Any]] = []
    for index, image_path in enumerate(image_paths):
        image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image_bgr is None:
            raise DemoInputError(f"OpenCV could not read image: {image_path}")
        output = model.predict(image_bgr)
        stem = f"{index:04d}_{image_path.stem}"
        _write_image(output_directories["lane_mask"] / f"{stem}.png", output.lane_mask)
        _write_image(output_directories["lane_overlay"] / f"{stem}.jpg", output.lane_overlay)
        _write_image(
            output_directories["obstacle_mask"] / f"{stem}.png",
            output.obstacle_mask,
        )
        _write_image(
            output_directories["object_overlay"] / f"{stem}.jpg",
            output.object_overlay,
        )
        _write_image(
            output_directories["combined_overlay"] / f"{stem}.jpg",
            output.combined_overlay,
        )
        payload = {"source": str(image_path.resolve()), **output.detections_payload()}
        (output_directories["detections"] / f"{stem}.json").write_text(
            json.dumps(payload, indent=2, ensure_ascii=True) + "\n",
            encoding="utf-8",
        )
        summaries.append(payload)

    summary = {
        "lane_weights": str(args.lane_weights.resolve()),
        "object_weights": str(args.object_weights.resolve()),
        "device": str(model.device),
        "object_score_threshold": args.object_score,
        "cone_score_threshold": args.cone_score,
        "object_checkpoint_epoch": int(model.object_checkpoint["epoch"]),
        "object_checkpoint_metrics": model.object_checkpoint.get("metrics", {}),
        "demo_board": "demo_board.jpg",
        "images": summaries,
    }
    (staging_root / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    return summary


def _discover_images(source: Path) -> list[Path]:
    if source.is_file() and source.suffix.lower() in IMAGE_EXTENSIONS:
        return [source]
    if source.is_dir():
        return sorted(
            path
            for path in source.rglob("*")
            if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        )
    raise DemoInputError(f"image source was not found or unsupported: {source}")


def _validate_source_output_boundary(source: Path, output: Path) -> None:
    source_path = source.resolve()
    output_path = output.resolve()
    overlaps = (
        source_path == output_path
        or output_path in source_path.parents
        or (source.is_dir() and source_path in output_path.parents)
    )
    if overlaps:
        raise DemoInputError(
            f"source and output directories must be disjoint: source={source_path}, "
            f"output={output_path}"
        )


def _validate_checkpoint_path(path: Path, name: str) -> None:
    if not path.is_file():
        raise DemoInputError(f"{name} checkpoint was not found: {path}")


def _validate_output_target(output: Path) -> None:
    if not output.exists():
        return
    is_junction = getattr(output, "is_junction", lambda: False)()
    if output.is_symlink() or is_junction or not output.is_dir():
        raise DemoInputError(f"output must be a regular directory: {output}")
    unmanaged = sorted(
        entry.name for entry in output.iterdir() if entry.name not in MANAGED_OUTPUT_NAMES
    )
    if unmanaged:
        names = ", ".join(unmanaged[:5])
        raise DemoInputError(
            f"refusing to replace output containing unmanaged entries: {names}"
        )


def _publish_staged_output(staging_root: Path, output: Path) -> None:
    if not output.exists():
        staging_root.replace(output)
        return

    backup = output.with_name(f".{output.name}-previous-{uuid4().hex}")
    output.replace(backup)
    try:
        staging_root.replace(output)
    except Exception:
        backup.replace(output)
        raise
    shutil.rmtree(backup)


def _probability(raw_value: str) -> float:
    try:
        value = float(raw_value)
    except ValueError as error:
        raise ArgumentTypeError(f"expected a number, got {raw_value!r}") from error
    if not 0.0 <= value <= 1.0:
        raise ArgumentTypeError(f"expected a value between 0 and 1, got {raw_value!r}")
    return value


def _positive_integer(raw_value: str) -> int:
    try:
        value = int(raw_value)
    except ValueError as error:
        raise ArgumentTypeError(f"expected an integer, got {raw_value!r}") from error
    if value < 1:
        raise ArgumentTypeError(f"expected a positive integer, got {raw_value!r}")
    return value


def _write_image(path: Path, image: object) -> None:
    if not cv2.imwrite(str(path), image):
        raise OSError(f"failed to write image: {path}")


if __name__ == "__main__":
    main()
