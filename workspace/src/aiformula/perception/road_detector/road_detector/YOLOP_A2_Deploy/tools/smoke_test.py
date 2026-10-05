#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PROJECT_ROOT.parent / "YOLOP_A2" / "demo_inputs" / "final"
EXPECTED_LABELS = {
    "01_red_cones": {"red_traffic_cone"},
    "02_yellow_cone": {"yellow_traffic_cone"},
    "03_green_cones": {"green_traffic_cone"},
    "04_blue_cones": {"blue_traffic_cone"},
    "05_person": {"person"},
    "06_red_traffic_light": {"red_traffic_light"},
    "07_green_traffic_light": {"green_traffic_light"},
}
TRACK_CASES = frozenset(tuple(EXPECTED_LABELS)[:5])
LIGHT_CASES = frozenset(tuple(EXPECTED_LABELS)[5:])
RED_PIXEL_ENTER_THRESHOLD = 600
RED_PIXEL_CLEAR_THRESHOLD = 100
OUTPUT_FILES = {
    "lane_mask": "png",
    "lane_overlay": "jpg",
    "obstacle_mask": "png",
    "object_overlay": "jpg",
    "combined_overlay": "jpg",
    "detections": "json",
}


class SmokeFailure(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the local YOLOP_A2 deployment smoke gate."
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "inference" / "smoke",
    )
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
    parser.add_argument("--device", default="auto")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    started = time.perf_counter()
    try:
        manifest = _validate_manifest(args.lane_weights, args.object_weights)
        demo_stdout = _run_demo(args, manifest)
        summary, checks = _validate_outputs(args.output, manifest)
        performance = _performance_summary(summary)
    except SmokeFailure as error:
        print(f"smoke_status=failed\nreason={error}", file=sys.stderr)
        return 1

    report = {
        "status": "passed",
        "elapsed_seconds": round(time.perf_counter() - started, 3),
        "environment": {
            "python": sys.version.split()[0],
            "torch": torch.__version__,
            "device": summary["device"],
            "cuda_available": torch.cuda.is_available(),
            "cuda_device": (
                torch.cuda.get_device_name(0) if torch.cuda.is_available() else None
            ),
        },
        "weights": manifest,
        "checks": checks,
        "performance": performance,
        "demo_stdout": demo_stdout,
    }
    report_path = args.output / "smoke_report.json"
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    print("smoke_status=passed")
    print(f"processed_images={len(summary['images'])}")
    print(f"warm_median_ms={performance['warm_median_ms']}")
    print(f"report={report_path.resolve()}")
    print(f"demo_board={(args.output / 'demo_board.jpg').resolve()}")
    return 0


def _validate_manifest(lane_weights: Path, object_weights: Path) -> dict[str, Any]:
    manifest_path = PROJECT_ROOT / "weights" / "manifest.json"
    if not manifest_path.is_file():
        raise SmokeFailure(f"weight manifest is missing: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    lane_hash = _sha256(lane_weights)
    object_hash = _sha256(object_weights)
    expected_lane_hash = str(manifest.get("lane", {}).get("sha256", ""))
    expected_object_hash = str(manifest.get("object", {}).get("sha256", ""))
    if lane_hash != expected_lane_hash:
        raise SmokeFailure(
            f"lane checkpoint hash mismatch: expected={expected_lane_hash}, actual={lane_hash}"
        )
    if object_hash != expected_object_hash:
        raise SmokeFailure(
            "object checkpoint hash mismatch: "
            f"expected={expected_object_hash}, actual={object_hash}"
        )
    return {
        "lane_sha256": lane_hash,
        "object_sha256": object_hash,
        "object_epoch": int(manifest["object"]["source_epoch"]),
        "object_score": float(manifest["thresholds"]["object_score"]),
        "cone_score": float(manifest["thresholds"]["cone_score"]),
    }


def _run_demo(args: argparse.Namespace, manifest: dict[str, Any]) -> str:
    command = [
        sys.executable,
        str(PROJECT_ROOT / "tools" / "demo.py"),
        "--source",
        str(args.source),
        "--output",
        str(args.output),
        "--lane-weights",
        str(args.lane_weights),
        "--object-weights",
        str(args.object_weights),
        "--device",
        args.device,
        "--object-score",
        str(manifest["object_score"]),
        "--cone-score",
        str(manifest["cone_score"]),
    ]
    completed = subprocess.run(
        command,
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode:
        details = completed.stderr.strip() or completed.stdout.strip()
        raise SmokeFailure(f"demo command failed with exit {completed.returncode}: {details}")
    return completed.stdout.strip()


def _validate_outputs(
    output_root: Path,
    manifest: dict[str, Any],
) -> tuple[dict[str, Any], list[str]]:
    summary_path = output_root / "summary.json"
    if not summary_path.is_file():
        raise SmokeFailure(f"demo summary is missing: {summary_path}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    images = summary.get("images")
    if not isinstance(images, list) or len(images) != len(EXPECTED_LABELS):
        raise SmokeFailure(
            f"expected {len(EXPECTED_LABELS)} image results, found "
            f"{len(images) if isinstance(images, list) else 'invalid'}"
        )
    if float(summary.get("object_score_threshold", -1.0)) != manifest["object_score"]:
        raise SmokeFailure("object score threshold does not match the weight manifest")
    if float(summary.get("cone_score_threshold", -1.0)) != manifest["cone_score"]:
        raise SmokeFailure("cone score threshold does not match the weight manifest")
    if int(summary.get("object_checkpoint_epoch", -1)) != manifest["object_epoch"]:
        raise SmokeFailure("object checkpoint epoch does not match the weight manifest")

    for directory_name, extension in OUTPUT_FILES.items():
        count = len(list((output_root / directory_name).glob(f"*.{extension}")))
        if count != len(images):
            raise SmokeFailure(
                f"{directory_name} contains {count} files for {len(images)} images"
            )

    seen_cases: set[str] = set()
    for index, payload in enumerate(images):
        case = Path(str(payload["source"])).stem
        if case not in EXPECTED_LABELS:
            raise SmokeFailure(f"unexpected smoke case: {case}")
        seen_cases.add(case)
        _validate_case(output_root, index, case, payload)
    if seen_cases != set(EXPECTED_LABELS):
        missing = sorted(set(EXPECTED_LABELS) - seen_cases)
        raise SmokeFailure(f"smoke cases are missing: {missing}")

    board = cv2.imread(str(output_root / "demo_board.jpg"), cv2.IMREAD_COLOR)
    if board is None or board.size == 0:
        raise SmokeFailure("demo board is missing or unreadable")
    checks = [
        "weight hashes match manifest",
        "seven semantic cases detected",
        "output counts and image shapes match",
        "lane and obstacle masks are binary mono8",
        "track obstacle masks exceed 1000 pixels",
        "traffic-light boxes are excluded from obstacle masks",
        "red-pixel adapter thresholds are satisfied",
        "demo board is readable",
    ]
    return summary, checks


def _validate_case(
    output_root: Path,
    index: int,
    case: str,
    payload: dict[str, Any],
) -> None:
    source = cv2.imread(str(payload["source"]), cv2.IMREAD_COLOR)
    if source is None:
        raise SmokeFailure(f"source image is unreadable: {payload['source']}")
    stem = f"{index:04d}_{case}"
    lane = cv2.imread(
        str(output_root / "lane_mask" / f"{stem}.png"),
        cv2.IMREAD_GRAYSCALE,
    )
    obstacle = cv2.imread(
        str(output_root / "obstacle_mask" / f"{stem}.png"),
        cv2.IMREAD_GRAYSCALE,
    )
    combined = cv2.imread(
        str(output_root / "combined_overlay" / f"{stem}.jpg"),
        cv2.IMREAD_COLOR,
    )
    if lane is None or obstacle is None or combined is None:
        raise SmokeFailure(f"one or more image outputs are unreadable for {case}")
    if lane.shape != source.shape[:2] or obstacle.shape != source.shape[:2]:
        raise SmokeFailure(f"mask dimensions do not match source for {case}")
    if combined.shape != source.shape:
        raise SmokeFailure(f"overlay dimensions do not match source for {case}")
    if not _is_binary_mask(lane) or not _is_binary_mask(obstacle):
        raise SmokeFailure(f"mask contains values outside 0/255 for {case}")

    detections = payload.get("detections", [])
    labels = {
        str(detection.get("semantic_label"))
        for detection in detections
        if isinstance(detection, dict)
    }
    missing_labels = EXPECTED_LABELS[case] - labels
    if missing_labels:
        raise SmokeFailure(f"{case} is missing labels: {sorted(missing_labels)}")

    if case in TRACK_CASES:
        if np.count_nonzero(lane) == 0:
            raise SmokeFailure(f"lane mask is empty for {case}")
        if np.count_nonzero(obstacle) <= 1000:
            raise SmokeFailure(f"obstacle mask is below the 1000-pixel gate for {case}")
    if case in LIGHT_CASES:
        if any(label.endswith("traffic_cone") for label in labels):
            raise SmokeFailure(f"traffic light case contains a cone detection: {case}")
        _validate_light_exclusion(obstacle, detections, case)
        _validate_light_state(payload, case)

    detection_path = output_root / "detections" / f"{stem}.json"
    detection_payload = json.loads(detection_path.read_text(encoding="utf-8"))
    if detection_payload.get("source") != payload.get("source"):
        raise SmokeFailure(f"detection JSON source mismatch for {case}")


def _validate_light_exclusion(
    obstacle_mask: np.ndarray,
    detections: list[dict[str, Any]],
    case: str,
) -> None:
    height, width = obstacle_mask.shape
    lights = [
        detection
        for detection in detections
        if detection.get("class_name") == "traffic light"
    ]
    if not lights:
        raise SmokeFailure(f"traffic light detection is missing for {case}")
    for detection in lights:
        x1, y1, x2, y2 = (float(value) for value in detection["box_xyxy"])
        left = min(max(int(np.floor(x1)), 0), width)
        top = min(max(int(np.floor(y1)), 0), height)
        right = min(max(int(np.ceil(x2)), 0), width)
        bottom = min(max(int(np.ceil(y2)), 0), height)
        if np.count_nonzero(obstacle_mask[top:bottom, left:right]):
            raise SmokeFailure(f"traffic light entered the obstacle mask for {case}")


def _validate_light_state(payload: dict[str, Any], case: str) -> None:
    state = str(payload.get("traffic_light_state"))
    red_pixels = int(payload.get("red_pixels_count", -1))
    if case == "06_red_traffic_light":
        if state != "red" or red_pixels < RED_PIXEL_ENTER_THRESHOLD:
            raise SmokeFailure(
                f"red-light adapter contract failed: state={state}, red_pixels={red_pixels}"
            )
        return
    if state != "green" or red_pixels > RED_PIXEL_CLEAR_THRESHOLD:
        raise SmokeFailure(
            f"green-light adapter contract failed: state={state}, red_pixels={red_pixels}"
        )


def _is_binary_mask(mask: np.ndarray) -> bool:
    return mask.dtype == np.uint8 and not np.any((mask != 0) & (mask != 255))


def _performance_summary(summary: dict[str, Any]) -> dict[str, float]:
    totals = [float(image["timings_ms"]["total"]) for image in summary["images"]]
    warm = totals[1:] if len(totals) > 1 else totals
    return {
        "cold_first_ms": round(totals[0], 3),
        "warm_median_ms": round(statistics.median(warm), 3),
        "warm_max_ms": round(max(warm), 3),
    }


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise SmokeFailure(f"checkpoint is missing: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


if __name__ == "__main__":
    raise SystemExit(main())
