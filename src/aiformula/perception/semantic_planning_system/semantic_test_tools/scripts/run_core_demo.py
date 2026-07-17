#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import shutil
import struct
import sys
import zlib

import numpy as np

WORKSPACE_SRC = Path(__file__).resolve().parents[2]
CORE_SRC = WORKSPACE_SRC / "semantic_planner_core"
if str(CORE_SRC) not in sys.path:
    sys.path.insert(0, str(CORE_SRC))

from semantic_planner_core.constraints import SemanticConstraints
from semantic_planner_core.costmap import SemanticCostmapBuilder
from semantic_planner_core.planner import LocalPrimitivePlanner
from semantic_planner_core.primitives import PrimitiveName, default_primitives


SCENARIOS = ("red_gate_left_blocked", "red_gate_left_open")
LEFT_PRIMITIVES = {
    PrimitiveName.SMALL_LEFT.value,
    PrimitiveName.MEDIUM_LEFT.value,
    PrimitiveName.STRONG_LEFT.value,
    PrimitiveName.LEFT_THEN_ALIGN.value,
}
CSV_COLUMNS = [
    "primitive_name",
    "valid",
    "selected",
    "total_cost",
    "semantic_cost",
    "lethal_hit",
    "red_gate_hit",
    "curvature_cost",
    "smoothness_cost",
    "heading_cost",
    "progress_cost",
    "reason",
]


def make_masks(scenario: str, height: int, width: int) -> tuple[np.ndarray, np.ndarray]:
    # Image convention: bottom rows are near the robot, top rows are farther ahead.
    # x increases left to right; y increases top to bottom.
    assert height > 20 and width > 20
    road = np.zeros((height, width), dtype=np.uint8)
    offroad = np.zeros_like(road)

    road[int(height * 0.35) :, int(width * 0.25) : int(width * 0.75)] = 255
    offroad[:, : int(width * 0.16)] = 255
    offroad[:, int(width * 0.82) :] = 255

    if scenario == "red_gate_left_open":
        road[int(height * 0.42) :, int(width * 0.16) : int(width * 0.48)] = 255
    elif scenario == "red_gate_left_blocked":
        offroad[int(height * 0.42) :, : int(width * 0.58)] = 255
    else:
        raise ValueError(f"Unknown scenario: {scenario}")
    return road, offroad


def red_gate_constraints() -> SemanticConstraints:
    return SemanticConstraints(
        red_light_active=True,
        red_forward_forbidden=True,
        allow_lane_follow=False,
        diversion_active=True,
        mode="RED_FORWARD_GATE",
    )


def run_scenario(scenario: str, out_dir: Path, height: int, width: int) -> tuple[str, int, Path]:
    road, offroad = make_masks(scenario, height, width)
    constraints = red_gate_constraints()
    builder = SemanticCostmapBuilder()
    costmap = builder.build(road, offroad, constraints)
    planner = LocalPrimitivePlanner()
    selected, details = planner.plan(costmap, constraints, current_yaw=0.0, theta_ref=0.0)

    scenario_dir = out_dir / scenario
    scenario_dir.mkdir(parents=True, exist_ok=True)
    save_png(scenario_dir / "road_mask.png", road)
    save_png(scenario_dir / "offroad_mask.png", offroad)
    save_png(scenario_dir / "costmap.png", costmap)
    save_png(scenario_dir / "selected_path_overlay.png", make_selected_overlay(costmap, selected.name))
    save_png(scenario_dir / "all_primitives_overlay.png", make_all_primitives_overlay(costmap, details, selected.name))

    rows = build_debug_rows(details, selected.name)
    write_csv(scenario_dir / "planner_debug.csv", rows)
    print_debug_table(scenario, rows)
    assert_scenario(scenario, selected.name, details)
    return selected.name, count_valid_moving(details), scenario_dir


def build_debug_rows(details: dict, selected_name: str) -> list[dict[str, str]]:
    rows = []
    for primitive in default_primitives():
        info = details.get(primitive.name, {})
        selected = primitive.name == selected_name
        rows.append(
            {
                "primitive_name": primitive.name,
                "valid": str(bool(info.get("valid", False))),
                "selected": str(selected),
                "total_cost": format_cost(info.get("total", "")),
                "semantic_cost": format_cost(info.get("semantic", "")),
                "lethal_hit": str(bool(info.get("lethal_hit", False))),
                "red_gate_hit": str(bool(info.get("red_gate_hit", False))),
                "curvature_cost": format_cost(info.get("curvature", "")),
                "smoothness_cost": format_cost(info.get("smoothness", "")),
                "heading_cost": format_cost(info.get("heading", "")),
                "progress_cost": format_cost(info.get("progress", "")),
                "reason": human_reason(info, selected),
            }
        )
    return rows


def human_reason(info: dict, selected: bool) -> str:
    reason = str(info.get("reason", "unknown"))
    if selected and reason == "fallback_stop":
        return "stop_fallback_no_valid_motion"
    if selected:
        return "selected_lowest_cost"
    if reason == "lethal_cost":
        return "invalid_lethal_cost"
    if reason == "red_forward_gate":
        return "invalid_red_forward_gate"
    if bool(info.get("valid", False)):
        return "valid_but_higher_cost"
    return reason


def assert_scenario(scenario: str, selected_name: str, details: dict) -> None:
    slow = details[PrimitiveName.SLOW_FORWARD.value]
    assert slow["valid"] is False
    assert slow["red_gate_hit"] is True

    if scenario == "red_gate_left_blocked":
        assert selected_name == PrimitiveName.STOP.value
        assert all(
            details[name]["valid"] is False and details[name]["lethal_hit"] is True
            for name in LEFT_PRIMITIVES
        )
    elif scenario == "red_gate_left_open":
        valid_left = [name for name in LEFT_PRIMITIVES if details[name]["valid"]]
        assert valid_left, "Expected at least one valid left primitive"
        assert selected_name != PrimitiveName.STOP.value
        assert selected_name != PrimitiveName.SLOW_FORWARD.value
        assert selected_name in LEFT_PRIMITIVES


def count_valid_moving(details: dict) -> int:
    return sum(
        1
        for primitive in default_primitives()
        if not primitive.is_stop and bool(details.get(primitive.name, {}).get("valid", False))
    )


def make_selected_overlay(costmap: np.ndarray, selected_name: str) -> np.ndarray:
    rgb = np.repeat(costmap[..., None], 3, axis=2).astype(np.uint8)
    for primitive in default_primitives():
        if primitive.name != selected_name or primitive.is_stop:
            continue
        pixels = [to_pixel(costmap.shape, point) for point in primitive.path_points]
        for start, end in zip(pixels, pixels[1:]):
            draw_line(rgb, start, end, (0, 255, 0))
        for point in pixels:
            draw_disc(rgb, point, 4, (0, 255, 0))
    return rgb


def make_all_primitives_overlay(costmap: np.ndarray, details: dict, selected_name: str) -> np.ndarray:
    rgb = np.repeat(costmap[..., None], 3, axis=2).astype(np.uint8)
    for primitive in default_primitives():
        if primitive.is_stop or not primitive.path_points:
            continue
        info = details.get(primitive.name, {})
        if primitive.name == selected_name:
            color = (0, 255, 0)
            radius = 3
        elif not bool(info.get("valid", False)):
            color = (255, 64, 64)
            radius = 2
        else:
            color = (0, 180, 255)
            radius = 2
        pixels = [to_pixel(costmap.shape, point) for point in primitive.path_points]
        for start, end in zip(pixels, pixels[1:]):
            draw_line(rgb, start, end, color)
        for point in pixels:
            draw_disc(rgb, point, radius, color)
    return rgb


def to_pixel(shape: tuple[int, ...], point: tuple[float, float]) -> tuple[int, int]:
    height, width = shape[:2]
    x_ratio, y_ratio = point
    x = int(round(max(0.0, min(1.0, x_ratio)) * (width - 1)))
    y = int(round(max(0.0, min(1.0, y_ratio)) * (height - 1)))
    return x, y


def draw_line(image: np.ndarray, start: tuple[int, int], end: tuple[int, int], color: tuple[int, int, int]) -> None:
    x0, y0 = start
    x1, y1 = end
    steps = max(abs(x1 - x0), abs(y1 - y0), 1)
    for index in range(steps + 1):
        t = index / steps
        x = int(round(x0 + (x1 - x0) * t))
        y = int(round(y0 + (y1 - y0) * t))
        if 0 <= y < image.shape[0] and 0 <= x < image.shape[1]:
            image[y, x] = color


def draw_disc(image: np.ndarray, center: tuple[int, int], radius: int, color: tuple[int, int, int]) -> None:
    cx, cy = center
    for y in range(cy - radius, cy + radius + 1):
        for x in range(cx - radius, cx + radius + 1):
            if 0 <= y < image.shape[0] and 0 <= x < image.shape[1]:
                if (x - cx) ** 2 + (y - cy) ** 2 <= radius**2:
                    image[y, x] = color


def save_png(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    arr = np.asarray(image, dtype=np.uint8)
    if arr.ndim == 2:
        color_type = 0
        raw_rows = [b"\x00" + arr[y].tobytes() for y in range(arr.shape[0])]
    elif arr.ndim == 3 and arr.shape[2] == 3:
        color_type = 2
        raw_rows = [b"\x00" + arr[y].tobytes() for y in range(arr.shape[0])]
    else:
        raise ValueError(f"Unsupported PNG image shape: {arr.shape}")
    data = zlib.compress(b"".join(raw_rows), level=9)
    with path.open("wb") as stream:
        stream.write(b"\x89PNG\r\n\x1a\n")
        stream.write(png_chunk(b"IHDR", struct.pack(">IIBBBBB", arr.shape[1], arr.shape[0], 8, color_type, 0, 0, 0)))
        stream.write(png_chunk(b"IDAT", data))
        stream.write(png_chunk(b"IEND", b""))


def png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    payload = chunk_type + data
    return struct.pack(">I", len(data)) + payload + struct.pack(">I", zlib.crc32(payload) & 0xFFFFFFFF)


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def print_debug_table(scenario: str, rows: list[dict[str, str]]) -> None:
    print(f"\nscenario: {scenario}")
    print(",".join(CSV_COLUMNS))
    for row in rows:
        print(",".join(row[column] for column in CSV_COLUMNS))


def format_cost(value: object) -> str:
    if value == "":
        return ""
    number = float(value)
    if math.isinf(number):
        return "inf"
    return f"{number:.6f}"


def prepare_output_dir(out_dir: Path) -> None:
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="all", choices=["all", *SCENARIOS])
    parser.add_argument("--height", type=int, default=120)
    parser.add_argument("--width", type=int, default=160)
    parser.add_argument("--output-dir", default="debug_outputs")
    args = parser.parse_args()

    scenarios = SCENARIOS if args.scenario == "all" else (args.scenario,)
    out_dir = Path(args.output_dir)
    prepare_output_dir(out_dir)

    results = {
        scenario: run_scenario(scenario, out_dir, args.height, args.width)
        for scenario in scenarios
    }

    print("\nsummary:")
    for scenario, (selected, valid_count, scenario_dir) in results.items():
        print(f"{scenario}: selected={selected}, valid_moving_primitives={valid_count}, output_dir={scenario_dir.resolve()}")
    print(f"debug_outputs: {out_dir.resolve()}")
    print("assertions: passed")


if __name__ == "__main__":
    main()
