"""Read-only record validation and source sealing; no image/model imports."""
from __future__ import annotations

import bisect
import hashlib
import json
import math
from pathlib import Path

SCHEMA = "aiformula_neo_stop_wait_media/v1"
RENDERER = Path(__file__).resolve().parent / "shared" / "recording_renderer.py"
RENDERER_SHA = "889a952a9c229d167b8b54100290db98def2f5a8cd40ca88137e8f5c7d1e32af"


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def beneath(root, name):
    path = (Path(root) / str(name)).resolve()
    path.relative_to(Path(root).resolve())
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def rows(path):
    result = [json.loads(line) for line in Path(path).read_text(encoding="utf-8-sig").splitlines() if line.strip()]
    previous = -math.inf
    for index, row in enumerate(result):
        wall = row.get("wall")
        if isinstance(wall, bool) or not isinstance(wall, (int, float)) or not math.isfinite(wall) or wall < previous:
            raise ValueError(f"Invalid/non-monotone wall time {path}:{index + 1}")
        previous = wall
    return result


def latest(data, times, wall):
    index = bisect.bisect_right(times, wall) - 1
    return (data[index], index) if index >= 0 else ({}, -1)


def get_path(value, path):
    for name in path:
        if not isinstance(value, dict) or name not in value:
            raise ValueError(f"Missing policy path {path}")
        value = value[name]
    return value


def check_policy(source, policy):
    if policy.get("reviewed_mode") != "stop_wait" or policy.get("bbox_distance_enabled") is not False or policy.get("automatic_detour_enabled") is not False:
        raise ValueError("Only the reviewed stop_wait mode with bbox and automatic detour disabled is allowed")
    checks = policy.get("checks")
    if not isinstance(checks, list) or len(checks) < 4:
        raise ValueError("Explicit recorded-configuration mode/bbox/detour checks required")
    roles = {"traffic_policy": "stop_wait", "bbox_distance": False,
             "distance_estimation": False, "automatic_detour": False}
    for role, expected in roles.items():
        matches = [item for item in checks if item.get("role") == role]
        if len(matches) != 1 or type(matches[0].get("equals")) is not type(expected) or matches[0]["equals"] != expected:
            raise ValueError(f"One exact recorded configuration check for {role} is required")
    for check in checks:
        actual = get_path(read_json(beneath(source, check["file"])), check["path"])
        expected = check["equals"]
        if type(actual) is not type(expected) or actual != expected:
            raise ValueError(f"Recorded policy mismatch {check['file']}:{check['path']}: {actual!r}")


def referenced_files(source, frames, traces):
    names = set()
    for row in frames:
        names.add(row["path"])
        names.update(row[key] for key in ("overlay", "rgb") if row.get(key))
    for row in traces:
        names.update(row[key] for key in ("overlay", "rgb", "lane_packet_file", "rgb_packet_file") if row.get(key))
    for name in names:
        beneath(source, name)
    return sorted(names)


def schedule(start, end, events, motion, speed):
    if speed not in (1, 2, 4):
        raise ValueError("Supported full-coverage playback speeds: 1, 2, 4")
    windows = [(start, min(end, start + 3), "capture_start"), (max(start, end - 3), end, "capture_end")]
    for event in events:
        if start <= event["wall"] <= end:
            label = str(event.get("kind") or event.get("event") or event.get("type") or "recorded_event")
            windows.append((max(start, event["wall"] - 3), min(end, event["wall"] + 6), label))
    prior = None
    for row in motion:
        closure = row.get("topo_closure") or ((row.get("semantic") or {}).get("status") or {}).get("closure") or {}
        key = (row.get("traffic_stop_required"), row.get("worker_generation"), row.get("semantic_running"),
               closure.get("closed"), closure.get("accepted"), closure.get("state"), closure.get("status"))
        if prior is not None and key != prior and start <= row["wall"] <= end:
            windows.append((max(start, row["wall"] - 3), min(end, row["wall"] + 6), "recorded_state_change"))
        prior = key
    if speed == 1:
        return [{"start": start, "end": end, "speed": 1, "reasons": ["requested_full_1x"]}]
    cuts = sorted({start, end, *(point for a, b, _ in windows for point in (a, b))})
    segments = []
    for a, b in zip(cuts, cuts[1:]):
        if b <= a:
            continue
        reasons = sorted({label for lo, hi, label in windows if lo < b and hi > a})
        segment = {"start": a, "end": b, "speed": 1 if reasons else speed, "reasons": reasons}
        if segments and segment["speed"] == segments[-1]["speed"]:
            segments[-1]["end"] = b
            segments[-1]["reasons"] = sorted(set(segments[-1]["reasons"] + reasons))
        else:
            segments.append(segment)
    return segments
