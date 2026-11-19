"""Recompute the field assessment from four restored November 19 ROS 2 bags."""
import argparse
from bisect import bisect_right
from collections import Counter
from datetime import datetime, timedelta, timezone
import hashlib
import itertools
import json
from math import pi
from pathlib import Path
import sqlite3
import struct

JST = timezone(timedelta(hours=9))
RADIUS_M = 0.127
MAP_TOPIC = "/semantic_drive/map_status"
GATE_TOPIC = "/aiformula_control/motor_controller/gate_status"
CAN_TOPIC = "/aiformula_sensing/vehicle_info"
IMU_TOPIC = "/aiformula_sensing/vectornav/imu"
RED_GATE = "semantic_hold:red_wait_first_lap"


def stamp(ns):
    return datetime.fromtimestamp(ns // 1_000_000_000, JST).strftime("%H:%M:%S") + ".%03d" % ((ns // 1_000_000) % 1000)


def messages(db, topic):
    return db.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = "
        "(SELECT id FROM topics WHERE name = ?) ORDER BY timestamp, id", (topic,)
    )


def string_data(payload):
    assert payload[:4] == b"\x00\x01\x00\x00"
    size = struct.unpack_from("<I", payload, 4)[0]
    return payload[8:8 + size - 1].decode("utf-8")


def objects(db, topic):
    return [(time, json.loads(string_data(data))) for time, data in messages(db, topic)]


def can_rpm_pairs(db, topic, expected_id):
    result = []
    for time, data in messages(db, topic):
        assert data[:4] == b"\x00\x01\x00\x00"
        header_length = struct.unpack_from("<I", data, 12)[0]
        can_offset = (16 + header_length + 3) // 4 * 4
        can_id = struct.unpack_from("<I", data, can_offset)[0]
        if can_id != expected_id or data[can_offset + 4:can_offset + 8] != b"\0\0\0\x08":
            continue
        right, left = struct.unpack_from("<ii", data, can_offset + 8)
        result.append((time, right, left))
    return result


def speed(pair):
    return sum(pair) * pi * RADIUS_M / 60.0


def semantic(db):
    gate = objects(db, GATE_TOPIC)
    states = objects(db, MAP_TOPIC)
    wheel = can_rpm_pairs(db, CAN_TOPIC, 0x711)
    hold = [(t, g) for t, g in gate if g["reason"] == RED_GATE]
    start, end = hold[0][0], hold[-1][0]
    window = [x for x in wheel if start <= x[0] <= end]
    before_zero = next(x for x in window if x[1:] == (0, 0))
    go_time = next(t for t, g in gate if t > end and g["reason"] == "semantic_go")
    roll_time = next(x for x in wheel if x[0] > go_time and min(x[1:]) > 0)
    by_one_second = [x for x in wheel if end < x[0] <= end + 1_000_000_000]
    lo = start - 1_000_000_000
    hi = end + 1_000_000_000
    reason = lambda g: g["reason"]
    color = [string_data(b) for t, b in messages(db, "/aiformula_perception/yolop_a3/traffic_light/state") if start <= t <= end]
    pixels = [struct.unpack_from("<i", b, 4)[0] for t, b in messages(db, "/aiformula_perception/yolop_a3/red_pixels_count") if start <= t <= end]
    return {
        "first_and_last_red_gate_jst": [stamp(start), stamp(end)],
        "red_wait_state_messages": sum(v["traffic_reason"] == "red_wait_first_lap" for _, v in states),
        "red_gate_messages": len(hold),
        "red_gate_zero_command_and_eight_zero_can_bytes": sum(g["command"] == [0, 0] and g["can_data"] == [0] * 8 for _, g in hold),
        "separate_red_gate_windows": sum(1 for key, _ in itertools.groupby(gate, lambda x: reason(x[1]) == RED_GATE) if key),
        "wait_window_color_counts": dict(Counter(color)), "red_pixels_range": [min(pixels), max(pixels)],
        "first_feedback_right_left_rpm": list(window[0][1:]), "first_feedback_jst": stamp(window[0][0]),
        "first_feedback_both_zero_jst": stamp(before_zero[0]), "last_feedback_in_window_right_left_rpm": list(window[-1][1:]),
        "after_final_wait_semantic_go_jst": stamp(go_time), "next_positive_pair_jst": stamp(roll_time[0]),
        "last_feedback_within_next_second": {"jst": stamp(by_one_second[-1][0]), "right_left_rpm": list(by_one_second[-1][1:])},
        "plot": {
            "origin_ns": start,
            "wheel": [[round((t - start) / 1e9, 5), right, left] for t, right, left in wheel if lo <= t <= hi],
            "gate": [[round((t - start) / 1e9, 5), reason(g)] for t, g in gate if lo <= t <= hi],
        },
    }


def concept(db):
    gate = objects(db, "/concept/gate/status")
    state = objects(db, "/concept/vehicle_state")
    decisions = objects(db, "/concept/decision")
    timed_commands = [(t, struct.unpack("<6d", data[-48:])[0]) for t, data in messages(db, "/concept/auto_cmd")]
    commands = [v for _, v in timed_commands]
    times = [t for t, _ in gate]
    wheel_auto = []
    wheel = can_rpm_pairs(db, CAN_TOPIC, 0x711)
    for t, right, left in wheel:
        index = bisect_right(times, t) - 1
        if index >= 0 and t - times[index] <= 200_000_000 and gate[index][1]["mode"] == "auto":
            wheel_auto.append(speed((right, left)))
    bad_states = [(t, s) for t, s in state if not s["valid"]]
    bad_gates = [t for t, g in gate if g["reason"] == "invalid_vehicle_state"]
    summary = {
        "auto_cmd_messages": len(commands), "decision_messages": len(decisions),
        "published_linear_mps_range": [min(commands), max(commands)],
        "planning_targets_mps": sorted({d["target_speed_mps"] for _, d in decisions}),
        "gate_caps_mps": sorted({g["max_auto_speed_mps"] for _, g in gate}),
        "gate_reasons": dict(Counter(g["reason"] for _, g in gate)),
        "invalid_state_reasons": dict(Counter(s["reason"] for _, s in bad_states)),
        "raw_wheel_samples_during_recent_auto_gate": len(wheel_auto),
        "max_wheel_speed_in_that_window_mps": round(max(wheel_auto), 4) if wheel_auto else None,
    }
    reverse = []
    if min(commands) < 0:
        motor = [(t, struct.unpack("<6d", b[-48:])[0]) for t, b in messages(db, "/concept/internal/motor_cmd")]
        internal_can = can_rpm_pairs(db, "/concept/internal/motor_can", 0x210)
        for (start, value), (end, next_value) in zip(timed_commands, timed_commands[1:]):
            if value >= 0:
                continue
            negative = [t for t, v in motor if start <= t < end and v < 0]
            requests = Counter((right, left) for t, right, left in internal_can if start <= t < end)
            feedback = Counter((right, left) for t, right, left in wheel if start <= t < end)
            reverse.append({
                "start_jst": stamp(start), "next_command_jst": stamp(end), "requested_mps": value,
                "next_command_mps": next_value, "window_ms": round((end - start) / 1e6, 3),
                "negative_motor_inputs": len(negative),
                "first_to_last_negative_motor_input_ms": round((negative[-1] - negative[0]) / 1e6, 3) if negative else None,
                "internal_can_right_left_rpm_counts": [[r, l, count] for (r, l), count in sorted(requests.items())],
                "wheel_feedback_right_left_rpm_counts": [[r, l, count] for (r, l), count in sorted(feedback.items())],
            })
    summary["reverse_command_windows"] = reverse
    if bad_gates:
        first, last = bad_gates[0], bad_gates[-1]
        imu = []
        for t, raw in messages(db, IMU_TOPIC):
            sec, nano = struct.unpack_from("<iI", raw, 4)
            imu.append((t, sec * 1_000_000_000 + nano))
        imu_times = [t for t, _ in imu]
        sample_windows = []
        for t, status in bad_states:
            i = bisect_right(imu_times, t)
            if not 0 < i < len(imu):
                continue
            prev_t, prev_header = imu[i - 1]
            next_t, next_header = imu[i]
            sample_windows.append({
                "invalid_state_jst": stamp(t), "reason": status["reason"],
                "imu_record_gap_ms": round((next_t - prev_t) / 1e6, 2),
                "imu_header_gap_ms": round((next_header - prev_header) / 1e6, 2),
                "age_of_preceding_imu_header_at_recording_ms": round((t - prev_header) / 1e6, 2),
            })
        summary["invalid_gate_window"] = {
            "first_jst": stamp(first), "last_jst": stamp(last), "span_s": round((last - first) / 1e9, 3),
            "all_gate_reasons_after_first": dict(Counter(g["reason"] for t, g in gate if t >= first)),
            "next_valid_vehicle_state_jst": next((stamp(t) for t, s in state if t > first and s["valid"]), None),
            "imu_at_invalid_observations": sample_windows,
            "plot": {
                "origin_ns": first,
                "state": [[round((t - first) / 1e9, 5), s["valid"]] for t, s in state if first - 900_000_000 <= t <= last],
                "gate": [[round((t - first) / 1e9, 5), g["mode"]] for t, g in gate if first - 900_000_000 <= t <= last],
            },
        }
    return summary


def file_hash(path):
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(block)
    return checksum.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, default=Path(__file__).resolve().parents[1] / "bag")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    catalog = json.loads((Path(__file__).resolve().parents[1] / "bag/index.json").read_text())
    records = []
    for row in catalog:
        semantic_trial = row["session"] == "rosbag2_2026_11_19-16_55_15"
        if not semantic_trial and not row["session"].startswith("concept_20261119_17"):
            continue
        item, = [f for f in row["files"] if f["file"].endswith(".db3")]
        db_file = args.bags / row["category"] / row["session"] / item["file"]
        assert file_hash(db_file) == item["sha256"]
        db = sqlite3.connect(db_file.resolve().as_uri() + "?mode=ro&immutable=1", uri=True)
        try:
            result = semantic(db) if semantic_trial else concept(db)
        finally:
            db.close()
        records.append({"session": row["session"], "db_sha256": item["sha256"], "result": result})
    assert len(records) == 4
    args.output.write_text(json.dumps(records, ensure_ascii=False, separators=(",", ":")) + "\n")


if __name__ == "__main__":
    main()
