"""Identify forward-driving speed and yaw response from native CAN and VectorNav."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


DT = 0.1
HORIZON = 12
TRAIN = ("1027_manual_1", "rosbag2_2026_10_27-speed1", "rosbag2_2026_10_27-stop1",
         "rosbag2_2026_10_27-turn1")
TEST = ("rosbag2_2026_10_27-mixed1", "rosbag2_2026_10_27-turn2", "rosbag2_2026_10_27-lya2")
PARAMETERS = ("speed_per_100rpm", "drive_tau_s", "lift_tau_s", "stop_tau_s",
              "left_yaw_gain", "right_yaw_gain", "straight_yaw_bias", "yaw_tau_s")


def step(speed, yaw_rate, common, difference, p):
    """common=(R+L)/200, difference=(R-L)/100, from RPM on the wire."""
    gain, drive, lift, stop, left, right, bias, turn = p
    target = gain * np.maximum(0.0, common)
    tau = np.where(target >= speed, drive, np.where(common < 0.05, stop, lift))
    next_speed = target + (speed - target) * np.exp(-DT / tau)
    target_yaw = np.where(difference >= 0, left, right) * difference + bias * common
    next_yaw = target_yaw + (yaw_rate - target_yaw) * np.exp(-DT / turn)
    return next_speed, next_yaw


def sample(stream, columns, times, max_age):
    # Zero-order hold. Command and sensor samples beyond gaps never enter a window.
    idx = np.searchsorted(stream[:, 1], times, side="right") - 1
    rows = stream[np.maximum(idx, 0)]
    return rows[:, columns], (idx >= 0) & (times - rows[:, 1] < max_age)


def recording(extracted, bags, name):
    meta = json.loads((extracted / (name + ".metadata.json")).read_text(encoding="utf8"))
    hashes = {}
    for src in meta["source_files"]:
        with (bags / name / src["file"]).open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if digest != src["sha256"]:
            raise ValueError("Recording differs from extracted data: " + src["file"])
        hashes[src["file"]] = digest
    with np.load(extracted / (name + ".npz"), allow_pickle=False) as data:
        can, vel, imu = (data[k] for k in ("bus_210", "velocity_body", "imu"))
    t = np.arange(max(a[0, 1] for a in (can, vel, imu)), min(a[-1, 1] for a in (can, vel, imu)), DT)
    # Some captures alternate zero and drive frames within 100 ms. Average the
    # actual zero-order-held CAN signal over the past interval instead of aliasing it.
    stamps = can[:, 1]
    integral = np.vstack((np.zeros(2), np.cumsum(np.diff(stamps)[:, None] * can[:-1, 7:9], axis=0)))
    def command_integral(times):
        idx = np.maximum(0, np.searchsorted(stamps, times, side="right")-1)
        return integral[idx] + (times-stamps[idx])[:, None]*can[idx, 7:9]
    rpm = (command_integral(t)-command_integral(t-DT))/DT
    _, ok_can = sample(can, [7], t, 0.13)
    # Any large gap overlapping the averaging interval invalidates that sample.
    for cut in np.flatnonzero(np.diff(stamps) > 0.13):
        ok_can &= ~((t > stamps[cut]+0.13) & (t-DT < stamps[cut+1]))
    ok_can &= t >= stamps[0]+DT
    xy, ok_v = sample(vel, [2, 3], t, 0.15)
    gyro, ok_w = sample(imu, [4], t, 0.15)
    common = rpm.mean(axis=1) / 100
    diff = (rpm[:, 0] - rpm[:, 1]) / 100
    values = np.column_stack((common, diff, np.linalg.norm(xy, axis=1), gyro[:, 0]))
    valid = ok_can & ok_v & ok_w & (common >= 0) & np.all(np.isfinite(values), axis=1)
    return values, valid, hashes


def windows(run, delay, stride):
    values, valid, _ = run
    starts = np.arange(4, len(values) - HORIZON, stride)
    eligible = np.lib.stride_tricks.sliding_window_view(valid, HORIZON + 5).all(axis=1)
    starts = starts[eligible[starts - 4] & ((values[starts, 2] > 0.25) | (values[starts, 0] > 0.25))]
    future = starts[:, None] + np.arange(HORIZON)
    return values[future-delay, :2], values[starts, 2:], values[future+1, 2:]


def predict(data, params):
    commands, initial, _ = data
    v, w = initial.T
    result = np.empty((len(v), HORIZON, 2))
    for j in range(HORIZON):
        v, w = step(v, w, commands[:, j, 0], commands[:, j, 1], params)
        result[:, j] = np.column_stack((v, w))
    return result


def fit(extracted: Path, bags: Path, train=TRAIN, test=TEST, continuous=("1027_manual_1",)):
    names = tuple(train) + tuple(test)
    if not train or not test or len(set(names)) != len(names):
        raise ValueError("Training and evaluation require separate, unique recording names")
    records = {name: recording(extracted, bags, name) for name in names}
    best = None
    for delay in range(5):
        parts = []
        # In the manual recording the outgoing ROS frame sometimes stays zero
        # while the car keeps moving. Use uninterrupted positive commands there;
        # learn stop response from the controlled steps, with equal weight per bag.
        for name in train:
            part = windows(records[name], delay, 3)
            if name in continuous:
                driving = np.all(part[0][:, :, 0] > 0.5, axis=1)
                part = tuple(a[driving] for a in part)
            if not len(part[0]):
                raise ValueError("No valid forward-driving training windows: "+name)
            parts.append(part)
        data = tuple(np.concatenate(col) for col in zip(*parts))
        sizes = [len(part[0]) for part in parts]
        bag_weight = np.concatenate([np.full(size, np.sqrt(np.mean(sizes)/size)) for size in sizes])
        def residual(params):
            return ((predict(data, params)[:, (2, 6, 11)] - data[2][:, (2, 6, 11)])
                    * bag_weight[:, None, None] / np.array([0.24, 0.12])).ravel()
        result = least_squares(residual, [1, 0.6, 0.85, 0.45, 0.25, 0.25, 0, 0.4],
                               bounds=([0.35, 0.08, 0.08, 0.08, 0.02, 0.02, -0.1, 0.08],
                                       [1.8, 3., 3., 3., 1.2, 1.2, 0.1, 3.]),
                               loss="soft_l1", max_nfev=70)
        if best is None or result.cost < best[0]:
            best = result.cost, delay, result.x
    _, delay, params = best
    metrics = {}
    for name in test:
        data = windows(records[name], delay, HORIZON)
        if not len(data[0]):
            raise ValueError("No valid forward-driving evaluation windows: "+name)
        # At 1s each group starts with the measured state. Persistence holds that state.
        truth = data[2][:, 9]
        predicted = predict(data, params)[:, 9]
        metrics[name] = {"windows": len(truth), "model_mae": np.abs(truth-predicted).mean(axis=0).tolist(),
                         "persistence_mae": np.abs(truth-data[1]).mean(axis=0).tolist()}
    return {"parameter_order": PARAMETERS, "parameters": params.tolist(), "delay_steps": delay,
            "dt_s": DT, "train": list(train), "heldout_1s_speed_yaw_mae": metrics,
            "source_sha256": {name: run[2] for name, run in records.items()}}
