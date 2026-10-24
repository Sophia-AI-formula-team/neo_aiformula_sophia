"""ROS-independent, past-only wheel + raw-rate planar odometry.

No orientation, position, INS velocity or GNSS is an input. Source timestamps
are ROS nanoseconds; receipt timestamps are monotonic nanoseconds. Callbacks
must pass their original receipt times, never relabel an old measurement.
"""
from collections import deque
from dataclasses import dataclass
import math
from numbers import Integral, Real
from types import MappingProxyType


DEFAULT_CONFIG = {
    "wheel_diameter_m": 0.254,
    "gyro_buffer_size": 256,
    "max_dt_s": 0.2,
    "max_gyro_age_s": 0.15,
    "max_wheel_age_s": 0.2,
    "max_receipt_age_s": 0.2,
    "max_speed_mps": 3.0,
    "max_yaw_rate_rps": 2.5,
    "max_acceleration_mps2": 5.0,
    "stopped_speed_mps": 0.05,
}


def _stamp(value, name):
    if isinstance(value, bool) or not isinstance(value, Integral) or not 0 < value < 2 ** 63:
        raise ValueError("invalid_" + name)
    return int(value)


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError("invalid_" + name)
    return float(value)


def _fresh(stamp, receipt, now, steady, source_age_s, receipt_age_s, name):
    if not 0 <= now - stamp <= int(source_age_s * 1e9):
        raise ValueError(name + "_source_stale_or_future")
    if not 0 <= steady - receipt <= int(receipt_age_s * 1e9):
        raise ValueError(name + "_receipt_stale_or_future")


def decode_honda_rpm(frame_id, data, dlc=8, is_rtr=False, is_extended=False, is_error=False):
    """Decode only the audited forward-only Honda 0x711 low-byte protocol.

    Return (left_rpm, right_rpm). Unknown IDs/flags, nonzero high bytes and
    reverse/signed values are rejected, not guessed or silently truncated.
    """
    if isinstance(frame_id, bool) or not isinstance(frame_id, Integral) or frame_id != 1809:
        raise ValueError("unsupported_can_id")
    if (any(type(flag) is not bool for flag in (is_rtr, is_extended, is_error))
            or is_rtr or is_extended or is_error):
        raise ValueError("unsupported_can_flags")
    if isinstance(dlc, bool) or not isinstance(dlc, Integral) or dlc != 8:
        raise ValueError("unsupported_can_dlc")
    try:
        values = tuple(data)
    except TypeError:
        raise ValueError("invalid_can_data")
    if (len(values) != 8 or any(isinstance(v, bool) or not isinstance(v, Integral)
                               or not 0 <= v <= 255 for v in values)):
        raise ValueError("invalid_can_data")
    if any(values[index] != 0 for index in (1, 2, 3, 5, 6, 7)):
        raise ValueError("unsupported_can_high_bytes_or_direction")
    return int(values[4]), int(values[0])


@dataclass(frozen=True)
class MotionSample:
    stamp_ns: int
    received_steady_ns: int
    x: float
    y: float
    yaw: float
    speed: float
    yaw_rate: float
    gyro_stamp_ns: int


class CausalWheelGyroOdometry:
    """Bounded planar dead reckoning with zero-order-held past gyro samples.

    ``accept_gyro(stamp_ns, receipt_ns, yaw_rate)`` stores a physical body-z
    angular rate, already transformed by the caller from its sensor frame.
    ``accept_wheels(..., now_ns, now_steady_ns)`` validates both clocks, takes
    only gyro data with source <= wheel source AND receipt <= wheel receipt,
    and returns an immutable sample. It never interpolates from future gyro.

    A time gap/restart is not repaired by clamping dt. The caller must HOLD.
    ``reset_origin(stopped=True)`` is an explicit stationary START operation;
    it preserves all source/receipt frontiers and gyro timestamps. No reset
    method is called automatically on a bad frame or a publisher restart.
    """

    def __init__(self, config=None):
        if config is not None and not isinstance(config, dict):
            raise ValueError("configuration_must_be_dict")
        cfg = dict(DEFAULT_CONFIG)
        if config:
            unknown = set(config) - set(cfg)
            if unknown:
                raise ValueError("unknown_configuration: " + ",".join(sorted(unknown)))
            cfg.update(config)
        size = cfg["gyro_buffer_size"]
        if isinstance(size, bool) or not isinstance(size, Integral) or not 2 <= size <= 4096:
            raise ValueError("invalid_gyro_buffer_size")
        for key in set(cfg) - {"gyro_buffer_size"}:
            cfg[key] = _number(cfg[key], key)
            if cfg[key] <= 0:
                raise ValueError("invalid_" + key)
        if (cfg["max_dt_s"] > 0.2 or cfg["max_gyro_age_s"] > 0.5
                or cfg["max_wheel_age_s"] > 0.5 or cfg["max_receipt_age_s"] > 0.5
                or not 0.05 <= cfg["wheel_diameter_m"] <= 1.0
                or cfg["max_speed_mps"] > 10 or cfg["max_yaw_rate_rps"] > 10
                or cfg["max_acceleration_mps2"] > 20
                or cfg["stopped_speed_mps"] >= cfg["max_speed_mps"]):
            raise ValueError("unsafe_configuration_range")
        self.config = MappingProxyType(cfg)
        self._gyro = deque(maxlen=int(size))
        self.last_gyro_stamp_ns = None
        self._last_gyro_receipt_ns = None
        self.last_wheel_stamp_ns = None
        self._last_wheel_receipt_ns = None
        self.last_sample = None
        self.reset_count = 0

    def accept_gyro(self, stamp_ns, receipt_ns, yaw_rate):
        stamp = _stamp(stamp_ns, "gyro_stamp")
        receipt = _stamp(receipt_ns, "gyro_receipt")
        rate = _number(yaw_rate, "yaw_rate")
        if abs(rate) > self.config["max_yaw_rate_rps"]:
            raise ValueError("yaw_rate_limit")
        if self.last_gyro_stamp_ns is not None and stamp <= self.last_gyro_stamp_ns:
            raise ValueError("gyro_stamp_not_increasing")
        if self._last_gyro_receipt_ns is not None and receipt < self._last_gyro_receipt_ns:
            raise ValueError("gyro_receipt_regressed")
        self._gyro.append((stamp, receipt, rate))
        self.last_gyro_stamp_ns, self._last_gyro_receipt_ns = stamp, receipt

    def accept_wheels(self, stamp_ns, receipt_ns, left_rpm, right_rpm, now_ns, now_steady_ns=None):
        stamp = _stamp(stamp_ns, "wheel_stamp")
        receipt = _stamp(receipt_ns, "wheel_receipt")
        now = _stamp(now_ns, "now")
        # In an immediate subscription callback the receipt is current steady
        # time. Queued/worker consumers MUST provide the actual later now.
        steady = receipt if now_steady_ns is None else _stamp(now_steady_ns, "steady_now")
        cfg = self.config
        _fresh(stamp, receipt, now, steady, cfg["max_wheel_age_s"], cfg["max_receipt_age_s"], "wheel")
        if self.last_wheel_stamp_ns is not None and stamp <= self.last_wheel_stamp_ns:
            raise ValueError("wheel_stamp_not_increasing")
        if self._last_wheel_receipt_ns is not None and receipt < self._last_wheel_receipt_ns:
            raise ValueError("wheel_receipt_regressed")
        left = _number(left_rpm, "left_rpm")
        right = _number(right_rpm, "right_rpm")
        if not (0 <= left <= 255 and 0 <= right <= 255):
            raise ValueError("unsupported_rpm_or_reverse")
        speed = (left + right) * 0.5 * math.pi * cfg["wheel_diameter_m"] / 60.0
        if speed > cfg["max_speed_mps"]:
            raise ValueError("speed_limit")
        past = [sample for sample in self._gyro if sample[0] <= stamp and sample[1] <= receipt]
        if not past:
            raise ValueError("no_past_gyro")
        gyro_stamp, gyro_receipt, rate = past[-1]
        _fresh(gyro_stamp, gyro_receipt, now, steady,
               cfg["max_gyro_age_s"], cfg["max_receipt_age_s"], "gyro")
        if stamp - gyro_stamp > int(cfg["max_gyro_age_s"] * 1e9):
            raise ValueError("gyro_wheel_gap")
        previous = self.last_sample
        if previous is None:
            x, y, yaw = 0.0, 0.0, 0.0
        else:
            dt = (stamp - previous.stamp_ns) * 1e-9
            if not 0 < dt <= cfg["max_dt_s"]:
                raise ValueError("wheel_dt_gap_or_restart")
            if abs(speed - previous.speed) > cfg["max_acceleration_mps2"] * dt:
                raise ValueError("wheel_acceleration_limit")
            # All data here has arrived before this wheel and precedes its
            # source frontier. Hold the prior accepted rate until the first
            # newer gyro, then each newer rate until the next source timestamp.
            yaw, x, y = previous.yaw, previous.x, previous.y
            cursor, held_rate = previous.stamp_ns, previous.yaw_rate
            segments = [(g[0], g[2]) for g in past if g[0] > cursor]
            segments.append((stamp, rate))
            for boundary, next_rate in segments:
                delta = (boundary - cursor) * 1e-9
                angle = held_rate * delta
                distance = speed * delta
                half = 0.5 * angle
                sinc = math.sin(half) / half if abs(half) > 1e-12 else 1.0
                x += distance * sinc * math.cos(yaw + half)
                y += distance * sinc * math.sin(yaw + half)
                yaw += angle
                cursor, held_rate = boundary, next_rate
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("nonfinite_integrated_pose")
        sample = MotionSample(stamp, receipt, x, y, yaw, speed, rate, gyro_stamp)
        self.last_sample = sample
        self.last_wheel_stamp_ns, self._last_wheel_receipt_ns = stamp, receipt
        return sample

    def reset_origin(self, *, stopped):
        if stopped is not True or (self.last_sample is not None
                                  and self.last_sample.speed > self.config["stopped_speed_mps"]):
            raise ValueError("origin_reset_requires_stopped_start")
        self.last_sample = None
        self.reset_count += 1

    def snapshot(self):
        return {"gyro_buffer_count": len(self._gyro), "reset_count": self.reset_count,
                "last_gyro_stamp_ns": self.last_gyro_stamp_ns,
                "last_wheel_stamp_ns": self.last_wheel_stamp_ns,
                "gnss_used": 0, "orientation_used": 0,
                "source": "honda_can_rpm_and_past_raw_body_gyro"}
