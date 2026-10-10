"""Past-only raw IMU tilt, independent of ROS, INS attitude and GNSS.

Initialization assumes a genuinely stationary sensor with calibrated mounting
and specific force pointing up. The numerical admission thresholds below are
engineering assumptions, NOT measured vehicle parameters. A quiet constant
rotation or horizontal accelerometer bias cannot be distinguished from gyro
bias or tilt by this initializer. Ready-state tilt uses gyro integration only;
accelerations are checked for valid format but never correct the attitude.

Source timestamps may explicitly mean driver receipt, not hardware sampling.
The caller must declare that provenance and supply real consumption clocks.
This module never changes timestamps or claims physical camera/IMU synchrony.
One serialized caller owns the estimator. Published samples are immutable.
"""
from collections import deque
from dataclasses import dataclass, replace
import math
from numbers import Integral, Real
from types import MappingProxyType

from .motion import DEFAULT_CONFIG as MOTION_DEFAULT_CONFIG


DEFAULT_CONFIG = {
    "initialization_duration_s": 1.0,
    "initialization_min_samples": 20,
    "initialization_max_samples": 512,
    "gravity_reference_mps2": 9.80665,
    "gravity_magnitude_tolerance_mps2": 0.5,
    "initialization_accel_max_deviation_mps2": 0.05,
    "initialization_gyro_max_norm_radps": 0.02,
    "initialization_gyro_max_deviation_radps": 0.002,
    "minimum_heading_denominator": 0.25,
}

_SOURCE_AGE_NS = int(MOTION_DEFAULT_CONFIG["max_gyro_age_s"] * 1e9)
_RECEIPT_AGE_NS = int(MOTION_DEFAULT_CONFIG["max_receipt_age_s"] * 1e9)
_WHEEL_AGE_NS = int(MOTION_DEFAULT_CONFIG["max_wheel_age_s"] * 1e9)
_MAX_DT_NS = int(MOTION_DEFAULT_CONFIG["max_dt_s"] * 1e9)
_STOPPED_SPEED = MOTION_DEFAULT_CONFIG["stopped_speed_mps"]


def _stamp(value, name):
    if isinstance(value, bool) or not isinstance(value, Integral) or not 0 < value < 2 ** 63:
        raise ValueError("invalid_" + name)
    return int(value)


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError("invalid_" + name)
    return float(value)


def _vector(value, name):
    try:
        result = tuple(value)
    except TypeError:
        raise ValueError("invalid_" + name)
    if len(result) != 3:
        raise ValueError("invalid_" + name)
    return tuple(_number(v, name) for v in result)


def _norm(vector):
    return math.sqrt(math.fsum(v * v for v in vector))


def _unit(vector):
    norm = _norm(vector)
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError("invalid_up_direction")
    return tuple(v / norm for v in vector)


def _advance(up, rate, dt):
    """Rodrigues: a fixed inertial up vector satisfies du_body/dt=-w x u."""
    magnitude = _norm(rate)
    if not math.isfinite(magnitude):
        raise ValueError("nonfinite_gyro_norm")
    if magnitude == 0 or dt == 0:
        return up
    axis = tuple(-v / magnitude for v in rate)
    angle = magnitude * dt
    if not math.isfinite(angle):
        raise ValueError("nonfinite_rotation_increment")
    sine, cosine = math.sin(angle), math.cos(angle)
    cross = (axis[1] * up[2] - axis[2] * up[1],
             axis[2] * up[0] - axis[0] * up[2],
             axis[0] * up[1] - axis[1] * up[0])
    dot = math.fsum(a * b for a, b in zip(axis, up))
    return _unit(tuple(cosine * up[i] + sine * cross[i] + (1.0 - cosine) * dot * axis[i]
                       for i in range(3)))


def _heading(up, rate, minimum):
    denominator = 1.0 - up[0] * up[0]
    if not math.isfinite(denominator) or denominator < minimum:
        raise ValueError("heading_projection_singular")
    result = (up[1] * rate[1] + up[2] * rate[2]) / denominator
    if not math.isfinite(result):
        raise ValueError("nonfinite_heading_rate")
    return result


def _fresh(source, receipt, now, steady):
    if not 0 <= now - source <= _SOURCE_AGE_NS:
        raise ValueError("attitude_source_stale_or_future")
    if not 0 <= steady - receipt <= _RECEIPT_AGE_NS:
        raise ValueError("attitude_receipt_stale_or_future")


@dataclass(frozen=True)
class StationaryEvidence:
    """Caller evidence, not an assertion that a real car is physically stopped.

    Wheel source/receipt must precede this IMU sample; output_held must describe
    the already-applied neutral/held output. This class is insufficient alone:
    the estimator also checks a sustained raw gyro/acceleration window.
    """
    wheel_stamp_ns: int
    wheel_receipt_ns: int
    speed_mps: float
    output_held: bool


@dataclass(frozen=True)
class InitializationEvidence:
    first_source_stamp_ns: int
    last_source_stamp_ns: int
    first_received_steady_ns: int
    last_received_steady_ns: int
    sample_count: int
    duration_s: float
    specific_force_mean: tuple
    gyro_mean: tuple
    gravity_magnitude_mps2: float
    accel_max_deviation_mps2: float
    gyro_max_norm_radps: float
    gyro_max_deviation_radps: float
    assumption: str = "stationary gravity and gyro bias; not physical stillness proof"


@dataclass(frozen=True)
class AttitudeSample:
    """The state AT the original IMU source frontier, never a relabeled forecast."""
    source_stamp_ns: int
    received_steady_ns: int
    up_body: tuple
    gyro_bias: tuple
    held_rate: tuple
    initialization: InitializationEvidence
    minimum_heading_denominator: float

    def heading_rate(self):
        return _heading(self.up_body, self.held_rate, self.minimum_heading_denominator)

    def propagate_to(self, wheel_stamp_ns, *, wheel_receipt_ns, now_source_ns, now_steady_ns):
        target = _stamp(wheel_stamp_ns, "wheel_stamp")
        receipt = _stamp(wheel_receipt_ns, "wheel_receipt")
        now = _stamp(now_source_ns, "now_source")
        steady = _stamp(now_steady_ns, "now_steady")
        _fresh(self.source_stamp_ns, self.received_steady_ns, now, steady)
        if not 0 <= target - self.source_stamp_ns <= _SOURCE_AGE_NS:
            raise ValueError("attitude_wheel_gap_or_future_source")
        if self.received_steady_ns > receipt:
            raise ValueError("attitude_had_not_arrived_at_wheel")
        if not 0 <= now - target <= _WHEEL_AGE_NS:
            raise ValueError("wheel_source_stale_or_future")
        if not 0 <= steady - receipt <= _RECEIPT_AGE_NS:
            raise ValueError("wheel_receipt_stale_or_future")
        up = _advance(self.up_body, self.held_rate, (target - self.source_stamp_ns) * 1e-9)
        _heading(up, self.held_rate, self.minimum_heading_denominator)
        return AttitudeSnapshot(self, target, receipt, up)


@dataclass(frozen=True)
class AttitudeSnapshot:
    """A bounded past-only wheel forecast retaining original IMU provenance."""
    source: AttitudeSample
    stamp_ns: int
    wheel_receipt_ns: int
    up_body: tuple

    @property
    def source_stamp_ns(self):
        return self.source.source_stamp_ns

    @property
    def received_steady_ns(self):
        return self.source.received_steady_ns

    @property
    def gyro_bias(self):
        return self.source.gyro_bias

    @property
    def held_rate(self):
        return self.source.held_rate

    @property
    def initialization(self):
        return self.source.initialization

    def heading_rate(self):
        return _heading(self.up_body, self.held_rate, self.source.minimum_heading_denominator)

    def propagate_to(self, wheel_stamp_ns, **kwargs):
        # Always restart from the ORIGINAL source. Chained forecasts cannot
        # renew freshness or accumulate another 150 ms of apparent validity.
        return self.source.propagate_to(wheel_stamp_ns, **kwargs)


class CausalRawAttitude:
    """Bounded initializer followed by held-rate, raw-only tilt propagation.

    An unqualified startup window returns None and clears only its candidate
    measurements, never source frontiers. Invalid clocks/format or ready-state
    propagation errors latch a terminal fault; a new object/session is needed.
    accept_imu never consumes a future rate for the previous time interval.
    """
    def __init__(self, config=None):
        if config is not None and not isinstance(config, dict):
            raise ValueError("configuration_must_be_dict")
        values = dict(DEFAULT_CONFIG)
        if config:
            if set(config) - set(values):
                raise ValueError("unknown_attitude_configuration")
            values.update(config)
        for key in ("initialization_min_samples", "initialization_max_samples"):
            v = values[key]
            if isinstance(v, bool) or not isinstance(v, Integral) or not 2 <= v <= 4096:
                raise ValueError("invalid_" + key)
            values[key] = int(v)
        if values["initialization_min_samples"] > values["initialization_max_samples"]:
            raise ValueError("initialization_sample_capacity")
        for key in set(values) - {"initialization_min_samples", "initialization_max_samples"}:
            values[key] = _number(values[key], key)
            if values[key] <= 0:
                raise ValueError("invalid_" + key)
        if values["minimum_heading_denominator"] > 1:
            raise ValueError("invalid_minimum_heading_denominator")
        self.config = MappingProxyType(values)
        self._window = deque(maxlen=values["initialization_max_samples"])
        self.last_source_stamp_ns = self.last_received_steady_ns = None
        self.last_sample = None
        self.fault_reason = None
        self.initialization_wait_reason = "no_samples"
        self.window_resets = self.window_evictions = 0

    @property
    def ready(self):
        return self.last_sample is not None and self.fault_reason is None

    def _clear_window(self, reason):
        self._window.clear()
        self.window_resets += 1
        self.initialization_wait_reason = reason

    def _stationary(self, evidence, stamp, receipt, now, steady):
        if evidence is None:
            return False
        if not isinstance(evidence, StationaryEvidence) or type(evidence.output_held) is not bool:
            raise ValueError("invalid_stationary_evidence")
        ws = _stamp(evidence.wheel_stamp_ns, "stationary_wheel_stamp")
        wr = _stamp(evidence.wheel_receipt_ns, "stationary_wheel_receipt")
        speed = _number(evidence.speed_mps, "stationary_speed")
        return (evidence.output_held and abs(speed) <= _STOPPED_SPEED
                and ws <= stamp and wr <= receipt
                and 0 <= now - ws <= _WHEEL_AGE_NS
                and 0 <= steady - wr <= _RECEIPT_AGE_NS)

    def _initialize(self, stamp, receipt, gyro, force, evidence, now, steady):
        cfg = self.config
        if not self._stationary(evidence, stamp, receipt, now, steady):
            self._clear_window("stationary_evidence_not_qualified")
            return None
        if _norm(gyro) > cfg["initialization_gyro_max_norm_radps"]:
            self._clear_window("initialization_gyro_not_stationary")
            return None
        if len(self._window) == self._window.maxlen:
            self.window_evictions += 1
        self._window.append((stamp, receipt, gyro, force))
        count = len(self._window)
        mean_g = tuple(math.fsum(row[2][i] for row in self._window) / count for i in range(3))
        mean_f = tuple(math.fsum(row[3][i] for row in self._window) / count for i in range(3))
        gyro_spread = max(_norm(tuple(row[2][i] - mean_g[i] for i in range(3))) for row in self._window)
        accel_spread = max(_norm(tuple(row[3][i] - mean_f[i] for i in range(3))) for row in self._window)
        gravity = _norm(mean_f)
        if abs(gravity - cfg["gravity_reference_mps2"]) > cfg["gravity_magnitude_tolerance_mps2"]:
            self._clear_window("initialization_gravity_magnitude")
            return None
        if accel_spread > cfg["initialization_accel_max_deviation_mps2"]:
            self._clear_window("initialization_accel_unstable")
            return None
        if gyro_spread > cfg["initialization_gyro_max_deviation_radps"]:
            self._clear_window("initialization_gyro_unstable")
            return None
        duration = (stamp - self._window[0][0]) * 1e-9
        if count < cfg["initialization_min_samples"] or duration < cfg["initialization_duration_s"]:
            self.initialization_wait_reason = "initialization_window_incomplete"
            return None
        up = _unit(mean_f)
        held = tuple(gyro[i] - mean_g[i] for i in range(3))
        _heading(up, held, cfg["minimum_heading_denominator"])
        initialization = InitializationEvidence(
            self._window[0][0], stamp, self._window[0][1], receipt, count, duration,
            mean_f, mean_g, gravity, accel_spread,
            max(_norm(row[2]) for row in self._window), gyro_spread)
        result = AttitudeSample(stamp, receipt, up, mean_g, held, initialization,
                                cfg["minimum_heading_denominator"])
        self._window.clear()
        self.initialization_wait_reason = None
        return result

    def accept_imu(self, source_ns, receipt_ns, gyro_xyz, specific_force_xyz,
                   stationary_evidence, *, now_source_ns, now_steady_ns, admission_check=None):
        """Return a committed sample or None while waiting for initialization.

        Optional admission_check(sample) is a PURE validator, called before
        source/sample commit. False or an exception latches a fault. It must
        not modify a second estimator: the caller serializes both owners and
        validates their admissions before publishing the paired state.
        """
        if self.fault_reason is not None:
            raise ValueError("attitude_fault_latched: " + self.fault_reason)
        try:
            stamp = _stamp(source_ns, "imu_source")
            receipt = _stamp(receipt_ns, "imu_receipt")
            now = _stamp(now_source_ns, "now_source")
            steady = _stamp(now_steady_ns, "now_steady")
            _fresh(stamp, receipt, now, steady)
            if self.last_source_stamp_ns is not None:
                if stamp <= self.last_source_stamp_ns:
                    raise ValueError("imu_source_not_increasing")
                if stamp - self.last_source_stamp_ns > _MAX_DT_NS:
                    raise ValueError("imu_source_gap")
                if receipt < self.last_received_steady_ns:
                    raise ValueError("imu_receipt_regressed")
            gyro = _vector(gyro_xyz, "gyro_xyz")
            force = _vector(specific_force_xyz, "specific_force_xyz")
            previous = self.last_sample
            if previous is None:
                candidate = self._initialize(stamp, receipt, gyro, force,
                                             stationary_evidence, now, steady)
            else:
                up = _advance(previous.up_body, previous.held_rate,
                              (stamp - previous.source_stamp_ns) * 1e-9)
                held = tuple(gyro[i] - previous.gyro_bias[i] for i in range(3))
                _heading(up, held, previous.minimum_heading_denominator)
                candidate = replace(previous, source_stamp_ns=stamp,
                                    received_steady_ns=receipt, up_body=up, held_rate=held)
            if candidate is not None and admission_check is not None:
                if admission_check(candidate) is False:
                    raise ValueError("attitude_external_admission_rejected")
            # Both published state and source frontiers advance only after all
            # validation. Candidate-window rejection is not a sensor reset.
            self.last_source_stamp_ns, self.last_received_steady_ns = stamp, receipt
            if candidate is not None:
                self.last_sample = candidate
            return candidate
        except Exception as error:
            self.fault_reason = str(error)
            self._window.clear()
            raise ValueError(self.fault_reason) from error

    def snapshot(self):
        return dict(ready=self.ready, fault_reason=self.fault_reason,
                    last_source_stamp_ns=self.last_source_stamp_ns,
                    last_received_steady_ns=self.last_received_steady_ns,
                    initialization_wait_reason=self.initialization_wait_reason,
                    retained_initialization_samples=len(self._window),
                    window_resets=self.window_resets, window_evictions=self.window_evictions,
                    dynamic_acceleration_used=False, ins_orientation_used=False, gnss_used=False)
