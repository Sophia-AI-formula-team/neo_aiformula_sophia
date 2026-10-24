"""Raw three-axis IMU + wheel odometry, with past-only attitude ownership.

The original planar odometer is unchanged. During stationary initialization
its raw-z integration remains available for diagnostics, but attitude=None is
NOT drive admission. Afterwards each gyro stores its own initialized tilt and
bias. A wheel selects exactly the same past source/receipt pair for both
heading integration and attitude; a newer IMU can never replace that pair.

One serialized caller owns this object. No INS orientation, GNSS, future
sample, moving acceleration correction or automatic origin reset is used.
"""
from collections import deque
from dataclasses import dataclass

from .attitude import CausalRawAttitude, StationaryEvidence
from .motion import CausalWheelGyroOdometry, MotionSample, _number, _stamp


@dataclass(frozen=True)
class ImuMotionSample(MotionSample):
    attitude: object = None


class CausalImuWheelOdometry(CausalWheelGyroOdometry):
    """The raw-IMU path; callers must hold output until sample.attitude exists.

Origin resets retain the attitude/bias and all sensor frontiers. Any rejected
IMU is terminal for this instance, including paired odometer admission. This
prevents continuing with only one half of a rejected sensor transaction.
"""
    def __init__(self, config=None, *, attitude_config=None):
        super().__init__(config)
        self.attitude_tracker = CausalRawAttitude(attitude_config)
        self._attitudes = deque(maxlen=self._gyro.maxlen)
        self._stationary_wheel = None
        self._imu_fault = None

    @property
    def ready(self):
        return self._imu_fault is None and self.attitude_tracker.ready

    @property
    def fault_reason(self):
        return self._imu_fault or self.attitude_tracker.fault_reason

    def accept_gyro(self, stamp_ns, receipt_ns, yaw_rate):
        raise ValueError("raw_three_axis_imu_required")

    def accept_imu(self, stamp_ns, receipt_ns, gyro_xyz, specific_force_xyz,
                   output_held, now_source_ns, now_steady_ns):
        if self.fault_reason is not None:
            raise ValueError("imu_motion_fault_latched: " + self.fault_reason)
        try:
            # Prevalidate every parent gyro admission before tracker commit.
            # The tracker owns the remaining clocks, raw vectors and tilt.
            stamp = _stamp(stamp_ns, "gyro_stamp")
            receipt = _stamp(receipt_ns, "gyro_receipt")
            if self.last_gyro_stamp_ns is not None and stamp <= self.last_gyro_stamp_ns:
                raise ValueError("gyro_stamp_not_increasing")
            if self._last_gyro_receipt_ns is not None and receipt < self._last_gyro_receipt_ns:
                raise ValueError("gyro_receipt_regressed")
            if type(output_held) is not bool:
                raise ValueError("invalid_output_held")
            try:
                gyro = tuple(gyro_xyz)
            except TypeError:
                raise ValueError("invalid_gyro_xyz")
            if len(gyro) != 3:
                raise ValueError("invalid_gyro_xyz")
            gyro = tuple(_number(value, "gyro_xyz") for value in gyro)

            def admit(candidate):
                rate = _number(candidate.heading_rate(), "heading_rate")
                if abs(rate) > self.config["max_yaw_rate_rps"]:
                    raise ValueError("yaw_rate_limit")

            if not self.attitude_tracker.ready and abs(gyro[2]) > self.config["max_yaw_rate_rps"]:
                raise ValueError("yaw_rate_limit")
            wheel = self._stationary_wheel
            evidence = (None if wheel is None else StationaryEvidence(
                wheel.stamp_ns, wheel.received_steady_ns, wheel.speed, output_held))
            attitude = self.attitude_tracker.accept_imu(
                stamp, receipt, gyro, specific_force_xyz, evidence,
                now_source_ns=now_source_ns, now_steady_ns=now_steady_ns,
                admission_check=admit)
            rate = gyro[2] if attitude is None else attitude.heading_rate()
            super().accept_gyro(stamp, receipt, rate)
            self._attitudes.append((stamp, receipt, attitude))
            return attitude
        except Exception as error:
            self._imu_fault = str(error)
            raise ValueError(self._imu_fault) from error

    def accept_wheels(self, stamp_ns, receipt_ns, left_rpm, right_rpm,
                      now_ns, now_steady_ns=None):
        if self.fault_reason is not None:
            raise ValueError("imu_motion_fault_latched: " + self.fault_reason)
        stamp = _stamp(stamp_ns, "wheel_stamp")
        receipt = _stamp(receipt_ns, "wheel_receipt")
        now = _stamp(now_ns, "now")
        steady = receipt if now_steady_ns is None else _stamp(now_steady_ns, "steady_now")
        past = [entry for entry in self._attitudes if entry[0] <= stamp and entry[1] <= receipt]
        attitude = None
        if past and past[-1][2] is not None:
            # Validate/propagate BEFORE parent pose publication. Snapshot owns
            # the selected original source, not tracker.last_sample.
            attitude = past[-1][2].propagate_to(stamp, wheel_receipt_ns=receipt,
                now_source_ns=now, now_steady_ns=steady)
        sample = super().accept_wheels(stamp, receipt, left_rpm, right_rpm, now, steady)
        paired = ImuMotionSample(sample.stamp_ns, sample.received_steady_ns,
            sample.x, sample.y, sample.yaw, sample.speed, sample.yaw_rate,
            sample.gyro_stamp_ns, attitude)
        self.last_sample = self._stationary_wheel = paired
        return paired

    def snapshot(self):
        result = super().snapshot()
        result.update(source="honda_can_rpm_and_past_raw_three_axis_imu",
                      raw_attitude=self.attitude_tracker.snapshot(),
                      startup_ready=self.ready, imu_fault_reason=self.fault_reason,
                      attitude_buffer_count=len(self._attitudes),
                      orientation_used=0, raw_tilt_used=int(self.ready))
        return result
