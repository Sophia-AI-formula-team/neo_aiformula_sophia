"""Causal, forward-only wheel/gyro samples for the archived Honda interface."""
from collections import deque
import math

from numbers import Integral


def decode_honda_rpm(frame_id, data, dlc=8, is_rtr=False, is_extended=False, is_error=False):
    """Match Neo motion.decode_honda_rpm's audited forward-only 0x711 format."""
    if type(frame_id) is bool or not isinstance(frame_id, Integral) or frame_id != 0x711:
        raise ValueError('unsupported_can_id')
    if any(type(flag) is not bool or flag for flag in (is_rtr, is_extended, is_error)):
        raise ValueError('unsupported_can_flags')
    if type(dlc) is bool or not isinstance(dlc, Integral) or dlc != 8:
        raise ValueError('unsupported_can_dlc')
    try:
        values = tuple(data)
    except TypeError as exc:
        raise ValueError('invalid_can_data') from exc
    if len(values) != 8 or any(type(v) is bool or not isinstance(v, Integral) or not 0 <= v <= 255 for v in values):
        raise ValueError('invalid_can_data')
    if any(values[i] for i in (1, 2, 3, 5, 6, 7)):
        raise ValueError('unsupported_can_high_bytes_or_direction')
    return int(values[4]), int(values[0])

MAX_WHEEL_AGE = .2
MAX_GYRO_AGE = .15


class VehicleState:
    def __init__(self):
        self.gyros = deque(maxlen=256)
        self.wheel = None
        self.reason = 'awaiting_wheels'
        self.last_wheel = -math.inf

    def gyro(self, value, source, stamp, receipt):
        if (not math.isfinite(value) or not 0 < source <= stamp
                or stamp-source > MAX_GYRO_AGE):
            self.gyros.clear()
            return
        if not self.gyros or source > self.gyros[-1][0]:
            self.gyros.append((source, receipt, value))

    def wheels(self, data, source, stamp, receipt, **flags):
        if flags.get('frame_id') != 0x711:
            return
        self.wheel = None
        if not 0 < source <= stamp or stamp-source > MAX_WHEEL_AGE or source <= self.last_wheel:
            self.reason = 'invalid_wheel_time'
            return
        self.last_wheel = source
        try:
            left, right = decode_honda_rpm(data=data, **flags)
        except ValueError as error:
            self.reason = str(error)
            return
        self.wheel = source, receipt, left, right

    def sample(self, stamp, now):
        record = dict(t=None, leftRpm=None, rightRpm=None, gyro=None, wheelRadius=.127,
                      valid=False, reason=self.reason, source='can_0x711_low_byte_vectornav_ros_z',
                      rpm_encoding='forward_only_uint8', gyro_convention='ROS angular_velocity.z')
        if self.wheel is None:
            return record
        t, receipt, left, right = self.wheel
        if not (0 <= stamp-t <= MAX_WHEEL_AGE and 0 <= now-receipt <= MAX_WHEEL_AGE):
            return dict(record, reason='wheel_timeout')
        gyro = next((g for g in reversed(self.gyros) if g[0] <= t), None)
        if gyro is None or t-gyro[0] > MAX_GYRO_AGE or not 0 <= now-gyro[1] <= MAX_GYRO_AGE:
            return dict(record, reason='gyro_timeout')
        return dict(record, t=t, leftRpm=left, rightRpm=right, gyro=gyro[2], gyro_t=gyro[0],
                    valid=True, reason='ok')
