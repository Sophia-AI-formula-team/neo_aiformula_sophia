"""ROS-independent time, unit and sign conversions at the driver boundary."""
from collections import deque
import math


class Feedback:
    def __init__(self):
        self.samples = deque(maxlen=128)

    def add(self, sample):
        if not sample.get('valid', True):
            self.samples.clear()
            return
        if not all(math.isfinite(sample[k]) for k in ('t','leftRpm','rightRpm','gyro','wheelRadius')):
            raise ValueError('non-finite wheel feedback')
        if sample['wheelRadius'] <= 0 or not sample['source']:
            raise ValueError('invalid wheel radius or feedback source')
        if not self.samples or sample['t'] > self.samples[-1]['t']:
            self.samples.append(sample)

    def at(self, capture, max_age):
        sample = next((s for s in reversed(self.samples) if s['t'] <= capture), None)
        if sample is None or capture - sample['t'] > max_age:
            raise ValueError('no recent wheel and gyro measurement at image capture')
        return sample


class StartRamp:
    """Keep only recent, actually applied forward progress across a freshness gap."""

    def __init__(self):
        self.gate_at = -math.inf
        self.brake_source = -math.inf
        self.clear()

    def clear(self):
        self.speed = 0.
        self.applied_at = -math.inf
        self.waiting = False

    def brake(self, source):
        self.brake_source = max(source, self.brake_source)
        self.clear()

    def update(self, data):
        at = float(data['t'])
        if not math.isfinite(at) or at <= self.gate_at:
            return
        self.gate_at = at
        if (data.get('mode') != 'auto' or data.get('output_enabled') is not True or data.get('paused') is not False
                or data.get('manual_held') or data.get('estop')):
            self.clear()
            return
        if data.get('reason') == 'waiting_fresh_auto':
            self.waiting = True
            return
        self.waiting = False
        speed, age = float(data['linear_mps']), data.get('auto_capture_age_s')
        if (data.get('auto_fresh') is True and data.get('converter_ready') is True
                and math.isfinite(speed) and 0. < speed <= .5 and age is not None
                and 0. <= age <= .5 and at - age > self.brake_source + 1e-6):
            self.speed, self.applied_at = speed, at
        else:
            self.clear()

    def reference(self, now, wheel):
        moving_wheel_mps = max(abs(wheel['leftRpm']), abs(wheel['rightRpm'])) * math.tau * wheel['wheelRadius'] / 60
        if (self.waiting and moving_wheel_mps < .04 and 0. <= now - self.gate_at <= .15
                and 0. <= now - self.applied_at <= .35):
            return self.speed
        return 0.


def native_sensor(sample, relative_capture):
    return dict(t=relative_capture, leftRpm=sample['leftRpm'], rightRpm=sample['rightRpm'],
                gyro=-sample['gyro'], wheelRadius=sample['wheelRadius'], source=sample['source'])


def ros_command(native, *, physical=False):
    speed, steer = map(float, native)
    if not math.isfinite(speed) or not math.isfinite(steer):
        raise ValueError('non-finite planner command')
    # In the checked rollout a zero-drive action brakes: its residual steering
    # contributes no powered yaw. The real differential drive would pivot.
    yaw = 0. if physical and speed == 0. else -steer
    return dict(linear_x_mps=speed, angular_z_radps=yaw)


def stamp(ns):
    sec, nanosec = divmod(int(ns), 1_000_000_000)
    return dict(sec=sec, nanosec=nanosec)
