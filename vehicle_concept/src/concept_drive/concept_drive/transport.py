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
