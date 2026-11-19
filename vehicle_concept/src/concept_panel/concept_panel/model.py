"""UI state independent of Qt and ROS."""
from collections import deque
import math


def stamp_seconds(value):
    if isinstance(value, dict):
        value = value['sec'] + value['nanosec'] * 1e-9
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError('无有效时间戳')
    return value


class Feed:
    def __init__(self):
        self.value = {}
        self.received = self.stamp = None

    def put(self, value, stamp, received):
        self.value, self.stamp, self.received = value, stamp, received

    def age(self, ros, steady):
        if self.received is None or self.stamp is None:
            return None
        if ros < self.stamp or steady < self.received:
            return None
        return max(steady-self.received, ros-self.stamp)


class Journal:
    def __init__(self):
        self.rows = deque(maxlen=120)
        self.last_by_channel = {}

    def add(self, channel, message, wall):
        previous = self.last_by_channel.get(channel)
        if previous and previous['message'] == message and previous in self.rows:
            previous['last'] = wall
            previous['count'] += 1
        else:
            event = dict(channel=channel, message=message, first=wall, last=wall, count=1)
            self.rows.appendleft(event)
            self.last_by_channel[channel] = event


class Action:
    """Advance through control services only after successful acknowledgements."""
    def __init__(self, command, value=None, image_gap=False):
        if command == 'resume' and image_gap:
            steps = ['pause', 'restart', 'resume']
        elif command == 'restart':
            steps = ['pause', 'restart']
        else:
            steps = [command]
        self.steps = deque(steps)
        self.value = value

    def completed(self, ok):
        self.steps.popleft()
        if not ok:
            self.steps.clear()
        return self.steps[0] if self.steps else None
