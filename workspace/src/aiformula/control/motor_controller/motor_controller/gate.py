"""Motor-side arbitration. All elapsed-time limits use a monotonic clock."""
from collections import deque
import math
from semantic_drive_bridge.policy import SignalCapturePolicy, Rejected, json_object

ZERO = (0.0, 0.0)


class MotorGate:
    def __init__(self, frame, command_timeout=0.25, signal_timeout=0.5,
                 status_timeout=0.6, vote_window=0.5, vote_samples=3):
        self.capture = SignalCapturePolicy(frame, signal_timeout)
        self.command_timeout = command_timeout
        self.signal_timeout = signal_timeout
        self.status_timeout = status_timeout
        self.vote_window = vote_window
        self.vote_samples = vote_samples
        self.votes = deque()
        self.red_latched = False
        self.signal_state = None
        self.signal_source = None
        self.signal_received = None
        self.signal_age = None
        self.auto = self.manual = None
        self.auto_time = self.manual_time = None
        self.joy_time = None
        self.manual_held = False
        self.manual_after = self.auto_after = -math.inf
        self.status_time = None
        self.status_source = None
        self.status_age = None
        self.status_ok = False
        self.reason = "startup_hold"

    def joy(self, held, now):
        if held != self.manual_held:
            self.auto = self.manual = None
            self.auto_after = self.manual_after = now
        self.manual_held = held
        self.joy_time = now

    def command(self, values, manual, now):
        if (len(values) != 2 or not all(math.isfinite(v) for v in values)
                or abs(values[0]) > 4.0 or abs(values[1]) > 3.0):
            if manual:
                self.manual = None
            else:
                self.auto = None
            return
        if manual:
            self.manual, self.manual_time = tuple(values), now
        else:
            self.auto, self.auto_time = tuple(values), now

    def signal(self, payload, ros_ns, now):
        try:
            observation = self.capture.convert(payload, ros_ns, now)
        except Rejected:
            # Invalid/replayed observations never refresh validity or the vote.
            return
        self.signal_state = observation['state']
        self.signal_source = observation['sequence']
        self.signal_age = observation['age_sec']
        self.signal_received = now
        self.votes.append((now, self.signal_state))
        while self.votes and now - self.votes[0][0] > self.vote_window:
            self.votes.popleft()
        states = [state for _, state in self.votes]
        stable = None
        for state in ('red', 'green'):
            if states.count(state) >= self.vote_samples and states.count(state) > len(states) / 2:
                stable = state
        if stable == 'red':
            self.red_latched = True
        elif stable == 'green' and self.signal_state == 'green' and self.red_latched:
            self.red_latched = False
            self.auto = None
            self.auto_after = now  # Old cached LYA velocity cannot resume motion.

    def status(self, payload, ros_ns, now):
        try:
            data = json_object(payload)
            source = data['ros_time_sec']
            if type(source) not in (int, float) or not math.isfinite(source):
                return
            age = ros_ns * 1e-9 - source
            if not 0 <= age <= self.status_timeout:
                return
            if self.status_source is not None and source <= self.status_source:
                return
            odom_age = data.get('odom_age_sec')
            points = data.get('point_age_sec', {})
            if not isinstance(points, dict):
                raise ValueError('invalid point ages')
            fresh = lambda x: type(x) in (int, float) and math.isfinite(x) and 0 <= x <= 0.5
            self.status_ok = (data.get('traffic_policy') == 'stop_wait'
                              and fresh(odom_age) and data.get('recording_points_valid') is True
                              and all(fresh(points.get(k)) for k in 'abc'))
            self.status_source, self.status_time, self.status_age = source, now, age
            if not self.status_ok:
                self.auto = None
        except (Rejected, ValueError, KeyError, TypeError):
            self.status_ok = False
            self.auto = None

    def hold(self, reason, now):
        self.reason = reason
        self.auto = None
        self.auto_after = now
        return ZERO

    def output(self, ros_ns, now, motion_enabled, owners_ok=True, manual_owners_ok=True):
        if not motion_enabled:
            self.manual = None
            return self.hold('deployment_motion_inhibit', now)
        # A lost joystick while manually held must not fall through to automatic.
        if self.manual_held:
            if not manual_owners_ok:
                self.manual = None
                return self.hold('multiple_manual_publishers', now)
            self.auto = None
            self.auto_after = now
            if (self.joy_time is None or now-self.joy_time > self.command_timeout
                    or self.manual is None or self.manual_time <= self.manual_after
                    or now-self.manual_time > self.command_timeout):
                self.manual = None
                self.reason = 'manual_input_timeout'
                return ZERO
            self.reason = 'manual_priority'
            return self.manual
        if not owners_ok:
            return self.hold('multiple_command_publishers', now)
        if self.capture.poll(ros_ns, now)['clock_fault']:
            return self.hold('signal_clock_fault', now)
        if (self.signal_received is None or now-self.signal_received > self.signal_timeout
                or self.signal_age + now-self.signal_received > self.signal_timeout
                or ros_ns < self.signal_source
                or (ros_ns-self.signal_source)*1e-9 > self.signal_timeout):
            return self.hold('signal_timeout', now)
        if self.red_latched:
            return self.hold('red_wait_green', now)
        # Stop while confirming a newly seen red; latch after three real frames.
        if self.signal_state == 'red':
            return self.hold('red_confirming', now)
        if (not self.status_ok or self.status_time is None
                or now-self.status_time + self.status_age > self.status_timeout
                or ros_ns*1e-9 < self.status_source
                or ros_ns*1e-9-self.status_source > self.status_timeout):
            return self.hold('topo_or_lane_odom_timeout', now)
        if (self.auto is None or self.auto_time <= self.auto_after
                or now-self.auto_time > self.command_timeout):
            return self.hold('automatic_command_timeout', now)
        self.reason = 'automatic_go'
        return self.auto
