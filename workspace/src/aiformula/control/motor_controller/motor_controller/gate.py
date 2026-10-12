"""Legacy speed input by default; semantic owns its optional lifecycle and decision."""
import json
import math

ZERO = (0.0, 0.0)


def parse_object(text):
    def invalid(value):
        raise ValueError(value)
    data = json.loads(text, parse_constant=invalid)
    if not isinstance(data, dict):
        raise ValueError('expected JSON object')
    return data


def joystick_command(axes, buttons, settings):
    enable, turbo = settings['enable_button'], settings['enable_turbo_button']
    if len(buttons) <= max(enable, turbo):
        raise ValueError('incomplete buttons')
    fast = turbo >= 0 and buttons[turbo] != 0
    held = fast or (enable >= 0 and buttons[enable] != 0)
    if not held:
        return False, ZERO
    values = []
    for key in ('linear', 'angular'):
        axis = settings['axis_' + key]
        if axis < 0:
            values.append(0.0)
            continue
        if axis >= len(axes) or not math.isfinite(axes[axis]):
            raise ValueError('incomplete or invalid axes')
        scale = settings['scale_' + key + ('_turbo' if fast else '')]
        values.append(float(axes[axis]) * scale)
    return True, tuple(values)


class MotorGate:
    def __init__(self, command_timeout=0.5, semantic_timeout=0.6):
        self.command_timeout = command_timeout
        self.semantic_timeout = semantic_timeout
        self.auto = self.manual = None
        self.auto_time = self.manual_time = None
        self.manual_held = False
        self.auto_after = -math.inf
        self.semantic_active = False
        self.session = None
        self.session_sequence = -1
        self.session_time = None
        self.session_stamp = None
        self.retired_sessions = set()
        self.session_conflict = False
        self.status_time = self.status_source = None
        self.status_allowed = False
        self.semantic_reason = 'waiting_semantic_status'
        self.reason = 'waiting_command'

    def joy(self, held, command, now):
        if held != self.manual_held:
            self.auto = None
            self.auto_after = now
        self.manual_held = held
        self.manual = tuple(command) if held else None
        self.manual_time = now

    def command(self, values, now):
        if len(values) != 2 or not all(math.isfinite(v) for v in values):
            self.auto = None
            return
        self.auto, self.auto_time = tuple(values), now

    def session_message(self, payload, ros_ns, now):
        try:
            d = parse_object(payload)
            identity, sequence, stamp, active = d['session'], d['sequence'], d['stamp_ns'], d['active']
            if (not isinstance(identity, str) or not identity or len(identity) > 128
                    or type(sequence) is not int or sequence < 0 or type(stamp) is not int
                    or type(active) is not bool or not 0 <= (ros_ns-stamp)*1e-9 <= self.semantic_timeout):
                return
            if identity in self.retired_sessions:
                return
            if identity != self.session:
                if not active:
                    return
                if self.semantic_active and now-self.session_time <= self.semantic_timeout:
                    self.session_conflict = True
                    self.auto = None
                    return
                if self.session is not None:
                    self.retired_sessions.add(self.session)
                self.session = identity
                self.session_conflict = False
                self.session_sequence = -1
                self.status_time = self.status_source = None
                self.status_allowed = False
                self.auto = None
                self.auto_after = now
            if sequence <= self.session_sequence:
                return
            self.session_sequence, self.session_time, self.session_stamp = sequence, now, stamp
            self.semantic_active = active
            if not active:
                self.retired_sessions.add(identity)
                self.session_conflict = False
                self.auto = None
                self.auto_after = now
        except (ValueError, TypeError, KeyError):
            return

    def status(self, payload, ros_ns, now):
        # Merely running a diagnostic/shadow MapAssist cannot change legacy mode.
        if not self.semantic_active:
            return
        try:
            d = parse_object(payload)
            source = d['ros_time_sec']
            if (type(source) not in (int,float) or not math.isfinite(source)
                    or not -1e-6 <= (ros_ns-round(source*1e9))*1e-9 <= self.semantic_timeout
                    or (self.status_source is not None and source <= self.status_source)):
                return
            if d.get('traffic_policy') != 'stop_wait':
                raise ValueError('incompatible semantic policy')
            mode = d.get('traffic_mode')
            stop = d.get('traffic_stop_required')
            reason = d.get('traffic_reason')
            if (mode not in ('normal', 'wait') or type(stop) is not bool
                    or stop != (mode == 'wait') or not isinstance(reason, str)):
                raise ValueError('missing or inconsistent semantic traffic decision')
            # MapAssist owns the red latch and green release. Its diagnostic
            # command/ABC readiness is not the existing LYA's command path.
            self.status_allowed = not stop
            self.status_time, self.status_source = now, source
            self.semantic_reason = reason
            if not self.status_allowed:
                self.auto = None
                self.auto_after = now
        except (ValueError, TypeError, KeyError):
            self.status_allowed = False
            self.auto = None
            self.semantic_reason = 'invalid_semantic_status'

    def hold(self, reason, now):
        self.reason = reason
        self.auto = None
        self.auto_after = now
        return ZERO

    def output(self, ros_ns, now):
        if self.manual_held:
            self.auto = None
            self.auto_after = now
            if self.manual is None or now-self.manual_time > self.command_timeout:
                self.reason = 'manual_joy_timeout'
                return ZERO  # Lost enable never silently hands over to LYA.
            self.reason = 'manual_priority'
            return self.manual
        if self.semantic_active:
            if self.session_conflict:
                return self.hold('multiple_semantic_sessions', now)
            if (self.session_time is None or now-self.session_time > self.semantic_timeout
                    or not 0 <= (ros_ns-self.session_stamp)*1e-9 <= self.semantic_timeout):
                return self.hold('semantic_session_timeout', now)
            if (self.status_time is None or now-self.status_time > self.semantic_timeout
                    or not -1e-6 <= (ros_ns-round(self.status_source*1e9))*1e-9 <= self.semantic_timeout):
                return self.hold('semantic_status_timeout', now)
            if not self.status_allowed:
                return self.hold('semantic_hold:' + self.semantic_reason, now)
        if self.auto is None or self.auto_time <= self.auto_after or now-self.auto_time > self.command_timeout:
            return self.hold('command_timeout', now)
        self.reason = 'semantic_go' if self.semantic_active else 'legacy_passthrough'
        return self.auto
