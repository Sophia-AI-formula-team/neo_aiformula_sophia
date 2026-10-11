"""Pure validation policies shared by the real rclpy entry points.

ROS timestamps and steady receipt time are distinct. No model, simulated ROS
module, cached color substitution, or controller implementation lives here.
"""
import json
import math


ZERO = (0.0,) * 6


class Rejected(ValueError):
    """An input was rejected; it must not refresh an observation or command."""


def finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Rejected(name + " must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise Rejected(name + " must be a finite number")
    if not math.isfinite(result):
        raise Rejected(name + " must be a finite number")
    return result


def positive_parameter(value, name):
    result = finite_number(value, name)
    if result <= 0:
        raise Rejected(name + " must be positive")
    return result


def _invalid_constant(value):
    raise Rejected("nonfinite JSON constant: " + value)


def json_object(payload):
    if not isinstance(payload, str):
        raise Rejected("payload must be JSON text")
    try:
        data = json.loads(payload, parse_constant=_invalid_constant)
    except (ValueError, TypeError, RecursionError) as error:
        raise Rejected("invalid JSON: " + str(error))
    if not isinstance(data, dict):
        raise Rejected("payload must be a JSON object")
    return data


def capture_stamp_ns(data, expected_frame_id):
    header = data.get("header")
    if not isinstance(header, dict) or header.get("frame_id") != expected_frame_id:
        raise Rejected("unexpected_frame_id")
    stamp = header.get("stamp")
    if not isinstance(stamp, dict):
        raise Rejected("missing_source_stamp")
    sec, nanosec = stamp.get("sec"), stamp.get("nanosec")
    if (type(sec) is not int or type(nanosec) is not int or sec < 0
            or sec > 2147483647 or not 0 <= nanosec < 1000000000):
        raise Rejected("invalid_source_stamp")
    source_ns = sec * 1000000000 + nanosec
    if source_ns <= 0:
        raise Rejected("zero_source_stamp")
    if type(data.get("sequence")) is not int or data["sequence"] != source_ns:
        raise Rejected("sequence_must_equal_source_nanoseconds")
    return source_ns


class ClockGuard:
    """A clock rollback is latched until this process/chain is restarted."""

    def __init__(self):
        self.last_ros_ns = None
        self.last_steady = None
        self.fault = None

    def observe(self, ros_ns, steady):
        if self.fault:
            raise Rejected(self.fault)
        if type(ros_ns) is not int or ros_ns < 0:
            self.fault = "invalid_ros_clock_restart_required"
        else:
            try:
                steady = finite_number(steady, "steady_time")
            except Rejected:
                self.fault = "invalid_steady_clock_restart_required"
            if self.fault is None and self.last_ros_ns is not None and ros_ns < self.last_ros_ns:
                self.fault = "ros_clock_rollback_restart_required"
            if (self.fault is None and self.last_steady is not None
                    and steady < self.last_steady):
                self.fault = "steady_clock_rollback_restart_required"
        if self.fault:
            raise Rejected(self.fault)
        self.last_ros_ns, self.last_steady = ros_ns, steady


class SignalCapturePolicy:
    def __init__(self, expected_frame_id, max_age_sec=0.5):
        if (not isinstance(expected_frame_id, str) or not expected_frame_id
                or expected_frame_id.strip() != expected_frame_id):
            raise Rejected("expected_frame_id must be explicitly configured")
        self.expected_frame_id = expected_frame_id
        self.max_age_sec = positive_parameter(max_age_sec, "max_age_sec")
        self.clock = ClockGuard()
        self.last_source_ns = 0
        self.last_accepted_source_ns = 0
        self.last_received_steady = None
        self.reason = "waiting_source_observation"

    def convert(self, payload, ros_ns, steady):
        """Return the existing core's atomic schema or raise without publishing.

        A well-formed source identity is consumed before validating its age and
        payload. Retrying the same rejected capture cannot turn it into a vote.
        A bad future identity can therefore require a full chain restart.
        """
        try:
            self.clock.observe(ros_ns, steady)
            data = json_object(payload)
            source_ns = capture_stamp_ns(data, self.expected_frame_id)
            if source_ns <= self.last_source_ns:
                raise Rejected("duplicate_or_rolled_back_source_stamp")
            self.last_source_ns = source_ns
            if ros_ns <= 0:
                raise Rejected("ros_clock_not_started")
            age_sec = (ros_ns - source_ns) * 1e-9
            if age_sec < 0:
                raise Rejected("future_source_stamp")
            if age_sec > self.max_age_sec:
                raise Rejected("stale_source_stamp")
            state = data.get("state")
            if state not in ("red", "green", "unknown"):
                raise Rejected("invalid_signal_state")
            if "distance_m" not in data or "uncertainty_m" not in data:
                raise Rejected("missing_range_fields")
            # None is meaningful: a real unknown has no distance; a real red
            # without distance reaches the core's existing structural-fault gate.
            distance = data.get("distance_m")
            if distance is not None:
                distance = positive_parameter(distance, "distance_m")
                uncertainty = finite_number(data["uncertainty_m"], "uncertainty_m")
                if uncertainty < 0:
                    raise Rejected("uncertainty_m must be nonnegative")
            else:
                # Match the core's missing-range semantics. In particular the
                # real RGB range helper returns uncertainty=None when invalid.
                # Keep the current color and normalize unused uncertainty;
                # never reject an actual unknown just because range is absent.
                uncertainty = 0.0
            result = dict(sequence=source_ns, state=state, age_sec=age_sec,
                          distance_m=distance, uncertainty_m=uncertainty)
        except Rejected as error:
            self.reason = str(error)
            raise
        self.last_accepted_source_ns = source_ns
        self.last_received_steady = steady
        self.reason = "accepted_current_capture"
        return result

    def poll(self, ros_ns, steady):
        try:
            self.clock.observe(ros_ns, steady)
        except Rejected as error:
            self.reason = str(error)
        if (self.clock.fault is None and self.last_received_steady is not None
                and steady - self.last_received_steady > self.max_age_sec):
            self.reason = "capture_stream_timeout_abstaining"
        return dict(reason=self.reason, clock_fault=self.clock.fault,
                    last_seen_sequence=self.last_source_ns,
                    last_accepted_sequence=self.last_accepted_source_ns,
                    publishing_cached_observations=False)


def validate_shadow_topic(topic):
    # Apply to both configured and rclpy-resolved publisher names: CLI remaps
    # must not bypass the shadow-only contract of this executable.
    if (not isinstance(topic, str) or not topic.startswith("/semantic_drive/shadow_")
            or "//" in topic or topic.endswith("/")):
        raise Rejected("output must remain under /semantic_drive/shadow_*")
    return topic


def parse_map_status(payload):
    """Read fields emitted by core63 MapAssist._publish_status, not a fake schema."""
    data = json_object(payload)
    reason = data.get("velocity_gate_reason")
    mode = data.get("traffic_mode")
    oa = data.get("avoidance_active")
    path_allowed = data.get("path_output_allowed")
    if not isinstance(reason, str) or not reason:
        raise Rejected("missing_velocity_gate_reason")
    if mode not in ("normal", "approach", "wait", "perception_suspended", "detour"):
        raise Rejected("invalid_traffic_mode")
    if oa is not None and type(oa) is not bool:
        raise Rejected("invalid_avoidance_active")
    if type(path_allowed) is not bool:
        raise Rejected("invalid_path_output_allowed")
    source_sec = finite_number(data.get("ros_time_sec"), "status.ros_time_sec")
    if source_sec <= 0:
        raise Rejected("invalid_status_ros_time")
    forwarding = reason in ("forwarding_controller", "signal_approach_speed_limit")
    owned_path = (oa is True or
                  (oa is False and path_allowed and mode in ("normal", "approach", "detour")))
    return dict(allowed=forwarding and owned_path, reason=reason,
                ros_time_sec=source_sec, traffic_mode=mode, avoidance_active=oa)


class CommandWatchdogPolicy:
    def __init__(self, timeout_sec=0.25):
        self.timeout_sec = positive_parameter(timeout_sec, "timeout_sec")
        self.clock = ClockGuard()
        self.command = None
        self.command_time = None
        self.status = None
        self.status_time = None
        self.last_status_ros_sec = None
        self.reason = "waiting_command_and_status"

    def _observe(self, ros_ns, steady):
        try:
            self.clock.observe(ros_ns, steady)
            if ros_ns == 0:
                raise Rejected("ros_clock_not_started")
        except Rejected as error:
            self.command = None
            self.reason = str(error)
            raise

    def receive_status(self, payload, ros_ns, steady):
        try:
            self._observe(ros_ns, steady)
            status = parse_map_status(payload)
            age = ros_ns * 1e-9 - status["ros_time_sec"]
            if age < 0 or age > self.timeout_sec:
                raise Rejected("future_or_stale_map_status")
            if (self.last_status_ros_sec is not None
                    and status["ros_time_sec"] <= self.last_status_ros_sec):
                raise Rejected("duplicate_or_out_of_order_map_status")
            self.last_status_ros_sec = status["ros_time_sec"]
            self.status, self.status_time = status, steady
            if not status["allowed"]:
                self.command = None
                self.reason = "map_status_hold:" + status["reason"]
        except Rejected as error:
            self.status = None
            self.command = None
            self.reason = str(error)
            raise

    def receive_command(self, values, ros_ns, steady):
        try:
            self._observe(ros_ns, steady)
            if not isinstance(values, (list, tuple)) or len(values) != 6:
                raise Rejected("command_requires_six_components")
            command = tuple(finite_number(value, "command") for value in values)
            if (self.status is None or not self.status["allowed"] or
                    steady - self.status_time > self.timeout_sec):
                raise Rejected("command_without_fresh_permitting_status")
            self.command, self.command_time = command, steady
            self.reason = "fresh_shadow_command"
        except Rejected as error:
            self.command = None
            self.reason = str(error)
            raise

    def output(self, ros_ns, steady):
        try:
            self._observe(ros_ns, steady)
        except Rejected:
            return ZERO
        if self.status is None or not self.status["allowed"]:
            self.command = None
            self.reason = "map_status_missing_or_hold"
        elif steady - self.status_time > self.timeout_sec:
            self.command = None
            self.reason = "map_status_receipt_timeout"
        elif self.command is None:
            self.reason = "waiting_new_command"
        elif steady - self.command_time > self.timeout_sec:
            self.command = None
            self.reason = "command_receipt_timeout"
        else:
            self.reason = "fresh_shadow_command"
            return self.command
        return ZERO

    def diagnostics(self, steady):
        return dict(reason=self.reason, clock_fault=self.clock.fault,
                    command_receipt_age_sec=None if self.command_time is None else max(0.0, steady-self.command_time),
                    status_receipt_age_sec=None if self.status_time is None else max(0.0, steady-self.status_time),
                    command_has_source_stamp=False, shadow_only=True)
