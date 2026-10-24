"""Pure policy tests. These do not install ROS stubs or claim DDS execution."""
import json
import math

import pytest

from semantic_drive_bridge.policy import (
    ClockGuard, CommandWatchdogPolicy, Rejected, SignalCapturePolicy,
    ZERO, parse_map_status, validate_shadow_topic,
)


FRAME = "zed_left_camera_optical_frame"
CMD = (1.0, 0.0, 0.0, 0.0, 0.0, 0.2)


def capture(stamp=10000000000, **changes):
    value = dict(header=dict(stamp=dict(sec=stamp // 1000000000,
                                       nanosec=stamp % 1000000000), frame_id=FRAME),
                 sequence=stamp, state="red", distance_m=12.0, uncertainty_m=0.2)
    value.update(changes)
    return json.dumps(value)


def status(stamp=10.0, **changes):
    # Exact fields and types from core63 MapAssist._publish_status. Additional
    # diagnostics are intentionally allowed; no fake rclpy object is involved.
    value = dict(phase="learning", reason="vision", ros_time_sec=stamp,
                 avoidance_active=False, path_output_allowed=True,
                 traffic_mode="normal", velocity_gate_reason="forwarding_controller",
                 point_age_sec=dict(a=0.05, b=0.05, c=0.05), odom_age_sec=0.01)
    value.update(changes)
    return json.dumps(value)


def ready_watchdog():
    policy = CommandWatchdogPolicy()
    policy.receive_status(status(), 10000000000, 20.0)
    policy.receive_command(CMD, 10010000000, 20.01)
    return policy


def test_receiver_age_includes_source_to_receiver_delay():
    policy = SignalCapturePolicy(FRAME)
    result = policy.convert(capture(), 10250000000, 20.0)
    assert result == dict(sequence=10000000000, state="red", age_sec=0.25,
                          distance_m=12.0, uncertainty_m=0.2)


def test_real_unknown_and_real_red_without_range_are_forwarded_as_themselves():
    policy = SignalCapturePolicy(FRAME)
    unknown = policy.convert(capture(state="unknown", distance_m=None, uncertainty_m=None), 10000000000, 20.0)
    with pytest.raises(Rejected, match="duplicate"):
        policy.convert(capture(state="unknown", distance_m=None, uncertainty_m=None), 10000000000, 20.01)
    red = policy.convert(capture(10100000000, distance_m=None, uncertainty_m=None), 10100000000, 20.1)
    assert unknown["state"] == "unknown" and unknown["distance_m"] is None
    assert red["state"] == "red" and red["distance_m"] is None
    assert unknown["uncertainty_m"] == red["uncertainty_m"] == 0.0


def test_missing_range_preserves_current_color_not_previous_successful_range():
    policy = SignalCapturePolicy(FRAME)
    policy.convert(capture(), 10000000000, 20.0)
    unknown = policy.convert(capture(10100000000, state="unknown", distance_m=None, uncertainty_m=None),
                             10100000000, 20.1)
    assert unknown["state"] == "unknown"
    assert unknown["distance_m"] is None
    assert unknown["uncertainty_m"] == 0.0


@pytest.mark.parametrize("clock,reason", [(9999999999, "future"), (10500000001, "stale")])
def test_future_and_expired_capture_abstain(clock, reason):
    policy = SignalCapturePolicy(FRAME)
    with pytest.raises(Rejected, match=reason):
        policy.convert(capture(), clock, 20.0)
    assert policy.last_accepted_source_ns == 0
    assert policy.last_received_steady is None


def test_capture_age_boundary_and_one_nanosecond_over():
    policy = SignalCapturePolicy(FRAME)
    assert policy.convert(capture(), 10500000000, 20.0)["age_sec"] == 0.5
    with pytest.raises(Rejected, match="stale"):
        policy.convert(capture(11000000000), 11500000001, 21.0)


def test_duplicate_and_out_of_order_never_refresh():
    policy = SignalCapturePolicy(FRAME)
    policy.convert(capture(), 10000000000, 20.0)
    for stamp in (10000000000, 9900000000):
        with pytest.raises(Rejected, match="duplicate_or_rolled_back"):
            policy.convert(capture(stamp), 10100000000, 20.1)
    assert policy.last_received_steady == 20.0
    assert policy.last_accepted_source_ns == 10000000000


def test_rejected_frame_cannot_be_corrected_and_replayed_as_new_vote():
    policy = SignalCapturePolicy(FRAME)
    with pytest.raises(Rejected, match="invalid_signal_state"):
        policy.convert(capture(state="yellow"), 10000000000, 20.0)
    with pytest.raises(Rejected, match="duplicate"):
        policy.convert(capture(), 10100000000, 20.1)
    assert policy.last_accepted_source_ns == 0
    assert policy.convert(capture(10200000000), 10200000000, 20.2)["state"] == "red"


def test_future_identity_does_not_become_fresh_when_clock_catches_up():
    policy = SignalCapturePolicy(FRAME)
    with pytest.raises(Rejected, match="future"):
        policy.convert(capture(), 9900000000, 20.0)
    with pytest.raises(Rejected, match="duplicate"):
        policy.convert(capture(), 10000000000, 20.1)


@pytest.mark.parametrize("changes", [
    {"sequence": True}, {"sequence": 10000000000.0}, {"sequence": 1},
    {"header": {}},
    {"header": {"stamp": {"sec": 10, "nanosec": 0}, "frame_id": "base_link"}},
    {"header": {"stamp": {"sec": 10, "nanosec": 1000000000}, "frame_id": FRAME}},
    {"header": {"stamp": {"sec": True, "nanosec": 0}, "frame_id": FRAME}},
])
def test_invalid_capture_identity(changes):
    policy = SignalCapturePolicy(FRAME)
    with pytest.raises(Rejected):
        policy.convert(capture(**changes), 10000000000, 20.0)
    assert policy.last_received_steady is None


def test_zero_stamp_and_unstarted_ros_clock_rejected():
    for payload, clock in ((capture(0), 0), (capture(1), 0)):
        with pytest.raises(Rejected):
            SignalCapturePolicy(FRAME).convert(payload, clock, 20.0)


@pytest.mark.parametrize("changes", [
    {"distance_m": float("nan")}, {"distance_m": float("inf")},
    {"distance_m": -1}, {"distance_m": True}, {"distance_m": "12"},
    {"uncertainty_m": -0.01}, {"uncertainty_m": float("nan")},
    {"uncertainty_m": True}, {"state": "RED"},
])
def test_invalid_capture_values_do_not_publish(changes):
    policy = SignalCapturePolicy(FRAME)
    with pytest.raises(Rejected):
        policy.convert(capture(**changes), 10000000000, 20.0)
    assert policy.last_received_steady is None


@pytest.mark.parametrize("payload", ["{", "null", "[]", "true", "123"])
def test_malformed_payload_never_substitutes_unknown(payload):
    policy = SignalCapturePolicy(FRAME)
    with pytest.raises(Rejected):
        policy.convert(payload, 10000000000, 20.0)
    assert policy.last_accepted_source_ns == 0


def test_missing_range_key_differs_from_explicit_null():
    data = json.loads(capture())
    del data["distance_m"]
    with pytest.raises(Rejected, match="missing_range_fields"):
        SignalCapturePolicy(FRAME).convert(json.dumps(data), 10000000000, 20.0)


def test_capture_timeout_on_paused_ros_clock_does_not_publish_cached_color():
    policy = SignalCapturePolicy(FRAME)
    policy.convert(capture(), 10000000000, 20.0)
    diagnostic = policy.poll(10000000000, 20.51)
    assert diagnostic["reason"] == "capture_stream_timeout_abstaining"
    assert diagnostic["publishing_cached_observations"] is False
    assert policy.last_received_steady == 20.0


def test_signal_clock_rollback_latches_through_later_forward_time():
    policy = SignalCapturePolicy(FRAME)
    policy.convert(capture(), 10000000000, 20.0)
    assert "rollback" in policy.poll(9999999999, 20.1)["clock_fault"]
    with pytest.raises(Rejected, match="restart_required"):
        policy.convert(capture(11000000000), 11000000000, 21.0)


def test_clock_guard_rejects_steady_rollback_and_invalid_clock():
    guard = ClockGuard()
    guard.observe(10, 20.0)
    with pytest.raises(Rejected, match="steady_clock_rollback"):
        guard.observe(11, 19.9)
    for ros_ns in (True, -1, 1.5):
        with pytest.raises(Rejected, match="invalid_ros_clock"):
            ClockGuard().observe(ros_ns, 20.0)


@pytest.mark.parametrize("value", [0, -1, math.inf, math.nan, True])
def test_timeout_parameters_require_finite_positive_numbers(value):
    with pytest.raises(Rejected):
        SignalCapturePolicy(FRAME, value)
    with pytest.raises(Rejected):
        CommandWatchdogPolicy(value)


@pytest.mark.parametrize("value", ["", " ", " frame ", None])
def test_required_explicit_frame(value):
    with pytest.raises(Rejected):
        SignalCapturePolicy(value)


def test_fresh_command_and_status_forward_six_finite_values():
    policy = ready_watchdog()
    assert policy.output(10020000000, 20.02) == CMD
    assert policy.diagnostics(20.02)["command_has_source_stamp"] is False


def test_command_cannot_precede_permitting_status_or_reappear_after_fault():
    policy = CommandWatchdogPolicy()
    with pytest.raises(Rejected, match="without_fresh"):
        policy.receive_command(CMD, 10000000000, 20.0)
    policy.receive_status(status(10.1), 10100000000, 20.1)
    assert policy.output(10100000000, 20.1) == ZERO
    policy.receive_command(CMD, 10100000000, 20.1)
    policy.receive_status(status(10.2, velocity_gate_reason="controller_command_timeout"), 10200000000, 20.2)
    assert policy.output(10200000000, 20.2) == ZERO
    policy.receive_status(status(10.3), 10300000000, 20.3)
    assert policy.output(10300000000, 20.3) == ZERO
    policy.receive_command(CMD, 10300000000, 20.3)
    assert policy.output(10300000000, 20.3) == CMD


def test_command_timeout_with_fresh_status():
    policy = ready_watchdog()
    policy.receive_status(status(10.25), 10250000000, 20.25)
    assert policy.output(10260000000, 20.26) == CMD
    assert policy.output(10260000001, 20.260001) == ZERO
    assert policy.reason == "command_receipt_timeout"


def test_status_timeout_even_with_fresh_command_and_paused_ros_time():
    policy = ready_watchdog()
    policy.receive_command(CMD, 10010000000, 20.24)
    assert policy.output(10010000000, 20.250001) == ZERO
    assert policy.reason == "map_status_receipt_timeout"


@pytest.mark.parametrize("reason,mode", [
    ("disabled", "normal"), ("controller_command_timeout", "normal"),
    ("approach_braking", "approach"), ("red_wait_first_lap", "wait"),
    ("signal_color_unknown", "perception_suspended"),
    ("unrecognized_future_gate", "normal"),
])
def test_core_hold_reasons_block_nonzero(reason, mode):
    policy = ready_watchdog()
    policy.receive_status(status(10.1, velocity_gate_reason=reason, traffic_mode=mode), 10100000000, 20.1)
    assert policy.output(10100000000, 20.1) == ZERO


def test_approach_and_explicit_oa_use_real_core_status_contract():
    assert parse_map_status(status(traffic_mode="approach", velocity_gate_reason="signal_approach_speed_limit"))["allowed"]
    assert parse_map_status(status(traffic_mode="wait", avoidance_active=True, path_output_allowed=False))["allowed"]
    assert not parse_map_status(status(avoidance_active=None))["allowed"]
    assert not parse_map_status(status(path_output_allowed=False))["allowed"]


@pytest.mark.parametrize("changes", [
    {"ros_time_sec": float("nan")}, {"ros_time_sec": 0},
    {"path_output_allowed": "true"}, {"avoidance_active": 1},
    {"velocity_gate_reason": None}, {"traffic_mode": "made_up"},
])
def test_invalid_map_status_clears_command(changes):
    policy = ready_watchdog()
    with pytest.raises(Rejected):
        policy.receive_status(status(10.1, **changes), 10100000000, 20.1)
    assert policy.output(10100000000, 20.1) == ZERO


@pytest.mark.parametrize("stamp", [10.0, 9.9, 10.6])
def test_old_duplicate_and_future_map_status_not_fresh(stamp):
    policy = ready_watchdog()
    with pytest.raises(Rejected):
        policy.receive_status(status(stamp), 10100000000, 20.1)
    assert policy.output(10100000000, 20.1) == ZERO


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), True, "1"])
def test_each_twist_component_must_be_finite(bad):
    for index in range(6):
        policy = ready_watchdog()
        values = list(CMD)
        values[index] = bad
        with pytest.raises(Rejected):
            policy.receive_command(values, 10020000000, 20.02)
        assert policy.output(10020000000, 20.02) == ZERO


def test_watchdog_ros_rollback_zero_and_restart_latch():
    policy = ready_watchdog()
    assert policy.output(10000000000, 20.02) == ZERO
    assert "ros_clock_rollback" in policy.clock.fault
    with pytest.raises(Rejected, match="restart_required"):
        policy.receive_status(status(11.0), 11000000000, 21.0)
    assert policy.output(11000000000, 21.0) == ZERO


def test_watchdog_does_not_start_with_zero_ros_clock():
    policy = CommandWatchdogPolicy()
    assert policy.output(0, 20.0) == ZERO
    assert policy.reason == "ros_clock_not_started"


@pytest.mark.parametrize("topic", [
    "/aiformula_control/game_pad/cmd_vel", "aiformula_control/game_pad/cmd_vel",
    "/aiformula_control/other/cmd_vel", "/cmd_vel", "~/shadow_cmd_vel",
    "/semantic_drive/shadow_cmd_vel/", "/semantic_drive/shadow_cmd_vel//escape",
])
def test_live_or_unresolved_output_names_rejected(topic):
    with pytest.raises(Rejected):
        validate_shadow_topic(topic)


def test_shadow_output_validation_applies_to_resolved_remap_too():
    assert validate_shadow_topic("/semantic_drive/shadow_cmd_vel")
    assert validate_shadow_topic("/semantic_drive/shadow_candidate")
    # The node applies the same function to Publisher.topic_name after remap.
    with pytest.raises(Rejected):
        validate_shadow_topic("/aiformula_control/game_pad/cmd_vel")
