import hashlib
import json
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lane_mapping_lya_reference.controller import (ClosedRouteController, Command,
    ControlFault, Pose, SafetyArbiter, fresh, load_bundle)


def circle(radius=10.0, count=360):
    chord = 2.0 * radius * math.sin(math.pi / count)
    return {"schema_version": 1, "closed": True, "diagnostics": {"valid": True},
        "route_samples": [{"s_m": i * chord,
                           "x_m": radius * math.cos(i * 2 * math.pi / count),
                           "y_m": radius * math.sin(i * 2 * math.pi / count),
                           "yaw_rad": i * 2 * math.pi / count + math.pi / 2,
                           "curvature_1pm": 1 / radius, "speed_mps": 0.8}
                          for i in range(count)]}


def pose(now=1_000_000_000, speed=0.0, x=10.0, y=0.0, yaw=math.pi / 2):
    return Pose(x, y, yaw, speed, now, now)


def feed(arbiter, now=1_000_000_000, speed=0.4, yaw=0.05):
    arbiter.update_pose(pose(now))
    assert arbiter.set_teacher((speed, 0, 0, 0, 0, yaw), now, now)


def repeated(safety_mode="fixed_only"):
    arbiter = SafetyArbiter(safety_mode=safety_mode, stopped_settle_s=0.1)
    feed(arbiter)
    assert arbiter.arm(1_000_000_000, 1_000_000_000)[0]
    assert arbiter.prepare_repeat(ClosedRouteController(circle()))[0]
    feed(arbiter, 1_010_000_000)
    feed(arbiter, 1_120_000_000)
    assert arbiter.start_repeat(1_120_000_000, 1_120_000_000)[0]
    return arbiter


def test_no_implicit_arm_or_resume_after_stale():
    a = SafetyArbiter()
    feed(a)
    assert a.command(1_000_000_000, 1_000_000_000, 0.05) == Command()
    assert a.arm(1_000_000_000, 1_000_000_000)[0]
    assert a.command(1_050_000_000, 1_050_000_000, 0.05).speed == pytest.approx(0.025)
    assert a.command(1_300_000_000, 1_300_000_000, 0.05) == Command()
    assert a.state == "HOLD"
    feed(a, 1_310_000_000)
    assert a.command(1_310_000_000, 1_310_000_000, 0.05) == Command()


def test_ros_clock_pause_does_not_keep_commands_alive():
    a = SafetyArbiter()
    feed(a)
    a.arm(1_000_000_000, 1_000_000_000)
    assert a.command(1_000_000_000, 1_300_000_000, 0.05) == Command()
    assert a.state == "HOLD"


@pytest.mark.parametrize("now", [0, 900_000_000, 2_000_000_000])
def test_invalid_clock_fails_freshness(now):
    assert not fresh(1_000_000_000, 1_000_000_000, now, 1_100_000_000, 0.25)


@pytest.mark.parametrize("values", [(float("nan"), 0, 0, 0, 0, 0),
    (float("inf"), 0, 0, 0, 0, 0), (-1, 0, 0, 0, 0, 0),
    (2.3, 0, 0, 0, 0, 0), (0.3, 1, 0, 0, 0, 0), (0.3, 0, 0, 0, 0, 0.5)])
def test_invalid_teacher_fails_closed(values):
    a = SafetyArbiter()
    feed(a)
    a.arm(1_000_000_000, 1_000_000_000)
    assert not a.set_teacher(values, 1_000_000_000, 1_000_000_000)
    assert a.command(1_000_000_000, 1_000_000_000, 0.05) == Command()


def test_fast_legacy_teacher_is_limited_preserving_curvature():
    a = SafetyArbiter(maximum_speed=0.8)
    assert a.set_teacher((2.0, 0, 0, 0, 0, 0.4), 1, 1)
    assert a.teacher.speed == 0.8
    assert a.teacher.yaw_rate == pytest.approx(0.16)
    assert a.raw_teacher.speed == 2.0


def test_estop_latches_and_reset_never_rearms():
    a = repeated()
    a.manual_estop = True
    a.estop("test")
    assert not a.reset_estop()[0]
    assert a.command(1_120_000_000, 1_120_000_000, 0.05) == Command()
    a.manual_estop = False
    assert a.reset_estop()[0]
    assert a.state == "DISARMED"
    assert not a.arm(1_120_000_000, 1_120_000_000)[0]
    assert a.command(1_120_000_000, 1_120_000_000, 0.05) == Command()


def test_handover_requires_new_stopped_settle_period():
    a = SafetyArbiter()
    feed(a)
    a.prepare_repeat(ClosedRouteController(circle()))
    assert not a.start_repeat(1_000_000_000, 1_000_000_000)[0]
    for ns in (1_010_000_000, 1_200_000_000, 1_400_000_000, 1_600_000_000, 1_800_000_000, 2_010_000_000):
        feed(a, ns)
    assert a.start_repeat(2_010_000_000, 2_010_000_000)[0]


def test_missing_pose_interval_does_not_count_as_stopped_settle():
    a = SafetyArbiter()
    feed(a)
    a.prepare_repeat(ClosedRouteController(circle()))
    feed(a, 1_010_000_000)
    feed(a, 5_010_000_000)
    assert not a.start_repeat(5_010_000_000, 5_010_000_000)[0]


def test_fixed_mode_ignores_teacher_after_selection_even_invalid():
    a = repeated()
    a.set_teacher((float("nan"),) * 6, 1_130_000_000, 1_130_000_000)
    assert a.state == "REPEAT"
    assert a.command(1_130_000_000, 1_130_000_000, 0.05).speed > 0
    assert not a.arm(1_130_000_000, 1_130_000_000)[0]


def test_reference_disagreement_falls_back_but_pose_loss_stops():
    a = repeated("lya_reference")
    feed(a, 1_130_000_000, speed=0.8, yaw=-0.4)
    fallback = a.command(1_130_000_000, 1_130_000_000, 0.05)
    assert fallback.speed == pytest.approx(0.025)
    assert fallback.yaw_rate == pytest.approx(-0.04)
    assert a.state == "FALLBACK"
    assert not a.resume_reference(1_130_000_000, 1_130_000_000)[0]
    assert a.command(1_500_000_000, 1_500_000_000, 0.05) == Command()
    assert a.state == "HOLD"


def test_reference_never_automatically_recovers():
    a = repeated("lya_reference")
    feed(a, 1_130_000_000, speed=0.8, yaw=-0.4)
    a.command(1_130_000_000, 1_130_000_000, 0.05)
    for index in range(1, 20):
        ns = 1_130_000_000 + index * 50_000_000
        feed(a, ns, speed=0.8, yaw=0.08)
        a.command(ns, ns, 0.05)
    assert a.state == "FALLBACK"
    assert a.resume_reference(ns, ns)[0]
    assert a.state == "REPEAT"


def test_reference_teacher_loss_never_uses_unchecked_candidate():
    a = repeated("lya_reference")
    a.update_pose(pose(1_500_000_000))
    assert a.command(1_500_000_000, 1_500_000_000, 0.05) == Command()
    assert a.state == "HOLD"


@pytest.mark.parametrize("keyword", ["maximum_speed", "maximum_yaw_rate", "pose_timeout_s",
    "teacher_timeout_s", "stopped_speed", "stopped_settle_s", "reference_speed_delta",
    "reference_yaw_delta", "reference_recovery_s", "accepted_teacher_max_speed",
    "maximum_acceleration", "maximum_yaw_acceleration"])
@pytest.mark.parametrize("bad", [0, -1, float("nan"), float("inf")])
def test_safety_thresholds_reject_invalid(keyword, bad):
    with pytest.raises(ValueError):
        SafetyArbiter(**{keyword: bad})


def test_controller_curvature_feedforward_and_acceleration_limits():
    c = ClosedRouteController(circle())
    p = pose()
    c.attach(p)
    first = c.update(p, 0.05)
    assert first.speed == pytest.approx(0.025)
    assert first.yaw_rate == pytest.approx(0.04)
    for _ in range(40):
        command = c.update(p, 0.05)
    assert command.speed == pytest.approx(0.8)
    assert command.yaw_rate == pytest.approx(0.08)


def test_initial_attachment_cannot_select_other_lane():
    c = ClosedRouteController(circle())
    with pytest.raises(ControlFault):
        c.attach(pose(x=-10, yaw=-math.pi / 2))
    with pytest.raises(ControlFault):
        c.attach(pose(yaw=-math.pi / 2))


def test_controller_cannot_jump_to_farther_arc():
    c = ClosedRouteController(circle())
    c.attach(pose())
    with pytest.raises(ControlFault):
        c.update(pose(x=-10, yaw=-math.pi / 2), 0.05)
    assert c.progress == 0.0


def test_controller_tracks_over_closed_seam_monotonically():
    c = ClosedRouteController(circle())
    c.attach(pose())
    previous = 0.0
    for i in range(2200):
        angle = i * 0.004
        c.update(pose(x=10 * math.cos(angle), y=10 * math.sin(angle), yaw=angle + math.pi / 2), 0.05)
        assert c.progress >= previous
        previous = c.progress
    assert c.progress > c.length


@pytest.mark.parametrize("mutation", ["invalid_diagnostics", "wrong_yaw", "wrong_curvature", "invalid_arc", "nan", "open_seam"])
def test_corrupt_geometry_rejected(mutation):
    route = circle()
    if mutation == "invalid_diagnostics":
        route["diagnostics"]["valid"] = False
    elif mutation == "wrong_yaw":
        route["route_samples"][0]["yaw_rad"] = 0.0
    elif mutation == "wrong_curvature":
        route["route_samples"][-1]["curvature_1pm"] = 0.0
    elif mutation == "invalid_arc":
        route["route_samples"][1]["s_m"] = 0.0
    elif mutation == "nan":
        route["route_samples"][0]["x_m"] = float("nan")
    else:
        route["route_samples"] = route["route_samples"][:180]
    with pytest.raises(ValueError):
        ClosedRouteController(route)


def bundle(tmp_path):
    coordinates = {"origin_lla": [35.0, 139.0, 10.0], "yaw_offset_rad": 0.0,
                   "frame_id": "lane_map", "heading_source": "vectornav_yaw_ned_to_enu"}
    route = dict(circle(), **coordinates)
    metadata = dict(finished=True, **coordinates)
    manifest = dict(schema_version=1, ready=True, **coordinates)
    for kind, payload in (("route", route), ("metadata", metadata)):
        raw = json.dumps(payload).encode("utf-8")
        (tmp_path / (kind + ".json")).write_bytes(raw)
        manifest[kind + "_path"] = kind + ".json"
        manifest[kind + "_sha256"] = hashlib.sha256(raw).hexdigest()
    path = tmp_path / "bundle.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path, manifest


def test_valid_bundle_roundtrip(tmp_path):
    path, _ = bundle(tmp_path)
    manifest, route, metadata = load_bundle(path)
    assert metadata["finished"]
    assert len(ClosedRouteController(route).samples) == 360


@pytest.mark.parametrize("mutation", ["checksum", "origin", "yaw", "path", "not_ready", "scalar"])
def test_bad_bundle_fails_closed(tmp_path, mutation):
    path, manifest = bundle(tmp_path)
    if mutation == "checksum":
        manifest["route_sha256"] = "wrong"
    elif mutation == "origin":
        manifest["origin_lla"][0] += 1.0
    elif mutation == "yaw":
        manifest["yaw_offset_rad"] += 1.0
    elif mutation == "path":
        manifest["route_path"] = "../route.json"
    elif mutation == "not_ready":
        manifest["ready"] = False
    else:
        manifest = []
    path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError):
        load_bundle(path)


def test_reference_startup_compares_nominal_not_acceleration_ramp():
    a = repeated("lya_reference")
    feed(a, 1_130_000_000, speed=0.8, yaw=0.08)
    output = a.command(1_130_000_000, 1_130_000_000, 0.05)
    assert a.state == "REPEAT"
    assert output.speed == pytest.approx(0.025)
    assert output.yaw_rate == pytest.approx(0.04)
    assert a.controller.last_metrics["requested_speed_mps"] == pytest.approx(0.8)


@pytest.mark.parametrize("mode", ["TEACH", "FALLBACK"])
def test_all_teacher_outputs_are_slew_limited(mode):
    if mode == "TEACH":
        a = SafetyArbiter()
        feed(a, speed=0.8, yaw=-0.4)
        a.arm(1_000_000_000, 1_000_000_000)
    else:
        a = repeated("lya_reference")
    previous = Command()
    for index in range(40):
        ns = 1_200_000_000 + index * 50_000_000
        feed(a, ns, speed=0.8, yaw=-0.4)
        output = a.command(ns, ns, 0.05)
        assert abs(output.speed - previous.speed) <= 0.025 + 1e-9
        assert abs(output.yaw_rate - previous.yaw_rate) <= 0.04 + 1e-9
        assert a.state == mode
        previous = output
    assert output == Command(0.8, -0.4)


@pytest.mark.parametrize("mode", ["TEACH", "REPEAT", "FALLBACK"])
@pytest.mark.parametrize("dt", [0, 0.5, float("nan")])
def test_timer_deadline_applies_to_all_active_modes(mode, dt):
    a = repeated("lya_reference")
    a.state = mode
    feed(a, 1_130_000_000, speed=0.8, yaw=0.08)
    assert a.command(1_130_000_000, 1_130_000_000, dt) == Command()
    assert a.state == "HOLD"


@pytest.mark.parametrize("mode", ["TEACH", "REPEAT", "FALLBACK"])
def test_teacher_zero_is_immediate_stop_not_slow_deceleration(mode):
    a = repeated("lya_reference")
    a.state = mode
    a.last_output = Command(0.8, 0.4)
    feed(a, 1_130_000_000, speed=0.0, yaw=0.0)
    assert a.command(1_130_000_000, 1_130_000_000, 0.05) == Command()
    assert a.last_output == Command()


def test_hard_stop_clears_ramp_before_explicit_rearm():
    a = SafetyArbiter()
    feed(a, speed=0.8)
    a.arm(1_000_000_000, 1_000_000_000)
    a.last_output = Command(0.8, 0.2)
    a.estop("test")
    assert a.last_output == Command()
    a.reset_estop()
    feed(a, 1_100_000_000, speed=0.8)
    assert a.arm(1_100_000_000, 1_100_000_000)[0]
    assert a.command(1_100_000_000, 1_100_000_000, 0.05).speed == pytest.approx(0.025)


def test_teacher_stop_then_go_between_ticks_still_restarts_ramp():
    a = SafetyArbiter()
    feed(a, speed=0.8)
    a.arm(1_000_000_000, 1_000_000_000)
    a.last_output = Command(0.8, 0.2)
    a.set_teacher((0, 0, 0, 0, 0, 0), 1_010_000_000, 1_010_000_000)
    assert a.last_output == Command()
    feed(a, 1_020_000_000, speed=0.8)
    assert a.command(1_020_000_000, 1_020_000_000, 0.05).speed == pytest.approx(0.025)


def test_reference_allows_slower_fixed_bend_at_matching_curvature():
    a = repeated("lya_reference")
    route = circle()
    for sample in route["route_samples"]:
        sample["speed_mps"] = 0.2
    a.controller = ClosedRouteController(route)
    a.controller.attach(a.pose)
    feed(a, 1_130_000_000, speed=0.8, yaw=0.08)
    output = a.command(1_130_000_000, 1_130_000_000, 0.05)
    assert a.state == "REPEAT"
    assert output.speed == pytest.approx(0.025)
    assert a.controller.last_metrics["requested_speed_mps"] == pytest.approx(0.2)


def test_yaw_fallback_never_accelerates_above_valid_candidate_slowdown():
    a = repeated("lya_reference")
    route = circle()
    for sample in route["route_samples"]:
        sample["speed_mps"] = 0.4
    a.controller = ClosedRouteController(route)
    a.controller.attach(a.pose)
    for index in range(40):
        ns = 1_130_000_000 + index * 50_000_000
        feed(a, ns, speed=0.8, yaw=-0.4)
        output = a.command(ns, ns, 0.05)
        assert a.state == "FALLBACK"
        assert output.speed <= 0.4 + 1e-9
        assert output.yaw_rate >= -0.2 - 1e-9
    assert output == Command(0.4, -0.2)
