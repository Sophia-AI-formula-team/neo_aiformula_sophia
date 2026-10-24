"""A fixed reference is explicit; teacher commands are not silently rewritten."""

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lane_mapping_lya_reference.controller import (
    ClosedRouteController, Command, ControlFault, Pose, SafetyArbiter,
)
from lane_mapping_lya_reference.route import RouteBuildError, build_route


NOW = 1_000_000_000


def fixed_circle(radius=20.0, count=360, reference=2.0):
    chord = 2.0 * radius * math.sin(math.pi / count)
    return {"schema_version": 1, "closed": True, "diagnostics": {"valid": True},
            "speed_policy": "fixed_reference", "reference_speed_mps": reference,
            "route_samples": [{"s_m": i * chord,
                "x_m": radius * math.cos(i * 2 * math.pi / count),
                "y_m": radius * math.sin(i * 2 * math.pi / count),
                "yaw_rad": i * 2 * math.pi / count + math.pi / 2,
                "curvature_1pm": 1.0 / radius, "speed_mps": reference}
                for i in range(count)]}


def controller(route=None, **kwargs):
    settings = dict(maximum_speed=2.0, maximum_yaw_rate=2.0, reference_speed_mps=2.0)
    settings.update(kwargs)
    return ClosedRouteController(fixed_circle() if route is None else route, **settings)


def pose(now=NOW, yaw=math.pi / 2):
    return Pose(20.0, 0.0, yaw, 0.0, now, now)


def teacher_arbiter(**kwargs):
    settings = dict(preserve_teacher_command=True, accepted_teacher_max_speed=0.0,
                    accepted_teacher_max_yaw_rate=2.0)
    settings.update(kwargs)
    arbiter = SafetyArbiter(**settings)
    arbiter.update_pose(pose())
    assert arbiter.set_teacher((2.1, 0, 0, 0, 0, 0.6), NOW, NOW)
    assert arbiter.arm(NOW, NOW)[0]
    return arbiter


def test_preserved_teacher_bypasses_repeat_point_eight_cap_and_slew():
    arbiter = teacher_arbiter()
    assert arbiter.maximum_speed == 0.8
    assert arbiter.maximum_yaw_rate == 0.4
    assert arbiter.command(NOW, NOW, 0.05) == Command(2.1, 0.6)


def test_preserved_teacher_obeys_explicit_acceptance_limits():
    arbiter = teacher_arbiter(accepted_teacher_max_speed=2.2)
    assert not arbiter.set_teacher((2.3, 0, 0, 0, 0, 0.6), NOW, NOW)
    assert arbiter.state == "HOLD"
    assert arbiter.command(NOW, NOW, 0.05) == Command()


@pytest.mark.parametrize("values", [
    (float("nan"), 0, 0, 0, 0, 0), (float("inf"), 0, 0, 0, 0, 0),
    (-0.1, 0, 0, 0, 0, 0), (2.1, 0.1, 0, 0, 0, 0), (2.1, 0, 0, 0, 0, 2.1),
])
def test_preserved_teacher_keeps_finite_forward_planar_and_yaw_gates(values):
    arbiter = teacher_arbiter()
    assert not arbiter.set_teacher(values, NOW, NOW)
    assert arbiter.command(NOW, NOW, 0.05) == Command()


@pytest.mark.parametrize("fault", ["pose", "teacher", "clock_pause", "deadline", "estop", "zero"])
def test_preserved_teacher_fault_or_stop_is_immediate_zero(fault):
    arbiter = teacher_arbiter()
    assert arbiter.command(NOW, NOW, 0.05).speed == 2.1
    now, steady, dt = NOW, NOW, 0.05
    if fault in ("pose", "teacher"):
        now = steady = NOW + 300_000_000
        if fault == "teacher":
            arbiter.update_pose(pose(now))
        else:
            assert arbiter.set_teacher((2.1, 0, 0, 0, 0, 0.6), now, steady)
    elif fault == "clock_pause":
        steady += 300_000_000
    elif fault == "deadline":
        dt = 0.5
    elif fault == "estop":
        arbiter.estop("fixture emergency stop")
    else:
        assert arbiter.set_teacher((0, 0, 0, 0, 0, 0), NOW, NOW)
    assert arbiter.command(now, steady, dt) == Command()
    assert arbiter.last_output == Command()


@pytest.mark.parametrize("candidate_valid", [False, True])
def test_preserved_fallback_uses_the_original_teacher_not_repeat_cap(candidate_valid):
    class Candidate:
        last_metrics = {"requested_speed_mps": 0.2, "requested_yaw_rate_rps": 0.0}

        def update(self, unused_pose, unused_dt):
            if not candidate_valid:
                raise ControlFault("fixture rejected candidate")
            return Command(0.2, 0.0)

    arbiter = teacher_arbiter(safety_mode="lya_reference")
    arbiter.controller = Candidate()
    arbiter.state = "FALLBACK"
    assert arbiter.command(NOW, NOW, 0.05) == Command(2.1, 0.6)
    assert arbiter.state == "FALLBACK"


def test_zero_teacher_acceptance_limit_is_only_a_preserve_mode_sentinel():
    with pytest.raises(ValueError, match="teacher speed limit"):
        SafetyArbiter(accepted_teacher_max_speed=0.0)
    assert teacher_arbiter().accepted_teacher_max_speed == 0.0


def test_fixed_reference_retains_two_with_feedback_and_startup_slew():
    follower = controller()
    p = pose(yaw=math.pi / 2 + 0.1)
    follower.attach(p)
    first = follower.update(p, 0.05)
    assert first.speed == pytest.approx(0.025)
    assert follower.last_metrics["v_ref_mps"] == 2.0
    assert follower.last_metrics["speed_policy"] == "fixed_reference"
    assert follower.last_metrics["requested_speed_mps"] == pytest.approx(2.0 * math.cos(0.1))
    for _ in range(100):
        command = follower.update(p, 0.05)
    assert command.speed == pytest.approx(2.0 * math.cos(0.1))
    assert follower.last_metrics["reference_speed_mps"] == 2.0


@pytest.mark.parametrize("mutation", ["old_profile", "different_reference", "curve_sample", "missing_reference"])
def test_fixed_mode_rejects_old_or_mixed_speed_policy(mutation):
    route = fixed_circle()
    if mutation == "old_profile":
        route.pop("speed_policy")
    elif mutation == "different_reference":
        route["reference_speed_mps"] = 1.0
    elif mutation == "curve_sample":
        route["route_samples"][5]["speed_mps"] = 0.5
    else:
        route.pop("reference_speed_mps")
    with pytest.raises(ValueError, match="speed policy|speed profile"):
        controller(route)


def test_legacy_profile_mode_does_not_silently_consume_fixed_bundle():
    with pytest.raises(ValueError, match="explicit fixed reference"):
        ClosedRouteController(fixed_circle())


@pytest.mark.parametrize("settings", [
    {"maximum_lateral_acceleration": 0.1}, {"maximum_yaw_rate": 0.05},
    {"maximum_speed": 1.0},
])
def test_fixed_reference_infeasibility_is_reported_not_slowed(settings):
    with pytest.raises(ValueError, match="fixed speed infeasible.*allowed="):
        controller(**settings)


def test_fixed_reference_runtime_rechecks_feasibility_without_slowing():
    follower = controller()
    follower.attach(pose())
    follower.maximum_lateral_acceleration = 0.1
    with pytest.raises(ControlFault, match="fixed speed infeasible"):
        follower.update(pose(), 0.05)


def lane_inputs(radius=12.0):
    angle = np.linspace(0.0, 2.0 * math.pi, 501)
    trace = np.column_stack((radius * np.cos(angle), radius * np.sin(angle), angle + math.pi / 2))
    lane_angle = np.linspace(0.0, 2.0 * math.pi, 2400, endpoint=False)
    lanes = np.vstack([np.column_stack((r * np.cos(lane_angle), r * np.sin(lane_angle)))
                       for r in (radius - 1.2, radius + 1.2)])
    return trace, lanes


def test_route_fixed_reference_has_no_legacy_curve_or_max_speed_replay():
    result = build_route(*lane_inputs(), config={"reference_speed_mps": 2.0})
    assert result["config"]["max_speed_mps"] == 1.0  # Legacy profile-only limit.
    assert result["speed_policy"] == "fixed_reference"
    assert result["reference_speed_mps"] == 2.0
    assert {row["speed_mps"] for row in result["route_samples"]} == {2.0}


def test_route_fixed_reference_rejects_lateral_limit_instead_of_lowering_speed():
    with pytest.raises(RouteBuildError, match="fixed speed infeasible.*allowed=") as caught:
        build_route(*lane_inputs(), config={"reference_speed_mps": 2.0, "max_lateral_accel_mps2": 0.1})
    assert caught.value.diagnostics["required_lateral_accel_mps2"] > 0.1
    assert caught.value.diagnostics["reference_speed_mps"] == 2.0


@pytest.mark.parametrize("value", [-1, float("nan"), float("inf"), True])
def test_route_invalid_reference_speed_rejected(value):
    with pytest.raises(RouteBuildError):
        build_route(*lane_inputs(), config={"reference_speed_mps": value})
