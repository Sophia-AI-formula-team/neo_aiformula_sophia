"""Pure-NumPy first-lap reference generation acceptance and rejection tests."""

import json
import math

import numpy as np
import pytest

from lane_mapping_lya_reference.route import RouteBuildError, build_route


def circle(radius=12.0, half_width=1.2, noise=0.0, seed=2):
    angle = np.linspace(0.0, 2.0 * math.pi, 501)
    radial = radius + noise * np.random.default_rng(seed).normal(size=len(angle))
    radial[-1] = radial[0]
    trace = np.column_stack((radial * np.cos(angle), radial * np.sin(angle), angle + math.pi / 2.0))
    lane_angle = np.linspace(0.0, 2.0 * math.pi, 2400, endpoint=False)
    lanes = np.vstack([np.column_stack((r * np.cos(lane_angle), r * np.sin(lane_angle)))
                       for r in (radius - half_width, radius + half_width)])
    return trace, lanes


def xy(result):
    return np.array([[sample["x_m"], sample["y_m"]] for sample in result["route_samples"]])


def test_closed_circle_schema_spacing_and_centering():
    trace, lanes = circle()
    result = build_route(trace, lanes)
    samples = result["route_samples"]
    route = xy(result)
    lengths = np.linalg.norm(np.roll(route, -1, axis=0) - route, axis=1)
    assert result["closed"] is True
    assert result["diagnostics"]["valid"] is True
    assert result["diagnostics"]["requires_operator_review"] is True
    assert result["diagnostics"]["obstacle_safety_verified"] is False
    assert np.max(np.abs(np.linalg.norm(route, axis=1) - 12.0)) < 0.025
    assert np.min(lengths) > 0.19
    assert np.max(lengths) <= 0.20 + 1e-6
    assert samples[0]["s_m"] == 0.0
    assert np.all(np.diff([sample["s_m"] for sample in samples]) > 0)
    assert np.linalg.norm(route[0] - route[-1]) > 0.19
    assert np.max(np.abs([sample["curvature_1pm"] for sample in samples])) < 0.10
    json.dumps(result, allow_nan=False)


def test_noisy_prior_is_pulled_towards_lane_center():
    trace, lanes = circle(noise=0.035)
    result = build_route(trace, lanes)
    assert np.max(np.abs(np.linalg.norm(xy(result), axis=1) - 12.0)) < 0.06
    assert result["diagnostics"]["final_support_fraction"] >= 0.98


def test_neighboring_lanes_do_not_replace_nearest_corridor():
    trace, lanes = circle()
    _, wider = circle(half_width=3.6)
    result = build_route(trace, np.vstack((lanes, wider)))
    widths = [s["left_clearance_m"] + s["right_clearance_m"] for s in result["route_samples"]]
    assert np.max(np.abs(np.array(widths) - 2.4)) < 0.03


def test_near_edge_is_not_skipped_in_favour_of_an_adjacent_lane():
    trace, _ = circle(radius=13.1)
    _, lanes = circle()
    _, wider = circle(half_width=3.6)
    with pytest.raises(RouteBuildError, match="too close"):
        build_route(trace, np.vstack((lanes, wider)))


def test_missing_long_lane_span_rejects_instead_of_bridging():
    trace, lanes = circle()
    angle = np.arctan2(lanes[:, 1], lanes[:, 0])
    lanes = lanes[(angle < 0.6) | (angle > 1.0)]
    with pytest.raises(RouteBuildError, match="missing") as caught:
        build_route(trace, lanes)
    assert caught.value.diagnostics["prior_support_fraction"] < 0.98
    assert caught.value.diagnostics["valid"] is False


@pytest.mark.parametrize("half_width", [0.35, 3.0])
def test_implausible_lane_width_rejected(half_width):
    trace, lanes = circle(half_width=half_width)
    with pytest.raises(RouteBuildError, match="width"):
        build_route(trace, lanes)


def test_incomplete_lap_position_rejected():
    trace, lanes = circle()
    with pytest.raises(RouteBuildError, match="return close"):
        build_route(trace[:400], lanes)


def test_wrong_closure_heading_rejected():
    trace, lanes = circle()
    trace[-1, 2] += math.pi / 2
    with pytest.raises(RouteBuildError, match="heading"):
        build_route(trace, lanes)


def test_stationary_or_short_recording_is_not_a_lap():
    trace, lanes = circle(radius=1.5, half_width=0.6)
    with pytest.raises(RouteBuildError, match="shorter"):
        build_route(trace, lanes)


def test_localization_jump_rejected():
    trace, lanes = circle()
    trace[100, :2] += 5.0
    with pytest.raises(RouteBuildError, match="jump"):
        build_route(trace, lanes)


def test_self_crossing_prior_rejected():
    angle = np.linspace(0.0, 2.0 * math.pi, 501)
    trace = np.column_stack((12.0 * np.sin(angle), 6.0 * np.sin(2.0 * angle),
                             np.arctan2(12.0 * np.cos(2.0 * angle), 12.0 * np.cos(angle))))
    _, lanes = circle()
    with pytest.raises(RouteBuildError, match="self-intersects"):
        build_route(trace, lanes)


def test_curvature_limit_rejects_unsafe_candidate():
    trace, lanes = circle()
    with pytest.raises(RouteBuildError, match="sharper"):
        build_route(trace, lanes, {"max_curvature_1pm": 0.04})


def test_speed_lateral_and_longitudinal_limits():
    # An ellipse produces different curve radii and verifies seam-aware deceleration.
    angle = np.linspace(0.0, 2.0 * math.pi, 601)
    center = np.column_stack((15.0 * np.cos(angle), 8.0 * np.sin(angle)))
    tangent = np.column_stack((-15.0 * np.sin(angle), 8.0 * np.cos(angle)))
    yaw = np.arctan2(tangent[:, 1], tangent[:, 0])
    trace = np.column_stack((center, yaw))
    dense = np.linspace(0.0, 2.0 * math.pi, 3600, endpoint=False)
    lane_center = np.column_stack((15.0 * np.cos(dense), 8.0 * np.sin(dense)))
    tangent = np.column_stack((-15.0 * np.sin(dense), 8.0 * np.cos(dense)))
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    lanes = np.vstack((lane_center + 1.2 * normal, lane_center - 1.2 * normal))
    result = build_route(trace, lanes, {"max_speed_mps": 3.0, "max_lateral_accel_mps2": 0.3})
    speeds = np.array([sample["speed_mps"] for sample in result["route_samples"]])
    curvature = np.array([sample["curvature_1pm"] for sample in result["route_samples"]])
    route = xy(result)
    ds = np.linalg.norm(np.roll(route, -1, axis=0) - route, axis=1)
    acceleration = (np.roll(speeds, -1) ** 2 - speeds ** 2) / (2.0 * ds)
    assert np.ptp(speeds) > 0.1
    assert np.all(speeds ** 2 * np.abs(curvature) <= 0.3 + 1e-8)
    assert np.max(acceleration) <= 0.4 + 1e-8
    assert np.min(acceleration) >= -0.5 - 1e-8


@pytest.mark.parametrize("config", [
    {"unknown": 1}, {"sample_spacing_m": 0}, {"max_speed_mps": float("nan")},
    {"min_support_fraction": 1.2}, {"min_edge_points": 2.2},
    {"max_speed_mps": True}, {"vehicle_half_width_m": 0.6},
])
def test_invalid_configuration_fails_closed(config):
    trace, lanes = circle()
    with pytest.raises(RouteBuildError):
        build_route(trace, lanes, config)


def test_nonfinite_input_rejected():
    trace, lanes = circle()
    lanes[0, 0] = float("nan")
    with pytest.raises(RouteBuildError, match="non-finite"):
        build_route(trace, lanes)


def test_nonmapping_configuration_fails_closed():
    trace, lanes = circle()
    with pytest.raises(RouteBuildError, match="mapping"):
        build_route(trace, lanes, [])


def test_duplicate_points_do_not_create_lane_support():
    trace, lanes = circle()
    sparse = lanes[::30]
    repeated = np.repeat(sparse, 5, axis=0)
    with pytest.raises(RouteBuildError, match="missing"):
        build_route(trace, repeated)


def test_duplicate_stationary_samples_are_removed():
    trace, lanes = circle()
    trace = np.insert(trace, 50, np.repeat(trace[49:50], 5, axis=0), axis=0)
    result = build_route(trace, lanes)
    assert result["diagnostics"]["valid"]


def test_sample_capacity_gate_is_explicit():
    trace, lanes = circle()
    with pytest.raises(RouteBuildError, match="max_route_samples"):
        build_route(trace, lanes, {"max_route_samples": 50})
