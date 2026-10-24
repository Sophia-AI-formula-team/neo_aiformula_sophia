"""Frozen-map causal matching, degeneracy and two-stage commit regressions."""

import json
import math

import numpy as np
import pytest

from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer
from lane_mapping_lya_reference.mapping_core import (
    GroundLookup, SparseConsensusMap, make_transform, transform_local_points,
)


def straight_scene(translation=(0.0, 0.0), heading=0.0):
    anchor_x = np.arange(-15.0, 20.01, 0.05)
    anchor = np.vstack([np.column_stack((anchor_x, np.full(len(anchor_x), side)))
                        for side in (-1.2, 1.2)])
    observed_x = np.arange(0.5, 8.01, 0.06)
    scan = np.vstack([np.column_stack((observed_x, np.full(len(observed_x), side)))
                      for side in (-1.2, 1.2)])
    cosine, sine = math.cos(heading), math.sin(heading)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return anchor @ rotation.T + translation, scan


def make_localizer(anchor, **config):
    # Ordinary unit tests do not assert host-specific wall-clock performance.
    return LaneMapLocalizer(anchor, dict(runtime_budget_ms=2000.0, **config))


def test_parallel_lane_corrects_lateral_and_yaw_not_longitudinal():
    anchor, scan = straight_scene()
    localizer = make_localizer(anchor)
    result = localizer.match(scan, [0.7, 0.30, 0.04], 100, prior_stamp_ns=90)
    assert result.accepted, result.metrics
    forward = np.array([math.cos(0.04), math.sin(0.04)])
    assert np.dot(result.pose_xyyaw[:2] - [0.7, 0.3], forward) == pytest.approx(0.0, abs=1e-8)
    assert result.pose_xyyaw[1] == pytest.approx(0.0, abs=0.012)
    assert result.pose_xyyaw[2] == pytest.approx(0.0, abs=0.003)
    assert not result.metrics["longitudinal_observable"]
    assert result.metrics["observable_rank"] == 2
    assert result.metrics["innovation_along_lane_m"] == pytest.approx(0.0, abs=1e-8)
    assert result.metrics["rmse_after_m"] < result.metrics["rmse_before_m"]
    json.dumps(result.metrics, allow_nan=False)


def test_parallel_degeneracy_respects_rotated_global_lane_direction():
    anchor, scan = straight_scene((20.0, -10.0), heading=0.7)
    normal = np.array([-math.sin(0.7), math.cos(0.7)])
    tangent = np.array([math.cos(0.7), math.sin(0.7)])
    true_xy = np.array([20.0, -10.0])
    prior_xy = true_xy + 0.25 * normal + 0.6 * tangent
    result = make_localizer(anchor).match(scan, [*prior_xy, 0.73], 100)
    assert result.accepted, result.metrics
    prior_forward = np.array([math.cos(0.73), math.sin(0.73)])
    assert np.dot(result.pose_xyyaw[:2] - prior_xy, prior_forward) == pytest.approx(0.0, abs=1e-7)
    assert np.dot(result.pose_xyyaw[:2] - true_xy, normal) == pytest.approx(0.0, abs=0.012)


def test_se2_translation_is_not_wrongly_bounded_at_map_origin():
    anchor, scan = straight_scene((50.0, 50.0))
    localizer = make_localizer(anchor)
    result = localizer.match(scan, [50.0, 50.2, 0.07], 100)
    assert result.accepted, result.metrics
    assert np.linalg.norm(result.correction_se2[:2]) > 2.0
    assert result.metrics["total_translation_at_vehicle_m"] < 0.3
    np.testing.assert_allclose(localizer.correct([50.0, 50.2, 0.07]), result.pose_xyyaw, atol=1e-12)


def test_initial_acquisition_can_correct_point_three_meters_without_clamping():
    anchor, scan = straight_scene()
    localizer = make_localizer(anchor)
    first = localizer.match(scan, [0.0, 0.3, 0.0], 100)
    assert first.accepted
    assert first.metrics["initial_acquisition"]
    second = localizer.match(scan, [0.0, 0.6, 0.0], 200)
    assert not second.accepted
    assert second.reason == "tracking_innovation_gate"
    assert localizer.last_accepted_stamp_ns == 100


def test_frozen_anchor_is_copied_and_never_updated_from_refinement():
    anchor, scan = straight_scene()
    localizer = make_localizer(anchor)
    frozen = localizer.anchor_points
    anchor[:] = 999.0
    for stamp in range(1, 6):
        assert localizer.match(scan, [0.0, 0.1, 0.0], stamp).accepted
    points, votes = localizer.refined_points_and_votes()
    assert len(points) > 0 and np.min(votes) >= 3
    np.testing.assert_array_equal(localizer.anchor_points, frozen)
    points[:] = 999.0
    assert np.max(localizer.refined_points_and_votes()[0]) < 100.0


def test_failed_match_does_not_update_any_refinement_or_correction():
    anchor, scan = straight_scene()
    localizer = make_localizer(anchor)
    assert localizer.match(scan, [0.0, 0.2, 0.0], 100).accepted
    before = localizer.refinement_metrics()
    correction = localizer.correction_se2
    rejected = localizer.match(scan + [0.0, 30.0], [0.0, 0.2, 0.0], 200)
    assert not rejected.accepted
    assert localizer.refinement_metrics() == before
    np.testing.assert_array_equal(localizer.correction_se2, correction)


def test_proposal_must_be_committed_after_runtime_freshness_review():
    anchor, scan = straight_scene()
    localizer = make_localizer(anchor)
    result = localizer.match(scan, [0.0, 0.3, 0.0], 100, commit=False)
    assert result.accepted
    assert localizer.last_accepted_stamp_ns is None
    assert localizer.refinement_metrics()["refinement_retained_frames"] == 0
    np.testing.assert_array_equal(localizer.correct([0.0, 0.3, 0.0]), [0.0, 0.3, 0.0])
    assert localizer.commit(result)
    assert not localizer.commit(result)
    assert localizer.last_accepted_stamp_ns == 100
    assert localizer.refinement_metrics()["refinement_retained_frames"] == 1


def test_dropped_or_superseded_proposal_never_bootstraps_next_fit():
    anchor, scan = straight_scene()
    localizer = make_localizer(anchor)
    first = localizer.match(scan, [0.0, 0.3, 0.0], 100, commit=False)
    second = localizer.match(scan, [0.0, 0.3, 0.0], 200, commit=False)
    assert second.metrics["initial_acquisition"]
    assert not localizer.commit(first)
    assert localizer.commit(second)
    assert localizer.refinement_metrics()["refinement_retained_frames"] == 1


def test_other_localizer_cannot_commit_a_foreign_proposal():
    anchor, scan = straight_scene()
    first, second = make_localizer(anchor), make_localizer(anchor)
    result = first.match(scan, [0.0, 0.0, 0.0], 100, commit=False)
    assert not second.commit(result)
    assert second.last_accepted_stamp_ns is None


@pytest.mark.parametrize("stamp", [0, -1, 1.5, True])
def test_invalid_stamp_rejected(stamp):
    anchor, scan = straight_scene()
    result = make_localizer(anchor).match(scan, [0.0, 0.0, 0.0], stamp)
    assert not result.accepted and result.reason == "invalid_mask_stamp"
    json.dumps(result.metrics, allow_nan=False)


def test_out_of_order_duplicate_and_future_prior_are_not_votes():
    anchor, scan = straight_scene()
    localizer = make_localizer(anchor)
    assert localizer.match(scan, [0.0, 0.0, 0.0], 100).accepted
    for stamp in (100, 99):
        assert localizer.match(scan, [0.0, 0.0, 0.0], stamp).reason == "nonincreasing_mask_stamp"
    future = localizer.match(scan, [0.0, 0.0, 0.0], 200, prior_stamp_ns=201)
    assert future.reason == "future_or_invalid_prior_stamp"
    assert localizer.refinement_metrics()["refinement_retained_frames"] == 1


def test_empty_anchor_and_no_overlap_fail_closed():
    _, scan = straight_scene()
    assert make_localizer(np.empty((0, 2))).match(scan, [0, 0, 0], 1).reason == "empty_anchor"
    anchor, _ = straight_scene()
    result = make_localizer(anchor).match(scan, [0, 20, 0], 1)
    assert not result.accepted and result.reason == "insufficient_overlap"


def test_isotropic_thick_patch_is_not_a_strong_lane_normal():
    grid = np.arange(-3.0, 3.01, 0.05)
    x, y = np.meshgrid(grid, grid)
    anchor = np.column_stack((x.ravel(), y.ravel()))
    result = make_localizer(anchor).match(anchor[::20], [0, 0, 0], 1)
    assert not result.accepted


def test_gross_random_clutter_does_not_get_accepted_as_overlap():
    anchor, scan = straight_scene()
    random = np.random.default_rng(42)
    clutter = random.uniform([0.0, -8.0], [10.0, 8.0], size=(600, 2))
    result = make_localizer(anchor).match(np.vstack((scan[:10], clutter)), [0, 0, 0], 1)
    assert not result.accepted


def test_minority_outliers_do_not_dominate_parallel_fit():
    anchor, scan = straight_scene()
    random = np.random.default_rng(42)
    noisy = scan + random.normal(0.0, 0.012, scan.shape)
    clutter = random.uniform([0.0, -6.0], [9.0, 6.0], size=(30, 2))
    result = make_localizer(anchor).match(np.vstack((noisy, clutter)), [0, 0.25, 0.025], 1)
    assert result.accepted, (result.reason, result.metrics)
    assert abs(result.pose_xyyaw[1]) < 0.025
    assert abs(result.pose_xyyaw[2]) < 0.006


def test_runtime_budget_rejection_has_no_side_effects():
    anchor, scan = straight_scene()
    localizer = LaneMapLocalizer(anchor, {"runtime_budget_ms": 0.000001})
    result = localizer.match(scan, [0, 0.2, 0], 1)
    assert not result.accepted
    assert result.metrics["budget_exceeded"]
    assert result.reason == "runtime_budget_exceeded"
    assert localizer.last_accepted_stamp_ns is None
    assert localizer.refinement_metrics()["refinement_retained_frames"] == 0


def test_refinement_capacities_and_duplicate_pixel_votes_are_bounded():
    anchor, scan = straight_scene()
    localizer = make_localizer(anchor, refinement_max_cells=30, refinement_max_frames=4,
                               refinement_min_frame_votes=3)
    scan = np.repeat(scan, 5, axis=0)
    first = localizer.match(scan, [0, 0, 0], 1)
    assert first.accepted
    assert len(localizer.refined_points_and_votes()[0]) == 0
    for stamp in range(2, 12):
        assert localizer.match(scan, [0, 0, 0], stamp).accepted
    statistics = localizer.refinement_metrics()
    assert statistics["refinement_candidate_cells"] <= 30
    assert statistics["refinement_retained_frames"] <= 4
    assert statistics["refinement_capacity_rejected_cells"] > 0
    assert np.max(localizer.refined_points_and_votes()[1]) > 4


def test_reset_clears_dynamic_state_not_frozen_anchor():
    anchor, scan = straight_scene()
    localizer = make_localizer(anchor)
    frozen = localizer.anchor_points
    assert localizer.match(scan, [0, 0.2, 0], 100).accepted
    localizer.reset()
    assert localizer.last_accepted_stamp_ns is None
    assert localizer.refinement_metrics()["refinement_retained_frames"] == 0
    np.testing.assert_array_equal(localizer.correction_se2, [0, 0, 0])
    np.testing.assert_array_equal(localizer.anchor_points, frozen)


@pytest.mark.parametrize("config", [
    {"unknown": 1}, {"runtime_budget_ms": float("nan")}, {"max_iterations": 2.5},
    {"max_scan_points": 5001}, {"max_anchor_points": True},
    {"min_overlap_ratio": 1.1}, {"hash_cell_m": 0.001},
    {"min_inliers": 1000}, {"normal_min_points": 2},
])
def test_bad_configuration_rejected(config):
    anchor, _ = straight_scene()
    with pytest.raises(ValueError):
        LaneMapLocalizer(anchor, config)


def test_anchor_capacity_is_rejected_not_silently_truncated():
    anchor, _ = straight_scene()
    with pytest.raises(ValueError, match="capacity"):
        LaneMapLocalizer(anchor, {"max_anchor_points": 20})


def test_confirmed_refinement_survives_rolling_window_expiry():
    anchor, scan = straight_scene()
    localizer = make_localizer(anchor, refinement_window_s=0.5)
    for stamp in (1000000000, 1100000000, 1200000000):
        assert localizer.match(scan, [0, 0, 0], stamp).accepted
    points, _ = localizer.refined_points_and_votes()
    assert len(points) > 0
    # New observations start a separate distant section after the recent window
    # expires. The confirmed first section must remain in the global refinement.
    assert localizer.match(scan, [10, 0, 0], 12000000000).accepted
    after, _ = localizer.refined_points_and_votes()
    assert set(map(tuple, points)).issubset(set(map(tuple, after)))


def test_persistent_refinement_overflow_stops_admission_without_erasing_history():
    anchor, scan = straight_scene()
    localizer = make_localizer(anchor, refinement_max_cells=160, refinement_window_s=0.5)
    for stamp in (1000000000, 1100000000, 1200000000):
        assert localizer.match(scan, [0, 0, 0], stamp).accepted
    before, _ = localizer.refined_points_and_votes()
    assert len(before) > 0
    for stamp in (10000000000, 10100000000, 10200000000):
        assert localizer.match(scan, [10, 0, 0], stamp).accepted
    after, _ = localizer.refined_points_and_votes()
    assert len(after) <= 160
    assert set(map(tuple, before)).issubset(set(map(tuple, after)))
    assert localizer.refinement_metrics()["refinement_growth_stopped"]
    assert localizer.refinement_metrics()["refinement_global_capacity_rejected_cells"] > 0


def test_scaled_lane_width_is_not_a_successful_fit():
    anchor, scan = straight_scene()
    scan[:, 1] *= 1.3
    result = make_localizer(anchor).match(scan, [0, 0, 0], 1)
    assert not result.accepted


def test_short_cross_section_cannot_separate_heading_from_lateral_motion():
    anchor, _ = straight_scene()
    scan = np.column_stack((np.linspace(4.999, 5.001, 40), np.full(40, 1.2)))
    result = make_localizer(anchor, scan_voxel_m=0.00001, min_scan_points=6,
                            min_inliers=6).match(scan, [0, 0, 0], 1)
    assert not result.accepted and result.reason == "unobservable_geometry"


def test_strong_geometry_only_corrects_longitudinal_when_explicitly_enabled():
    horizontal = np.arange(-2.0, 8.0, 0.05)
    vertical = np.arange(-2.0, 2.0, 0.05)
    anchor = np.vstack((np.column_stack((horizontal, np.full(len(horizontal), -2.0))),
                        np.column_stack((horizontal, np.full(len(horizontal), 2.0))),
                        np.column_stack((np.full(len(vertical), -2.0), vertical)),
                        np.column_stack((np.full(len(vertical), 8.0), vertical))))
    result = make_localizer(anchor, allow_longitudinal_correction=True).match(
        anchor.copy(), [0.20, -0.15, 0.03], 1)
    assert result.accepted, (result.reason, result.metrics)
    assert result.metrics["longitudinal_observable"]
    assert result.metrics["observable_rank"] == 3
    np.testing.assert_allclose(result.pose_xyyaw, [0, 0, 0], atol=0.012)


def test_multiscale_normals_recognize_accumulated_thick_lane_bands():
    x, thickness = np.meshgrid(np.arange(-10., 20.01, .1), np.arange(-.35, .351, .1))
    anchor = np.vstack([np.column_stack((x.ravel(), thickness.ravel() + side))
                        for side in (-1.2, 1.2)])
    _, scan = straight_scene()
    small = make_localizer(anchor, normal_max_radius_m=.45)
    adaptive = make_localizer(anchor)
    result = adaptive.match(scan, [0, .2, .02], 1)
    assert result.accepted, (result.reason, result.metrics)
    assert result.metrics["anchor_multiscale_line_points"] > 0
    assert np.sum(adaptive._index.normal_valid) > np.sum(small._index.normal_valid)
    assert abs(result.pose_xyyaw[1]) < .1
    assert result.metrics["anchor_line_thickness_p95_m"] <= .30
    assert adaptive.config["normal_max_variance_ratio"] == .15
    assert adaptive.config["min_overlap_ratio"] == .55


def test_multiscale_does_not_promote_isotropic_interior_to_lane_support():
    grid = np.arange(-3., 3.01, .1)
    x, y = np.meshgrid(grid, grid)
    anchor = np.column_stack((x.ravel(), y.ravel()))
    localizer = make_localizer(anchor)
    interior = np.linalg.norm(localizer.anchor_points, axis=1) < 1.0
    assert not np.any(localizer._index.normal_valid[interior])
    result = localizer.match(anchor[np.linalg.norm(anchor, axis=1) < .8], [0, 0, 0], 1)
    assert not result.accepted


def test_malformed_commit_does_not_change_state():
    anchor, _ = straight_scene()
    localizer = make_localizer(anchor)
    assert not localizer.commit(None)
    assert not localizer.commit({"accepted": True})


def _render_circle(lanes, pose, rng=None):
    difference = lanes - pose[:2]
    forward = math.cos(pose[2]) * difference[:, 0] + math.sin(pose[2]) * difference[:, 1]
    left = -math.sin(pose[2]) * difference[:, 0] + math.cos(pose[2]) * difference[:, 1]
    visible = forward > 1.0
    u = 320.0 - 254.0 * left[visible] / forward[visible]
    v = 180.0 + 254.0 / forward[visible]
    if rng is not None:
        u += rng.normal(0.0, 0.25, len(u))
        v += rng.normal(0.0, 0.25, len(v))
    u, v = np.rint(u).astype(int), np.rint(v).astype(int)
    visible = (u >= 0) & (u < 640) & (v >= 0) & (v < 360)
    mask = np.zeros((360, 640), dtype=np.uint8)
    mask[v[visible], u[visible]] = 255
    if rng is not None:
        mask[rng.integers(0, 360, 30), rng.integers(0, 640, 30)] = 255
    return mask


def test_full_circle_rendered_mask_ground_truth_does_not_drift_in_nullspace():
    """Regression for residual improving while pose RMSE grew 0.233 -> 0.321 m.

    Anchor comes only from a completed first lap of rendered masks, never from
    truth lane coordinates. Truth is used to render/evaluate an independent lap.
    This fixed fixture retains the noise, salt pixels and drift of the failure.
    """
    radius = 12.0
    angle = np.linspace(-math.pi / 2, 3 * math.pi / 2, 5001, endpoint=False)
    lanes = np.vstack([np.column_stack((r * np.cos(angle), radius + r * np.sin(angle)))
                       for r in (radius - 1.2, radius + 1.2)])
    lookup = GroundLookup(640, 360, [254., 0., 320., 0., 254., 180., 0., 0., 1.],
                          make_transform([0., 0., 1.], [-.5, .5, -.5, .5]), .5)
    mapper = SparseConsensusMap(.1, 5, 1, max_candidate_cells=30000, max_confirmed_cells=30000)
    for index, theta in enumerate(np.linspace(-math.pi / 2, 3 * math.pi / 2, 1001)):
        pose = np.array([radius * math.cos(theta), radius + radius * math.sin(theta), theta + math.pi / 2])
        local, sensitivity, _ = lookup.project_mask_with_sensitivity(_render_circle(lanes, pose), 127)
        world = transform_local_points(local, *pose)
        keys = mapper.points_to_unique_keys(world)
        reliable = mapper.points_to_unique_keys(world[sensitivity <= .1])
        mapper.update(keys, reliable, stamp_ns=1000000000 + index * 50000000)
    anchor, _ = mapper.confirmed_points_and_votes()
    localizer = make_localizer(anchor)
    rng = np.random.default_rng(2718)
    errors = []
    before = []
    accepted = 0
    for index, theta in enumerate(np.linspace(-math.pi / 2, 3 * math.pi / 2, 160, endpoint=False)):
        truth = np.array([radius * math.cos(theta), radius + radius * math.sin(theta), theta + math.pi / 2])
        prior = truth + [0.20, -0.12, 0.03]
        local, sensitivity, _ = lookup.project_mask_with_sensitivity(_render_circle(lanes, truth, rng), 127)
        result = localizer.match(local[sensitivity <= .1], prior, 60000000000 + index * 50000000)
        if result.accepted:
            accepted += 1
            assert abs(result.metrics["total_along_prior_forward_m"]) < 1e-8
        corrected = result.pose_xyyaw if result.accepted else localizer.correct(prior)
        errors.append(np.linalg.norm(corrected[:2] - truth[:2]))
        before.append(np.linalg.norm(prior[:2] - truth[:2]))
    assert accepted >= 150
    assert np.sqrt(np.mean(np.square(errors))) < np.sqrt(np.mean(np.square(before))) * 0.9
    assert np.max(errors) < 0.35
