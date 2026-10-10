"""Independent contracts for current-mask line geometry, not vehicle evidence.

Synthetic points isolate geometry/transaction failures; they are not a replay,
parameter calibration, or proof of a completed lap. Only ordinary unit-test wall
clock allowance is enlarged, as in test_mask_localization.py. Matching, residual,
observability, innovation, vote, and capacity gates retain production defaults.
"""

import math

import numpy as np
import pytest

from lane_mapping_lya_reference.mask_localization import (
    LaneMapLocalizer, _FrozenIndex, _configuration, _voxel_means,
)


def horizontal(y, start=0.5, stop=8.0, spacing=0.05):
    x = np.arange(start, stop + spacing * 0.25, spacing)
    return np.column_stack((x, np.full(len(x), y)))


def rectangle(x0, x1, y0, y1, spacing=0.05):
    x, y = np.meshgrid(np.arange(x0, x1 + spacing * 0.25, spacing),
                       np.arange(y0, y1 + spacing * 0.25, spacing))
    return np.column_stack((x.ravel(), y.ravel()))


def lane_scene():
    anchor = np.vstack([horizontal(y, -4.0, 12.0) for y in (-1.8, 1.8)])
    scan = np.vstack([horizontal(y) for y in (-1.8, 1.8)])
    return anchor, scan


def localizer(anchor, **config):
    return LaneMapLocalizer(anchor, dict(runtime_budget_ms=2000.0, **config))


def committed_snapshot(matcher):
    points, votes = matcher.refined_points_and_votes()
    return (matcher.correction_se2, matcher.last_accepted_stamp_ns,
            matcher.refinement_metrics(), points, votes, matcher.anchor_points)


def assert_committed_unchanged(matcher, snapshot):
    correction, stamp, metrics, points, votes, anchor = snapshot
    np.testing.assert_array_equal(matcher.correction_se2, correction)
    assert matcher.last_accepted_stamp_ns == stamp
    assert matcher.refinement_metrics() == metrics
    np.testing.assert_array_equal(matcher.refined_points_and_votes()[0], points)
    np.testing.assert_array_equal(matcher.refined_points_and_votes()[1], votes)
    np.testing.assert_array_equal(matcher.anchor_points, anchor)


@pytest.mark.parametrize("reliable_depth", [0.10, 0.30, 0.60])
def test_complete_support_keeps_clipped_bar_front_at_the_same_pose(reliable_depth):
    """A reliability crop must not rotate a long bar's geometric normal.

    Both lane lines and the bar are observed in this same synthetic frame. The
    bar's near end is fixed; only how much of it is reliable for fitting changes.
    Full stable support is not allowed to become extra positional observations.
    """
    anchor, lanes = lane_scene()
    bar = rectangle(4.0, 7.0, -0.20, 0.20)
    support = np.vstack((lanes, bar))
    reliable_bar = bar[bar[:, 0] <= 4.0 + reliable_depth + 1e-9]
    reliable = np.vstack((lanes, reliable_bar))
    matcher = localizer(np.vstack((anchor, bar)))
    result = matcher.match(reliable, [0.0, 0.12, 0.015], 1,
                           scan_support_xy=support)
    assert result.accepted, (result.reason, result.metrics)
    assert abs(result.pose_xyyaw[0]) < 0.01
    assert abs(result.pose_xyyaw[1]) < 0.03
    assert abs(result.pose_xyyaw[2]) < 0.005
    assert result.metrics["observable_rank"] == 2
    assert not result.metrics["longitudinal_observable"]
    assert result.metrics["input_scan_points"] == len(reliable)
    assert result.metrics["scan_support_points"] > result.metrics["scan_points"]
    assert result.metrics["inliers"] <= result.metrics["scan_line_points"]


def test_disconnected_crosswalk_bars_never_supply_each_others_pca_support():
    cfg = _configuration(None)
    bars = [rectangle(3.0, 5.0, center - 0.10, center + 0.10)
            for center in (-1.0, 0.0, 1.0)]
    support = _voxel_means(np.vstack(bars), cfg["scan_voxel_m"])
    index = _FrozenIndex(support, cfg)
    assert len(np.unique(index.components)) == 3
    # At the three separated near ends, multiscale support must follow each
    # actual long bar, never the apparent transverse line between their ends.
    front = support[:, 0] < 3.15
    valid = front & index.normal_valid
    assert np.count_nonzero(valid) >= 3
    assert np.all(np.abs(index.normals[valid, 1]) > 0.95)
    assert np.all(np.abs(index.line_points[valid, 0] - support[valid, 0]) < 0.03)


def test_isotropic_blob_does_not_swallow_valid_line_overlap():
    anchor, lanes = lane_scene()
    baseline = localizer(anchor).match(lanes, [0.0, 0.15, 0.02], 1)
    assert baseline.accepted, baseline.reason
    # This region is separate from the real lines and has no stable local line
    # direction. It dominates raw point count but must not be called line error.
    blob = rectangle(12.0, 15.0, -1.5, 1.5, spacing=0.10)
    scan = np.vstack((lanes, blob))
    result = localizer(anchor).match(scan, [0.0, 0.15, 0.02], 1,
                                     scan_support_xy=scan)
    assert result.accepted, (result.reason, result.metrics)
    assert result.metrics["scan_geometric_rejections"] > 0
    assert result.metrics["scan_line_points"] < result.metrics["scan_points"]
    assert result.metrics["overlap_ratio"] >= 0.95
    np.testing.assert_allclose(result.pose_xyyaw, baseline.pose_xyyaw, atol=0.015)


def test_mostly_unmatched_good_lines_remain_in_overlap_denominator():
    anchor = horizontal(0.0, -4.0, 12.0)
    scan = np.vstack([horizontal(y) for y in (0.0, -3.0, 3.0)])
    matcher = localizer(anchor)
    before = committed_snapshot(matcher)
    result = matcher.match(scan, [0.0, 0.0, 0.0], 1, scan_support_xy=scan)
    assert not result.accepted, result.metrics
    assert result.reason == "insufficient_overlap"
    assert result.metrics["scan_line_points"] >= 0.9 * result.metrics["scan_points"]
    assert result.metrics["overlap_ratio"] < matcher.config["min_overlap_ratio"]
    assert_committed_unchanged(matcher, before)


def test_line_eligibility_is_independent_of_anchor_overlap():
    anchor, lanes = lane_scene()
    blob = rectangle(12.0, 15.0, -1.5, 1.5, spacing=0.10)
    scan = np.vstack((lanes, blob))
    matched = localizer(anchor).match(scan, [0.0, 0.0, 0.0], 1,
                                      scan_support_xy=scan)
    unrelated = localizer(anchor + [0.0, 10.0]).match(
        scan, [0.0, 0.0, 0.0], 1, scan_support_xy=scan)
    assert matched.accepted, matched.reason
    assert not unrelated.accepted and unrelated.reason == "insufficient_overlap"
    for field in ("scan_points", "scan_support_points", "scan_geometric_rejections",
                  "scan_line_points"):
        assert matched.metrics[field] == unrelated.metrics[field]
    assert unrelated.metrics["scan_line_points"] >= 30


def test_duplicate_matching_pixels_cannot_hide_majority_unmatched_lines():
    anchor = horizontal(0.0, -4.0, 12.0)
    good = horizontal(0.0)
    wrong = np.vstack([horizontal(y) for y in (-3.0, 3.0)])
    sparse = np.vstack((good, wrong))
    duplicated = np.vstack((np.repeat(good, 40, axis=0), wrong))
    reference = localizer(anchor).match(sparse, [0.0, 0.0, 0.0], 1,
                                        scan_support_xy=sparse)
    result = localizer(anchor).match(duplicated, [0.0, 0.0, 0.0], 1,
                                     scan_support_xy=duplicated)
    assert not reference.accepted and not result.accepted
    assert result.reason == "insufficient_overlap"
    assert result.metrics["overlap_ratio"] < 0.55
    assert abs(result.metrics["overlap_ratio"] - reference.metrics["overlap_ratio"]) < 0.02


def test_wider_matching_band_cannot_outvote_two_unmatched_equal_length_lines():
    """Independent longitudinal support, not white-pixel area, sets overlap.

    The correct band has several occupied normal-direction voxels. Both wrong
    lines have the same longitudinal span and are independently good geometry.
    Raw inlier count can be a majority here; independent supported length is not.
    """
    anchor = horizontal(0.0, -4.0, 12.0)
    good_band = rectangle(0.5, 8.0, -0.10, 0.10)
    wrong = np.vstack([horizontal(y) for y in (-3.0, 3.0)])
    scan = np.vstack((good_band, wrong))
    matcher = localizer(anchor)
    before = committed_snapshot(matcher)
    result = matcher.match(scan, [0.0, 0.0, 0.0], 1, scan_support_xy=scan)
    assert not result.accepted, result.metrics
    assert result.reason in ("insufficient_overlap", "insufficient_final_inliers")
    assert result.metrics["overlap_ratio"] < matcher.config["min_overlap_ratio"]
    assert_committed_unchanged(matcher, before)


def test_mostly_normal_incompatible_good_lines_are_not_removed_from_denominator():
    anchor = np.vstack([horizontal(y, -4.0, 12.0) for y in (0.0, 1.5)])
    good = horizontal(1.5)
    y = np.arange(-0.50, 0.501, 0.05)
    wrong = np.vstack([np.column_stack((np.full(len(y), x), y))
                       for x in np.arange(0.7, 8.01, 0.7)])
    # Wrong vertical lines are geometrically valid and close to the y=0 anchor,
    # but their normals are incompatible. They must count as failed evidence.
    scan = np.vstack((good, wrong))
    matcher = localizer(anchor)
    result = matcher.match(scan, [0.0, 0.0, 0.0], 1, scan_support_xy=scan)
    assert not result.accepted, result.metrics
    assert result.reason == "insufficient_overlap"
    assert result.metrics["scan_line_points"] > 1.5 * len(_voxel_means(good, 0.10))
    assert result.metrics["overlap_ratio"] < matcher.config["min_overlap_ratio"]


def test_nearest_search_chooses_compatible_candidate_not_nearest_then_discard():
    horizontal_line = horizontal(0.45, -2.0, 2.0)
    y = np.arange(-2.0, 0.001, 0.05)
    vertical_line = np.column_stack((np.zeros(len(y)), y))
    matcher = localizer(np.vstack((horizontal_line, vertical_line)))
    query = np.array([[0.0, 0.03]])
    geometric = matcher._index.nearest(query)[0]
    positive = matcher._index.nearest(query, np.array([[0.0, 1.0]]))[0]
    negative = matcher._index.nearest(query, np.array([[0.0, -1.0]]))[0]
    assert geometric >= 0 and positive >= 0
    assert abs(matcher.anchor_points[geometric, 1]) < 0.10
    assert matcher.anchor_points[positive, 1] == pytest.approx(0.45)
    assert positive == negative
    assert matcher._index.nearest(np.array([[0.0, 3.0]]),
                                 np.array([[0.0, 1.0]]))[0] == -1


@pytest.mark.parametrize("heading", [0.7, -1.2])
def test_scan_normals_rotate_with_vehicle_not_with_world_axes(heading):
    anchor, scan = lane_scene()
    rotation = np.array([[math.cos(heading), -math.sin(heading)],
                         [math.sin(heading), math.cos(heading)]])
    origin = np.array([50.0, -20.0])
    normal = rotation @ np.array([0.0, 1.0])
    prior = [*(origin + 0.15 * normal), heading + 0.02]
    result = localizer(anchor @ rotation.T + origin).match(
        scan, prior, 1, scan_support_xy=scan)
    assert result.accepted, (result.reason, result.metrics)
    assert abs((result.pose_xyyaw[:2] - origin) @ normal) < 0.015
    assert abs(result.pose_xyyaw[2] - heading) < 0.004


def test_scan_support_capacity_rejection_preserves_committed_state():
    anchor, scan = lane_scene()
    matcher = localizer(anchor, max_scan_support_points=600)
    assert matcher.match(scan, [0.0, 0.10, 0.0], 1).accepted
    before = committed_snapshot(matcher)
    too_large = np.vstack((scan, rectangle(10.0, 14.0, -2.0, 2.0, spacing=0.10)))
    result = matcher.match(scan, [0.0, 0.10, 0.0], 2, scan_support_xy=too_large)
    assert not result.accepted and result.reason == "scan_geometry_capacity"
    assert result.metrics["scan_support_points"] > 600
    assert not matcher.commit(result)
    assert_committed_unchanged(matcher, before)


@pytest.mark.parametrize("support", [np.array([[np.nan, 0.0]]),
                                     np.zeros((10, 3)),
                                     np.zeros((200001, 2))])
def test_malformed_or_oversized_support_is_rejected_without_committing(support):
    anchor, scan = lane_scene()
    matcher = localizer(anchor)
    assert matcher.match(scan, [0.0, 0.10, 0.0], 1).accepted
    before = committed_snapshot(matcher)
    result = matcher.match(scan, [0.0, 0.10, 0.0], 2, scan_support_xy=support)
    assert not result.accepted and result.reason == "invalid_input"
    assert_committed_unchanged(matcher, before)


@pytest.mark.parametrize("stamp,prior_stamp,reason", [
    (100, 90, "nonincreasing_mask_stamp"),
    (99, 90, "nonincreasing_mask_stamp"),
    (200, 201, "future_or_invalid_prior_stamp"),
])
def test_full_support_does_not_bypass_mask_or_prior_timestamp_fences(stamp, prior_stamp, reason):
    anchor, scan = lane_scene()
    matcher = localizer(anchor)
    assert matcher.match(scan, [0.0, 0.10, 0.0], 100, scan_support_xy=scan).accepted
    before = committed_snapshot(matcher)
    result = matcher.match(scan, [0.0, 0.10, 0.0], stamp,
                           prior_stamp_ns=prior_stamp, scan_support_xy=scan)
    assert not result.accepted and result.reason == reason
    assert_committed_unchanged(matcher, before)


def test_support_only_points_never_become_fit_or_refinement_votes():
    anchor, support = lane_scene()
    reliable = support[support[:, 0] <= 4.0 + 1e-9]
    matcher = localizer(anchor)
    original = committed_snapshot(matcher)
    proposal = matcher.match(reliable, [0.0, 0.10, 0.0], 1,
                             scan_support_xy=support, commit=False)
    assert proposal.accepted, (proposal.reason, proposal.metrics)
    assert_committed_unchanged(matcher, original)
    assert matcher.commit(proposal)
    assert not matcher.commit(proposal)
    for stamp in (2, 3):
        result = matcher.match(reliable, [0.0, 0.10, 0.0], stamp,
                               scan_support_xy=support)
        assert result.accepted, (result.reason, result.metrics)
    points, votes = matcher.refined_points_and_votes()
    assert len(points) > 0 and np.all(votes >= 3)
    assert np.max(points[:, 0]) <= 4.10
    assert np.min(points[:, 0]) >= 0.40
    assert matcher.refinement_metrics()["refinement_retained_frames"] == 3
    np.testing.assert_array_equal(matcher.anchor_points, original[-1])


def test_rejected_geometry_revokes_an_older_uncommitted_proposal():
    anchor, scan = lane_scene()
    matcher = localizer(anchor)
    before = committed_snapshot(matcher)
    proposal = matcher.match(scan, [0.0, 0.10, 0.0], 1,
                             scan_support_xy=scan, commit=False)
    assert proposal.accepted
    rejected = matcher.match(scan, [0.0, 0.10, 0.0], 2,
                              scan_support_xy=np.empty((0, 2)), commit=False)
    assert not rejected.accepted
    assert not matcher.commit(proposal)
    assert not matcher.commit(rejected)
    assert_committed_unchanged(matcher, before)


def test_geometry_preparation_still_obeys_runtime_deadline():
    anchor, scan = lane_scene()
    matcher = LaneMapLocalizer(anchor, {"runtime_budget_ms": 0.000001})
    before = committed_snapshot(matcher)
    result = matcher.match(scan, [0.0, 0.10, 0.0], 1, scan_support_xy=scan)
    assert not result.accepted
    assert result.reason == "runtime_budget_exceeded"
    assert result.metrics["budget_exceeded"]
    assert_committed_unchanged(matcher, before)


@pytest.mark.parametrize("bad_kind", [
    "nan", "infinity", "negative", "short", "long", "matrix", "scalar", "text", "empty",
])
def test_invalid_scan_sensitivity_cannot_change_committed_geometry(bad_kind):
    anchor, scan = lane_scene()
    matcher = localizer(anchor)
    assert matcher.match(scan, [0.0, 0.10, 0.0], 1).accepted
    before = committed_snapshot(matcher)
    sensitivity = np.full(len(scan), 0.02)
    if bad_kind in ("nan", "infinity", "negative"):
        sensitivity[10] = {"nan": np.nan, "infinity": np.inf, "negative": -0.01}[bad_kind]
    elif bad_kind == "short":
        sensitivity = sensitivity[:-1]
    elif bad_kind == "long":
        sensitivity = np.append(sensitivity, 0.02)
    elif bad_kind == "matrix":
        sensitivity = sensitivity[:, None]
    elif bad_kind == "scalar":
        sensitivity = 0.02
    elif bad_kind == "text":
        sensitivity = ["not a sensitivity"] * len(scan)
    else:
        sensitivity = np.empty(0)
    result = matcher.match(scan, [0.0, 0.10, 0.0], 2, scan_support_xy=scan,
                           scan_sensitivity=sensitivity, commit=False)
    assert not result.accepted and result.reason == "invalid_input"
    assert not matcher.commit(result)
    # Seen-stamp rejection fences may advance; committed pose/map/votes may not.
    assert_committed_unchanged(matcher, before)


@pytest.mark.parametrize("uniform_sensitivity", [0.0, 0.03, 0.10])
@pytest.mark.parametrize("mostly_wrong", [False, True])
def test_uniform_sensitivity_preserves_the_no_sensitivity_decision(uniform_sensitivity,
                                                                 mostly_wrong):
    if mostly_wrong:
        anchor = horizontal(0.0, -4.0, 12.0)
        scan = np.vstack([horizontal(y) for y in (0.0, -3.0, 3.0)])
        prior = [0.0, 0.0, 0.0]
    else:
        anchor, scan = lane_scene()
        prior = [0.0, 0.12, 0.015]
    baseline = localizer(anchor).match(scan, prior, 1, scan_support_xy=scan)
    weighted = localizer(anchor).match(scan, prior, 1, scan_support_xy=scan,
                                       scan_sensitivity=np.full(len(scan), uniform_sensitivity))
    assert weighted.accepted == baseline.accepted
    assert weighted.reason == baseline.reason
    assert weighted.accepted is not mostly_wrong
    np.testing.assert_allclose(weighted.pose_xyyaw, baseline.pose_xyyaw, atol=1e-10)
    np.testing.assert_allclose(weighted.correction_se2, baseline.correction_se2, atol=1e-10)
    for field in ("overlap_ratio", "inlier_ratio", "scan_feature_weight_sum"):
        assert weighted.metrics[field] == pytest.approx(baseline.metrics[field], abs=1e-10)
    assert weighted.metrics["sensitivity_is_calibrated_covariance"] is False


@pytest.mark.parametrize("support_sensitivity", [0.20, 0.50])
def test_high_sensitivity_direction_support_never_becomes_a_positional_vote(support_sensitivity):
    anchor, support = lane_scene()
    # Mirror the caller's current-frame sensitivity selection explicitly. The
    # localizer receives J only for the reliable subset; high-J support supplies
    # directions, never extra positional measurements or refinement votes.
    all_sensitivity = np.where(support[:, 0] <= 4.0 + 1e-9, 0.02, support_sensitivity)
    reliable = all_sensitivity <= 0.10
    positions = support[reliable].copy()
    sensitivity = all_sensitivity[reliable].copy()
    positions_before, support_before, sensitivity_before = positions.copy(), support.copy(), sensitivity.copy()
    matcher = localizer(anchor)
    for stamp in (1, 2, 3):
        result = matcher.match(positions, [0.0, 0.10, 0.0], stamp,
                               scan_support_xy=support, scan_sensitivity=sensitivity)
        assert result.accepted, (result.reason, result.metrics)
        assert result.metrics["input_scan_points"] == len(positions)
        assert result.metrics["scan_support_points"] > result.metrics["scan_points"]
        assert result.metrics["inliers"] <= result.metrics["scan_line_points"]
    points, votes = matcher.refined_points_and_votes()
    assert len(points) > 0 and np.all(votes >= 3)
    assert np.max(points[:, 0]) <= 4.10
    assert matcher.refinement_metrics()["refinement_retained_frames"] == 3
    np.testing.assert_array_equal(positions, positions_before)
    np.testing.assert_array_equal(support, support_before)
    np.testing.assert_array_equal(sensitivity, sensitivity_before)


@pytest.mark.parametrize("matching_pixel_repeats", [1, 25])
@pytest.mark.parametrize("good_j,wrong_j", [
    pytest.param(0.09, 0.01, id="near_wrong_lines_are_more_reliable"),
    pytest.param(0.03, 0.03, id="equal_reliability"),
    pytest.param(0.01, 0.09, id="wrong_lines_still_inside_reliable_cut"),
])
def test_majority_reliable_wrong_lines_cannot_be_hidden_by_j_or_pixel_density(
        matching_pixel_repeats, good_j, wrong_j):
    """All three equal-length features are valid and below the reliable cut.

    No pose-dependent selection may remove the two wrong lines. Repeat pixels
    and projection-J weighting must not turn minority physical support into a
    successful localization certificate. This is a safety contract, not a test
    of a particular weighting formula or an assumption of raw-count overlap.
    """
    anchor = horizontal(0.0, -4.0, 12.0)
    good = np.repeat(rectangle(0.5, 8.0, -0.10, 0.10), matching_pixel_repeats, axis=0)
    wrong = np.vstack([horizontal(y) for y in (-3.0, 3.0)])
    scan = np.vstack((good, wrong))
    sensitivity = np.concatenate((np.full(len(good), good_j), np.full(len(wrong), wrong_j)))
    assert np.all((sensitivity >= 0.0) & (sensitivity <= 0.10))
    matcher = localizer(anchor)
    before = committed_snapshot(matcher)
    result = matcher.match(scan, [0.0, 0.0, 0.0], 1, scan_support_xy=scan,
                           scan_sensitivity=sensitivity)
    assert not result.accepted, (result.pose_xyyaw, result.metrics)
    assert_committed_unchanged(matcher, before)
