"""Immutable, bounded tracking handoff; no ROS or driving acceptance implied."""

from dataclasses import FrozenInstanceError, fields
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from lane_mapping_lya_reference import mask_localization as localization
from lane_mapping_lya_reference.mask_localization import (
    CommittedLocalizationState, LaneMapLocalizer)


def lane_points():
    x = np.linspace(1., 7., 241)
    return np.vstack([np.column_stack((x, np.full(len(x), side))) for side in (-1., 1.)])


@pytest.fixture
def tracked():
    points = lane_points()
    localizer = LaneMapLocalizer(points, {"scan_voxel_m": .12})
    for index, drift in enumerate((.2, .25, .3), 1):
        result = localizer.match(points, [0., drift, 0.], index * 100000000)
        assert result.accepted, result
    assert len(localizer.refined_points_and_votes()[0]) > 0
    return localizer, points


def test_capture_is_constant_size_no_anchor_or_index_work(tracked, monkeypatch):
    localizer, _ = tracked

    def forbidden(*args, **kwargs):
        raise AssertionError("capture must not traverse geometry or construct an index")

    monkeypatch.setattr(localization, "_points", forbidden)
    monkeypatch.setattr(localization, "_voxel_means", forbidden)
    monkeypatch.setattr(localization, "_FrozenIndex", forbidden)
    state = localizer.capture_tracking_state()
    assert {field.name for field in fields(state)} == {
        "config", "correction_se2", "last_accepted_stamp_ns", "last_seen_stamp_ns",
        "motion_information"}
    assert state.motion_information is None  # Explicit legacy calls create no motion state.
    assert state.last_accepted_stamp_ns == state.last_seen_stamp_ns == 300000000
    np.testing.assert_allclose(state.correction_se2, [0., -.3, 0.], atol=1e-12)


def test_capture_is_immutable_and_owns_config_and_correction(tracked):
    localizer, _ = tracked
    state = localizer.capture_tracking_state()
    assert isinstance(state.correction_se2, tuple)
    with pytest.raises(FrozenInstanceError):
        state.last_seen_stamp_ns = 9
    with pytest.raises(TypeError):
        state.config["tracking_max_translation_m"] = 99.
    with pytest.raises(TypeError):
        state.correction_se2[1] = 99.
    localizer.config["scan_voxel_m"] = .11
    localizer.reset()
    assert state.config["scan_voxel_m"] == .12
    np.testing.assert_allclose(state.correction_se2, [0., -.3, 0.], atol=1e-12)
    assert state.last_seen_stamp_ns == 300000000


def test_rebuild_uses_captured_committed_state_not_later_source_or_pending(tracked):
    localizer, points = tracked
    pending = localizer.match(points, [0., .35, 0.], 400000000, commit=False)
    assert pending.accepted
    state = localizer.capture_tracking_state()
    assert localizer.commit(pending)
    child = LaneMapLocalizer.from_tracking_state(points, state)
    assert child.config == state.config and child.config["scan_voxel_m"] == .12
    np.testing.assert_allclose(child.correction_se2, [0., -.3, 0.], atol=1e-12)
    np.testing.assert_allclose(localizer.correction_se2, [0., -.35, 0.], atol=1e-12)
    assert child.last_accepted_stamp_ns == 300000000
    assert child.last_seen_stamp_ns == 400000000
    assert child._pending_result is None and child._pending_data is None
    assert not child.commit(pending)
    assert child.accepted_frames == child.rejected_frames == child.committed_frames == 0
    assert child.refinement_metrics()["refinement_retained_frames"] == 0
    assert len(child.refined_points_and_votes()[0]) == 0
    stale = child.match(points, [0., .3, 0.], 400000000, commit=False)
    assert not stale.accepted and stale.reason == "nonincreasing_mask_stamp"
    result = child.match(points, [0., .3, 0.], 500000000, commit=False)
    assert result.accepted and result.metrics["initial_acquisition"] is False
    assert child.commit(result) and not child.commit(result)


def test_anchor_source_comparison_uses_exact_original_normalizer_without_index(monkeypatch):
    # Binary fractions ensure the order-independence check itself has exact sums.
    source = np.array([[.125, .125], [.140625, .125], [.5, .125], [.515625, .125]])
    localizer = LaneMapLocalizer(source)
    assert len(localizer.anchor_points) < len(source)
    before = localizer.anchor_points.copy()

    def forbidden(*args, **kwargs):
        raise AssertionError("source equality must not rebuild a spatial index")

    monkeypatch.setattr(localization, "_FrozenIndex", forbidden)
    assert localizer.matches_anchor_source(source)
    assert localizer.matches_anchor_source(source[::-1])
    assert localizer.matches_anchor_source(before)
    changed = source.copy()
    changed[0, 1] += .0625
    assert not localizer.matches_anchor_source(changed)
    assert not localizer.matches_anchor_source(source[:-1])
    assert not localizer.matches_anchor_source([[float("nan"), 0.]])
    assert not localizer.matches_anchor_source([[0., 0., 0.]])
    np.testing.assert_array_equal(localizer.anchor_points, before)
    assert localizer.last_seen_stamp_ns is None


@pytest.mark.parametrize("bad", [None, {}, object()])
def test_rebuild_rejects_untyped_state(bad):
    with pytest.raises(ValueError, match="immutable committed"):
        LaneMapLocalizer.from_tracking_state(lane_points(), bad)


@pytest.mark.parametrize("field,value", [
    ("correction_se2", (0., float("nan"), 0.)),
    ("correction_se2", (0., 0.)),
    ("last_seen_stamp_ns", True),
    ("last_seen_stamp_ns", -1),
    ("last_seen_stamp_ns", 2.5),
    ("last_seen_stamp_ns", None),
    ("last_accepted_stamp_ns", 101),
    ("config", {"runtime_budget_ms": -1.}),
])
def test_immutable_state_rejects_invalid_boundaries(field, value):
    values = dict(config={}, correction_se2=(0., 0., 0.),
                  last_accepted_stamp_ns=100, last_seen_stamp_ns=100)
    values[field] = value
    with pytest.raises(ValueError):
        CommittedLocalizationState(**values)


def test_fresh_state_has_no_invented_correction_or_time():
    source = LaneMapLocalizer(np.empty((0, 2)))
    state = source.capture_tracking_state()
    child = LaneMapLocalizer.from_tracking_state(lane_points(), state)
    np.testing.assert_array_equal(child.correction_se2, np.zeros(3))
    assert child.last_accepted_stamp_ns is None and child.last_seen_stamp_ns is None
    result = child.match(lane_points(), [0., .2, 0.], 100000000)
    assert result.accepted and result.metrics["initial_acquisition"] is True
