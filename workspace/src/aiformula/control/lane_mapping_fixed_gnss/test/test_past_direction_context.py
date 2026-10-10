"""Past direction context is structure, never unconfirmed position evidence.

Synthetic geometry and controlled commit failures exercise real core objects.
The usual unit-test matcher time allowance is enlarged; no runtime, vehicle or
full-lap claim follows. No match, innovation, support or residual gate is relaxed.
"""

from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss import mapping
from lane_mapping_lya_reference.mapping_core import transform_local_points
from lane_mapping_lya_reference.mask_localization import (
    LaneMapLocalizer, _FrozenIndex, _configuration, _voxel_means,
)


def rectangle(x0, x1, y0, y1, spacing=.1):
    x, y = np.meshgrid(np.arange(x0, x1 + spacing * .25, spacing),
                       np.arange(y0, y1 + spacing * .25, spacing))
    return np.column_stack((x.ravel(), y.ravel()))


def lanes():
    x = np.arange(.5, 8.001, .05)
    return np.vstack([np.column_stack((x, np.full(len(x), y))) for y in (-1., 1.)])


def index_snapshot(index):
    return tuple(value.copy() for value in (
        index.anchor, index.normal_valid, index.normals, index.line_points,
        index.normal_radii, index.normal_thickness))


def assert_index_unchanged(index, before):
    for actual, expected in zip(index_snapshot(index), before):
        np.testing.assert_array_equal(actual, expected)


def test_dense_past_context_below_capacity_does_not_crash_query_initialization():
    x = np.arange(-1., 1.001, .1)
    anchor = np.column_stack((x, np.zeros(len(x))))
    context = rectangle(-2., 2., -2., 2.)
    assert len(context) < _configuration(None)["max_scan_support_points"]
    index = _FrozenIndex(anchor, _configuration(None), direction_support=context)
    np.testing.assert_array_equal(index.anchor, anchor)
    assert np.all(np.isfinite(index.line_points))


def test_remote_bridge_cannot_change_disconnected_local_pca_support():
    """Only points beyond every query radius are added; local geometry is equal.

    A global union-find alone incorrectly joins the two locally separate thick
    bands. In the original prototype that turns valid local geometry into an
    unobservable patch, even though none of the local samples changed.
    """
    cfg = _configuration(None)
    bands = np.vstack((rectangle(-.2, .2, -3., 3.),
                       rectangle(.5, .9, -3., 3.)))
    bridge = np.column_stack((np.arange(.2, .501, .1), np.full(4, 3.)))
    assert np.all(np.linalg.norm(bridge, axis=1) > cfg["normal_max_radius_m"])
    separate = _FrozenIndex(_voxel_means(bands, cfg["anchor_voxel_m"]), cfg)
    joined = _FrozenIndex(_voxel_means(np.vstack((bands, bridge)), cfg["anchor_voxel_m"]), cfg)
    first = int(np.argmin(np.linalg.norm(separate.anchor, axis=1)))
    second = int(np.argmin(np.linalg.norm(joined.anchor, axis=1)))
    np.testing.assert_allclose(separate.anchor[first], joined.anchor[second], atol=1e-14)
    assert separate.normal_valid[first]
    assert joined.normal_valid[second], "remote topology changed unchanged local support"
    assert abs(float(separate.normals[first] @ joined.normals[second])) > .999999
    np.testing.assert_allclose(joined.line_points[second], separate.line_points[first], atol=1e-12)


@pytest.mark.parametrize("context_y", [-.1, 0., .1])
def test_past_context_mean_never_becomes_confirmed_line_center(context_y):
    cfg = _configuration(None)
    anchor = rectangle(3., 3.2, -.2, .2)
    # Collinear context isolates the promised direction invariance. A finite
    # rectangle's endpoint PCA can genuinely tilt, which is a different test.
    context = rectangle(3., 6., context_y, context_y)
    index = _FrozenIndex(anchor, cfg, direction_support=context)
    assert index.direction_supported_points > 0
    use = index.normal_valid
    # All confirmed points have the same normal-direction center y=0. Moving
    # the unconfirmed context mean cannot move that positional reference.
    assert np.all(np.abs(index.normals[use, 1]) > .999999)
    np.testing.assert_allclose(index.line_points[use, 1], 0., atol=1e-12)
    np.testing.assert_array_equal(index.anchor, anchor)


def test_direction_context_cannot_supply_missing_confirmed_position_count():
    cfg = _configuration(None)
    anchor = np.array([[3., -.15], [3., -.05], [3., .05], [3., .15]])
    context = rectangle(3., 6., -.2, .2)
    assert len(anchor) < cfg["normal_min_points"]
    index = _FrozenIndex(anchor, cfg, direction_support=context)
    assert index.direction_supported_points == 0
    assert not np.any(index.normal_valid)
    np.testing.assert_array_equal(index.line_points, anchor)


def test_nonadjacent_direction_context_does_not_change_anchor_geometry():
    cfg = _configuration(None)
    anchor = _voxel_means(lanes(), cfg["anchor_voxel_m"])
    original = _FrozenIndex(anchor, cfg)
    unrelated = _FrozenIndex(anchor, cfg, direction_support=anchor + [0., 20.])
    assert unrelated.direction_supported_points == 0
    assert_index_unchanged(unrelated, index_snapshot(original))


def test_built_direction_reference_does_not_alias_input_arrays():
    anchor = rectangle(3., 3.2, -.2, .2)
    context = rectangle(3., 6., -.2, .2)
    matcher = LaneMapLocalizer(anchor, {"runtime_budget_ms": 2000.}, context, 1)
    before = index_snapshot(matcher._index)
    anchor[:] = [90., 90.]
    context[:] = [-90., -90.]
    assert_index_unchanged(matcher._index, before)


class Clock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now


def bootstrap():
    clock = Clock()
    engine = mapping.TeachMapEngine({"keyframe_refresh_s": .25},
                                   matcher_config={"runtime_budget_ms": 2000.}, clock=clock)
    points = lanes()
    sensitivity = np.full(len(points), .05)
    for index in range(1, 6):
        result = engine.process(points, sensitivity, [0., 0., 0.], index * 100000000)
        assert result["accepted"], result
    return engine, clock, points, sensitivity


def context_snapshot(engine):
    return (engine._past_direction_support, engine._past_direction_support.copy(),
            engine._past_direction_support_stamp_ns, engine._map.last_stamp_ns,
            np.asarray(engine._trace).copy(), engine.points_and_votes())


def assert_context_unchanged(engine, before):
    owner, values, stamp, frontier, trace, map_data = before
    assert engine._past_direction_support is owner
    np.testing.assert_array_equal(engine._past_direction_support, values)
    assert engine._past_direction_support_stamp_ns == stamp
    assert engine._map.last_stamp_ns == frontier
    np.testing.assert_array_equal(engine._trace, trace)
    for current, previous in zip(engine.points_and_votes(), map_data):
        np.testing.assert_array_equal(current, previous)


def test_candidate_uses_previous_committed_context_not_current_mask(monkeypatch):
    engine, _, points, sensitivity = bootstrap()
    prior_context = engine._past_direction_support.copy()
    previous_stamp = engine._past_direction_support_stamp_ns
    current = np.vstack((points, [[12., 4.]]))
    current_j = np.append(sensitivity, .4)
    captures = []
    original = mapping.LaneMapLocalizer

    def create(anchor, config=None, anchor_support_xy=None, anchor_support_stamp_ns=None):
        captures.append((np.array(anchor_support_xy), anchor_support_stamp_ns))
        return original(anchor, config, anchor_support_xy, anchor_support_stamp_ns)

    monkeypatch.setattr(mapping, "LaneMapLocalizer", create)
    row = engine.process(current, current_j, [0., 0., 0.], 600000000)
    assert row["accepted"], row
    assert len(captures) == 1
    np.testing.assert_array_equal(captures[0][0], prior_context)
    assert captures[0][1] == previous_stamp == 500000000
    assert captures[0][1] == row["matched_against_frontier_ns"] < row["stamp_ns"]
    np.testing.assert_allclose(engine._past_direction_support,
        _voxel_means(transform_local_points(current, *row["pose_xyyaw"]),
                     engine._geometry_config["scan_voxel_m"]))
    assert engine._past_direction_support_stamp_ns == 600000000


@pytest.mark.parametrize("failure", ["geometry", "guard", "deadline", "capacity", "commit"])
def test_rejected_frame_never_replaces_committed_direction_context(monkeypatch, failure):
    engine, clock, points, sensitivity = bootstrap()
    before = context_snapshot(engine)
    guard = None
    expected = None
    if failure == "geometry":
        points = points + [0., 15.]
    elif failure == "guard":
        @contextmanager
        def guard():
            yield False
        expected = "commit_guard_rejected"
    elif failure == "deadline":
        @contextmanager
        def guard():
            clock.now += .081
            yield True
        expected = "processing_deadline"
    elif failure == "capacity":
        monkeypatch.setattr(engine, "_capacity_allows", lambda *_: False)
        expected = "map_capacity"
    else:
        monkeypatch.setattr(LaneMapLocalizer, "commit", lambda *_: False)
        expected = "matcher_commit_rejected"
    row = engine.process(points, sensitivity, [0., 0., 0.], 600000000, commit_guard=guard)
    assert not row["accepted"], row
    if expected is not None:
        assert row["reason"] == expected
    assert_context_unchanged(engine, before)


def test_direction_context_is_readonly_and_published_inside_guard():
    engine, _, points, sensitivity = bootstrap()
    old_context = engine._past_direction_support
    phases = []

    @contextmanager
    def guard():
        phases.append(engine._past_direction_support_stamp_ns)
        assert engine._past_direction_support is old_context
        yield True
        phases.append(engine._past_direction_support_stamp_ns)
        assert engine._past_direction_support is not old_context

    row = engine.process(points, sensitivity, [0., 0., 0.], 600000000, commit_guard=guard)
    assert row["accepted"], row
    assert phases == [500000000, 600000000]
    assert not engine._past_direction_support.flags.writeable
    with pytest.raises(ValueError):
        engine._past_direction_support.flags.writeable = True
    before = engine._past_direction_support.copy()
    points[:] = 100.
    np.testing.assert_array_equal(engine._past_direction_support, before)


def test_prepared_candidate_is_frozen_when_next_committed_context_changes():
    engine, _, points, sensitivity = bootstrap()
    prepared = engine._prepare_matcher(np.zeros(3), 600000000)
    before = index_snapshot(prepared.matcher._index)
    assert prepared.direction_source_stamp_ns == 500000000
    assert prepared.direction_owner is engine._past_direction_support
    row = engine.process(points, sensitivity, [0., 0., 0.], 600000000)
    assert row["accepted"], row
    assert engine._past_direction_support_stamp_ns == 600000000
    assert prepared.direction_source_stamp_ns == 500000000
    assert prepared.direction_owner is not engine._past_direction_support
    assert_index_unchanged(prepared.matcher._index, before)


def test_freeze_repeat_handoff_does_not_import_temporary_direction_context():
    engine, _, points, sensitivity = bootstrap()
    row = engine.process(points, sensitivity, [0., 0., 0.], 600000000)
    assert row["accepted"], row
    state = engine._matcher.capture_tracking_state()
    repeated = engine.repeat_localizer()
    assert repeated.anchor_direction_source_stamp_ns is None
    assert getattr(repeated._index, "direction_supported_points", 0) == 0
    assert repeated.capture_tracking_state() == state
    assert repeated.matches_anchor_source(engine.freeze().consensus_xy)


@pytest.mark.parametrize("direction_stamp", [100, 101])
def test_current_or_future_direction_context_cannot_validate_a_mask(direction_stamp):
    points = lanes()
    matcher = LaneMapLocalizer(points, {"runtime_budget_ms": 2000.},
                               points, direction_stamp)
    before = index_snapshot(matcher._index)
    result = matcher.match(points, [0., 0., 0.], 100, scan_support_xy=points)
    assert not result.accepted
    assert result.reason == "future_or_current_anchor_direction"
    assert matcher.last_accepted_stamp_ns is None
    assert matcher.committed_frames == 0
    np.testing.assert_array_equal(matcher.correction_se2, np.zeros(3))
    assert_index_unchanged(matcher._index, before)


@pytest.mark.parametrize("invalid_stamp", [None, 500000001, 600000000])
def test_engine_does_not_prepare_from_context_newer_than_its_map_frontier(invalid_stamp):
    engine, _, _, _ = bootstrap()
    engine._past_direction_support_stamp_ns = invalid_stamp
    with pytest.raises(ValueError, match="uncommitted evidence"):
        engine._prepare_matcher(np.zeros(3), 600000000)
    assert engine._candidate is None
    assert engine._matcher is None
    assert engine._map.last_stamp_ns == 500000000


def test_candidate_cache_is_bound_to_context_owner_not_just_map_frontier():
    engine, _, _, _ = bootstrap()
    first = engine._prepare_matcher(np.zeros(3), 600000000)
    original_context = engine._past_direction_support
    before = index_snapshot(first.matcher._index)
    # Simulate a newly owned immutable context with equal coordinates/stamp;
    # cached objects may not rely on some other owner's array identity.
    engine._past_direction_support = mapping._readonly(original_context, 2)
    assert engine._past_direction_support is not original_context
    second = engine._prepare_matcher(np.zeros(3), 600000000)
    assert second is not first
    assert second.direction_owner is engine._past_direction_support
    assert first.direction_owner is original_context
    assert_index_unchanged(first.matcher._index, before)


def test_failed_candidate_fallback_keeps_active_original_direction_context(monkeypatch):
    engine, _, points, sensitivity = bootstrap()
    first = engine.process(points, sensitivity, [0., 0., 0.], 600000000)
    assert first["accepted"], first
    active = engine._matcher
    active_geometry = index_snapshot(active._index)
    assert active.anchor_direction_source_stamp_ns == 500000000
    candidate = engine._prepare_matcher(np.zeros(3), 900000000)
    assert candidate.matcher.anchor_direction_source_stamp_ns == 600000000
    original = candidate.matcher.match

    def reject(*args, **kwargs):
        result = original(*args, **kwargs)
        assert result.accepted, result
        candidate.matcher.discard_pending()
        return replace(result, accepted=False, reason="unobservable_geometry")

    monkeypatch.setattr(candidate.matcher, "match", reject)
    row = engine.process(points, sensitivity, [0., 0., 0.], 900000000)
    assert row["accepted"], row
    assert row["metrics"]["selected_reference"] == "active"
    assert engine._matcher is active
    assert active.anchor_direction_source_stamp_ns == 500000000
    assert engine._past_direction_support_stamp_ns == 900000000
    assert candidate.matcher.anchor_direction_source_stamp_ns == 600000000
    assert_index_unchanged(active._index, active_geometry)


@pytest.mark.parametrize("explicit_none", [False, True])
def test_public_direction_hint_without_source_stamp_is_rejected(explicit_none):
    points = lanes()
    kwargs = {"anchor_support_xy": points}
    if explicit_none:
        kwargs["anchor_support_stamp_ns"] = None
    with pytest.raises(ValueError, match="source stamp"):
        LaneMapLocalizer(points, {"runtime_budget_ms": 2000.}, **kwargs)


@pytest.mark.parametrize("bad_stamp", [0, -1, False, 1.5, "1", 2 ** 63])
def test_public_direction_hint_rejects_nonpositive_or_nonintegral_source_stamp(bad_stamp):
    points = lanes()
    with pytest.raises(ValueError, match="source stamp"):
        LaneMapLocalizer(points, {"runtime_budget_ms": 2000.}, points, bad_stamp)


def test_explicit_past_direction_stamp_allows_current_match():
    points = lanes()
    matcher = LaneMapLocalizer(points, {"runtime_budget_ms": 2000.}, points, 1)
    result = matcher.match(points, [0., 0., 0.], 2, prior_stamp_ns=1,
                           scan_support_xy=points)
    assert result.accepted, result
    assert matcher.anchor_direction_source_stamp_ns == 1
    assert matcher.last_accepted_stamp_ns == 2


def test_without_direction_hint_existing_unseeded_semantics_are_unchanged():
    points = lanes()
    matcher = LaneMapLocalizer(points, {"runtime_budget_ms": 2000.})
    assert matcher.anchor_direction_source_stamp_ns is None
    assert matcher.last_accepted_stamp_ns is None
    result = matcher.match(points, [0., 0., 0.], 1)
    assert result.accepted, result
    assert result.metrics["initial_acquisition"]
