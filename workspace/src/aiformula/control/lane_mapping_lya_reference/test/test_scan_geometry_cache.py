"""Current-scan cache equivalence, not a runtime or driving benchmark.

Every comparison runs the real matcher twice: with its scan cache and with that
cache cleared before each frame. Only the unit-test wall budget is 2000 ms;
geometry, overlap, capacity, innovation and vote gates retain their defaults.
"""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer, _voxel_means


GEOMETRY = ("normal_valid", "normals", "line_points", "normal_radii",
            "normal_thickness", "normal_position_support", "direction_borrowed")
NON_SEMANTIC_METRICS = {"elapsed_ms", "scan_reused_identical_geometry_points",
                        "anchor_reused_identical_geometry_points"}


def line(y, start=.5, stop=8., step=.11):
    x = np.arange(start, stop, step)
    return np.column_stack((x, np.full(len(x), y)))


def rectangle(x0, x1, y0, y1, step=.05):
    x, y = np.meshgrid(np.arange(x0, x1 + step * .25, step),
                       np.arange(y0, y1 + step * .25, step))
    return np.column_stack((x.ravel(), y.ravel()))


def lanes():
    return np.vstack((line(-1.8), line(1.8)))


def matcher(anchor=None):
    return LaneMapLocalizer(lanes() if anchor is None else anchor,
                            {"runtime_budget_ms": 2000.})


def assert_bytes(actual, expected):
    assert actual.shape == expected.shape
    assert actual.dtype == expected.dtype
    assert actual.tobytes() == expected.tobytes()


def geometry_snapshot(index):
    return {name: getattr(index, name).copy() for name in
            ("anchor", "components", "computed_normals") + GEOMETRY}


def assert_index_unchanged(index, snapshot):
    for name, previous in snapshot.items():
        assert_bytes(getattr(index, name), previous)


def committed_snapshot(value):
    points, votes = value.refined_points_and_votes()
    return dict(correction=value.correction_se2, stamp=value.last_accepted_stamp_ns,
                commits=value.committed_frames, metrics=value.refinement_metrics(),
                points=points, votes=votes, anchor=value.anchor_points)


def assert_committed_unchanged(value, snapshot):
    assert_bytes(value.correction_se2, snapshot["correction"])
    assert value.last_accepted_stamp_ns == snapshot["stamp"]
    assert value.committed_frames == snapshot["commits"]
    assert value.refinement_metrics() == snapshot["metrics"]
    points, votes = value.refined_points_and_votes()
    assert_bytes(points, snapshot["points"])
    assert_bytes(votes, snapshot["votes"])
    assert_bytes(value.anchor_points, snapshot["anchor"])


def assert_same_result(cached, fresh):
    assert (cached.accepted, cached.reason, cached.stamp_ns) == (fresh.accepted, fresh.reason, fresh.stamp_ns)
    assert_bytes(cached.pose_xyyaw, fresh.pose_xyyaw)
    assert_bytes(cached.correction_se2, fresh.correction_se2)
    left = {k: v for k, v in cached.metrics.items() if k not in NON_SEMANTIC_METRICS}
    right = {k: v for k, v in fresh.metrics.items() if k not in NON_SEMANTIC_METRICS}
    assert left == right


def assert_current_geometry_equal(cached, fresh, reliable):
    """Unrequested memoized rows are not current measurement support."""
    a, b = cached._scan_geometry_index, fresh._scan_geometry_index
    assert_bytes(a.anchor, b.anchor)
    scan = _voxel_means(reliable, cached.config["scan_voxel_m"])
    if len(scan) > cached.config["max_scan_points"]:
        scan = scan[np.linspace(0, len(scan) - 1, cached.config["max_scan_points"]).astype(np.int64)]
    cells = {tuple(key): i for i, key in enumerate(np.floor(a.anchor / .1).astype(np.int64))}
    targets = np.asarray([cells[tuple(key)] for key in np.floor(scan / .1).astype(np.int64)], dtype=np.int64)
    for name in GEOMETRY:
        assert_bytes(getattr(a, name)[targets], getattr(b, name)[targets])


def step_pair(cached, fresh, reliable, support, stamp, prior=(0., .10, .01), commit=True):
    fresh._scan_geometry_index = None  # The independent no-cache reference.
    arguments = dict(prior_xyyaw=prior, stamp_ns=stamp, prior_stamp_ns=stamp,
                     commit=commit, scan_support_xy=support,
                     scan_sensitivity=np.full(len(reliable), .03))
    observed = cached.match(reliable, **arguments)
    expected = fresh.match(reliable, **arguments)
    assert_same_result(observed, expected)
    assert_current_geometry_equal(cached, fresh, reliable)
    assert_committed_unchanged(cached, committed_snapshot(fresh))
    return observed


@pytest.mark.parametrize("scene", ["same", "changed", "thick", "clipped"])
def test_frame_sequence_matches_fresh_geometry_exactly(scene):
    base = lanes()
    bar = rectangle(4., 7., -.20, .20)
    anchor = np.vstack((base, bar)) if scene in ("thick", "clipped") else base
    cached, fresh = matcher(anchor), matcher(anchor)
    hits = []
    previous = None
    for number in range(4):
        if scene == "same":
            reliable = support = base.copy()
        elif scene == "changed":
            reliable = support = base.copy()
            if number >= 2:
                reliable[(reliable[:, 0] > 4.) & (reliable[:, 1] > 0.), 1] += .025
        else:
            support = np.vstack((base, bar))
            depth = (1.5 if scene == "thick" else [.10, .30, .60, .10][number])
            reliable = np.vstack((base, bar[bar[:, 0] <= 4. + depth + 1e-9]))
        result = step_pair(cached, fresh, reliable, support, number + 1)
        assert result.accepted, (result.reason, result.metrics)
        hits.append(result.metrics["scan_reused_identical_geometry_points"])
        if previous is not None:
            assert_index_unchanged(*previous)
        previous = (cached._scan_geometry_index, geometry_snapshot(cached._scan_geometry_index))
    assert hits[0] == 0 and max(hits[1:]) > 0


def test_new_current_targets_are_computed_not_replaced_by_old_requested_mask():
    support = lanes()
    cached, fresh = matcher(), matcher()
    for stamp, reliable in enumerate((support[support[:, 1] < 0.],
                                      support[support[:, 1] > 0.], support), 1):
        result = step_pair(cached, fresh, reliable, support, stamp, prior=(0., .05, 0.))
        assert result.accepted
        assert result.metrics["scan_line_points"] == len(_voxel_means(reliable, .1))


@pytest.mark.parametrize("key,replacement", [("normal_min_points", 12), ("normal_radius_m", .65)])
def test_mutated_public_config_invalidates_old_geometry_settings(key, replacement):
    cached, fresh = matcher(), matcher()
    points = lanes()
    assert step_pair(cached, fresh, points, points, 1, prior=(0., 0., 0.)).accepted
    source = cached._scan_geometry_index
    source_geometry = geometry_snapshot(source)
    original_config = dict(source.cfg)
    assert source.cfg is not cached.config
    with pytest.raises(TypeError):
        source.cfg[key] = replacement

    # The public dictionary keeps its existing mutable contract. Both matchers
    # retain the same old anchor; only the current-scan cache differs here.
    cached.config[key] = fresh.config[key] = replacement
    assert dict(source.cfg) == original_config
    assert cached._index.cfg[key] == original_config[key]
    result = step_pair(cached, fresh, points, points, 2, prior=(0., 0., 0.))
    assert result.accepted
    assert result.metrics["scan_reused_identical_geometry_points"] == 0
    assert cached._scan_geometry_index.cfg[key] == replacement
    assert source.normal_radii.tobytes() != cached._scan_geometry_index.normal_radii.tobytes()
    assert_index_unchanged(source, source_geometry)


@pytest.mark.parametrize("offset,reason", [(.9, "insufficient_overlap"), (.3, "tracking_innovation_gate")])
def test_rejected_current_cache_changes_neither_committed_pose_nor_map(offset, reason):
    cached, fresh = matcher(), matcher()
    points = lanes()
    assert step_pair(cached, fresh, points, points, 1, prior=(0., 0., 0.)).accepted
    before = committed_snapshot(cached)
    old = cached._scan_geometry_index
    old_values = geometry_snapshot(old)
    wrong = points + [0., offset]
    for stamp in (2, 3):
        result = step_pair(cached, fresh, wrong, wrong, stamp, prior=(0., 0., 0.), commit=False)
        assert not result.accepted and result.reason == reason
        assert cached._pending_result is None and cached._pending_data is None
        assert not cached.commit(result)
        assert_committed_unchanged(cached, before)
    assert cached._scan_geometry_index is not old
    assert result.metrics["scan_reused_identical_geometry_points"] > 0
    assert_index_unchanged(old, old_values)


def test_cached_geometry_does_not_alias_mutable_caller_arrays():
    points, support = lanes(), lanes()
    original = points.copy()
    cached = matcher()
    assert cached.match(points, [0., 0., 0.], 1, scan_support_xy=support).accepted
    source = cached._scan_geometry_index
    before = geometry_snapshot(source)
    assert not np.shares_memory(source.anchor, points)
    assert not np.shares_memory(source.anchor, support)
    assert not source.anchor.flags.writeable
    with pytest.raises(ValueError):
        source.anchor.flags.writeable = True
    points[:] = -22.
    support[:] = 23.
    result = cached.match(original, [0., 0., 0.], 2, scan_support_xy=original)
    assert result.accepted and result.metrics["scan_reused_identical_geometry_points"] > 0
    assert_index_unchanged(source, before)


def test_fork_shares_only_source_memoization_and_does_not_mutate_owner():
    source = matcher()
    points = lanes()
    assert source.match(points, [0., .05, 0.], 1).accepted
    pending = source.match(points, [0., .08, 0.], 2, commit=False)
    assert pending.accepted
    source_cache = source._scan_geometry_index
    before_cache, before_state = geometry_snapshot(source_cache), committed_snapshot(source)
    child = source.fork_with_anchor(source.anchor_points)
    reference = LaneMapLocalizer.from_tracking_state(source.anchor_points, source.capture_tracking_state())
    assert child._scan_geometry_index is source_cache
    assert reference._scan_geometry_index is None
    assert child._pending_result is None and child._pending_data is None
    assert not child.commit(pending)
    result = step_pair(child, reference, points, points, 3, prior=(0., .07, 0.))
    assert result.accepted and result.metrics["scan_reused_identical_geometry_points"] > 0
    assert child._scan_geometry_index is not source_cache
    assert source._scan_geometry_index is source_cache
    assert source._pending_result is pending
    assert_index_unchanged(source_cache, before_cache)
    assert_committed_unchanged(source, before_state)


def test_reset_clears_scan_cache_and_tracking_state():
    value = matcher()
    points = lanes()
    assert value.match(points, [0., .05, 0.], 1).accepted
    assert value._scan_geometry_index is not None
    value.reset()
    assert value._scan_geometry_index is None
    assert value.last_accepted_stamp_ns is None and value.last_seen_stamp_ns is None
    assert value.committed_frames == 0
    assert_bytes(value.correction_se2, np.zeros(3))
    result = value.match(points, [0., .05, 0.], 1)
    assert result.accepted and result.metrics["scan_reused_identical_geometry_points"] == 0


def test_committed_state_handoff_never_serializes_or_restores_scan_geometry():
    source = matcher()
    assert source.match(lanes(), [0., .05, 0.], 1).accepted
    state = source.capture_tracking_state()
    assert set(state.__dataclass_fields__) == {
        "config", "correction_se2", "last_accepted_stamp_ns", "last_seen_stamp_ns", "motion_information"}
    assert state.motion_information is None  # Legacy acquisition carries no motion factor or scan cache.
    repeated = LaneMapLocalizer.from_tracking_state(source.anchor_points, state)
    assert repeated._scan_geometry_index is None
    assert repeated.capture_tracking_state() == state
    assert repeated._pending_result is None
    assert repeated.refinement_metrics()["refinement_retained_frames"] == 0


@pytest.mark.parametrize("failure", ["nonincreasing", "future_prior", "invalid_scan", "invalid_support"])
def test_input_fences_do_not_consume_or_replace_scan_cache(failure):
    value = matcher()
    points = lanes()
    assert value.match(points, [0., 0., 0.], 10).accepted
    source, before = value._scan_geometry_index, committed_snapshot(value)
    source_values = geometry_snapshot(source)
    stamp, prior_stamp, scan, support = 11, 11, points, points
    if failure == "nonincreasing":
        stamp = prior_stamp = 10
    elif failure == "future_prior":
        prior_stamp = 12
    elif failure == "invalid_scan":
        scan = np.array([[np.nan, 0.]])
    else:
        support = np.array([[np.nan, 0.]])
    result = value.match(scan, [0., 0., 0.], stamp, prior_stamp_ns=prior_stamp,
                         scan_support_xy=support, commit=False)
    assert not result.accepted
    assert value._scan_geometry_index is source
    assert_index_unchanged(source, source_values)
    assert_committed_unchanged(value, before)


def test_4000_support_cap_is_checked_even_with_warm_geometry_cache():
    points = lanes()
    padding = 4001 - len(points)
    number = np.arange(padding)
    remote = np.column_stack((1.01 + (number % 80) * .21, 5.01 + (number // 80) * .21))
    support = np.vstack((points, remote))
    assert np.max(np.linalg.norm(support, axis=1)) < 30.
    assert len(_voxel_means(support, .1)) == 4001
    cached, fresh = matcher(), matcher()
    result = step_pair(cached, fresh, points, support[:-1], 1, prior=(0., 0., 0.))
    assert result.accepted and result.metrics["scan_support_points"] == 4000
    assert cached._scan_geometry_index._adjacency_bits is not None
    source, before = cached._scan_geometry_index, committed_snapshot(cached)
    values = geometry_snapshot(source)
    result = cached.match(points, [0., 0., 0.], 2, scan_support_xy=support, commit=False)
    assert not result.accepted and result.reason == "scan_geometry_capacity"
    assert cached._scan_geometry_index is source
    assert_index_unchanged(source, values)
    assert_committed_unchanged(cached, before)
