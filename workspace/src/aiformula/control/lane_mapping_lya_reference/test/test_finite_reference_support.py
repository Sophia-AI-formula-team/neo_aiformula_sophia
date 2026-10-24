"""Finite old-map line support: independent graph oracle and ownership tests.

No scene truth, timing relaxation of production defaults, or acceptance-rate
claim. The one matching setup uses an explicitly ordinary 2000 ms unit budget;
all geometry, residual, overlap and innovation thresholds remain unchanged.
"""
import importlib
import math
import os

import numpy as np
import pytest

from lane_mapping_lya_reference import mask_localization as module


def line(start=0., stop=3., y=0., step=.05):
    x = np.arange(start, stop + step * .25, step)
    return np.column_stack((x, np.full(len(x), y)))


def rotate(points, angle):
    c, s = math.cos(angle), math.sin(angle)
    return points @ np.array([[c, s], [-s, c]])


def cfg(**kw):
    return module._configuration(kw)


def independent_patch(anchor, config, target, radius):
    """Reproduce cell-order/truncation without calling index._patch."""
    keys = np.floor(anchor / config["hash_cell_m"]).astype(np.int64)
    key = keys[target]
    reach = int(math.ceil(radius / config["hash_cell_m"]))
    candidates = []
    for dx in range(-reach, reach + 1):
        for dy in range(-reach, reach + 1):
            candidates.extend(np.flatnonzero(np.all(keys == key + [dx, dy], axis=1)).tolist())
    candidates = np.asarray(candidates, dtype=np.int64)
    if len(candidates) > config["max_patch_points"]:
        center = (key.astype(float) + .5) * config["hash_cell_m"]
        distance = np.sum((anchor[candidates] - center) ** 2, axis=1)
        candidates = candidates[np.argsort(distance, kind="stable")[:config["max_patch_points"]]]
    return candidates


def independent_support(index, target):
    """Brute-force seed-induced graph, not global-component membership."""
    radius = index.normal_radii[target]
    candidates = independent_patch(index.anchor, index.cfg, target, radius)
    delta = index.anchor[candidates] - index.anchor[target]
    candidates = candidates[np.sum(delta * delta, axis=1) <= radius * radius]
    if target not in candidates:
        return np.empty(0, dtype=np.int64)
    points = index.anchor[candidates]
    delta = points[:, None, :] - points[None, :, :]
    adjacency = np.sum(delta * delta, axis=2) <= index.cfg["normal_connectivity_m"] ** 2
    start = int(np.flatnonzero(candidates == target)[0])
    seen = {start}
    pending = [start]
    while pending:
        node = pending.pop()
        for neighbor in np.flatnonzero(adjacency[node]):
            neighbor = int(neighbor)
            if neighbor not in seen:
                seen.add(neighbor)
                pending.append(neighbor)
    return candidates[np.array([i in seen for i in range(len(candidates))])]


def independent_bounds(index, ids):
    result = np.tile([np.inf, -np.inf], (len(ids), 1))
    for row, target in enumerate(ids):
        if not index.normal_valid[target]:
            continue
        support = independent_support(index, int(target))
        if not len(support):
            continue
        normal = index.normals[target]
        tangent = np.array([-normal[1], normal[0]])
        positions = (index.anchor[support] - index.line_points[target]) @ tangent
        result[row] = [np.min(positions), np.max(positions)]
    return result


@pytest.fixture(autouse=True)
def python_backend(monkeypatch):
    # Native equivalence is covered explicitly below. Source-only CI still runs
    # all semantics/ownership tests; no optional backend hides these assertions.
    monkeypatch.setattr(module, "_GRAPH_NATIVE", None)


@pytest.mark.parametrize("angle", [0., .17, math.pi / 2, -.8])
@pytest.mark.parametrize("offset", [[0., 0.], [.49, -.51]])
def test_bounds_match_independent_induced_graph_and_patch_order(angle, offset):
    points = rotate(np.vstack((line(y=-1.3), line(y=1.3))), angle) + offset
    index = module._FrozenIndex(points, cfg())
    ids = np.arange(len(points), dtype=np.int64)
    actual = index.line_support_bounds(ids)
    expected = independent_bounds(index, ids)
    np.testing.assert_allclose(actual, expected, rtol=0, atol=2e-15)
    assert np.all(actual[:, 0] <= actual[:, 1])


def test_empty_duplicate_and_noncontiguous_ids_preserve_order_and_owned_output():
    index = module._FrozenIndex(line(), cfg())
    empty = index.line_support_bounds([])
    assert empty.shape == (0, 2) and empty.dtype == np.float64
    ids = np.array([0, 7, 20, 5, 0, 21, 60, 9], dtype=np.int64)[::2]
    actual = index.line_support_bounds(ids)
    np.testing.assert_array_equal(actual, independent_bounds(index, ids))
    assert actual[0].tobytes() == actual[2].tobytes()
    original = actual.copy()
    actual[:] = 12345.
    np.testing.assert_array_equal(index.line_support_bounds(ids), original)


@pytest.mark.parametrize("bad", [[True], [1.0], ["1"], [[1]], [-1], [61],
                                 np.array([2 ** 63], dtype=np.uint64), np.array([False])])
def test_invalid_id_contract_is_explicit(bad):
    index = module._FrozenIndex(line(), cfg())
    with pytest.raises(ValueError):
        index.line_support_bounds(bad)


def test_invalid_normal_has_empty_interval_without_nan():
    points = np.array([[0., 0.], [.1, .1], [.2, .2]])
    index = module._FrozenIndex(points, cfg())
    assert not index.normal_valid.any()
    actual = index.line_support_bounds([2, 0, 2])
    np.testing.assert_array_equal(actual, np.tile([np.inf, -np.inf], (3, 1)))
    assert not np.isnan(actual).any()


@pytest.mark.parametrize("angle", [0., .4, -1.2])
def test_correspondences_reject_endpoint_extrapolation_without_changing_nearest(angle):
    points = rotate(line(stop=2., step=.1), angle)
    matcher = module.LaneMapLocalizer(points)
    # Interior margins isolate rotation from floating boundary evaluation;
    # exact inclusive/nextafter behavior is tested separately on an axis.
    query = rotate(np.array([[.05, .01], [1., .01], [1.95, .01], [2.2, .01], [-.2, .01]]), angle)
    old_nearest = matcher._index.nearest(query)
    assert np.all(old_nearest >= 0), "all queries are inside the original .65 m distance gate"
    _, _, selected, _, residual = matcher._correspondences(query, np.zeros(3))
    assert np.array_equal(selected, np.array([0, 1, 2]))
    assert len(residual) == 3
    np.testing.assert_array_equal(matcher._index.nearest(query), old_nearest)


def test_axis_aligned_exact_support_boundary_is_inclusive_no_extrapolation_padding():
    matcher = module.LaneMapLocalizer(line(stop=2., step=.1))
    end = float(np.max(matcher._index.anchor[:, 0]))
    query = np.array([[end, .01], [np.nextafter(end, -np.inf), .01],
                      [np.nextafter(end, np.inf), .01]])
    selected = matcher._correspondences(query, np.zeros(3))[2]
    np.testing.assert_array_equal(selected, [0, 1])


def test_invalid_nearest_endpoint_is_dropped_not_reassociated_to_other_line():
    points = np.vstack((line(stop=2., y=0., step=.1), line(stop=3., y=.3, step=.1)))
    matcher = module.LaneMapLocalizer(points)
    query = np.array([[2.1, .03]])
    nearest = matcher._index.nearest(query)
    assert nearest[0] >= 0
    assert abs(matcher._index.anchor[nearest[0], 1]) < .01
    # A second line would satisfy the old distance gate, but nearest selection
    # is not rerun after rejecting the unsupported first reference segment.
    assert np.min(np.linalg.norm(matcher._index.anchor[matcher._index.anchor[:, 1] > .2] - query, axis=1)) < .65
    assert matcher._correspondences(query, np.zeros(3))[2].size == 0


def test_remote_u_bridge_cannot_expand_local_segment():
    lower = line(0., 4., 0., .05)
    upper = line(0., 4., .35, .05)
    bridge = np.column_stack((np.full(8, 4.), np.linspace(0., .35, 8)))
    points = np.unique(np.vstack((lower, upper, bridge)), axis=0)
    index = module._FrozenIndex(points, cfg())
    target = int(np.argmin(np.sum((points - [1., 0.]) ** 2, axis=1)))
    assert len(np.unique(index.components)) == 1
    support = independent_support(index, target)
    assert np.all(points[support, 1] == 0.)
    np.testing.assert_array_equal(index.line_support_bounds([target]), independent_bounds(index, [target]))


def test_borrowed_direction_never_borrows_unconfirmed_longitudinal_extent():
    x, y = np.meshgrid(np.array([-.1, 0., .1]), np.array([-.1, 0., .1]))
    confirmed = np.column_stack((x.ravel(), y.ravel()))
    hint = line(-3., 3., 0., .05)
    ordinary = module._FrozenIndex(confirmed, cfg())
    assert not ordinary.normal_valid.any(), "the confirmed square is isotropic"
    index = module._FrozenIndex(confirmed, cfg(), direction_support=hint)
    ids = np.flatnonzero(index.direction_borrowed)
    assert len(ids) > 0
    actual = index.line_support_bounds(ids)
    np.testing.assert_allclose(actual, independent_bounds(index, ids), rtol=0, atol=1e-15)
    assert np.all(actual[:, 1] - actual[:, 0] <= .200000000000001)
    assert len(hint) > len(confirmed), "past direction context is much longer than confirmed geometry"


def test_lazy_cached_bounds_do_not_recompute_or_leak_mutable_output(monkeypatch):
    index = module._FrozenIndex(line(), cfg())
    ids = np.array([0, 12, 30, 60, 12])
    expected = index.line_support_bounds(ids)
    def forbidden(*args, **kwargs):
        raise AssertionError("already computed finite bounds must be memoized")
    monkeypatch.setattr(index, "_patch", forbidden)
    again = index.line_support_bounds(ids[::-1])
    np.testing.assert_array_equal(again, expected[::-1])


def test_geometry_reuse_does_not_share_extent_cache_across_different_indexes():
    old = module._FrozenIndex(line(stop=2., step=.1), cfg())
    old_ids = np.array([len(old.anchor) - 1])
    frozen = old.line_support_bounds(old_ids)
    new = module._FrozenIndex(line(stop=2.3, step=.1), cfg(), reuse_index=old)
    new_id = int(np.argmin(np.sum((new.anchor - [2., 0.]) ** 2, axis=1)))
    actual = new.line_support_bounds([new_id])
    np.testing.assert_array_equal(actual, independent_bounds(new, [new_id]))
    assert not np.array_equal(actual, frozen)
    np.testing.assert_array_equal(old.line_support_bounds(old_ids), frozen)


def test_finite_query_does_not_mutate_committed_pose_motion_or_refinement():
    anchor = np.vstack((line(-2., 8., -1.3), line(-2., 8., 1.3)))
    scan = np.vstack((line(0., 4., -1.3), line(0., 4., 1.3)))
    matcher = module.LaneMapLocalizer(anchor, {"runtime_budget_ms": 2000.})
    result = matcher.match(scan, np.zeros(3), 1_000_000_000,
                           motion_source_stamp_ns=1_000_000_000)
    assert result.accepted, result.reason
    state = matcher.capture_tracking_state()
    refined = matcher.refined_points_and_votes()
    counters = (matcher.accepted_frames, matcher.rejected_frames, matcher.committed_frames)
    matcher._index.line_support_bounds(np.arange(len(matcher._anchor)))
    matcher._correspondences(np.array([[8.2, -1.3], [-2.2, 1.3]]), np.zeros(3))
    assert matcher.capture_tracking_state() == state
    assert counters == (matcher.accepted_frames, matcher.rejected_frames, matcher.committed_frames)
    for actual, expected in zip(matcher.refined_points_and_votes(), refined):
        np.testing.assert_array_equal(actual, expected)
    assert matcher._pending_result is None


@pytest.mark.parametrize("shape", ["lines", "arc", "u", "borrowed", "clipped_patch"])
def test_real_native_and_python_bounds_and_correspondences_agree(monkeypatch, shape):
    try:
        native = importlib.import_module("_lane_mapping_lya_native")
    except ImportError:
        if os.environ.get("LANE_REQUIRE_NATIVE") == "1":
            pytest.fail("LANE_REQUIRE_NATIVE=1 but the native backend is unavailable")
        pytest.skip("native comparison only; all finite-support semantics also run in Python")
    hint = None
    configuration = cfg()
    if shape == "lines":
        points = np.vstack((line(y=-1.3), line(y=1.3)))
    elif shape == "arc":
        phi = np.linspace(-.2, .2, 101)
        points = np.column_stack((16. * np.sin(phi), 16. * (1. - np.cos(phi))))
    elif shape == "u":
        points = np.unique(np.vstack((line(0., 3., 0.), line(0., 3., .35),
            np.column_stack((np.full(8, 3.), np.linspace(0., .35, 8))))), axis=0)
    elif shape == "borrowed":
        x, y = np.meshgrid([-.1, 0., .1], [-.1, 0., .1])
        points = np.column_stack((x.ravel(), y.ravel()))
        hint = line(-3., 3.)
    else:
        x, y = np.meshgrid(np.linspace(0., 2., 41), np.linspace(-.15, .15, 7))
        points = np.column_stack((x.ravel(), y.ravel()))
        configuration = cfg(max_patch_points=16)
    python = module._FrozenIndex(points, configuration, direction_support=hint)
    assert python._native_graph is None
    monkeypatch.setattr(module, "_GRAPH_NATIVE", native)
    compiled = module._FrozenIndex(points, configuration, direction_support=hint)
    assert compiled._native_graph is not None, "native geometry path must really be present"
    ids = np.arange(len(points), dtype=np.int64)
    for name in ("normals", "normal_valid", "line_points", "normal_radii", "direction_borrowed"):
        np.testing.assert_array_equal(getattr(compiled, name), getattr(python, name))
    np.testing.assert_array_equal(compiled.line_support_bounds(ids), python.line_support_bounds(ids))
    np.testing.assert_allclose(compiled.line_support_bounds(ids), independent_bounds(compiled, ids), rtol=0, atol=2e-15)
    query = points[::5] + [.04, .01]
    first = module.LaneMapLocalizer(np.empty((0, 2)))
    second = module.LaneMapLocalizer(np.empty((0, 2)))
    first._index, second._index = python, compiled
    monkeypatch.setattr(module, "_GRAPH_NATIVE", None)
    old_result = first._correspondences(query, np.zeros(3))
    monkeypatch.setattr(module, "_GRAPH_NATIVE", native)
    new_result = second._correspondences(query, np.zeros(3))
    for old, new in zip(old_result, new_result):
        np.testing.assert_array_equal(old, new)
