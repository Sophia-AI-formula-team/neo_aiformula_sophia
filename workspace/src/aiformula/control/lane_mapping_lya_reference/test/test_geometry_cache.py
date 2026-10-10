"""Synthetic cache equivalence against an independently rebuilt real index.

No pose, matching gate, plant parameter or timer is mocked. These tests exercise
geometry memoization only; passing them is not a runtime or driving benchmark.
"""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from lane_mapping_lya_reference.mask_localization import _FrozenIndex, _configuration


GEOMETRY = ("normal_valid", "normals", "line_points", "normal_radii",
            "normal_thickness", "normal_position_support", "direction_borrowed")


def line(y=0., start=-2., stop=2., spacing=.1):
    x = np.arange(start, stop + spacing * .25, spacing)
    return np.column_stack((x, np.full(len(x), y)))


def square():
    x, y = np.meshgrid(np.arange(-.15, .151, .05), np.arange(-.15, .151, .05))
    return np.column_stack((x.ravel(), y.ravel()))


def lanes():
    return np.vstack((line(-1.), line(1.)))


def index(points, **kwargs):
    return _FrozenIndex(points, _configuration(None), **kwargs)


def snapshot(value):
    return tuple(getattr(value, name).copy() for name in ("anchor",) + GEOMETRY)


def assert_unchanged(value, before):
    for actual, expected in zip(snapshot(value), before):
        np.testing.assert_array_equal(actual, expected)


def assert_equivalent(actual, expected, targets=None):
    selected = slice(None) if targets is None else targets
    for name in GEOMETRY:
        np.testing.assert_array_equal(getattr(actual, name)[selected],
                                      getattr(expected, name)[selected], err_msg=name)
    if targets is None:
        assert actual.thickness_p95_m == expected.thickness_p95_m


def test_same_anchor_reuses_geometry_without_mutating_or_aliasing_source():
    points = lanes()
    old = index(points)
    before = snapshot(old)
    cached, fresh = index(points.copy(), reuse_index=old), index(points.copy())
    assert cached.reused_geometry_points == len(points)
    assert_equivalent(cached, fresh)
    assert_unchanged(old, before)
    for name in GEOMETRY:
        assert not np.shares_memory(getattr(cached, name), getattr(old, name))


@pytest.mark.parametrize("change", ["move", "add", "delete"])
@pytest.mark.parametrize("where", ["near", "far"])
def test_changed_geometry_invalidates_affected_neighborhood_only(change, where):
    points = np.vstack((lanes(), line(1., 15., 19.)))
    old = index(points)
    before = snapshot(old)
    target = np.array([0. if where == "near" else 17., 1.])
    changed = int(np.argmin(np.linalg.norm(points - target, axis=1)))
    revised = points.copy()
    if change == "move":
        revised[changed, 1] += .12
    elif change == "add":
        revised = np.vstack((revised, target + [0., .12]))
    else:
        revised = np.delete(revised, changed, axis=0)
    cached, fresh = index(revised, reuse_index=old), index(revised)
    assert 0 < cached.reused_geometry_points < len(revised)
    assert_equivalent(cached, fresh)
    assert_unchanged(old, before)


def test_reordered_points_do_not_copy_geometry_by_stale_global_indices():
    points = lanes()
    old = index(points)
    revised = np.roll(points[::-1], 7, axis=0).copy()
    assert_equivalent(index(revised, reuse_index=old), index(revised))


@pytest.mark.parametrize("replacement", ["vertical", "absent"])
def test_changed_past_context_never_reuses_old_borrowed_direction(replacement):
    points = square()
    horizontal = line(spacing=.05)
    old = index(points, direction_support=horizontal)
    before = snapshot(old)
    assert np.count_nonzero(old.direction_borrowed) == len(points)
    support = horizontal[:, ::-1].copy() if replacement == "vertical" else None
    cached = index(points.copy(), direction_support=support, reuse_index=old)
    fresh = index(points.copy(), direction_support=support)
    assert cached.reused_geometry_points == 0
    assert_equivalent(cached, fresh)
    if replacement == "vertical":
        assert np.all(cached.direction_borrowed)
        assert np.max(np.abs(np.sum(old.normals * cached.normals, axis=1))) < 1e-12
    else:
        assert not np.any(cached.direction_borrowed)
        assert not np.any(cached.normal_valid)
    assert_unchanged(old, before)


def test_new_past_context_fills_cached_invalid_geometry():
    points = square()
    old = index(points)
    assert not np.any(old.normal_valid)
    support = line(spacing=.05)
    cached = index(points.copy(), direction_support=support, reuse_index=old)
    fresh = index(points.copy(), direction_support=support)
    assert np.all(cached.direction_borrowed)
    # Own-PCA failure is now memoized before the hint admission check. Reusing
    # that exact failure must not prevent fresh context from filling its missing
    # direction; it does not count as reusing the old hint or valid geometry.
    assert cached.reused_geometry_points == len(points)
    assert_equivalent(cached, fresh)


def test_genuinely_computed_invalid_geometry_can_be_reused():
    points = square()
    old = index(points)
    assert not np.any(old.normal_valid)
    cached = index(points.copy(), reuse_index=old)
    assert cached.reused_geometry_points == len(points)
    assert_equivalent(cached, index(points.copy()))


def test_old_invalid_target_is_recomputed_when_new_neighbors_form_a_line():
    points = np.array([[-.1, 0.], [0., 0.], [.1, 0.]])
    old = index(points)
    assert not np.any(old.normal_valid)
    revised = np.vstack((points, line(start=.2, stop=2.), line(start=-2., stop=-.2)))
    cached, fresh = index(revised, reuse_index=old), index(revised)
    assert np.all(cached.normal_valid[:len(points)])
    assert_equivalent(cached, fresh)


@pytest.mark.parametrize("old_requested", ["none", "one", "other_lane"])
def test_uncomputed_targets_are_not_reused_as_computed_invalid(old_requested):
    points = lanes()
    requested = {"none": [], "one": [0], "other_lane": list(range(len(points) // 2))}[old_requested]
    old = index(points, normal_targets=np.asarray(requested, dtype=np.int64))
    before = snapshot(old)
    assert np.count_nonzero(old.normal_valid) == len(requested)
    cached, fresh = index(points.copy(), reuse_index=old), index(points.copy())
    assert np.all(fresh.normal_valid)
    assert_equivalent(cached, fresh)
    assert cached.reused_geometry_points == len(requested)
    assert_unchanged(old, before)


def test_new_requested_target_is_computed_after_a_different_partial_index():
    points = lanes()
    old = index(points, normal_targets=np.array([0], dtype=np.int64))
    target = np.array([len(points) - 1], dtype=np.int64)
    fresh = index(points.copy(), normal_targets=target)
    cached = index(points.copy(), normal_targets=target, reuse_index=old)
    assert np.all(fresh.normal_valid[target])
    assert_equivalent(cached, fresh, target)


def test_new_normal_query_is_computed_after_a_different_query_index():
    points = lanes()
    old = index(points, normal_queries=points[[0]])
    queries = points[[-1]].copy()
    fresh = index(points.copy(), normal_queries=queries)
    cached = index(points.copy(), normal_queries=queries, reuse_index=old)
    np.testing.assert_array_equal(cached.query_nearest, fresh.query_nearest)
    assert_equivalent(cached, fresh, fresh.query_nearest)
    assert np.all(cached.normal_valid[cached.query_nearest])


def test_changed_geometry_configuration_cannot_reuse_old_results():
    points = lanes()
    old = index(points)
    config = _configuration({"normal_radius_m": .6})
    cached = _FrozenIndex(points.copy(), config, reuse_index=old)
    fresh = _FrozenIndex(points.copy(), config)
    assert cached.reused_geometry_points == 0
    assert_equivalent(cached, fresh)


def u_shape():
    # The connector is outside both test circles. Global connectedness must
    # not merge the two locally disjoint arms into a false central paint band.
    return np.vstack((line(-.15), line(.15), [[2., 0.]]))


def padded(points, count):
    number = count - len(points)
    row = np.arange(number)
    remote = np.column_stack((20. + (row % 70) * .5, 20. + (row // 70) * .5))
    return np.vstack((points, remote))


def test_4000_and_4001_graph_paths_produce_exact_same_local_u_geometry():
    points = u_shape()
    targets = np.array([np.argmin(np.linalg.norm(points - query, axis=1))
                        for query in ([0., -.15], [0., .15])])
    small = index(padded(points, 4000), normal_targets=targets)
    large = index(padded(points, 4001), normal_targets=targets)
    assert small._adjacency_bits is not None and large._adjacency_bits is None
    assert np.all(small.normal_valid[targets])
    assert_equivalent(small, large, targets)
    np.testing.assert_allclose(small.line_points[targets, 1], points[targets, 1], atol=1e-14)
    for fresh, total in ((small, 4000), (large, 4001)):
        cached = index(padded(points, total), normal_targets=targets, reuse_index=fresh)
        assert_equivalent(cached, fresh, targets)


def test_remote_u_connection_change_does_not_change_cached_local_centers():
    points = u_shape()
    old = index(points)
    revised = points[:-1].copy()
    cached, fresh = index(revised, reuse_index=old), index(revised)
    assert cached.reused_geometry_points > 0
    assert_equivalent(cached, fresh)
