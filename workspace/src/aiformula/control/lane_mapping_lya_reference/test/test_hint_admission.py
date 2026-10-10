"""Exact admission oracle for past direction, not a runtime/accuracy benchmark.

The oracle computes EVERY support normal without cache, then admits hints only
for confirmed-anchor points whose own PCA failed. Production may compute only
queried normals, but must produce identical geometry. All numeric gates remain
at production defaults; no timer, sensor, pose, or map input is substituted.
"""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

import lane_mapping_lya_reference.mask_localization as localization


GEOMETRY = ("normals", "line_points", "normal_valid", "normal_radii",
            "normal_thickness", "normal_position_support", "direction_borrowed")


def line(y=0., start=-3., stop=3., spacing=.05):
    x = np.arange(start, stop + spacing * .25, spacing)
    return np.column_stack((x, np.full(len(x), y)))


def square(center=(0., 0.)):
    x, y = np.meshgrid(np.arange(-.15, .151, .05), np.arange(-.15, .151, .05))
    return np.column_stack((x.ravel(), y.ravel())) + center


def index(points, **kwargs):
    return localization._FrozenIndex(points, localization._configuration(None), **kwargs)


def full_uncached_oracle(anchor, support):
    """Independent full-query policy reference, without the production hint method."""
    own = index(anchor.copy())
    hint = index(support.copy())  # ALL support directions, never queried-subset.
    result = {name: getattr(own, name).copy() for name in GEOMETRY}
    cfg = own.cfg
    nearest = hint.nearest(anchor, valid_only=False)
    covered = (nearest >= 0) & ~own.normal_valid
    covered[covered] &= hint.normal_valid[nearest[covered]]
    covered[covered] &= np.linalg.norm(anchor[covered] - support[nearest[covered]], axis=1) <= cfg["normal_connectivity_m"]
    for key, all_indices in own.cells.items():
        chosen = np.asarray(all_indices, dtype=np.int64)
        chosen = chosen[covered[chosen]]
        if not len(chosen):
            continue
        candidates = own._patch(key, cfg["normal_max_radius_m"])
        difference = anchor[candidates][None, :, :] - anchor[chosen, None, :]
        eligible = ((np.sum(difference * difference, axis=2) <= cfg["normal_max_radius_m"] ** 2)
                    & (own.components[chosen, None] == own.components[candidates][None, :]))
        eligible = own._local_support(chosen, candidates, eligible)
        counts = np.sum(eligible, axis=1)
        normals = hint.normals[nearest[chosen]]
        projected = np.einsum("nki,ni->nk", difference, normals)
        mean = np.sum(projected * eligible, axis=1) / np.maximum(counts, 1)
        variance = np.sum((projected - mean[:, None]) ** 2 * eligible, axis=1) / np.maximum(counts, 1)
        valid = ((counts >= cfg["normal_min_points"])
                 & (variance <= cfg["normal_max_thickness_m"] ** 2))
        selected = chosen[valid]
        result["normals"][selected] = normals[valid]
        result["direction_borrowed"][selected] = True
        result["normal_valid"][selected] = True
        result["normal_thickness"][selected] = np.sqrt(np.maximum(variance[valid], 0.))
        result["normal_position_support"][selected] = counts[valid]
        result["normal_radii"][selected] = cfg["normal_max_radius_m"]
        result["line_points"][selected] = anchor[selected] + normals[valid] * mean[valid, None]
    return own, result


def scene(kind):
    anchor = np.vstack((square(), line(3.)))
    support = np.vstack((line(), line(3.)[:, ::-1]))
    if kind == "remote":
        support = support + [20., 20.]
    elif kind == "dense_patch":
        support = np.vstack([line(y, -3., 3.) for y in np.arange(-.2, .201, .05)])
    elif kind == "u_remote":
        support = np.vstack((line(-.15), line(.15), [[3., 0.]]))
        anchor = np.vstack((square(), line(3.), square((0., 6.))))
        support = np.vstack((support, line(6.)))
    elif kind == "cell_boundary":
        anchor = anchor + [.499999999, -.500000001]
        support = support + [.499999999, -.500000001]
    elif kind == "rotated":
        angle = .37
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        anchor, support = anchor @ rotation.T, support @ rotation.T
    elif kind == "reordered":
        anchor, support = anchor[::-1].copy(), np.roll(support, 11, axis=0)
    return anchor, support


@pytest.mark.parametrize("kind", ["ordinary", "remote", "dense_patch", "u_remote", "cell_boundary", "rotated", "reordered"])
@pytest.mark.parametrize("reuse", [False, True])
def test_subset_hint_queries_exactly_match_full_uncached_reference(kind, reuse):
    anchor, support = scene(kind)
    own, expected = full_uncached_oracle(anchor, support)
    before = {name: getattr(own, name).copy() for name in GEOMETRY}
    actual = index(anchor.copy(), direction_support=support, reuse_index=own if reuse else None)
    for name in GEOMETRY:
        np.testing.assert_array_equal(getattr(actual, name), expected[name], err_msg=name)
        np.testing.assert_array_equal(getattr(own, name), before[name], err_msg="source mutated: " + name)
        assert not np.shares_memory(getattr(actual, name), getattr(own, name))
        assert not getattr(actual, name).flags.writeable
    assert actual.direction_supported_points == int(np.sum(expected["direction_borrowed"]))
    for name in GEOMETRY:
        np.testing.assert_array_equal(getattr(actual, name)[own.normal_valid],
                                      getattr(own, name)[own.normal_valid], err_msg="valid geometry replaced: " + name)


def test_fully_valid_anchor_skips_hint_index_even_when_hint_direction_conflicts(monkeypatch):
    anchor = line(spacing=.1)
    support = anchor[:, ::-1].copy()
    own = index(anchor)
    assert np.all(own.normal_valid)
    calls = []
    constructor = localization._FrozenIndex.__init__

    def spy(self, points, cfg, *args, **kwargs):
        calls.append(dict(kwargs))
        return constructor(self, points, cfg, *args, **kwargs)
    monkeypatch.setattr(localization._FrozenIndex, "__init__", spy)
    actual = index(anchor.copy(), direction_support=support)
    assert len(calls) == 1, "valid geometry must not build or consume a hint index"
    assert actual.direction_supported_points == 0
    for name in GEOMETRY:
        np.testing.assert_array_equal(getattr(actual, name), getattr(own, name))


def test_hint_queries_only_unresolved_anchor_points_and_keep_whole_support(monkeypatch):
    anchor, support = scene("ordinary")
    own = index(anchor)
    unresolved = anchor[~own.normal_valid]
    assert 0 < len(unresolved) < len(anchor)
    calls = []
    constructor = localization._FrozenIndex.__init__

    def spy(self, points, cfg, *args, **kwargs):
        if kwargs.get("normal_queries") is not None:
            calls.append((points.copy(), kwargs["normal_queries"].copy()))
        return constructor(self, points, cfg, *args, **kwargs)
    monkeypatch.setattr(localization._FrozenIndex, "__init__", spy)
    actual = index(anchor.copy(), direction_support=support)
    assert len(calls) == 1
    np.testing.assert_array_equal(calls[0][0], support)
    np.testing.assert_array_equal(calls[0][1], unresolved)
    assert np.all(actual.direction_borrowed[~own.normal_valid])
    assert not np.any(actual.direction_borrowed[own.normal_valid])


def test_new_context_does_not_reuse_borrowed_direction_from_old_context():
    anchor = square()
    horizontal = line()
    vertical = horizontal[:, ::-1].copy()
    old = index(anchor, direction_support=horizontal)
    old_geometry = {name: getattr(old, name).copy() for name in GEOMETRY}
    actual = index(anchor.copy(), direction_support=vertical, reuse_index=old)
    _, expected = full_uncached_oracle(anchor, vertical)
    assert np.all(actual.direction_borrowed)
    assert actual.reused_geometry_points == 0
    assert np.max(np.abs(np.sum(old.normals * actual.normals, axis=1))) < 1e-12
    for name in GEOMETRY:
        np.testing.assert_array_equal(getattr(actual, name), expected[name])
        np.testing.assert_array_equal(getattr(old, name), old_geometry[name])


def test_context_never_supplies_missing_confirmed_position_count():
    anchor = np.array([[-.05, -.05], [-.05, .05], [.05, -.05], [.05, .05]])
    actual = index(anchor, direction_support=line())
    assert len(anchor) < actual.cfg["normal_min_points"]
    assert not np.any(actual.normal_valid)
    assert not np.any(actual.direction_borrowed)
    assert actual.direction_supported_points == 0
