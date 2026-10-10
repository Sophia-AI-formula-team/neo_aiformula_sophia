"""Exact current-image cross-section density, independent of map association.

The oracle deliberately uses a full distance matrix and ordinary Python sets,
not the production packed graph, global components, forest shortcut or flood.
These small synthetic tests are geometry checks, not runtime acceptance.
"""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from lane_mapping_lya_reference.mask_localization import (
    LaneMapLocalizer, _FrozenIndex, _configuration, _line_weights,
)


def reference_weights(cloud, support_ids, normals, cfg):
    """Full, uncached induced graph; count every reliable observation."""
    difference = cloud[:, None, :] - cloud[None, :, :]
    adjacency = np.sum(difference * difference, axis=2) <= cfg["normal_connectivity_m"] ** 2
    result = []
    for seed, normal in zip(support_ids, normals):
        delta = cloud - cloud[seed]
        tangent = np.array([-normal[1], normal[0]])
        allowed = ((np.abs(np.sum(delta * tangent, axis=1)) <= cfg["scan_voxel_m"] * .5)
                   & (np.sum(delta * delta, axis=1) <= cfg["normal_max_radius_m"] ** 2))
        reached = {int(seed)}
        pending = [int(seed)]
        while pending:
            current = pending.pop()
            for other in np.flatnonzero(adjacency[current] & allowed):
                other = int(other)
                if other not in reached:
                    reached.add(other)
                    pending.append(other)
        # Do not turn support_ids into a set: repeated associations are votes.
        result.append(1. / sum(int(value) in reached for value in support_ids))
    return np.asarray(result)


def general_weights(index, support_ids, normals, cfg):
    """The pre-optimization general local-support formulation, frozen here."""
    cloud = index.anchor
    delta = cloud[None, :, :] - cloud[support_ids, None, :]
    tangent = np.column_stack((-normals[:, 1], normals[:, 0]))
    allowed = ((np.abs(np.einsum("nki,ni->nk", delta, tangent)) <= cfg["scan_voxel_m"] * .5)
               & (np.sum(delta * delta, axis=2) <= cfg["normal_max_radius_m"] ** 2)
               & (index.components[support_ids, None] == index.components[None, :]))
    connected = index._local_support(support_ids, np.arange(len(cloud)), allowed)
    return 1. / np.sum(connected[:, support_ids], axis=1)


def assert_exact(cloud, support_ids, normals, points=None):
    cfg = _configuration(None)
    cloud = np.asarray(cloud, dtype=np.float64)
    support_ids = np.asarray(support_ids, dtype=np.int64)
    normals = np.asarray(normals, dtype=np.float64)
    if points is None:
        points = cloud[support_ids]
    index = _FrozenIndex(cloud, cfg, normal_targets=np.empty(0, dtype=np.int64))
    before = cloud.tobytes()
    expected = reference_weights(cloud, support_ids, normals, cfg)
    np.testing.assert_array_equal(general_weights(index, support_ids, normals, cfg), expected)
    actual = _line_weights(points, normals, index, support_ids, cfg)
    np.testing.assert_array_equal(actual, expected)
    assert cloud.tobytes() == before
    return actual


@pytest.mark.parametrize("bridge_x", [.6, 3.])
@pytest.mark.parametrize("joined", [False, True])
@pytest.mark.parametrize("angle", [0., .13])
def test_near_and_remote_u_cannot_merge_separate_cross_sections(bridge_x, joined, angle):
    # The near bridge fits inside the radius but outside the tangent slab.
    x = np.arange(0., 3.001, .1)
    cloud = np.vstack([np.column_stack((x, np.full(len(x), y))) for y in (-.3, .3)])
    if joined:
        cloud = np.vstack((cloud, [[bridge_x, -.15], [bridge_x, 0.], [bridge_x, .15]]))
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    cloud = cloud @ rotation.T
    normals = np.tile(np.array([0., 1.]) @ rotation.T, (2, 1))
    weights = assert_exact(cloud, [0, len(x)], normals)
    np.testing.assert_array_equal(weights, [1., 1.])


def test_real_connected_cross_section_counts_only_reliable_vertices():
    # Intermediate full-support vertices make a genuine visible connection;
    # they provide structure, but not additional denominator votes.
    cloud = np.column_stack((np.zeros(9), np.arange(9) * .1))
    weights = assert_exact(cloud, [0, 4, 8], np.tile([0., 1.], (3, 1)))
    np.testing.assert_array_equal(weights, np.full(3, 1. / 3.))


def test_duplicate_support_ids_preserve_observation_multiplicity():
    cloud = np.array([[0., 0.], [0., .1], [0., .2], [0., .8]])
    weights = assert_exact(cloud, [0, 1, 1, 2, 3], np.tile([0., 1.], (5, 1)))
    np.testing.assert_array_equal(weights, [.25, .25, .25, .25, 1.])


def test_slab_uses_full_support_seed_not_shifted_reliable_voxel_mean():
    # Both means can occupy the same .1 m voxel, but shifting the slab to the
    # reliable mean would exclude the vertical bridge and split these votes.
    cloud = np.array([[.001, .001], [.001, .15], [.001, .3]])
    ids = np.array([0, 2])
    points = cloud[ids] + [.098, 0.]
    assert np.array_equal(np.floor(points / .1), np.floor(cloud[ids] / .1))
    weights = assert_exact(cloud, ids, np.tile([0., 1.], (2, 1)), points=points)
    np.testing.assert_array_equal(weights, [.5, .5])


@pytest.mark.parametrize("side", [-1., 1.])
@pytest.mark.parametrize("position", ["inside", "on", "outside"])
def test_slab_nextafter_boundary_is_inclusive_and_exact(side, position):
    edge = .05
    distance = {"inside": np.nextafter(edge, 0.), "on": edge,
                "outside": np.nextafter(edge, np.inf)}[position]
    cloud = np.array([[0., 0.], [side * distance, .1]])
    weights = assert_exact(cloud, [0, 1], np.tile([0., 1.], (2, 1)))
    expected = 1. if position == "outside" else .5
    np.testing.assert_array_equal(weights, [expected, expected])


@pytest.mark.parametrize("side", [-1., 1.])
@pytest.mark.parametrize("position", ["inside", "on", "outside"])
def test_radius_nextafter_boundary_is_inclusive_and_exact(side, position):
    edge = 1.35
    distance = {"inside": np.nextafter(edge, 0.), "on": edge,
                "outside": np.nextafter(edge, np.inf)}[position]
    # A connected full-support chain reaches the boundary reliable vertex.
    y = np.append(np.arange(0., 1.31, .15), distance) * side
    cloud = np.column_stack((np.zeros(len(y)), y))
    weights = assert_exact(cloud, [0, len(y) - 1], np.tile([0., 1.], (2, 1)))
    expected = 1. if position == "outside" else .5
    np.testing.assert_array_equal(weights, [expected, expected])


@pytest.mark.parametrize("position", ["inside", "on", "outside"])
def test_connectivity_nextafter_boundary_is_inclusive_and_exact(position):
    edge = .22
    distance = {"inside": np.nextafter(edge, 0.), "on": edge,
                "outside": np.nextafter(edge, np.inf)}[position]
    weights = assert_exact([[0., 0.], [0., distance]], [0, 1], np.tile([0., 1.], (2, 1)))
    expected = 1. if position == "outside" else .5
    np.testing.assert_array_equal(weights, [expected, expected])


@pytest.mark.parametrize("seed", range(4))
def test_random_graph_reordering_and_batch_boundary_match_uncached_oracle(seed):
    rng = np.random.default_rng(seed)
    cloud = rng.uniform(-1., 1., (139, 2))
    ids = rng.permutation(len(cloud))[:71]  # spans the production 64-row batch
    ids[-2:] = ids[:2]  # mixed duplicate/nonduplicate associations
    angles = rng.uniform(-np.pi, np.pi, len(ids))
    normals = np.column_stack((np.cos(angles), np.sin(angles)))
    weights = assert_exact(cloud, ids, normals)
    order = rng.permutation(len(cloud))
    inverse = np.empty(len(cloud), dtype=np.int64)
    inverse[order] = np.arange(len(cloud))
    np.testing.assert_array_equal(assert_exact(cloud[order], inverse[ids], normals), weights)


@pytest.mark.parametrize("joined", [False, True])
def test_near_u_match_rejects_at_overlap_not_later_geometry_gate(joined):
    x = 1.6 + np.arange(46) * .11
    offsets = (-.1, 0., .1)
    arms = [np.column_stack((x, np.full(len(x), y + offset)))
            for y in (-.3, .3) for offset in offsets]
    good = [np.column_stack((x[:9], np.full(9, 2. + offset))) for offset in offsets]
    scan = np.vstack([arm[:7] for arm in arms] + good)
    bridge = np.column_stack((np.full(3, 2.6), [-.15, 0., .15]))
    support = np.vstack(arms + good + ([bridge] if joined else []))
    anchor_x = np.arange(0., 8., .11)
    anchor = np.vstack([np.column_stack((anchor_x, np.full(len(anchor_x), 2. + offset)))
                        for offset in offsets])
    matcher = LaneMapLocalizer(anchor)
    assert matcher.config["runtime_budget_ms"] == 40.
    assert matcher.config["min_overlap_ratio"] == .55
    result = matcher.match(scan, [0., 0., 0.], 1, commit=False,
                           scan_support_xy=support, scan_sensitivity=np.full(len(scan), .01))
    assert not result.accepted
    assert result.reason == "insufficient_overlap", result.metrics
    assert result.metrics["scan_eligible_pixels"] == (67 if joined else 69)
    assert result.metrics["scan_feature_weight_sum"] == pytest.approx(23.)
    assert result.metrics["overlap_ratio"] == pytest.approx(9. / 23.)
    assert matcher._pending_result is None
    assert matcher.last_accepted_stamp_ns is None


def test_empty_reliable_scan_has_empty_weights():
    weights = assert_exact(np.empty((0, 2)), [], np.empty((0, 2)))
    assert weights.shape == (0,)
