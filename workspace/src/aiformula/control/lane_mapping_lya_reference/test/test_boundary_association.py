"""Finite geometric association oracles, independent of captured-prefix quality."""
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from lane_mapping_lya_reference.paint_geometry import BoundaryReference
from lane_mapping_lya_reference.boundary_localization import FrozenBoundaryIndex
from lane_mapping_lya_reference.mask_localization import _configuration


def index(anchors, segments, owners, normals=None):
    n = len(anchors)
    reference = BoundaryReference(np.asarray(anchors, dtype=float),
        np.tile([0., 1.], (n, 1)) if normals is None else np.asarray(normals, dtype=float),
        np.asarray(segments, dtype=float), np.asarray(owners, dtype=np.int64),
        np.full(n, 5), np.ones(n, dtype=np.int64), np.arange(n), 1000000000)
    return FrozenBoundaryIndex(reference, _configuration(None))


def test_nearest_mean_must_not_steal_a_neighbours_observed_span():
    target = index([[-.04, 0.], [.10, 0.]],
        [[[.035, 0.], [.055, 0.]], [[.08, 0.], [.12, 0.]]], [0, 1])
    query = np.array([[.052, .01]])
    # B's mean is closer; A's actual finite mean-line segment is closer.
    assert np.argmin(np.linalg.norm(target.anchor-query, axis=1)) == 1
    nearest = target.nearest(query, np.array([[0., 1.]]))
    np.testing.assert_array_equal(nearest, [0])
    assert target.covered(query, nearest)[0]


def test_closest_endpoint_is_rejected_without_farther_interior_rescue():
    target = index([[.05, 0.], [.15, .2]],
        [[[0., 0.], [.1, 0.]], [[0., .2], [.3, .2]]], [0, 1])
    query = np.array([[.15, .01]])
    nearest = target.nearest(query, np.array([[0., 1.]]))
    np.testing.assert_array_equal(nearest, [0])
    assert not target.covered(query, nearest)[0]
    assert target.covered(query, np.array([1]))[0]


def test_disconnected_intervals_do_not_become_a_hull():
    target = index([[0., 0.]], [[[-.6, 0.], [-.2, 0.]], [[.2, 0.], [.6, 0.]]], [0, 0])
    query = np.array([[0., .01]])
    nearest = target.nearest(query, np.array([[0., 1.]]))
    np.testing.assert_array_equal(nearest, [0])
    assert not target.covered(query, nearest)[0]


def test_broadphase_indexes_finite_support_not_only_consensus_mean():
    target = index([[-2., 0.]], [[[0., 0.], [.1, 0.]]], [0])
    query = np.array([[.05, .01]])
    nearest = target.nearest(query, np.array([[0., 1.]]))
    np.testing.assert_array_equal(nearest, [0])
    assert target.covered(query, nearest)[0]


def test_signed_normals_and_stable_tie_order_are_preserved():
    target = index([[0., 0.], [0., 0.], [0., 0.]],
        [[[-.1, 0.], [.1, 0.]]] * 3, [0, 1, 2], [[0., -1.], [0., 1.], [0., 1.]])
    query = np.array([[0., .01], [0., -.01]])
    np.testing.assert_array_equal(target.nearest(query, np.array([[0., 1.], [0., -1.]])), [1, 0])


@pytest.mark.parametrize('distance', [.65, np.nextafter(.65, np.inf)])
def test_original_distance_gate_applies_to_the_finite_geometry(distance):
    target = index([[0., 0.]], [[[-.1, 0.], [.1, 0.]]], [0])
    actual = target.nearest(np.array([[0., distance]]), np.array([[0., 1.]]))
    np.testing.assert_array_equal(actual, [0 if distance <= .65 else -1])


def test_against_literal_signed_point_to_finite_segment_oracle():
    rng = np.random.default_rng(20261011)
    anchors = rng.uniform(-2., 2., (60, 2))
    angles = rng.uniform(-np.pi, np.pi, 60)
    normals = np.column_stack((np.cos(angles), np.sin(angles)))
    tangents = np.column_stack((-normals[:, 1], normals[:, 0]))
    intervals = rng.uniform(-.6, .6, (60, 2, 2))
    intervals.sort(axis=2)
    segments = [np.stack((anchors[i]+tangents[i]*a, anchors[i]+tangents[i]*b))
                for i in range(60) for a, b in intervals[i]]
    target = index(anchors, segments, np.repeat(np.arange(60), 2), normals)
    queries = rng.uniform(-2.5, 2.5, (90, 2))
    query_angles = rng.uniform(-np.pi, np.pi, len(queries))
    directions = np.column_stack((np.cos(query_angles), np.sin(query_angles)))
    expected = []
    for point, direction in zip(queries, directions):
        best, owner = np.inf, -1
        for i, spans in enumerate(target.intervals):
            if direction @ normals[i] < np.cos(np.pi/6.):
                continue
            for a, b in spans:
                start, end = anchors[i]+tangents[i]*a, anchors[i]+tangents[i]*b
                edge = end-start
                alpha = np.clip((point-start) @ edge / (edge @ edge), 0., 1.)
                squared = float(np.sum((point-(start+alpha*edge))**2))
                if squared < best:
                    best, owner = squared, i
        expected.append(owner if best <= .65**2 else -1)
    np.testing.assert_array_equal(target.nearest(queries, directions), expected)
