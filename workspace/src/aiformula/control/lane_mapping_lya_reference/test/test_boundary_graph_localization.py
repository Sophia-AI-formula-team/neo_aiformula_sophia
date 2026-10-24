"""Connected finite consensus geometry; no added tangential information."""
from pathlib import Path
import sys
from dataclasses import replace
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from lane_mapping_lya_reference.paint_geometry import BoundaryReference
from lane_mapping_lya_reference.boundary_localization import BoundaryLaneLocalizer


def graph(points, normals, edges):
    xy = np.asarray(points, dtype=float)
    normal = np.asarray(normals, dtype=float)
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    pairs = np.asarray(edges, dtype=np.int64).reshape(-1, 2)
    count, links = len(xy), len(pairs)
    return BoundaryReference(xy, normal, np.empty((0, 2, 2)), np.empty(0, dtype=np.int64),
        np.full(count, 10), np.full(count, 5, dtype=np.int64), np.arange(count), 1000000000,
        directed_edges=pairs, edge_frame_votes=np.full(links, 5),
        edge_reliable_votes=np.full(links, 2, dtype=np.int64),
        edge_last_stamp_ns=np.full(links, 1000000000, dtype=np.int64))


def test_smooth_shared_vertex_is_one_normal_factor_not_a_false_frontier():
    reference = graph([[-.1, .005], [0., 0.], [.1, .005]],
                      [[-.1, -1.], [0., -1.], [.1, -1.]], [[0, 1], [1, 2]])
    matcher = BoundaryLaneLocalizer(reference)
    _, _, selected, normals, residual = matcher._correspondences(
        np.array([[0., -.02]]), np.zeros(3), np.array([[0., -1.]]))
    np.testing.assert_array_equal(selected, [0])
    np.testing.assert_array_equal(normals, [[0., -1.]])
    np.testing.assert_allclose(residual, [.02], atol=1e-15, rtol=0)
    # Exactly one normal residual; no extra x/tangent measurement at the node.
    assert normals.shape == (1, 2) and residual.shape == (1,)


def test_interior_residual_uses_the_same_chord_as_nearest_distance():
    reference = graph([[0., 0.], [1., .1]], [[0., 1.], [0., 1.]], [[0, 1]])
    matcher = BoundaryLaneLocalizer(reference)
    point = np.array([[.5, .08]])
    _, _, selected, normals, residual = matcher._correspondences(point, np.zeros(3), np.array([[0., 1.]]))
    expected = np.array([-.1, 1.])/np.sqrt(1.01)
    np.testing.assert_array_equal(selected, [0])
    np.testing.assert_allclose(normals[0], expected, rtol=0, atol=1e-15)
    np.testing.assert_allclose(residual, point @ expected, rtol=0, atol=1e-15)


def test_nearest_true_terminal_blocks_farther_supported_chord():
    reference = graph([[0., 0.], [.1, 0.], [-1., .3], [1., .3]],
                      [[0., 1.]]*4, [[0, 1], [2, 3]])
    matcher = BoundaryLaneLocalizer(reference)
    _, _, selected, _, _ = matcher._correspondences(
        np.array([[-.05, .01]]), np.zeros(3), np.array([[0., 1.]]))
    assert not len(selected)
    assert matcher._last_correspondence_support_metrics['nearest_correspondences_before_support'] == 1


def test_subset_cut_does_not_remain_an_internal_joint():
    reference = graph([[-.1, .005], [0., 0.], [.1, .005]],
                      [[-.1, -1.], [0., -1.], [.1, -1.]], [[0, 1], [1, 2]])
    matcher = BoundaryLaneLocalizer(reference.subset(np.array([0, 1])))
    _, _, selected, _, _ = matcher._correspondences(
        np.array([[0., -.02]]), np.zeros(3), np.array([[0., -1.]]))
    assert not len(selected)


def test_explicit_empty_graph_never_falls_back_to_old_pixel_face_spans():
    reference = graph([[0., 0.]], [[0., 1.]], [])
    reference = replace(reference, support_segments=np.array([[[-1., 0.], [1., 0.]]]),
                        segment_owner=np.array([0], dtype=np.int64))
    matcher = BoundaryLaneLocalizer(reference)
    _, _, selected, _, _ = matcher._correspondences(
        np.array([[0., .01]]), np.zeros(3), np.array([[0., 1.]]))
    assert not len(selected)


def test_subsetting_a_branch_does_not_certify_a_fake_degree_two_joint():
    reference = graph([[-.1, .005], [0., 0.], [.1, .005], [.11, .009]],
                      [[-.1, -1.], [0., -1.], [.1, -1.], [.1, -1.]],
                      [[0, 1], [1, 2], [1, 3]])
    for frozen in (reference, reference.subset(np.array([0, 1, 2]))):
        matcher = BoundaryLaneLocalizer(frozen)
        _, _, selected, _, _ = matcher._correspondences(
            np.array([[0., -.02]]), np.zeros(3), np.array([[0., -1.]]))
        assert not len(selected)
        assert matcher._last_correspondence_support_metrics['nearest_correspondences_before_support'] == 1


def test_branch_interiors_remain_observed_geometry_not_deleted_for_nearest_search():
    reference = graph([[-.1, 0.], [0., 0.], [.1, 0.], [.1, .03]],
                      [[0., 1.]]*4, [[0, 1], [1, 2], [1, 3]])
    matcher = BoundaryLaneLocalizer(reference)
    _, _, selected, normals, residual = matcher._correspondences(
        np.array([[-.05, .01]]), np.zeros(3), np.array([[0., 1.]]))
    np.testing.assert_array_equal(selected, [0])
    np.testing.assert_allclose(residual, [.01], atol=1e-15, rtol=0)


def test_single_remaining_branch_edge_at_exact_alpha_zero_is_not_a_true_terminal():
    reference = graph([[-.1, 0.], [0., 0.], [.1, 0.], [.1, .03]],
                      [[0., 1.]]*4, [[0, 1], [1, 2], [1, 3]])
    matcher = BoundaryLaneLocalizer(reference.subset(np.array([1, 2])))
    _, _, selected, _, _ = matcher._correspondences(
        np.array([[0., .02]]), np.zeros(3), np.array([[0., 1.]]))
    assert not len(selected)
    assert matcher._last_correspondence_support_metrics['nearest_correspondences_before_support'] == 1


@pytest.mark.parametrize('points,edges', [
    ([[-.1, 0.], [0., 0.], [-.1, .01]], [[0, 1], [1, 2]]),
    ([[-.1, 0.], [0., 0.]], [[0, 1], [1, 0]])])
def test_directed_hairpin_cannot_be_certified_by_flipping_chord_normals(points, edges):
    reference = graph(points, [[0., 1.]]*len(points), edges)
    matcher = BoundaryLaneLocalizer(reference)
    _, _, selected, _, _ = matcher._correspondences(
        np.array([[.02, 0.]]), np.zeros(3), np.array([[0., 1.]]))
    assert not len(selected)
    assert matcher._last_correspondence_support_metrics['nearest_correspondences_before_support'] == 1


@pytest.mark.parametrize('field', ['directed_edges', 'edge_frame_votes', 'edge_reliable_votes',
                                 'edge_last_stamp_ns', 'full_in_degree', 'full_out_degree'])
def test_frozen_identity_includes_graph_evidence(field):
    reference = graph([[0., 0.], [1., 0.]], [[0., 1.], [0., 1.]], [[0, 1]])
    matcher = BoundaryLaneLocalizer(reference)
    value = getattr(reference, field).copy()
    if field == 'directed_edges':
        value = value[:, ::-1]
    elif field in ('edge_frame_votes', 'full_in_degree', 'full_out_degree'):
        value += 1
    else:
        value -= 1
    changed = {field:value}
    if field == 'directed_edges':
        changed.update(full_in_degree=None, full_out_degree=None)
    assert not matcher.matches_anchor_source(replace(reference, **changed))


@pytest.mark.parametrize('degrees', [0., 1., 7., 15., 30., 45., 60., 89., 90., 127., 179., 271.])
@pytest.mark.parametrize('translation', [[0., 0.], [100., -80.]])
def test_shared_vertex_normal_factor_is_rigid_transform_equivariant(degrees, translation):
    theta = np.deg2rad(20.)
    points = np.array([[-1., 0.], [0., 0.], [np.cos(theta), np.sin(theta)]])
    normals = np.array([[0., -1.], [np.sin(theta/2), -np.cos(theta/2)],
                        [np.sin(theta), -np.cos(theta)]])
    angle = np.deg2rad(degrees)
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    reference = graph(points @ rotation.T+translation, normals @ rotation.T, [[0, 1], [1, 2]])
    matcher = BoundaryLaneLocalizer(reference)
    _, _, selected, actual_normals, residual = matcher._correspondences(
        np.array([[0., -.1]]) @ rotation.T+translation, np.zeros(3), normals[1:2] @ rotation.T)
    np.testing.assert_array_equal(selected, [0])
    np.testing.assert_allclose(actual_normals, normals[1:2] @ rotation.T, atol=2e-15, rtol=0)
    np.testing.assert_allclose(residual, [.1*np.cos(theta/2)], atol=3e-14, rtol=0)


@pytest.mark.parametrize('translation', [[0., 0.], [100., -80.]])
def test_true_interior_and_outside_are_not_expanded_by_arithmetic_endpoint_classification(translation):
    reference = graph(np.array([[0., 0.], [1., 0.]])+translation, [[0., 1.]]*2, [[0, 1]])
    matcher = BoundaryLaneLocalizer(reference)
    points = np.array([[-1e-8, .01], [1e-8, .01], [1.-1e-8, .01], [1.+1e-8, .01]])+translation
    _, _, selected, _, _ = matcher._correspondences(points, np.zeros(3), np.array([[0., 1.]]*4))
    np.testing.assert_array_equal(selected, [1, 2])


def test_overlapping_endpoint_roundoff_bands_fail_closed():
    reference = graph([[100., 100.], [np.nextafter(100., np.inf), 100.]],
                      [[0., 1.]]*2, [[0, 1]])
    matcher = BoundaryLaneLocalizer(reference)
    _, _, selected, _, _ = matcher._correspondences(
        np.array([[100., 100.01]]), np.zeros(3), np.array([[0., 1.]]))
    assert not len(selected)


def test_arithmetic_joint_classification_does_not_relax_normal_angle_gate():
    theta = np.deg2rad(20.)
    reference = graph([[-1., 0.], [0., 0.], [np.cos(theta), np.sin(theta)]],
        [[0., -1.], [np.sin(theta/2), -np.cos(theta/2)], [np.sin(theta), -np.cos(theta)]],
        [[0, 1], [1, 2]])
    matcher = BoundaryLaneLocalizer(reference)
    _, _, selected, _, _ = matcher._correspondences(np.array([[0., -.1]]), np.zeros(3),
        np.array([[np.sin(np.deg2rad(60.)), -np.cos(np.deg2rad(60.))]]))
    assert not len(selected)


def test_nonfinite_endpoint_uncertainty_is_explicitly_invalid_not_merely_far_from_endpoint():
    from lane_mapping_lya_reference.boundary_localization import _endpoint_dot_uncertainty
    point = np.array([[1e308, 1e308]])
    _, near, valid = _endpoint_dot_uncertainty(point, -point, point, -point, point)
    np.testing.assert_array_equal(near, [False])
    np.testing.assert_array_equal(valid, [False])
