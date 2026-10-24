"""V3 finite-domain contracts, independently of domain consensus construction.

These tests supply explicit reference evidence; they do not fabricate extra
votes in BoundaryConsensus, run a pose solver, or change admission thresholds.
"""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from lane_mapping_lya_reference.paint_geometry import BoundaryReference
from lane_mapping_lya_reference.boundary_localization import BoundaryLaneLocalizer


CERTIFICATE_FIELDS = (
    "certified_model_segments", "certified_model_owner",
    "certified_model_frame_votes", "certified_model_reliable_votes",
    "certified_model_last_stamp_ns",
)
REFERENCE_FIELDS = (
    "points_xy", "normals", "support_segments", "segment_owner", "votes",
    "reliable_votes", "record_ids", "directed_edges", "edge_frame_votes",
    "edge_reliable_votes", "edge_last_stamp_ns", "full_in_degree",
    "full_out_degree",
) + CERTIFICATE_FIELDS
FRONTIER = 1000000000


def reference(points=((0., 0.),), edges=(), segments=(((-1., 0.), (1., 0.)),),
              owners=(0,), normals=None):
    xy = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    normal = (np.tile([0., 1.], (len(xy), 1)) if normals is None
              else np.asarray(normals, dtype=np.float64))
    pairs = np.asarray(edges, dtype=np.int64).reshape(-1, 2)
    spans = np.asarray(segments, dtype=np.float64).reshape(-1, 2, 2)
    count, links, certificates = len(xy), len(pairs), len(spans)
    return BoundaryReference(
        xy, normal, np.empty((0, 2, 2)), np.empty(0, dtype=np.int64),
        np.full(count, 10, dtype=np.int64), np.full(count, 5, dtype=np.int64),
        np.arange(count, dtype=np.int64)+100, FRONTIER,
        directed_edges=pairs, edge_frame_votes=np.full(links, 5, dtype=np.int64),
        edge_reliable_votes=np.full(links, 2, dtype=np.int64),
        edge_last_stamp_ns=np.full(links, FRONTIER, dtype=np.int64),
        certified_model_segments=spans,
        certified_model_owner=np.asarray(owners, dtype=np.int64),
        certified_model_frame_votes=np.full(certificates, 5, dtype=np.int64),
        certified_model_reliable_votes=np.full(certificates, 1, dtype=np.int64),
        certified_model_last_stamp_ns=np.full(certificates, FRONTIER, dtype=np.int64))


def query(frozen, points, normals=None, config=None):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    normals = (np.tile([0., 1.], (len(points), 1)) if normals is None
               else np.asarray(normals, dtype=np.float64))
    matcher = BoundaryLaneLocalizer(frozen, config)
    return matcher, matcher._index.graph.query(points, normals)


def assert_bytes(actual, expected):
    assert actual.dtype == expected.dtype
    assert actual.shape == expected.shape
    assert actual.tobytes() == expected.tobytes()


def test_explicit_empty_certificates_are_v3_not_legacy_and_never_reuse_raw_spans():
    empty = reference(segments=(), owners=())
    graph = replace(empty, **{name: None for name in CERTIFICATE_FIELDS})
    legacy = replace(graph, directed_edges=None, edge_frame_votes=None,
                     edge_reliable_votes=None, edge_last_stamp_ns=None,
                     full_in_degree=None, full_out_degree=None)
    assert (legacy.boundary_version, graph.boundary_version, empty.boundary_version) == (1, 2, 3)
    assert legacy.certified_model_representation is None
    assert graph.certified_model_representation is None
    assert isinstance(empty.certified_model_representation, str)
    assert empty.certified_model_representation
    empty = replace(empty, support_segments=np.array([[[-1., 0.], [1., 0.]]]),
                    segment_owner=np.array([0], dtype=np.int64))
    _, (_, _, _, covered) = query(empty, [[0., .01]])
    assert not covered.any()


@pytest.mark.parametrize("field", CERTIFICATE_FIELDS)
def test_partial_certificate_schema_is_rejected(field):
    with pytest.raises(ValueError):
        replace(reference(), **{field: None})


def test_certificates_require_explicit_graph_even_when_graph_is_empty():
    with pytest.raises(ValueError):
        replace(reference(), directed_edges=None, edge_frame_votes=None,
                edge_reliable_votes=None, edge_last_stamp_ns=None,
                full_in_degree=None, full_out_degree=None)


@pytest.mark.parametrize("field,value", [
    ("certified_model_segments", np.zeros((1, 2, 2))),
    ("certified_model_segments", np.array([[[-1., 0.], [np.nan, 0.]]])),
    ("certified_model_segments", np.zeros((1, 4))),
    ("certified_model_owner", np.array([-1])),
    ("certified_model_owner", np.array([1])),
    ("certified_model_owner", np.array([False])),
    ("certified_model_owner", np.array([0.])),
    ("certified_model_frame_votes", np.array([0])),
    ("certified_model_frame_votes", np.array([11])),
    ("certified_model_reliable_votes", np.array([0])),
    ("certified_model_reliable_votes", np.array([6])),
    ("certified_model_last_stamp_ns", np.array([0])),
    ("certified_model_last_stamp_ns", np.array([FRONTIER+1])),
    ("certified_model_last_stamp_ns", np.array([float(FRONTIER)])),
])
def test_certificate_evidence_is_shape_finite_owner_and_frontier_bounded(field, value):
    with pytest.raises(ValueError):
        replace(reference(), **{field: value})


def test_one_reliable_vote_cannot_exceed_its_owner_reliable_evidence():
    with pytest.raises(ValueError):
        replace(reference(), reliable_votes=np.array([0], dtype=np.int64))


def test_certificate_arrays_are_owned_immutable_bytes():
    original = reference()
    arrays = {name: getattr(original, name).copy() for name in CERTIFICATE_FIELDS}
    frozen = replace(original, **arrays)
    for name, array in arrays.items():
        before = getattr(frozen, name).tobytes()
        array.flat[0] += 1
        assert getattr(frozen, name).tobytes() == before
        with pytest.raises(ValueError):
            getattr(frozen, name).setflags(write=True)


def test_certificates_alone_supply_valid_normals_and_one_normal_residual():
    matcher, (owners, targets, normals, covered) = query(reference(), [[.2, .03]])
    np.testing.assert_array_equal(owners, [0])
    np.testing.assert_array_equal(covered, [True])
    np.testing.assert_array_equal(matcher._index.normal_valid, [True])
    np.testing.assert_allclose(targets, [[.2, 0.]], atol=1e-15, rtol=0)
    np.testing.assert_array_equal(normals, [[0., 1.]])
    _, _, selected, directions, residual = matcher._correspondences(
        np.array([[.2, .03]]), np.zeros(3), np.array([[0., 1.]]))
    np.testing.assert_array_equal(selected, [0])
    assert directions.shape == (1, 2) and residual.shape == (1,)
    np.testing.assert_allclose(residual, [.03], atol=1e-15, rtol=0)


def test_certificate_residual_normal_belongs_to_actual_segment_not_owner_plane():
    frozen = reference(segments=(((0., 0.), (1., .1)),))
    _, (_, targets, normals, covered) = query(frozen, [[.5, .08]])
    direction = np.array([-.1, 1.])/np.sqrt(1.01)
    np.testing.assert_array_equal(covered, [True])
    np.testing.assert_allclose(normals[0], direction, atol=1e-15, rtol=0)
    np.testing.assert_allclose(np.sum((np.array([[.5, .08]])-targets)*normals, axis=1),
                               [np.dot([.5, .08], direction)], atol=1e-15, rtol=0)


@pytest.mark.parametrize("query_normal", [[0., -1.], [np.sin(np.deg2rad(60.)), .5]])
def test_certificate_signed_direction_gate_is_not_relaxed(query_normal):
    _, (owners, _, _, covered) = query(reference(), [[0., .01]], [query_normal])
    np.testing.assert_array_equal(owners, [-1])
    assert not covered.any()


def test_nearer_branch_certificate_blocks_farther_valid_graph_and_survives_subset():
    frozen = reference([[0., 0.], [-1., 0.], [-1., .02], [-1., .3], [1., .3]],
                       [[0, 1], [0, 2], [3, 4]], [((0., 0.), (1., 0.))])
    for selected in (frozen, frozen.subset(np.array([0, 3, 4], dtype=np.int64))):
        _, (owners, targets, _, covered) = query(selected, [[.5, .01]])
        np.testing.assert_array_equal(owners, [0])
        np.testing.assert_allclose(targets, [[.5, 0.]], atol=1e-15, rtol=0)
        assert not covered.any()
        assert selected.full_out_degree[0] == 2


def test_nearer_graph_terminal_blocks_farther_certificate():
    frozen = reference([[0., 0.], [.1, 0.], [0., .3]], [[0, 1]],
                       [((-1., .3), (1., .3))], [2])
    _, (owners, _, _, covered) = query(frozen, [[-.05, .01]])
    np.testing.assert_array_equal(owners, [0])
    assert not covered.any()


def test_equal_distance_graph_wins_even_when_certificate_would_be_covered():
    # Both distances are exactly .125 in binary64. The graph only offers its
    # true endpoint; the certificate interior must not rescue that tie.
    frozen = reference([[0., 0.], [.5, 0.], [0., .125]], [[0, 1]],
                       [((-.5, .125), (.5, .125))], [2])
    _, (owners, targets, _, covered) = query(frozen, [[-.125, 0.]])
    np.testing.assert_array_equal(owners, [0])
    np.testing.assert_array_equal(targets, [[0., 0.]])
    assert not covered.any()


def test_certificate_endpoint_does_not_borrow_its_owners_smooth_graph_joint():
    # Shared ownership does not certify the independent span's endpoint.
    frozen = reference([[-1., .2], [0., .2], [1., .2]], [[0, 1], [1, 2]],
                       [((0., 0.), (1., 0.))], [1])
    matcher, (owners, _, _, covered) = query(frozen, [[-1e-8, .01], [1e-8, .01]])
    assert matcher._index.graph.smooth_joint[1]
    np.testing.assert_array_equal(owners, [1, 1])
    np.testing.assert_array_equal(covered, [False, True])


@pytest.mark.parametrize("degrees", [0., 30., 89., 177.])
def test_true_certificate_extent_not_expanded_under_rigid_transforms(degrees):
    angle = np.deg2rad(degrees)
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    translation = np.array([100., -80.])
    points = np.array([[0., 0.]]) @ rotation.T+translation
    spans = np.array([[[0., 0.], [1., 0.]]]) @ rotation.T+translation
    normal = np.array([[0., 1.]]) @ rotation.T
    frozen = reference(points, segments=spans, normals=normal)
    queries = np.array([[-1e-8, .01], [1e-8, .01], [1.-1e-8, .01], [1.+1e-8, .01]])
    _, (_, _, _, covered) = query(frozen, queries @ rotation.T+translation, np.repeat(normal, 4, axis=0))
    np.testing.assert_array_equal(covered, [False, True, True, False])


def test_patch_capacity_counts_union_of_graph_and_certificate_owners():
    # Graph alone has four owners, certificates three, union seven > six.
    frozen = reference([[-.1, .1], [.1, .1], [-.1, .2], [.1, .2], [0., 0.],
                        [0., .03], [0., .06]], [[0, 1], [2, 3]],
                       [((-.1, y), (.1, y)) for y in (0., .03, .06)], [4, 5, 6])
    _, (owners, _, _, covered) = query(frozen, [[0., .01]], config={"max_patch_points": 6})
    np.testing.assert_array_equal(owners, [-1])
    assert not covered.any()


def test_multiple_certificates_for_same_owner_do_not_inflate_owner_capacity():
    frozen = reference([[0., 0.]], segments=[((-.1, i*.01), (.1, i*.01)) for i in range(7)],
                       owners=[0]*7)
    _, (owners, _, _, covered) = query(frozen, [[0., .001]], config={"max_patch_points": 6})
    np.testing.assert_array_equal(owners, [0])
    np.testing.assert_array_equal(covered, [True])


@pytest.mark.parametrize("kind", ["certificates", "raw_plus_certificate", "graph_plus_certificate"])
def test_combined_reference_geometry_capacity_is_not_separate_per_new_type(kind):
    frozen = reference()
    with pytest.raises(ValueError, match="capacity"):
        if kind == "certificates":
            replace(frozen, certified_model_segments=np.zeros((200001, 2, 2)))
        elif kind == "raw_plus_certificate":
            replace(frozen, support_segments=np.zeros((200000, 2, 2)),
                    segment_owner=np.zeros(200000, dtype=np.int64))
        else:
            # 200000 distinct, non-self directed pairs. Rejection must be the
            # combined capacity, not malformed duplicate graph evidence.
            ids = np.arange(200000, dtype=np.int64)
            sources = ids//448
            pairs = np.column_stack((sources, (sources+ids%448+1)%449))
            count = len(pairs)
            base = reference(np.zeros((449, 2)), segments=(), owners=())
            replace(base, directed_edges=pairs,
                    edge_frame_votes=np.full(count, 5, dtype=np.int64),
                    edge_reliable_votes=np.full(count, 1, dtype=np.int64),
                    edge_last_stamp_ns=np.full(count, FRONTIER, dtype=np.int64),
                    full_in_degree=None, full_out_degree=None,
                    **{name: getattr(frozen, name) for name in CERTIFICATE_FIELDS})


def test_subset_remaps_all_eighteen_arrays_preserving_full_degrees_and_order():
    frozen = reference([[0., 0.], [1., 0.], [2., 0.], [3., 0.]],
                       [[0, 1], [1, 2], [1, 3]],
                       [((i-.2, 0.), (i+.2, 0.)) for i in (0., 1., 2., 3.)], [0, 1, 2, 3])
    frozen = replace(frozen, support_segments=frozen.certified_model_segments.copy(),
                     segment_owner=np.arange(4, dtype=np.int64))
    ids = np.array([3, 1], dtype=np.int64)
    subset = frozen.subset(ids)
    remap = np.array([-1, 1, -1, 0], dtype=np.int64)
    node_fields = {"points_xy", "normals", "votes", "reliable_votes", "record_ids",
                   "full_in_degree", "full_out_degree"}
    old_edges = frozen.directed_edges
    edge_mask = np.all(remap[old_edges] >= 0, axis=1)
    raw_mask = remap[frozen.segment_owner] >= 0
    certificate_mask = remap[frozen.certified_model_owner] >= 0
    for field in REFERENCE_FIELDS:
        original = getattr(frozen, field)
        if field in node_fields:
            expected = original[ids]
        elif field == "directed_edges":
            expected = remap[original[edge_mask]]
        elif field.startswith("edge_"):
            expected = original[edge_mask]
        elif field == "segment_owner":
            expected = remap[original[raw_mask]]
        elif field == "support_segments":
            expected = original[raw_mask]
        elif field == "certified_model_owner":
            expected = remap[original[certificate_mask]]
        else:
            expected = original[certificate_mask]
        assert_bytes(getattr(subset, field), expected)
    assert subset.boundary_version == 3
    assert subset.source_frontier_ns == frozen.source_frontier_ns
    assert subset.full_out_degree[1] == 2
    empty = frozen.subset(np.array([], dtype=np.int64))
    assert empty.boundary_version == 3
    assert all(getattr(empty, name).shape[0] == 0 for name in REFERENCE_FIELDS)


@pytest.mark.parametrize("field", CERTIFICATE_FIELDS)
def test_frozen_identity_rejects_certificate_only_change(field):
    frozen = reference([[0., 0.], [0., 1.]])
    matcher = BoundaryLaneLocalizer(frozen)
    assert matcher.matches_anchor_source(frozen)
    value = getattr(frozen, field).copy()
    if field == "certified_model_segments":
        value[0, 0, 0] = np.nextafter(value[0, 0, 0], 0.)
    elif field == "certified_model_owner":
        value[0] = 1
    elif field == "certified_model_last_stamp_ns":
        value[0] -= 1
    else:
        value[0] += 1
    assert not matcher.matches_anchor_source(replace(frozen, **{field: value}))


def test_frozen_identity_distinguishes_empty_v3_from_v2_even_with_equal_geometry():
    frozen = reference(segments=(), owners=())
    matcher = BoundaryLaneLocalizer(frozen)
    assert not matcher.matches_anchor_source(replace(frozen, **{name: None for name in CERTIFICATE_FIELDS}))
