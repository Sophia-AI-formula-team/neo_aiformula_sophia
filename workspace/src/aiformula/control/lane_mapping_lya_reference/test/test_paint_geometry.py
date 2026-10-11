"""Independent pixel-face, plane and causal boundary-map oracles (no ROS)."""
from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np
import pytest

from lane_mapping_lya_reference.paint_geometry import (
    BoundaryConsensus, BoundaryObservation, BoundaryReference,
    REASON_J4_UNOBSERVED, extract_boundary_observation,
)


def lookup(width=120, height=120, scale=.02):
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1.
    return SimpleNamespace(width=width, height=height,
                           camera_matrix=np.array([[1. / scale, 0., width / 2.],
                                                   [0., 1. / scale, height / 2.], [0., 0., 1.]]),
                           base_from_camera=transform, max_sensitivity_m_per_px=.5,
                           plane_offset_m=0.)


def extract(mask, calibration=None, stamp=1, up=(0., 0., 1.), **kwargs):
    calibration = calibration or lookup(mask.shape[1], mask.shape[0])
    return extract_boundary_observation(mask, calibration, up, stamp, **kwargs)


def stripe(width=8):
    mask = np.zeros((120, 120), dtype=np.uint8)
    mask[5:115, 60:60 + width] = 255
    return mask


def literal_faces(mask):
    expected = set()
    h, w = mask.shape
    for y in range(h):
        for x in range(w):
            for dx, dy in ((1, 0), (0, 1)):
                xx, yy = x + dx, y + dy
                if xx < w and yy < h and bool(mask[y, x]) != bool(mask[yy, xx]):
                    expected.add((x + dx * .5, y + dy * .5))
    return expected


def tiny_observation(points=((.01, .01), (.02, .02)), normals=None, stamp=1,
                     contour_ids=None, reliable=True):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    n = len(points)
    normals = np.tile([1., 0.], (n, 1)) if normals is None else np.asarray(normals, dtype=float)
    segments = np.stack((points + [0., -.002], points + [0., .002]), axis=1)
    true = np.ones(n, dtype=bool)
    closed = contour_ids is None
    links = np.arange(n, dtype=np.int64)
    return BoundaryObservation(points, normals, segments, np.full(n, .02), true, true,
                               true, true, true & reliable, stamp, segments.copy(),
                               np.zeros((n, 2), dtype=np.int64), np.ones((n, 2), dtype=np.int64),
                               np.roll(links, 1) if closed else np.full(n, -1, dtype=np.int64),
                               np.roll(links, -1) if closed else np.full(n, -1, dtype=np.int64),
                               np.zeros(n, dtype=np.int64) if closed else np.asarray(contour_ids, dtype=np.int64),
                               np.zeros(n, dtype=np.int64), np.full(n, 6, dtype=np.int64), np.full(n, .45))


@pytest.mark.parametrize("kind", ["empty", "full", "stripe", "diagonal", "checkerboard"])
def test_only_internal_actual_white_black_faces(kind):
    mask = np.zeros((20, 24), dtype=np.uint8)
    if kind == "full":
        mask[:] = 255
    elif kind == "stripe":
        mask[:, 8:12] = 255
    elif kind == "diagonal":
        for y in range(20):
            mask[y, y:y + 2] = 255
    elif kind == "checkerboard":
        mask = ((np.indices(mask.shape).sum(axis=0) % 2) * 255).astype(np.uint8)
    observed = extract(mask)
    faces = set(map(tuple, observed.pixel_segments.mean(axis=1)))
    assert faces == literal_faces(mask)
    for white, black in zip(observed.white_pixels, observed.black_pixels):
        assert mask[white[1], white[0]] == 255
        assert mask[black[1], black[0]] == 0
        assert np.abs(white - black).sum() == 1


def test_straight_stripe_normals_outward_points_not_pca_centroids():
    observed = extract(stripe())
    centers = observed.pixel_segments.mean(axis=1)
    expected = np.column_stack(((centers[:, 0] - 60.) * .02, -(centers[:, 1] - 60.) * .02))
    np.testing.assert_allclose(observed.points, expected, rtol=0, atol=3.e-16)
    middle = (centers[:, 1] > 35) & (centers[:, 1] < 85)
    assert np.all(observed.valid[middle])
    left = middle & (centers[:, 0] == 59.5)
    right = middle & (centers[:, 0] == 67.5)
    np.testing.assert_allclose(observed.normals[left], np.tile([-1., 0.], (left.sum(), 1)), atol=1.e-13)
    np.testing.assert_allclose(observed.normals[right], np.tile([1., 0.], (right.sum(), 1)), atol=1.e-13)


@pytest.mark.parametrize("width", [1, 2, 3])
def test_thin_stripe_opposite_sides_cannot_share_normal_support(width):
    # Keep this exact STRAIGHT-line oracle farther than .45 m from either real
    # end cap. The original 120-row fixture accidentally included the physical
    # cap in the fit window, where an exactly axis-aligned PCA is not an oracle.
    mask = np.zeros((220, 120), dtype=np.uint8)
    mask[5:215, 60:60 + width] = 255
    observed = extract(mask, lookup(height=220, scale=.01))
    centers = observed.pixel_segments.mean(axis=1)
    middle = (centers[:, 1] > 100) & (centers[:, 1] < 120)
    assert np.all(observed.valid[middle])
    assert np.max(np.abs(observed.normals[middle, 1])) < 1.e-12
    assert set(np.round(observed.normals[middle, 0], 6)) == {-1., 1.}
    assert np.all(observed.normal_support_count[middle] <= 91)


def test_actual_end_caps_are_observed_faces_not_artificial_centerline_points():
    observed = extract(stripe(3), lookup(scale=.01))
    centers = observed.pixel_segments.mean(axis=1)
    top_cap = np.all(observed.pixel_segments[:, :, 1] == 4.5, axis=1)
    assert top_cap.sum() == 3
    assert np.all(observed.metric_valid[top_cap])
    expected_xy = np.column_stack(((centers[top_cap, 0] - 60) * .01,
                                   -(centers[top_cap, 1] - 60) * .01))
    np.testing.assert_allclose(observed.points[top_cap], expected_xy, atol=1.e-16)
    # A corner's fitted tangent is not uniquely the long side's direction.
    # Any admitted normal must nevertheless face the observed black side.
    admitted = top_cap & observed.valid
    assert np.all(observed.normals[admitted, 1] > 0)


def test_diagonal_contact_keeps_four_connected_white_contours_separate():
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[3, 3] = mask[4, 4] = 255
    observed = extract(mask)
    assert len(np.unique(observed.contour_ids)) == 2
    assert not observed.valid.any()
    for cid in np.unique(observed.contour_ids):
        whites = observed.white_pixels[observed.contour_ids == cid]
        assert len(np.unique(whites, axis=0)) == 1


def test_curved_paint_signed_normals_point_from_white_to_black():
    y, x = np.indices((160, 160))
    radius = np.hypot(x - 80, y - 80)
    mask = ((radius > 42) & (radius < 50)).astype(np.uint8) * 255
    observed = extract(mask, lookup(160, 160, .015))
    assert np.count_nonzero(observed.valid) > 200
    white_black = observed.black_pixels - observed.white_pixels
    expected_outward = white_black * [.015, -.015]
    assert np.all(np.sum(observed.normals[observed.valid] * expected_outward[observed.valid], axis=1) > 0)
    # Inner and outer contours are distinct; no spatial fit joins their sides.
    assert len(np.unique(observed.contour_ids)) == 2


def test_fov_truncation_does_not_invent_caps_and_invalid_j_evidence_remains():
    mask = stripe()
    mask[:5, 60:68] = 255
    observed = extract(mask)
    # Real side faces may END at the image border. No face ALONG that border
    # may be fabricated, because no outside black pixel was observed.
    assert not np.any(np.all(observed.pixel_segments[:, :, 1] == -.5, axis=1))
    near_top = observed.pixel_segments.mean(axis=1)[:, 1] == 0
    assert np.any(near_top)
    assert np.all(~observed.stable[near_top])
    assert np.all(observed.reason_codes[near_top] & REASON_J4_UNOBSERVED)


def test_sensitivity_cut_removes_no_faces_and_adds_no_cut_boundary():
    mask = stripe()
    ordinary = extract(mask)
    calibration = lookup(scale=.2)
    calibration.max_sensitivity_m_per_px = .15
    rejected = extract(mask, calibration)
    np.testing.assert_array_equal(rejected.pixel_segments, ordinary.pixel_segments)
    assert np.all(rejected.metric_valid)
    assert not rejected.stable.any()
    assert not rejected.valid.any()


def test_scalar_ray_plane_oracle_for_tilt_and_signed_offset():
    calibration = lookup()
    calibration.plane_offset_m = -.013
    normal = np.array([.03, -.04, math.sqrt(1 - .03 ** 2 - .04 ** 2)])
    observed = extract(stripe(), calibration, up=normal)
    forward = np.array([1., 0., 0.]) - normal[0] * normal
    forward /= np.linalg.norm(forward)
    left = np.cross(normal, forward)
    for index in range(0, len(observed.points), 13):
        uv = observed.pixel_segments[index].mean(axis=0)
        ray = calibration.base_from_camera[:3, :3] @ np.linalg.solve(calibration.camera_matrix, [uv[0], uv[1], 1.])
        origin = calibration.base_from_camera[:3, 3]
        point = origin + ray * ((calibration.plane_offset_m - float(normal @ origin)) / float(normal @ ray))
        expected = [float(forward @ point), float(left @ point)]
        np.testing.assert_allclose(observed.points[index], expected, rtol=1.e-13, atol=1.e-14)


def test_immutability_and_transform_does_not_change_source_or_pixels():
    observed = extract(stripe())
    before = observed.points.tobytes()
    for name in ("points", "normals", "segments", "valid", "pixel_segments", "previous"):
        with pytest.raises(ValueError):
            getattr(observed, name).setflags(write=True)
    transformed = observed.transformed([3., -2., math.pi / 2])
    np.testing.assert_allclose(transformed.points, observed.points @ np.array([[0., 1.], [-1., 0.]]) + [3., -2.], atol=1.e-14)
    assert transformed.source_stamp_ns == observed.source_stamp_ns
    assert observed.points.tobytes() == before
    np.testing.assert_array_equal(transformed.pixel_segments, observed.pixel_segments)


@pytest.mark.parametrize("stamp", [0, -1, True, 1.5, "1", 2 ** 63])
def test_invalid_source_stamps_rejected(stamp):
    with pytest.raises(ValueError):
        extract(stripe(), stamp=stamp)


def test_boundary_capacity_is_explicit_not_silent_sampling():
    with pytest.raises(ValueError, match="capacity"):
        extract(stripe(), config={"max_input_edges": 4})


def test_five_votes_one_reliable_frame_and_actual_mean_per_frame():
    consensus = BoundaryConsensus()
    for frame in range(1, 6):
        observation = tiny_observation(stamp=frame, reliable=frame == 5)
        proposal = consensus.prepare(observation)
        assert consensus.last_stamp_ns == (None if frame == 1 else frame - 1)
        assert consensus.can_commit(proposal)
        assert consensus.commit(proposal)
        assert not consensus.can_commit(proposal)
        assert not consensus.commit(proposal)
    snapshot = consensus.snapshot()
    assert len(snapshot.points_xy) == consensus.confirmed_count == 1
    np.testing.assert_array_equal(snapshot.votes, [5])
    np.testing.assert_array_equal(snapshot.reliable_votes, [1])
    np.testing.assert_allclose(snapshot.points_xy, [[.015, .015]], rtol=0, atol=1.e-17)
    assert len(snapshot.support_segments) == 2


def test_pixel_repetition_does_not_add_votes_or_move_frame_mean_weight():
    consensus = BoundaryConsensus()
    for stamp in range(1, 6):
        # Two pixels one frame, 20 another: each frame is one vote.
        count = 20 if stamp % 2 else 2
        x = .02 if stamp % 2 else .04
        obs = tiny_observation(np.tile([x, .01], (count, 1)), stamp=stamp)
        assert consensus.commit(consensus.prepare(obs))
    snapshot = consensus.snapshot()
    assert snapshot.votes.tolist() == [5]
    np.testing.assert_allclose(snapshot.points_xy[0], [.028, .01], rtol=0, atol=1.e-17)


def test_opposite_normals_same_cell_are_separate_records():
    consensus = BoundaryConsensus()
    for stamp in range(1, 6):
        obs = tiny_observation(normals=[[1., 0.], [-1., 0.]], stamp=stamp)
        consensus.commit(consensus.prepare(obs))
    snapshot = consensus.snapshot()
    assert len(snapshot.points_xy) == 2
    assert set(snapshot.normals[:, 0]) == {-1., 1.}
    assert snapshot.votes.tolist() == [5, 5]


def test_distinct_same_frame_contours_cannot_claim_one_old_record():
    consensus = BoundaryConsensus()
    consensus.commit(consensus.prepare(tiny_observation(stamp=1)))
    obs = tiny_observation(points=[[.011, .01], [.019, .02]], contour_ids=[0, 1], stamp=2)
    proposal = consensus.prepare(obs)
    assert proposal.stats["ambiguous_groups"] == 2
    assert proposal.stats["candidate_records"] == 3
    consensus.commit(proposal)
    assert max(record.votes for record in consensus._records.values()) == 1


def test_unused_prepare_wrong_owner_stale_generation_and_source_do_not_commit():
    consensus, other = BoundaryConsensus(), BoundaryConsensus()
    first = consensus.prepare(tiny_observation(stamp=1))
    stale = consensus.prepare(tiny_observation(stamp=2))
    assert consensus.last_stamp_ns is None
    assert not other.commit(first)
    assert consensus.commit(first)
    assert not consensus.commit(stale)
    with pytest.raises(ValueError, match="not increasing"):
        consensus.prepare(tiny_observation(stamp=1))
    assert consensus.last_stamp_ns == 1


def test_capacity_rejection_is_transactional():
    consensus = BoundaryConsensus({"max_candidate_cells": 1})
    obs = tiny_observation(points=[[.01, .01], [.3, .3]])
    with pytest.raises(ValueError, match="capacity"):
        consensus.prepare(obs)
    assert consensus.last_stamp_ns is None
    assert consensus._records == {}


def test_ttl_only_expires_unconfirmed_and_empty_commit_advances_frontier():
    consensus = BoundaryConsensus({"candidate_ttl_s": 1.})
    for stamp in range(1, 6):
        consensus.commit(consensus.prepare(tiny_observation(stamp=stamp)))
    extra = tiny_observation([[.3, .3]], stamp=6)
    consensus.commit(consensus.prepare(extra))
    assert len(consensus._records) == 2
    empty = extract(np.zeros((10, 10), dtype=np.uint8), stamp=2000000000)
    token = consensus.prepare(empty)
    assert len(consensus._records) == 2
    assert token.stats["expired_candidates"] == 1
    consensus.commit(token)
    assert len(consensus._records) == consensus.confirmed_count == 1
    assert consensus.snapshot().source_frontier_ns == 2000000000


def test_reference_subset_remaps_segments_and_never_expands_support():
    reference = BoundaryReference(np.array([[0., 0.], [1., 0.], [2., 0.]]),
                                  np.tile([0., 1.], (3, 1)),
                                  np.array([[[0., 0.], [.1, 0.]], [[1., 0.], [1.1, 0.]], [[2., 0.], [2.1, 0.]]]),
                                  np.array([0, 1, 2]), np.array([5, 6, 7]), np.array([1, 2, 3]),
                                  np.array([8, 10, 19]), 999)
    subset = reference.subset([2, 0])
    assert subset.record_ids.tolist() == [19, 8]
    assert subset.segment_owner.tolist() == [1, 0]
    np.testing.assert_array_equal(subset.support_segments, reference.support_segments[[0, 2]])
    assert subset.source_frontier_ns == 999
    assert reference.subset([]).points_xy.shape == (0, 2)
    for bad in ([1, 1], [-1], [3], [True], [1.5]):
        with pytest.raises(ValueError):
            reference.subset(bad)
    with pytest.raises(ValueError):
        subset.points_xy.setflags(write=True)


def test_unconfirmed_neighbour_never_supplies_finite_support():
    consensus = BoundaryConsensus()
    for stamp in range(1, 5):
        consensus.commit(consensus.prepare(tiny_observation([[.01, .01]], stamp=stamp)))
    consensus.commit(consensus.prepare(tiny_observation([[.01, .01], [.2, .01]], stamp=5)))
    reference = consensus.snapshot()
    assert len(reference.points_xy) == 1
    assert len(reference.support_segments) == 0


def test_batched_snapshot_membership_is_byteexact_to_literal_per_record_oracle(monkeypatch):
    consensus = BoundaryConsensus()
    points = np.column_stack((np.arange(20) * .11, np.zeros(20)))
    for stamp in range(1, 6):
        consensus.commit(consensus.prepare(tiny_observation(points, stamp=stamp)))
    consensus.commit(consensus.prepare(tiny_observation([[2.3, .1]], stamp=6)))
    records = [record for _, record in sorted(consensus._records.items()) if consensus._confirmed(record)]
    ids = np.asarray([record.record_id for record in records], dtype=np.int64)
    original_isin = np.isin
    expected_segments, expected_owners = [], []
    for slot, record in enumerate(records):
        known = np.all(original_isin(record.neighbours, ids), axis=1)
        expected_segments.extend(record.segments[known])
        expected_owners.extend([slot] * int(np.count_nonzero(known)))
    calls = []
    def spy(elements, test_elements, *args, **kwargs):
        calls.append((np.asarray(elements).shape, np.asarray(test_elements).shape))
        return original_isin(elements, test_elements, *args, **kwargs)
    monkeypatch.setattr(np, "isin", spy)
    snapshot = consensus.snapshot()
    assert len(calls) == 1
    assert snapshot.support_segments.tobytes() == np.asarray(expected_segments, dtype=np.float64).reshape(-1, 2, 2).tobytes()
    assert snapshot.segment_owner.tobytes() == np.asarray(expected_owners, dtype=np.int64).tobytes()


def perspective_calibration():
    # Independent forward pinhole camera, not a fitted trial image/calibration.
    return SimpleNamespace(width=320, height=240,
        camera_matrix=np.array([[180., 0., 160.], [0., 180., 100.], [0., 0., 1.]]),
        base_from_camera=np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.],
                                   [0., -1., 0., .65], [0., 0., 0., 1.]]),
        max_sensitivity_m_per_px=.5, plane_offset_m=0.)


def perspective_paint(side, width=.12, truncated=False):
    # Raster from an independent analytic ray/plane solution: X=h*fy/(v-cy),
    # Y=-h*(u-cx)/(v-cy). A straight metric stripe is a pixel staircase.
    yy, xx = np.indices((240, 320), dtype=np.float64)
    distance = np.where(yy > 100., yy - 100., np.nan)
    world_x = .65 * 180. / distance
    world_y = -.65 * (xx - 160.) / distance
    active = ((world_x >= (.5 if truncated else .9)) & (world_x <= 4.)
              & (side * world_y >= .75) & (side * world_y <= .75 + width))
    return active.astype(np.uint8) * 255


def literal_contour_fit(observation, target, domain=None):
    """Linked-face oracle: image topology admission, metric arc/PCA evidence.

    No smooth-fit grouping, window constructor, eigensolver helper or fitted
    normal is reused. R14 can supply an explicitly partitioned domain; corner
    partitions have separate analytic tests. This oracle then checks EVERY
    admitted ID, arc radius and PCA value inside that same domain.
    """
    outward = observation.black_pixels - observation.white_pixels
    cid = observation.contour_ids[target]
    path_ids = np.flatnonzero(observation.contour_ids == cid)
    closed = np.all(observation.previous[path_ids] >= 0) and np.all(observation.next[path_ids] >= 0)
    if domain is not None:
        path_ids, closed = domain
    admitted = set(map(int, path_ids))
    left = min(255, (len(path_ids) - 1) // 2) if closed else min(255, len(path_ids) - 1)
    right = min(256, len(path_ids) - 1 - left) if closed else min(256, len(path_ids) - 1)
    samples = [(int(target), 0.)]
    for links, cap, vertex in ((observation.previous, left, 0), (observation.next, right, 1)):
        current, arc = int(target), 0.
        direction_samples = []
        for _ in range(cap):
            nxt = int(links[current])
            if (nxt < 0 or nxt not in admitted or not observation.stable[nxt]
                    or np.dot(outward[target], outward[nxt]) < 0):
                break
            corner = observation.segments[current, vertex]
            arc += float(np.linalg.norm(observation.points[current] - corner)
                         + np.linalg.norm(observation.points[nxt] - corner))
            direction_samples.append((nxt, arc))
            current = nxt
        samples = direction_samples[::-1] + samples if vertex == 0 else samples + direction_samples
    for radius in (.45, .9, 1.35):
        ids = [index for index, distance in samples if distance <= radius]
        if len(ids) < 6:
            continue
        xy = observation.points[ids]
        delta = xy - xy.mean(axis=0)
        covariance = delta.T @ delta / len(ids)
        values, vectors = np.linalg.eigh(covariance)
        normal = vectors[:, 0]
        # Determine sign with the independent pinhole white/black ray solution.
        uv_white, uv_black = observation.white_pixels[target], observation.black_pixels[target]
        def ray(uv):
            return np.array([.65 * 180. / (uv[1] - 100.), -.65 * (uv[0] - 160.) / (uv[1] - 100.)])
        white_to_black = ray(uv_black) - ray(uv_white)
        white_to_black /= np.linalg.norm(white_to_black)
        if (values[1] > 1.e-5 and values[0] <= .30 ** 2 and values[0] <= .15 * values[1]
                and abs(np.dot(normal, white_to_black)) > 1.e-12):
            if np.dot(normal, white_to_black) < 0:
                normal *= -1
            return ids, radius, normal
    return [], None, None


@pytest.mark.parametrize('side', [-1, 1])
def test_projective_staircase_is_not_an_opposite_paint_side(side):
    observation = extract(perspective_paint(side), perspective_calibration())
    uv = observation.pixel_segments.mean(axis=1)
    selected = np.flatnonzero(observation.stable & (uv[:, 1] > 155.) & (uv[:, 1] < 185.))
    assert len(selected) > 40
    outward = observation.black_pixels - observation.white_pixels
    tangent = observation.segments[:, 1] - observation.segments[:, 0]
    metric = np.column_stack((tangent[:, 1], -tangent[:, 0]))
    metric /= np.linalg.norm(metric, axis=1)[:, None]
    # Directed contour orientation is consistent, so its metric right normal
    # has the same relative angle as the final white->black signed normals.
    nxt = observation.next[selected]
    keep = nxt >= 0
    image_dot = np.sum(outward[selected[keep]] * outward[nxt[keep]], axis=1)
    metric_dot = np.sum(metric[selected[keep]] * metric[nxt[keep]], axis=1)
    assert np.count_nonzero((image_dot == 0) & (metric_dot < -.2)) > 20
    assert np.all(observation.reliable[selected])
    for target in selected[::7]:
        ids, radius, normal = literal_contour_fit(observation, int(target))
        assert ids and radius is not None
        assert len(ids) == observation.normal_support_count[target]
        assert radius == observation.normal_radius[target]
        np.testing.assert_allclose(observation.normals[target], normal, rtol=0, atol=2.e-13)


@pytest.mark.parametrize('side', [-1, 1])
def test_projective_short_cap_cannot_import_opposite_side_support(side):
    # .06 m gives >2 actual pixels at the far cap; .015 m was subpixel there
    # and produced disconnected single-pixel components, not this test's stripe.
    observation = extract(perspective_paint(side, width=.06), perspective_calibration())
    uv = observation.pixel_segments.mean(axis=1)
    # At the far physical end of this thin stripe, the complete closed contour
    # is nearby in pixel space. The target's image-opposite face must still stop
    # traversal; changing to metric dot cannot decide this topology correctly.
    targets = np.flatnonzero(observation.stable & (uv[:, 1] > 131.) & (uv[:, 1] < 139.))
    assert len(targets) >= 10
    outward = observation.black_pixels - observation.white_pixels
    # Reconstruct only the explicit fit-domain boundary. The literal walk/PCA
    # below remains independent and must match all admitted IDs and values;
    # it must not expect the old smooth fit to cross a newly separated turn.
    from lane_mapping_lya_reference.paint_geometry import _direction_paths, DEFAULT_CONFIG
    paths, visited = [], set()
    for seed in list(np.flatnonzero(observation.previous < 0))+list(range(len(observation.points))):
        if seed in visited:
            continue
        current, path = int(seed), []
        while current >= 0 and current not in visited:
            visited.add(current)
            path.append(current)
            current = int(observation.next[current])
        paths.append((np.asarray(path, dtype=np.int64), current == seed))
    def actual_ray(uv):
        return np.column_stack((.65*180./(uv[:,1]-100.),
                                -.65*(uv[:,0]-160.)/(uv[:,1]-100.)))
    pieces = _direction_paths(observation.points, observation.segments,
                              observation.stable, paths, dict(DEFAULT_CONFIG),
                              white_xy=actual_ray(observation.white_pixels),
                              black_xy=actual_ray(observation.black_pixels))
    domains = {int(index): (path, closed) for path, closed in pieces for index in path}
    checked = 0
    for target in targets:
        ids, radius, normal = literal_contour_fit(observation, int(target), domains[int(target)])
        if not ids:
            continue
        assert np.all(outward[ids] @ outward[target] >= 0)
        assert observation.valid[target]
        assert observation.normal_support_count[target] == len(ids)
        np.testing.assert_allclose(observation.normals[target], normal, rtol=0, atol=3.e-13)
        checked += 1
    assert checked >= 6


def test_projective_image_truncation_keeps_only_internal_faces():
    mask = perspective_paint(-1, truncated=True)
    observation = extract(mask, perspective_calibration())
    assert set(map(tuple, observation.pixel_segments.mean(axis=1))) == literal_faces(mask)
    # The right lane leaves the RIGHT image edge before the bottom edge.
    assert not np.any(np.all(observation.pixel_segments[:, :, 0] == 319.5, axis=1))
    ending = np.any(observation.pixel_segments[:, :, 0] == 319.5, axis=1)
    assert ending.any()
    assert np.all(observation.reason_codes[ending] & REASON_J4_UNOBSERVED)


def test_sheared_staircase_normal_faces_analytic_black_halfplane():
    # Invertible camera skew gives metric Jacobian [[.02,.04],[0,-.02]].
    # Pixel stripe normal (1,1) transforms by J^-T to (50,50), but an
    # individual horizontal pixel face normal transforms to (0,-50).
    # Dotting these normals flips the true side; a normal must instead be
    # signed against the actual projected white-to-black displacement J*ey.
    calibration = lookup()
    calibration.camera_matrix = np.array([[50., -100., 60.], [0., 50., 60.], [0., 0., 1.]])
    yy, xx = np.indices((120, 120))
    mask = ((xx + yy >= 110) & (xx + yy <= 115)).astype(np.uint8) * 255
    observation = extract(mask, calibration)
    uv = observation.pixel_segments.mean(axis=1)
    middle = (uv[:, 0] > 40) & (uv[:, 0] < 75) & (uv[:, 1] > 40) & (uv[:, 1] < 75)
    assert middle.sum() > 60
    assert np.all(observation.valid[middle])
    image_outward = observation.black_pixels[middle] - observation.white_pixels[middle]
    expected = np.sign(image_outward.sum(axis=1))[:, None] * np.array([[1., 1.]]) / math.sqrt(2.)
    assert np.all(np.sum(observation.normals[middle] * expected, axis=1) > .995)
    actual_displacement = image_outward @ np.array([[.02, 0.], [.04, -.02]])
    assert np.all(np.sum(observation.normals[middle] * actual_displacement, axis=1) > 0)


def graph_observation(points, stamp, order=None, reliable=True):
    """Explicit ordered observation topology, independent of the map/index."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    n = len(points)
    observation = tiny_observation(points, normals=np.tile([0., 1.], (n, 1)), stamp=stamp,
                                   contour_ids=np.arange(n), reliable=reliable)
    previous, following = np.full(n, -1, dtype=np.int64), np.full(n, -1, dtype=np.int64)
    ids = np.arange(n, dtype=np.int64)
    if order is not None:
        for left, right in zip(order, order[1:]):
            following[left], previous[right] = right, left
        ids[list(order)] = n
    return replace(observation, previous=previous, next=following, contour_ids=ids)


def confirmed_graph(points=((.01, .01), (.11, .01), (.21, .01)), config=None):
    consensus = BoundaryConsensus(config)
    for stamp in range(1, 6):
        observation = graph_observation(points, stamp, order=list(range(len(points))))
        assert consensus.commit(consensus.prepare(observation))
    return consensus


def test_graph_requires_own_five_votes_not_just_confirmed_endpoint_nodes():
    consensus = BoundaryConsensus()
    points = [[.01, .01], [.11, .01]]
    for stamp in range(1, 5):
        consensus.commit(consensus.prepare(graph_observation(points, stamp)))
    consensus.commit(consensus.prepare(graph_observation(points, 5, order=[0, 1])))
    assert consensus.confirmed_count == 2
    assert consensus.snapshot().directed_edges.shape == (0, 2)
    assert list(consensus._edges.values())[0].votes == 1
    for stamp in range(6, 10):
        consensus.commit(consensus.prepare(graph_observation(points, stamp, order=[0, 1])))
    reference = consensus.snapshot()
    np.testing.assert_array_equal(reference.directed_edges, [[0, 1]])
    np.testing.assert_array_equal(reference.edge_frame_votes, [5])
    np.testing.assert_array_equal(reference.votes, [9, 9])
    np.testing.assert_array_equal(reference.edge_last_stamp_ns, [9])


def test_link_requires_one_joint_reliable_observation_not_separate_reliable_endpoints():
    consensus = BoundaryConsensus()
    for stamp in range(1, 6):
        observation = graph_observation([[.01, .01], [.11, .01]], stamp, order=[0, 1])
        observation = replace(observation, reliable=np.array([stamp % 2 == 1, stamp % 2 == 0]))
        consensus.commit(consensus.prepare(observation))
    assert consensus.confirmed_count == 2
    edge = list(consensus._edges.values())[0]
    assert edge.votes == 5 and edge.reliable_votes == 0
    assert consensus.snapshot().directed_edges.shape == (0, 2)
    consensus.commit(consensus.prepare(graph_observation([[.01, .01], [.11, .01]], 6, order=[0, 1])))
    np.testing.assert_array_equal(consensus.snapshot().edge_reliable_votes, [1])


def test_contiguous_run_crossing_old_sector_boundary_is_one_vote():
    consensus = BoundaryConsensus()
    angle = .01
    normals = np.array([[math.cos(angle), -math.sin(angle)], [math.cos(angle), math.sin(angle)]])
    for stamp in range(1, 6):
        observation = tiny_observation(stamp=stamp, normals=normals)
        proposal = consensus.prepare(observation)
        assert proposal.stats['ambiguous_groups'] == 0
        consensus.commit(proposal)
    reference = consensus.snapshot()
    assert len(reference.points_xy) == 1
    assert reference.votes.tolist() == [5]
    assert reference.directed_edges.shape == (0, 2)


def test_same_cell_reentry_stays_distinct_and_only_observed_order_links():
    consensus = confirmed_graph([[.01, .01], [.12, .01], [.02, .02]])
    reference = consensus.snapshot()
    assert len(reference.points_xy) == 3
    cells = np.rint(reference.points_xy / .1).astype(int)
    assert np.count_nonzero(np.all(cells == [0, 0], axis=1)) == 2
    order = [int(np.argmin(np.linalg.norm(reference.points_xy - p, axis=1)))
             for p in ([.01, .01], [.12, .01], [.02, .02])]
    assert set(map(tuple, reference.directed_edges)) == {(order[0], order[1]), (order[1], order[2])}
    assert len(consensus._edges) == 2


def test_signed_pairwise_compatibility_not_only_consecutive_angle():
    angle = np.deg2rad([0., 20., 40.])
    observation = graph_observation([[.01, .01], [.02, .01], [.03, .01]], 1, order=[0, 1, 2])
    observation = replace(observation, normals=np.column_stack((np.cos(angle), np.sin(angle))))
    consensus = BoundaryConsensus()
    prepared = consensus.prepare(observation)
    assert len(prepared.updates) == 2
    assert sorted(len(record.segments) for record in prepared.updates) == [1, 2]


@pytest.mark.parametrize('missing_kind', ['j_gap', 'fov_cut', 'different_contour'])
def test_graph_does_not_link_unobserved_gaps(missing_kind):
    consensus = BoundaryConsensus()
    for stamp in range(1, 6):
        observation = graph_observation([[.01, .01], [.11, .01], [.21, .01]], stamp,
                                        order=[0, 1, 2] if missing_kind == 'j_gap' else None)
        if missing_kind == 'j_gap':
            valid = np.array([True, False, True])
            observation = replace(observation, valid=valid, stable=valid, reliable=valid,
                                  sensitivity=np.array([.02, 1., .02]))
        consensus.commit(consensus.prepare(observation))
    assert len(consensus._edges) == 0
    assert consensus.snapshot().directed_edges.shape == (0, 2)


def test_graph_source_pixel_repetition_does_not_multiply_votes():
    consensus = BoundaryConsensus()
    for stamp in range(1, 6):
        points = np.repeat([[.01, .01], [.11, .01], [.21, .01]], 1 if stamp % 2 else 9, axis=0)
        consensus.commit(consensus.prepare(graph_observation(points, stamp, order=list(range(len(points))))))
    reference = consensus.snapshot()
    assert reference.votes.tolist() == [5, 5, 5]
    assert reference.edge_frame_votes.tolist() == [5, 5]
    assert reference.edge_reliable_votes.tolist() == [5, 5]


def test_graph_closed_loop_last_link_requires_actual_observation():
    consensus = BoundaryConsensus()
    for stamp in range(1, 6):
        observation = graph_observation([[.01, .01], [.11, .01], [.21, .01]], stamp, order=[0, 1, 2])
        observation = replace(observation, previous=np.array([2, 0, 1]), next=np.array([1, 2, 0]))
        consensus.commit(consensus.prepare(observation))
    assert set(map(tuple, consensus.snapshot().directed_edges)) == {(0, 1), (1, 2), (2, 0)}


def test_graph_confirmed_branch_is_not_arbitrarily_certified_as_joint():
    consensus = BoundaryConsensus()
    points = [[.01, .01], [.11, .01], [.21, .01]]
    for stamp in range(1, 11):
        order = [0, 1] if stamp <= 5 else [0, 2]
        consensus.commit(consensus.prepare(graph_observation(points, stamp, order=order)))
    assert consensus.confirmed_count == 3
    assert len(consensus._edges) == 2
    assert all(edge.votes == 5 for edge in consensus._edges.values())
    reference = consensus.snapshot()
    np.testing.assert_array_equal(reference.directed_edges, [[0, 1], [0, 2]])
    np.testing.assert_array_equal(reference.full_in_degree, [0, 1, 1])
    np.testing.assert_array_equal(reference.full_out_degree, [2, 0, 0])
    # Keep the nearer branch as a nearest candidate; ambiguity only blocks
    # treating its vertex as a uniquely observed continuation.
    cut = reference.subset([0, 1])
    np.testing.assert_array_equal(cut.directed_edges, [[0, 1]])
    np.testing.assert_array_equal(cut.full_out_degree, [2, 0])


def test_graph_subset_can_only_remove_links_no_gap_or_internal_joint_recovery():
    reference = confirmed_graph().snapshot()
    cut = reference.subset([0, 2])
    assert cut.directed_edges.shape == (0, 2)
    partial = reference.subset([1, 2])
    np.testing.assert_array_equal(partial.directed_edges, [[0, 1]])
    assert np.count_nonzero(partial.directed_edges[:, 1] == 0) == 0
    np.testing.assert_array_equal(partial.full_in_degree, [1, 1])
    np.testing.assert_array_equal(partial.full_out_degree, [1, 0])
    assert partial.source_frontier_ns == reference.source_frontier_ns
    for name in ('directed_edges', 'edge_frame_votes', 'edge_reliable_votes', 'edge_last_stamp_ns',
                 'full_in_degree', 'full_out_degree'):
        with pytest.raises(ValueError):
            getattr(partial, name).setflags(write=True)


def test_graph_prepare_discard_wrong_owner_and_generation_are_atomic():
    consensus = confirmed_graph()
    reference = consensus.snapshot()
    baseline = tuple((key, value) for key, value in consensus._edges.items())
    proposal = consensus.prepare(graph_observation([[.01, .01], [.11, .01], [.21, .01]], 6, order=[0, 2, 1]))
    assert tuple(consensus._edges.items()) == baseline
    assert not BoundaryConsensus().commit(proposal)
    alternate = consensus.prepare(graph_observation([[.01, .01], [.11, .01], [.21, .01]], 7, order=[0, 1, 2]))
    assert consensus.commit(alternate)
    assert not consensus.commit(proposal)
    assert set(consensus._edges) == {(0, 1), (1, 2)}
    assert all(edge.votes == 6 for edge in consensus._edges.values())
    assert reference.edge_frame_votes.tolist() == [5, 5]


def test_graph_capacity_rejection_leaves_node_edge_and_frontier_unchanged():
    consensus = BoundaryConsensus({'max_support_segments': 4})
    points = [[.01, .01], [.11, .01], [.21, .01], [.31, .01]]
    for stamp in range(1, 6):
        consensus.commit(consensus.prepare(graph_observation(points, stamp)))
    consensus.commit(consensus.prepare(graph_observation(points, 6, order=[0, 1, 2, 3])))
    before = (dict(consensus._records), dict(consensus._edges), consensus._generation, consensus.last_stamp_ns)
    with pytest.raises(ValueError, match='topology edge capacity'):
        consensus.prepare(graph_observation(points, 7, order=[0, 2, 1, 3]))
    assert all(consensus._records[k] is v for k, v in before[0].items())
    assert consensus._edges == before[1]
    assert (consensus._generation, consensus.last_stamp_ns) == before[2:]


def test_graph_ttl_expires_only_unconfirmed_links_without_expiring_confirmed_nodes():
    consensus = BoundaryConsensus({'candidate_ttl_s': 1.})
    for stamp in range(1, 6):
        consensus.commit(consensus.prepare(graph_observation([[.01, .01], [.11, .01]], stamp)))
    consensus.commit(consensus.prepare(graph_observation([[.01, .01], [.11, .01]], 6, order=[0, 1])))
    assert len(consensus._edges) == 1
    empty = extract(np.zeros((10, 10), dtype=np.uint8), stamp=2000000000)
    proposal = consensus.prepare(empty)
    assert proposal.stats['topology_expired_edges'] == 1
    assert len(consensus._edges) == 1
    consensus.commit(proposal)
    assert consensus._edges == {} and consensus.confirmed_count == 2
    established = confirmed_graph(config={'candidate_ttl_s': 1.})
    established.commit(established.prepare(empty))
    assert len(established.snapshot().directed_edges) == 2


def test_graph_none_legacy_vs_explicit_empty_are_not_interchangeable():
    reference = confirmed_graph().snapshot()
    legacy = replace(reference, directed_edges=None, edge_frame_votes=None,
                     edge_reliable_votes=None, edge_last_stamp_ns=None,
                     full_in_degree=None, full_out_degree=None)
    assert legacy.topology_representation is None
    assert legacy.subset([0, 1]).directed_edges is None
    empty = replace(reference, directed_edges=np.empty((0, 2), dtype=np.int64),
                    edge_frame_votes=[], edge_reliable_votes=[], edge_last_stamp_ns=[])
    assert empty.topology_representation == 'confirmed_contour_node_chords_v1'
    assert empty.subset([0, 1]).directed_edges.shape == (0, 2)


@pytest.mark.parametrize('bad', ['partial', 'self', 'duplicate', 'outside', 'future',
                                'zero_vote', 'too_many_votes', 'reliable_over_endpoint', 'bool', 'float'])
def test_graph_reference_rejects_invalid_or_future_evidence(bad):
    reference = confirmed_graph().snapshot()
    changes = {}
    if bad == 'partial': changes['edge_frame_votes'] = None
    if bad == 'self': changes['directed_edges'] = np.array([[0, 0], [1, 2]])
    if bad == 'duplicate': changes['directed_edges'] = np.array([[0, 1], [0, 1]])
    if bad == 'outside': changes['directed_edges'] = np.array([[0, 3], [1, 2]])
    if bad == 'future': changes['edge_last_stamp_ns'] = [5, 6]
    if bad == 'zero_vote': changes['edge_frame_votes'] = [0, 5]
    if bad == 'too_many_votes': changes['edge_frame_votes'] = [6, 5]
    if bad == 'reliable_over_endpoint': changes.update(reliable_votes=[1, 1, 1], edge_reliable_votes=[2, 2])
    if bad == 'bool': changes['directed_edges'] = np.zeros((2, 2), dtype=bool)
    if bad == 'float': changes['directed_edges'] = np.array([[0., 1.], [1., 2.]])
    with pytest.raises(ValueError):
        replace(reference, **changes)


@pytest.mark.parametrize('bad', ['partial', 'negative', 'below_local', 'capacity', 'boolean',
                                'floating', 'shape', 'legacy_with_degree'])
def test_graph_full_degree_validation_prevents_false_continuation(bad):
    reference = confirmed_graph().snapshot()
    changes = {}
    if bad == 'partial': changes['full_in_degree'] = None
    if bad == 'negative': changes['full_in_degree'] = [-1, 1, 1]
    if bad == 'below_local': changes['full_out_degree'] = [0, 1, 0]
    if bad == 'capacity': changes['full_in_degree'] = [200001, 1, 1]
    if bad == 'boolean': changes['full_in_degree'] = [False, True, True]
    if bad == 'floating': changes['full_in_degree'] = [0., 1., 1.]
    if bad == 'shape': changes['full_in_degree'] = [[0, 1, 1]]
    if bad == 'legacy_with_degree':
        changes.update(directed_edges=None, edge_frame_votes=None,
                       edge_reliable_votes=None, edge_last_stamp_ns=None)
    with pytest.raises(ValueError):
        replace(reference, **changes)


def test_manual_graph_derives_full_degrees_only_when_not_supplied():
    reference = confirmed_graph().snapshot()
    manual = replace(reference, full_in_degree=None, full_out_degree=None)
    np.testing.assert_array_equal(manual.full_in_degree, [0, 1, 1])
    np.testing.assert_array_equal(manual.full_out_degree, [1, 1, 0])
    # A historical omitted branch remains evidence even when the local subset
    # happens to look like an ordinary 1-in/1-out chain.
    inherited = replace(reference, full_out_degree=[1, 2, 0])
    subset = inherited.subset([0, 1, 2])
    assert subset.full_in_degree[1] == 1 and subset.full_out_degree[1] == 2


@pytest.mark.parametrize('kind', ['straight', 'curved', 'fov_cut', 'j_cut'])
def test_real_pixel_contours_confirm_only_their_actual_directed_adjacency(kind):
    mask = stripe()
    if kind == 'curved':
        yy, xx = np.indices((120, 120))
        radius = np.hypot(xx - 60., yy - 60.)
        mask = ((radius > 30.) & (radius < 38.)).astype(np.uint8) * 255
    if kind == 'fov_cut':
        mask[:5, 60:68] = 255
    observation = extract(mask)
    if kind == 'j_cut':
        # A sensor-reliability hole retains the real faces but cannot create a
        # contour shortcut. It does not supply a fabricated white/black cap.
        cut = np.abs(observation.points[:, 1]) < .12
        stable = observation.stable & ~cut
        observation = replace(observation, stable=stable, valid=stable & observation.normal_valid,
                              reliable=observation.reliable & ~cut,
                              position_valid=observation.position_valid & ~cut,
                              position_reliable=observation.position_reliable & ~cut,
                              sensitivity=np.where(cut, 1., observation.sensitivity))
    consensus = BoundaryConsensus()
    first = consensus.prepare(observation)
    face_by_segment = {segment.tobytes(): face for face, segment in enumerate(observation.segments)}
    face_owner = {}
    for record in first.updates:
        for segment in record.segments:
            face_owner[face_by_segment[segment.tobytes()]] = record.record_id
    expected = set()
    for face, owner in face_owner.items():
        following = int(observation.next[face])
        if following in face_owner and face_owner[following] != owner:
            expected.add((owner, face_owner[following]))
    assert len(expected) > 10
    assert consensus.commit(first)
    for stamp in range(2, 6):
        assert consensus.commit(consensus.prepare(replace(observation, source_stamp_ns=stamp)))
    reference = consensus.snapshot()
    actual = {tuple(reference.record_ids[edge]) for edge in reference.directed_edges}
    assert actual == expected
    assert np.all(reference.votes == 5)
    assert np.all(reference.edge_frame_votes == 5)
    np.testing.assert_array_equal(reference.full_in_degree,
                                  np.bincount(reference.directed_edges[:, 1], minlength=len(reference.points_xy)))
    np.testing.assert_array_equal(reference.full_out_degree,
                                  np.bincount(reference.directed_edges[:, 0], minlength=len(reference.points_xy)))
