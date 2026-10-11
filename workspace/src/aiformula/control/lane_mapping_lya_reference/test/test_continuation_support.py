"""Independent continuation evidence tests; no estimator or recorded inputs.

Fraction checks certify exact-real geometry of the supplied binary64 vertices
and returned binary64 alpha, NOT physical calibration or projection accuracy.
The implementation may conservatively lose support; it may never expand it.
"""

from dataclasses import FrozenInstanceError
from fractions import Fraction as F
import math

import numpy as np
import pytest

from lane_mapping_lya_reference.continuation_support import (
    ContinuationFrame, clip_segment_quads, common_intervals, retain_frames,
    projection_alpha_bounds,
)


COSINE = math.cos(math.pi / 6.)
SEGMENT = np.array([[0., 0.], [1., 0.]])
NORMAL = np.array([0., 1.])


def fraction(value):
    return F.from_float(float(value))


def exact_point(point):
    return tuple(fraction(value) for value in point)


def subtract(left, right):
    return tuple(a-b for a, b in zip(left, right))


def cross(left, right):
    return left[0]*right[1]-left[1]*right[0]


def exact_clip(segment, quad):
    """Independent rational polygon half-plane intersection, no NumPy math."""
    points = [exact_point(point) for point in quad]
    p, end = [exact_point(point) for point in segment]
    delta = subtract(end, p)
    edges = [subtract(points[(i+1) % 4], points[i]) for i in range(4)]
    turns = [cross(edges[i], edges[(i+1) % 4]) for i in range(4)]
    if all(value > 0 for value in turns):
        sign = 1
    elif all(value < 0 for value in turns):
        sign = -1
    else:
        return None
    if p == end:
        return None
    lower, upper = F(0), F(1)
    for vertex, edge in zip(points, edges):
        a = sign*cross(edge, delta)
        b = sign*cross(edge, subtract(p, vertex))
        if a > 0:
            lower = max(lower, -b/a)
        elif a < 0:
            upper = min(upper, -b/a)
        elif b < 0:
            return None
    return None if lower >= upper else (lower, upper)


def assert_inward(segment, quad, interval):
    exact = exact_clip(segment, quad)
    assert exact is not None
    lo, hi = map(fraction, interval)
    assert F(0) <= exact[0] <= lo < hi <= exact[1] <= F(1)
    points = [exact_point(point) for point in quad]
    p, end = [exact_point(point) for point in segment]
    delta = subtract(end, p)
    sign = 1 if cross(subtract(points[1], points[0]),
                      subtract(points[2], points[1])) > 0 else -1
    # Exact endpoint containment proves every intermediate point by convexity.
    for alpha in (lo, hi):
        point = tuple(x+alpha*d for x, d in zip(p, delta))
        for i, vertex in enumerate(points):
            edge = subtract(points[(i+1) % 4], vertex)
            assert sign*cross(edge, subtract(point, vertex)) >= 0


def rectangle(lower=-.2, upper=1.2, bottom=-.2, top=.2):
    return np.array([[lower, bottom], [upper, bottom],
                     [upper, top], [lower, top]], dtype=np.float64)


def frame(stamp, quads=None, reliable=True, normals=None):
    q = np.array([rectangle()] if quads is None else quads, dtype=np.float64).reshape(-1, 4, 2)
    n = (np.tile(NORMAL, (len(q), 1)) if normals is None
         else np.asarray(normals, dtype=np.float64).reshape(-1, 2))
    trusted = (np.full(len(q), reliable, dtype=bool) if isinstance(reliable, bool)
               else np.asarray(reliable, dtype=bool))
    return ContinuationFrame(stamp, q, n, trusted)


def frames(count=5, quads=None, reliable_at=(0,), start=1, normals=None):
    return tuple(frame(start+i, quads, i in reliable_at, normals) for i in range(count))


def common(hypotheses, segment=SEGMENT, normal=NORMAL, k=5, r=1):
    return common_intervals(segment, normal, hypotheses, minimum_frames=k,
                            minimum_reliable=r, cosine=COSINE)


def contains(spans, value):
    return any(lo <= value <= hi for lo, hi in spans)


@pytest.mark.parametrize('reverse', [False, True])
@pytest.mark.parametrize('power,angle,translation', [
    (0, 0., (0., 0.)), (0, .731, (3.2, -9.4)),
    (-400, -.321, (0., 0.)), (400, 1.12, (0., 0.)),
    (-15, 2.91, (1.e6, -1.e6)), (10, math.pi/2, (-4096., 8192.)),
])
def test_clip_fraction_safety_under_binary64_scales_rotations(reverse, power, angle, translation):
    scale = math.ldexp(1., power)
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    quad = (rectangle(.2, .8) * scale) @ rotation.T + translation
    segment = (SEGMENT * scale) @ rotation.T + translation
    if reverse:
        quad = quad[::-1]
    before = (segment.tobytes(), quad.tobytes())
    spans, valid = clip_segment_quads(segment, quad[None])
    assert valid.tolist() == [True]
    assert_inward(segment, quad, spans[0])
    assert before == (segment.tobytes(), quad.tobytes())


def test_random_convex_affine_quads_fraction_subset_oracle():
    rng = np.random.default_rng(20261012)
    accepted = 0
    for _ in range(64):
        angle = rng.uniform(-math.pi, math.pi)
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s], [s, c]])
        scale = math.ldexp(1., int(rng.integers(-40, 40)))
        shear = np.array([[rng.uniform(.5, 2.), rng.uniform(-.4, .4)], [0., rng.uniform(.5, 2.)]])
        transform = rotation @ shear * scale
        offset = rng.uniform(-2., 2., 2) * scale
        q = rectangle(.1, .9) @ transform.T + offset
        seg = SEGMENT @ transform.T + offset
        if rng.integers(0, 2):
            q = q[::-1]
        spans, valid = clip_segment_quads(seg, q[None])
        if valid[0]:
            accepted += 1
            assert_inward(seg, q, spans[0])
    assert accepted == 64


def test_clip_root_outside_quad_still_finds_true_finite_intersection():
    q = rectangle(.25, .75)
    spans, valid = clip_segment_quads(SEGMENT, q[None])
    assert valid[0]
    assert spans[0, 0] >= .25 and spans[0, 1] <= .75
    assert spans[0, 0] < .3 and spans[0, 1] > .7
    assert_inward(SEGMENT, q, spans[0])


def test_slanted_perspective_quad_fraction_oracle():
    q = np.array([[.2, -.7], [.9, -.2], [.65, .8], [.1, .3]])
    spans, valid = clip_segment_quads(SEGMENT, q[None])
    assert valid[0]
    assert_inward(SEGMENT, q, spans[0])


@pytest.mark.parametrize('quad', [
    rectangle(0., 1., .1, .3),  # tangential ranges overlap, but no 2-D support
    np.zeros((4, 2)),
    np.array([[0., 0.], [1., 0.], [2., 0.], [3., 0.]]),
    np.array([[0., -1.], [1., 1.], [0., 1.], [1., -1.]]),
    np.array([[0., -1.], [1., -1.], [.4, 0.], [0., 1.]]),
    np.full((4, 2), np.nan),
    np.full((4, 2), np.inf),
    rectangle() * 1.e200,
])
def test_invalid_degenerate_nonconvex_overflow_or_off_line_quads_fail_closed(quad):
    spans, valid = clip_segment_quads(SEGMENT, quad[None])
    assert not valid[0]
    assert np.array_equal(spans, np.zeros((1, 2)))


@pytest.mark.parametrize('segment', [np.zeros((2, 2)), np.full((2, 2), np.nan),
                                     np.full((2, 2), np.inf)])
def test_invalid_or_zero_segment_fail_closed(segment):
    spans, valid = clip_segment_quads(segment, rectangle()[None])
    assert not valid[0]
    assert not np.any(spans)


def test_point_contact_and_small_true_gap_do_not_become_area_support():
    # Segment only touches the lower left corner of the quad.
    seg = np.array([[-1., -1.], [0., 0.]])
    q = rectangle(0., 1., 0., 1.)
    assert not clip_segment_quads(seg, q[None])[1][0]
    q = rectangle(0., 1., 1.e-8, 1.)
    assert not clip_segment_quads(SEGMENT, q[None])[1][0]


def test_empty_shape_and_mixed_invalid_rows_are_preserved():
    spans, valid = clip_segment_quads(SEGMENT, np.empty((0, 4, 2)))
    assert spans.shape == (0, 2) and valid.shape == (0,)
    quads = np.stack((rectangle(), np.full((4, 2), np.nan), rectangle(.3, .6)))
    spans, valid = clip_segment_quads(SEGMENT, quads)
    assert valid.tolist() == [True, False, True]
    for i in (0, 2):
        assert_inward(SEGMENT, quads[i], spans[i])


@pytest.mark.parametrize('segment,quad', [
    (np.zeros((2,)), np.zeros((1, 4, 2))),
    (SEGMENT, np.zeros((4, 2))), (SEGMENT, np.zeros((1, 3, 2))),
])
def test_clip_shape_errors_are_not_silent(segment, quad):
    with pytest.raises(ValueError):
        clip_segment_quads(segment, quad)


def test_frame_owns_immutable_geometry_and_flags():
    q = rectangle()[None]
    n = NORMAL[None].copy()
    trusted = np.ones(1, dtype=bool)
    item = ContinuationFrame(1, q, n, trusted)
    before = tuple(value.tobytes() for value in (item.quads, item.normals, item.reliable))
    q[:] = 40.; n[:] = -2.; trusted[:] = False
    assert before == tuple(value.tobytes() for value in (item.quads, item.normals, item.reliable))
    for value in (item.quads, item.normals, item.reliable):
        with pytest.raises(ValueError):
            value.setflags(write=True)
    with pytest.raises(FrozenInstanceError):
        item.stamp_ns = 2
    assert item.storage_units == 2
    assert frame(2, []).storage_units == 1


@pytest.mark.parametrize('stamp', [0, -1, 1., True, np.int64(1)])
def test_frame_requires_positive_exact_python_source_stamp(stamp):
    with pytest.raises(ValueError):
        frame(stamp)


@pytest.mark.parametrize('normal', [(0., 0.), (0., 2.), (np.nan, 1.)])
def test_frame_rejects_nonfinite_or_nonunit_direction(normal):
    with pytest.raises(ValueError):
        frame(1, normals=[normal])


def test_frame_rejects_nonboolean_reliable_and_capacity_before_copy():
    with pytest.raises(ValueError):
        ContinuationFrame(1, rectangle()[None], NORMAL[None], np.array([1]))
    with pytest.raises(ValueError):
        ContinuationFrame(1, np.broadcast_to(rectangle(), (200001, 4, 2)),
                          np.broadcast_to(NORMAL, (200001, 2)), np.zeros(200001, dtype=bool))


def test_five_distinct_frames_plus_pointwise_reliable_evidence_required():
    assert not common({(1, 2): frames(4)})
    assert not common({(1, 2): frames(5, reliable_at=())})
    assert contains(common({(1, 2): frames(5)}), .5)


def test_many_duplicate_quads_in_one_frame_are_only_one_vote():
    repeated = [rectangle()] * 25
    assert not common({(1, 2): frames(1, repeated)})
    assert not common({(1, 2): frames(4, repeated)})


def test_same_source_repeated_in_hypothesis_never_becomes_five_votes():
    item = frame(1)
    try:
        result = common({(1, 2): (item,) * 5})
    except ValueError:
        return  # Explicit invalid-input rejection is fail-closed too.
    assert not result


def test_each_hypothesis_requires_its_own_five_frames_not_pooled_ten():
    assert not common({(1, 2): frames(5), (1, 3): frames(4, start=10)})
    assert contains(common({(1, 2): frames(5), (1, 3): frames(5, start=10)}), .5)


def test_each_hypothesis_requires_its_own_reliable_frame():
    assert not common({(1, 2): frames(5), (1, 3): frames(5, reliable_at=(), start=10)})


@pytest.mark.parametrize('extra', [{}, {(1, -1): ()}, {(-1, 2): ()}, {(1, 3): ()},
                                    {(1, 3): frames(5, [])}])
def test_empty_inventory_gap_and_unobserved_hypothesis_veto(extra):
    hypotheses = {} if not extra else dict({(1, 2): frames(5)}, **{})
    hypotheses.update(extra)
    assert not common(hypotheses)


def test_full_gap_signature_vetoes_even_with_good_quads():
    assert not common({(1, 2): frames(5), (-1, 3): frames(5, start=10)})


def test_pointwise_reliability_never_borrows_from_distant_face_same_frame():
    q = [rectangle(-.2, .4), rectangle(.6, 1.2)]
    items = tuple(frame(i+1, q, [True, False]) for i in range(5))
    spans = common({(1, 2): items})
    assert contains(spans, .2)
    assert not contains(spans, .8)
    assert not contains(spans, .5)


def test_partial_common_support_is_not_a_whole_primitive_waiver():
    first = frames(5, [rectangle(-.1, .8)])
    second = frames(5, [rectangle(.2, 1.1)], start=10)
    spans = common({(1, 2): first, (1, 3): second})
    assert contains(spans, .5)
    assert not contains(spans, .1) and not contains(spans, .9)
    assert all(lo >= .2 and hi <= .8 for lo, hi in spans)


def test_union_of_own_quads_preserves_black_gap_not_tangent_hull():
    q = [rectangle(-.1, .4), rectangle(.6, 1.1)]
    spans = common({(1, 2): frames(5, q)})
    assert len(spans) == 2
    assert contains(spans, .2) and contains(spans, .8)
    assert not contains(spans, .5)


def test_parallel_alias_has_same_tangent_range_but_no_permission():
    spans = common({(1, 2): frames(5),
                    (1, 3): frames(5, [rectangle(-.2, 1.2, .3, .7)], start=10)})
    assert not spans


@pytest.mark.parametrize('angle,accepted', [(0., True), (.2, True), (math.pi, False),
                                         (.7, False)])
def test_signed_normal_gate_per_face_not_average(angle, accepted):
    n = (math.sin(angle), math.cos(angle))
    spans = common({(1, 2): frames(5, normals=[n])})
    assert contains(spans, .5) is accepted


def test_moving_mean_or_normal_reclips_same_immutable_evidence():
    hypotheses = {(1, 2): frames(5)}
    before = [item.quads.tobytes() for item in hypotheses[(1, 2)]]
    assert contains(common(hypotheses), .5)
    assert not common(hypotheses, segment=SEGMENT + (0., 1.))
    assert not common(hypotheses, normal=-NORMAL)
    assert before == [item.quads.tobytes() for item in hypotheses[(1, 2)]]
    assert contains(common(hypotheses), .5)  # Reprojection of old, still-owned evidence.


def test_rolling_retains_latest_k_union_latest_r_reliable_stamps_once():
    retained = ()
    for stamp in range(1, 11):
        retained = retain_frames(retained, frame(stamp, reliable=stamp in (1, 4)), 5, 2)
    assert [item.stamp_ns for item in retained] == [1, 4, 6, 7, 8, 9, 10]
    retained = retain_frames(retained, frame(11), 5, 2)
    assert [item.stamp_ns for item in retained] == [4, 7, 8, 9, 10, 11]
    assert len(retained) <= 7


def test_new_frame_never_mutates_retained_tuple_or_data():
    previous = frames(5)
    before = tuple((item.stamp_ns, item.quads.tobytes()) for item in previous)
    following = retain_frames(previous, frame(6), 5, 1)
    assert following is not previous
    assert before == tuple((item.stamp_ns, item.quads.tobytes()) for item in previous)


@pytest.mark.parametrize('stamp', [1, 4, 5])
def test_rolling_current_duplicate_or_out_of_order_source_rejected(stamp):
    with pytest.raises(ValueError):
        retain_frames(frames(5), frame(stamp), 5, 1)


@pytest.mark.parametrize('previous', [(frame(1), frame(1)), (frame(2), frame(1))])
def test_rolling_rejects_corrupted_duplicate_or_nonmonotone_past(previous):
    with pytest.raises(ValueError):
        retain_frames(previous, frame(3), 5, 1)


@pytest.mark.parametrize('k,r', [(0, 1), (5, 0), (5, 6)])
def test_rolling_rejects_invalid_count_contract(k, r):
    with pytest.raises(ValueError):
        retain_frames((), frame(1), k, r)


def test_retention_is_conservative_against_full_history_at_same_geometry():
    all_frames, retained = [], ()
    for stamp in range(1, 13):
        offset = (stamp % 3)*.125
        item = frame(stamp, [rectangle(offset, offset+.55)], reliable=stamp in (1, 6, 9))
        all_frames.append(item)
        retained = retain_frames(retained, item, 5, 1)
    for shift in (0., .125, -.125):
        segment = SEGMENT + (shift, 0.)
        full = common({(1, 2): tuple(all_frames)}, segment=segment)
        limited = common({(1, 2): retained}, segment=segment)
        for lo, hi in limited:
            assert any(old_lo <= lo and hi <= old_hi for old_lo, old_hi in full)


def test_rolling_does_not_delete_an_empty_competing_hypothesis():
    hypotheses = {(1, 2): frames(5), (1, 3): (frame(1, []),)}
    for stamp in range(6, 15):
        hypotheses[(1, 2)] = retain_frames(hypotheses[(1, 2)], frame(stamp), 5, 1)
    assert (1, 3) in hypotheses
    assert not common(hypotheses)


def exact_alpha(point, segment):
    start, end = [exact_point(p) for p in segment]
    delta, offset = subtract(end, start), subtract(exact_point(point), start)
    return sum((a*b for a, b in zip(offset, delta)), F(0)) / sum((d*d for d in delta), F(0))


@pytest.mark.parametrize('power,offset', [(-400, (0., 0.)), (400, (0., 0.)),
                                       (-15, (1.e6, -1.e6)), (0, (1.e14, -1.e14)),
                                       (0, (0., 0.))])
def test_projection_alpha_fraction_enclosure_random_scales_offsets(power, offset):
    rng = np.random.default_rng(102212)
    scale = math.ldexp(1., power)
    segments = rng.uniform(-2., 2., (24, 2, 2))*scale + offset
    points = rng.uniform(-3., 3., (24, 2))*scale + offset
    before = (segments.tobytes(), points.tobytes())
    lo, hi, valid = projection_alpha_bounds(points, segments)
    assert np.all(valid)
    for point, segment, lower, upper in zip(points, segments, lo, hi):
        alpha = exact_alpha(point, segment)
        assert fraction(lower) <= alpha <= fraction(upper)
    assert before == (segments.tobytes(), points.tobytes())


@pytest.mark.parametrize('alpha', [0., 1., .25, .5, -.00000001, 1.00000001])
def test_projection_encloses_true_end_or_permission_boundary_without_epsilon(alpha):
    point = np.array([[alpha, .13]])
    lo, hi, valid = projection_alpha_bounds(point, SEGMENT[None])
    assert valid[0]
    assert fraction(lo[0]) <= fraction(alpha) <= fraction(hi[0])
    # Exact equality in rounded alpha does not certify which side it lies on.
    assert lo[0] < alpha < hi[0]
    assert not (alpha <= lo[0] and hi[0] <= 1.)


@pytest.mark.parametrize('segment,point', [
    (np.zeros((2, 2)), np.zeros(2)),
    (np.full((2, 2), np.nan), np.zeros(2)),
    (SEGMENT, np.array([np.inf, 0.])),
    (SEGMENT*1.e200, np.zeros(2)),
    (SEGMENT*1.e-250, np.zeros(2)),
])
def test_projection_invalid_overflow_underflow_or_zero_length_fail_closed(segment, point):
    assert not projection_alpha_bounds(point[None], segment[None])[2][0]


def test_projection_empty_and_shape_contract():
    lo, hi, valid = projection_alpha_bounds(np.empty((0, 2)), np.empty((0, 2, 2)))
    assert lo.shape == hi.shape == valid.shape == (0,)
    with pytest.raises(ValueError):
        projection_alpha_bounds(np.zeros(2), SEGMENT[None])
    with pytest.raises(ValueError):
        projection_alpha_bounds(np.zeros((2, 2)), SEGMENT[None])
