"""Finite 2-D evidence shared by every observed contour continuation.

Cell-run identities may subdivide the same observed contour differently between
frames. A union of historical edges is therefore not itself a physical fork.
This module does NOT merge those identities. It certifies only the portion of
an unchanged public primitive independently observed under EVERY continuation.
The matcher must still choose its original global nearest primitive first.

Footprints are nominal projected binary64 quadrilaterals, not pose/calibration
confidence regions. All clipping arithmetic encloses those supplied vertices;
no tangent hull, spatial padding, or error covariance is manufactured here.
"""

from dataclasses import dataclass

import numpy as np

from .footprint_geometry import _point, _sub, _mul, _down, _up, _convex


REPRESENTATION = 'all_continuations_owned_quad_support_v1'
FIELDS = ('continuation_intervals', 'continuation_primitive',
          'continuation_frame_votes', 'continuation_reliable_votes',
          'continuation_last_stamp_ns')


def _immutable(value, dtype, shape):
    array = np.asarray(value, dtype=dtype)
    if array.shape != shape:
        raise ValueError('invalid continuation evidence shape')
    return np.frombuffer(array.tobytes(), dtype=dtype).reshape(shape)


@dataclass(frozen=True)
class ContinuationFrame:
    """One source frame, unioned before counting; no duplicate pixel votes."""
    stamp_ns: int
    quads: np.ndarray
    normals: np.ndarray
    reliable: np.ndarray

    def __post_init__(self):
        count = len(self.quads)
        if (type(self.stamp_ns) is not int or self.stamp_ns <= 0 or count > 200000
                or np.asarray(self.reliable).dtype.kind != 'b'):
            raise ValueError('invalid continuation frame')
        for name, dtype, shape in (('quads', np.float64, (count, 4, 2)),
                                  ('normals', np.float64, (count, 2)),
                                  ('reliable', np.bool_, (count,))):
            object.__setattr__(self, name, _immutable(getattr(self, name), dtype, shape))
        if (not np.all(np.isfinite(self.quads)) or not np.all(np.isfinite(self.normals))
                or np.any(np.abs(np.linalg.norm(self.normals, axis=1)-1.) > 1.e-6)):
            raise ValueError('invalid continuation geometry')

    @property
    def storage_units(self):
        # Empty/missing observations consume metadata capacity too.
        return 1 + len(self.quads)


def retain_frames(previous, current, minimum_frames, minimum_reliable):
    """Bound witnesses, not confirmation thresholds, without inventing votes.

    Keep the latest k source frames and the latest r frames with any reliable
    face. Their union is at most k+r frames. Dropping older evidence can only
    REMOVE support from this additional certificate; it never changes the old
    node/edge/domain votes or allows a newly seen hypothesis to borrow them.
    No query, current model, or desired match controls witness selection.
    """
    ordered = tuple(previous) + (current,)
    if (minimum_frames < 1 or not 1 <= minimum_reliable <= minimum_frames
            or any(second.stamp_ns <= first.stamp_ns for first, second in zip(ordered, ordered[1:]))):
        raise ValueError('invalid continuation witness order/count')
    frames = ordered
    keep = {frame.stamp_ns for frame in frames[-minimum_frames:]}
    reliable = [frame for frame in frames if np.any(frame.reliable)]
    keep.update(frame.stamp_ns for frame in reliable[-minimum_reliable:])
    return tuple(frame for frame in frames if frame.stamp_ns in keep)


def _cross(left, right):
    return _sub(_mul((left[0][..., 0], left[1][..., 0]),
                     (right[0][..., 1], right[1][..., 1])),
                _mul((left[0][..., 1], left[1][..., 1]),
                     (right[0][..., 0], right[1][..., 0])))


def clip_segment_quads(segment, quads):
    """Inward alpha intervals of one fixed segment inside convex quads.

    For alpha in [0,1], each half-plane has g=b+a*alpha. Outward coefficient
    bounds imply g >= b_lo+a_lo*alpha. Solving these sufficient inequalities
    with INWARD division gives a subset of the exact-real clip. Ambiguous,
    nonconvex, overflowing, zero-length and merely point contacts fail closed.
    Roots need not lie inside a quad; this is not the extraction root clipper.
    """
    segment, quads = np.asarray(segment, dtype=np.float64), np.asarray(quads, dtype=np.float64)
    if segment.shape != (2, 2) or quads.ndim != 3 or quads.shape[1:] != (4, 2):
        raise ValueError('invalid continuation clip shape')
    count = len(quads)
    output = np.zeros((count, 2))
    good = np.zeros(count, dtype=bool)
    if not count or not np.all(np.isfinite(segment)) or np.array_equal(segment[0], segment[1]):
        return output, good
    with np.errstate(over='ignore', under='ignore', invalid='ignore', divide='ignore'):
        edge = _sub(_point(np.roll(quads, -1, axis=1)), _point(quads))
        next_edge = (np.roll(edge[0], -1, axis=1), np.roll(edge[1], -1, axis=1))
        turn = _cross(edge, next_edge)
        positive = np.all(turn[0] > 0., axis=1)
        b = _cross(edge, _sub(_point(segment[0]), _point(quads)))
        a = _cross(edge, _sub(_point(segment[1]), _point(segment[0])))
        b_lower = np.where(positive[:, None], b[0], -b[1])
        a_lower = np.where(positive[:, None], a[0], -a[1])
        finite = np.all(np.isfinite(a_lower) & np.isfinite(b_lower), axis=1)
        rising, falling = a_lower > 0., a_lower < 0.
        ratio = -b_lower/a_lower
        lower = np.maximum(0., np.max(np.where(rising, _up(ratio), -np.inf), axis=1))
        upper = np.minimum(1., np.min(np.where(falling, _down(ratio), np.inf), axis=1))
        good = (_convex(quads) & finite & np.all(np.isfinite(quads), axis=(1, 2))
                & ~np.any((a_lower == 0.) & (b_lower < 0.), axis=1)
                & np.isfinite(lower) & np.isfinite(upper) & (lower < upper))
    output[good, 0], output[good, 1] = lower[good], upper[good]
    return output, good


def projection_alpha_bounds(points, segments):
    """Enclose real orthogonal alpha; rounded boundary equality is not proof.

    Permission intervals describe the nominal binary64-endpoint primitive.
    These bounds cover its exact-real projection, not physical calibration or
    the later rounded target/residual. Uncertain internal permission endpoints
    are rejected instead of enlarging their support by a numerical epsilon.
    """
    points, segments = np.asarray(points, dtype=np.float64), np.asarray(segments, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (2,) or segments.shape != (len(points), 2, 2):
        raise ValueError('invalid projection enclosure shape')
    with np.errstate(over='ignore', under='ignore', divide='ignore', invalid='ignore'):
        delta = _sub(_point(segments[:, 1]), _point(segments[:, 0]))
        offset = _sub(_point(points), _point(segments[:, 0]))
        numerator = _mul(offset, delta)
        denominator = _mul(delta, delta)
        lo = _down(numerator[0][:, 0]+numerator[0][:, 1])
        hi = _up(numerator[1][:, 0]+numerator[1][:, 1])
        dl = _down(denominator[0][:, 0]+denominator[0][:, 1])
        dh = _up(denominator[1][:, 0]+denominator[1][:, 1])
        quotients = np.stack((lo/dl, lo/dh, hi/dl, hi/dh))
        lower, upper = _down(np.min(quotients, axis=0)), _up(np.max(quotients, axis=0))
        good = ((dl > 0.) & np.isfinite(dh) & np.isfinite(lower) & np.isfinite(upper)
                & np.all(np.isfinite(points), axis=1) & np.all(np.isfinite(segments), axis=(1, 2)))
    return lower, upper, good


def _union(intervals):
    result = []
    for lower, upper in sorted((float(lower), float(upper)) for lower, upper in intervals):
        if lower >= upper:
            continue
        if result and lower <= result[-1][1]:
            result[-1] = (result[-1][0], max(result[-1][1], upper))
        else:
            result.append((float(lower), float(upper)))
    return tuple(result)


def _voted_intervals(frame_spans, trusted_spans, minimum_frames, minimum_reliable):
    events = {}
    for side, groups in enumerate((frame_spans, trusted_spans)):
        for spans in groups:
            for lower, upper in spans:
                events.setdefault(lower, [0, 0])[side] += 1
                events.setdefault(upper, [0, 0])[side] -= 1
    result, votes, trusted, previous = [], 0, 0, None
    for location, change in sorted(events.items()):
        if previous is not None and votes >= minimum_frames and trusted >= minimum_reliable:
            result.append((previous, location))
        votes += change[0]
        trusted += change[1]
        previous = location
    return _union(result)


def _intersection(first, second):
    result, i, j = [], 0, 0
    while i < len(first) and j < len(second):
        lower, upper = max(first[i][0], second[j][0]), min(first[i][1], second[j][1])
        if lower < upper:
            result.append((lower, upper))
        if first[i][1] < second[j][1]:
            i += 1
        else:
            j += 1
    return tuple(result)


def common_intervals(segment, normal, hypotheses, *, minimum_frames, minimum_reliable, cosine):
    """Query-independent, finite support present under every past hypothesis.

    Callers must also verify completeness against GLOBAL incident edges. A
    gap/FOV/unassigned neighbour (-1), unknown or under-observed hypothesis
    vetoes the certificate; an empty collection is never universal evidence.
    """
    if not hypotheses:
        return ()
    groups = sorted(hypotheses.items())
    frames_all = []
    for signature, frames in groups:
        if (min(signature) < 0 or len(frames) < minimum_frames
                or any(second.stamp_ns <= first.stamp_ns for first, second in zip(frames, frames[1:]))):
            return ()
        frames_all.extend(frames)
    # One vectorized clip for a primitive. Repeating interval arithmetic per
    # source frame dominated the otherwise bounded k+r witness calculation.
    quads = np.concatenate([frame.quads for frame in frames_all])
    normals = np.concatenate([frame.normals for frame in frames_all])
    reliable = np.concatenate([frame.reliable for frame in frames_all])
    spans, good = clip_segment_quads(segment, quads)
    good &= normals @ normal >= cosine
    common = ((0., 1.),)
    offset = 0
    for _, frames in groups:
        frame_spans, trusted_spans = [], []
        for frame in frames:
            stop = offset + len(frame.quads)
            current, admitted = spans[offset:stop], good[offset:stop]
            frame_spans.append(_union(current[admitted]))
            trusted_spans.append(_union(current[admitted & reliable[offset:stop]]))
            offset = stop
        observed = _voted_intervals(frame_spans, trusted_spans, minimum_frames, minimum_reliable)
        common = _intersection(common, observed)
        if not common:
            return ()
    return common
