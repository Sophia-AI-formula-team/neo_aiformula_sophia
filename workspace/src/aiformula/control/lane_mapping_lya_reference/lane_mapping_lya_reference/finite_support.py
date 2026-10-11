"""Bounded causal finite consensus domains; no mapper or sensor dependencies.

Intervals describe a finite consensus model, not raw geometric ground truth.
The creation chart never moves. Adjacent closed pieces may share an endpoint;
their counters are alternative conservative certificates, never summed there.
Only positive-length pieces are stored, so isolated endpoint coincidences do
not become extra support. All public state and exported buffers are immutable.
"""
from dataclasses import dataclass
import math
import numbers
import struct

import numpy as np

try:
    import _lane_mapping_lya_native as _FINITE_NATIVE
except ImportError:
    _FINITE_NATIVE = None

# An older installed extension may still accelerate unrelated graph kernels.
# This additional optional kernel is selected only by its own exact API.
_NATIVE_PROJECTION = (getattr(_FINITE_NATIVE, 'finite_projection_bounds', None)
    if getattr(_FINITE_NATIVE, 'FINITE_PROJECTION_BOUNDS_API_VERSION', None) == 1 else None)

MAX_ITEMS = 200000
MAX_COORDINATE = 1.e7
MAX_STAMP = 2**63 - 1
MAX_VOTES = 2**31 - 1
UNIT_TOLERANCE = 64. * np.finfo(np.float64).eps


def _integer(value, name, minimum=1, maximum=MAX_STAMP):
    # Internal immutable states already hold exact builtins. Keep the public
    # validation/conversion path for every other type, including subclasses.
    if type(value) is not int:
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Integral):
            raise ValueError(name + ' must be an integer')
        value = int(value)
    if not minimum <= value <= maximum:
        raise ValueError(name + ' outside supported range')
    return value


def _number(value, name):
    if type(value) is float:
        result = value
    else:
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real):
            raise ValueError(name + ' must be a real number')
        try:
            result = float(value)
        except (ValueError, OverflowError) as error:
            raise ValueError(name + ' cannot be represented as float64') from error
    if not math.isfinite(result):
        raise ValueError(name + ' must be finite')
    return result


def _vector(value, name, unit=False):
    try:
        if len(value) != 2:
            raise ValueError(name + ' must have two elements')
        result = tuple(_number(v, name) for v in value)
    except TypeError as error:
        raise ValueError(name + ' must have two elements') from error
    if unit:
        if abs(math.hypot(*result) - 1.) > UNIT_TOLERANCE:
            raise ValueError(name + ' must already be unit length')
    elif max(map(abs, result)) > MAX_COORDINATE:
        raise ValueError(name + ' outside coordinate bound')
    return result


def _thresholds(frame_votes, reliable_votes):
    frame_votes = _integer(frame_votes, 'minimum_frame_votes', maximum=MAX_VOTES)
    reliable_votes = _integer(reliable_votes, 'minimum_reliable_votes', maximum=frame_votes)
    return frame_votes, reliable_votes


def _cosine(value):
    value = _number(value, 'normal_cosine')
    if not 0. < value <= 1.:
        raise ValueError('normal_cosine must lie in (0, 1]')
    return value


@dataclass(frozen=True)
class Interval:
    lower: float
    upper: float
    frame_votes: int
    reliable_votes: int
    last_stamp_ns: int

    def __post_init__(self):
        lower = _number(self.lower, 'interval.lower')
        upper = _number(self.upper, 'interval.upper')
        if not lower < upper or max(abs(lower), abs(upper)) > 4. * MAX_COORDINATE:
            raise ValueError('interval requires bounded positive extent')
        votes = _integer(self.frame_votes, 'interval.frame_votes', maximum=MAX_VOTES)
        reliable = _integer(self.reliable_votes, 'interval.reliable_votes', minimum=0, maximum=votes)
        stamp = _integer(self.last_stamp_ns, 'interval.last_stamp_ns')
        for name, value in (('lower', lower), ('upper', upper), ('frame_votes', votes),
                            ('reliable_votes', reliable), ('last_stamp_ns', stamp)):
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class DomainState:
    origin: tuple
    tangent: tuple
    intervals: tuple
    source_stamp_ns: int

    def __post_init__(self):
        origin = _vector(self.origin, 'origin')
        tangent = _vector(self.tangent, 'tangent', unit=True)
        stamp = _integer(self.source_stamp_ns, 'source_stamp_ns')
        try:
            intervals = tuple(self.intervals)
        except TypeError as error:
            raise ValueError('intervals must be a finite sequence') from error
        if len(intervals) > MAX_ITEMS or any(type(item) is not Interval for item in intervals):
            raise ValueError('invalid interval sequence or capacity')
        for index, piece in enumerate(intervals):
            if piece.last_stamp_ns > stamp:
                raise ValueError('interval observation is newer than state frontier')
            if index and intervals[index-1].upper > piece.lower:
                raise ValueError('interval interiors must be sorted and disjoint')
        object.__setattr__(self, 'origin', origin)
        object.__setattr__(self, 'tangent', tangent)
        object.__setattr__(self, 'intervals', intervals)
        object.__setattr__(self, 'source_stamp_ns', stamp)


def _nextafter_binary64(value, direction):
    """One IEEE binary64 step, including signed zero, on Python 3.8.

    This replaces only the scalar operation, not the interval arithmetic order
    or its conservative outward rounding. Integer bit stepping avoids creating
    a NumPy error-state context for every scalar subnormal operation.
    """
    if math.isnan(value) or math.isnan(direction):
        return value + direction
    if value == direction:
        return direction
    if value == 0.:
        return math.copysign(float.fromhex('0x0.0000000000001p-1022'), direction)
    bits = struct.unpack('>Q', struct.pack('>d', value))[0]
    bits += 1 if (direction > value) == (value > 0.) else -1
    return struct.unpack('>d', struct.pack('>Q', bits))[0]


# Foxy's Python 3.8 has no stdlib nextafter. Both implementations have the
# same bit-exact contract, tested independently against NumPy's IEEE oracle.
_nextafter = getattr(math, 'nextafter', _nextafter_binary64)


def _outward(value):
    if not math.isfinite(value):
        raise ValueError('nonfinite interval arithmetic')
    lower, upper = _nextafter(value, -math.inf), _nextafter(value, math.inf)
    if not math.isfinite(lower) or not math.isfinite(upper):
        raise ValueError('unbounded interval arithmetic')
    return lower, upper


def _sub_bounds(first, second):
    if first == second:
        return 0., 0.
    return _outward(first - second)


def _multiply_bounds(interval, scalar):
    if scalar == 0. or interval == (0., 0.):
        return 0., 0.
    lower, upper = interval if scalar > 0. else interval[::-1]
    return _outward(lower * scalar)[0], _outward(upper * scalar)[1]


def projection_bounds(point, origin, tangent):
    """IEEE enclosure of the exact dot product of the supplied float64 values.

    Each nontrivial subtraction, multiplication and sum is rounded OUTWARD.
    This numerical enclosure is subsequently used to shrink, never extend,
    a measured segment's supported interval.
    """
    parts = [_multiply_bounds(_sub_bounds(float(point[i]), float(origin[i])), float(tangent[i]))
             for i in (0, 1)]
    if parts[0] == (0., 0.):
        return parts[1]
    if parts[1] == (0., 0.):
        return parts[0]
    return _outward(parts[0][0] + parts[1][0])[0], _outward(parts[0][1] + parts[1][1])[1]


def _validated_projection_bounds(point, origin, tangent):
    """Internal finite inputs only; public conversion/error ordering is intact."""
    if _NATIVE_PROJECTION is None:
        return projection_bounds(point, origin, tangent)
    return _NATIVE_PROJECTION(float(point[0]), float(origin[0]), float(tangent[0]),
                              float(point[1]), float(origin[1]), float(tangent[1]))


def _arrays(segments, normals, reliable):
    try:
        segments, normals, reliable = np.asarray(segments), np.asarray(normals), np.asarray(reliable)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError('malformed segment arrays') from error
    if segments.ndim != 3 or segments.shape[1:] != (2, 2) or len(segments) > MAX_ITEMS:
        raise ValueError('segments must be bounded N x 2 x 2')
    if normals.shape != segments.shape or reliable.shape != (len(segments),):
        raise ValueError('endpoint normal or reliability shape mismatch')
    if segments.dtype.kind not in 'fi' or normals.dtype.kind not in 'fi' or reliable.dtype.kind != 'b':
        raise ValueError('segments/normals must be numeric; reliable must be boolean')
    segments, normals = segments.astype(np.float64, copy=False), normals.astype(np.float64, copy=False)
    if not np.all(np.isfinite(segments)) or np.any(np.abs(segments) > MAX_COORDINATE):
        raise ValueError('nonfinite or out-of-bound segments')
    if (not np.all(np.isfinite(normals)) or np.any(np.abs(normals) > 1. + UNIT_TOLERANCE)
            or np.any(np.abs(np.linalg.norm(normals, axis=2) - 1.) > UNIT_TOLERANCE)):
        raise ValueError('endpoint normals must already be finite unit vectors')
    return segments, normals, reliable


def _connected_spans(edges, projected):
    """Inner projection of each EXACT physical-endpoint connected component.

    A continuous affine image of a connected union of closed segments is an
    interval. Thus only its exterior extrema need inward rounding. Shrinking
    each A-B and B-C independently invents an unsupported strip at their shared
    B, even though the original observed union contains that point. No distance
    tolerance, approximate join, historical vote or normal inference is used.
    """
    parents, ranks = {}, {}

    def root(vertex):
        if vertex not in parents:
            parents[vertex], ranks[vertex] = vertex, 0
        while parents[vertex] != vertex:
            parents[vertex] = parents[parents[vertex]]
            vertex = parents[vertex]
        return vertex

    for first, second in edges:
        a, b = root(first), root(second)
        if a == b:
            continue
        if ranks[a] < ranks[b]:
            a, b = b, a
        parents[b] = a
        if ranks[a] == ranks[b]:
            ranks[a] += 1
    extrema = {}
    for vertex in parents:
        owner = root(vertex)
        lower, upper = projected[vertex]
        if owner not in extrema:
            extrema[owner] = [upper, lower]
        else:
            extrema[owner][0] = min(extrema[owner][0], upper)
            extrema[owner][1] = max(extrema[owner][1], lower)
    return [(lower, upper) for lower, upper in extrema.values() if lower < upper]


def _current_union(segments, normals, reliable, origin, tangent, cosine):
    normal = (-tangent[1], tangent[0])
    vertices, projected, edges, trusted_edges = {}, [], [], []
    for segment, directions, trusted in zip(segments, normals, reliable):
        if any(float(np.dot(direction, normal)) < cosine for direction in directions):
            continue
        pair = []
        for point in segment:
            # Numeric equality of both finite binary64 coordinates, never a
            # rounded spatial bin. Signed zeros denote the same physical point.
            key = tuple(map(float, point))
            if key not in vertices:
                vertices[key] = len(projected)
                projected.append(_validated_projection_bounds(point, origin, tangent))
            pair.append(vertices[key])
        edges.append(tuple(pair))
        if trusted:
            trusted_edges.append(tuple(pair))
    # Reliability has its OWN connectivity: an unreliable A-B bridge must not
    # join two otherwise separated trusted domains. All frame counts still
    # reduce to a boolean union before the one-per-source-frame vote below.
    all_trusted = len(edges) == len(trusted_edges)
    spans = [(lower, upper, all_trusted) for lower, upper in _connected_spans(edges, projected)]
    if trusted_edges and not all_trusted:
        spans.extend((lower, upper, True) for lower, upper in _connected_spans(trusted_edges, projected))
    events = {}
    for lower, upper, trusted in spans:
        for edge, count, rel in ((lower, 1, int(trusted)), (upper, -1, -int(trusted))):
            row = events.setdefault(edge, [0, 0]); row[0] += count; row[1] += rel
    boundaries = sorted(events)
    pieces, count, trusted = [], 0, 0
    for index, lower in enumerate(boundaries[:-1]):
        count += events[lower][0]; trusted += events[lower][1]
        if count:
            upper, rel = boundaries[index+1], trusted > 0
            if pieces and pieces[-1][1] == lower and pieces[-1][2] == rel:
                pieces[-1] = (pieces[-1][0], upper, rel)
            else:
                pieces.append((lower, upper, rel))
    return pieces


def advance_domain(previous, *, origin, current_frame_normal, segments, normals, reliable,
                   stamp_ns, minimum_frame_votes, minimum_reliable_votes, candidate_ttl_ns,
                   normal_cosine, max_intervals):
    """Return one new immutable state, or raise without changing the old state.

    TTL zero disables expiry, matching existing consensus. Only UNCONFIRMED
    pieces expire. One entire source frame contributes at most one vote per
    position, irrespective of segment density or overlapping runs.
    """
    origin = _vector(origin, 'origin')
    frame_normal = _vector(current_frame_normal, 'current_frame_normal', unit=True)
    stamp = _integer(stamp_ns, 'stamp_ns')
    minimum, minimum_reliable = _thresholds(minimum_frame_votes, minimum_reliable_votes)
    ttl = _integer(candidate_ttl_ns, 'candidate_ttl_ns', minimum=0)
    cosine = _cosine(normal_cosine)
    capacity = _integer(max_intervals, 'max_intervals', maximum=MAX_ITEMS)
    segments, normals, reliable = _arrays(segments, normals, reliable)
    if previous is not None and type(previous) is not DomainState:
        raise ValueError('previous must be DomainState or None')
    if previous is None:
        tangent, old = (frame_normal[1], -frame_normal[0]), ()
    else:
        if stamp <= previous.source_stamp_ns:
            raise ValueError('source stamp must strictly increase')
        if len(previous.intervals) > capacity:
            raise ValueError('previous state exceeds requested capacity')
        origin, tangent = previous.origin, previous.tangent
        if any(p.frame_votes > minimum or p.reliable_votes > minimum_reliable for p in previous.intervals):
            raise ValueError('previous counters exceed declared saturation bounds')
        old = tuple(p for p in previous.intervals if not
                    (ttl and stamp-p.last_stamp_ns > ttl and
                     (p.frame_votes < minimum or p.reliable_votes < minimum_reliable)))
    current = _current_union(segments, normals, reliable, origin, tangent, cosine)
    boundaries = sorted({v for p in old for v in (p.lower, p.upper)} |
                        {v for p in current for v in p[:2]})
    result, old_index, current_index = [], 0, 0
    for lower, upper in zip(boundaries, boundaries[1:]):
        while old_index < len(old) and old[old_index].upper <= lower:
            old_index += 1
        while current_index < len(current) and current[current_index][1] <= lower:
            current_index += 1
        past = old[old_index] if old_index < len(old) and old[old_index].lower <= lower else None
        seen = current[current_index] if current_index < len(current) and current[current_index][0] <= lower else None
        if past is None and seen is None:
            continue
        votes = min(minimum, (0 if past is None else past.frame_votes) + int(seen is not None))
        reliable_votes = min(minimum_reliable, (0 if past is None else past.reliable_votes) +
                             int(seen is not None and seen[2]))
        last = stamp if seen is not None else past.last_stamp_ns
        # Coalesce numeric pieces BEFORE constructing validated public values.
        # Bounds/counters/stamps and the capacity decision are unchanged; every
        # final interval still passes the full Interval/DomainState validation.
        # Intermediate fragments would be immediately replaced by the union.
        piece = (lower, upper, votes, reliable_votes, last)
        if (result and result[-1][1] == lower and
                result[-1][2:] == (votes, reliable_votes, last)):
            result[-1] = (result[-1][0], upper, votes, reliable_votes, last)
        else:
            result.append(piece)
            if len(result) > capacity:
                raise ValueError('finite domain interval capacity exceeded; update rejected')
    return DomainState(origin, tangent, tuple(Interval(*piece) for piece in result), stamp)


def _compact_confirmed_domain(state, minvotes, minreliable):
    """Canonical confirmed unions for a session with FROZEN vote thresholds.

    Only BoundaryConsensus's immutable configuration may use this operation.
    Public advance_domain retains its original per-location stamp semantics.
    Confirmed pieces never expire and their counters are saturated, so their
    individual last-seen stamps no longer affect future votes or TTL. Keep the
    maximum contributing frontier, exactly as certified_segments already does.
    Unconfirmed pieces and every positive gap remain untouched. Do not use a
    compacted state in a session which changes the confirmation thresholds.

    Storage counts can decrease; the unchanged capacity then bounds canonical
    pieces, not redundant partitions of the same confirmed geometric domain.
    """
    if type(state) is not DomainState:
        raise ValueError('state must be DomainState')
    minimum, trusted = _thresholds(minvotes, minreliable)
    pieces, result, index = state.intervals, [], 0
    while index < len(pieces):
        first = pieces[index]
        following = index + 1
        upper, latest = first.upper, first.last_stamp_ns
        if first.frame_votes == minimum and first.reliable_votes == trusted:
            while following < len(pieces):
                other = pieces[following]
                if (other.lower != upper or other.frame_votes != minimum
                        or other.reliable_votes != trusted):
                    break
                upper, latest = other.upper, max(latest, other.last_stamp_ns)
                following += 1
        result.append(first if following == index + 1 else
                      Interval(first.lower, upper, minimum, trusted, latest))
        index = following
    if len(result) == len(pieces):
        return state
    return DomainState(state.origin, state.tangent, tuple(result), state.source_stamp_ns)


def _freeze(value, shape, dtype):
    array = np.asarray(value, dtype=dtype).reshape(shape)
    return np.frombuffer(array.tobytes(), dtype=dtype).reshape(shape)


def _empty_export():
    return (_freeze([], (0, 2, 2), np.float64), _freeze([], (0,), np.int64),
            _freeze([], (0,), np.int64), _freeze([], (0,), np.int64))


def _export_endpoint(state, point, tangent, denominator, mean_coordinate, piece, lower):
    parameter = piece.lower if lower else piece.upper
    for _ in range(8):
        scale = (parameter-mean_coordinate)/denominator
        endpoint = tuple(point[i] + scale*tangent[i] for i in (0, 1))
        bounds = _validated_projection_bounds(endpoint, state.origin, state.tangent)
        if bounds[0] >= piece.lower and bounds[1] <= piece.upper:
            return endpoint
        # Only correct the proven numerical envelope, not geometric mismatch.
        # No fallback to an endpoint outside the observed domain is permitted.
        # A near-zero chart parameter can have an ULP much smaller than the
        # world-coordinate addition. Its nextafter alone may leave the world
        # endpoint unchanged forever. Advance by one representational envelope
        # of the actual world arithmetic, then re-certify INSIDE the old domain.
        resolution = sum(abs(state.tangent[i]) * (
            _nextafter(max(abs(point[i]), abs(scale*tangent[i]), abs(endpoint[i])), math.inf)
            - max(abs(point[i]), abs(scale*tangent[i]), abs(endpoint[i]))) for i in (0, 1))
        if lower:
            parameter = _nextafter(parameter + max(resolution, piece.lower-bounds[0]), math.inf)
        else:
            parameter = _nextafter(parameter - max(resolution, bounds[1]-piece.upper), -math.inf)
        if not piece.lower < parameter < piece.upper:
            break
    raise ValueError('finite model endpoint cannot be represented inside certified chart domain')


def certified_segments(state, point, current_normal, normal_cosine, minvotes, minreliable):
    """Export immutable (segments, frame_votes, reliable_votes, last_stamp_ns).

    The line passes through the current fused point with current normal.
    Endpoints solve the ORIGINAL creation-chart coordinate, never an offset
    relative to the moving mean. This is a finite consensus model, not truth.
    """
    if type(state) is not DomainState:
        raise ValueError('state must be DomainState')
    point = _vector(point, 'point')
    normal = _vector(current_normal, 'current_normal', unit=True)
    cosine = _cosine(normal_cosine)
    minimum, minimum_reliable = _thresholds(minvotes, minreliable)
    chart_normal = (-state.tangent[1], state.tangent[0])
    if sum(a*b for a, b in zip(normal, chart_normal)) < cosine:
        return _empty_export()
    tangent = (normal[1], -normal[0])
    denominator = sum(a*b for a, b in zip(state.tangent, tangent))
    if not denominator > 0.:
        raise ValueError('invalid current-to-creation chart intersection')
    mean_coordinate = sum(t*(p-o) for t, p, o in zip(state.tangent, point, state.origin))
    segments, votes, reliable, stamps = [], [], [], []
    confirmed = []
    for piece in state.intervals:
        if piece.frame_votes < minimum or piece.reliable_votes < minimum_reliable:
            continue
        # Adjacent pointwise-certified domains form one finite supported union.
        # Do NOT move this merge into the state: its per-location last stamps
        # are needed for expiry of still-unconfirmed pieces. Export counts are
        # conservative lower bounds; stamp is the latest contributing frontier,
        # not a claim that every location was observed on that same frame.
        if confirmed and confirmed[-1][1] == piece.lower:
            old = confirmed[-1]
            confirmed[-1] = (old[0], piece.upper,
                min(old[2], piece.frame_votes), min(old[3], piece.reliable_votes),
                max(old[4], piece.last_stamp_ns))
        else:
            confirmed.append((piece.lower, piece.upper, piece.frame_votes,
                              piece.reliable_votes, piece.last_stamp_ns))
    for values in confirmed:
        piece = Interval(*values)
        first = _export_endpoint(state, point, tangent, denominator, mean_coordinate, piece, True)
        second = _export_endpoint(state, point, tangent, denominator, mean_coordinate, piece, False)
        progress = sum((second[i]-first[i])*tangent[i] for i in (0, 1))
        if not math.isfinite(progress) or not progress > 0.:
            raise ValueError('certified finite segment collapsed or reversed during export')
        segments.append((first, second)); votes.append(piece.frame_votes)
        reliable.append(piece.reliable_votes); stamps.append(piece.last_stamp_ns)
    count = len(segments)
    return (_freeze(segments, (count, 2, 2), np.float64), _freeze(votes, (count,), np.int64),
            _freeze(reliable, (count,), np.int64), _freeze(stamps, (count,), np.int64))
