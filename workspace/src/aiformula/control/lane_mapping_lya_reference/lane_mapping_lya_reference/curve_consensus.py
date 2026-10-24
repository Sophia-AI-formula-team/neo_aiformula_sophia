"""Bounded, nominal equal-frame curves in an unchanged creation chart.

Arithmetic shared by the explicit, opt-in v5 mapper and its scalar oracle.
V5 is not enabled in TeachMapEngine or approved for runtime control. Inputs
are complete ORIGINAL directed final-root pairs after the caller's old
root/direction/J/ownership gates. No inferred polynomial, invented root join,
pose, image, GNSS, truth, future interpolation or branch choice is made here.
Quads/adjacency/invalid flags are retained evidence, not a new2D permission.

One frame contributes at most one affine value on any abscissa. Latest5
frames plus latest1 reliable frame are bounded witnesses, not all history.
Ambiguous retained frames make the owner UNKNOWN; their labels are not dropped
to get a good answer. Fixed5+1 confirmation is additional to all old gates.
Outputs describe supplied binary64 finite piecewise models, never an exact
quadratic, subpixel physical truth or a successful runtime implementation.
"""
from dataclasses import dataclass, field
import math
from numbers import Integral, Real

import numpy as np
from .curve_rounding import step as _step
from .curve_evidence import PairPacket
from .curve_evidence_blocks import BlockPacket
from . import curve_evidence as _evidence
from . import curve_evidence_blocks as _blocks


def _reader_entry(owner, name):
    value = getattr(owner, name)
    function = value.fget if type(value) is property else value
    return owner, name, value, function, getattr(function, '__code__', None)


_NORMAL_READERS = tuple(_reader_entry(owner, name) for owner, names in (
    (PairPacket, ('normals', 'gather_original')),
    (BlockPacket, ('normals', 'gather_original')),
    (_evidence, ('_FIELDS',)),
    (_blocks, ('_decode', '_layout', '_prepared', '_LAYOUT', '_SEAL')),
    (_blocks.FaceBlock, ('get',)), (np, ('frombuffer',))) for name in names)


def _normal_readers_unchanged():
    return all(getattr(owner, name) is value
               and getattr(function, '__code__', None) is code
               for owner, name, value, function, code in _NORMAL_READERS)


REPRESENTATION = 'finite_equal_frame_creation_chart_directed_pairs_v2'
SOURCE_KIND = 'actual_final_observation_directed_root_pairs'
SAME_OWNER_MODEL_NEXT = 1
CROSS_OWNER_INCIDENT = 2
MINIMUM_FRAMES = 5
MINIMUM_RELIABLE_FRAMES = 1
MAX_RETAINED_FRAMES = MINIMUM_FRAMES + MINIMUM_RELIABLE_FRAMES
MAX_STORAGE_UNITS = 200000
MAX_OCCURRENCES = 200000
MAX_COORDINATE = 1.e7
MAX_STAMP = 2**63-1
UNIT_ROUNDOFF = 64.*np.finfo(np.float64).eps
_STATE_SEAL = object()
_STATE_UNITS = 20
_RAW_FIELDS = (
    ('roles', np.uint8, ()), ('face_ids', np.int64, (2,)),
    ('previous_next', np.int64, (2, 2)), ('contour_ids', np.int64, (2,)),
    ('endpoint_owners', np.int64, (2,)), ('model_roots', np.float64, (2, 2)),
    ('normals', np.float64, (2, 2)), ('quads', np.float64, (2, 4, 2)),
    ('quad_valid', np.bool_, (2,)), ('reliable', np.bool_, (2,)),
    ('valid', np.bool_, (2,)),
)


def _integer(value, name, minimum=0, maximum=MAX_STAMP):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral):
        raise ValueError(name+' must be an integer')
    value = int(value)
    if not minimum <= value <= maximum:
        raise ValueError(name+' outside bounded range')
    return value


def _number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(name+' must be real')
    if isinstance(value, np.floating) and value.dtype.itemsize > 8:
        raise ValueError(name+' must not silently round a wider supplied chart')
    try:
        value = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError(name+' is not binary64 representable') from error
    if not math.isfinite(value):
        raise ValueError(name+' must be finite')
    return value


def _vector(value, name, *, unit=False):
    try:
        if len(value) != 2:
            raise ValueError(name+' must have two components')
        result = tuple(_number(x, name) for x in value)
    except TypeError as error:
        raise ValueError(name+' must have two components') from error
    if max(map(abs, result)) > MAX_COORDINATE:
        raise ValueError(name+' outside coordinate bound')
    if unit and abs(math.hypot(*result)-1.) > UNIT_ROUNDOFF:
        raise ValueError(name+' must already be a unit vector')
    return result


def _key(owner_id, signed_side, signature):
    owner_id = _integer(owner_id, 'owner_id')
    side = _integer(signed_side, 'signed_side', minimum=-1, maximum=1)
    if side == 0:
        raise ValueError('signed_side must be -1 or +1')
    try:
        if len(signature) != 2:
            raise ValueError('signature must have two original neighbor IDs')
        signature = tuple(_integer(x, 'neighbor_id', minimum=-1) for x in signature)
    except TypeError as error:
        raise ValueError('invalid original continuation signature') from error
    return owner_id, side, signature


def _dot(first, second):
    return float(first[0])*float(second[0])+float(first[1])*float(second[1])


def _projection_bounds(point, origin, tangent):
    """Outward enclosure of exact supplied binary64 t.(point-origin)."""
    with np.errstate(all='ignore'):
        bounds = []
        for x, o, t in zip(point, origin, tangent):
            delta = float(x)-o
            lo, hi = _step(delta, -np.inf), _step(delta, np.inf)
            products = (lo*t, hi*t)
            bounds.append((_step(min(products), -np.inf),
                           _step(max(products), np.inf)))
        return (_step(bounds[0][0]+bounds[1][0], -np.inf),
                _step(bounds[0][1]+bounds[1][1], np.inf))


def _normal(derivative):
    length = math.hypot(*derivative)
    if not math.isfinite(length) or length == 0.:
        raise ArithmeticError('invalid finite curve derivative')
    return (-derivative[1]/length, derivative[0]/length)


@dataclass(frozen=True)
class FrameCurve:
    """Complete original directed-pair occurrences, never own-fragment samples.

    Each row has both source faces/owners/roots/normals/quads and original
    neighbor/contour/valid/reliable fields. Role1 is same-owner model-next;
    role2 is a cross-owner incident copy into one actual endpoint owner. Source
    next and reciprocal previous must agree with the supplied pair, never
    invented adjacency. A supplied source tag does not authenticate transport.

    Native exact base ndarrays only. Invalid endpoint placeholders are retained
    with valid=False/reliable=False/quad_valid=False and make the owner UNKNOWN.
    A valid endpoint's root/normal must be finite. Invalid quad placeholders
    are independently retained with quad_valid=False, not used as geometry or
    connection permission. No sorting/dedup/input mutation; fields own bytes.
    """
    owner_id: int
    signed_side: int
    signature: tuple
    stamp_ns: int
    roles: np.ndarray
    face_ids: np.ndarray
    previous_next: np.ndarray
    contour_ids: np.ndarray
    endpoint_owners: np.ndarray
    model_roots: np.ndarray
    normals: np.ndarray
    quads: np.ndarray
    quad_valid: np.ndarray
    reliable: np.ndarray
    valid: np.ndarray
    source_kind: str = SOURCE_KIND

    def __post_init__(self):
        owner, side, signature = _key(self.owner_id, self.signed_side, self.signature)
        stamp = _integer(self.stamp_ns, 'stamp_ns', minimum=1)
        if type(self.source_kind) is not str or self.source_kind != SOURCE_KIND:
            raise ValueError('only original directed final-root pairs are supported')
        if (type(self.face_ids) is not np.ndarray or self.face_ids.ndim != 2
                or self.face_ids.shape[1] != 2):
            raise ValueError('face_ids must contain both original pair endpoints')
        count = len(self.face_ids)
        if count > MAX_OCCURRENCES:
            raise ValueError('frame occurrence capacity')
        for name, dtype, tail in _RAW_FIELDS:
            value = getattr(self, name)
            if (type(value) is not np.ndarray or value.dtype != np.dtype(dtype)
                    or not value.dtype.isnative or value.shape != (count,)+tail):
                raise ValueError('invalid native dtype/shape: '+name)
        if self.storage_units > MAX_STORAGE_UNITS:
            raise ValueError('complete pair packet exceeds original global capacity bound')
        if (np.any(self.face_ids < 0) or np.any(self.previous_next < -1)
                or np.any(self.contour_ids < 0) or np.any(self.endpoint_owners < 0)):
            raise ValueError('invalid original face/neighbor/contour ID')
        if (np.any(self.face_ids[:, 0] == self.face_ids[:, 1])
                or np.any(self.previous_next[:, 0, 1] != self.face_ids[:, 1])
                or np.any(self.previous_next[:, 1, 0] != self.face_ids[:, 0])
                or np.any(self.contour_ids[:, 0] != self.contour_ids[:, 1])):
            raise ValueError('pair must be an original reciprocal directed contour next')
        same = self.roles == SAME_OWNER_MODEL_NEXT
        incident = self.roles == CROSS_OWNER_INCIDENT
        if not bool(np.all(same | incident)):
            raise ValueError('unknown original source-pair role')
        if (np.any(same & np.any(self.endpoint_owners != owner, axis=1))
                or np.any(incident & (self.endpoint_owners[:, 0] == self.endpoint_owners[:, 1]))
                or np.any(incident & ~np.any(self.endpoint_owners == owner, axis=1))):
            raise ValueError('role does not match both original endpoint owners')
        if np.any(self.reliable & ~self.valid) or np.any(self.quad_valid & ~self.valid):
            raise ValueError('an invalid source occurrence cannot be reliable')
        for name, mask in (('model_roots', self.valid), ('normals', self.valid),
                           ('quads', self.quad_valid)):
            value = getattr(self, name)[mask]
            if not np.isfinite(value).all() or np.any(np.abs(value) > MAX_COORDINATE):
                raise ValueError('nonfinite/unbounded valid source geometry: '+name)
        for normal in self.normals[self.valid]:
            if abs(math.hypot(float(normal[0]), float(normal[1]))-1.) > UNIT_ROUNDOFF:
                raise ValueError('valid source normals must already be unit vectors')
        # Temporary packet-local source consistency, bounded by its complete
        # capacity. Not a retained/global registry, not a dedup of occurrences.
        sources = {}
        per_face = ('previous_next', 'contour_ids', 'endpoint_owners', 'model_roots',
                    'normals', 'quads', 'quad_valid', 'reliable', 'valid')
        for row, pair in enumerate(self.face_ids):
            for endpoint, face in enumerate(pair):
                face = int(face)
                old = sources.get(face)
                if old is None:
                    sources[face] = (row, endpoint)
                elif any(np.asarray(getattr(self, name)[row, endpoint]).tobytes()
                         != np.asarray(getattr(self, name)[old]).tobytes() for name in per_face):
                    raise ValueError('same source face has contradictory original endpoint fields')
        for name, _, _ in _RAW_FIELDS:
            value = getattr(self, name)
            # CPython may intern a one-byte tobytes() result (the role of a
            # single incident pair).  Copy through a bytearray so distinct
            # packets also own distinct immutable nonempty backing storage.
            backing = bytes(bytearray(value.tobytes()))
            frozen = np.frombuffer(backing, dtype=value.dtype).reshape(value.shape)
            object.__setattr__(self, name, frozen)
        object.__setattr__(self, 'owner_id', owner)
        object.__setattr__(self, 'signed_side', side)
        object.__setattr__(self, 'signature', signature)
        object.__setattr__(self, 'stamp_ns', stamp)
        object.__setattr__(self, '_has_reliable', bool(np.any(np.all(self.reliable, axis=1))))

    @property
    def storage_units(self):
        # Record and signature headers plus all7 scalar/tag/cache entries;
        # each raw scalar/shape/array descriptor also counts. Not heap bytes.
        return 9+sum(value.size+1+value.ndim for value in
                     (getattr(self, name) for name, _, _ in _RAW_FIELDS))

    @property
    def payload_bytes(self):
        return sum(getattr(self, name).nbytes for name, _, _ in _RAW_FIELDS)


@dataclass(frozen=True)
class _Affine:
    lower: float
    upper: float
    intercept: float
    slope: float
    occurrence: int
    reliable: bool
    lower_endpoint: int
    upper_endpoint: int


@dataclass(frozen=True)
class _ChartFrame:
    packet: FrameCurve
    status: str
    pieces: tuple

    @property
    def storage_units(self):
        # Own record+packet reference+status+piece-sequence header, then all
        # raw packet storage and each eight-field affine record plus its header.
        # Endpoint indices identify the original face/root via the raw occurrence,
        # even when the source next is decreasing in the unchanged chart.
        return 4+self.packet.storage_units+9*len(self.pieces)


@dataclass(frozen=True)
class ConsensusPiece:
    """An independently finite nominal affine model, not a permission token.

    Endpoint world coordinates are rounded INSIDE this event/domain interval;
    no interpolation/connection to another exported piece is authorized.
    Contributors are distinct causal source stamps, not fragment/face counts.
    world_derivative is the rounded inverse-chart model derivative, not an
    exact-real equality or physical derivative measurement.
    """
    lower: float
    upper: float
    intercept: float
    slope: float
    contributors: tuple
    reliable_contributors: tuple
    source_stamp_ns: int
    segment: tuple
    world_derivative: tuple
    normal: tuple

    @property
    def frame_votes(self):
        return min(MINIMUM_FRAMES, len(self.contributors))

    @property
    def reliable_votes(self):
        return min(MINIMUM_RELIABLE_FRAMES, len(self.reliable_contributors))

    @property
    def storage_units(self):
        # Record1, coefficients/domain4, stamp1, world geometry8, sequence
        # headers5, two nested endpoint headers2, and ALL contributing stamps.
        return 21+len(self.contributors)+len(self.reliable_contributors)


@dataclass(frozen=True)
class CurveState:
    owner_id: int
    signed_side: int
    signature: tuple
    origin: tuple
    tangent: tuple
    normal_cosine: float
    max_storage_units: int
    source_frontier_ns: int
    frames: tuple
    pieces: tuple
    status: str
    reason: str
    _seal: object = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        if self._seal is not _STATE_SEAL:
            raise ValueError('use create_curve_state/advance_curve for immutable states')

    @property
    def storage_units(self):
        return (_STATE_UNITS+sum(frame.storage_units for frame in self.frames)
                +sum(piece.storage_units for piece in self.pieces))

    @property
    def payload_bytes(self):
        return sum(frame.packet.payload_bytes for frame in self.frames)


def create_curve_state(owner_id, signed_side, signature, origin, tangent, *,
                       normal_cosine, max_storage_units):
    """Fix one owner/side/continuation chart; no query-dependent origin update.

    normal_cosine is the caller's EXISTING gate, not a new configurable default.
    A future integration must bind it to the unchanged production policy.
    Caller also reserves this complete state in the old global capacity budget.
    """
    owner, side, signature = _key(owner_id, signed_side, signature)
    origin = _vector(origin, 'creation origin')
    tangent = _vector(tangent, 'creation tangent', unit=True)
    cosine = _number(normal_cosine, 'normal_cosine')
    if not 0. < cosine <= 1.:
        raise ValueError('normal_cosine must lie in (0,1]')
    capacity = _integer(max_storage_units, 'max_storage_units',
                        minimum=_STATE_UNITS, maximum=MAX_STORAGE_UNITS)
    return CurveState(owner, side, signature, origin, tangent, cosine, capacity,
                      0, (), (), 'empty', '', _STATE_SEAL)


def _chart_frame(state, packet):
    if not bool(np.all(packet.valid)):
        return _ChartFrame(packet, 'invalid_source_support', ())
    tangent, origin = state.tangent, state.origin
    normal = (-tangent[1], tangent[0])
    denominator = _dot(tangent, tangent)
    pieces, ends = [], []
    try:
        for index, (fragment, source_normals) in enumerate(zip(packet.model_roots, packet.normals)):
            if any(_dot(source_normal, normal) < state.normal_cosine for source_normal in source_normals):
                return _ChartFrame(packet, 'signed_direction', ())
            start, stop = fragment
            start_bounds = _projection_bounds(start, origin, tangent)
            stop_bounds = _projection_bounds(stop, origin, tangent)
            if not all(math.isfinite(x) for x in start_bounds+stop_bounds):
                return _ChartFrame(packet, 'fold_or_zero_or_uncertain_chart', ())
            if start_bounds[1] < stop_bounds[0]:
                lower_endpoint, upper_endpoint = 0, 1
                lower_bounds, upper_bounds = start_bounds, stop_bounds
            elif stop_bounds[1] < start_bounds[0]:
                lower_endpoint, upper_endpoint = 1, 0
                lower_bounds, upper_bounds = stop_bounds, start_bounds
            else:
                return _ChartFrame(packet, 'fold_or_zero_or_uncertain_chart', ())
            # Only a derived endpoint descriptor is reversed. The creation chart,
            # original directed next, signed normals and all raw bytes stay put.
            lower, upper = lower_bounds[1], upper_bounds[0]
            delta0 = tuple(float(x)-o for x, o in zip(start, origin))
            delta1 = tuple(float(x)-o for x, o in zip(stop, origin))
            u0, u1 = _dot(tangent, delta0), _dot(tangent, delta1)
            v0, v1 = _dot(normal, delta0), _dot(normal, delta1)
            slope = (v1-v0)/(u1-u0)
            intercept = v0-slope*u0
            derivative = tuple((tangent[i]+normal[i]*slope)/denominator for i in (0, 1))
            fitted_normal = _normal(derivative)
            if not all(math.isfinite(x) for x in (slope, intercept)+derivative):
                return _ChartFrame(packet, 'numerical', ())
            if any(_dot(fitted_normal, source_normal) < state.normal_cosine
                   for source_normal in source_normals):
                return _ChartFrame(packet, 'pair_direction', ())
            pieces.append(_Affine(lower, upper, intercept, slope, index,
                                  bool(np.all(packet.reliable[index])),
                                  lower_endpoint, upper_endpoint))
            ends.append((lower_bounds, upper_bounds,
                         tuple(fragment[lower_endpoint]), tuple(fragment[upper_endpoint])))
    except ArithmeticError:
        return _ChartFrame(packet, 'numerical', ())
    # Sort only the derived descriptors; complete raw occurrence order survives.
    order = sorted(range(len(pieces)), key=lambda i: (pieces[i].lower, pieces[i].upper, i))
    for first, second in zip(order, order[1:]):
        previous, following = ends[first], ends[second]
        if (packet.face_ids[first, pieces[first].upper_endpoint]
                == packet.face_ids[second, pieces[second].lower_endpoint]
                and previous[3] == following[2]):
            # Same original face and identical final root, not world-coordinate
            # coincidence/snap between different branches. Source fields above
            # were checked consistent for every occurrence of this face.
            continue
        if not previous[1][1] < following[0][0]:
            return _ChartFrame(packet, 'overlap_or_nonunique_chart', ())
    return _ChartFrame(packet, 'known', tuple(pieces[i] for i in order))


def _endpoint_scalar(state, lower, upper, intercept, slope, *, first):
    tangent = state.tangent
    normal = (-tangent[1], tangent[0])
    denominator = _dot(tangent, tangent)
    parameter = lower if first else upper
    with np.errstate(all='ignore'):
        for _ in range(8):
            v = intercept+slope*parameter
            point = tuple(state.origin[i]+(tangent[i]*parameter+normal[i]*v)/denominator
                          for i in (0, 1))
            if not all(math.isfinite(x) and abs(x) <= MAX_COORDINATE for x in point):
                raise ArithmeticError('unrepresentable finite endpoint')
            bounds = _projection_bounds(point, state.origin, tangent)
            if bounds[0] >= lower and bounds[1] <= upper:
                return point, bounds
            if not all(math.isfinite(x) for x in bounds):
                raise ArithmeticError('uncertain endpoint projection')
            # A chart coordinate near zero may have a much smaller ULP than
            # the rounded world-coordinate addition. Move by its actual
            # representational envelope, never by a geometric epsilon/slack.
            resolution = 0.
            for i in (0, 1):
                offset = (tangent[i]*parameter+normal[i]*v)/denominator
                scale = max(abs(state.origin[i]), abs(offset), abs(point[i]))
                resolution += abs(tangent[i])*(_step(scale, np.inf)-scale)
            if first:
                parameter = _step(parameter+max(resolution, lower-bounds[0], 0.), np.inf)
            else:
                parameter = _step(parameter-max(resolution, bounds[1]-upper, 0.), -np.inf)
            if not lower < parameter < upper:
                break
    raise ArithmeticError('cannot certify rounded endpoint inside original finite domain')


def _endpoint(state, lower, upper, intercept, slope, *, first):
    return _endpoint_native.dispatch(state, lower, upper, intercept, slope,
        first=first, bindings=_ENDPOINT_BINDINGS)


def _overlay(state, frames):
    if any(frame.status != 'known' for frame in frames):
        reason = next(frame.status for frame in frames if frame.status != 'known')
        return (), 'unknown', reason
    events = sorted({bound for frame in frames for piece in frame.pieces
                     for bound in (piece.lower, piece.upper)})
    cursors, result = [0]*len(frames), []
    normal_tables = {}
    cache_normals = _normal_readers_unchanged()
    def occurrence_normals(packet, ordinal):
        # Immutable source packets used to reconstruct the entire normal table
        # for every event interval. Gather at its first original read only, then
        # reuse within THIS overlay invocation. Nothing is retained in a state.
        if cache_normals and type(packet) in (PairPacket, BlockPacket):
            key = id(packet)
            table = normal_tables.get(key)
            if table is None:
                table = packet.normals
                normal_tables[key] = table
            return table[ordinal]
        return packet.normals[ordinal]
    normal = (-state.tangent[1], state.tangent[0])
    denominator = _dot(state.tangent, state.tangent)
    try:
        for lower, upper in zip(events, events[1:]):
            active = []
            for slot, frame in enumerate(frames):
                cursor = cursors[slot]
                while cursor < len(frame.pieces) and frame.pieces[cursor].upper <= lower:
                    cursor += 1
                cursors[slot] = cursor
                if cursor < len(frame.pieces):
                    piece = frame.pieces[cursor]
                    if piece.lower <= lower and piece.upper >= upper:
                        active.append((frame.packet, piece))
            trusted = tuple(packet.stamp_ns for packet, piece in active if piece.reliable)
            if len(active) < MINIMUM_FRAMES or len(trusted) < MINIMUM_RELIABLE_FRAMES:
                continue
            intercept = math.fsum(piece.intercept for _, piece in active)/len(active)
            slope = math.fsum(piece.slope for _, piece in active)/len(active)
            derivative = tuple((state.tangent[i]+normal[i]*slope)/denominator for i in (0, 1))
            fused_normal = _normal(derivative)
            if not all(math.isfinite(x) for x in (intercept, slope)+derivative):
                raise ArithmeticError('nonfinite fused affine model')
            if any(_dot(fused_normal, endpoint_normal) < state.normal_cosine
                   for packet, piece in active for endpoint_normal in occurrence_normals(packet, piece.occurrence)):
                return (), 'unknown', 'fused_direction'
            first, first_bounds = _endpoint(state, lower, upper, intercept, slope, first=True)
            second, second_bounds = _endpoint(state, lower, upper, intercept, slope, first=False)
            if not first_bounds[1] < second_bounds[0]:
                raise ArithmeticError('exported finite curve collapsed')
            stamps = tuple(packet.stamp_ns for packet, _ in active)
            result.append(ConsensusPiece(lower, upper, intercept, slope, stamps,
                                         trusted, max(stamps), (first, second), derivative, fused_normal))
    except ArithmeticError:
        return (), 'unknown', 'numerical_export'
    return tuple(result), ('confirmed' if result else 'unconfirmed'), ''


def advance_curve(previous, current, *, available_before_ns):
    """Prepare a new immutable state or raise without modifying previous/input.

    available_before_ns is an explicit caller assertion of the already-arrived
    source-time frontier; this helper does not authenticate transport chronology.
    UNKNOWN retains the accepted packet/frontier but exposes NO model pieces.
    Capacity/type/key/time rejection leaves the prior state completely unchanged.
    """
    if type(previous) is not CurveState or previous._seal is not _STATE_SEAL:
        raise ValueError('previous must be a prepared CurveState')
    if type(current) is not FrameCurve:
        raise ValueError('current must be a complete FrameCurve')
    available = _integer(available_before_ns, 'available_before_ns', minimum=1)
    if (current.owner_id, current.signed_side, current.signature) != (
            previous.owner_id, previous.signed_side, previous.signature):
        raise ValueError('owner/signed side/continuation must not be merged')
    if not previous.source_frontier_ns < current.stamp_ns <= available:
        raise ValueError('duplicate/noncausal/future source frame')
    packets = tuple(frame.packet for frame in previous.frames)+(current,)
    keep = {packet.stamp_ns for packet in packets[-MINIMUM_FRAMES:]}
    reliable = tuple(packet for packet in packets if packet._has_reliable)
    keep.update(packet.stamp_ns for packet in reliable[-MINIMUM_RELIABLE_FRAMES:])
    retained = tuple(frame for frame in previous.frames if frame.packet.stamp_ns in keep)
    # Reject the raw lower bound before deriving new geometry. No packet slicing,
    # silent subsampling, cap expansion or hypothesis eviction repairs capacity.
    if _STATE_UNITS+sum(frame.storage_units for frame in retained)+current.storage_units+4 > previous.max_storage_units:
        raise ValueError('complete retained packet capacity exceeded')
    frames = retained+(_chart_frame(previous, current),)
    if len(frames) > MAX_RETAINED_FRAMES:
        raise ValueError('invalid bounded witness retention')
    pieces, status, reason = _overlay(previous, frames)
    candidate = CurveState(previous.owner_id, previous.signed_side, previous.signature,
        previous.origin, previous.tangent, previous.normal_cosine, previous.max_storage_units,
        current.stamp_ns, frames, pieces, status, reason, _STATE_SEAL)
    if candidate.storage_units > previous.max_storage_units:
        raise ValueError('complete derived curve capacity exceeded; update rejected')
    return candidate


# Registration stores only class/seal identities. It does not retain any map
# state and avoids a circular import with compact/retained factory modules.
from . import curve_endpoint_native as _endpoint_native
import sys as _sys
_endpoint_native._register_state_type(CurveState, _STATE_SEAL)
_ENDPOINT_BINDINGS = _endpoint_native.bind_helpers(_sys.modules[__name__])
