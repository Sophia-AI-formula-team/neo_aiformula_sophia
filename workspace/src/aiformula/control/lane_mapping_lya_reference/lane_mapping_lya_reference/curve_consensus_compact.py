"""Shared-evidence candidate using the unchanged finite curve arithmetic.

Used by the explicit BoundaryConsensus(curve_geometry=True) development path.
TeachMapEngine/control do not enable that path; runtime acceptance is pending.
The old scalar/header gate remains a preserved diagnostic failure; this module
counts support records in the original mapper dimension and separately reports
physical payload/scalars. Original200000 capacity and5+1 witnesses stay fixed.
"""
from dataclasses import dataclass, field

from . import curve_consensus as arithmetic
from . import curve_evidence as evidence


REPRESENTATION = "finite_equal_frame_shared_evidence_creation_chart_v3"
_SEAL = object()


@dataclass(frozen=True)
class _PacketView:
    """Temporary complete gathers, released after unchanged chart derivation."""
    valid: object
    model_roots: object
    normals: object
    face_ids: object
    reliable: object


@dataclass(frozen=True)
class CurveState:
    owner_id: int
    signed_side: int
    signature: tuple
    origin: tuple
    tangent: tuple
    normal_cosine: float
    max_support_records: int
    source_frontier_ns: int
    frames: tuple
    pieces: tuple
    status: str
    reason: str
    _seal: object = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        if self._seal is not _SEAL:
            raise ValueError("use prepared compact curve factories")


def create_curve_state(owner_id, signed_side, signature, origin, tangent, *,
                       normal_cosine, max_support_records):
    owner, side, signature = arithmetic._key(owner_id, signed_side, signature)
    origin = arithmetic._vector(origin, "creation origin")
    tangent = arithmetic._vector(tangent, "creation tangent", unit=True)
    cosine = arithmetic._number(normal_cosine, "normal_cosine")
    if not 0. < cosine <= 1.:
        raise ValueError("normal_cosine must lie in (0,1]")
    capacity = arithmetic._integer(max_support_records, "max_support_records",
                                   minimum=20, maximum=200000)
    return CurveState(owner, side, signature, origin, tangent, cosine, capacity,
                      0, (), (), "empty", "", _SEAL)


def _chart_frame(state, packet):
    # Reuse the EXACT old gates/order/arithmetic. No rounded normal, new chart,
    # refit, speculative vectorized dot or cached coefficient recomputation.
    view = _PacketView(*(packet.gather_original(name) for name in
                        ("valid", "model_roots", "normals", "face_ids", "reliable")))
    derived = arithmetic._chart_frame(state, view)
    # The temporary gathers are NOT retained. All raw provenance resides in
    # the shared source and complete ordered pair/role references.
    return arithmetic._ChartFrame(packet, derived.status, derived.pieces)


def advance_curve(previous, current, *, available_before_ns):
    """Prepare without mutating old state or authenticating source transport."""
    if type(previous) is not CurveState or previous._seal is not _SEAL:
        raise ValueError("expected prepared compact state")
    if type(current) is not evidence.PairPacket or current._seal is not evidence._PACKET_SEAL:
        raise ValueError("expected complete shared original pair packet")
    available = arithmetic._integer(available_before_ns, "available_before_ns", minimum=1)
    if (current.owner_id, current.signed_side, current.signature) != (
            previous.owner_id, previous.signed_side, previous.signature):
        raise ValueError("owner/side/signature must not be merged")
    if not previous.source_frontier_ns < current.stamp_ns <= available:
        raise ValueError("duplicate/noncausal/future source frame")
    packets = tuple(frame.packet for frame in previous.frames) + (current,)
    keep = {packet.stamp_ns for packet in packets[-arithmetic.MINIMUM_FRAMES:]}
    reliable = tuple(packet for packet in packets if packet._has_reliable)
    keep.update(packet.stamp_ns for packet in reliable[-arithmetic.MINIMUM_RELIABLE_FRAMES:])
    retained = tuple(frame for frame in previous.frames if frame.packet.stamp_ns in keep)
    frames = retained + (_chart_frame(previous, current),)
    if len(frames) > arithmetic.MAX_RETAINED_FRAMES:
        raise ValueError("invalid bounded witness retention")
    pieces, status, reason = arithmetic._overlay(previous, frames)
    result = CurveState(previous.owner_id, previous.signed_side, previous.signature,
        previous.origin, previous.tangent, previous.normal_cosine,
        previous.max_support_records, current.stamp_ns, frames, pieces,
        status, reason, _SEAL)
    if inventory((result,))["support_records"] > previous.max_support_records:
        raise ValueError("complete compact support capacity exceeded")
    return result


def inventory(states):
    """Full staged-map inventory, not a query/window or per-owner credit.

    Charges all retained source tables, complete occurrences, state/key metadata,
    affine records/ordinals and exported pieces including contributor records.
    UNKNOWN and alternative signatures are not pruned. Shared source tables
    count once; every table pinned by even one old owner counts in full.
    Additional original map/continuation/permission capacity is caller-owned and
    must be added BEFORE publication. Payload bytes are not a heap/WCET proof.
    """
    distinct = {}
    for state in states:
        if type(state) is not CurveState or state._seal is not _SEAL:
            raise ValueError("expected complete prepared state inventory")
        key = (state.owner_id, state.signed_side, state.signature)
        if key in distinct:
            raise ValueError("duplicate owner/side/signature in staged map")
        distinct[key] = state
    frames = tuple(frame for state in distinct.values() for frame in state.frames)
    raw = evidence.reachable_inventory(frame.packet for frame in frames)
    affines = sum(len(frame.pieces) for frame in frames)
    pieces = tuple(piece for state in distinct.values() for piece in state.pieces)
    contributors = sum(len(piece.contributors) + len(piece.reliable_contributors)
                       for piece in pieces)
    # Conservative explicit metadata record charges. The original semantic
    # quad/support unit is not multiplied by its coordinate component count.
    records = (raw["support_records"] + 20 * len(distinct) + 4 * len(frames)
               + 2 * affines + 8 * len(pieces) + contributors)
    scalars = (raw["scalar_entries"] + 20 * len(distinct) + 4 * len(frames)
               + 9 * affines + sum(piece.storage_units for piece in pieces))
    result = dict(raw)
    result.update(states=len(distinct), retained_frames=len(frames),
                  affine_records=affines, exported_pieces=len(pieces),
                  contributor_records=contributors, support_records=records,
                  scalar_entries=scalars,
                  raw_payload_bytes=raw["payload_bytes"])
    # Canonical field payload only: affine4 binary64+3 int64+1 bool;
    # exported domain/coefficient/stamp/geometry13 eight-byte fields and ALL
    # contributor int64s. Python object/tuple/allocator overhead is NOT included
    # and must not be reported as measured heap or actual process memory.
    result["derived_canonical_payload_bytes"] = (57 * affines
        + 8 * (13 * len(pieces) + contributors))
    result["raw_plus_canonical_payload_bytes"] = (result["raw_payload_bytes"]
        + result["derived_canonical_payload_bytes"])
    return result
