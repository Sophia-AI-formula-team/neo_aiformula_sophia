"""Transactional v5 export of finite curves and reject-only source envelopes.

This module neither associates owners nor commits a map.  The caller supplies
the complete prospective map, including pending owners and every retained key.
Means are metadata only: no mean/chord geometry is made on a failed curve key.
The retained weighted development ledger is not a measured heap size and is
not yet proved equivalent to the original quad/Interval capacity dimension.
"""
from dataclasses import dataclass
from bisect import bisect_right
import math
from types import MappingProxyType

import numpy as np

from . import curve_consensus as arithmetic
from . import curve_consensus_compact as compact
from . import curve_evidence as evidence
from . import curve_evidence_blocks as blocks
from .continuation_support import common_intervals, FIELDS as CONTINUATION_FIELDS
from .curve_reference import CurveGeometry
from .finite_support import DomainState
from .curve_rounding import step as _step


@dataclass(frozen=True)
class BuiltCurveGeometry:
    geometry: CurveGeometry
    permissions: object
    inventory: object
    support_records: int


@dataclass(frozen=True)
class OrphanSourceFrame:
    """Original unassociated quad evidence, not a fabricated curve owner."""
    source: object
    has_reliable: bool

    def __post_init__(self):
        dense = (type(self.source) is evidence.SourceFrame
                 and self.source._seal is evidence._SOURCE_SEAL
                 and np.any(self.source.quad_valid & (self.source.endpoint_owners < 0)))
        retained = blocks.is_unowned_capsule(self.source) and bool(self.source.blocks)
        if type(self.has_reliable) is not bool or not (dense or retained):
            raise ValueError("expected original unowned valid-quad source")


def retain_orphan_sources(previous, current, *, available_before_ns):
    """Same latest-five plus latest reliable witness; no frame/source trimming."""
    frames = tuple(previous)
    if any(type(frame) is not OrphanSourceFrame for frame in frames):
        raise ValueError("invalid previous unowned source inventory")
    if type(current) is not OrphanSourceFrame:
        raise ValueError("invalid current unowned source")
    available = arithmetic._integer(available_before_ns, "available_before_ns", minimum=1)
    if (current.source.stamp_ns > available
            or (frames and current.source.stamp_ns <= frames[-1].source.stamp_ns)
            or any(b.source.stamp_ns <= a.source.stamp_ns for a, b in zip(frames, frames[1:]))):
        raise ValueError("duplicate/noncausal/future unowned source")
    frames += (current,)
    keep = {frame.source.stamp_ns for frame in frames[-5:]}
    reliable = tuple(frame for frame in frames if frame.has_reliable)
    keep.update(frame.source.stamp_ns for frame in reliable[-1:])
    retained = tuple(frame for frame in frames if frame.source.stamp_ns in keep)
    if len(retained) > 6:
        raise ValueError("invalid original 5/1 unowned source retention")
    return retained


def _bounded(lower, upper):
    if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
        raise ArithmeticError("unbounded curve competitor enclosure")
    return (lower, upper)


def _add(a, b):
    return _bounded(_step(a[0] + b[0], -np.inf), _step(a[1] + b[1], np.inf))


def _neg(a):
    return (-a[1], -a[0])


def _mul(a, b):
    values = tuple(x * y for x in a for y in b)
    if not all(math.isfinite(value) for value in values):
        raise ArithmeticError("unbounded curve competitor product")
    return _bounded(_step(min(values), -np.inf), _step(max(values), np.inf))


def _div(a, b):
    if b[0] <= 0. <= b[1]:
        raise ArithmeticError("uncertain curve competitor denominator")
    values = tuple(x / y for x in a for y in b)
    if not all(math.isfinite(value) for value in values):
        raise ArithmeticError("unbounded curve competitor quotient")
    return _bounded(_step(min(values), -np.inf), _step(max(values), np.inf))


def _projection(point, origin, vector):
    terms = [_mul(_add((float(x), float(x)), (-o, -o)), (t, t))
             for x, o, t in zip(point, origin, vector)]
    return _add(terms[0], terms[1])


def _world_envelope(state, span, intercept, slope):
    tangent = state.tangent
    normal = (-tangent[1], tangent[0])
    denominator = arithmetic._dot(tangent, tangent)
    if not math.isfinite(denominator) or denominator <= 0.:
        raise ArithmeticError("invalid original curve chart denominator")
    v = _add(intercept, _mul(slope, span))
    return tuple(_add((state.origin[i], state.origin[i]),
                      _div(_add(_mul((tangent[i], tangent[i]), span),
                                _mul((normal[i], normal[i]), v)),
                           (denominator, denominator))) for i in (0, 1))


def _unknown_affine(state, roots):
    """Bound original chart arithmetic; never turn it into nominal geometry."""
    normal = (-state.tangent[1], state.tangent[0])
    u = [_projection(point, state.origin, state.tangent) for point in roots]
    v = [_projection(point, state.origin, normal) for point in roots]
    slope = _div(_add(v[1], _neg(v[0])), _add(u[1], _neg(u[0])))
    intercept = _add(v[0], _neg(_mul(slope, u[0])))
    span = (min(u[0][0], u[1][0]), max(u[0][1], u[1][1]))
    return span, intercept, slope


def _own(packet, field, validity):
    if blocks.is_block_packet(packet):
        return packet.own_original(field, **{validity + "_only": True})
    source = packet.source
    return getattr(source, field)[(source.endpoint_owners == packet.owner_id)
                                  & getattr(source, validity)]


def _source(packet):
    return packet.lease.unowned if blocks.is_block_packet(packet) else packet.source


def _source_key(source):
    return id(source.frame) if blocks.is_unowned_capsule(source) else id(source)


def _unowned_quads(source):
    if blocks.is_unowned_capsule(source):
        return source.gather_original("quads")
    return source.quads[source.quad_valid & (source.endpoint_owners < 0)]


def _state_box(state):
    """ALL owned faces, quads and both incident endpoints, with IEEE bounds.

    A source-root hull alone is not a proof for rounded affine coefficients.
    Bound each computed/unknown pair and every possible coefficient average.
    A nonfunctional/unbounded chart returns no proven finite envelope.
    """
    lower = np.full(2, np.inf)
    upper = np.full(2, -np.inf)
    affines = []
    def include(values):
        nonlocal lower, upper
        data = np.asarray(values, dtype=np.float64).reshape(-1, 2)
        if len(data):
            if not np.isfinite(data).all():
                raise ArithmeticError("invalid required source competitor coordinates")
            lower = np.minimum(lower, np.min(data, axis=0))
            upper = np.maximum(upper, np.max(data, axis=0))
    for frame in state.frames:
        packet = frame.packet
        include(_own(packet, "model_roots", "valid"))
        include(_own(packet, "quads", "quad_valid"))
        # Do not drop the opposite owner of an incident endpoint occurrence.
        pair_roots = packet.gather_original("model_roots")
        pair_valid = packet.gather_original("valid")
        include(pair_roots[pair_valid])
        if frame.status == "known":
            for piece in frame.pieces:
                affines.append(((piece.lower, piece.upper),
                                (piece.intercept, piece.intercept),
                                (piece.slope, piece.slope)))
        else:
            for roots, valid in zip(pair_roots, pair_valid):
                if not bool(np.all(valid)):
                    raise ArithmeticError("invalid required incident source roots")
                affines.append(_unknown_affine(state, roots))
    for span, intercept, slope in affines:
        envelope = _world_envelope(state, span, intercept, slope)
        include(((envelope[0][0], envelope[1][0]),
                 (envelope[0][1], envelope[1][1])))
    if affines:
        # Any active equal-frame subset has its coefficient mean within these
        # ranges. Two outward ULPs cover fsum and the following division; the
        # subsequent affine/world operations are separately enclosed above.
        span = (min(item[0][0] for item in affines), max(item[0][1] for item in affines))
        intercept = (min(item[1][0] for item in affines), max(item[1][1] for item in affines))
        slope = (min(item[2][0] for item in affines), max(item[2][1] for item in affines))
        for _ in range(2):
            intercept = (_step(intercept[0], -np.inf), _step(intercept[1], np.inf))
            slope = (_step(slope[0], -np.inf), _step(slope[1], np.inf))
        envelope = _world_envelope(state, span, intercept, slope)
        include(((envelope[0][0], envelope[1][0]),
                 (envelope[0][1], envelope[1][1])))
    if not np.isfinite(lower).all() or not np.isfinite(upper).all():
        raise ArithmeticError("missing finite competitor source inventory")
    lower, upper = np.nextafter(lower, -np.inf), np.nextafter(upper, np.inf)
    if (not np.isfinite(lower).all() or not np.isfinite(upper).all()
            or np.any(np.abs(lower) > 1.e7) or np.any(np.abs(upper) > 1.e7)):
        raise ArithmeticError("unrepresentable finite competitor box")
    return (tuple(lower), tuple(upper))


def _unowned_quad_box(quad):
    """Enclose one original finite world footprint, without a curve chart.

    This box describes only the convex footprint of this unassociated face.
    It cannot replace an owned key's root/incident/affine model enclosure.
    Outward subnormal steps remain representable even under a strict caller
    FP policy; the local policy is restored before returning or rejecting.
    """
    data = np.asarray(quad, dtype=np.float64)
    if data.shape != (4, 2) or not np.isfinite(data).all():
        raise ArithmeticError("invalid required unowned quad coordinates")
    with np.errstate(all="ignore"):
        lower = np.nextafter(np.min(data, axis=0), -np.inf)
        upper = np.nextafter(np.max(data, axis=0), np.inf)
    if (not np.isfinite(lower).all() or not np.isfinite(upper).all()
            or np.any(np.abs(lower) > 1.e7) or np.any(np.abs(upper) > 1.e7)):
        raise ArithmeticError("unrepresentable finite unowned quad box")
    return tuple(lower), tuple(upper)


def _union_spans(spans):
    output = []
    for lower, upper in sorted(spans):
        if not lower < upper:
            continue
        if output and lower <= output[-1][1]:
            output[-1] = (output[-1][0], max(output[-1][1], upper))
        else:
            output.append((lower, upper))
    return tuple(output)


def _remaining(span, covered):
    """Conservative closed complement; never bridge a finite support hole."""
    lower, upper = span
    result, cursor = [], lower
    for start, stop in covered:
        if stop <= cursor or start >= upper:
            continue
        if cursor < start:
            result.append((cursor, min(start, upper)))
        cursor = max(cursor, stop)
        if cursor >= upper:
            break
    if cursor < upper:
        result.append((cursor, upper))
    return tuple(result)


def _envelope_box(envelope):
    lower = tuple(_step(value[0], -np.inf) for value in envelope)
    upper = tuple(_step(value[1], np.inf) for value in envelope)
    if not all(math.isfinite(value) and abs(value) <= 1.e7 for value in lower+upper):
        raise ArithmeticError("unrepresentable finite source-complement box")
    return lower, upper


def _remaining_boxes(state, covered):
    """Keep ALL pending spans of a partly ready key as local competitors.

    `_overlay` may expose only a subset of its source events. Source quads are
    clipped by an outward chart slab, not dropped because any piece is ready.
    Root/quad tables are scanned directly, including faces absent from pairs.
    """
    covered = _union_spans(covered)
    boxes = {}
    stops = tuple(stop for _, stop in covered)
    def add_box(pending, envelope):
        gap = bisect_right(stops, pending[0])
        lower, upper = _envelope_box(envelope)
        old = boxes.get(gap)
        if old is not None:
            lower = tuple(min(a, b) for a, b in zip(old[0], lower))
            upper = tuple(max(a, b) for a, b in zip(old[1], upper))
        boxes[gap] = (lower, upper)
    tangent, origin = state.tangent, state.origin
    normal = (-tangent[1], tangent[0])
    # Unlike the model's rounded denominator, this interval also encloses the
    # exact inverse chart of original quad/root coordinates.
    denominator = _add(_mul((tangent[0], tangent[0]), (tangent[0], tangent[0])),
                       _mul((tangent[1], tangent[1]), (tangent[1], tangent[1])))
    all_affines = tuple(affine for frame in state.frames for affine in frame.pieces)
    if all_affines:
        mean_intercept = (min(item.intercept for item in all_affines),
                          max(item.intercept for item in all_affines))
        mean_slope = (min(item.slope for item in all_affines),
                      max(item.slope for item in all_affines))
        for _ in range(2):
            mean_intercept = (_step(mean_intercept[0], -np.inf), _step(mean_intercept[1], np.inf))
            mean_slope = (_step(mean_slope[0], -np.inf), _step(mean_slope[1], np.inf))
    def source_boxes(points):
        data = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        if not len(data):
            return
        if not np.isfinite(data).all():
            raise ArithmeticError("invalid required remaining source coordinates")
        u = tuple(_projection(point, origin, tangent) for point in data)
        v = tuple(_projection(point, origin, normal) for point in data)
        span = (min(value[0] for value in u), max(value[1] for value in u))
        lateral = (min(value[0] for value in v), max(value[1] for value in v))
        for pending in _remaining(span, covered):
            envelope = tuple(_add((origin[i], origin[i]),
                _div(_add(_mul((tangent[i], tangent[i]), pending),
                          _mul((normal[i], normal[i]), lateral)), denominator)) for i in (0, 1))
            add_box(pending, envelope)
    for frame in state.frames:
        if frame.status != "known":
            raise ArithmeticError("partly ready key has unknown source chart")
        packet = frame.packet
        # A point is a separate original face witness, not a gathered hull of
        # disconnected faces. Incident endpoint multiplicity is kept too.
        for point in _own(packet, "model_roots", "valid"):
            source_boxes((point,))
        for quad in _own(packet, "quads", "quad_valid"):
            source_boxes(quad)
        roots = packet.gather_original("model_roots").reshape(-1, 2)
        valid = packet.gather_original("valid").reshape(-1)
        for point, admitted in zip(roots, valid):
            if not admitted:
                raise ArithmeticError("invalid required remaining incident source")
            source_boxes((point,))
        for affine in frame.pieces:
            for pending in _remaining((affine.lower, affine.upper), covered):
                add_box(pending, _world_envelope(state, pending, mean_intercept, mean_slope))
    # One conservative union per connected complement component, never one
    # hull spanning distinct ready islands. Raw rows/roles/multiplicity remain
    # completely stored in the key's immutable source evidence.
    return tuple(boxes[gap] for gap in sorted(boxes))


def _finite_piece(state, piece, interval, cosine):
    lower, upper = max(piece.lower, interval.lower), min(piece.upper, interval.upper)
    if not lower < upper:
        return None
    first, first_bounds = arithmetic._endpoint(state, lower, upper,
        piece.intercept, piece.slope, first=True)
    second, second_bounds = arithmetic._endpoint(state, lower, upper,
        piece.intercept, piece.slope, first=False)
    if not first_bounds[1] < second_bounds[0]:
        raise ArithmeticError("rounded finite curve domain collapsed")
    delta = tuple(b-a for a, b in zip(first, second))
    normal = arithmetic._normal(delta)
    chart_normal = (-state.tangent[1], state.tangent[0])
    if arithmetic._dot(normal, chart_normal) < cosine:
        raise ArithmeticError("rounded finite curve signed chart direction")
    active = 0
    for frame in state.frames:
        if frame.packet.stamp_ns not in piece.contributors:
            continue
        candidates = [item for item in frame.pieces
                      if item.lower <= piece.lower and item.upper >= piece.upper]
        if len(candidates) != 1:
            raise ArithmeticError("incomplete original curve contributor occurrence")
        original = candidates[0]
        packet = frame.packet
        source_normals = (packet.occurrence_normals(original.occurrence)
            if blocks.is_block_packet(packet) else
            packet.source.normals[packet.face_indices[original.occurrence]])
        if any(arithmetic._dot(normal, value) < cosine for value in source_normals):
            raise ArithmeticError("rounded finite curve original support direction")
        active += 1
    if active != len(piece.contributors) or active < 5 or not piece.reliable_contributors:
        raise ArithmeticError("incomplete finite curve 5/1 evidence")
    return ((first, second), normal, min(piece.frame_votes, interval.frame_votes),
            min(piece.reliable_votes, interval.reliable_votes),
            max(piece.source_stamp_ns, interval.last_stamp_ns),
            (first_bounds[1], second_bounds[0]))


def _permission_arrays(records, edges, continuations, config, geometry):
    by_id = {record.record_id: record for record in records}
    incoming, outgoing = {}, {}
    def confirmed(record):
        return (record.votes >= config["minimum_frame_votes"]
                and record.reliable_votes >= config["minimum_reliable_votes"])
    for edge in edges.values():
        if (edge.source_id not in by_id or edge.target_id not in by_id):
            raise ValueError("incomplete global incident owner inventory")
        if confirmed(edge) and confirmed(by_id[edge.source_id]) and confirmed(by_id[edge.target_id]):
            outgoing.setdefault(edge.source_id, set()).add(edge.target_id)
            incoming.setdefault(edge.target_id, set()).add(edge.source_id)
    spans, primitives, stamps = [], [], []
    cosine = math.cos(config["normal_max_angle_rad"])
    for index, (segment, normal, slot) in enumerate(zip(geometry.segments,
                                                     geometry.normals, geometry.owner)):
        owner = records[int(slot)].record_id
        before, after = incoming.get(owner, set()), outgoing.get(owner, set())
        if len(before) <= 1 and len(after) <= 1:
            continue
        hypotheses = continuations.get(owner, {})
        if (not hypotheses or not before.issubset({key[0] for key in hypotheses})
                or not after.issubset({key[1] for key in hypotheses})):
            continue
        permitted = common_intervals(segment, normal, hypotheses,
            minimum_frames=config["minimum_frame_votes"],
            minimum_reliable=config["minimum_reliable_votes"], cosine=cosine)
        if permitted:
            spans.extend(permitted)
            primitives.extend([index]*len(permitted))
            frontier = max(frame.stamp_ns for frames in hypotheses.values() for frame in frames)
            stamps.extend([frontier]*len(permitted))
    count = len(spans)
    values = (np.asarray(spans, dtype=np.float64).reshape(-1, 2),
              np.asarray(primitives, dtype=np.int64),
              np.full(count, config["minimum_frame_votes"], dtype=np.int64),
              np.full(count, config["minimum_reliable_votes"], dtype=np.int64),
              np.asarray(stamps, dtype=np.int64))
    return MappingProxyType({name: np.frombuffer(value.tobytes(), dtype=value.dtype).reshape(value.shape)
                             for name, value in zip(CONTINUATION_FIELDS, values)})


def build_curve_geometry(records, domains, states, edges, continuations, config,
                         source_frontier_ns, *, base_support_records, orphan_sources=()):
    """Prepare a COMPLETE global bundle or reject before the caller commits.

    records must be stable record-ID order and include pending owners. All
    keys must be supplied; the builder never selects a convenient signature.
    No spatial crop or hidden evidence eviction is a capacity remedy.
    """
    records = tuple(records)
    if (tuple(record.record_id for record in records) != tuple(sorted(
            {record.record_id for record in records}))
            or config["minimum_frame_votes"] != 5 or config["minimum_reliable_votes"] != 1):
        raise ValueError("v5 requires complete ordered owners and original 5/1")
    source_frontier_ns = arithmetic._integer(source_frontier_ns, "source_frontier_ns", minimum=1)
    base_support_records = arithmetic._integer(base_support_records, "base_support_records", minimum=0)
    orphan_sources = tuple(orphan_sources)
    if (len(orphan_sources) > 6 or any(type(frame) is not OrphanSourceFrame for frame in orphan_sources)
            or any(frame.source.stamp_ns > source_frontier_ns for frame in orphan_sources)
            or any(b.source.stamp_ns <= a.source.stamp_ns
                   for a, b in zip(orphan_sources, orphan_sources[1:]))):
        raise ValueError("invalid complete retained unowned source inventory")
    packets = tuple(frame.packet for state in states.values() for frame in state.frames)
    block_mode = (any(blocks.is_block_packet(packet) for packet in packets)
                  or any(blocks.is_unowned_capsule(frame.source) for frame in orphan_sources))
    if block_mode:
        if (any(not blocks.is_block_packet(packet) for packet in packets)
                or any(not blocks.is_unowned_capsule(frame.source) for frame in orphan_sources)):
            raise ValueError("cannot mix dense and retained-block source inventories")
        inventory = compact.inventory(states.values(),
            orphan_capsules=tuple(frame.source for frame in orphan_sources))
    else:
        inventory = compact.inventory(states.values())
    slots = {record.record_id: index for index, record in enumerate(records)}
    if any(key != (state.owner_id, state.signed_side, state.signature)
           or state.owner_id not in slots for key, state in states.items()):
        raise ValueError("staged curve keys/owner inventory differ")
    grouped = {owner: [] for owner in slots}
    for key in sorted(states):
        state = states[key]
        if (state.source_frontier_ns > source_frontier_ns
                or any(frame.packet.stamp_ns > source_frontier_ns for frame in state.frames)):
            raise ValueError("future curve source evidence")
        grouped[state.owner_id].append(state)
    segments, normals, owners, votes, reliable, stamps = [], [], [], [], [], []
    boxes, box_owners, status = [], [], np.zeros(len(records), dtype=np.int64)
    complete, reasons = True, {}
    partial_keys, remaining_boxes, competing_keys = 0, 0, 0
    cosine = math.cos(config["normal_max_angle_rad"])
    sources = {_source_key(_source(packet)): _source(packet) for packet in packets}
    extra_sources = {_source_key(frame.source): frame.source for frame in orphan_sources
                     if _source_key(frame.source) not in sources}
    sources.update(extra_sources)
    # Count physical source tables once globally, but ALL independent retained
    # orphan reference/ordinal/reliability/header records remain charged.
    extra_records = 0 if block_mode else sum(source.support_records for source in extra_sources.values())
    extra_scalars = 0 if block_mode else sum(source.scalar_entries for source in extra_sources.values())
    extra_payload = 0 if block_mode else sum(source.payload_bytes for source in extra_sources.values())
    if not block_mode:
        inventory["source_tables"] += len(extra_sources)
        inventory["source_faces"] += sum(len(source.face_ids) for source in extra_sources.values())
    inventory["support_records"] += extra_records + 4*len(orphan_sources)
    inventory["scalar_entries"] += extra_scalars + 4*len(orphan_sources)
    inventory["payload_bytes"] += extra_payload
    inventory["raw_payload_bytes"] += extra_payload
    inventory["raw_plus_canonical_payload_bytes"] += extra_payload
    # Every reachable table is scanned once, including tables pinned only by
    # an old state. Each unowned finite quad is a separate local UNKNOWN
    # competitor, never a fabricated node/normal/curve or nearby-owner match.
    unowned, standalone_boxes = 0, 0
    for source in sources.values():
        for quad in _unowned_quads(source):
            unowned += 1
            try:
                boxes.append(_unowned_quad_box(quad))
                box_owners.append(-1)
                standalone_boxes += 1
            except ArithmeticError as error:
                complete = False
                reason = str(error)
                reasons[reason] = reasons.get(reason, 0)+1
    for record in records:
        owner, slot = record.record_id, slots[record.record_id]
        keys = grouped[owner]
        if len(keys) > 1:
            competing_keys += len(keys)
        domain = domains.get(owner)
        confirmed_owner = record.votes >= 5 and record.reliable_votes >= 1
        owner_status = 2 if keys and confirmed_owner else 1
        if type(domain) is not DomainState or not keys:
            owner_status, complete = 0, False
            reasons["missing_owner_domain_or_keys"] = reasons.get("missing_owner_domain_or_keys", 0)+1
        for state in keys:
            if type(domain) is DomainState and (np.asarray((state.origin, state.tangent), dtype=np.float64).tobytes()
                    != np.asarray((domain.origin, domain.tangent), dtype=np.float64).tobytes()
                                      or state.normal_cosine != cosine):
                raise ValueError("curve/finite domain original creation chart differs")
            local, rejected = [], False
            if state.status == "confirmed" and confirmed_owner and type(domain) is DomainState:
                try:
                    for piece in state.pieces:
                        for interval in domain.intervals:
                            if interval.frame_votes < 5 or interval.reliable_votes < 1:
                                continue
                            exported = _finite_piece(state, piece, interval, cosine)
                            if exported is not None:
                                local.append(exported)
                except ArithmeticError:
                    # Reject the WHOLE key, not just the inconvenient fragment.
                    local, rejected = [], True
            if local:
                for segment, normal, frame_votes, reliable_votes, last, _ in local:
                    segments.append(segment); normals.append(normal); owners.append(slot)
                    votes.append(frame_votes); reliable.append(reliable_votes); stamps.append(last)
            pending = state.status != "confirmed" or not confirmed_owner or not local
            if state.status in ("unknown", "empty") or rejected:
                owner_status = 0
            elif pending and owner_status != 0:
                owner_status = 1
            # Distinct source signatures are alternatives, not duplicate world
            # coordinates. Even two individually ready alternatives retain
            # their reject-only competition instead of selecting one key.
            if pending or rejected or len(keys) > 1:
                try:
                    boxes.append(_state_box(state)); box_owners.append(slot)
                except ArithmeticError as error:
                    complete = False
                    reason = str(error)
                    reasons[reason] = reasons.get(reason, 0)+1
            else:
                try:
                    remaining = _remaining_boxes(state, (item[-1] for item in local))
                    boxes.extend(remaining)
                    box_owners.extend([slot]*len(remaining))
                    if remaining:
                        partial_keys += 1
                        remaining_boxes += len(remaining)
                        if owner_status != 0:
                            owner_status = 1
                except ArithmeticError as error:
                    complete = False
                    reason = str(error)
                    reasons[reason] = reasons.get(reason, 0)+1
        status[slot] = owner_status
    geometry = CurveGeometry(np.asarray(segments, dtype=np.float64).reshape(-1, 2, 2),
        np.asarray(normals, dtype=np.float64).reshape(-1, 2), np.asarray(owners, dtype=np.int64),
        np.asarray(votes, dtype=np.int64), np.asarray(reliable, dtype=np.int64),
        np.asarray(stamps, dtype=np.int64), status,
        np.asarray(boxes, dtype=np.float64).reshape(-1, 2, 2),
        np.asarray(box_owners, dtype=np.int64), complete)
    permissions = _permission_arrays(records, edges, continuations, config, geometry)
    # Explicit semantic record ledger: one geometry header, ALL owner status
    # entries, nominal segment records, reject boxes and permission records.
    export_records = 1 + len(records)+len(segments)+len(boxes)+len(permissions["continuation_primitive"])
    total = base_support_records + inventory["support_records"] + export_records
    if total > config["max_support_segments"]:
        raise ValueError("combined v5 shared source/export/permission capacity")
    metrics = dict(inventory)
    metrics.update(base_support_records=base_support_records,
        export_support_records=export_records, total_support_records=total,
        nominal_segments=len(segments), blocker_boxes=len(boxes),
        continuation_permissions=len(permissions["continuation_primitive"]),
        geometry_complete=complete, incomplete_reasons=MappingProxyType(reasons),
        unowned_valid_quads=unowned, standalone_unowned_boxes=standalone_boxes,
        retained_orphan_sources=len(orphan_sources),
        partially_ready_keys=partial_keys, pending_complement_boxes=remaining_boxes,
        competing_signature_keys=competing_keys)
    return BuiltCurveGeometry(geometry, permissions, MappingProxyType(metrics), total)


# Capture the original implementation and helper identities once during module
# initialization. Batch scratch work never persists in the map or changes its
# capacity ledger. Unsupported arithmetic/readers fall back to this exact scalar
# helper once; v4 never imports this opt-in curve builder.
_remaining_boxes_scalar = _remaining_boxes
from . import curve_envelope_batch as _envelope_batch
import sys as _sys
_ENVELOPE_BINDINGS = _envelope_batch.bind_scalar_helpers(_sys.modules[__name__])


def _remaining_boxes(state, covered):
    return _envelope_batch.dispatch_remaining_boxes(
        state, covered, bindings=_ENVELOPE_BINDINGS)
