"""Opt-in, lossless batching of v5 reject-only complement envelopes.

No hook is installed on import. A declined call returns None and its caller
must run the bound scalar helper to retain its first warning/exception.
Scratch arrays and optional diagnostics belong to one call; source evidence,
occurrences, gap order, confirmations and capacity gates are unchanged.
"""
from bisect import bisect_right
from dataclasses import dataclass
import math

import numpy as np

from . import curve_consensus as arithmetic
from . import curve_consensus_compact as compact
from . import curve_evidence as evidence
from . import curve_evidence_blocks as blocks
from . import curve_rounding as rounding


_DEFAULT_FP = {"divide": "warn", "over": "warn", "under": "ignore", "invalid": "warn"}


@dataclass(frozen=True)
class ScalarBindings:
    """Immutable helper baseline, prepared before installing a dispatch hook."""
    scalar: object
    scalar_remaining_boxes: object
    entries: tuple


def _entry(owner, name):
    value = getattr(owner, name)
    return (owner, name, value, getattr(value, "__code__", None),
            getattr(value, "__defaults__", None),
            tuple(sorted((getattr(value, "__kwdefaults__", None) or {}).items())))


def bind_scalar_helpers(scalar=None):
    """Capture current scalar readers; does not modify any module binding.

    Integrators bind once after defining the scalar helper and before exposing
    the wrapper. Rebinding after a monkeypatch intentionally defines a new
    baseline and is not a way to approve altered arithmetic.
    """
    if scalar is None:
        from . import curve_geometry_builder as scalar
    names = ("_union_spans", "_remaining", "_envelope_box", "_projection",
             "_world_envelope", "_own", "_add", "_neg", "_mul", "_div",
             "_bounded", "_step", "bisect_right", "np", "math", "arithmetic", "blocks")
    entries = [_entry(scalar, name) for name in names]
    entries += [_entry(arithmetic, name) for name in ("_dot", "_ChartFrame", "_Affine")]
    entries += [_entry(compact, name) for name in ("CurveState", "_SEAL")]
    entries += [_entry(evidence, name) for name in
                ("SourceFrame", "PairPacket", "_SOURCE_SEAL", "_PACKET_SEAL", "_FIELDS", "np")]
    entries += [_entry(np, name) for name in (
        "geterr", "geterrcall", "errstate", "nextafter", "add", "multiply",
        "divide", "where", "isfinite", "asarray", "concatenate", "empty",
        "stack", "flatnonzero", "column_stack", "any", "all", "abs", "arange",
        "dtype", "ndarray", "float64", "int64", "uint8", "bool_", "inf", "frombuffer")]
    entries += [_entry(math, name) for name in ("isfinite",)]
    entries += [_entry(rounding, name) for name in (
        "_NUMPY_NEXTAFTER", "_NUMPY_FLOAT64", "_MATH_NEXTAFTER",
        "_MINIMUM", "_MAXIMUM", "np", "math")]
    entries += [_entry(blocks, name) for name in
                ("is_block_packet", "_decode", "_layout", "_prepared", "_SEAL",
                 "BlockPacket", "KeyFrameLease", "FrameIdentity", "FaceBlock", "_LAYOUT", "np")]
    entries += [_entry(blocks.BlockPacket, name) for name in
                ("gather_original", "own_original")]
    entries += [_entry(blocks.FaceBlock, "get")]
    entries += [_entry(evidence.PairPacket, "gather_original")]
    helper_name = "_remaining_boxes_scalar" if hasattr(scalar, "_remaining_boxes_scalar") else "_remaining_boxes"
    entries.append(_entry(scalar, helper_name))
    helper = getattr(scalar, helper_name)
    return ScalarBindings(scalar, helper, tuple(entries))


_DOT = arithmetic._dot


def _default_bindings():
    # The builder captures this baseline during its initialization, not on the
    # first candidate call. Importing this module first cannot rebind a patched
    # helper or create a partially initialized builder cycle.
    from . import curve_geometry_builder
    return curve_geometry_builder._ENVELOPE_BINDINGS


def new_diagnostics():
    return {"calls": 0, "hits": 0, "fallbacks": 0, "fallback_reasons": {},
            "source_events": 0, "source_points": 0, "incident_occurrences": 0,
            "affine_events": 0, "pending_rows": 0, "source_pending_rows": 0,
            "affine_pending_rows": 0, "output_boxes": 0}


def _unchanged(bindings):
    if type(bindings) is not ScalarBindings:
        return False
    for owner, name, value, code, defaults, kwdefaults in bindings.entries:
        current = getattr(owner, name, None)
        if (current is not value or getattr(current, "__code__", None) is not code
                or getattr(current, "__defaults__", None) is not defaults
                or tuple(sorted((getattr(current, "__kwdefaults__", None) or {}).items()))
                   != kwdefaults):
            return False
    return True


def _binary64_environment():
    """Decline non-nearest arithmetic or flushed subnormals before batching."""
    one, half = 1., float.fromhex("0x1p-53")
    successor = float.fromhex("0x1.0000000000001p+0")
    second = float.fromhex("0x1.0000000000002p+0")
    tiny = float.fromhex("0x0.0000000000001p-1022")
    return (float(np.add(one, half)) == one
            and float(np.add(successor, half)) == second
            and float(np.add(-one, -half)) == -one
            and float(np.multiply(tiny, one)).hex() == tiny.hex()
            and float(np.multiply(-tiny, one)).hex() == (-tiny).hex()
            and float(np.nextafter(0., np.inf)).hex() == tiny.hex())


class _Decline(Exception):
    pass


def _require(condition, reason):
    if not condition:
        raise _Decline(reason)


def _interval(value):
    """Check an already computed binary64 interval without widening it."""
    _require(bool(np.all(np.isfinite(value))), "nonfinite_interval")
    _require(bool(np.all(value[..., 0] <= value[..., 1])), "reversed_interval")
    return value


def _add(first, second):
    # Scalar order is lower add -> step -> upper add -> step. Batching keeps
    # each row's numeric operations, not the ordering of raw FP sticky flags;
    # unsupported policies/numerics decline to the original scalar helper.
    lower = np.add(first[..., 0], second[..., 0])
    upper = np.add(first[..., 1], second[..., 1])
    return _interval(np.stack((np.nextafter(lower, -np.inf),
                               np.nextafter(upper, np.inf)), axis=-1))


def _ordered_extrema(values):
    """Python min/max keep the FIRST equal value, including signed zero."""
    lower, upper = values[0], values[0]
    for value in values[1:]:
        lower = np.where(value < lower, value, lower)
        upper = np.where(value > upper, value, upper)
    return lower, upper


def _mul(first, second):
    # Four separate binary64 products, in the scalar nested-loop order.
    products = (np.multiply(first[..., 0], second[..., 0]),
                np.multiply(first[..., 0], second[..., 1]),
                np.multiply(first[..., 1], second[..., 0]),
                np.multiply(first[..., 1], second[..., 1]))
    _require(all(bool(np.all(np.isfinite(value))) for value in products), "nonfinite_product")
    lower, upper = _ordered_extrema(products)
    return _interval(np.stack((np.nextafter(lower, -np.inf),
                               np.nextafter(upper, np.inf)), axis=-1))


def _div(first, second):
    _require(not bool(np.any((second[..., 0] <= 0.) & (second[..., 1] >= 0.))),
             "uncertain_denominator")
    quotients = (np.divide(first[..., 0], second[..., 0]),
                 np.divide(first[..., 0], second[..., 1]),
                 np.divide(first[..., 1], second[..., 0]),
                 np.divide(first[..., 1], second[..., 1]))
    _require(all(bool(np.all(np.isfinite(value))) for value in quotients), "nonfinite_quotient")
    lower, upper = _ordered_extrema(quotients)
    return _interval(np.stack((np.nextafter(lower, -np.inf),
                               np.nextafter(upper, np.inf)), axis=-1))


def _point_interval(value):
    return np.asarray((value, value), dtype=np.float64)


def _project_points(points, origin, vector):
    """Exact operation graph of scalar._projection; no dot or subtraction rewrite."""
    terms = []
    for axis in (0, 1):
        coordinate = np.column_stack((points[:, axis], points[:, axis]))
        shifted = _add(coordinate, _point_interval(-origin[axis]))
        terms.append(_mul(shifted, _point_interval(vector[axis])))
    return _add(terms[0], terms[1])


def _source_spans(points, starts, lengths, origin, tangent):
    normal = (-tangent[1], tangent[0])
    u = _project_points(points, origin, tangent)
    v = _project_points(points, origin, normal)
    spans, lateral = u[starts].copy(), v[starts].copy()
    # Each entity is one original root or one original four-vertex quad.
    # Preserve vertex order rather than use an implementation-dependent reduce.
    for vertex in (1, 2, 3):
        selected = np.flatnonzero(lengths > vertex)
        if not len(selected):
            continue
        rows = starts[selected] + vertex
        for target, projected in ((spans, u), (lateral, v)):
            old = target[selected]
            more = projected[rows]
            target[selected, 0] = np.where(more[:, 0] < old[:, 0], more[:, 0], old[:, 0])
            target[selected, 1] = np.where(more[:, 1] > old[:, 1], more[:, 1], old[:, 1])
    return spans, lateral


def _world(points, lateral, origin, tangent, denominator):
    normal = (-tangent[1], tangent[0])
    coordinates = []
    for axis in (0, 1):
        numerator = _add(_mul(_point_interval(tangent[axis]), points),
                         _mul(_point_interval(normal[axis]), lateral))
        coordinates.append(_add(_point_interval(origin[axis]), _div(numerator, denominator)))
    return np.stack(coordinates, axis=1)


def _source_structure(source):
    _require(type(source) is evidence.SourceFrame and source._seal is evidence._SOURCE_SEAL,
             "unsealed_source")
    count = len(source.face_ids)
    for name, dtype, tail in evidence._FIELDS:
        value = getattr(source, name)
        _require(type(value) is np.ndarray and value.dtype == np.dtype(dtype)
                 and value.dtype.isnative and value.shape == (count,) + tail
                 and value.flags.c_contiguous and not value.flags.writeable,
                 "unsupported_source_array")


def _candidate(state, covered, chunk_size, scalar):
    _require(type(state) is compact.CurveState and state._seal is compact._SEAL,
             "unsealed_state")
    _require(type(covered) is tuple and all(type(span) is tuple and len(span) == 2
             and all(type(value) is float and math.isfinite(value) for value in span)
             for span in covered), "unsupported_covered")
    _require(type(chunk_size) is int and 1 <= chunk_size <= 65536, "unsupported_chunk_size")
    _require(type(state.origin) is tuple and type(state.tangent) is tuple
             and len(state.origin) == 2 and len(state.tangent) == 2
             and all(type(value) is float and math.isfinite(value)
                     for value in state.origin + state.tangent), "unsupported_chart")
    _require(type(state.frames) is tuple, "unsupported_frames")
    # Do not touch .pieces on an unsupported frame (its property may have
    # observable effects). A decline leaves the scalar as the first reader.
    _require(all(type(frame) is arithmetic._ChartFrame for frame in state.frames),
             "unsupported_frame_type")
    _require(all(type(frame.pieces) is tuple for frame in state.frames),
             "unsupported_affine_inventory")
    covered = scalar._union_spans(covered)
    stops = tuple(stop for _, stop in covered)
    tangent, origin = state.tangent, state.origin
    normal = (-tangent[1], tangent[0])
    # This is the interval inverse-chart denominator, not the rounded dot below.
    denominator = np.asarray(scalar._add(
        scalar._mul((tangent[0], tangent[0]), (tangent[0], tangent[0])),
        scalar._mul((tangent[1], tangent[1]), (tangent[1], tangent[1]))), dtype=np.float64)
    all_affines = tuple(affine for frame in state.frames for affine in frame.pieces)
    mean_intercept = mean_slope = None
    if all_affines:
        _require(all(type(item) is arithmetic._Affine and all(
            type(value) is float and math.isfinite(value)
            for value in (item.lower, item.upper, item.intercept, item.slope))
            for item in all_affines), "unsupported_affine")
        mean_intercept = (min(item.intercept for item in all_affines),
                          max(item.intercept for item in all_affines))
        mean_slope = (min(item.slope for item in all_affines),
                      max(item.slope for item in all_affines))
        for _ in range(2):
            mean_intercept = (scalar._step(mean_intercept[0], -np.inf),
                              scalar._step(mean_intercept[1], np.inf))
            mean_slope = (scalar._step(mean_slope[0], -np.inf),
                          scalar._step(mean_slope[1], np.inf))
    parts, starts, lengths, events = [], [], [], []
    point_count, incident_count = 0, 0
    checked_sources = set()

    def add_sources(values, vertices):
        nonlocal point_count
        _require(type(values) is np.ndarray and values.dtype == np.dtype(np.float64)
                 and values.dtype.isnative, "unsupported_coordinate_array")
        data = values.reshape(-1, vertices, 2)
        if not len(data):
            return
        _require(bool(np.all(np.isfinite(data))), "nonfinite_source")
        parts.append(data.reshape(-1, 2))
        for row in range(len(data)):
            descriptor = len(starts)
            starts.append(point_count + row * vertices)
            lengths.append(vertices)
            events.append((0, descriptor))
        point_count += len(data) * vertices

    for frame in state.frames:
        _require(type(frame) is arithmetic._ChartFrame and frame.status == "known",
                 "unknown_source_chart")
        packet = frame.packet
        dense = type(packet) is evidence.PairPacket and packet._seal is evidence._PACKET_SEAL
        retained = blocks.is_block_packet(packet)
        _require(dense or retained, "unsealed_packet")
        if dense:
            source = packet.source
            if id(source) not in checked_sources:
                _source_structure(source)
                checked_sources.add(id(source))
        else:
            _require(type(packet.lease) is blocks.KeyFrameLease
                     and packet.lease._seal is blocks._SEAL
                     and type(packet.lease.frame) is blocks.FrameIdentity
                     and packet.lease.frame._seal is blocks._SEAL,
                     "unsealed_lease")
        _require(type(packet.face_indices) is np.ndarray
                 and packet.face_indices.dtype == np.dtype(np.int64)
                 and packet.face_indices.dtype.isnative
                 and packet.face_indices.ndim == 2 and packet.face_indices.shape[1] == 2
                 and packet.face_indices.flags.c_contiguous
                 and not packet.face_indices.flags.writeable, "unsupported_pair_indices")
        # Same original own-root/quad selection, including faces absent from
        # every pair. Retained storage decodes complete original row bytes.
        add_sources(scalar._own(packet, "model_roots", "valid"), 1)
        add_sources(scalar._own(packet, "quads", "quad_valid"), 4)
        roots = packet.gather_original("model_roots").reshape(-1, 2)
        valid = packet.gather_original("valid").reshape(-1)
        _require(bool(np.all(valid)), "invalid_incident_source")
        # Preserve both endpoints of every occurrence, including repeats.
        add_sources(roots, 1)
        incident_count += len(roots)
        for affine in frame.pieces:
            events.append((1, affine))
    source_events = len(starts)
    if parts:
        points = np.concatenate(parts, axis=0)
        descriptor_starts = np.asarray(starts, dtype=np.int64)
        descriptor_lengths = np.asarray(lengths, dtype=np.int64)
        # Projection arrays are O(original source points), never sources x gaps.
        spans, laterals = _source_spans(points, descriptor_starts, descriptor_lengths, origin, tangent)
    else:
        spans = laterals = np.empty((0, 2), dtype=np.float64)
    pending, kinds, lateral_rows, gaps = [], [], [], []
    for kind, item in events:
        if kind == 0:
            span = tuple(float(value) for value in spans[item])
            lateral = tuple(float(value) for value in laterals[item])
        else:
            span, lateral = (item.lower, item.upper), None
        # Enumerate only actual complements. No source x every-gap matrix.
        for remaining in scalar._remaining(span, covered):
            pending.append(remaining)
            kinds.append(kind)
            lateral_rows.append(lateral)
            gaps.append(bisect_right(stops, remaining[0]))
    rows = len(pending)
    boxes_by_row = np.empty((rows, 2, 2), dtype=np.float64)
    rounded_denominator = None
    if any(kinds):
        # Lazy: old _world_envelope only uses this for an affine with a gap.
        value = arithmetic._dot(tangent, tangent)
        _require(math.isfinite(value) and value > 0., "invalid_rounded_denominator")
        rounded_denominator = _point_interval(value)
    for begin in range(0, rows, chunk_size):
        end = min(rows, begin + chunk_size)
        domain = np.asarray(pending[begin:end], dtype=np.float64)
        kind_array = np.asarray(kinds[begin:end], dtype=np.uint8)
        world = np.empty((end - begin, 2, 2), dtype=np.float64)
        for kind in (0, 1):
            selected = np.flatnonzero(kind_array == kind)
            if not len(selected):
                continue
            span = domain[selected]
            if kind == 0:
                lateral = np.asarray([lateral_rows[begin + int(row)] for row in selected], dtype=np.float64)
                bound = denominator
            else:
                lateral = _add(np.asarray(mean_intercept, dtype=np.float64),
                               _mul(np.asarray(mean_slope, dtype=np.float64), span))
                bound = rounded_denominator
            world[selected] = _world(span, lateral, origin, tangent, bound)
        # The original _envelope_box adds one further outward ULP per bound.
        lower = np.nextafter(world[:, :, 0], -np.inf)
        upper = np.nextafter(world[:, :, 1], np.inf)
        _require(bool(np.all(np.isfinite(lower))) and bool(np.all(np.isfinite(upper)))
                 and not bool(np.any(np.abs(lower) > 1.e7))
                 and not bool(np.any(np.abs(upper) > 1.e7)), "unrepresentable_box")
        boxes_by_row[begin:end] = np.stack((lower, upper), axis=1)
    boxes = {}
    # Fold in the original source/entity/occurrence/pending order. Python min/max
    # deliberately preserve their original equality and signed-zero behavior.
    for gap, row in zip(gaps, boxes_by_row):
        lower, upper = tuple(map(float, row[0])), tuple(map(float, row[1]))
        old = boxes.get(gap)
        if old is not None:
            lower = tuple(min(a, b) for a, b in zip(old[0], lower))
            upper = tuple(max(a, b) for a, b in zip(old[1], upper))
        boxes[gap] = (lower, upper)
    result = tuple(boxes[gap] for gap in sorted(boxes))
    return result, {"source_events": source_events, "source_points": point_count,
        "incident_occurrences": incident_count, "affine_events": len(all_affines),
        "pending_rows": rows, "source_pending_rows": kinds.count(0),
        "affine_pending_rows": kinds.count(1), "output_boxes": len(result)}


def candidate_remaining_boxes(state, covered, *, diagnostics=None, chunk_size=4096,
                              bindings=None):
    """Return exact candidate boxes or None; covered must be a reusable tuple.

    On None, run the bound scalar helper once with the same state and tuple.
    Optional diagnostics are caller-owned. This function never installs a hook.
    """
    baseline = _default_bindings() if bindings is None else bindings
    log = diagnostics
    if log is not None:
        log["calls"] = log.get("calls", 0) + 1
    try:
        # Identity checks precede even the caller-policy read. Speculative
        # numeric failures must not add warnings before the scalar fallback.
        _require(_unchanged(baseline), "changed_method_identity")
        _require(np.geterr() == _DEFAULT_FP, "nondefault_fp")
        with np.errstate(all="ignore"):
            _require(_binary64_environment(), "unsupported_binary64_environment")
            result, metrics = _candidate(state, covered, chunk_size, baseline.scalar)
    except _Decline as error:
        reason = str(error)
    except (ArithmeticError, ValueError, TypeError, IndexError, AttributeError, KeyError):
        reason = "unsupported_or_numeric_failure"
    else:
        if log is not None:
            log["hits"] = log.get("hits", 0) + 1
            for name, value in metrics.items():
                log[name] = log.get(name, 0) + value
        return result
    if log is not None:
        log["fallbacks"] = log.get("fallbacks", 0) + 1
        reasons = log.setdefault("fallback_reasons", {})
        reasons[reason] = reasons.get(reason, 0) + 1
    return None


def dispatch_remaining_boxes(state, covered, *, diagnostics=None, chunk_size=4096,
                             bindings=None):
    """Explicit opt-in dispatch; a generator is consumed exactly once."""
    baseline = _default_bindings() if bindings is None else bindings
    reusable = tuple(covered)
    result = candidate_remaining_boxes(state, reusable, diagnostics=diagnostics,
        chunk_size=chunk_size, bindings=baseline)
    if result is None:
        return baseline.scalar_remaining_boxes(state, reusable)
    return result

