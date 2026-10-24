"""Independent scalar-oracle and refusal tests for optional envelope batching.

The oracle retains the literal per-operation NumPy outward steps. Block input
is compared with its original complete dense source; decoding is not its oracle.
"""
from dataclasses import replace
import math
import unittest
from unittest import mock

import numpy as np

from lane_mapping_lya_reference import curve_envelope_batch as target
from lane_mapping_lya_reference import curve_geometry_builder as scalar

def _os(value, direction):
    return float(np.nextafter(np.float64(value), direction))


def _ob(lower, upper):
    if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
        raise ArithmeticError("unbounded curve competitor enclosure")
    return lower, upper


def _oa(first, second):
    return _ob(_os(first[0] + second[0], -np.inf), _os(first[1] + second[1], np.inf))


def _om(first, second):
    values = []
    for x in first:
        for y in second:
            values.append(x * y)
    if not all(math.isfinite(value) for value in values):
        raise ArithmeticError("unbounded curve competitor product")
    return _ob(_os(min(values), -np.inf), _os(max(values), np.inf))


def _od(first, second):
    if second[0] <= 0. <= second[1]:
        raise ArithmeticError("uncertain curve competitor denominator")
    values = []
    for x in first:
        for y in second:
            values.append(x / y)
    if not all(math.isfinite(value) for value in values):
        raise ArithmeticError("unbounded curve competitor quotient")
    return _ob(_os(min(values), -np.inf), _os(max(values), np.inf))


def _op(point, origin, vector):
    terms = []
    for x, o, t in zip(point, origin, vector):
        terms.append(_om(_oa((float(x), float(x)), (-o, -o)), (t, t)))
    return _oa(terms[0], terms[1])


def _ow(state, span, intercept, slope):
    tangent = state.tangent
    normal = (-tangent[1], tangent[0])
    # Explicit two multiply operations and one addition, independent of _dot.
    denominator = float(tangent[0]) * float(tangent[0]) + float(tangent[1]) * float(tangent[1])
    if not math.isfinite(denominator) or denominator <= 0.:
        raise ArithmeticError("invalid original curve chart denominator")
    lateral = _oa(intercept, _om(slope, span))
    coordinates = []
    for axis in (0, 1):
        numerator = _oa(_om((tangent[axis], tangent[axis]), span),
                        _om((normal[axis], normal[axis]), lateral))
        coordinates.append(_oa((state.origin[axis], state.origin[axis]),
                               _od(numerator, (denominator, denominator))))
    return tuple(coordinates)


def _ou(spans):
    result = []
    for lower, upper in sorted(spans):
        if not lower < upper:
            continue
        if result and lower <= result[-1][1]:
            result[-1] = result[-1][0], max(result[-1][1], upper)
        else:
            result.append((lower, upper))
    return tuple(result)


def _or(span, covered):
    lower, upper = span
    result, cursor = [], lower
    for first, last in covered:
        if last <= cursor or first >= upper:
            continue
        if cursor < first:
            result.append((cursor, min(first, upper)))
        cursor = max(cursor, last)
        if cursor >= upper:
            break
    if cursor < upper:
        result.append((cursor, upper))
    return tuple(result)


def manual_remaining_boxes(state, covered):
    """Independent scalar oracle; does not call any production arithmetic helper."""
    covered = _ou(covered)
    stops, boxes = tuple(upper for _, upper in covered), {}

    def add(pending, envelope):
        # Hand-written bisect_right, including equality on the right.
        low, high = 0, len(stops)
        while low < high:
            middle = (low + high) // 2
            if pending[0] < stops[middle]:
                high = middle
            else:
                low = middle + 1
        lower = tuple(_os(value[0], -np.inf) for value in envelope)
        upper = tuple(_os(value[1], np.inf) for value in envelope)
        if not all(math.isfinite(value) and abs(value) <= 1.e7 for value in lower + upper):
            raise ArithmeticError("unrepresentable finite source-complement box")
        old = boxes.get(low)
        if old is not None:
            lower = tuple(min(a, b) for a, b in zip(old[0], lower))
            upper = tuple(max(a, b) for a, b in zip(old[1], upper))
        boxes[low] = lower, upper

    tangent, origin = state.tangent, state.origin
    normal = (-tangent[1], tangent[0])
    denominator = _oa(_om((tangent[0], tangent[0]), (tangent[0], tangent[0])),
                      _om((tangent[1], tangent[1]), (tangent[1], tangent[1])))
    affines = tuple(piece for frame in state.frames for piece in frame.pieces)
    if affines:
        intercept = min(piece.intercept for piece in affines), max(piece.intercept for piece in affines)
        slope = min(piece.slope for piece in affines), max(piece.slope for piece in affines)
        for _ in range(2):
            intercept = _os(intercept[0], -np.inf), _os(intercept[1], np.inf)
            slope = _os(slope[0], -np.inf), _os(slope[1], np.inf)

    def source_event(points):
        data = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        if not len(data):
            return
        if not np.isfinite(data).all():
            raise ArithmeticError("invalid required remaining source coordinates")
        u = tuple(_op(point, origin, tangent) for point in data)
        v = tuple(_op(point, origin, normal) for point in data)
        span = min(value[0] for value in u), max(value[1] for value in u)
        lateral = min(value[0] for value in v), max(value[1] for value in v)
        for pending in _or(span, covered):
            envelope = []
            for axis in (0, 1):
                numerator = _oa(_om((tangent[axis], tangent[axis]), pending),
                                _om((normal[axis], normal[axis]), lateral))
                envelope.append(_oa((origin[axis], origin[axis]), _od(numerator, denominator)))
            add(pending, tuple(envelope))

    for frame in state.frames:
        if frame.status != "known":
            raise ArithmeticError("partly ready key has unknown source chart")
        packet, source = frame.packet, frame.packet.source
        own = source.endpoint_owners == state.owner_id
        for point in source.model_roots[own & source.valid]:
            source_event((point,))
        for quad in source.quads[own & source.quad_valid]:
            source_event(quad)
        for face in packet.face_indices.reshape(-1):
            if not source.valid[face]:
                raise ArithmeticError("invalid required remaining incident source")
            source_event((source.model_roots[face],))
        for affine in frame.pieces:
            for pending in _or((affine.lower, affine.upper), covered):
                add(pending, _ow(state, pending, intercept, slope))
    return tuple(boxes[key] for key in sorted(boxes))


def _make_state(*, seed=20261015, frames=5, owners=None, angle=0., origin=(0., 0.), reliable_at=0):
    """Genuine sealed source/packet factories and the real compact advance path."""
    rng = np.random.default_rng(seed)
    count = 6 if owners is None else len(owners)
    owners = np.zeros(count, dtype=np.int64) if owners is None else np.asarray(owners, dtype=np.int64)
    tangent = (math.cos(angle), math.sin(angle))
    normal = (-tangent[1], tangent[0])
    positions = np.arange(count, dtype=np.float64) + rng.uniform(-.025, .025, count)
    # A straight sequence keeps genuine charts known; quads need not be centred.
    points = np.asarray(origin) + positions[:, None] * np.asarray(tangent)
    offsets = np.asarray(((-.04, -.025), (.04, -.025), (.04, .025), (-.04, .025)))
    quads = points[:, None, :] + offsets[None, :, :]
    links = np.column_stack((np.arange(count, dtype=np.int64) - 1,
                             np.arange(count, dtype=np.int64) + 1))
    links[-1, 1] = -1
    pairs = np.column_stack((np.arange(count - 1, dtype=np.int64),
                             np.arange(1, count, dtype=np.int64)))
    selected = np.any(owners[pairs] == 0, axis=1)
    pairs = pairs[selected]
    roles = np.where(owners[pairs[:, 0]] == owners[pairs[:, 1]],
                     target.evidence.SAME_OWNER_MODEL_NEXT,
                     target.evidence.CROSS_OWNER_INCIDENT).astype(np.uint8)
    state = target.compact.create_curve_state(0, 1, (-1, -1), tuple(map(float, origin)), tangent,
        normal_cosine=math.cos(.45), max_support_records=200000)
    for frame_index in range(frames):
        reliable = np.full(count, frame_index == reliable_at, dtype=np.bool_)
        source = target.evidence.SourceFrame(
            stamp_ns=frame_index + 1, face_ids=np.arange(count, dtype=np.int64),
            previous_next=links, contour_ids=np.zeros(count, dtype=np.int64),
            endpoint_owners=owners, model_roots=points,
            normals=np.tile(normal, (count, 1)).astype(np.float64), quads=quads,
            quad_valid=np.ones(count, dtype=np.bool_), reliable=reliable,
            valid=np.ones(count, dtype=np.bool_))
        packet = target.evidence.PairPacket(source, 0, 1, (-1, -1), roles, pairs)
        state = target.compact.advance_curve(state, packet, available_before_ns=frame_index + 1)
    return state


def _bytes(value):
    return np.asarray(value, dtype=np.float64).reshape(-1, 2, 2).tobytes(order="C")


def _snapshot(state):
    result = []
    for frame in state.frames:
        packet = frame.packet
        result.extend((name, getattr(packet.source, name).dtype.str,
                       getattr(packet.source, name).shape,
                       getattr(packet.source, name).tobytes(),
                       getattr(packet.source, name).flags.writeable)
                      for name, _, _ in target.evidence._FIELDS)
        result.extend((name, getattr(packet, name).dtype.str, getattr(packet, name).shape,
                       getattr(packet, name).tobytes(), getattr(packet, name).flags.writeable)
                      for name in ("roles", "face_indices"))
    return tuple(result)


def _outcome(function, *args):
    try:
        return "value", _bytes(function(*args))
    except Exception as error:
        return "exception", type(error), str(error)


class CandidateTests(unittest.TestCase):
    def assert_exact(self, state, covered, *, chunk_size=4096):
        before = _snapshot(state)
        log = target.new_diagnostics()
        expected = manual_remaining_boxes(state, covered)
        candidate = target.candidate_remaining_boxes(state, covered, diagnostics=log, chunk_size=chunk_size)
        self.assertIsNotNone(candidate, log)
        self.assertEqual(_bytes(expected), _bytes(candidate))
        self.assertEqual(_bytes(expected), _bytes(scalar._remaining_boxes_scalar(state, covered)))
        self.assertEqual(before, _snapshot(state))
        self.assertEqual(log["hits"], 1)
        self.assertEqual(log["fallbacks"], 0)
        return candidate, log

    def test_seeded_real_sealed_packets_match_manual_oracle(self):
        for seed in (20261015, 1, 17, 9001, 65537, 2147483647):
            with self.subTest(seed=seed):
                state = _make_state(seed=seed, angle=.17, origin=(.3, -.7))
                self.assertTrue(all(frame.status == "known" for frame in state.frames))
                self.assert_exact(state, ((.4, 1.25), (2.1, 2.75), (3.25, 4.0)))

    def test_no_ready_coverage_and_full_coverage(self):
        state = _make_state()
        self.assert_exact(state, ())
        result, log = self.assert_exact(state, ((-100., 100.),))
        self.assertEqual(result, ())
        self.assertEqual(log["pending_rows"], 0)

    def test_hole_islands_stay_separate_and_touching_intervals_merge(self):
        state = _make_state()
        covered = ((2., 2.5), (.5, 1.), (1., 1.5), (3.2, 4.1), (4.1, 4.1))
        result, log = self.assert_exact(state, covered)
        self.assertGreaterEqual(len(result), 3)
        self.assertGreater(log["pending_rows"], len(result))
        self.assertEqual(_bytes(result), _bytes(manual_remaining_boxes(state, _ou(covered))))

    def test_chunk_boundaries_do_not_change_order_or_bytes(self):
        state = _make_state(angle=-.21)
        covered = ((.2, .75), (1.2, 2.), (2.7, 3.2), (4.1, 4.6))
        expected = None
        for chunk in (1, 2, 7, 31, 4096):
            result, _ = self.assert_exact(state, covered, chunk_size=chunk)
            if expected is None:
                expected = _bytes(result)
            self.assertEqual(expected, _bytes(result))

    def test_same_owner_and_incident_roles_keep_every_endpoint_occurrence(self):
        state = _make_state(owners=(0, 0, 1, 0, 0, 1, 0))
        _, log = self.assert_exact(state, ((.7, 1.1), (2.5, 3.), (4.2, 4.8)))
        expected_incident = sum(frame.packet.face_indices.size for frame in state.frames)
        expected_events = sum(int(np.sum(frame.packet.source.valid
            & (frame.packet.source.endpoint_owners == 0)))
            + int(np.sum(frame.packet.source.quad_valid
            & (frame.packet.source.endpoint_owners == 0)))
            + frame.packet.face_indices.size for frame in state.frames)
        self.assertEqual(log["incident_occurrences"], expected_incident)
        self.assertEqual(log["source_events"], expected_events)
        self.assertTrue(any(np.any(frame.packet.roles == target.evidence.CROSS_OWNER_INCIDENT)
                            for frame in state.frames))
        self.assertTrue(any(np.any(frame.packet.roles == target.evidence.SAME_OWNER_MODEL_NEXT)
                            for frame in state.frames))

    def test_latest_five_plus_older_reliable_frame_is_not_reduced(self):
        state = _make_state(frames=9, reliable_at=0)
        self.assertEqual(tuple(frame.packet.stamp_ns for frame in state.frames), (1, 5, 6, 7, 8, 9))
        _, log = self.assert_exact(state, ((.3, .8), (2., 3.)))
        self.assertEqual(log["incident_occurrences"], sum(frame.packet.face_indices.size for frame in state.frames))

    def test_projection_signed_zero_subnormal_cancellation_and_boundaries(self):
        tiny = float(np.nextafter(np.float64(0.), np.inf))
        points = np.asarray(((-0., 0.), (0., -0.), (tiny, -tiny),
                             (1.e7, -1.e7), (1., np.nextafter(1., np.inf)),
                             (-1., np.nextafter(-1., -np.inf))), dtype=np.float64)
        for origin in ((0., -0.), (-0., 0.), (1., -1.), (1.e7, -1.e7)):
            for vector in ((1., 0.), (-1., -0.), (0., 1.), (-0., -1.),
                           (math.cos(.23), math.sin(.23))):
                with self.subTest(origin=origin, vector=vector), np.errstate(all="ignore"):
                    expected = np.asarray([_op(point, origin, vector) for point in points], dtype=np.float64)
                    actual = target._project_points(points, origin, vector)
                    self.assertEqual(expected.tobytes(), actual.tobytes())

    def test_four_product_extrema_ties_preserve_first_signed_zero(self):
        first = np.asarray(((-0., 0.), (-1., 1.), (-2., -1.), (0., 0.)), dtype=np.float64)
        second = np.asarray(((0., -0.), (-0., 0.), (-3., -2.), (-0., 0.)), dtype=np.float64)
        with np.errstate(all="ignore"):
            expected = np.asarray([_om(tuple(a), tuple(b)) for a, b in zip(first, second)])
            actual = target._mul(first, second)
        self.assertEqual(expected.tobytes(), actual.tobytes())

    def test_every_source_and_affine_envelope_row_matches_manual_bits(self):
        state = _make_state(angle=-.31, origin=(.25, -.75))
        tiny = float(np.nextafter(np.float64(0.), np.inf))
        pending = np.asarray(((-0., 0.), (-tiny, tiny), (-1., 1.), (.5, 2.)), dtype=np.float64)
        lateral = np.asarray(((-0., 0.), (-tiny, tiny), (-.3, .7), (-2., -1.)), dtype=np.float64)
        tangent, origin = state.tangent, state.origin
        normal = (-tangent[1], tangent[0])
        denominator = _oa(_om((tangent[0], tangent[0]), (tangent[0], tangent[0])),
                          _om((tangent[1], tangent[1]), (tangent[1], tangent[1])))
        expected = []
        for span, cross in zip(pending, lateral):
            span, cross = tuple(map(float, span)), tuple(map(float, cross))
            envelope = []
            for axis in (0, 1):
                numerator = _oa(_om((tangent[axis], tangent[axis]), span),
                                _om((normal[axis], normal[axis]), cross))
                envelope.append(_oa((origin[axis], origin[axis]), _od(numerator, denominator)))
            expected.append(envelope)
        with np.errstate(all="ignore"):
            actual = target._world(pending, lateral, origin, tangent, np.asarray(denominator))
        expected = np.asarray(expected, dtype=np.float64)
        self.assertEqual(expected.tobytes(), actual.tobytes())
        intercept, slope = (-.2, .1), (-.4, .3)
        expected_affine = np.asarray([_ow(state, tuple(map(float, span)), intercept, slope) for span in pending])
        rounded = float(tangent[0]) * float(tangent[0]) + float(tangent[1]) * float(tangent[1])
        with np.errstate(all="ignore"):
            cross = target._add(np.asarray(intercept), target._mul(np.asarray(slope), pending))
            actual_affine = target._world(pending, cross, origin, tangent, np.asarray((rounded, rounded)))
        self.assertEqual(expected_affine.tobytes(), actual_affine.tobytes())
        # Compare the complete final lower/upper bits of EVERY pending row,
        # before gap aggregation can hide a mistaken non-extremal row.
        for oracle_rows, batch_rows in ((expected, actual), (expected_affine, actual_affine)):
            oracle_boxes = np.asarray([(tuple(_os(value[0], -np.inf) for value in row),
                                        tuple(_os(value[1], np.inf) for value in row))
                                       for row in oracle_rows], dtype=np.float64)
            with np.errstate(all="ignore"):
                batch_boxes = np.stack((np.nextafter(batch_rows[:, :, 0], -np.inf),
                                        np.nextafter(batch_rows[:, :, 1], np.inf)), axis=1)
            self.assertEqual(oracle_boxes.tobytes(), batch_boxes.tobytes())

    def test_strict_floating_point_policy_declines_and_restores_policy(self):
        state, covered = _make_state(), ((.5, 1.5),)
        previous = np.geterr()
        for policy in (dict(all="raise"), dict(under="warn"), dict(over="ignore")):
            with self.subTest(policy=policy), np.errstate(**policy):
                active = np.geterr()
                self.assertIsNone(target.candidate_remaining_boxes(state, covered))
                self.assertEqual(np.geterr(), active)
                self.assertEqual(_outcome(scalar._remaining_boxes_scalar, state, covered),
                                 _outcome(target.dispatch_remaining_boxes, state, covered))
        self.assertEqual(np.geterr(), previous)

    def test_changed_scalar_method_identity_declines_and_preserves_error(self):
        state, covered = _make_state(), ((.5, 1.5),)
        with mock.patch.object(scalar, "_projection", side_effect=RuntimeError("identity sentinel")):
            self.assertIsNone(target.candidate_remaining_boxes(state, covered))
            self.assertEqual(_outcome(scalar._remaining_boxes_scalar, state, covered),
                             _outcome(target.dispatch_remaining_boxes, state, covered))
        with mock.patch.object(target.arithmetic, "_dot", wraps=target._DOT):
            self.assertIsNone(target.candidate_remaining_boxes(state, covered))

    def test_changed_numpy_nextafter_identity_declines(self):
        state, covered = _make_state(), ((.5, 1.5),)
        old = np.nextafter
        with mock.patch.object(np, "nextafter", wraps=old):
            self.assertIsNone(target.candidate_remaining_boxes(state, covered))

    def test_unknown_frame_is_preserved_and_scalar_first_error_is_recovered(self):
        state = _make_state()
        changed = replace(state.frames[-1], status="unknown_original_chart")
        unknown = replace(state, frames=state.frames[:-1] + (changed,))
        before = _snapshot(unknown)
        self.assertIsNone(target.candidate_remaining_boxes(unknown, ()))
        self.assertEqual(_outcome(manual_remaining_boxes, unknown, ()),
                         _outcome(target.dispatch_remaining_boxes, unknown, ()))
        self.assertEqual(before, _snapshot(unknown))

    def test_unsupported_frame_property_is_not_speculatively_read(self):
        state = _make_state()
        reads = []
        class UnsupportedFrame:
            status = "unknown"
            @property
            def pieces(self):
                reads.append("pieces")
                return ()
        unsupported = replace(state, frames=(UnsupportedFrame(),))
        self.assertIsNone(target.candidate_remaining_boxes(unsupported, ()))
        self.assertEqual(reads, [])
        result = _outcome(target.dispatch_remaining_boxes, unsupported, ())
        self.assertEqual(result, ("exception", ArithmeticError,
                                  "partly ready key has unknown source chart"))
        self.assertEqual(reads, ["pieces"])

    def test_unowned_valid_quad_and_invalid_placeholder_bytes_are_still_retained(self):
        state = _make_state()
        frame = state.frames[0]
        source, packet = frame.packet.source, frame.packet
        count = len(source.face_ids)
        extended = target.evidence.SourceFrame(
            stamp_ns=source.stamp_ns,
            face_ids=np.arange(count + 1, dtype=np.int64),
            previous_next=np.concatenate((source.previous_next, np.asarray(((-1, -1),), dtype=np.int64))),
            contour_ids=np.r_[source.contour_ids, np.int64(1)],
            endpoint_owners=np.r_[source.endpoint_owners, np.int64(-1)],
            model_roots=np.concatenate((source.model_roots, np.asarray(((np.nan, np.nan),)))),
            normals=np.concatenate((source.normals, np.asarray(((np.nan, np.nan),)))),
            quads=np.concatenate((source.quads, np.asarray((((10., 10.), (10.1, 10.),
                                                           (10.1, 10.1), (10., 10.1)),)))),
            quad_valid=np.r_[source.quad_valid, np.bool_(True)],
            reliable=np.r_[source.reliable, np.bool_(False)],
            valid=np.r_[source.valid, np.bool_(False)])
        changed_packet = target.evidence.PairPacket(extended, packet.owner_id, packet.signed_side,
            packet.signature, packet.roles, packet.face_indices)
        changed_frame = replace(frame, packet=changed_packet)
        changed = replace(state, frames=(changed_frame,) + state.frames[1:])
        old_count = target.compact.inventory((state,))["source_faces"]
        self.assert_exact(changed, ((.5, 1.5),))
        self.assertEqual(target.compact.inventory((changed,))["source_faces"], old_count + 1)
        self.assertTrue(extended.quad_valid[-1])
        self.assertEqual(int(extended.endpoint_owners[-1]), -1)
        self.assertFalse(extended.valid[-1])

    def test_later_unknown_cannot_replace_earlier_numeric_scalar_error(self):
        state = _make_state(frames=2)
        # Adversarial internal chart: force an earlier finite-envelope rejection.
        affine = replace(state.frames[0].pieces[0], intercept=2.e7)
        first = replace(state.frames[0], pieces=(affine,))
        later = replace(state.frames[1], status="later_unknown")
        adversarial = replace(state, frames=(first, later))
        self.assertIsNone(target.candidate_remaining_boxes(adversarial, ()))
        expected = _outcome(manual_remaining_boxes, adversarial, ())
        self.assertEqual(expected[0], "exception")
        self.assertEqual(expected[2], "unrepresentable finite source-complement box")
        self.assertEqual(expected, _outcome(target.dispatch_remaining_boxes, adversarial, ()))

    def test_unsupported_type_dtype_and_chunk_decline_without_input_write(self):
        state, covered = _make_state(), ((.5, 1.5),)
        self.assertIsNone(target.candidate_remaining_boxes(object(), covered))
        self.assertIsNone(target.candidate_remaining_boxes(state, covered, chunk_size=0))
        self.assertIsNone(target.candidate_remaining_boxes(state, covered, chunk_size=True))
        source = state.frames[0].packet.source
        # A deliberately broken sealed fixture tests defense against corruption.
        old = source.model_roots
        foreign = old.astype(">f8" if np.little_endian else "<f8")
        foreign.flags.writeable = False
        try:
            object.__setattr__(source, "model_roots", foreign)
            before = foreign.tobytes()
            self.assertIsNone(target.candidate_remaining_boxes(state, covered))
            self.assertEqual(before, foreign.tobytes())
        finally:
            object.__setattr__(source, "model_roots", old)

    def test_dispatch_generator_is_consumed_once_and_candidate_has_hit_coverage(self):
        state = _make_state()
        source = ((.5, 1.5), (2., 2.5))
        seen = []
        def covered():
            for item in source:
                seen.append(item)
                yield item
        log = target.new_diagnostics()
        expected = manual_remaining_boxes(state, source)
        actual = target.dispatch_remaining_boxes(state, covered(), diagnostics=log)
        self.assertEqual(seen, list(source))
        self.assertEqual(_bytes(expected), _bytes(actual))
        self.assertEqual((log["calls"], log["hits"], log["fallbacks"]), (1, 1, 0))

    def test_distinct_signatures_are_independent_calls_not_selected_alternatives(self):
        state = _make_state()
        alternative = replace(state, signature=(2, 3))
        log = target.new_diagnostics()
        for value in (state, alternative):
            expected = manual_remaining_boxes(value, ((.5, 1.5),))
            actual = target.dispatch_remaining_boxes(value, ((.5, 1.5),), diagnostics=log)
            self.assertEqual(_bytes(expected), _bytes(actual))
        self.assertEqual(log["hits"], 2)



def _retained(state):
    frames = []
    for frame in state.frames:
        projected, _ = target.blocks.project_frame(frame.packet.source, (frame.packet,))
        frames.append(replace(frame, packet=projected[0]))
    return replace(state, frames=tuple(frames))


def _block_snapshot(state):
    return tuple((tuple((block.face_id, block.payload)
                        for block in frame.packet.lease.required_blocks),
                  tuple((block.face_id, block.payload)
                        for block in frame.packet.lease.unowned.blocks),
                  frame.packet.roles.tobytes(), frame.packet.face_indices.tobytes())
                 for frame in state.frames)


class RetainedAndBindingTests(unittest.TestCase):
    def test_production_wrapper_uses_fixed_baseline_and_batch_path(self):
        state, covered = _make_state(), ((.5, 1.5),)
        candidate = target.candidate_remaining_boxes
        with mock.patch.object(target, 'candidate_remaining_boxes', wraps=candidate) as called:
            actual = scalar._remaining_boxes(state, iter(covered))
        self.assertEqual(called.call_count, 1)
        self.assertIs(called.call_args.kwargs['bindings'], scalar._ENVELOPE_BINDINGS)
        self.assertEqual(_bytes(actual), _bytes(manual_remaining_boxes(state, covered)))
        with mock.patch.object(scalar, '_projection', side_effect=ArithmeticError('fixed baseline')):
            self.assertIsNone(target.candidate_remaining_boxes(state, covered))
            self.assertEqual(_outcome(scalar._remaining_boxes_scalar, state, covered),
                             _outcome(scalar._remaining_boxes, state, covered))

    def test_changed_scalar_body_code_declines(self):
        state = _make_state()
        helper = scalar._remaining_boxes_scalar
        old_code = helper.__code__
        def changed(state, covered):
            raise ArithmeticError('changed scalar body')
        try:
            helper.__code__ = changed.__code__
            self.assertIsNone(target.candidate_remaining_boxes(state, ()))
            self.assertEqual(_outcome(scalar._remaining_boxes, state, ()),
                             ('exception', ArithmeticError, 'changed scalar body'))
        finally:
            helper.__code__ = old_code

    def test_retained_blocks_match_complete_dense_oracle_and_scalar(self):
        for owners in ((0, 0, 0, 0, 0, 0), (0, 0, 1, 0, 0, 1, 0)):
            for angle in (0., .17, -.23):
                with self.subTest(owners=owners, angle=angle):
                    dense = _make_state(owners=owners, angle=angle, origin=(.3, -.7))
                    retained = _retained(dense)
                    before = _block_snapshot(retained)
                    for covered in ((), ((.4, 1.25), (2.1, 2.75), (3.25, 4.)),
                                    ((-100., 100.),)):
                        log = target.new_diagnostics()
                        actual = target.candidate_remaining_boxes(retained, covered, diagnostics=log)
                        self.assertIsNotNone(actual, log)
                        expected = manual_remaining_boxes(dense, covered)
                        self.assertEqual(_bytes(actual), _bytes(expected))
                        self.assertEqual(_bytes(actual),
                                         _bytes(scalar._remaining_boxes_scalar(retained, covered)))
                        self.assertEqual(log["hits"], 1)
                        self.assertEqual(log["incident_occurrences"],
                                         sum(f.packet.face_indices.size for f in dense.frames))
                    self.assertEqual(before, _block_snapshot(retained))

    def test_retained_sixth_reliable_frame_and_chunk_order_are_preserved(self):
        dense = _make_state(frames=9, reliable_at=0, angle=.19)
        retained = _retained(dense)
        self.assertEqual(tuple(f.packet.stamp_ns for f in retained.frames), (1, 5, 6, 7, 8, 9))
        covered = ((.2, .75), (1.2, 2.), (2.7, 3.2), (4.1, 4.6))
        expected = _bytes(manual_remaining_boxes(dense, covered))
        for chunk in (1, 2, 7, 31, 4096):
            log = target.new_diagnostics()
            actual = target.candidate_remaining_boxes(retained, covered,
                                                      chunk_size=chunk, diagnostics=log)
            self.assertIsNotNone(actual, log)
            self.assertEqual(_bytes(actual), expected)

    def test_isolated_own_root_and_quad_are_not_lost_by_block_projection(self):
        dense = _make_state(frames=1)
        frame, source = dense.frames[0], dense.frames[0].packet.source
        count = len(source.face_ids)
        # Add an own root and independent quad with no pair occurrence, plus
        # an unowned quad. Both kinds remain in their complete original rows.
        extended = target.evidence.SourceFrame(
            stamp_ns=source.stamp_ns, face_ids=np.arange(count + 2, dtype=np.int64),
            previous_next=np.concatenate((source.previous_next,
                                          np.asarray(((-1, -1), (-1, -1)), dtype=np.int64))),
            contour_ids=np.r_[source.contour_ids, np.int64(1), np.int64(2)],
            endpoint_owners=np.r_[source.endpoint_owners, np.int64(0), np.int64(-1)],
            model_roots=np.concatenate((source.model_roots, np.asarray(((8., .2), (np.nan, np.nan))))),
            normals=np.concatenate((source.normals, np.asarray(((0., 1.), (np.nan, np.nan))))),
            quads=np.concatenate((source.quads, np.asarray((
                ((7.8, .1), (8.2, .1), (8.2, .3), (7.8, .3)),
                ((10., 10.), (10.1, 10.), (10.1, 10.1), (10., 10.1)))))),
            quad_valid=np.r_[source.quad_valid, np.bool_(True), np.bool_(True)],
            reliable=np.r_[source.reliable, np.bool_(True), np.bool_(False)],
            valid=np.r_[source.valid, np.bool_(True), np.bool_(False)])
        packet = target.evidence.PairPacket(extended, 0, 1, (-1, -1),
                                           frame.packet.roles, frame.packet.face_indices)
        dense = replace(dense, frames=(replace(frame, packet=packet),))
        retained = _retained(dense)
        self.assertIn(count, [b.face_id for b in retained.frames[0].packet.lease.own_blocks])
        self.assertEqual([b.face_id for b in retained.frames[0].packet.lease.unowned.blocks],
                         [count + 1])
        log = target.new_diagnostics()
        actual = target.candidate_remaining_boxes(retained, ((.5, 1.5),), diagnostics=log)
        self.assertIsNotNone(actual, log)
        self.assertEqual(_bytes(actual), _bytes(manual_remaining_boxes(dense, ((.5, 1.5),))))
        self.assertGreater(max(box[1][0] for box in actual), 8.)

    def test_changed_retained_reader_declines_before_its_first_read(self):
        retained = _retained(_make_state())
        baseline = target.bind_scalar_helpers()
        with mock.patch.object(target.blocks.BlockPacket, "own_original",
                               side_effect=RuntimeError("retained reader sentinel")) as changed:
            self.assertIsNone(target.candidate_remaining_boxes(retained, (), bindings=baseline))
            changed.assert_not_called()
            expected = _outcome(baseline.scalar_remaining_boxes, retained, ())
            actual = _outcome(lambda s, c: target.dispatch_remaining_boxes(
                s, c, bindings=baseline), retained, ())
            self.assertEqual(expected, actual)

    def test_changed_reader_code_without_rebinding_declines(self):
        state = _make_state()
        baseline = target.bind_scalar_helpers()
        helper = scalar._projection
        old_code = helper.__code__
        def changed(point, origin, vector):
            raise RuntimeError("same function code sentinel")
        try:
            helper.__code__ = changed.__code__
            self.assertIsNone(target.candidate_remaining_boxes(state, (), bindings=baseline))
            self.assertEqual(_outcome(baseline.scalar_remaining_boxes, state, ()),
                _outcome(lambda s, c: target.dispatch_remaining_boxes(
                    s, c, bindings=baseline), state, ()))
        finally:
            helper.__code__ = old_code

    def test_changed_scalar_rounding_binding_declines(self):
        state = _make_state()
        baseline = target.bind_scalar_helpers()
        with mock.patch.object(target.rounding, "_MATH_NEXTAFTER", object()):
            self.assertIsNone(target.candidate_remaining_boxes(state, (), bindings=baseline))

    def test_changed_block_decoder_numpy_reader_declines_without_calling_it(self):
        state = _retained(_make_state())
        with mock.patch.object(np, 'frombuffer', side_effect=RuntimeError('decoder sentinel')) as changed:
            self.assertIsNone(target.candidate_remaining_boxes(state, ()))
            changed.assert_not_called()

    def test_changed_numpy_policy_reader_is_not_speculatively_called(self):
        baseline, state = target.bind_scalar_helpers(), _make_state()
        with mock.patch.object(np, "geterr", side_effect=RuntimeError("policy sentinel")) as changed:
            self.assertIsNone(target.candidate_remaining_boxes(state, (), bindings=baseline))
            changed.assert_not_called()

    def test_failed_environment_probe_declines_without_global_diagnostics(self):
        state = _make_state()
        self.assertFalse(hasattr(target, "DIAGNOSTICS"))
        log = target.new_diagnostics()
        with mock.patch.object(target, "_binary64_environment", return_value=False):
            self.assertIsNone(target.candidate_remaining_boxes(state, (), diagnostics=log))
        self.assertEqual(log["fallbacks"], 1)
        self.assertEqual(log["fallback_reasons"], {"unsupported_binary64_environment": 1})
        fresh = target.new_diagnostics()
        self.assertEqual(fresh["calls"], 0)
        self.assertEqual(fresh["fallback_reasons"], {})

if __name__ == "__main__":
    unittest.main()

