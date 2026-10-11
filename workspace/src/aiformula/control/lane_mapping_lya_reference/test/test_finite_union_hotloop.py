"""Literal R19 union oracle for allocation-only hot-loop changes.

The oracle below is the unmodified _current_union from R19 commit
01442bbf75cb68213dff3c05b88f3bef8639a877 (finite source aa85d639f290...).
Projection/connectivity are deliberately the same unchanged dependencies;
this suite isolates the hot loop, then checks public state/export histories.
No private diagnostic path, recorded trajectory or simulator data is used.
"""
from dataclasses import fields, is_dataclass
import math
import random
import struct
import unittest
from unittest.mock import patch

import numpy as np

from lane_mapping_lya_reference import finite_support as fs

_connected_spans = fs._connected_spans
_validated_projection_bounds = fs._validated_projection_bounds


def literal_r19_union(segments, normals, reliable, origin, tangent, cosine):
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


def identity(value):
    if isinstance(value, np.ndarray):
        return value.dtype.str, value.shape, value.strides, value.tobytes(), value.flags.writeable
    if is_dataclass(value):
        return tuple((field.name, identity(getattr(value, field.name))) for field in fields(value))
    if isinstance(value, dict):
        return tuple((key, identity(item)) for key, item in sorted(value.items()))
    if isinstance(value, (list, tuple)):
        return tuple(identity(item) for item in value)
    if isinstance(value, (float, np.floating)):
        return struct.pack('>d', float(value))
    return value


def outcome(function, *args, **kwargs):
    try:
        return 'value', function(*args, **kwargs)
    except Exception as error:
        return 'error', (type(error).__name__, str(error))


def geometry(pairs=(), *, normal=(0., 1.), origin=(0., 0.), flags=None):
    tangent = (normal[1], -normal[0])
    positions = np.asarray(pairs, dtype=np.float64).reshape(-1, 2)
    segments = np.asarray(origin) + positions[:, :, None] * np.asarray(tangent)
    normals = np.tile(normal, (len(positions), 2, 1))
    reliable = np.ones(len(positions), dtype=bool) if flags is None else np.asarray(flags, dtype=bool)
    return segments, normals, reliable, origin, tangent, math.cos(math.pi/6.)


class UnionHotloopTests(unittest.TestCase):
    def assert_union(self, values):
        before = identity(values)
        expected = outcome(literal_r19_union, *values)
        self.assertEqual(identity(values), before)
        actual = outcome(fs._current_union, *values)
        self.assertEqual(identity(values), before)
        self.assertEqual(identity(actual), identity(expected))
        self.assertEqual(actual[0], 'value')
        return actual[1]

    def test_empty(self):
        self.assertEqual(self.assert_union(geometry()), [])

    def test_single_segment(self):
        self.assertEqual(len(self.assert_union(geometry(((0., 1.),)))), 1)

    def test_all_untrusted(self):
        pieces = self.assert_union(geometry(((0., 1.), (1., 2.)), flags=(False, False)))
        self.assertEqual(len(pieces), 1)
        self.assertFalse(pieces[0][2])

    def test_mixed_reliability_bridge_is_not_trusted(self):
        pieces = self.assert_union(geometry(((0., 1.), (1., 2.), (2., 3.)), flags=(True, False, True)))
        for coordinate, expected in ((.5, True), (1.5, False), (2.5, True)):
            covering = [reliable for lower, upper, reliable in pieces if lower < coordinate < upper]
            self.assertEqual(covering, [expected])

    def test_duplicate_edges_and_reverse_endpoints(self):
        self.assert_union(geometry(((0., 1.), (1., 0.), (0., 1.), (1., 2.), (2., 1.)),
                                   flags=(False, True, False, False, True)))

    def test_branch_and_cycle_exact_endpoint_connectivity(self):
        values = list(geometry(((0., 1.),)*5))
        values[0] = np.array([[[0., 0.], [1., 0.]], [[1., 0.], [2., .1]],
                              [[1., 0.], [2., -.1]], [[2., .1], [2., -.1]],
                              [[2., -.1], [0., 0.]]])
        self.assert_union(values)

    def test_real_gap_and_one_ulp_gap(self):
        for start in (float(np.nextafter(1., math.inf)), 1.125):
            with self.subTest(start=start):
                pieces = self.assert_union(geometry(((0., 1.), (start, 2.))))
                self.assertEqual(len(pieces), 2)
                self.assertLess(pieces[0][1], pieces[1][0])

    def test_signed_zero_first_vertex_is_not_replaced(self):
        values = list(geometry(((0., 1.), (0., -1.))))
        values[0][0, 0] = (-0., +0.)
        values[0][1, 0] = (+0., -0.)
        self.assert_union(values)

    def test_first_seen_projection_order_and_signed_zero_bits(self):
        values = list(geometry(((0., 1.), (0., -1.), (1., 2.))))
        values[0][0, 0] = (-0., +0.)
        values[0][1, 0] = (+0., -0.)
        histories = []
        original_projection = _validated_projection_bounds
        for function in (literal_r19_union, fs._current_union):
            seen = []

            def project(point, origin, tangent):
                seen.append(identity(tuple(map(float, point))))
                return original_projection(point, origin, tangent)

            with patch.dict(literal_r19_union.__globals__, {'_validated_projection_bounds': project}), \
                    patch.object(fs, '_validated_projection_bounds', project):
                function(*values)
            histories.append(seen)
        self.assertEqual(histories[0], histories[1])
        self.assertEqual(len(histories[0]), 4)
        self.assertEqual(histories[0][0], identity((-0., +0.)))

    def test_zero_extent_is_not_new_support(self):
        self.assertEqual(self.assert_union(geometry(((1., 1.),))), [])

    def test_opposite_normals_reject_identically(self):
        values = list(geometry(((0., 1.), (2., 3.))))
        values[1][0, 0] *= -1.
        values[1][1, 1] *= -1.
        self.assertEqual(self.assert_union(values), [])

    def test_cosine_threshold_and_neighbouring_binary64_values(self):
        cosine = math.cos(math.pi/6.)
        for value in (float(np.nextafter(cosine, -math.inf)), cosine,
                      float(np.nextafter(cosine, math.inf))):
            for endpoint in (0, 1):
                values = list(geometry(((0., 1.),)))
                values[1][0, endpoint] = (math.sqrt(1.-value*value), value)
                with self.subTest(value=value, endpoint=endpoint):
                    pieces = self.assert_union(values)
                    self.assertEqual(bool(pieces), value >= cosine)

    def test_dot_arguments_results_and_short_circuit_order(self):
        values = list(geometry(((0., 1.), (1., 2.), (2., 3.)), normal=(.6, .8)))
        values[1][1, 0] *= -1.
        values[1][2, 1] *= -1.
        original_dot = np.dot
        traces = []
        for function in (literal_r19_union, fs._current_union):
            seen = []

            def dot(first, second):
                value = original_dot(first, second)
                seen.append((identity(np.asarray(first)), identity(np.asarray(second)), identity(value)))
                return value

            with patch.object(np, 'dot', dot):
                function(*values)
            traces.append(seen)
        self.assertEqual(traces[0], traces[1])
        self.assertEqual(len(traces[0]), 5)

    def test_first_rejected_dot_does_not_evaluate_second_endpoint(self):
        values = list(geometry(((0., 1.),)))
        values[1][0, 0] *= -1.
        original_dot = np.dot
        for function in (literal_r19_union, fs._current_union):
            count = [0]

            def dot(first, second):
                count[0] += 1
                if count[0] > 1:
                    raise RuntimeError('second endpoint must not execute')
                return original_dot(first, second)

            with patch.object(np, 'dot', dot):
                self.assertEqual(function(*values), [])
            self.assertEqual(count[0], 1)

    def test_dot_error_is_not_silenced_or_retried(self):
        values = geometry(((0., 1.),))
        original_dot = np.dot
        results = []
        for function in (literal_r19_union, fs._current_union):
            count = [0]

            def dot(first, second):
                count[0] += 1
                if count[0] == 2:
                    raise FloatingPointError('injected second dot failure')
                return original_dot(first, second)

            with patch.object(np, 'dot', dot):
                results.append(outcome(function, *values))
            self.assertEqual(count[0], 2)
        self.assertEqual(results[0], results[1])
        self.assertEqual(results[0][0], 'error')

    def test_float64_strided_fortran_and_readonly_arrays(self):
        source = geometry(((-2., -1.), (-1., 0.), (0., 1.), (2., 3.)))
        for layout in ('strided', 'fortran', 'readonly'):
            values = list(source)
            for index in (0, 1, 2):
                array = source[index]
                if layout == 'strided':
                    wide = np.empty((len(array)*2,)+array.shape[1:], dtype=array.dtype)
                    wide[::2] = array
                    values[index] = wide[::2]
                elif layout == 'fortran':
                    values[index] = np.asfortranarray(array)
                else:
                    values[index] = np.frombuffer(array.tobytes(), dtype=array.dtype).reshape(array.shape)
            with self.subTest(layout=layout):
                self.assert_union(values)

    def test_public_float32_and_non_native_endian_conversion_unchanged(self):
        for dtype in (np.float32, np.dtype('>f8')):
            args = self.public_args(1)
            args['segments'] = args['segments'].astype(dtype)
            args['normals'] = args['normals'].astype(dtype)
            self.assert_public_same(None, **args)

    def test_strict_numpy_error_mode_does_not_change_valid_union(self):
        with np.errstate(all='raise'):
            self.assert_union(geometry(((-.5, .25), (.25, 1.)), normal=(.6, .8), origin=(3., -4.)))

    def test_random_graphs_and_input_permutations(self):
        rng = np.random.default_rng(0x1920)
        for case in range(64):
            count = int(rng.integers(1, 16))
            vertices = rng.integers(-16, 17, size=(10, 2)).astype(float)/8.
            edges = rng.integers(0, len(vertices), size=(count, 2))
            normal = ((0., 1.), (.6, .8), (-.8, .6), (0., -1.))[case % 4]
            values = list(geometry(((0., 1.),)*count, normal=normal))
            values[0] = vertices[edges]
            values[1][rng.random((count, 2)) < .2] *= -1.
            values[2] = rng.random(count) < .5
            for order in (np.arange(count), rng.permutation(count)):
                selected = [values[i][order] if i < 3 else values[i] for i in range(6)]
                with self.subTest(case=case, permutation=order.tolist()):
                    self.assert_union(selected)

    @staticmethod
    def public_args(stamp, pairs=((0., 1.), (1., 2.)), flags=None, **overrides):
        segments, normals, reliable, origin, tangent, cosine = geometry(pairs, flags=flags)
        values = dict(origin=origin, current_frame_normal=(-tangent[1], tangent[0]),
            segments=segments, normals=normals, reliable=reliable, stamp_ns=stamp,
            minimum_frame_votes=5, minimum_reliable_votes=1, candidate_ttl_ns=0,
            normal_cosine=cosine, max_intervals=1000)
        values.update(overrides)
        return values

    def assert_public_same(self, previous, **values):
        before = identity((previous, values))
        with patch.object(fs, '_current_union', literal_r19_union):
            expected = outcome(fs.advance_domain, previous, **values)
        actual = outcome(fs.advance_domain, previous, **values)
        self.assertEqual(identity(actual), identity(expected))
        self.assertEqual(identity((previous, values)), before)
        return actual

    def test_five_plus_one_history_state_and_export_bits(self):
        old_state = new_state = None
        for stamp in range(1, 8):
            values = self.public_args(stamp, flags=(stamp == 3, stamp == 3))
            with patch.object(fs, '_current_union', literal_r19_union):
                old_state = fs.advance_domain(old_state, **values)
            new_state = fs.advance_domain(new_state, **values)
            self.assertEqual(identity(new_state), identity(old_state))
            old_export = outcome(fs.certified_segments, old_state, (.3, .01), (0., 1.), .8, 5, 1)
            new_export = outcome(fs.certified_segments, new_state, (.3, .01), (0., 1.), .8, 5, 1)
            self.assertEqual(identity(new_export), identity(old_export))
            self.assertEqual(old_export[0], 'value')
            self.assertEqual(bool(len(old_export[1][0])), stamp >= 5)

    def test_random_causal_histories_with_ttl_and_capacity_errors(self):
        rng = random.Random(0x192017)
        for case in range(8):
            old_state = new_state = None
            for stamp in range(1, 13):
                pairs = [(a/8., (a+rng.randint(1, 6))/8.) for a in
                         (rng.randint(-8, 8) for _ in range(rng.randint(0, 7)))]
                flags = [bool(rng.getrandbits(1)) for _ in pairs]
                values = self.public_args(stamp, pairs=pairs, flags=flags,
                    candidate_ttl_ns=3, max_intervals=8 if case % 2 else 1000)
                with patch.object(fs, '_current_union', literal_r19_union):
                    old = outcome(fs.advance_domain, old_state, **values)
                new = outcome(fs.advance_domain, new_state, **values)
                with self.subTest(case=case, stamp=stamp):
                    self.assertEqual(identity(new), identity(old))
                if old[0] == 'error':
                    break  # A rejected attempt never changes either committed state.
                old_state, new_state = old[1], new[1]
                self.assertEqual(identity(outcome(fs.certified_segments, old_state, (0., 0.), (0., 1.), .8, 5, 1)),
                                 identity(outcome(fs.certified_segments, new_state, (0., 0.), (0., 1.), .8, 5, 1)))

    def test_public_validation_and_capacity_exceptions_stay_exact(self):
        cases = [dict(stamp_ns=0), dict(stamp_ns=True), dict(max_intervals=0),
                 dict(normals=np.zeros((2, 2, 2))), dict(segments=np.full((2, 2, 2), math.nan)),
                 dict(reliable=np.array([1, 1])), dict(normal_cosine=0.),
                 dict(max_intervals=1, segments=geometry(((0., 1.), (2., 3.)))[0])]
        for options in cases:
            with self.subTest(options=options):
                self.assertEqual(self.assert_public_same(None, **self.public_args(1, **options))[0], 'error')

    def test_empty_frame_strict_ttl_and_confirmed_survival(self):
        previous = fs.DomainState((0., 0.), (1., 0.), (
            fs.Interval(0., 1., 4, 1, 1), fs.Interval(2., 3., 5, 1, 1)), 1)
        equal = self.assert_public_same(previous, **self.public_args(4, pairs=(), candidate_ttl_ns=3))[1]
        expired = self.assert_public_same(previous, **self.public_args(5, pairs=(), candidate_ttl_ns=3))[1]
        self.assertEqual(len(equal.intervals), 2)
        self.assertEqual(identity(expired.intervals), identity((previous.intervals[1],)))


if __name__ == '__main__':
    unittest.main()
