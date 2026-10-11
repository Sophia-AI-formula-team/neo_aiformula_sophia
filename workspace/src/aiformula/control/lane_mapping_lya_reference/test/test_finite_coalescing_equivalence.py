"""R17 coalescing equivalence against literal pre-optimization public functions.

The two old functions below are copied verbatim except their names from source
SHA256 1bca7b32914f48475b6591f4cf76c037cc4845fd5a3be74fe9d1a7caab322225.
They retain the old eager Interval construction, capacity location, validation,
and export merge. Unchanged projection/export primitives are shared deliberately:
this test isolates tuple coalescing, not a second numerical geometry algorithm.
No private diagnostic file, recordings, mapper, timings, or Unity are consumed.
"""
from dataclasses import fields, is_dataclass
import math
import random
import struct
import unittest
from unittest.mock import patch

import numpy as np

from lane_mapping_lya_reference import finite_support as fs
from lane_mapping_lya_reference.finite_support import (
    DomainState, Interval, MAX_ITEMS, _arrays, _cosine, _current_union,
    _empty_export, _export_endpoint, _freeze, _integer, _thresholds, _vector,
)

BEFORE_SOURCE_SHA256 = '1bca7b32914f48475b6591f4cf76c037cc4845fd5a3be74fe9d1a7caab322225'
COSINE = math.cos(math.pi / 6.)


def old_advance_domain(previous, *, origin, current_frame_normal, segments, normals, reliable,
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
        piece = Interval(lower, upper, votes, reliable_votes, last)
        if (result and result[-1].upper == lower and
                (result[-1].frame_votes, result[-1].reliable_votes, result[-1].last_stamp_ns) ==
                (votes, reliable_votes, last)):
            result[-1] = Interval(result[-1].lower, upper, votes, reliable_votes, last)
        else:
            result.append(piece)
            if len(result) > capacity:
                raise ValueError('finite domain interval capacity exceeded; update rejected')
    return DomainState(origin, tangent, tuple(result), stamp)


def old_certified_segments(state, point, current_normal, normal_cosine, minvotes, minreliable):
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
        if confirmed and confirmed[-1].upper == piece.lower:
            old = confirmed[-1]
            confirmed[-1] = Interval(old.lower, piece.upper,
                min(old.frame_votes, piece.frame_votes), min(old.reliable_votes, piece.reliable_votes),
                max(old.last_stamp_ns, piece.last_stamp_ns))
        else:
            confirmed.append(piece)
    for piece in confirmed:
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


def identity(value):
    """Include signed zero, array layout/dtype and all public state fields."""
    if isinstance(value, np.ndarray):
        return (value.dtype.str, value.shape, value.tobytes(order='C'), value.flags.writeable)
    if is_dataclass(value):
        return tuple((item.name, identity(getattr(value, item.name))) for item in fields(value))
    if isinstance(value, dict):
        return tuple((key, identity(item)) for key, item in sorted(value.items()))
    if isinstance(value, (tuple, list)):
        return tuple(identity(item) for item in value)
    if isinstance(value, (float, np.floating)):
        return struct.pack('>d', float(value))
    return value


def outcome(function, *args, **kwargs):
    try:
        return 'value', function(*args, **kwargs)
    except Exception as error:
        return 'error', (type(error), str(error))


def inputs(intervals=(), *, origin=(0., 0.), normal=(0., 1.), stamp=11, **options):
    tangent = np.array((normal[1], -normal[0]))
    pairs = np.asarray(intervals, dtype=np.float64).reshape(-1, 2)
    segments = np.asarray(origin) + pairs[:, :, None] * tangent
    values = dict(origin=origin, current_frame_normal=normal, segments=segments,
        normals=np.tile(normal, (len(pairs), 2, 1)), reliable=np.ones(len(pairs), dtype=bool),
        stamp_ns=stamp, minimum_frame_votes=5, minimum_reliable_votes=1,
        candidate_ttl_ns=0, normal_cosine=COSINE, max_intervals=1000)
    values.update(options)
    return values


def state(rows=(), *, origin=(0., 0.), tangent=(1., 0.), stamp=10):
    return DomainState(origin, tangent, tuple(Interval(*row) for row in rows), stamp)


class CoalescingEquivalenceTests(unittest.TestCase):
    def assert_same(self, old_function, new_function, *args, **kwargs):
        before = identity((args, kwargs))
        expected = outcome(old_function, *args, **kwargs)
        self.assertEqual(identity((args, kwargs)), before, 'old oracle mutated inputs')
        actual = outcome(new_function, *args, **kwargs)
        self.assertEqual(identity((args, kwargs)), before, 'candidate mutated inputs')
        self.assertEqual(identity(actual), identity(expected))
        return actual

    def advance(self, previous=None, intervals=(), **kwargs):
        return self.assert_same(old_advance_domain, fs.advance_domain, previous,
                                **inputs(intervals, **kwargs))

    def export(self, previous, point=(0., 0.), normal=(0., 1.), **kwargs):
        return self.assert_same(old_certified_segments, fs.certified_segments,
            previous, point, normal, kwargs.get('cosine', COSINE),
            kwargs.get('votes', 5), kwargs.get('reliable', 1))

    def overlay(self, previous, current, **kwargs):
        # Isolate boundary coalescing from unchanged projection arithmetic.
        with patch.dict(old_advance_domain.__globals__, {'_current_union': lambda *a: current}):
            with patch.object(fs, '_current_union', return_value=current):
                return self.advance(previous, **kwargs)

    def test_empty_initial_state(self):
        result = self.advance()
        self.assertEqual(result[0], 'value')
        self.assertEqual(result[1].intervals, ())

    def test_empty_observation_preserves_confirmed_state(self):
        previous = state(((0., 1., 5, 1, 3), (2., 3., 5, 1, 4)))
        result = self.advance(previous, stamp=20, candidate_ttl_ns=1)
        self.assertEqual(identity(result[1].intervals), identity(previous.intervals))

    def test_adjacent_equal_metadata_coalesces_exactly(self):
        previous = state(tuple((i / 8., (i+1) / 8., 2, 1, 8) for i in range(8)))
        result = self.advance(previous)
        self.assertEqual(len(result[1].intervals), 1)
        self.assertEqual(identity(result[1].intervals[0]), identity(Interval(0., 1., 2, 1, 8)))

    def test_each_metadata_field_prevents_merge(self):
        for rows in (((0., 1., 2, 1, 8), (1., 2., 3, 1, 8)),
                     ((0., 1., 2, 0, 8), (1., 2., 2, 1, 8)),
                     ((0., 1., 2, 1, 8), (1., 2., 2, 1, 9))):
            with self.subTest(rows=rows):
                self.assertEqual(len(self.advance(state(rows))[1].intervals), 2)

    def test_current_overlay_saturates_before_metadata_merge(self):
        previous = state(((0., 1., 4, 0, 7), (1., 2., 5, 1, 8)))
        result = self.overlay(previous, [(0., 2., True)], max_intervals=2)
        self.assertEqual(identity(result[1].intervals), identity((Interval(0., 2., 5, 1, 11),)))

    def test_reliability_subdomain_is_not_promoted(self):
        result = self.overlay(None, [(0., 1., False), (1., 2., True), (2., 3., False)])
        self.assertEqual(tuple(p.reliable_votes for p in result[1].intervals), (0, 1, 0))

    def test_partial_observation_updates_only_actual_last_stamp(self):
        previous = state(((0., 3., 5, 1, 7),))
        result = self.overlay(previous, [(1., 2., True)])
        self.assertEqual(tuple(p.last_stamp_ns for p in result[1].intervals), (7, 11, 7))

    def test_true_and_one_ulp_gaps_are_not_filled(self):
        upper = float(np.nextafter(1., math.inf))
        for gap in (upper, 1.125):
            previous = state(((0., 1., 5, 1, 8), (gap, 2., 5, 1, 8)))
            self.assertEqual(len(self.advance(previous)[1].intervals), 2)

    def test_signed_zero_bounds_survive_old_ordering(self):
        previous = state(((-1., -0., 2, 1, 8), (+0., 1., 3, 1, 8)))
        self.advance(previous)
        self.overlay(previous, [(-1., 1., True)])

    def test_ttl_equality_and_strict_expiry(self):
        previous = state(((0., 1., 2, 1, 5), (1., 2., 5, 1, 5), (2., 3., 5, 0, 5)), stamp=5)
        equality = self.advance(previous, stamp=15, candidate_ttl_ns=10)[1]
        expired = self.advance(previous, stamp=16, candidate_ttl_ns=10)[1]
        self.assertEqual(len(equality.intervals), 3)
        self.assertEqual(identity(expired.intervals), identity((previous.intervals[1],)))

    def test_ttl_zero_retains_old_candidates(self):
        previous = state(((0., 1., 1, 0, 1),), stamp=1)
        self.assertEqual(len(self.advance(previous, stamp=100000, candidate_ttl_ns=0)[1].intervals), 1)

    def test_dense_duplicate_connected_segments_one_vote(self):
        intervals = tuple((i / 16., (i+1) / 16.) for i in range(16)) * 5
        result = self.advance(None, intervals)
        self.assertEqual(len(result[1].intervals), 1)
        self.assertEqual(result[1].intervals[0].frame_votes, 1)

    def test_interval_capacity_exact_and_exceeded(self):
        current = [(0., 1., True), (2., 3., True), (4., 5., True)]
        self.assertEqual(self.overlay(None, current, max_intervals=3)[0], 'value')
        failure = self.overlay(None, current, max_intervals=2)
        self.assertEqual(failure, ('error', (ValueError,
            'finite domain interval capacity exceeded; update rejected')))

    def test_previous_capacity_rejected_even_if_next_frame_would_merge(self):
        previous = state(((0., 1., 4, 0, 7), (1., 2., 5, 1, 8)))
        failure = self.overlay(previous, [(0., 2., True)], max_intervals=1)
        self.assertEqual(failure, ('error', (ValueError, 'previous state exceeds requested capacity')))

    def test_many_current_fragments_merge_before_capacity_check(self):
        current = [(i / 64., (i+1) / 64., True) for i in range(64)]
        result = self.overlay(None, current, max_intervals=1)
        self.assertEqual(result[0], 'value')
        self.assertEqual(len(result[1].intervals), 1)

    def test_source_stamp_and_saturation_validation_equivalent(self):
        previous = state(((0., 1., 5, 1, 8),))
        for options in (dict(stamp=10), dict(stamp=9), dict(minimum_frame_votes=4),
                        dict(minimum_reliable_votes=0), dict(max_intervals=0)):
            with self.subTest(options=options):
                self.assertEqual(self.advance(previous, **options)[0], 'error')

    def test_malformed_public_input_errors_equivalent(self):
        cases = (dict(origin=(math.nan, 0.)), dict(origin=(True, 0.)),
                 dict(current_frame_normal=(0., 2.)), dict(normal_cosine=0.),
                 dict(stamp_ns=True), dict(candidate_ttl_ns=-1),
                 dict(max_intervals=MAX_ITEMS+1), dict(reliable=np.array([1], dtype=int)),
                 dict(segments=np.zeros((1, 2))), dict(normals=np.zeros((0, 2, 2))),
                 dict(segments=np.array([[[math.inf, 0.], [1., 0.]]])))
        for options in cases:
            values = inputs(((0., 1.),))
            values.update(options)
            with self.subTest(options=options):
                self.assertEqual(self.assert_same(old_advance_domain, fs.advance_domain,
                                                 None, **values)[0], 'error')

    def test_previous_wrong_type_rejection_unchanged(self):
        self.assertEqual(self.advance(object())[0], 'error')

    def test_random_overlay_with_exact_binary_boundaries(self):
        rng = random.Random(0xC0A1E5CE)
        successes = failures = 0
        for case in range(192):
            rows, current = [], []
            for i in range(12):
                lower, upper = (i-6) / 8., (i-5) / 8.
                if rng.random() < .7:
                    votes = rng.randint(1, 5)
                    rows.append((lower, upper, votes, rng.randint(0, 1), rng.randint(1, 10)))
                if rng.random() < .7:
                    current.append((lower, upper, bool(rng.getrandbits(1))))
            previous = state(rows)
            with self.subTest(case=case):
                result = self.overlay(previous, current, stamp=rng.randint(11, 24),
                    candidate_ttl_ns=rng.choice((0, 3, 10)), max_intervals=rng.randint(1, 20))
                successes += result[0] == 'value'
                failures += result[0] == 'error'
        self.assertGreater(successes, 0)
        self.assertGreater(failures, 0)

    def test_random_causal_histories_public_state_bits(self):
        rng = random.Random(0xCA05A1)
        for case in range(12):
            normal = rng.choice(((0., 1.), (0., -1.), (.6, .8), (-.8, .6)))
            origin = rng.choice(((0., 0.), (.2, .01), (123., -57.)))
            old_state = new_state = None
            for frame in range(12):
                pairs = []
                for _ in range(rng.randint(0, 5)):
                    lower = rng.randint(-8, 4) / 8.
                    pairs.append((lower, lower+rng.randint(1, 6) / 8.))
                values = inputs(pairs, origin=origin, normal=normal, stamp=frame+1,
                    reliable=np.asarray([bool(rng.getrandbits(1)) for _ in pairs]),
                    candidate_ttl_ns=3, max_intervals=1000)
                # Empty flags must remain bool, like genuine empty observations.
                values['reliable'] = values['reliable'].astype(bool)
                old_outcome = outcome(old_advance_domain, old_state, **values)
                new_outcome = outcome(fs.advance_domain, new_state, **values)
                with self.subTest(case=case, frame=frame):
                    self.assertEqual(identity(new_outcome), identity(old_outcome))
                    self.assertEqual(new_outcome[0], 'value')
                    old_state, new_state = old_outcome[1], new_outcome[1]
                    self.export(new_state, point=origin, normal=normal)

    def test_empty_and_unconfirmed_export(self):
        self.assertEqual(len(self.export(state())[1][0]), 0)
        self.assertEqual(len(self.export(state(((0., 1., 4, 1, 4),)))[1][0]), 0)

    def test_export_merges_adjacent_confirmed_with_min_votes_max_stamp(self):
        previous = state(((0., .5, 6, 3, 7), (.5, 1., 5, 2, 9),
                          (1., 1.5, 7, 1, 8), (2., 3., 5, 1, 6)))
        result = self.export(previous)
        self.assertEqual(result[0], 'value')
        segments, votes, reliable, stamps = result[1]
        self.assertEqual(len(segments), 2)
        self.assertEqual(votes.tolist(), [5, 5])
        self.assertEqual(reliable.tolist(), [1, 1])
        self.assertEqual(stamps.tolist(), [9, 6])

    def test_unconfirmed_positive_gap_blocks_export_merge(self):
        previous = state(((0., 1., 5, 1, 5), (1., 2., 4, 1, 6), (2., 3., 5, 1, 7)))
        self.assertEqual(len(self.export(previous)[1][0]), 2)

    def test_export_rejects_unrepresentable_isolated_sliver(self):
        previous = state(((.009999999999999997, .010000000000000004, 5, 1, 5),),
                         origin=(0., .01), stamp=5)
        result = self.export(previous, point=(.001666666666666667, .01))
        self.assertEqual(result[0], 'error')
        self.assertIn('cannot be represented', result[1][1])

    def test_export_adjacent_sliver_merges_without_changing_state(self):
        edge = .01
        next_edge = float(np.nextafter(edge, math.inf))
        previous = state(((0., edge, 5, 1, 5), (edge, next_edge, 5, 1, 6),
                          (next_edge, .1, 5, 1, 7)), origin=(.2, .01))
        self.assertEqual(len(self.export(previous, point=(.21, .01))[1][0]), 1)

    def test_export_moved_fused_mean_and_rotated_line_bits(self):
        for tangent in ((1., 0.), (.6, .8), (-.8, .6)):
            previous = state(((-1., -.2, 5, 1, 6), (-.2, .5, 5, 1, 8)),
                             origin=(.2, .01), tangent=tangent)
            angle = math.atan2(tangent[1], tangent[0]) + .1
            normal = (-math.sin(angle), math.cos(angle))
            self.export(previous, point=(.4, -.2), normal=normal)

    def test_export_incompatible_normal_empty(self):
        previous = state(((0., 1., 5, 1, 8),))
        result = self.export(previous, normal=(0., -1.))
        self.assertEqual(result[0], 'value')
        self.assertEqual(len(result[1][0]), 0)

    def test_export_validation_errors_equivalent(self):
        previous = state(((0., 1., 5, 1, 8),))
        for args in ((None, (0., 0.), (0., 1.), COSINE, 5, 1),
                     (previous, (math.nan, 0.), (0., 1.), COSINE, 5, 1),
                     (previous, (0., 0.), (0., 2.), COSINE, 5, 1),
                     (previous, (0., 0.), (0., 1.), -1., 5, 1),
                     (previous, (0., 0.), (0., 1.), COSINE, 0, 1)):
            self.assertEqual(self.assert_same(old_certified_segments, fs.certified_segments,
                                             *args)[0], 'error')

    def test_random_export_metadata_and_geometry_bits(self):
        rng = random.Random(0xE7F017)
        successes = 0
        for case in range(96):
            rows = []
            for i in range(20):
                if rng.random() < .9:
                    votes = rng.randint(3, 7)
                    rows.append(((i-10) / 8., (i-9) / 8., votes,
                                 rng.randint(0, 2), rng.randint(1, 10)))
            tangent = rng.choice(((1., 0.), (.6, .8), (-.8, .6)))
            origin = rng.choice(((0., 0.), (.2, .01), (123., -57.)))
            previous = state(rows, origin=origin, tangent=tangent)
            angle = math.atan2(tangent[1], tangent[0]) + rng.uniform(-.2, .2)
            with self.subTest(case=case):
                result = self.export(previous,
                    point=tuple(origin[i]+rng.uniform(-.1, .1) for i in (0, 1)),
                    normal=(-math.sin(angle), math.cos(angle)))
                successes += result[0] == 'value'
        self.assertGreater(successes, 0)

    def test_export_buffers_remain_immutable(self):
        result = self.export(state(((0., 1., 5, 1, 8),)))
        for array in result[1]:
            self.assertFalse(array.flags.writeable)
            with self.assertRaises(ValueError):
                array.setflags(write=True)


if __name__ == '__main__':
    unittest.main()
