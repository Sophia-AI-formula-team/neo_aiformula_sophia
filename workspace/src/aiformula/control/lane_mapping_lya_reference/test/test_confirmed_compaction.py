"""Fixed-session confirmation compaction with explicit representation limits.

Scope: finite_support._compact_confirmed_domain, called only after the unchanged
public advance_domain by a consensus owner with frozen confirmation thresholds.
No estimator, recordings, map, Unity, or private snapshot dependency is used.
Capacity acceptance is intentionally NOT included in the equivalence claim:
compaction changes stored interval count, not the configured capacity value.
"""
from dataclasses import fields, is_dataclass
from fractions import Fraction
import math
import random
import struct
import unittest
from unittest.mock import patch

import numpy as np

from lane_mapping_lya_reference import finite_support as fs


COSINE = math.cos(math.pi / 6.)


def bits(value):
    if isinstance(value, np.ndarray):
        return (value.shape, value.dtype.str, value.tobytes(order='C'), value.flags.writeable)
    if is_dataclass(value):
        return tuple((field.name, bits(getattr(value, field.name))) for field in fields(value))
    if isinstance(value, (tuple, list)):
        return tuple(bits(item) for item in value)
    if isinstance(value, (float, np.floating)):
        return struct.pack('>d', float(value))
    return value


def make_state(rows=(), *, stamp=20, origin=(0., 0.), tangent=(1., 0.)):
    return fs.DomainState(origin, tangent, tuple(fs.Interval(*row) for row in rows), stamp)


def compact(value, votes=5, reliable=1):
    # Expected location; no fallback implementation that could hide a missing API.
    return fs._compact_confirmed_domain(value, votes, reliable)


def pending(value, votes, reliable):
    return tuple(piece for piece in value.intervals
                 if piece.frame_votes < votes or piece.reliable_votes < reliable)


def at_exact(value, coordinate):
    return tuple((piece.frame_votes, piece.reliable_votes)
                 for piece in value.intervals
                 if Fraction(piece.lower) < coordinate < Fraction(piece.upper))


def export_outcome(value, point, normal, votes, reliable):
    try:
        arrays = fs.certified_segments(value, point, normal, COSINE, votes, reliable)
        return 'value', bits(arrays)
    except ValueError as error:
        return 'error', type(error), str(error)


def observation(intervals, normal=(0., 1.), origin=(0., 0.), reliability=True):
    tangent = np.array((normal[1], -normal[0]), dtype=np.float64)
    parameters = np.asarray(intervals, dtype=np.float64).reshape(-1, 2)
    segments = np.asarray(origin) + parameters[:, :, None] * tangent
    reliable = (np.full(len(segments), reliability, dtype=bool)
                if isinstance(reliability, bool) else np.asarray(reliability, dtype=bool))
    return dict(origin=origin, current_frame_normal=normal, segments=segments,
                normals=np.tile(normal, (len(segments), 2, 1)), reliable=reliable)


def step(value, intervals=(), *, stamp, votes=5, reliable=1, ttl=0,
         normal=(0., 1.), origin=(0., 0.), reliability=True, capacity=20000):
    return fs.advance_domain(value,
        **observation(intervals, normal=normal, origin=origin, reliability=reliability),
        stamp_ns=stamp, minimum_frame_votes=votes, minimum_reliable_votes=reliable,
        candidate_ttl_ns=ttl, normal_cosine=COSINE, max_intervals=capacity)


class FixedSessionCompactionTests(unittest.TestCase):
    def assert_semantic_identity(self, plain, reduced, votes, reliable):
        self.assertEqual(bits((plain.origin, plain.tangent, plain.source_stamp_ns)),
                         bits((reduced.origin, reduced.tangent, reduced.source_stamp_ns)))
        # Unconfirmed pieces retain EXACT per-location times and partition.
        self.assertEqual(bits(pending(plain, votes, reliable)),
                         bits(pending(reduced, votes, reliable)))
        edges = sorted({edge for state in (plain, reduced) for piece in state.intervals
                        for edge in (piece.lower, piece.upper)})
        for first, second in zip(edges, edges[1:]):
            # Rational midpoint also handles adjacent binary64 endpoints.
            midpoint = (Fraction(first) + Fraction(second)) / 2
            self.assertEqual(at_exact(plain, midpoint), at_exact(reduced, midpoint))

    def assert_export_identity(self, plain, reduced, votes=5, reliable=1):
        normal = (-plain.tangent[1], plain.tangent[0])
        for displacement, angle in ((0., 0.), (.013, .1), (-.021, -.1)):
            point = tuple(plain.origin[i] + displacement * normal[i] for i in (0, 1))
            c, s = math.cos(angle), math.sin(angle)
            rotated = (c*normal[0]-s*normal[1], s*normal[0]+c*normal[1])
            self.assertEqual(export_outcome(plain, point, rotated, votes, reliable),
                             export_outcome(reduced, point, rotated, votes, reliable))

    def test_empty_returns_original_identity(self):
        previous = make_state()
        self.assertIs(compact(previous), previous)

    def test_no_eligible_merge_returns_original_identity(self):
        previous = make_state(((0., 1., 5, 1, 1), (1., 2., 4, 1, 2),
                               (3., 4., 5, 1, 3)))
        self.assertIs(compact(previous), previous)

    def test_two_saturated_neighbors_merge_with_max_frontier(self):
        previous = make_state(((0., 1., 5, 1, 2), (1., 2., 5, 1, 11)))
        before = bits(previous)
        result = compact(previous)
        self.assertIsNot(result, previous)
        self.assertEqual(bits(result.intervals), bits((fs.Interval(0., 2., 5, 1, 11),)))
        self.assertEqual(bits(previous), before)
        self.assert_semantic_identity(previous, result, 5, 1)
        self.assert_export_identity(previous, result)

    def test_chain_merge_is_transitive_and_preserves_state_frontier(self):
        previous = make_state(((0., 1., 5, 1, 17), (1., 2., 5, 1, 3),
                               (2., 3., 5, 1, 11)), stamp=20)
        result = compact(previous)
        self.assertEqual(bits(result.intervals), bits((fs.Interval(0., 3., 5, 1, 17),)))
        self.assertEqual(result.source_stamp_ns, 20)
        self.assert_export_identity(previous, result)

    def test_other_interval_objects_and_order_remain_identical(self):
        previous = make_state(((-2., -1., 2, 0, 1), (0., 1., 5, 1, 2),
                               (1., 2., 5, 1, 3), (3., 4., 4, 1, 4)))
        result = compact(previous)
        self.assertIs(result.intervals[0], previous.intervals[0])
        self.assertIs(result.intervals[-1], previous.intervals[-1])
        self.assertEqual(len(result.intervals), 3)

    def test_real_positive_gap_never_merges(self):
        previous = make_state(((0., 1., 5, 1, 1), (1.001, 2., 5, 1, 2)))
        self.assertIs(compact(previous), previous)

    def test_one_ulp_gap_never_merges(self):
        following = float(np.nextafter(1., math.inf))
        previous = make_state(((0., 1., 5, 1, 1), (following, 2., 5, 1, 2)))
        self.assertIs(compact(previous), previous)

    def test_unconfirmed_gap_is_not_deleted_or_joined(self):
        previous = make_state(((0., 1., 5, 1, 1), (1., 1.25, 5, 0, 2),
                               (1.25, 2., 5, 1, 3)))
        self.assertIs(compact(previous), previous)

    def test_requires_exact_saturation_not_merely_greater_equal(self):
        # Hand-created states can exceed the session cap; do not hide that
        # inconsistency by taking a minimum before public advance validates it.
        for counts in ((6, 1), (5, 2), (4, 1), (5, 0)):
            previous = make_state(((0., 1., counts[0], counts[1], 1),
                                   (1., 2., 5, 1, 2)))
            self.assertIs(compact(previous), previous)

    def test_fixed_nondefault_thresholds(self):
        previous = make_state(((0., 1., 3, 2, 1), (1., 2., 3, 2, 2)))
        result = compact(previous, 3, 2)
        self.assertEqual(bits(result.intervals), bits((fs.Interval(0., 2., 3, 2, 2),)))
        self.assert_semantic_identity(previous, result, 3, 2)
        self.assert_export_identity(previous, result, 3, 2)

    def test_signed_zero_endpoints_and_frozen_chart(self):
        previous = make_state(((-1., -0., 5, 1, 1), (+0., 1., 5, 1, 2)),
                              origin=(-0., .2), tangent=(.6, .8))
        result = compact(previous)
        self.assertEqual(len(result.intervals), 1)
        self.assertEqual(bits(result.origin), bits(previous.origin))
        self.assertEqual(bits(result.tangent), bits(previous.tangent))
        self.assert_export_identity(previous, result)

    def test_idempotent_returns_compacted_object(self):
        previous = make_state(((0., 1., 5, 1, 1), (1., 2., 5, 1, 2)))
        reduced = compact(previous)
        self.assertIs(compact(reduced), reduced)

    def test_new_domain_runs_full_validation_and_does_not_mutate_on_failure(self):
        previous = make_state(((0., 1., 5, 1, 1), (1., 2., 5, 1, 2)))
        before = bits(previous)
        with patch.object(fs.DomainState, '__post_init__',
                          side_effect=ValueError('independent domain validation sentinel')):
            with self.assertRaisesRegex(ValueError, 'independent domain validation sentinel'):
                compact(previous)
        self.assertEqual(bits(previous), before)

    def test_invalid_thresholds_rejected_even_for_empty_state(self):
        previous = make_state()
        for votes, reliable in ((0, 1), (5, 0), (5, 6), (True, 1), (5, False),
                                (5., 1), (5, 1.), (math.inf, 1), (5, math.nan)):
            with self.subTest(votes=votes, reliable=reliable):
                with self.assertRaises(ValueError):
                    compact(previous, votes, reliable)
                self.assertEqual(previous.intervals, ())

    def test_invalid_state_type_rejected(self):
        for previous in (None, (), {}, object()):
            with self.assertRaises(ValueError):
                compact(previous)

    def test_public_advance_remains_original_per_location_stamp_semantics(self):
        previous = make_state(((0., 1., 5, 1, 1), (1., 2., 5, 1, 2)), stamp=2)
        plain = step(previous, stamp=3)
        self.assertEqual(bits(plain.intervals), bits(previous.intervals))
        self.assertEqual(len(plain.intervals), 2)
        self.assertEqual(len(compact(plain).intervals), 1)

    def test_empty_future_observations_keep_confirmed_and_expire_candidates(self):
        previous = make_state(((0., 1., 5, 1, 1), (1., 2., 5, 1, 2),
                               (3., 4., 4, 1, 2), (4., 5., 5, 0, 2)), stamp=2)
        reduced = compact(previous)
        for stamp in (7, 8, 20):
            plain_next = step(previous, stamp=stamp, ttl=5)
            compact_next = compact(step(reduced, stamp=stamp, ttl=5))
            self.assert_semantic_identity(plain_next, compact_next, 5, 1)
            self.assert_export_identity(plain_next, compact_next)
            if stamp == 7:
                self.assertEqual(len(pending(compact_next, 5, 1)), 2)
            else:
                self.assertEqual(pending(compact_next, 5, 1), ())

    def test_partial_reobservation_keeps_export_frontier_and_saturated_counts(self):
        previous = make_state(((0., 1., 5, 1, 1), (1., 2., 5, 1, 2)), stamp=2)
        reduced = compact(previous)
        for stamp, observed in enumerate((((.25, .75),), ((1.25, 1.75),), ()), 3):
            previous = step(previous, observed, stamp=stamp)
            reduced = compact(step(reduced, observed, stamp=stamp))
            self.assert_semantic_identity(previous, reduced, 5, 1)
            self.assert_export_identity(previous, reduced)
            self.assertEqual(len(reduced.intervals), 1)

    def test_new_candidate_extension_preserves_exact_ttl_history(self):
        previous = make_state(((0., 1., 5, 1, 1), (1., 2., 5, 1, 2)), stamp=2)
        reduced = compact(previous)
        for stamp, observed, trusted in ((3, ((1.5, 2.5),), False),
                                        (4, ((2., 3.),), True), (10, (), True)):
            previous = step(previous, observed, stamp=stamp, ttl=5, reliability=trusted)
            reduced = compact(step(reduced, observed, stamp=stamp, ttl=5, reliability=trusted))
            self.assert_semantic_identity(previous, reduced, 5, 1)
            self.assert_export_identity(previous, reduced)

    def test_rejected_advance_never_changes_original_or_compacted_state(self):
        previous = make_state(((0., 1., 5, 1, 1), (1., 2., 5, 1, 2)), stamp=2)
        reduced = compact(previous)
        before = bits((previous, reduced))
        for candidate in (previous, reduced):
            with self.assertRaises(ValueError):
                step(candidate, stamp=2)
        self.assertEqual(bits((previous, reduced)), before)

    def test_random_fixed_threshold_histories_semantic_and_export_identity(self):
        rng = random.Random(0x1817C0)
        successful_exports = 0
        for votes, reliable in ((5, 1), (3, 2), (1, 1)):
            for case in range(4):
                normal = rng.choice(((0., 1.), (0., -1.), (.6, .8), (-.8, .6)))
                origin = rng.choice(((0., 0.), (.2, .01), (123., -57.)))
                previous = reduced = None
                for frame in range(18):
                    pairs = [(-.25, .25)]
                    for _ in range(rng.randint(0, 4)):
                        lower = rng.randint(-12, 8) / 16.
                        pairs.append((lower, lower+rng.randint(1, 8) / 16.))
                    trusted = [frame % 3 != 0] + [bool(rng.getrandbits(1)) for _ in pairs[1:]]
                    arguments = dict(stamp=1+2*frame, votes=votes, reliable=reliable, ttl=8,
                                     normal=normal, origin=origin, reliability=trusted)
                    previous = step(previous, pairs, **arguments)
                    before = bits(reduced)
                    next_reduced = step(reduced, pairs, **arguments)
                    self.assertEqual(bits(reduced), before)
                    reduced = compact(next_reduced, votes, reliable)
                    with self.subTest(votes=votes, reliable=reliable, case=case, frame=frame):
                        self.assert_semantic_identity(previous, reduced, votes, reliable)
                        self.assert_export_identity(previous, reduced, votes, reliable)
                        self.assertEqual(bits(next_reduced.origin), bits(reduced.origin))
                        result = export_outcome(reduced, origin, normal, votes, reliable)
                        successful_exports += (result[0] == 'value'
                                               and result[1][0][0][0] > 0)
        self.assertGreater(successful_exports, 0)

    def test_changing_threshold_is_explicitly_outside_equivalence_contract(self):
        # Counterexample retained as documentation: no test pretends compaction
        # can retain per-location TTL facts for an arbitrary future threshold.
        previous = make_state(((0., 1., 5, 1, 1), (1., 2., 5, 1, 10)), stamp=10)
        reduced = compact(previous)
        raised_plain = step(previous, stamp=12, votes=6, ttl=5)
        raised_reduced = step(reduced, stamp=12, votes=6, ttl=5)
        self.assertEqual(bits(raised_plain.intervals), bits((previous.intervals[1],)))
        self.assertEqual(len(raised_reduced.intervals), 1)
        self.assertEqual(raised_reduced.intervals[0].lower, 0.)
        self.assertNotEqual(bits(raised_plain), bits(raised_reduced))

    def test_storage_count_differs_without_changing_configured_capacity(self):
        previous = make_state(((0., 1., 5, 1, 1), (1., 2., 5, 1, 2)), stamp=2)
        reduced = compact(previous)
        # Same configured cap. Old two-piece representation splits into three
        # timestamp regions; the already-compacted session representation does
        # not need those internal age partitions. This is a disclosed capacity
        # difference, NOT a claim that every previous rejection is preserved.
        with self.assertRaisesRegex(ValueError, 'capacity'):
            step(previous, ((0., .5),), stamp=3, capacity=2)
        candidate = step(reduced, ((0., .5),), stamp=3, capacity=2)
        self.assertLessEqual(len(candidate.intervals), 2)
        final = compact(candidate)
        unconstrained = step(previous, ((0., .5),), stamp=3, capacity=100)
        self.assert_semantic_identity(unconstrained, final, 5, 1)
        self.assert_export_identity(unconstrained, final)


if __name__ == '__main__':
    unittest.main()
