"""Exact affine-image oracles for connected, measured finite support.

Only supplied binary64 coordinates enter the Fraction oracle. It neither calls
production projection_bounds nor reproduces its rounding algorithm. A connected
physical segment graph has a connected affine image, but that fact certifies
only parameter coverage, not a single-valued curve or an accurate line model.
No mapper, recording, truth pose, timing override, or external fixture is used.
"""
from dataclasses import FrozenInstanceError
from fractions import Fraction
import math
import random
import unittest

import numpy as np

from lane_mapping_lya_reference import finite_support as fs


COSINE = math.cos(math.pi / 6.)


def exact(value):
    return Fraction.from_float(float(value))


def project_exact(point, origin, tangent):
    return sum(((exact(p) - exact(o)) * exact(t)
                for p, o, t in zip(point, origin, tangent)), Fraction(0))


def closed_union(intervals):
    """Literal rational closed union; no epsilon or lattice dilation."""
    result = []
    for lower, upper in sorted(intervals):
        if lower == upper:
            continue  # A lone point cannot make a positive support interval.
        if result and lower <= result[-1][1]:
            result[-1] = (result[-1][0], max(upper, result[-1][1]))
        else:
            result.append((lower, upper))
    return result


def accepted_indices(normals, tangent, cosine):
    normal = (-exact(tangent[1]), exact(tangent[0]))
    # Fixtures are separated from the threshold. This is an exact signed gate,
    # not abs(normal dot normal), nor a rounding copy of production np.dot.
    return [i for i, pair in enumerate(normals)
            if all(sum((exact(v) * n for v, n in zip(direction, normal)),
                       Fraction(0)) >= exact(cosine) for direction in pair)]


def segment_union_exact(segments, normals, reliable, origin, tangent,
                        trusted_only=False):
    spans = []
    for index in accepted_indices(normals, tangent, COSINE):
        if trusted_only and not reliable[index]:
            continue
        values = [project_exact(point, origin, tangent)
                  for point in segments[index]]
        spans.append((min(values), max(values)))
    return closed_union(spans)


def physical_components_exact(segments, indices):
    """Independent O(E^2) graph traversal, for small fixtures only.

    A signed zero is the same mathematical point. No rounded chart coordinate
    or tolerance is used to decide physical endpoint identity.
    """
    endpoints = {i: {tuple(exact(v) for v in p) for p in segments[i]}
                 for i in indices}
    pending = set(indices)
    result = []
    while pending:
        component = {min(pending)}
        pending -= component
        changed = True
        while changed:
            changed = False
            vertices = set().union(*(endpoints[i] for i in component))
            additions = {i for i in pending if vertices & endpoints[i]}
            if additions:
                component |= additions
                pending -= additions
                changed = True
        result.append(sorted(component))
    return result


def observation(segments, tangent=(1., 0.), reliable=True, normals=None):
    segments = np.asarray(segments, dtype=np.float64).reshape(-1, 2, 2)
    if normals is None:
        normals = np.tile((-tangent[1], tangent[0]), (len(segments), 2, 1))
    normals = np.asarray(normals, dtype=np.float64).reshape(-1, 2, 2)
    if isinstance(reliable, (bool, np.bool_)):
        reliable = np.full(len(segments), reliable, dtype=bool)
    return segments, normals, np.asarray(reliable, dtype=bool)


def path_segments(points):
    return [[a, b] for a, b in zip(points, points[1:])]


def state_step(previous, segments, *, reliable=True, stamp=None, origin=(0., 0.),
               tangent=(1., 0.), normals=None, ttl=0, capacity=200000):
    segments, normals, reliable = observation(segments, tangent, reliable, normals)
    return fs.advance_domain(previous, origin=origin,
        current_frame_normal=(-tangent[1], tangent[0]), segments=segments,
        normals=normals, reliable=reliable,
        stamp_ns=(1 if previous is None else previous.source_stamp_ns + 1)
        if stamp is None else stamp,
        minimum_frame_votes=5, minimum_reliable_votes=1,
        candidate_ttl_ns=ttl, normal_cosine=COSINE, max_intervals=capacity)


def counters_at(state, coordinate):
    records = [p for p in state.intervals if p.lower <= coordinate <= p.upper]
    # Adjacent endpoint certificates are alternatives, never additive votes.
    return (max((p.frame_votes for p in records), default=0),
            max((p.reliable_votes for p in records), default=0))


class ExactConnectedProjectionTests(unittest.TestCase):
    def assert_safe_union(self, segments, *, origin=(0., 0.), tangent=(1., 0.),
                          reliable=True, normals=None):
        arrays = observation(segments, tangent, reliable, normals)
        before = tuple((a.shape, a.dtype.str, a.tobytes()) for a in arrays)
        result = fs._current_union(*arrays, origin, tangent, COSINE)
        self.assertEqual(before, tuple((a.shape, a.dtype.str, a.tobytes()) for a in arrays))
        base = segment_union_exact(*arrays, origin, tangent)
        trusted = segment_union_exact(*arrays, origin, tangent, trusted_only=True)
        last = None
        for lower, upper, is_trusted in result:
            self.assertTrue(math.isfinite(lower) and math.isfinite(upper))
            self.assertLess(lower, upper)
            if last is not None:
                self.assertLessEqual(last, lower)
            last = upper
            a, b = exact(lower), exact(upper)
            self.assertTrue(any(lo <= a and b <= hi for lo, hi in base),
                            (result, base, 'invented base coverage'))
            if is_trusted:
                self.assertTrue(any(lo <= a and b <= hi for lo, hi in trusted),
                                (result, trusted, 'invented trusted coverage'))
        # Prove the component-span premise independently of floating arithmetic.
        for only_trusted in (False, True):
            indices = accepted_indices(arrays[1], tangent, COSINE)
            if only_trusted:
                indices = [i for i in indices if arrays[2][i]]
            for component in physical_components_exact(arrays[0], indices):
                ranges = []
                for i in component:
                    values = [project_exact(p, origin, tangent) for p in arrays[0][i]]
                    ranges.append((min(values), max(values)))
                united = closed_union(ranges)
                if united:
                    self.assertEqual(len(united), 1)
                    self.assertEqual(united[0], (min(p[0] for p in ranges),
                                                max(p[1] for p in ranges)))
        return result

    def test_exact_joint_is_one_domain_not_two_inward_rounded_segments(self):
        points = ((.1, .4), (.2, .4), (.3, .4))
        result = self.assert_safe_union(path_segments(points), origin=(.1, .4))
        self.assertEqual(len(result), 1)
        joint = project_exact(points[1], (.1, .4), (1., 0.))
        self.assertLess(exact(result[0][0]), joint)
        self.assertGreater(exact(result[0][1]), joint)
        self.assertTrue(result[0][2])

    def test_reverse_chain_and_reverse_each_segment_are_continuous(self):
        segments = path_segments(((.1, 0.), (.2, 0.), (.4, 0.), (.8, 0.)))
        expected = self.assert_safe_union(segments)
        self.assertEqual(len(expected), 1)
        self.assertEqual(self.assert_safe_union(segments[::-1]), expected)
        self.assertEqual(self.assert_safe_union([s[::-1] for s in segments]), expected)

    def test_reordered_duplicates_have_identical_coverage(self):
        segments = path_segments(((.1, .3), (.2, .4), (.4, .5), (.7, .8)))
        expected = self.assert_safe_union(segments, tangent=(.6, .8))
        many = [segments[i] for i in (2, 0, 1, 0, 2, 1, 1)]
        self.assertEqual(self.assert_safe_union(many, tangent=(.6, .8)), expected)

    def test_backtracking_chain_affine_image_has_no_internal_gaps(self):
        result = self.assert_safe_union(path_segments(
            ((.1, 0.), (.8, .1), (.2, .3), (.6, .4), (.15, .5))))
        self.assertEqual(len(result), 1)
        self.assertLess(result[0][0], .15)
        self.assertGreater(result[0][1], .7)

    def test_loop_affine_image_is_connected(self):
        result = self.assert_safe_union(path_segments(
            ((.1, .1), (.8, .1), (.8, .8), (.1, .8), (.1, .1))),
            tangent=(.6, .8))
        self.assertEqual(len(result), 1)

    def test_branch_affine_image_is_connected_without_single_value_claim(self):
        center = (.4, .2)
        segments = [[center, p] for p in ((.1, .8), (.9, .1), (.3, -.4))]
        result = self.assert_safe_union(segments)
        self.assertEqual(len(result), 1)
        self.assertLess(result[0][0], .2)
        self.assertGreater(result[0][1], .8)

    def test_all_zero_extent_and_constant_projection_do_not_make_support(self):
        self.assertEqual(self.assert_safe_union([[(1., 2.), (1., 2.)]]), [])
        self.assertEqual(self.assert_safe_union(
            path_segments(((1., 0.), (1., 1.), (1., 2.)))), [])

    def test_zero_extent_connector_does_not_break_real_chain(self):
        segments = [[(.1, 0.), (.2, 0.)], [(.2, 0.), (.2, 0.)],
                    [(.2, 0.), (.3, 0.)]]
        self.assertEqual(len(self.assert_safe_union(segments)), 1)

    def test_signed_zero_is_exact_same_physical_endpoint(self):
        result = self.assert_safe_union([[(-.3, 0.), (0., -0.)],
                                         [(-0., 0.), (.3, 0.)]])
        self.assertEqual(len(result), 1)
        self.assertLess(result[0][0], 0.)
        self.assertGreater(result[0][1], 0.)

    def test_large_coordinate_subtraction_and_cancellation_are_contained(self):
        origin = (8000000., -8000000.)
        points = [(origin[0] + x, origin[1] + y)
                  for x, y in ((.125, -.25), (.5, -.375), (.75, -.125), (1., .125))]
        result = self.assert_safe_union(path_segments(points), origin=origin,
                                        tangent=(.6, .8))
        self.assertEqual(len(result), 1)
        joint = project_exact(points[1], origin, (.6, .8))
        self.assertLess(exact(result[0][0]), joint)
        self.assertGreater(exact(result[0][1]), joint)

    def test_one_ulp_real_gap_must_not_be_filled(self):
        gap_right = float(np.nextafter(np.float64(1.), np.float64(math.inf)))
        result = self.assert_safe_union([[(0., 0.), (1., 0.)],
                                         [(gap_right, 0.), (2., 0.)]])
        self.assertEqual(len(result), 2)
        midpoint = (exact(1.) + exact(gap_right)) / 2
        self.assertFalse(any(exact(a) <= midpoint <= exact(b) for a, b, _ in result))

    def test_close_but_distinct_2d_endpoints_do_not_form_a_component(self):
        # The projected gap is real even though the physical points differ by
        # only one ULP in each coordinate. No near-neighbor merge is legitimate.
        u = float(np.nextafter(np.float64(1.), np.float64(math.inf)))
        result = self.assert_safe_union([[(0., 0.), (1., 1.)], [(u, u), (2., 2.)]],
                                        tangent=(.6, .8))
        self.assertEqual(len(result), 2)

    def test_different_physical_components_can_have_overlapping_projected_union(self):
        segments = [[(0., 0.), (2., 0.)], [(1., 3.), (3., 3.)]]
        self.assertEqual(len(physical_components_exact(np.asarray(segments), [0, 1])), 2)
        result = self.assert_safe_union(segments)
        self.assertEqual(len(result), 1)
        self.assertLess(result[0][0], 1.)
        self.assertGreater(result[0][1], 2.)

    def test_signed_normal_rejected_bridge_cannot_connect_base_or_trusted(self):
        segments = path_segments(((0., 0.), (1., 0.), (2., 0.), (3., 0.)))
        normals = np.tile((0., 1.), (3, 2, 1))
        normals[1, 1] = (0., -1.)
        result = self.assert_safe_union(segments, normals=normals)
        self.assertEqual(len(result), 2)
        self.assertFalse(any(a <= 1.5 <= b for a, b, _ in result))

    def test_unreliable_bridge_connects_base_but_never_trusted_domains(self):
        segments = path_segments(((0., 0.), (1., 0.), (2., 0.), (3., 0.)))
        result = self.assert_safe_union(segments, reliable=[True, False, True])
        base = closed_union([(exact(a), exact(b)) for a, b, _ in result])
        trusted = closed_union([(exact(a), exact(b)) for a, b, r in result if r])
        self.assertEqual(len(base), 1)
        self.assertEqual(len(trusted), 2)
        self.assertFalse(any(a <= exact(1.5) <= b for a, b in trusted))
        self.assertTrue(any(a < 1.5 < b and not r for a, b, r in result))

    def test_duplicate_trusted_piece_does_not_certify_other_unreliable_tail(self):
        segments = [[(0., 0.), (1., 0.)], [(1., 0.), (3., 0.)]]
        result = self.assert_safe_union(segments + [segments[0]] * 9,
                                       reliable=[True, False] + [True] * 9)
        self.assertTrue(any(a < .5 < b and r for a, b, r in result))
        self.assertTrue(any(a < 2. < b and not r for a, b, r in result))

    def test_empty_has_no_coverage(self):
        self.assertEqual(self.assert_safe_union([]), [])

    def test_bounded_random_graphs_are_exact_subsets_and_permutation_invariant(self):
        rng = random.Random(20261011)
        for trial in range(48):
            tangent = ((1., 0.), (0., 1.), (.6, .8))[trial % 3]
            origin = (rng.randint(-30, 30) / 8., rng.randint(-30, 30) / 8.)
            vertices = [(rng.randint(-60, 60) / 8., rng.randint(-60, 60) / 8.)
                        for _ in range(7)]
            pairs = [(rng.randrange(7), rng.randrange(7)) for _ in range(9)]
            segments = [[vertices[a], vertices[b]] for a, b in pairs]
            reliable = [bool(rng.getrandbits(1)) for _ in pairs]
            with self.subTest(trial=trial):
                result = self.assert_safe_union(segments, origin=origin,
                    tangent=tangent, reliable=reliable)
                order = list(range(len(segments)))
                rng.shuffle(order)
                alternate = self.assert_safe_union([segments[i][::-1] for i in order],
                    origin=origin, tangent=tangent, reliable=[reliable[i] for i in order])
                self.assertEqual(alternate, result)


class ConnectedDomainTransactionTests(unittest.TestCase):
    CHAIN = (((.1, 0.), (.2, 0.)), ((.2, 0.), (.3, 0.)))

    def test_five_distinct_frames_confirm_joint_without_extra_vertex_votes(self):
        state = None
        for index in range(1, 6):
            state = state_step(state, self.CHAIN, reliable=index == 3)
            self.assertEqual(len(state.intervals), 1)
            self.assertEqual(counters_at(state, .2), (index, int(index >= 3)))
            if index < 5:
                self.assertEqual(len(fs.certified_segments(
                    state, (0., 0.), (0., 1.), COSINE, 5, 1)[0]), 0)
        exported = fs.certified_segments(state, (0., 0.), (0., 1.), COSINE, 5, 1)
        self.assertEqual(exported[0].shape, (1, 2, 2))
        self.assertLess(exported[0][0, 0, 0], .2)
        self.assertGreater(exported[0][0, 1, 0], .2)
        np.testing.assert_array_equal(exported[1], [5])
        np.testing.assert_array_equal(exported[2], [1])

    def test_many_duplicates_and_backtracking_still_one_vote_per_frame(self):
        segments = list(self.CHAIN) * 30 + [s[::-1] for s in self.CHAIN] * 30
        state = state_step(None, segments)
        self.assertEqual(len(state.intervals), 1)
        self.assertEqual(counters_at(state, .2), (1, 1))
        state = state_step(state, segments)
        self.assertEqual(counters_at(state, .2), (2, 1))

    def test_unreliable_bridge_does_not_earn_reliable_votes_across_five_frames(self):
        segments = path_segments(((0., 0.), (1., 0.), (2., 0.), (3., 0.)))
        state = None
        for _ in range(5):
            state = state_step(state, segments, reliable=[True, False, True])
        self.assertEqual(counters_at(state, .5), (5, 1))
        self.assertEqual(counters_at(state, 1.5), (5, 0))
        self.assertEqual(counters_at(state, 2.5), (5, 1))
        exported = fs.certified_segments(state, (0., 0.), (0., 1.), COSINE, 5, 1)[0]
        self.assertEqual(len(exported), 2)
        self.assertFalse(any(min(s[:, 0]) <= 1.5 <= max(s[:, 0]) for s in exported))

    def test_true_gap_stays_unobserved_after_confirmation(self):
        segments = [[(0., 0.), (1., 0.)], [(2., 0.), (3., 0.)]]
        state = None
        for _ in range(7):
            state = state_step(state, segments)
        self.assertEqual(counters_at(state, 1.5), (0, 0))
        self.assertEqual(counters_at(state, .5), (5, 1))
        self.assertEqual(len(fs.certified_segments(
            state, (0., 0.), (0., 1.), COSINE, 5, 1)[0]), 2)

    def test_ttl_drops_only_unconfirmed_connected_support(self):
        state = None
        for stamp in range(1, 6):
            state = state_step(state, self.CHAIN, stamp=stamp)
        state = state_step(state, [[(2., 0.), (3., 0.)]], stamp=6)
        old = state
        at_boundary = state_step(state, [], stamp=16, ttl=10)
        self.assertEqual(counters_at(at_boundary, 2.5), (1, 1))
        expired = state_step(state, [], stamp=17, ttl=10)
        self.assertEqual(counters_at(expired, 2.5), (0, 0))
        self.assertEqual(counters_at(expired, .2), (5, 1))
        self.assertEqual(counters_at(old, 2.5), (1, 1))
        self.assertEqual(old.source_stamp_ns, 6)

    def test_same_or_past_stamp_rejection_leaves_original_state_unchanged(self):
        state = state_step(None, self.CHAIN, stamp=10)
        before = (state.origin, state.tangent, state.intervals, state.source_stamp_ns)
        for stamp in (9, 10):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                state_step(state, [[(5., 0.), (6., 0.)]], stamp=stamp)
            self.assertEqual((state.origin, state.tangent, state.intervals,
                              state.source_stamp_ns), before)

    def test_capacity_rejection_is_transactional_after_connected_projection(self):
        state = state_step(None, self.CHAIN, stamp=1, capacity=1)
        before = (state.origin, state.tangent, state.intervals, state.source_stamp_ns)
        with self.assertRaises(ValueError):
            state_step(state, [[(2., 0.), (3., 0.)], [(4., 0.), (5., 0.)]],
                       stamp=2, capacity=1)
        self.assertEqual((state.origin, state.tangent, state.intervals,
                          state.source_stamp_ns), before)
        self.assertEqual(counters_at(state, .2), (1, 1))
        with self.assertRaises(FrozenInstanceError):
            state.source_stamp_ns = 2

    def test_input_arrays_are_not_mutated_when_capacity_rejects(self):
        segments, normals, reliable = observation(
            [[(0., 0.), (1., 0.)], [(2., 0.), (3., 0.)]])
        before = tuple(a.tobytes() for a in (segments, normals, reliable))
        with self.assertRaises(ValueError):
            state_step(None, segments, normals=normals, reliable=reliable, capacity=1)
        self.assertEqual(tuple(a.tobytes() for a in (segments, normals, reliable)), before)


if __name__ == '__main__':
    unittest.main()
