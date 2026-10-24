"""Independent NumPy oracle for exact grouped native nearest reduction.

CI requires the actual native module; source-only tests may skip this module.
No production index, source history, vehicle truth or replay is imported.
"""
import math
import unittest
from concurrent.futures import ThreadPoolExecutor

import numpy as np


import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
if os.environ.get("LANE_REQUIRE_NATIVE") == "1":
    import _lane_mapping_lya_native as native
else:
    native = pytest.importorskip("_lane_mapping_lya_native")
assert native.NEAREST_API_VERSION == 1


def arguments(query, anchor, groups, allowed=None, limit=1.0):
    """Fixture serialization; runtime dispatch must check native f64/i64 dtype.

    Candidate arrays are already the exact Python _patch result AFTER stable
    truncation and normal_valid filtering. No native step may reorder them.
    A strided array is serialized in logical C order; no input is modified.
    """
    query = np.asarray(query, dtype=np.float64).reshape(-1, 2)
    anchor = np.asarray(anchor, dtype=np.float64).reshape(-1, 2)
    query_offsets, candidate_offsets = [0], [0]
    query_ids, candidate_ids = [], []
    for targets, candidates in groups:
        query_ids.extend(targets)
        candidate_ids.extend(candidates)
        query_offsets.append(len(query_ids))
        candidate_offsets.append(len(candidate_ids))
    mask = None if allowed is None else b"".join(np.asarray(x, dtype=bool).tobytes() for x in allowed)
    return (query.tobytes(), anchor.tobytes(),
            np.asarray(query_offsets, dtype=np.int64).tobytes(),
            np.asarray(query_ids, dtype=np.int64).tobytes(),
            np.asarray(candidate_offsets, dtype=np.int64).tobytes(),
            np.asarray(candidate_ids, dtype=np.int64).tobytes(), mask, float(limit))


def numpy_oracle(query, anchor, groups, allowed=None, limit=1.0):
    query = np.asarray(query, dtype=np.float64).reshape(-1, 2)
    anchor = np.asarray(anchor, dtype=np.float64).reshape(-1, 2)
    result = np.full(len(query), -1, dtype=np.int64)
    for group, (targets, candidates) in enumerate(groups):
        targets = np.asarray(targets, dtype=np.int64)
        candidates = np.asarray(candidates, dtype=np.int64)
        if not len(candidates) or not len(targets):
            continue
        # Exactly the original localizer distance arithmetic. Do not replace
        # this with norm^2 + norm^2 - 2*dot or a candidate sort/KD-tree.
        with np.errstate(over="ignore", under="ignore", invalid="raise"):
            difference = query[targets, None, :] - anchor[candidates][None, :, :]
            distances = np.sum(difference * difference, axis=2)
        if allowed is not None:
            distances[~np.asarray(allowed[group], dtype=bool)] = np.inf
        choices = np.argmin(distances, axis=1)
        selected = candidates[choices].copy()
        selected[~(distances[np.arange(len(targets)), choices] <= limit)] = -1
        result[targets] = selected
    return result


class ExactNearestTests(unittest.TestCase):
    def compare(self, query, anchor, groups, allowed=None, limit=1.0):
        args = arguments(query, anchor, groups, allowed, limit)
        before = tuple(args)
        output = native.nearest_grouped(*args)
        self.assertIs(type(output), bytes)
        expected = numpy_oracle(query, anchor, groups, allowed, limit)
        self.assertEqual(output, expected.tobytes())
        self.assertEqual(args, before)
        return np.frombuffer(output, dtype=np.int64)

    def test_empty_query_and_anchor(self):
        self.compare([], [], [])
        self.compare([], [[0., 0.]], [])

    def test_zero_query_group_preserves_csr_mask_alignment(self):
        self.compare([[0., 0.], [1., 0.]], [[0., 0.], [1., 0.]],
                     [([], [1]), ([1, 0], [0, 1])],
                     [np.empty((0, 1), dtype=bool), np.ones((2, 2), dtype=bool)])

    def test_concurrent_calls_share_only_immutable_inputs(self):
        rng = np.random.RandomState(511)
        queries, anchors = rng.normal(size=(52, 2)), rng.normal(size=(67, 2))
        groups = [(list(range(0, 19)), list(range(66, -1, -1))),
                  (list(range(19, 52)), list(range(3, 51)))]
        allowed = [rng.uniform(size=(len(q), len(c))) > .3 for q, c in groups]
        args = arguments(queries, anchors, groups, allowed, .65 ** 2)
        before = tuple(args)
        expected = numpy_oracle(queries, anchors, groups, allowed, .65 ** 2).tobytes()
        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(lambda _: native.nearest_grouped(*args), range(24)))
        self.assertEqual(results, [expected] * 24)
        self.assertEqual(args, before)

    def test_empty_candidates_return_minus_one_including_infinite_limit(self):
        for limit in (0., 1., np.inf):
            with self.subTest(limit=limit):
                out = self.compare([[0., 0.], [1., 0.]], [], [([1, 0], [])], limit=limit)
                np.testing.assert_array_equal(out, [-1, -1])

    def test_groups_restore_original_query_order_and_empty_group(self):
        out = self.compare([[0., 0.], [5., 0.], [10., 0.]],
                           [[0., 0.], [5., 0.], [10., 0.]],
                           [([2, 0], [2, 0]), ([1], [])])
        np.testing.assert_array_equal(out, [0, -1, 2])

    def test_strict_less_preserves_first_candidate_tie_not_lowest_id(self):
        out = self.compare([[0., 0.]], [[1., 0.], [-1., 0.], [0., 1.]],
                           [([0], [2, 1, 0])], limit=1.)
        self.assertEqual(out.tolist(), [2])

    def test_duplicate_candidates_preserve_position_and_allowed_mask(self):
        out = self.compare([[0., 0.]], [[1., 0.], [-1., 0.]],
                           [([0], [1, 1, 0])], [np.array([[False, True, True]])], 1.)
        self.assertEqual(out.tolist(), [1])

    def test_squared_threshold_nextafter_and_zero(self):
        for distance in (np.nextafter(.65, 0.), .65, np.nextafter(.65, np.inf)):
            for limit in (np.nextafter(.65 ** 2, 0.), .65 ** 2, np.nextafter(.65 ** 2, np.inf), 0.):
                with self.subTest(distance=distance, limit=limit):
                    self.compare([[0., 0.]], [[distance, 0.]], [([0], [0])], limit=limit)

    def test_two_term_square_diagonal_near_threshold(self):
        for x in (.132, np.nextafter(.132, np.inf), np.nextafter(.132, 0.)):
            for y in (.176, np.nextafter(.176, np.inf), np.nextafter(.176, 0.)):
                with self.subTest(x=x, y=y):
                    self.compare([[0., 0.]], [[x, y], [.22, 0.]], [([0], [0, 1])], limit=.22 ** 2)

    def test_signed_zero_and_subnormal_underflow(self):
        tiny = np.nextafter(0., 1.)
        self.compare([[-0., 0.], [tiny, -tiny]],
                     [[0., -0.], [-0., 0.], [tiny, tiny]],
                     [([1, 0], [2, 1, 0])], limit=0.)

    def test_all_incompatible_with_finite_limit(self):
        out = self.compare([[0., 0.]], [[0., 0.], [.1, 0.]],
                           [([0], [1, 0])], [np.array([[False, False]])], .65 ** 2)
        self.assertEqual(out.tolist(), [-1])

    def test_legacy_infinite_limit_all_inf_argmin_chooses_first(self):
        # This is exact old NumPy behavior, not a newly recommended policy:
        # argmin([inf, inf])=0 and inf<=inf admits the first candidate.
        out = self.compare([[0., 0.]], [[0., 0.], [.1, 0.]],
                           [([0], [1, 0])], [np.array([[False, False]])], np.inf)
        self.assertEqual(out.tolist(), [1])

    def test_finite_inputs_overflow_distance_as_numpy_without_new_coordinate_cap(self):
        for limit in (1., np.inf):
            with self.subTest(limit=limit):
                self.compare([[1e308, 1e308]], [[-1e308, -1e308], [0., 0.]],
                             [([0], [1, 0])], limit=limit)
        self.compare([[1e7 + 3., 1e7 + 3.]], [[1e7 + 3., 1e7 + 3.]],
                     [([0], [0])], limit=0.)

    def test_original_per_group_matmul_mask_is_used_unchanged(self):
        rng = np.random.RandomState(1222)
        queries, anchors = rng.normal(size=(21, 2)), rng.normal(size=(25, 2))
        angles = np.array([0., math.pi / 6., np.nextafter(math.pi / 6., 0.),
                           np.nextafter(math.pi / 6., np.inf), math.pi])
        qn = np.column_stack((np.cos(np.resize(angles, 21)), np.sin(np.resize(angles, 21))))
        an = np.column_stack((np.cos(np.resize(angles, 25)), np.sin(np.resize(angles, 25))))
        groups = [(list(range(0, 9)), list(range(20, -1, -1))),
                  (list(range(9, 21)), list(range(3, 25)))]
        allowed = [np.abs(qn[targets] @ an[candidates].T) >= math.cos(math.pi / 6.)
                   for targets, candidates in groups]
        before = [a.tobytes() for a in allowed]
        self.compare(queries, anchors, groups, allowed, .65 ** 2)
        self.assertEqual([a.tobytes() for a in allowed], before)

    def test_random_grouped_masks_and_queries(self):
        for seed in range(12):
            with self.subTest(seed=seed):
                rng = np.random.RandomState(seed + 22000)
                nq, na = 30 + seed, 67 + seed
                queries, anchors = rng.normal(size=(nq, 2)), rng.normal(size=(na, 2))
                targets = rng.permutation(nq)
                groups, allowed = [], []
                for part in np.array_split(targets, 5):
                    candidates = rng.permutation(na)[:int(rng.randint(0, na + 1))]
                    groups.append((part.tolist(), candidates.tolist()))
                    allowed.append(rng.uniform(size=(len(part), len(candidates))) > .4)
                self.compare(queries, anchors, groups, allowed, .65 ** 2)

    def test_none_mask_equals_all_true(self):
        query = np.array([[0., 0.], [1., 1.]])
        anchor = np.array([[0., 0.], [1., 0.]])
        groups = [([1, 0], [1, 0])]
        a = self.compare(query, anchor, groups)
        b = self.compare(query, anchor, groups, [np.ones((2, 2), dtype=bool)])
        self.assertEqual(a.tobytes(), b.tobytes())

    def test_strided_queries_anchors_and_masks_are_logically_serialized(self):
        query_base = np.arange(32, dtype=np.float64).reshape(8, 4)
        anchor_base = np.arange(48, dtype=np.float64).reshape(12, 4)
        queries, anchors = query_base[::-2, ::2], anchor_base[::-2, 1::2]
        before = query_base.tobytes(), anchor_base.tobytes()
        groups = [([3, 1, 0, 2], [5, 2, 0, 4])]
        mask_base = np.tile([True, False, True, True, False, False, True, True], (4, 1))
        masks = [mask_base[:, ::2]]
        self.assertFalse(queries.flags.c_contiguous)
        self.assertFalse(anchors.flags.c_contiguous)
        self.assertFalse(masks[0].flags.c_contiguous)
        self.compare(queries, anchors, groups, masks, 1000.)
        self.assertEqual((query_base.tobytes(), anchor_base.tobytes()), before)

    def test_stable_patch_cutoff_order_is_consumed_not_resorted(self):
        # Model only the already-established stable cutoff; production keeps
        # its Python _patch implementation and arithmetic unchanged.
        anchors = np.tile([[1., 0.], [-1., 0.]], (300, 1))
        collected = np.arange(599, -1, -1, dtype=np.int64)
        distance = np.sum(anchors[collected] ** 2, axis=1)
        candidates = collected[np.argsort(distance, kind="stable")[:512]]
        valid = (candidates % 3) != 0
        candidates = candidates[valid]
        out = self.compare([[0., 0.]], anchors, [([0], candidates.tolist())], limit=1.)
        self.assertEqual(out.tolist(), [int(candidates[0])])

    def test_inclusive_point_and_patch_caps(self):
        anchors = np.column_stack((np.arange(4000, dtype=np.float64), np.zeros(4000)))
        queries = anchors.copy()
        groups = [(list(range(4000)), [3999])]
        self.compare(queries, anchors, groups, limit=0.)
        for count in (512, 2048):
            with self.subTest(candidates=count):
                self.compare(queries[:1], anchors, [([0], list(range(count - 1, -1, -1)))], limit=np.inf)


class InputValidationTests(unittest.TestCase):
    def base(self):
        return list(arguments([[0., 0.], [1., 0.]], [[0., 0.], [1., 0.]],
                              [([0], [0]), ([1], [1])], [np.ones((1, 1), dtype=bool)] * 2))

    def fails(self, args, exception=ValueError):
        with self.assertRaises(exception):
            native.nearest_grouped(*args)

    def test_immutable_exact_bytes_required(self):
        for slot in range(7):
            for constructor in (bytearray, memoryview):
                with self.subTest(slot=slot, constructor=constructor):
                    args = self.base()
                    args[slot] = constructor(args[slot])
                    self.fails(args, TypeError)

    def test_lengths_rejected_before_access(self):
        for slot in range(7):
            with self.subTest(slot=slot):
                args = self.base()
                args[slot] = args[slot] + b"\0"
                self.fails(args)

    def test_squared_limit_requires_builtin_float(self):
        for value in (0, True, np.float64(1.), "1", None):
            with self.subTest(value=value):
                args = self.base()
                args[-1] = value
                self.fails(args, TypeError)
        for value in (-1., -np.inf, np.nan):
            with self.subTest(value=value):
                args = self.base()
                args[-1] = value
                self.fails(args)

    def test_nonfinite_input_coordinates_rejected(self):
        for slot in (0, 1):
            for bad in (np.inf, -np.inf, np.nan):
                with self.subTest(slot=slot, bad=bad):
                    args = self.base()
                    points = np.frombuffer(args[slot], dtype=np.float64).copy()
                    points[1] = bad
                    args[slot] = points.tobytes()
                    self.fails(args)

    def test_offsets_zero_monotonic_end_shape_and_signed_range(self):
        for slot in (2, 4):
            for values in ([1, 1, 2], [0, 2, 1], [0, -1, 2], [0, 1, 3],
                           [0, (1 << 63) - 1, 2], [0, -(1 << 63), 2], [], [0, 2]):
                with self.subTest(slot=slot, values=values):
                    args = self.base()
                    args[slot] = np.asarray(values, dtype=np.int64).tobytes()
                    self.fails(args)

    def test_query_unique_complete_and_candidate_range(self):
        for slot, values in ((3, [0, 0]), (3, [0, -1]), (3, [0, 2]), (3, [0]),
                             (5, [0, -1]), (5, [0, 2]), (5, [0, (1 << 63) - 1])):
            with self.subTest(slot=slot, values=values):
                args = self.base()
                args[slot] = np.asarray(values, dtype=np.int64).tobytes()
                self.fails(args)

    def test_mask_shape_and_binary_values(self):
        for mask in (b"", b"\1", b"\1\1\1", b"\2\1", b"\1\xff"):
            with self.subTest(mask=mask):
                args = self.base()
                args[6] = mask
                self.fails(args)

    def test_point_and_candidate_caps_reject_not_truncate(self):
        large = np.zeros((4001, 2), dtype=np.float64)
        self.fails(arguments(large, [[0., 0.]], [(list(range(4001)), [0])]))
        self.fails(arguments([[0., 0.]], large, [([0], [0])]))
        self.fails(arguments([[0., 0.]], [[0., 0.]], [([0], [0] * 2049)]))

    def test_empty_anchor_with_candidate_rejected(self):
        self.fails(arguments([[0., 0.]], [], [([0], [0])]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
