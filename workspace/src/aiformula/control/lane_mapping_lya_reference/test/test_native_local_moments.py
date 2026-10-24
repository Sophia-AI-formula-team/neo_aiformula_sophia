"""Independent original-NumPy oracle for exact local PCA sufficient statistics.

Only the finite byte-protocol kernel is tested here. These tests do not change
production gates, import its PCA implementation, or claim a runtime speedup.
"""
from concurrent.futures import ThreadPoolExecutor
from enum import IntEnum
import gc
import os
from pathlib import Path
import sys
import unittest

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
if os.environ.get("LANE_REQUIRE_NATIVE") == "1":
    import _lane_mapping_lya_native as native
else:
    native = pytest.importorskip("_lane_mapping_lya_native")
assert native.LOCAL_MOMENTS_API_VERSION == 1


def numpy_oracle(difference, support):
    """Keep original K, row order, separate operations and reduction axes."""
    weights = support.astype(np.float64)
    count = np.maximum(np.sum(support, axis=1, dtype=np.int64).astype(np.float64), 1.)
    with np.errstate(all="ignore"):
        means = np.sum(difference * weights[:, :, None], axis=1) / count[:, None]
        centered = difference - means[:, None, :]
        covariance = np.einsum("nki,nkj,nk->nij", centered, centered, weights) / count[:, None, None]
    return means, covariance


def literal_oracle(difference, support):
    """Explicit double operations; no NumPy reduction or production helper."""
    rows, columns = support.shape
    means = np.empty((rows, 2), dtype=np.float64)
    covariance = np.empty((rows, 2, 2), dtype=np.float64)
    for row in range(rows):
        count = max(sum(int(x) for x in support[row]), 1)
        for axis in range(2):
            total = 0.0
            for column in range(columns):
                term = float(difference[row, column, axis]) * float(support[row, column])
                total = total + term
            means[row, axis] = total / float(count)
        for i in range(2):
            for j in range(2):
                total = 0.0
                for column in range(columns):
                    left = float(difference[row, column, i]) - float(means[row, i])
                    right = float(difference[row, column, j]) - float(means[row, j])
                    product = left * right
                    product = product * float(support[row, column])
                    total = total + product
                covariance[row, i, j] = total / float(count)
    return means, covariance


class NativeLocalMomentsTests(unittest.TestCase):
    def compare(self, difference, support, check_literal=False):
        difference = np.asarray(difference, dtype=np.float64)
        support = np.asarray(support, dtype=np.bool_)
        self.assertEqual(difference.shape, support.shape + (2,))
        args = (difference.tobytes(), support.tobytes(), int(len(support)), int(support.shape[1]))
        before = tuple(args)
        actual = native.local_moments(*args)
        self.assertIs(type(actual), tuple)
        self.assertEqual(len(actual), 2)
        self.assertTrue(all(type(value) is bytes for value in actual))
        expected = numpy_oracle(difference, support)
        self.assertEqual(actual, tuple(value.tobytes() for value in expected))
        self.assertEqual(len(actual[0]), len(support) * 2 * 8)
        self.assertEqual(len(actual[1]), len(support) * 4 * 8)
        self.assertEqual(args, before)
        if check_literal:
            literal = literal_oracle(difference, support)
            self.assertEqual(actual, tuple(value.tobytes() for value in literal))
        return actual

    def test_known_mean_and_covariance(self):
        actual = self.compare([[[1., 2.], [3., 4.], [9., 11.]]], [[1, 1, 0]], True)
        self.assertEqual(actual[0], np.array([[2., 3.]], dtype=np.float64).tobytes())
        self.assertEqual(actual[1], np.ones((1, 2, 2), dtype=np.float64).tobytes())

    def test_single_candidate(self):
        self.compare([[[1.25, -8.]], [[-0., 0.]]], [[1], [1]], True)

    def test_all_false_support(self):
        rng = np.random.RandomState(771)
        self.compare(rng.uniform(-20., 20., (7, 31, 2)), np.zeros((7, 31), bool), True)

    def test_all_true_support(self):
        rng = np.random.RandomState(772)
        self.compare(rng.normal(size=(11, 129, 2)), np.ones((11, 129), bool), True)

    def test_sparse_support_keeps_original_candidate_order(self):
        rng = np.random.RandomState(773)
        self.compare(rng.normal(size=(9, 257, 2)), rng.uniform(size=(9, 257)) > .91, True)

    def test_signed_zero_means_covariance_and_cross_terms(self):
        values = np.array([[-0., 0.], [0., -0.], [-0., -0.], [0., 0.]], dtype=np.float64)
        for mask in ([1, 1, 1, 1], [0, 0, 0, 0], [1, 0, 1, 0]):
            with self.subTest(mask=mask):
                self.compare(values[None, :, :], np.asarray(mask)[None, :], True)

    def test_exact_coordinate_bounds(self):
        self.compare([[[2e7, -2e7], [-2e7, 2e7], [2e7, 2e7], [-2e7, -2e7]]],
                     [[1, 1, 1, 1]], True)

    def test_cancellation_and_mixed_binary_scales(self):
        values = np.array([[2**24, 2**-40], [2**-28, -2**-40],
                           [-2**24, 2**-40], [-2**-28, 2**-48],
                           [1., -1.], [-1., 1.]], dtype=np.float64)
        self.compare(values[None, :, :], np.ones((1, len(values)), bool), True)

    def test_tiny_normal_and_subnormal_values(self):
        tiny = np.nextafter(0., 1.)
        values = np.array([[tiny, -tiny], [1e-160, -1e-160],
                           [-1e-160, 1e-160], [1e-307, 1e-307]], dtype=np.float64)
        self.compare(values[None, :, :], [[1, 1, 1, 0]], True)

    def test_random_shapes_at_reduction_boundaries(self):
        rng = np.random.RandomState(774)
        for columns in (2, 3, 7, 8, 9, 15, 16, 17, 127, 128, 129, 255, 256, 257, 512):
            for rows in (1, 3, 17):
                with self.subTest(rows=rows, columns=columns):
                    self.compare(rng.normal(size=(rows, columns, 2)),
                                 rng.uniform(size=(rows, columns)) > .4)

    def test_geometric_thin_and_thick_bands(self):
        rng = np.random.RandomState(775)
        for thickness in (.001, .1, .4):
            values = rng.normal(size=(13, 97, 2)) * [1.35, thickness]
            mask = np.sum(values * values, axis=2) <= 1.35 ** 2
            with self.subTest(thickness=thickness):
                self.compare(values, mask, True)

    def test_duplicate_candidates_and_permutations(self):
        rng = np.random.RandomState(776)
        values = np.repeat(rng.normal(size=(4, 31, 2)), 3, axis=1)
        mask = rng.uniform(size=values.shape[:2]) > .3
        for order in (np.arange(93), np.arange(92, -1, -1), rng.permutation(93)):
            self.compare(values[:, order], mask[:, order], True)

    def test_zero_rows_with_nonzero_column_dimension(self):
        self.compare(np.empty((0, 2048, 2)), np.empty((0, 2048), bool), True)

    def test_zero_columns_with_nonzero_row_dimension(self):
        self.compare(np.empty((4000, 0, 2)), np.empty((4000, 0), bool), True)

    def test_both_dimensions_zero(self):
        self.compare(np.empty((0, 0, 2)), np.empty((0, 0), bool), True)

    def test_maximum_rows_with_small_columns(self):
        values = np.arange(4000 * 2 * 2, dtype=np.float64).reshape(4000, 2, 2) / 1024.
        self.compare(values, np.ones((4000, 2), bool))

    def test_maximum_columns_with_small_rows(self):
        rng = np.random.RandomState(777)
        self.compare(rng.normal(size=(2, 2048, 2)), rng.uniform(size=(2, 2048)) > .3, True)

    def test_strided_arrays_serialize_in_logical_c_order(self):
        rng = np.random.RandomState(778)
        original = rng.normal(size=(8, 48, 4))
        values = original[::2, ::-2, ::2]
        mask = (rng.uniform(size=(8, 48)) > .3)[::2, ::-2]
        before = original.tobytes()
        self.assertFalse(values.flags.c_contiguous)
        self.compare(values, mask, True)
        self.assertEqual(original.tobytes(), before)

    def test_inputs_and_outputs_own_immutable_bytes(self):
        args = (np.arange(12, dtype=np.float64).reshape(2, 3, 2).tobytes(), bytes([1, 1, 0, 0, 1, 1]), 2, 3)
        expected = native.local_moments(*args)
        saved = tuple(memoryview(part) for part in expected)
        del args, expected
        gc.collect()
        self.assertTrue(all(part.readonly for part in saved))
        self.assertEqual(tuple(part.tobytes() for part in saved),
                         tuple(x.tobytes() for x in numpy_oracle(np.arange(12, dtype=np.float64).reshape(2, 3, 2),
                             np.array([[1, 1, 0], [0, 1, 1]], dtype=bool))))

    def test_concurrent_calls_share_only_immutable_inputs(self):
        rng = np.random.RandomState(779)
        values = rng.normal(size=(37, 129, 2))
        mask = rng.uniform(size=(37, 129)) > .2
        args = (values.tobytes(), mask.tobytes(), 37, 129)
        expected = tuple(x.tobytes() for x in numpy_oracle(values, mask))
        before = tuple(args)
        with ThreadPoolExecutor(max_workers=4) as pool:
            output = list(pool.map(lambda _: native.local_moments(*args), range(16)))
        self.assertEqual(output, [expected] * 16)
        self.assertEqual(args, before)

    def test_wrong_argument_count(self):
        for args in ((), (b"",), (b"", b"", 0), (b"", b"", 0, 0, 0)):
            with self.subTest(args=args), self.assertRaises(TypeError):
                native.local_moments(*args)

    def test_bytes_are_exact_not_mutable_or_buffer_views(self):
        class ByteSubclass(bytes):
            pass
        for replacement in (None, "", bytearray(), memoryview(b""),
                            memoryview(bytearray()).toreadonly(), np.empty(0, np.uint8), ByteSubclass()):
            for index in (0, 1):
                args = [b"", b"", 0, 0]
                args[index] = replacement
                with self.subTest(type=type(replacement).__name__, index=index), self.assertRaises(TypeError):
                    native.local_moments(*args)

    def test_dimension_types_are_builtin_integers(self):
        class IntegerSubclass(int):
            pass
        class EnumInteger(IntEnum):
            ZERO = 0
        for replacement in (True, False, 0., np.int64(0), "0", None, IntegerSubclass(0), EnumInteger.ZERO):
            for index in (2, 3):
                args = [b"", b"", 0, 0]
                args[index] = replacement
                with self.subTest(type=type(replacement).__name__, index=index), self.assertRaises(TypeError):
                    native.local_moments(*args)

    def test_dimension_ranges_checked_even_if_other_dimension_is_zero(self):
        for rows, columns in ((-1, 0), (0, -1), (4001, 0), (0, 2049), (4001, 2049)):
            with self.subTest(rows=rows, columns=columns), self.assertRaises((ValueError, OverflowError)):
                native.local_moments(b"", b"", rows, columns)

    def test_huge_dimensions_cannot_overflow_buffer_calculation(self):
        for rows, columns in ((2**100, 0), (0, 2**100), (2**63, 2**63), (-2**100, 0)):
            with self.subTest(rows=rows, columns=columns), self.assertRaises((ValueError, OverflowError)):
                native.local_moments(b"", b"", rows, columns)

    def test_difference_buffer_length_is_exact(self):
        good = np.zeros((2, 3, 2), dtype=np.float64).tobytes()
        for bad in (good[:-1], good[:-8], good + b"\0", good + bytes(8)):
            with self.subTest(length=len(bad)), self.assertRaises(ValueError):
                native.local_moments(bad, bytes(6), 2, 3)

    def test_support_buffer_length_is_exact(self):
        values = np.zeros((2, 3, 2), dtype=np.float64).tobytes()
        for bad in (bytes(5), bytes(7), bytes(6 * 8)):
            with self.subTest(length=len(bad)), self.assertRaises(ValueError):
                native.local_moments(values, bad, 2, 3)

    def test_empty_shape_requires_empty_buffers(self):
        for rows, columns in ((0, 0), (0, 2048), (4000, 0)):
            for bad_values, bad_mask in ((b"x", b""), (b"", b"x")):
                with self.subTest(rows=rows, columns=columns), self.assertRaises(ValueError):
                    native.local_moments(bad_values, bad_mask, rows, columns)

    def test_support_values_must_be_zero_or_one(self):
        values = np.zeros((1, 3, 2), dtype=np.float64).tobytes()
        for invalid in (2, 127, 255):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                native.local_moments(values, bytes([1, invalid, 0]), 1, 3)

    def test_nonfinite_coordinates_rejected_even_when_masked_out(self):
        for invalid in (np.nan, np.inf, -np.inf):
            for mask in (0, 1):
                values = np.array([[[0., invalid]]], dtype=np.float64)
                with self.subTest(invalid=invalid, mask=mask), self.assertRaises(ValueError):
                    native.local_moments(values.tobytes(), bytes([mask]), 1, 1)

    def test_just_outside_coordinate_bound_rejected_even_when_masked_out(self):
        for invalid in (np.nextafter(2e7, np.inf), np.nextafter(-2e7, -np.inf)):
            for mask in (0, 1):
                values = np.array([[[invalid, 0.]]], dtype=np.float64)
                with self.subTest(invalid=invalid, mask=mask), self.assertRaises(ValueError):
                    native.local_moments(values.tobytes(), bytes([mask]), 1, 1)

    def test_validation_error_does_not_poison_next_call_or_old_output(self):
        args = (np.array([[[1., 2.], [3., 4.]]]).tobytes(), b"\x01\x01", 1, 2)
        expected = native.local_moments(*args)
        for bad in ((args[0], b"\x01\xff", 1, 2), (args[0][:-1], args[1], 1, 2)):
            with self.assertRaises(ValueError):
                native.local_moments(*bad)
            self.assertEqual(native.local_moments(*args), expected)
        self.assertEqual(expected, tuple(x.tobytes() for x in numpy_oracle(
            np.array([[[1., 2.], [3., 4.]]]), np.ones((1, 2), dtype=bool))))


if __name__ == "__main__":
    unittest.main()
