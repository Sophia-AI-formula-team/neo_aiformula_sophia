"""Exact native radius graph, independent NumPy and five-bucket oracles."""
import hashlib
from pathlib import Path
import struct
import sys
import unittest

import numpy as np
import os
import pytest
sys.path.insert(0, str(Path(__file__).parents[1]))
if os.environ.get("LANE_REQUIRE_NATIVE") == "1":
    import _lane_mapping_lya_native as native
else:
    native = pytest.importorskip("_lane_mapping_lya_native")


def square_threshold(radius):
    # Keep exactly the current Python expression, including its rounding and
    # OverflowError. The C API receives this result, not the unsquared radius.
    return radius ** 2


def numpy_pair_oracle(points, keys, radius):
    n = len(points)
    if not n:
        return b""
    difference = points[:, None, :] - points[None, :, :]
    with np.errstate(under="ignore"):
        connected = np.sum(difference * difference, axis=2) <= square_threshold(radius)
    # Python ints avoid overflow when the signed int64 endpoints are compared.
    neighbors = np.array([[abs(int(a[0]) - int(b[0])) <= 1 and
                           abs(int(a[1]) - int(b[1])) <= 1 for b in keys] for a in keys])
    return np.packbits(connected & neighbors, axis=1, bitorder="little").tobytes()


def original_bucket_oracle(points, keys, radius):
    """R5's original five-cell enumeration and NumPy distance expression."""
    buckets = {}
    n = len(points)
    for index, key in enumerate(keys):
        buckets.setdefault(tuple(map(int, key)), []).append(index)
    matrix = np.zeros((n, n), dtype=bool)
    for key, members in buckets.items():
        candidates = list(members)
        for dx, dy in ((-1, -1), (-1, 0), (-1, 1), (0, -1)):
            candidates.extend(buckets.get((key[0] + dx, key[1] + dy), ()))
        members = np.asarray(members, dtype=np.int64)
        candidates = np.asarray(candidates, dtype=np.int64)
        difference = points[members, None, :] - points[candidates][None, :, :]
        with np.errstate(under="ignore"):
            connected = np.sum(difference * difference, axis=2) <= square_threshold(radius)
        matrix[members[:, None], candidates[None, :]] = connected
        matrix[candidates[None, :], members[:, None]] = connected
    return np.packbits(matrix, axis=1, bitorder="little").tobytes()


class RadiusGraphTests(unittest.TestCase):
    def assert_graph(self, points, radius=.22, keys=None):
        points = np.ascontiguousarray(points, dtype=np.float64).reshape(-1, 2)
        keys = np.ascontiguousarray(np.floor(points / radius) if keys is None else keys,
                                    dtype=np.int64).reshape(-1, 2)
        before_points, before_keys = points.tobytes(), keys.tobytes()
        actual = native.radius_graph(before_points, before_keys, len(points), square_threshold(radius))
        self.assertEqual(actual, numpy_pair_oracle(points, keys, radius))
        self.assertEqual(actual, original_bucket_oracle(points, keys, radius))
        self.assertEqual(points.tobytes(), before_points)
        self.assertEqual(keys.tobytes(), before_keys)
        self.assertIs(type(actual), bytes)
        self.assertEqual(len(actual), len(points) * ((len(points) + 7) // 8))
        return actual

    def test_compiled_radius_capability_and_source(self):
        self.assertEqual(native.API_VERSION, 1)
        self.assertEqual(native.RADIUS_GRAPH_API_VERSION, 1)
        digest = hashlib.sha256((Path(__file__).parents[1] / "native" / "lane_graph_native.c").read_bytes()).hexdigest()
        self.assertEqual(native.SOURCE_SHA256, digest)

    def test_empty_graph(self):
        self.assertEqual(native.radius_graph(b"", b"", 0, 0.), b"")

    def test_native_representation_and_signed_zero(self):
        points = struct.pack("=dddd", -0.0, 0.0, 0.0, -0.0)
        keys = struct.pack("=qqqq", 0, 0, 0, 0)
        self.assertEqual(native.radius_graph(points, keys, 2, .22 ** 2), b"\3\3")
        self.assert_graph([[-0.0, 0.0], [0.0, -0.0]])

    def test_axis_nextafter_inside_exact_outside(self):
        for radius in (.22, .1, .45, 1., 1.35):
            distances = [np.nextafter(radius, 0.), radius, np.nextafter(radius, np.inf)]
            for distance in distances:
                for axis in (0, 1):
                    for sign in (-1., 1.):
                        point = [0., 0.]
                        point[axis] = sign * distance
                        with self.subTest(radius=radius, distance=distance, axis=axis, sign=sign):
                            self.assert_graph([[0., 0.], point], radius)

    def test_diagonal_critical_square_and_sum_order(self):
        radius = .22
        for xbase, ybase in ((.132, .176), (radius / np.sqrt(2.), radius / np.sqrt(2.)),
                             (np.nextafter(radius, 0.), np.finfo(float).eps)):
            xs = [np.nextafter(xbase, -np.inf), xbase, np.nextafter(xbase, np.inf)]
            ys = [np.nextafter(ybase, -np.inf), ybase, np.nextafter(ybase, np.inf)]
            for x in xs:
                for y in ys:
                    for sx, sy in ((1., 1.), (-1., 1.), (1., -1.), (-1., -1.)):
                        with self.subTest(x=x, y=y, sx=sx, sy=sy):
                            self.assert_graph([[0., 0.], [sx * x, sy * y]], radius)

    def test_negative_bucket_boundaries_and_input_order(self):
        rng = np.random.default_rng(20261011)
        points = np.array([[x, y] for x in (-.44000000000000006, -.44, -.22, -1e-15, 0., .22, .44)
                           for y in (-.22, -1e-15, 0., .22)])
        for _ in range(10):
            self.assert_graph(points[rng.permutation(len(points))])

    def test_int64_endpoint_adjacency_does_not_overflow(self):
        low, high = -(1 << 63), (1 << 63) - 1
        keys = [(low, low), (low + 1, low), (high, high), (high - 1, high),
                (low, high), (low + 1, high - 1), (high, low), (0, 0), (1, 1), (2, 0)]
        self.assert_graph(np.zeros((len(keys), 2)), keys=keys)

    def test_keys_gate_pairs_even_if_coordinates_coincide(self):
        result = self.assert_graph(np.zeros((3, 2)), keys=[(0, 0), (1, 1), (2, 0)])
        self.assertEqual(result, bytes([3, 7, 6]))

    def test_circle_gate_rejects_far_points_in_adjacent_keys(self):
        result = self.assert_graph([[0., 0.], [1., 0.], [.22, 0.]], keys=[(0, 0)] * 3)
        self.assertEqual(result, bytes([5, 2, 5]))

    def test_random_clouds_all_bits_equal_both_oracles(self):
        rng = np.random.default_rng(610112)
        for n in (1, 2, 7, 8, 9, 31, 63, 64, 65, 129, 256):
            for trial in range(6):
                radius = (.1, .22, .45)[trial % 3]
                points = rng.uniform(-2., 2., size=(n, 2))
                with self.subTest(n=n, trial=trial):
                    self.assert_graph(points, radius)

    def test_dense_and_duplicate_points_keep_all_exact_edges(self):
        rng = np.random.default_rng(613)
        points = rng.uniform(-.01, .01, size=(129, 2))
        points[::2] = points[0]
        result = self.assert_graph(points)
        self.assertEqual(result, ((1 << 129) - 1).to_bytes(17, "little") * 129)

    def test_point_bounds_inclusive(self):
        self.assert_graph([[-1e7, -1e7], [1e7, 1e7], [0., 0.]])

    def test_small_radius_underflow_and_python_overflow_are_preserved(self):
        # Explicit keys avoid imposing an unrelated float-to-int overflow on
        # this public byte-level boundary test.
        for radius in (float(np.nextafter(0., 1.)), 1e-160):
            with self.subTest(radius=radius):
                self.assert_graph([[0., 0.], [1e-160, 0.], [.22, 0.]], radius, keys=[(0, 0)] * 3)
        with self.assertRaises(OverflowError):
            self.assert_graph([[0., 0.]], sys.float_info.max, keys=[(0, 0)])

    def test_explicit_infinite_or_signed_zero_squared_threshold(self):
        points = np.array([[0., 0.], [1., 0.]], dtype=np.float64).tobytes()
        keys = np.zeros((2, 2), dtype=np.int64).tobytes()
        self.assertEqual(native.radius_graph(points, keys, 2, float("inf")), b"\3\3")
        for threshold in (0., -0.):
            self.assertEqual(native.radius_graph(points, keys, 2, threshold), b"\1\2")

    def test_maximum_size_and_padding_are_bounded(self):
        for n in (3999, 4000):
            points = np.zeros((n, 2), dtype=np.float64)
            keys = np.column_stack((np.arange(n, dtype=np.int64) * 3, np.zeros(n, dtype=np.int64)))
            actual = native.radius_graph(points.tobytes(), keys.tobytes(), n, .22 ** 2)
            expected = b"".join((1 << index).to_bytes((n + 7) // 8, "little") for index in range(n))
            self.assertEqual(actual, expected)

    def test_invalid_points_fail_before_native_loop(self):
        for value in (float("nan"), float("inf"), -float("inf"),
                      float(np.nextafter(1e7, np.inf)), float(np.nextafter(-1e7, -np.inf))):
            with self.subTest(value=value), self.assertRaises(ValueError):
                native.radius_graph(struct.pack("=dd", value, 0.), b"\0" * 16, 1, .22 ** 2)

    def test_squared_threshold_type_and_range_are_strict(self):
        for radius in (True, False, 1, "0.22", None):
            with self.subTest(radius=radius), self.assertRaises(TypeError):
                native.radius_graph(b"", b"", 0, radius)
        for radius in (-.22, float("nan"), -float("inf")):
            with self.subTest(radius=radius), self.assertRaises(ValueError):
                native.radius_graph(b"", b"", 0, radius)

    def test_rejects_wrong_types_lengths_and_point_capacity(self):
        class DerivedBytes(bytes):
            pass
        for value in (bytearray(16), memoryview(bytes(16)), [0] * 16, "x", None, DerivedBytes(16)):
            for position in (0, 1):
                args = [b"\0" * 16, b"\0" * 16, 1, .22]
                args[position] = value
                with self.subTest(position=position, value=type(value)), self.assertRaises(TypeError):
                    native.radius_graph(*args)
        for length in (0, 15, 17, 32):
            for position in (0, 1):
                args = [b"\0" * 16, b"\0" * 16, 1, .22]
                args[position] = b"\0" * length
                with self.subTest(length=length, position=position), self.assertRaises(ValueError):
                    native.radius_graph(*args)
        for n in (-1, 4001, 1 << 100):
            with self.subTest(n=n), self.assertRaises(ValueError):
                native.radius_graph(b"", b"", n, .22)
        for n in (True, False, 1., "1", None):
            with self.subTest(n=n), self.assertRaises(TypeError):
                native.radius_graph(b"", b"", n, .22)


if __name__ == "__main__":
    unittest.main(verbosity=2)
