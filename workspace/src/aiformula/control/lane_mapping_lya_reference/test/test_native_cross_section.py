"""Exact NumPy geometry plus independent BFS and bitset weight oracles.

CI requires the actual native module; source-only tests may skip this module.
This file does not alter the production package global native backend.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import datetime
import struct
import unittest

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


def graph_rows(size, edges):
    rows = [1 << vertex for vertex in range(size)]
    for left, right in edges:
        rows[left] |= 1 << right
        rows[right] |= 1 << left
    return rows


def pack_rows(rows):
    width = (len(rows) + 7) // 8
    return b"".join(value.to_bytes(width, "little") for value in rows)


def numpy_allowed(cloud, normals, ids, half_slab, radius_squared):
    # Exact original independent ufunc expression; no combined dot/einsum/FMA.
    dx = cloud[None, :, 0] - cloud[ids, None, 0]
    dy = cloud[None, :, 1] - cloud[ids, None, 1]
    along = dx * -normals[:, None, 1] + dy * normals[:, None, 0]
    return ((np.abs(along) <= half_slab) &
            (dx * dx + dy * dy <= radius_squared))


def weights_oracle(rows, cloud, normals, ids, half_slab, radius_squared):
    result = np.empty(len(ids), dtype=np.float64)
    for start in range(0, len(ids), 64):
        stop = min(start + 64, len(ids))
        permitted_rows = numpy_allowed(cloud, normals[start:stop], ids[start:stop],
                                       half_slab, radius_squared)
        for offset, permitted in enumerate(permitted_rows):
            seed = int(ids[start + offset])
            seen = {seed} if permitted[seed] else set()
            pending = deque(seen)
            while pending:
                vertex = pending.popleft()
                # Set/queue BFS, not the implementation's bitset traversal.
                for neighbor in np.flatnonzero(permitted):
                    neighbor = int(neighbor)
                    if neighbor not in seen and rows[vertex] & (1 << neighbor):
                        seen.add(neighbor)
                        pending.append(neighbor)
            count = sum(int(vertex) in seen for vertex in ids)
            result[start + offset] = 1.0 / count
    return result


def legacy_bitset_oracle(rows, cloud, normals, ids, half_slab, radius_squared):
    result = np.empty(len(ids), dtype=np.float64)
    for start in range(0, len(ids), 64):
        stop = min(start + 64, len(ids))
        allowed = numpy_allowed(cloud, normals[start:stop], ids[start:stop],
                                half_slab, radius_squared)
        for offset, row in enumerate(np.packbits(allowed, axis=1, bitorder="little")):
            mask = int.from_bytes(row.tobytes(), "little")
            reached = frontier = 1 << int(ids[start + offset])
            while frontier and reached != mask:
                bit = frontier & -frontier
                frontier ^= bit
                added = rows[bit.bit_length() - 1] & mask & ~reached
                reached |= added
                frontier |= added
            count = sum(bool(reached & (1 << int(vertex))) for vertex in ids)
            result[start + offset] = 1.0 / count
    return result


class CrossSectionWeightsTests(unittest.TestCase):
    def invoke(self, rows, cloud, normals, ids, half_slab=0.05, radius_squared=1.35 ** 2,
               capsule=None):
        cloud = np.asarray(cloud, dtype=np.float64).reshape(-1, 2)
        normals = np.asarray(normals, dtype=np.float64).reshape(-1, 2)
        ids = np.asarray(ids, dtype=np.int64)
        if capsule is None:
            capsule = native.prepare_graph(pack_rows(rows), len(rows))
        raw = native.cross_section_weights(capsule, len(rows), cloud.tobytes(),
                                           normals.tobytes(), ids.tobytes(),
                                           half_slab, radius_squared)
        self.assertIs(type(raw), bytes)
        self.assertEqual(len(raw), len(ids) * 8)
        return np.frombuffer(raw, dtype=np.float64)

    def check_oracles(self, rows, cloud, normals, ids, half_slab=0.05,
                      radius_squared=1.35 ** 2):
        cloud = np.asarray(cloud, dtype=np.float64).reshape(-1, 2)
        normals = np.asarray(normals, dtype=np.float64).reshape(-1, 2)
        ids = np.asarray(ids, dtype=np.int64)
        before = (cloud.tobytes(), normals.tobytes(), ids.tobytes())
        with np.errstate(all="ignore"):
            expected = weights_oracle(rows, cloud, normals, ids, half_slab, radius_squared)
            literal = legacy_bitset_oracle(rows, cloud, normals, ids, half_slab, radius_squared)
            actual = self.invoke(rows, cloud, normals, ids, half_slab, radius_squared)
        self.assertEqual(actual.tobytes(), expected.tobytes())
        self.assertEqual(actual.tobytes(), literal.tobytes())
        self.assertEqual(before, (cloud.tobytes(), normals.tobytes(), ids.tobytes()))
        return actual

    def test_api_version_and_binary(self):
        self.assertEqual(native.CROSS_SECTION_WEIGHTS_API_VERSION, 1)
        self.assertTrue(str(native.__file__).endswith((".pyd", ".so")))

    def test_n0_and_empty_queries(self):
        self.assertEqual(self.invoke([], [], [], []).tobytes(), b"")
        self.assertEqual(self.invoke([1], [[0, 0]], [], []).tobytes(), b"")

    def test_seed_center_is_cloud_not_reliable_point_mean(self):
        cloud = [[0, 0], [0.05, 0], [0.0500000001, 0], [0, 0.5]]
        rows = graph_rows(4, [(0, 1), (1, 2), (0, 3)])
        result = self.check_oracles(rows, cloud, [[0, 1]] * 4, [0, 1, 2, 3])
        self.assertEqual(result[0], 1.0 / 3.0)

    def test_u_bridge_outside_tangent_slab_does_not_join_bands(self):
        cloud = [[0, 0], [0, 0.1], [0.2, 0.1], [0.2, 0.3], [0, 0.3], [0, 0.4]]
        rows = graph_rows(6, [(i, i + 1) for i in range(5)])
        result = self.check_oracles(rows, cloud, [[0, 1]] * 4, [0, 1, 4, 5])
        self.assertEqual(result.tobytes(), np.full(4, 0.5).tobytes())

    def test_radius_excluded_bridge_does_not_join_near_nodes(self):
        cloud = [[0, 0], [0, 0.1], [0, 2], [0, 0.3], [0, 0.4]]
        rows = graph_rows(5, [(i, i + 1) for i in range(4)])
        self.check_oracles(rows, cloud, [[0, 1]] * 4, [0, 1, 3, 4], 0.05, 0.5 ** 2)

    def test_duplicate_support_ids_preserve_multiplicity(self):
        cloud = [[0, 0], [0, 0.1], [1, 0]]
        rows = graph_rows(3, [(0, 1), (1, 2)])
        ids = [0, 0, 1, 1, 1, 2]
        result = self.check_oracles(rows, cloud, [[0, 1]] * len(ids), ids)
        self.assertEqual(result.tobytes(), np.array([0.2] * 5 + [1.0]).tobytes())

    def test_normals_need_not_be_unit_or_nonzero(self):
        cloud = [[0, 0], [0.04, 0], [0.08, 0], [0, 0.2]]
        rows = graph_rows(4, [(0, 1), (1, 2), (0, 3)])
        self.check_oracles(rows, cloud, [[0, 0], [0, 2], [0, -3], [4, 5]], [0, 1, 2, 3])

    def test_slab_axis_nextafter_both_signs(self):
        half_slab = 0.05
        values = [0.0, np.nextafter(half_slab, -np.inf), half_slab,
                  np.nextafter(half_slab, np.inf)]
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                cloud = np.column_stack((sign * np.array(values), np.zeros(4)))
                rows = graph_rows(4, [(0, i) for i in range(1, 4)])
                self.check_oracles(rows, cloud, [[0, 1]] * 4, list(range(4)),
                                   half_slab, 4.0)

    def test_radius_axis_nextafter_and_python_squared_threshold(self):
        for radius in (0.22, 0.45, 1.35):
            values = [0, np.nextafter(radius, -np.inf), radius, np.nextafter(radius, np.inf)]
            for sign in (-1, 1):
                cloud = np.column_stack((np.zeros(4), sign * np.array(values)))
                with self.subTest(radius=radius, sign=sign):
                    self.check_oracles(graph_rows(4, [(0, i) for i in range(1, 4)]),
                                       cloud, [[0, 1]] * 4, list(range(4)), 0.05, radius ** 2)

    def test_diagonal_and_cancellation_nextafter_boundaries(self):
        for normal in ((0.6, 0.8), (2 ** -0.5, 2 ** -0.5), (1e10, 1e10)):
            cloud = np.array([[0, 0], [0.3, 0.4], [-0.3, -0.4], [1, 1]], dtype=np.float64)
            normals = np.repeat([normal], 4, axis=0)
            along = float(abs(cloud[1, 0] * -normal[1] + cloud[1, 1] * normal[0]))
            squared = float(cloud[1, 0] ** 2 + cloud[1, 1] ** 2)
            for half_slab in (np.nextafter(along, -np.inf), along, np.nextafter(along, np.inf)):
                if half_slab < 0:
                    continue
                for radius_squared in (np.nextafter(squared, -np.inf), squared,
                                       np.nextafter(squared, np.inf)):
                    self.check_oracles(graph_rows(4, [(0, 1), (0, 2), (0, 3)]),
                                       cloud, normals, [0, 1, 2, 3],
                                       float(half_slab), float(radius_squared))

    def test_signed_zero_underflow_and_infinite_explicit_thresholds(self):
        cloud = [[-0.0, 0.0], [0.0, -0.0], [1e-170, 0], [0, 1e-170]]
        rows = graph_rows(4, [(0, 1), (1, 2), (2, 3)])
        for half_slab, radius_squared in ((-0.0, 0.0), (1e-170, 0.0),
                                           (float("inf"), float("inf"))):
            self.check_oracles(rows, cloud, [[0, -0.0]] * 4, [0, 1, 2, 3],
                               half_slab, radius_squared)

    def test_finite_large_normals_keep_ieee_overflow_comparison(self):
        cloud = [[0, 0], [1e7, 1e7], [-1e7, -1e7], [1e7, -1e7]]
        rows = graph_rows(4, [(0, i) for i in range(1, 4)])
        normals = [[1e308, 1e308], [1e308, -1e308], [0, 1e308], [1e308, 0]]
        for half_slab in (0.05, float("inf")):
            self.check_oracles(rows, cloud, normals, [0, 1, 2, 3],
                               half_slab, float("inf"))

    def test_random_current_graphs_and_queries(self):
        rng = np.random.default_rng(2026101122)
        for size in (1, 7, 9, 63, 64, 65, 129):
            for repeat in range(4):
                cloud = rng.normal(0, 0.5, (size, 2)).astype(np.float64)
                edges = [(i, j) for i in range(size) for j in range(i + 1, size)
                         if np.sum((cloud[i] - cloud[j]) ** 2) <= 0.4 ** 2]
                ids = rng.integers(0, size, 73, dtype=np.int64)
                normals = rng.normal(size=(len(ids), 2))
                with self.subTest(size=size, repeat=repeat):
                    self.check_oracles(graph_rows(size, edges), cloud, normals, ids, 0.1, 1.35 ** 2)

    def test_directed_graph_and_no_self_edges_still_count_seed(self):
        rows = [2, 4, 0]
        self.check_oracles(rows, [[0, 0], [0, 0.1], [0, 0.2]],
                           [[0, 1]] * 3, [0, 1, 2])

    def test_tail_padding_is_ignored_by_prepared_graph(self):
        size = 9
        rows = graph_rows(size, [(i, i + 1) for i in range(size - 1)])
        padding = ((1 << 16) - 1) ^ ((1 << size) - 1)
        capsule = native.prepare_graph(pack_rows([row | padding for row in rows]), size)
        cloud = np.column_stack((np.zeros(size), np.arange(size) * 0.1))
        normals = [[0, 1]] * size
        expected = self.check_oracles(rows, cloud, normals, list(range(size)))
        self.assertEqual(self.invoke(rows, cloud, normals, list(range(size)),
                                     capsule=capsule).tobytes(), expected.tobytes())

    def test_concurrent_prepared_graph_reads_do_not_mutate_inputs(self):
        size = 65
        rows = graph_rows(size, [(i, i + 1) for i in range(size - 1)])
        cloud = np.column_stack((np.zeros(size), np.arange(size) * 0.03))
        capsule = native.prepare_graph(pack_rows(rows), size)
        cases = [(np.repeat([[0.0, scale]], size, axis=0), np.arange(size, dtype=np.int64))
                 for scale in (0.0, 0.5, 1.0, 2.0)] * 6
        expected = [weights_oracle(rows, cloud, normals, ids, 0.05, 1.35 ** 2).tobytes()
                    for normals, ids in cases]
        def run(case):
            return self.invoke(rows, cloud, case[0], case[1], capsule=capsule).tobytes()
        with ThreadPoolExecutor(max_workers=4) as pool:
            self.assertEqual(list(pool.map(run, cases)), expected)

    def test_n4000_q4000_capacity_with_duplicates(self):
        size = 4000
        rows = graph_rows(size, [])
        result = self.invoke(rows, np.zeros((size, 2)), np.zeros((size, 2)),
                             np.zeros(size, dtype=np.int64), 0.0, 0.0)
        self.assertEqual(result.tobytes(), np.full(size, 1.0 / size).tobytes())

    def args(self):
        rows = graph_rows(2, [(0, 1)])
        return [native.prepare_graph(pack_rows(rows), 2), 2,
                np.array([[0, 0], [0, 0.1]], dtype=np.float64).tobytes(),
                np.array([[0, 1], [0, 1]], dtype=np.float64).tobytes(),
                np.array([0, 1], dtype=np.int64).tobytes(), 0.05, 1.35 ** 2]

    def test_bad_graph_and_size_rejected(self):
        args = self.args()
        for value in (b"", None, object(), datetime.datetime_CAPI):
            with self.assertRaises(TypeError):
                native.cross_section_weights(value, *args[1:])
        for value in (True, 2.0, "2"):
            with self.assertRaises(TypeError):
                native.cross_section_weights(args[0], value, *args[2:])
        for value in (-1, 1, 4001, 2 ** 100):
            with self.assertRaises(ValueError):
                native.cross_section_weights(args[0], value, *args[2:])

    def test_only_exact_immutable_bytes_and_exact_shapes(self):
        class BytesSubclass(bytes):
            pass
        for position in (2, 3, 4):
            for convert in (bytearray, memoryview, BytesSubclass):
                args = self.args()
                args[position] = convert(args[position])
                with self.assertRaises(TypeError):
                    native.cross_section_weights(*args)
            args = self.args()
            args[position] += b"\x00"
            with self.assertRaises(ValueError):
                native.cross_section_weights(*args)
        args = self.args()
        args[4] = np.zeros(4001, dtype=np.int64).tobytes()
        with self.assertRaises(ValueError):
            native.cross_section_weights(*args)

    def test_support_id_range_no_silent_clip(self):
        for ids in ((-1, 0), (0, 2), (0, 2 ** 63 - 1), (-(2 ** 63), 0)):
            args = self.args()
            args[4] = struct.pack("=qq", *ids)
            with self.assertRaises(ValueError):
                native.cross_section_weights(*args)

    def test_cloud_bounds_and_nonfinite_inputs_rejected(self):
        for value in (np.nextafter(1e7, np.inf), np.nextafter(-1e7, -np.inf),
                      np.nan, np.inf, -np.inf):
            args = self.args()
            cloud = np.frombuffer(args[2], dtype=np.float64).copy()
            cloud[0] = value
            args[2] = cloud.tobytes()
            with self.assertRaises(ValueError):
                native.cross_section_weights(*args)
        for value in (np.nan, np.inf, -np.inf):
            args = self.args()
            normals = np.frombuffer(args[3], dtype=np.float64).copy()
            normals[0] = value
            args[3] = normals.tobytes()
            with self.assertRaises(ValueError):
                native.cross_section_weights(*args)

    def test_thresholds_are_precomputed_builtin_float_nonnegative(self):
        for position in (5, 6):
            for value in (True, 1, "1", np.float32(1)):
                args = self.args()
                args[position] = value
                with self.assertRaises(TypeError):
                    native.cross_section_weights(*args)
            for value in (-1.0, float("nan"), -float("inf")):
                args = self.args()
                args[position] = value
                with self.assertRaises(ValueError):
                    native.cross_section_weights(*args)
        with self.assertRaises(OverflowError):
            float(1e308) ** 2  # Remains Python's responsibility before native call.


if __name__ == "__main__":
    unittest.main(verbosity=2)
