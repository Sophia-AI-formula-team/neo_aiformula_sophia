"""Independent BFS/original-NumPy oracle for grouped local geometry.

No production index, vehicle truth, or relaxed geometry gate is imported.
The byte protocol is mandatory in native CI and explicitly skipped if unbuilt.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import gc
import hashlib
import os
from pathlib import Path
import struct
import sys
import unittest

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
if os.environ.get("LANE_REQUIRE_NATIVE") == "1":
    import _lane_mapping_lya_native as native
else:
    native = pytest.importorskip("_lane_mapping_lya_native")
assert native.LOCAL_GEOMETRY_API_VERSION == 1


def i64(values):
    values = tuple(values)
    return struct.pack("=" + "q" * len(values), *values)


def graph_rows(size, edges):
    rows = [1 << index for index in range(size)]
    for left, right in edges:
        rows[left] |= 1 << right
        rows[right] |= 1 << left
    return rows


def packed_graph(rows):
    width = (len(rows) + 7) // 8
    return b"".join(row.to_bytes(width, "little") for row in rows)


def independent_forest(rows):
    parents = [-1] * len(rows)
    seen = set()
    for seed in range(len(rows)):
        if seed in seen:
            continue
        seen.add(seed)
        pending = deque([seed])
        while pending:
            vertex = pending.popleft()
            for neighbor in range(len(rows)):
                if neighbor not in seen and rows[vertex] & (1 << neighbor):
                    seen.add(neighbor)
                    parents[neighbor] = vertex
                    pending.append(neighbor)
    return parents


def make_args(rows, points, groups, radius_squared=1., minimum_points=0,
              labels=None, parents=None, capsule=None):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if labels is None:
        labels = [0] * len(rows)
    if parents is None:
        parents = independent_forest(rows)
    if capsule is None:
        capsule = native.prepare_graph(packed_graph(rows), len(rows))
    query_offsets, candidate_offsets = [0], [0]
    query_ids, candidate_ids = [], []
    for queries, candidates in groups:
        query_ids.extend(queries)
        candidate_ids.extend(candidates)
        query_offsets.append(len(query_ids))
        candidate_offsets.append(len(candidate_ids))
    return (capsule, len(rows), points.tobytes(), i64(labels), i64(parents),
            i64(query_offsets), i64(query_ids), i64(candidate_offsets),
            i64(candidate_ids), radius_squared, minimum_points)


def numpy_bfs_oracle(rows, points, groups, radius_squared=1., minimum_points=0, labels=None):
    """No forest certificate: independently traverse the exact induced graph."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    labels = np.asarray([0] * len(rows) if labels is None else labels, dtype=np.int64)
    counts_parts, mean_parts, covariance_parts = [], [], []
    for queries, candidates in groups:
        queries = np.asarray(queries, dtype=np.int64)
        candidates = np.asarray(candidates, dtype=np.int64)
        # These are the original per-cell expressions and candidate order.
        difference = points[candidates][None, :, :] - points[queries][:, None, :]
        with np.errstate(all="ignore"):
            allowed = ((np.sum(difference * difference, axis=2) <= radius_squared)
                       & (labels[queries, None] == labels[candidates][None, :]))
        support = np.zeros(allowed.shape, dtype=bool)
        for row, seed in enumerate(queries):
            permitted = {int(vertex) for vertex, selected in zip(candidates, allowed[row]) if selected}
            reached = set()
            if int(seed) in permitted:
                reached.add(int(seed))
                pending = deque([int(seed)])
                while pending:
                    vertex = pending.popleft()
                    for neighbor in permitted - reached:
                        if rows[vertex] & (1 << neighbor):
                            reached.add(neighbor)
                            pending.append(neighbor)
            support[row] = [int(vertex) in reached for vertex in candidates]
        count = np.sum(support, axis=1, dtype=np.int64)
        weights = support.astype(np.float64)
        denominator = np.maximum(count.astype(np.float64), 1.)
        with np.errstate(all="ignore"):
            means = np.sum(difference * weights[:, :, None], axis=1) / denominator[:, None]
            centered = difference - means[:, None, :]
            covariance = np.einsum("nki,nkj,nk->nij", centered, centered, weights) / denominator[:, None, None]
        means[count < minimum_points] = 0.
        covariance[count < minimum_points] = 0.
        counts_parts.append(count)
        mean_parts.append(means)
        covariance_parts.append(covariance)
    if not groups:
        return b"", b"", b""
    return (np.concatenate(counts_parts).tobytes(), np.concatenate(mean_parts).tobytes(),
            np.concatenate(covariance_parts).tobytes())


class NativeLocalGeometryTests(unittest.TestCase):
    def compare(self, rows, points, groups, radius_squared=1., minimum_points=0,
                labels=None, parents=None):
        args = make_args(rows, points, groups, radius_squared, minimum_points, labels, parents)
        before = tuple(args)
        actual = native.local_geometry(*args)
        expected = numpy_bfs_oracle(rows, points, groups, radius_squared, minimum_points, labels)
        self.assertIs(type(actual), tuple)
        self.assertEqual(len(actual), 3)
        self.assertTrue(all(type(part) is bytes for part in actual))
        self.assertEqual(actual, expected)
        self.assertEqual(args, before)
        count = sum(len(group[0]) for group in groups)
        self.assertEqual(tuple(len(part) for part in actual), (count * 8, count * 16, count * 32))
        return actual

    def default_args(self):
        rows = graph_rows(4, [(0, 1), (1, 2)])
        return list(make_args(rows, [[0., 0.], [.2, 0.], [.4, .1], [1., 1.]],
                              [([0, 2], [0, 1, 2]), ([3], [3, 2])]))

    def test_source_hash_and_api_version(self):
        source = Path(__file__).parents[1] / "native" / "lane_graph_native.c"
        self.assertEqual(native.SOURCE_SHA256, hashlib.sha256(source.read_bytes()).hexdigest())
        self.assertEqual(native.LOCAL_GEOMETRY_API_VERSION, 1)

    def test_known_candidate_minus_target_mean(self):
        rows = graph_rows(3, [(0, 1), (1, 2)])
        result = self.compare(rows, [[2., 3.], [3., 5.], [1., 1.]], [([0], [0, 1])], 5., 2)
        self.assertEqual(result[0], i64([2]))
        self.assertEqual(result[1], np.array([[.5, 1.]], dtype=np.float64).tobytes())
        self.assertEqual(result[2], np.array([[[.25, .5], [.5, 1.]]], dtype=np.float64).tobytes())

    def test_output_follows_query_stream_not_vertex_order(self):
        rows = graph_rows(5, [(0, 1), (1, 2), (2, 3)])
        self.compare(rows, np.arange(10, dtype=np.float64).reshape(5, 2) / 10.,
                     [([4, 2], [4, 3, 2]), ([0, 3, 1], [0, 1, 3, 2])], 4.)

    def test_minimum_gate_keeps_real_counts_and_zeroes_only_rejected_moments(self):
        rows = graph_rows(4, [(0, 1), (1, 2)])
        points = [[0., 0.], [.1, 0.], [.2, .1], [.3, .1]]
        for minimum in (0, 1, 2, 3, 4, 2048):
            with self.subTest(minimum=minimum):
                result = self.compare(rows, points, [([0, 3], [0, 1, 2, 3])], 1., minimum)
                self.assertEqual(result[0], i64([3, 1]))

    def test_missing_seed_does_not_use_connected_other_candidates(self):
        rows = graph_rows(5, [(0, 1), (1, 2), (2, 3), (3, 4)])
        result = self.compare(rows, np.zeros((5, 2)), [([0, 1, 2], [2, 3, 4])], 1., 2)
        self.assertEqual(result[0], i64([0, 0, 3]))
        self.assertEqual(result[1][:32], bytes(32))
        self.assertEqual(result[2][:64], bytes(64))

    def test_equal_component_size_is_not_a_connectivity_certificate(self):
        rows = graph_rows(4, [(0, 1), (2, 3)])
        result = self.compare(rows, np.zeros((4, 2)), [([0], [0, 2])])
        self.assertEqual(result[0], i64([1]))

    def test_remote_u_bridge_outside_radius_cannot_join_bands(self):
        points = [[0., 0.], [.4, 0.], [2., 0.], [2., .4], [.4, .4], [0., .4]]
        rows = graph_rows(6, [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)])
        result = self.compare(rows, points, [([0, 5], list(range(6)))], .6 ** 2)
        self.assertEqual(result[0], i64([2, 2]))

    def test_near_u_bridge_inside_radius_restores_connectivity(self):
        points = [[0., 0.], [.2, 0.], [.4, 0.], [.4, .2], [.2, .2], [0., .2]]
        rows = graph_rows(6, [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)])
        result = self.compare(rows, points, [([0], list(range(6)))], .6 ** 2)
        self.assertEqual(result[0], i64([6]))

    def test_component_labels_filter_even_graph_connected_vertices(self):
        rows = graph_rows(4, [(0, 1), (1, 2), (2, 3)])
        result = self.compare(rows, np.zeros((4, 2)), [([0, 2], list(range(4)))],
                              labels=[0, 0, 2, 2])
        self.assertEqual(result[0], i64([2, 2]))

    def test_directed_graph_respects_explicit_labels_and_reachability(self):
        rows = [0b0011, 0b0110, 0b1100, 0b1000]
        result = self.compare(rows, np.zeros((4, 2)), [([0, 1, 2, 3], list(range(4)))],
                              labels=[0, 1, 1, 1], parents=[-1] * 4)
        self.assertEqual(result[0], i64([1, 3, 2, 1]))

    def test_forest_certificate_and_full_flood_are_exactly_equivalent(self):
        rows = graph_rows(8, [(i, i + 1) for i in range(7)] + [(0, 7), (1, 6)])
        points = np.array([[0., 0.], [.1, 0.], [.2, 0.], [.3, 0.],
                           [.3, .2], [.2, .2], [.1, .2], [0., .2]])
        groups = [([0, 3, 7], [7, 6, 5, 4, 3, 2, 1, 0])]
        normal = self.compare(rows, points, groups, .22 ** 2)
        no_certificate = self.compare(rows, points, groups, .22 ** 2, parents=[-1] * 8)
        self.assertEqual(normal, no_certificate)

    def test_cross_group_reuse_resets_seed_and_parent_positions(self):
        rows = graph_rows(6, [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)])
        groups = [([0], [0, 1, 2]), ([3], [2, 1, 0]), ([5], [5, 4, 3]),
                  ([1], [2, 1, 0]), ([4], [0, 1, 2])]
        result = self.compare(rows, np.zeros((6, 2)), groups)
        self.assertEqual(result[0], i64([3, 0, 3, 3, 0]))

    def test_distinct_vertex_ids_with_identical_coordinates_keep_multiplicity(self):
        rows = graph_rows(4, [(0, 1), (1, 2), (2, 3)])
        result = self.compare(rows, np.zeros((4, 2)), [([0], [3, 1, 0, 2])])
        self.assertEqual(result[0], i64([4]))

    def test_candidate_permutation_keeps_original_reduction_order(self):
        rng = np.random.RandomState(701)
        size = 33
        rows = graph_rows(size, [(i, i + 1) for i in range(size - 1)])
        points = rng.normal(size=(size, 2)) * np.array([1., 1e-6])
        for candidates in (list(range(size)), list(range(size - 1, -1, -1)), rng.permutation(size).tolist()):
            self.compare(rows, points, [([0, 7, 19], candidates)], float("inf"))

    def test_radius_nextafter_and_two_component_sum_boundary(self):
        rows = graph_rows(5, [(0, i) for i in range(1, 5)])
        points = [[0., 0.], [.3, .4], [np.nextafter(.5, 0.), 0.],
                  [np.nextafter(.5, np.inf), 0.], [-.5, -0.]]
        for squared in (np.nextafter(.25, 0.), .25, np.nextafter(.25, np.inf)):
            with self.subTest(limit=squared):
                self.compare(rows, points, [([0], [4, 2, 3, 0, 1])], float(squared))

    def test_zero_radius_signed_zero_and_tiny_underflow(self):
        tiny = np.nextafter(0., 1.)
        rows = graph_rows(4, [(0, 1), (1, 2), (2, 3)])
        self.compare(rows, [[-0., 0.], [0., -0.], [tiny, -tiny], [1e-160, -1e-160]],
                     [([0, 2], [2, 0, 3, 1])], 0.)

    def test_exact_point_bounds_and_large_cancellation(self):
        rows = graph_rows(5, [(0, i) for i in range(1, 5)])
        self.compare(rows, [[1e7, -1e7], [-1e7, 1e7], [1e-20, -1e-20],
                            [-1e-20, 1e-20], [0., 0.]],
                     [([0, 1], [2, 0, 4, 1, 3])], float("inf"))

    def test_random_graphs_geometry_and_grouping(self):
        rng = np.random.RandomState(702)
        for size in (1, 3, 8, 17, 33, 65):
            points = rng.normal(size=(size, 2))
            edges = [(i, j) for i in range(size) for j in range(i) if rng.uniform() < .15]
            rows = graph_rows(size, edges)
            permutation = rng.permutation(size).tolist()
            split = size // 2
            groups = [(permutation[:split], rng.permutation(size).tolist()),
                      (permutation[split:], rng.permutation(size).tolist())]
            for squared in (.01, .45 ** 2, 1.35 ** 2, float("inf")):
                with self.subTest(size=size, squared=squared):
                    self.compare(rows, points, groups, squared, 3)

    def test_strided_points_serialized_in_c_order(self):
        rng = np.random.RandomState(703)
        source = rng.normal(size=(18, 4))
        points = source[::2, ::2]
        before = source.tobytes()
        rows = graph_rows(9, [(i, i + 1) for i in range(8)])
        self.assertFalse(points.flags.c_contiguous)
        self.compare(rows, points, [([0, 2, 5], list(range(8, -1, -1)))], 2.)
        self.assertEqual(source.tobytes(), before)

    def test_empty_graph_and_no_groups(self):
        self.compare([], np.empty((0, 2)), [])

    def test_empty_groups_before_between_after_nonempty_groups(self):
        rows = graph_rows(3, [(0, 1), (1, 2)])
        self.compare(rows, [[0., 0.], [.1, 0.], [.2, .1]],
                     [([], [2, 1]), ([0], [0, 1]), ([], []), ([2], []), ([], [0])])

    def test_maximum_points_and_queries_without_dense_fixture(self):
        size = 4000
        rows = [1 << i for i in range(size)]
        result = self.compare(rows, np.zeros((size, 2)), [(list(range(size)), [0])],
                              parents=[-1] * size)
        self.assertEqual(result[0], i64([1] + [0] * (size - 1)))

    def test_maximum_groups_with_empty_rows(self):
        self.compare([1], [[0., 0.]], [([], []) for _ in range(4000)])

    def test_maximum_candidates_per_group(self):
        size = 2048
        rows = [1 << i for i in range(size)]
        result = self.compare(rows, np.zeros((size, 2)), [([size - 1], list(range(size)))],
                              parents=[-1] * size)
        self.assertEqual(result[0], i64([1]))

    def test_concurrent_calls_retain_immutable_inputs(self):
        rng = np.random.RandomState(704)
        rows = graph_rows(31, [(i, i + 1) for i in range(30)])
        points = rng.normal(size=(31, 2))
        groups = [([0, 3, 7], list(range(31))), ([13, 29], list(range(30, -1, -1)))]
        args = make_args(rows, points, groups, .7, 3)
        before = tuple(args)
        expected = numpy_bfs_oracle(rows, points, groups, .7, 3)
        with ThreadPoolExecutor(max_workers=4) as pool:
            outputs = list(pool.map(lambda _: native.local_geometry(*args), range(16)))
        self.assertEqual(outputs, [expected] * 16)
        self.assertEqual(args, before)

    def test_output_bytes_survive_input_and_capsule_lifetime(self):
        args = self.default_args()
        result = native.local_geometry(*args)
        expected = tuple(result)
        views = tuple(memoryview(value) for value in result)
        del args, result
        gc.collect()
        self.assertTrue(all(view.readonly for view in views))
        self.assertEqual(tuple(view.tobytes() for view in views), expected)

    def test_wrong_argument_count(self):
        args = self.default_args()
        for bad in ([], args[:-1], args + [0]):
            with self.subTest(count=len(bad)), self.assertRaises(TypeError):
                native.local_geometry(*bad)

    def test_graph_must_be_correct_prepared_capsule(self):
        for invalid in (None, b"", object(), 0):
            args = self.default_args()
            args[0] = invalid
            with self.subTest(type=type(invalid).__name__), self.assertRaises(TypeError):
                native.local_geometry(*args)
        args = self.default_args()
        args[0] = native.prepare_graph(packed_graph([1]), 1)
        with self.assertRaises(ValueError):
            native.local_geometry(*args)

    def test_exact_immutable_byte_arguments(self):
        class ByteSubclass(bytes):
            pass
        for index in range(2, 9):
            for converter in (bytearray, memoryview, ByteSubclass):
                args = self.default_args()
                args[index] = converter(args[index])
                with self.subTest(index=index, converter=converter.__name__), self.assertRaises(TypeError):
                    native.local_geometry(*args)
            args = self.default_args()
            args[index] = memoryview(bytearray(args[index])).toreadonly()
            with self.subTest(index=index, readonly_mutable_alias=True), self.assertRaises(TypeError):
                native.local_geometry(*args)

    def test_integer_arguments_require_builtin_int_not_bool_or_subclass(self):
        class IntegerSubclass(int):
            pass
        for index in (1, 10):
            for invalid in (True, False, 0., np.int64(0), IntegerSubclass(0), "0", None):
                args = self.default_args()
                args[index] = invalid
                with self.subTest(index=index, type=type(invalid).__name__), self.assertRaises(TypeError):
                    native.local_geometry(*args)

    def test_integer_bounds_and_huge_integers(self):
        for index, values in ((1, (-1, 4001, 2**100, -2**100)),
                              (10, (-1, 2049, 2**100, -2**100))):
            for invalid in values:
                args = self.default_args()
                args[index] = invalid
                with self.subTest(index=index, value=invalid), self.assertRaises((ValueError, OverflowError)):
                    native.local_geometry(*args)

    def test_radius_requires_builtin_float(self):
        class FloatSubclass(float):
            pass
        for invalid in (0, True, np.float64(1.), FloatSubclass(1.), "1", None):
            args = self.default_args()
            args[9] = invalid
            with self.subTest(type=type(invalid).__name__), self.assertRaises(TypeError):
                native.local_geometry(*args)

    def test_radius_rejects_nan_negative_and_negative_infinity(self):
        for invalid in (float("nan"), -1., float("-inf")):
            args = self.default_args()
            args[9] = invalid
            with self.subTest(value=invalid), self.assertRaises(ValueError):
                native.local_geometry(*args)

    def test_point_labels_parent_lengths_are_exact(self):
        for index in (2, 3, 4):
            for delta in (-1, -8, 1, 8):
                args = self.default_args()
                value = args[index]
                args[index] = value[:delta] if delta < 0 else value + bytes(delta)
                with self.subTest(index=index, delta=delta), self.assertRaises(ValueError):
                    native.local_geometry(*args)

    def test_csr_and_id_bytes_must_be_aligned_native_int64(self):
        for index in (5, 6, 7, 8):
            for delta in (-1, 1):
                args = self.default_args()
                args[index] = args[index][:-1] if delta < 0 else args[index] + b"\0"
                with self.subTest(index=index, delta=delta), self.assertRaises(ValueError):
                    native.local_geometry(*args)

    def test_offsets_start_zero_monotone_and_match_exact_end(self):
        for index, invalids in ((5, ([], [1, 2, 3], [0, 3, 2], [0, -1, 3], [0, 2, 2], [0, 2, 4])),
                                (7, ([], [1, 3, 5], [0, 5, 3], [0, -1, 5], [0, 3, 4], [0, 3, 6]))):
            for invalid in invalids:
                args = self.default_args()
                args[index] = i64(invalid)
                with self.subTest(index=index, invalid=invalid), self.assertRaises(ValueError):
                    native.local_geometry(*args)

    def test_query_and_candidate_group_counts_must_match(self):
        args = self.default_args()
        args[7] = i64([0, 5])
        with self.assertRaises(ValueError):
            native.local_geometry(*args)

    def test_offset_int64_extremes_rejected_without_overflow(self):
        for index in (5, 7):
            for invalid in (-(2**63), 2**63 - 1):
                args = self.default_args()
                args[index] = i64([0, invalid, invalid])
                with self.subTest(index=index, invalid=invalid), self.assertRaises(ValueError):
                    native.local_geometry(*args)

    def test_group_query_and_candidate_capacities(self):
        for index, bad in ((5, i64([0] * 4002)), (6, i64([0] * 4001))):
            args = self.default_args()
            args[index] = bad
            with self.subTest(index=index), self.assertRaises(ValueError):
                native.local_geometry(*args)
        args = self.default_args()
        args[5], args[6], args[7], args[8] = i64([0, 1]), i64([0]), i64([0, 2049]), i64([0] * 2049)
        with self.assertRaises(ValueError):
            native.local_geometry(*args)

    def test_total_candidate_byte_capacity_checked_before_traversal(self):
        args = self.default_args()
        # One short-lived zero-filled allocation exercises the 62.5 MiB CSR byte
        # bound; it is rejected by length before graph traversal or copying.
        args[8] = bytes((4000 * 2048 + 1) * 8)
        with self.assertRaises(ValueError):
            native.local_geometry(*args)

    def test_ids_must_be_in_range_even_in_zero_query_groups(self):
        for index in (6, 8):
            for invalid in (-1, 4, -(2**63), 2**63 - 1):
                args = self.default_args()
                values = list(struct.unpack("=" + "q" * (len(args[index]) // 8), args[index]))
                values[-1] = invalid
                args[index] = i64(values)
                with self.subTest(index=index, invalid=invalid), self.assertRaises(ValueError):
                    native.local_geometry(*args)
        args = make_args([1], [[0., 0.]], [([], [1])])
        with self.assertRaises(ValueError):
            native.local_geometry(*args)

    def test_duplicate_queries_rejected_within_and_across_groups(self):
        rows = graph_rows(3, [(0, 1), (1, 2)])
        for groups in ([([0, 0], [0, 1])], [([0], [0]), ([0], [0, 1])]):
            with self.subTest(groups=groups), self.assertRaises(ValueError):
                native.local_geometry(*make_args(rows, np.zeros((3, 2)), groups))

    def test_duplicate_candidates_rejected_within_each_group_including_unused_group(self):
        rows = graph_rows(3, [(0, 1), (1, 2)])
        for groups in ([([0], [0, 0, 1])], [([], [2, 2])], [([0], [0]), ([1], [0, 1, 1])]):
            with self.subTest(groups=groups), self.assertRaises(ValueError):
                native.local_geometry(*make_args(rows, np.zeros((3, 2)), groups))

    def test_component_labels_are_bounded_even_without_queries(self):
        for labels in ((-1, 0, 0, 0), (0, 0, 0, 4), (0, 2**63 - 1, 0, 0)):
            args = self.default_args()
            args[3] = i64(labels)
            with self.subTest(labels=labels), self.assertRaises(ValueError):
                native.local_geometry(*args)
        with self.assertRaises(ValueError):
            native.local_geometry(*make_args([1], [[0., 0.]], [], labels=[1]))

    def test_points_reject_nonfinite_and_out_of_bound_even_if_unused(self):
        for invalid in (float("nan"), float("inf"), float("-inf"),
                        np.nextafter(1e7, np.inf), np.nextafter(-1e7, -np.inf)):
            args = self.default_args()
            values = np.frombuffer(args[2], dtype=np.float64).copy()
            values[-1] = invalid
            args[2] = values.tobytes()
            with self.subTest(value=invalid), self.assertRaises(ValueError):
                native.local_geometry(*args)
        with self.assertRaises(ValueError):
            native.local_geometry(*make_args([1], [[float("nan"), 0.]], []))

    def test_invalid_forest_parent_bounds_self_cycle_and_nonedge(self):
        for parents in ((-2, 0, 1, -1), (-1, 4, 1, -1), (-1, 0, 2, -1),
                        (1, 0, -1, -1), (-1, 3, 1, -1)):
            args = self.default_args()
            args[4] = i64(parents)
            with self.subTest(parents=parents), self.assertRaises(ValueError):
                native.local_geometry(*args)

    def test_one_way_parent_edge_is_not_a_forest_certificate(self):
        rows = [0b11, 0b10]
        with self.assertRaises(ValueError):
            native.local_geometry(*make_args(rows, np.zeros((2, 2)), [([1], [0, 1])], parents=[-1, 0]))

    def test_invalid_forest_validated_even_without_queries(self):
        rows = graph_rows(3, [(0, 1), (1, 2), (0, 2)])
        with self.assertRaises(ValueError):
            native.local_geometry(*make_args(rows, np.zeros((3, 2)), [], parents=[2, 0, 1]))

    def test_error_does_not_poison_subsequent_valid_call(self):
        good = self.default_args()
        expected = native.local_geometry(*good)
        for position, bad in ((9, float("nan")), (3, i64([-1, 0, 0, 0])), (8, i64([0, 0, 2, 3, 2]))):
            args = list(good)
            args[position] = bad
            with self.assertRaises(ValueError):
                native.local_geometry(*args)
            self.assertEqual(native.local_geometry(*good), expected)


if __name__ == "__main__":
    unittest.main()
