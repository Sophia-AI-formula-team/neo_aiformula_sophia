"""Independent local-support oracle/FFI checks; requires this source's binary.

No simulator, core import, floating-point gate, or ground truth is involved.
CI requires the extension; source-only runs explicitly skip this test module.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import datetime
import hashlib
from pathlib import Path
import random
import struct
import unittest

import os
import sys
import pytest
sys.path.insert(0, str(Path(__file__).parents[1]))
if os.environ.get("LANE_REQUIRE_NATIVE") == "1":
    import _lane_mapping_lya_native as native
else:
    native = pytest.importorskip("_lane_mapping_lya_native")


def i64(values):
    values = tuple(values)
    return struct.pack("=" + "q" * len(values), *values)


def unpack_i64(value):
    return struct.unpack("=" + "q" * (len(value) // 8), value)


def graph_rows(size, edges):
    rows = [1 << vertex for vertex in range(size)]
    for left, right in edges:
        rows[left] |= 1 << right
        rows[right] |= 1 << left
    return rows


def pack_graph(rows):
    width = (len(rows) + 7) // 8
    return b"".join(row.to_bytes(width, "little") for row in rows)


def bfs_forest(rows):
    """Queue BFS generates a valid forest independently of native bit order."""
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


def bfs_support(rows, candidates, targets, allowed):
    """Set/queue oracle; no component-size or forest-edge shortcut."""
    columns = len(candidates)
    output = bytearray(len(allowed))
    counts = []
    for number, seed in enumerate(targets):
        row = allowed[number * columns:(number + 1) * columns]
        permitted = {vertex for vertex, chosen in zip(candidates, row) if chosen}
        seen = set()
        if seed in permitted:
            seen.add(seed)
            pending = deque([seed])
            while pending:
                vertex = pending.popleft()
                for neighbor in permitted - seen:
                    if rows[vertex] & (1 << neighbor):
                        seen.add(neighbor)
                        pending.append(neighbor)
        for column, vertex in enumerate(candidates):
            output[number * columns + column] = int(vertex in seen)
        counts.append(len(seen))
    return bytes(output), i64(counts)


class LocalSupportTests(unittest.TestCase):
    def invoke(self, rows, candidates, targets, allowed, parents=None, capsule=None):
        if parents is None:
            parents = bfs_forest(rows)
        if capsule is None:
            capsule = native.prepare_graph(pack_graph(rows), len(rows))
        return native.local_support(
            capsule, len(rows), i64(candidates), i64(targets), bytes(allowed), i64(parents))

    def check_oracle(self, rows, candidates, targets, allowed, parents=None):
        original = (list(rows), list(candidates), list(targets), bytes(allowed))
        result = self.invoke(rows, candidates, targets, allowed, parents)
        self.assertEqual(result, bfs_support(rows, candidates, targets, allowed))
        self.assertEqual(len(result[0]), len(targets) * len(candidates))
        self.assertEqual(len(result[1]), len(targets) * 8)
        self.assertTrue(all(value in (0, 1) for value in result[0]))
        self.assertEqual(original, (list(rows), list(candidates), list(targets), bytes(allowed)))
        return result

    def test_binary_provenance_and_api_are_explicit(self):
        expected = hashlib.sha256((Path(__file__).parents[1] / "native" / "lane_graph_native.c").read_bytes()).hexdigest()
        self.assertEqual(native.SOURCE_SHA256, expected)
        self.assertEqual(native.API_VERSION, 1)
        self.assertEqual(native.LOCAL_SUPPORT_API_VERSION, 1)
        self.assertEqual(native.MAX_CANDIDATES, 2048)
        self.assertTrue(str(native.__file__).endswith((".pyd", ".so")))

    def test_empty_graph_queries_and_candidates(self):
        self.assertEqual(self.invoke([], [], [], b""), (b"", b""))
        rows = graph_rows(4, [(0, 1), (1, 2)])
        self.assertEqual(self.invoke(rows, [3, 1, 2], [], b""), (b"", b""))
        self.assertEqual(self.invoke(rows, [], [0, 3, 1], b""), (b"", i64([0, 0, 0])))

    def test_single_vertex_allowed_and_disallowed(self):
        self.check_oracle([1], [0], [0, 0], b"\x01\x00")

    def test_missing_seed_connected_candidate_set_is_not_support(self):
        rows = graph_rows(5, [(0, 1), (1, 2), (2, 3), (3, 4)])
        result = self.check_oracle(rows, [2, 3, 4], [0, 1, 2], b"\x01" * 9)
        self.assertEqual(result, (b"\x00" * 6 + b"\x01" * 3, i64([0, 0, 3])))

    def test_disallowed_seed_zeroes_entire_row(self):
        rows = graph_rows(4, [(0, 1), (1, 2), (2, 3)])
        self.assertEqual(
            self.check_oracle(rows, [0, 1, 2, 3], [1], bytes([1, 0, 1, 1])),
            (b"\x00" * 4, i64([0])))

    def test_u_bridge_cannot_connect_locally_disconnected_bands(self):
        # 0-1-2-3-4-5-6 is one global U; removing its remote middle isolates arms.
        rows = graph_rows(7, [(vertex, vertex + 1) for vertex in range(6)])
        candidates = [6, 2, 0, 5, 1]
        targets = [0, 6, 2, 3]
        result = self.check_oracle(rows, candidates, targets, b"\x01" * 20)
        self.assertEqual(unpack_i64(result[1]), (3, 2, 3, 0))

    def test_nontrivial_forest_certificate_and_flood_agree(self):
        rows = graph_rows(8, [(0, 1), (1, 2), (1, 3), (3, 4), (3, 5), (5, 6), (6, 7), (2, 7)])
        candidates = [7, 1, 3, 2, 0, 6, 4, 5]
        targets = [7, 2, 4, 0]
        allowed = bytes([1] * 8 + [1, 1, 0, 1, 1, 1, 0, 1] + [0, 0, 1, 0, 0, 0, 1, 1] + [1] * 8)
        reference = self.check_oracle(rows, candidates, targets, allowed)
        # A partial (empty) forest is valid and forces exact flood for >1 point.
        self.assertEqual(reference, self.check_oracle(rows, candidates, targets, allowed, [-1] * 8))

    def test_parent_edges_outside_candidates_do_not_certify_row(self):
        rows = graph_rows(5, [(0, 1), (1, 2), (2, 3), (3, 4)])
        result = self.check_oracle(rows, [4, 0, 2], [4, 0, 2], b"\x01" * 9)
        self.assertEqual(unpack_i64(result[1]), (1, 1, 1))

    def test_directed_extra_edges_use_flood_when_forest_is_empty(self):
        rows = [1 | 2, 2 | 4, 4, 8 | 1]
        self.check_oracle(rows, [3, 2, 1, 0], [0, 1, 2, 3], b"\x01" * 16, [-1] * 4)

    def test_byte_word_boundaries_and_tail_padding(self):
        for size in (7, 8, 9, 63, 64, 65, 127, 129):
            with self.subTest(size=size):
                rows = graph_rows(size, [(vertex, vertex + 1) for vertex in range(size - 1)])
                if size % 8:
                    padding = ((1 << (8 * ((size + 7) // 8))) - 1) ^ ((1 << size) - 1)
                    rows_with_padding = [row | padding for row in rows]
                else:
                    rows_with_padding = rows
                capsule = native.prepare_graph(pack_graph(rows_with_padding), size)
                candidates = list(reversed(range(size)))
                allowed = bytes(int(index % 5 != 1) for _ in range(3) for index in range(size))
                targets = [0, size // 2, size - 1]
                self.assertEqual(self.invoke(rows, candidates, targets, allowed, capsule=capsule),
                                 bfs_support(rows, candidates, targets, allowed))

    def test_random_undirected_graphs_masks_and_candidate_order(self):
        rng = random.Random(761031)
        for case in range(75):
            size = rng.choice((2, 3, 7, 17, 33, 65, 129, 256))
            edges = [(left, right) for left in range(size) for right in range(left + 1, size)
                     if rng.random() < min(0.3, 4 / size)]
            rows = graph_rows(size, edges)
            candidates = rng.sample(range(size), rng.randrange(size + 1))
            targets = [rng.randrange(size) for _ in range(12)]
            allowed = bytes(rng.randrange(2) for _ in range(len(candidates) * len(targets)))
            with self.subTest(case=case, size=size):
                self.check_oracle(rows, candidates, targets, allowed)
                self.check_oracle(rows, candidates, targets, allowed, [-1] * size)

    def test_n4000_candidate2048_query4000_zero_rows_are_bounded(self):
        size = 4000
        rows = graph_rows(size, [])
        candidates = list(range(2048))
        targets = list(range(4000))
        allowed = bytes(len(candidates) * len(targets))
        result = self.invoke(rows, candidates, targets, allowed)
        self.assertEqual(result, (allowed, bytes(8 * len(targets))))

    def test_max_candidates_nonzero_chain(self):
        rows = graph_rows(4000, [(vertex, vertex + 1) for vertex in range(3999)])
        candidates = list(reversed(range(2048)))
        targets = [0, 2047, 2048, 3999]
        result = self.invoke(rows, candidates, targets, b"\x01" * (4 * 2048))
        self.assertEqual(result, (b"\x01" * 4096 + b"\x00" * 4096, i64([2048, 2048, 0, 0])))

    def test_reused_capsule_concurrent_queries_have_no_shared_mutation(self):
        rows = graph_rows(129, [(vertex, vertex + 1) for vertex in range(128)])
        capsule = native.prepare_graph(pack_graph(rows), len(rows))
        candidates = list(reversed(range(129)))
        cases = [(list(range(0, 129, 16)), bytes(int((i + number) % 7 != 0) for i in range(9 * 129)))
                 for number in range(24)]
        expected = [bfs_support(rows, candidates, targets, allowed) for targets, allowed in cases]
        def run(case):
            return self.invoke(rows, candidates, case[0], case[1], capsule=capsule)
        with ThreadPoolExecutor(max_workers=4) as pool:
            self.assertEqual(list(pool.map(run, cases)), expected)

    def default_args(self):
        rows = graph_rows(4, [(0, 1), (1, 2), (2, 3)])
        return [native.prepare_graph(pack_graph(rows), 4), 4, i64([0, 1, 2, 3]),
                i64([0, 3]), b"\x01" * 8, i64([-1, 0, 1, 2])]

    def test_requires_prepared_capsule_and_matching_size(self):
        args = self.default_args()
        for replacement in (b"", None, object(), datetime.datetime_CAPI):
            with self.subTest(value=type(replacement).__name__), self.assertRaises(TypeError):
                native.local_support(replacement, *args[1:])
        with self.assertRaises(ValueError):
            native.local_support(args[0], 3, *args[2:])
        for size in (True, 4.0, "4"):
            with self.assertRaises(TypeError):
                native.local_support(args[0], size, *args[2:])
        for size in (-1, 4001, 2 ** 100):
            with self.assertRaises(ValueError):
                native.local_support(args[0], size, *args[2:])

    def test_byte_inputs_reject_mutable_subclass_and_wrong_lengths(self):
        class BytesSubclass(bytes):
            pass
        for position in (2, 3, 4, 5):
            for convert in (bytearray, memoryview, BytesSubclass):
                args = self.default_args()
                args[position] = convert(args[position])
                with self.subTest(position=position, type=convert.__name__), self.assertRaises(TypeError):
                    native.local_support(*args)
            args = self.default_args()
            args[position] = args[position] + b"\x00"
            with self.subTest(position=position), self.assertRaises(ValueError):
                native.local_support(*args)

    def test_capacity_limits_are_rejected_before_access(self):
        for position, length in ((2, 2049), (3, 4001)):
            args = self.default_args()
            args[position] = i64([0] * length)
            with self.assertRaises(ValueError):
                native.local_support(*args)

    def test_candidates_unique_and_ids_in_range(self):
        for position, value in ((2, [0, 1, 1, 3]), (2, [-1, 1, 2, 3]),
                                (2, [0, 1, 2, 4]), (2, [-(2 ** 63), 1, 2, 3]),
                                (3, [-1, 3]), (3, [0, 4]), (3, [0, 2 ** 63 - 1])):
            args = self.default_args()
            args[position] = i64(value)
            with self.subTest(position=position, value=value), self.assertRaises(ValueError):
                native.local_support(*args)

    def test_all_allowed_values_validated_even_missing_or_disallowed_seed(self):
        for allowed in (bytes([0, 2, 0, 0, 0, 0, 0, 0]), bytes([0] * 7 + [255])):
            args = self.default_args()
            args[4] = allowed
            with self.assertRaises(ValueError):
                native.local_support(*args)
        args = self.default_args()
        args[2], args[4] = i64([1, 2]), bytes([0, 2, 0, 0])
        with self.assertRaises(ValueError):
            native.local_support(*args)

    def test_parent_bounds_self_cycle_and_nonedge_rejected(self):
        for parents in ((-2, 0, 1, 2), (-1, 4, 1, 2), (-1, 0, 2, 2),
                        (1, 0, -1, 2), (-1, 3, 1, 2)):
            args = self.default_args()
            args[5] = i64(parents)
            with self.subTest(parents=parents), self.assertRaises(ValueError):
                native.local_support(*args)

    def test_forged_n_minus_one_cycle_cannot_certify_disconnected_row(self):
        # Three cyclic edges + isolated vertex = n-1, but not connected.
        rows = graph_rows(4, [(0, 1), (1, 2), (2, 0)])
        with self.assertRaises(ValueError):
            self.invoke(rows, [0, 1, 2, 3], [0], b"\x01" * 4, [2, 0, 1, -1])

    def test_one_way_parent_edge_cannot_certify_both_seed_directions(self):
        rows = [1 | 2, 2]
        with self.assertRaises(ValueError):
            self.invoke(rows, [0, 1], [1], b"\x01" * 2, [-1, 0])

    def test_invalid_forest_rejected_even_without_queries(self):
        rows = graph_rows(3, [(0, 1), (1, 2)])
        with self.assertRaises(ValueError):
            self.invoke(rows, [], [], b"", [1, 0, -1])

    def test_native_int64_byte_contract_and_output_order(self):
        self.assertEqual(struct.calcsize("=q"), 8)
        rows = graph_rows(4, [(0, 1), (1, 2)])
        result = self.check_oracle(rows, [3, 2, 0, 1], [3, 0, 2], b"\x01" * 12)
        self.assertEqual(result, (bytes([1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1]), i64([1, 3, 3])))


if __name__ == "__main__":
    unittest.main(verbosity=2)
