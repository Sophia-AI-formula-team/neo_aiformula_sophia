"""Pure-stdlib exact topology and FFI boundary tests; compiled module required."""
import hashlib
from concurrent.futures import ThreadPoolExecutor
import datetime
from pathlib import Path
import random
import unittest

import os
import sys
import pytest
sys.path.insert(0, str(Path(__file__).parents[1]))
if os.environ.get('LANE_REQUIRE_NATIVE') == '1':
    import _lane_mapping_lya_native as native
else:
    native = pytest.importorskip('_lane_mapping_lya_native')
"""R5 lowest-bit traversal oracle; no geometry approximation or dependencies."""


def pack_rows(rows, n):
    width = (n + 7) // 8
    return b"".join(row.to_bytes(width, "little") for row in rows)


def unpack_rows(packed, n):
    width = (n + 7) // 8
    mask = (1 << n) - 1
    return [int.from_bytes(packed[start:start + width], "little") & mask
            for start in range(0, len(packed), width)] if width else []


def connected_subsets(adjacency, n, allowed, seeds):
    neighbors = unpack_rows(adjacency, n)
    results = []
    for mask, seed in zip(unpack_rows(allowed, n), seeds):
        reached = frontier = (1 << seed) if mask & (1 << seed) else 0
        while frontier and reached != mask:
            bit = frontier & -frontier
            frontier ^= bit
            added = neighbors[bit.bit_length() - 1] & mask & ~reached
            reached |= added
            frontier |= added
        results.append(reached)
    return pack_rows(results, n)


def graph_forest(adjacency, n):
    neighbors = unpack_rows(adjacency, n)
    labels, parents = list(range(n)), [-1] * n
    unseen = (1 << n) - 1
    while unseen:
        seed = unseen & -unseen
        reached = frontier = seed
        while frontier:
            bit = frontier & -frontier
            frontier ^= bit
            node = bit.bit_length() - 1
            added = neighbors[node] & ~reached
            children = added
            while children:
                child = children & -children
                children ^= child
                parents[child.bit_length() - 1] = node
            reached |= added
            frontier |= added
        for index in range(n):
            if reached & (1 << index):
                labels[index] = seed.bit_length() - 1
        unseen &= ~reached
    return labels, parents


def independent_subset(adjacency, n, allowed, seed):
    """Set/stack BFS checks connectivity without reusing the bitset algorithm."""
    if not (allowed & (1 << seed)):
        return 0
    rows = unpack_rows(adjacency, n)
    found, stack = {seed}, [seed]
    while stack:
        current = stack.pop()
        for other in range(n):
            if other not in found and (allowed & rows[current] & (1 << other)):
                found.add(other)
                stack.append(other)
    return sum(1 << value for value in found)



def graph(n, edges, symmetric=True):
    rows = [1 << index for index in range(n)]
    for first, second in edges:
        rows[first] |= 1 << second
        if symmetric:
            rows[second] |= 1 << first
    return pack_rows(rows, n)


class NativeTopologyTests(unittest.TestCase):
    def assert_subsets(self, adjacency, n, masks, seeds, independent=False):
        allowed = pack_rows(masks, n)
        before = adjacency, allowed, list(seeds)
        actual = native.connected_subsets(adjacency, n, allowed, seeds)
        self.assertEqual(actual, connected_subsets(adjacency, n, allowed, seeds))
        prepared = native.prepare_graph(adjacency, n)
        self.assertEqual(native.connected_subsets(prepared, n, allowed, seeds), actual)
        self.assertEqual(native.graph_forest(prepared, n), graph_forest(adjacency, n))
        self.assertEqual((adjacency, allowed, list(seeds)), before)
        self.assertIs(type(actual), bytes)
        if independent:
            self.assertEqual(unpack_rows(actual, n),
                             [independent_subset(adjacency, n, mask, seed)
                              for mask, seed in zip(masks, seeds)])

    def test_compiled_source_identity(self):
        expected = hashlib.sha256((Path(__file__).parents[1] / "native/lane_graph_native.c").read_bytes()).hexdigest()
        self.assertEqual(native.API_VERSION, 1)
        self.assertEqual(native.SOURCE_SHA256, expected)
        self.assertEqual((native.MAX_POINTS, native.MAX_QUERIES), (4000, 4000))
        self.assertTrue(native.__file__.endswith((".pyd", ".so")), native.__file__)

    def test_empty_contract(self):
        self.assertEqual(native.graph_forest(b"", 0), ([], []))
        self.assertEqual(native.connected_subsets(b"", 0, b"", []), b"")
        empty = native.prepare_graph(b"", 0)
        self.assertEqual(native.graph_forest(empty, 0), ([], []))
        self.assertEqual(native.connected_subsets(empty, 0, b"", []), b"")

    def test_prepared_graph_rejects_wrong_size_and_foreign_capsule(self):
        prepared = native.prepare_graph(b"\1", 1)
        with self.assertRaises(ValueError):
            native.graph_forest(prepared, 0)
        with self.assertRaises(ValueError):
            native.connected_subsets(prepared, 2, b"\1", [0])
        with self.assertRaises(TypeError):
            native.graph_forest(datetime.datetime_CAPI, 1)
        with self.assertRaises(TypeError):
            native.prepare_graph(prepared, 1)

    def test_prepared_graph_can_be_read_concurrently(self):
        n = 129
        packed = graph(n, [(index, index + 1) for index in range(n - 1)])
        prepared = native.prepare_graph(packed, n)
        expected = graph_forest(packed, n)
        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(lambda unused: native.graph_forest(prepared, n), range(100)))
        self.assertEqual(results, [expected] * 100)

    def test_prepared_graph_owns_copy_after_input_reference_disappears(self):
        for _ in range(1000):
            prepared = native.prepare_graph(bytes([3, 3]), 2)
            self.assertEqual(native.graph_forest(prepared, 2), ([0, 0], [-1, 0]))

    def test_byte_word_and_capacity_boundaries(self):
        for n in (1, 7, 8, 9, 63, 64, 65, 127, 128, 129, 3999, 4000):
            with self.subTest(n=n):
                adjacency = graph(n, ((index, index + 1) for index in range(n - 1)))
                masks = [(1 << n) - 1, 0, 1, 1 << (n - 1)]
                self.assert_subsets(adjacency, n, masks, [0, n - 1, n - 1, n - 1])
                self.assertEqual(native.graph_forest(adjacency, n), graph_forest(adjacency, n))

    def test_maximum_query_count_zero_masks(self):
        n = 4000
        adjacency = graph(n, [])
        allowed = b"\0" * (4000 * ((n + 7) // 8))
        self.assertEqual(native.connected_subsets(adjacency, n, allowed, list(range(n))), allowed)

    def test_complete_and_disconnected_graphs(self):
        for n in (1, 65, 137):
            for adjacency in (graph(n, []), pack_rows([(1 << n) - 1] * n, n)):
                with self.subTest(n=n, complete=adjacency[:1] == b"\xff"):
                    self.assertEqual(native.graph_forest(adjacency, n), graph_forest(adjacency, n))
                    self.assert_subsets(adjacency, n, [(1 << n) - 1] * 3, [0, n // 2, n - 1])

    def test_parent_order_matches_first_discovery(self):
        adjacency = graph(8, [(0, 6), (0, 3), (3, 7), (3, 1), (6, 2), (1, 2), (2, 4)])
        labels, parents = native.graph_forest(adjacency, 8)
        self.assertEqual((labels, parents), graph_forest(adjacency, 8))
        self.assertEqual(parents, [-1, 3, 1, 0, 2, -1, 0, 3])

    def test_remote_u_bridge_is_not_local_connectivity(self):
        adjacency = graph(7, [(index, index + 1) for index in range(6)])
        mask = sum(1 << index for index in (0, 1, 5, 6))
        reached = native.connected_subsets(adjacency, 7, pack_rows([mask, mask], 7), [0, 6])
        self.assertEqual(unpack_rows(reached, 7), [(1 << 0) | (1 << 1), (1 << 5) | (1 << 6)])
        self.assert_subsets(adjacency, 7, [mask, mask], [0, 6], independent=True)

    def test_missing_or_disallowed_seed_is_zero(self):
        adjacency = graph(4, [(0, 1), (1, 2), (2, 3)])
        self.assert_subsets(adjacency, 4, [7, 0, 14], [3, 0, 0], independent=True)
        self.assertEqual(native.connected_subsets(adjacency, 4, bytes([7, 0, 14]), [3, 0, 0]), b"\0\0\0")

    def test_duplicate_seeds_are_independent_queries(self):
        adjacency = graph(9, [(index, index + 1) for index in range(8)])
        self.assert_subsets(adjacency, 9, [511, 15, 3, 0, 1], [0] * 5, independent=True)

    def test_tail_padding_never_escapes(self):
        for n in (1, 7, 9, 63, 65, 127, 3999):
            width = (n + 7) // 8
            adjacency = b"\xff" * (n * width)
            allowed = b"\xff" * width
            result = native.connected_subsets(adjacency, n, allowed, [0])
            self.assertEqual(result, ((1 << n) - 1).to_bytes(width, "little"))
            self.assertEqual(native.graph_forest(adjacency, n), ([0] * n, [-1] + [0] * (n - 1)))

    def test_directed_input_still_matches_literal_r5_order(self):
        # Production graph is symmetric. The FFI need not invent that guarantee;
        # even this atypical input has the same defined traversal as the oracle.
        adjacency = graph(6, [(1, 0), (1, 4), (4, 2), (3, 0), (5, 3)], symmetric=False)
        self.assertEqual(native.graph_forest(adjacency, 6), graph_forest(adjacency, 6))
        self.assert_subsets(adjacency, 6, [63] * 6, list(range(6)), independent=True)

    def test_random_graph_masks_match_bits_and_parents(self):
        rng = random.Random(61011)
        for n in (2, 5, 17, 63, 64, 65, 131):
            for trial in range(12):
                edges = [(first, second) for first in range(n) for second in range(first + 1, n)
                         if rng.random() < (0.04 if trial % 2 else 0.4)]
                adjacency = graph(n, edges)
                masks = [rng.getrandbits(n) for _ in range(16)]
                seeds = [rng.randrange(n) for _ in masks]
                with self.subTest(n=n, trial=trial):
                    self.assert_subsets(adjacency, n, masks, seeds, independent=n <= 17)
                    self.assertEqual(native.graph_forest(adjacency, n), graph_forest(adjacency, n))

    def test_repeated_calls_do_not_retain_foreign_graph_state(self):
        for _ in range(100):
            self.assertEqual(native.graph_forest(b"\x03\x03", 2), ([0, 0], [-1, 0]))
            self.assertEqual(native.graph_forest(b"\x01\x02", 2), ([0, 1], [-1, -1]))

    def test_rejects_wrong_n_type(self):
        for value in (True, False, 1.0, "1", None):
            with self.subTest(value=value), self.assertRaises(TypeError):
                native.graph_forest(b"\1", value)

    def test_rejects_n_outside_capacity(self):
        for value in (-1, 4001, 1 << 100):
            with self.subTest(value=value), self.assertRaises(ValueError):
                native.graph_forest(b"", value)

    def test_rejects_mutable_and_nonbytes_graph(self):
        class DerivedBytes(bytes):
            pass
        for value in (bytearray(b"\1"), memoryview(b"\1"), [1], "x", None, DerivedBytes(b"\1")):
            with self.subTest(value=type(value)), self.assertRaises(TypeError):
                native.graph_forest(value, 1)

    def test_rejects_wrong_adjacency_length(self):
        for n, adjacency in ((0, b"x"), (1, b""), (1, b"xx"), (9, b"\0" * 17), (9, b"\0" * 19)):
            with self.subTest(n=n, length=len(adjacency)), self.assertRaises(ValueError):
                native.graph_forest(adjacency, n)

    def test_rejects_wrong_allowed_type(self):
        for value in (bytearray(b"\1"), memoryview(b"\1"), [1], "x", None):
            with self.subTest(value=type(value)), self.assertRaises(TypeError):
                native.connected_subsets(b"\1", 1, value, [0])

    def test_rejects_wrong_allowed_length(self):
        for value in (b"", b"\1\1"):
            with self.assertRaises(ValueError):
                native.connected_subsets(b"\1", 1, value, [0])

    def test_rejects_noninteger_and_boolean_seeds(self):
        for seed in (True, False, 0.0, "0", None):
            with self.subTest(seed=seed), self.assertRaises(TypeError):
                native.connected_subsets(b"\1", 1, b"\1", [seed])

    def test_rejects_out_of_range_seeds_instead_of_wrapping(self):
        for seed in (-1, 1, 1 << 100):
            with self.subTest(seed=seed), self.assertRaises(ValueError):
                native.connected_subsets(b"\1", 1, b"\1", [seed])
        with self.assertRaises(ValueError):
            native.connected_subsets(b"", 0, b"", [0])

    def test_rejects_unbounded_iterators_and_excess_queries(self):
        for seeds in (iter([0]), {0}, {"seed": 0}, "0", b"0", None):
            with self.subTest(seeds=type(seeds)), self.assertRaises(TypeError):
                native.connected_subsets(b"\1", 1, b"\1", seeds)
        with self.assertRaises(ValueError):
            native.connected_subsets(b"\1", 1, b"\1" * 4001, [0] * 4001)

    def test_sequence_protocol_errors_propagate(self):
        class BrokenSequence:
            def __len__(self):
                return 2
            def __getitem__(self, index):
                raise LookupError("synthetic sequence failure")
        with self.assertRaisesRegex(LookupError, "synthetic sequence failure"):
            native.connected_subsets(b"\1", 1, b"\1\1", BrokenSequence())


if __name__ == "__main__":
    unittest.main(verbosity=2)
