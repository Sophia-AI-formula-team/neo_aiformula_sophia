"""Exact graph batching regressions, without Git history or timing assertions.

The <=4000 oracle forms blockwise all-pairs distances and uses ordinary sets,
not production buckets, packed graph, batching or the forest shortcut. The
larger-graph oracle freezes the previous ordered union-find contract. These
are geometry/resource-bound tests, not runtime acceptance.
"""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from lane_mapping_lya_reference import mask_localization as module


def dense_graph(points, radius):
    matrix = np.empty((len(points), len(points)), dtype=bool)
    for start in range(0, len(points), 64):
        delta = points[start:start + 64, None, :] - points[None, :, :]
        matrix[start:start + 64] = np.sum(delta * delta, axis=2) <= radius ** 2
    return matrix


def reference_components(self):
    count = len(self.anchor)
    # This independent set-based oracle intentionally never invokes native code.
    self._native_graph = None
    self._adjacency_packed = None
    self._forest_children = [[] for _ in range(count)]
    self._forest_parents = np.full(count, -1, dtype=np.int64)
    parent = np.arange(count, dtype=np.int64)
    radius = self.cfg["normal_connectivity_m"]
    if count <= 4000:
        matrix = dense_graph(self.anchor, radius)
        self._adjacency_bits = [int.from_bytes(row.tobytes(), "little") for row in
                                np.packbits(matrix, axis=1, bitorder="little")]
        unseen = set(range(count))
        while unseen:
            seed = min(unseen)
            reached, pending = {seed}, {seed}
            while pending:
                current = min(pending)
                pending.remove(current)
                added = set(np.flatnonzero(matrix[current]).tolist()) - reached
                for child in added:
                    self._forest_parents[child] = current
                reached.update(added)
                pending.update(added)
            parent[list(reached)] = seed
            unseen.difference_update(reached)
        return parent

    # Preserve the old >4000 ordered union-find labels and child lists, not
    # merely equivalent partitions. This branch intentionally has no matrix.
    self._adjacency_bits = None
    buckets = {}
    for index, key in enumerate(np.floor(self.anchor / radius).astype(np.int64)):
        buckets.setdefault(tuple(key), []).append(index)

    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for key, members in buckets.items():
        candidates = list(members)
        for dx, dy in ((-1, -1), (-1, 0), (-1, 1), (0, -1)):
            candidates.extend(buckets.get((key[0] + dx, key[1] + dy), ()))
        if len(members) > 512 or len(candidates) > 2048:
            raise ValueError("normal connectivity bucket capacity exceeded")
        members, candidates = np.asarray(members), np.asarray(candidates)
        delta = self.anchor[members, None, :] - self.anchor[candidates][None, :, :]
        rows, cols = np.nonzero(np.sum(delta * delta, axis=2) <= radius ** 2)
        for left, right in zip(members[rows], candidates[cols]):
            a, b = root(int(left)), root(int(right))
            if a != b:
                parent[b] = a
                self._forest_children[int(left)].append(int(right))
    return np.asarray([root(index) for index in range(count)], dtype=np.int64)


def build_graph(points):
    index = object.__new__(module._FrozenIndex)
    index.anchor, index.cfg = points, module._configuration(None)
    index.components = index._components()
    # Mirror __init__'s post-graph bookkeeping before testing _local_support.
    index._component_sizes = np.bincount(index.components, minlength=len(points))
    return index


def assert_graph_exact(actual, expected):
    assert actual.components.tobytes() == expected.components.tobytes()
    assert actual._adjacency_bits == expected._adjacency_bits
    assert actual._forest_parents.tobytes() == expected._forest_parents.tobytes()
    assert actual._forest_children == expected._forest_children


def assert_matches_reference(points):
    before = points.tobytes()
    actual = build_graph(points)
    expected = SimpleNamespace(anchor=points, cfg=actual.cfg)
    expected.components = reference_components(expected)
    assert_graph_exact(actual, expected)
    assert points.tobytes() == before
    return actual


@pytest.mark.parametrize("position", ["inside", "on", "outside"])
@pytest.mark.parametrize("origin", [0., -.22, .22, -.44])
def test_radius_nextafter_and_negative_bucket_boundaries(position, origin):
    radius = .22
    value = {"inside": np.nextafter(radius, -np.inf), "on": radius,
             "outside": np.nextafter(radius, np.inf)}[position]
    points = np.array([[origin, 0.], [origin + value, 0.],
                       [origin, value], [origin + value, value]])
    assert_matches_reference(points)


def test_empty_graph_keeps_original_empty_arrays():
    result = assert_matches_reference(np.empty((0, 2)))
    assert result.components.dtype == np.int64
    assert result._adjacency_bits == []


@pytest.mark.parametrize("count", [1, 17, 129, 700, 4000, 4001])
def test_shuffled_sparse_graph_and_exact_large_graph_fallback(count):
    points = np.random.RandomState(10429 + count).uniform(-15., 15., (count, 2))
    result = assert_matches_reference(points)
    if count > 4000:
        assert result._adjacency_bits is None
        np.testing.assert_array_equal(result._forest_parents, np.full(count, -1))
    else:
        assert len(result._adjacency_bits) == count


@pytest.mark.parametrize("bridge_x", [.6, 3.])
def test_near_remote_u_induced_support_cannot_follow_removed_bridge(bridge_x):
    x = np.arange(0., 3.001, .1)
    points = np.vstack((np.c_[x, np.zeros(len(x))], np.c_[x, np.full(len(x), .4)],
                        [[bridge_x, .1], [bridge_x, .2], [bridge_x, .3]]))
    order = np.random.RandomState(44).permutation(len(points))
    points = points[order]
    actual = assert_matches_reference(points)
    adjacency = dense_graph(points, .22)
    targets = np.flatnonzero(points[:, 0] == 0.)
    ids = np.arange(len(points), dtype=np.int64)
    delta = points[None, :, :] - points[targets, None, :]
    allowed = np.sum(delta * delta, axis=2) <= 1.35 ** 2
    allowed &= points[None, :, 0] < .5  # remove even the near visible bridge
    expected = np.zeros_like(allowed)
    for row, seed in enumerate(targets):
        reached, pending = {int(seed)}, [int(seed)]
        while pending:
            current = pending.pop()
            added = set(np.flatnonzero(adjacency[current] & allowed[row])) - reached
            reached.update(added)
            pending.extend(added)
        expected[row, list(reached)] = True
    observed = actual._local_support(targets, ids, allowed.copy())
    assert observed.tobytes() == expected.tobytes()
    for row, seed in enumerate(targets):
        assert np.all(points[observed[row], 1] == points[seed, 1])


@pytest.mark.parametrize("members", [255, 256, 257, 512])
def test_distance_temporaries_never_exceed_65536_pairs(monkeypatch, members):
    # One original rectangular query straddles the 65536-pair boundary.
    # Observe the actual temporary passed to sum, not source-code spelling.
    points = np.zeros((members, 2))
    original_sum = module.np.sum
    observed_sizes = []

    def tracked_sum(values, *args, **kwargs):
        axis = kwargs.get("axis", args[0] if args else None)
        if values.ndim == 2 and values.shape[1] == 2 and axis == 1:
            observed_sizes.append(len(values))
        return original_sum(values, *args, **kwargs)

    monkeypatch.setattr(module.np, "sum", tracked_sum)
    result = build_graph(points)
    assert observed_sizes
    assert max(observed_sizes) <= 65536
    assert sum(observed_sizes) == members * members
    if members == 256:
        assert observed_sizes == [65536]
    if members > 256:
        assert len(observed_sizes) > 1
    assert result._adjacency_bits == [(1 << members) - 1] * members
    np.testing.assert_array_equal(result.components, np.zeros(members, dtype=np.int64))
    np.testing.assert_array_equal(result._forest_parents, [-1] + [0] * (members - 1))


@pytest.mark.parametrize("kind", ["members", "candidates"])
def test_original_bucket_capacity_fail_closed(kind):
    if kind == "members":
        points = np.zeros((513, 2))
    else:
        points = np.concatenate([
            np.tile([a * .22 + .01, b * .22 + .01], (n, 1))
            for a, b, n in ((0, 0, 512), (-1, -1, 512), (-1, 0, 512),
                            (-1, 1, 512), (0, -1, 1))])
    with pytest.raises(ValueError, match="normal connectivity bucket capacity exceeded"):
        build_graph(points)


@pytest.mark.parametrize("layout", ["thick", "clipped", "permuted"])
def test_full_index_geometry_is_identical_to_unbatched_all_pairs_graph(monkeypatch, layout):
    x = np.arange(-1., 2.01, .1)
    points = np.vstack([np.c_[x, .08 * x * x + y] for y in (-.08, 0., .08)])
    if layout == "clipped":
        points = points[points[:, 0] < .65]
    elif layout == "permuted":
        points = points[np.random.RandomState(88).permutation(len(points))]
    cfg = module._configuration(None)
    actual = module._FrozenIndex(points, cfg)
    monkeypatch.setattr(module._FrozenIndex, "_components", reference_components)
    expected = module._FrozenIndex(points, cfg)
    assert_graph_exact(actual, expected)
    for name in ("normals", "line_points", "normal_valid", "normal_radii",
                 "normal_thickness", "normal_position_support", "direction_borrowed"):
        assert getattr(actual, name).tobytes() == getattr(expected, name).tobytes(), name
    assert actual.patch_truncations == expected.patch_truncations
