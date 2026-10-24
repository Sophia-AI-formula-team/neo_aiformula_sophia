"""Final local-support counts and skipped impossible PCA rows, exact oracles.

The three methods in FrozenR4Index are copied verbatim from release
d1c5d33df28f9dea4fcd8e814c6e9a5f93c3eef8, Git blob 111d61fc8e481ef67a3868447547c756f5e111a5.
They freeze the prior PCA, past-hint and support-mask computation without
requiring Git history in CI. Graph construction, nearest and patch selection
are inherited (not changed by this optimization); an independent candidate-
graph BFS separately verifies connectivity/counts. No clock or gates change.
These are unit/equivalence tests, not runtime-performance acceptance.
"""
import math
from pathlib import Path
import sys
from types import MappingProxyType

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from lane_mapping_lya_reference import mask_localization as production


class FrozenR4Index(production._FrozenIndex):
    def __init__(self, anchor, cfg, normal_targets=None, normal_queries=None, direction_support=None,
                 reuse_index=None):
        self.anchor = anchor
        # Cache equality must compare construction-time settings, not a caller's mutable alias.
        self.cfg = MappingProxyType(dict(cfg))
        self.cell = cfg["hash_cell_m"]
        self.cells = {}
        keys = np.floor(anchor / self.cell).astype(np.int64)
        for index, key in enumerate(keys):
            self.cells.setdefault((int(key[0]), int(key[1])), []).append(index)
        self._cell_geometry = {key: anchor[indices].tobytes() for key, indices in self.cells.items()}
        self.components = self._components()
        self._component_sizes = np.bincount(self.components, minlength=len(anchor))
        self.normals = np.zeros_like(anchor)
        self.line_points = anchor.copy()
        self.normal_valid = np.zeros(len(anchor), dtype=bool)
        self.normal_radii = np.zeros(len(anchor), dtype=np.float64)
        self.normal_thickness = np.zeros(len(anchor), dtype=np.float64)
        self.normal_position_support = np.zeros(len(anchor), dtype=np.int64)
        self.direction_supported_points = 0
        self.direction_borrowed = np.zeros(len(anchor), dtype=bool)
        self.reused_geometry_points = 0
        self.patch_truncations = 0
        self._patch_cache = {}
        self.query_nearest = None
        if normal_queries is not None:
            found = self.nearest(normal_queries, valid_only=False)
            self.query_nearest = found
            normal_targets = np.unique(found[found >= 0])
        requested = np.ones(len(anchor), dtype=bool)
        if normal_targets is not None:
            requested[:] = False
            requested[np.asarray(normal_targets, dtype=np.int64)] = True
        cached = np.zeros(len(anchor), dtype=bool)
        if reuse_index is not None and reuse_index.cfg == cfg:
            for key, all_indices in self.cells.items():
                if self._cell_geometry[key] != reuse_index._cell_geometry.get(key):
                    continue
                current = np.asarray(all_indices, dtype=np.int64)
                previous = np.asarray(reuse_index.cells[key], dtype=np.int64)
                eligible = (~self.direction_borrowed[current] & ~reuse_index.direction_borrowed[previous]
                            & reuse_index.computed_normals[previous])
                dependencies = np.where(reuse_index.normal_valid[previous],
                                        reuse_index.normal_radii[previous], cfg["normal_max_radius_m"])
                for radius in np.unique(dependencies[eligible]):
                    reach = int(math.ceil(radius / self.cell))
                    # A previously valid smallest-scale fit depends only on
                    # that radius and all smaller attempted radii. Invalid
                    # geometry depends on the entire maximum neighborhood.
                    # Compare complete cells, including patch-truncation input.
                    same = all(self._cell_geometry.get((key[0] + dx, key[1] + dy))
                               == reuse_index._cell_geometry.get((key[0] + dx, key[1] + dy))
                               for dx in range(-reach, reach + 1) for dy in range(-reach, reach + 1))
                    if not same:
                        continue
                    subset = eligible & (dependencies == radius)
                    new, old = current[subset], previous[subset]
                    for name in ("normals", "line_points", "normal_valid", "normal_radii",
                                 "normal_thickness", "normal_position_support"):
                        getattr(self, name)[new] = getattr(reuse_index, name)[old]
                    cached[new] = True
                    self.reused_geometry_points += len(new)
        radii = [cfg["normal_radius_m"]]
        while radii[-1] < cfg["normal_max_radius_m"]:
            radii.append(min(radii[-1] * 2.0, cfg["normal_max_radius_m"]))
        # Choose the SMALLEST scale passing the unchanged line-support test.
        # Thin lane marks retain small-scale geometry. A rasterized/accumulated
        # wide band may require a larger neighborhood to reveal its long axis.
        # Isotropic patches and intersections are not rescued by relaxing ratio.
        for radius in radii:
            for key, all_indices in self.cells.items():
                indices = np.asarray(all_indices, dtype=np.int64)
                indices = indices[requested[indices] & ~self.normal_valid[indices] & ~cached[indices]]
                if not len(indices):
                    continue
                candidate_indices = self._patch(key, radius)
                if len(candidate_indices) < cfg["normal_min_points"]:
                    continue
                targets = anchor[indices]
                candidates = anchor[candidate_indices]
                difference = candidates[None, :, :] - targets[:, None, :]
                support = ((np.sum(difference * difference, axis=2) <= radius * radius)
                           & (self.components[indices, None] == self.components[candidate_indices][None, :]))
                support = self._local_support(indices, candidate_indices, support)
                weights = support.astype(np.float64)
                counts = np.sum(weights, axis=1)
                safe_count = np.maximum(counts, 1.0)
                means = np.sum(difference * weights[:, :, None], axis=1) / safe_count[:, None]
                centered = difference - means[:, None, :]
                covariance = np.einsum("nki,nkj,nk->nij", centered, centered, weights) / safe_count[:, None, None]
                eigenvalues, eigenvectors = np.linalg.eigh(covariance)
                valid = ((counts >= cfg["normal_min_points"]) &
                         (eigenvalues[:, 1] > 1e-5) &
                         (eigenvalues[:, 0] <= cfg["normal_max_thickness_m"] ** 2) &
                         (eigenvalues[:, 0] <= cfg["normal_max_variance_ratio"] * eigenvalues[:, 1]))
                chosen = indices[valid]
                normals = eigenvectors[valid, :, 0]
                self.normals[chosen] = normals
                self.normal_valid[chosen] = True
                self.normal_radii[chosen] = radius
                self.normal_thickness[chosen] = np.sqrt(np.maximum(eigenvalues[valid, 0], 0.0))
                self.normal_position_support[chosen] = counts[valid].astype(np.int64)
                # Match the local band center in its normal direction, not any
                # arbitrarily nearest pixel inside a thick, filled lane ribbon.
                offset = np.sum(means[valid] * normals, axis=1)
                self.line_points[chosen] = anchor[chosen] + normals * offset[:, None]
        if direction_support is not None:
            self._past_direction_support(direction_support)
        self.normals.flags.writeable = False
        self.normal_valid.flags.writeable = False
        self.line_points.flags.writeable = False
        self.normal_radii.flags.writeable = False
        self.normal_thickness.flags.writeable = False
        self.normal_position_support.flags.writeable = False
        self.direction_borrowed.flags.writeable = False
        self.computed_normals = requested | self.direction_borrowed | cached
        self.computed_normals.flags.writeable = False
        self.thickness_p95_m = (float(np.percentile(self.normal_thickness[self.normal_valid], 95))
                                if np.any(self.normal_valid) else None)

    def _past_direction_support(self, support):
        """Use past paint direction without importing its unconfirmed position.

        The caller owns the strictly-past source frontier. Only normals may be
        borrowed. Line centers/thickness and all matching positions remain
        calculated from the original CONFIRMED anchor, within its own component.
        """
        cfg = self.cfg
        unresolved = np.flatnonzero(~self.normal_valid)
        if not len(unresolved):
            return
        # Past low-reliability paint may fill a missing direction, never replace
        # geometry already observable from confirmed map points. Unconditional
        # replacement caused self-reinforcing heading/lateral drift in replay.
        hint = _FrozenIndex(support, cfg, normal_queries=self.anchor[unresolved])
        nearest = np.full(len(self.anchor), -1, dtype=np.int64)
        nearest[unresolved] = hint.query_nearest
        covered = nearest >= 0
        covered[covered] &= hint.normal_valid[nearest[covered]]
        covered[covered] &= np.linalg.norm(self.anchor[covered] - support[nearest[covered]], axis=1) <= cfg["normal_connectivity_m"]
        self.direction_supported_points = 0
        for key, all_indices in self.cells.items():
            indices = np.asarray(all_indices, dtype=np.int64)
            indices = indices[covered[indices]]
            if not len(indices):
                continue
            candidates = self._patch(key, cfg["normal_max_radius_m"])
            difference = self.anchor[candidates][None, :, :] - self.anchor[indices, None, :]
            support_mask = ((np.sum(difference * difference, axis=2) <= cfg["normal_max_radius_m"] ** 2)
                            & (self.components[indices, None] == self.components[candidates][None, :]))
            support_mask = self._local_support(indices, candidates, support_mask)
            counts = np.sum(support_mask, axis=1)
            normals = hint.normals[nearest[indices]]
            projected = np.einsum("nki,ni->nk", difference, normals)
            mean = np.sum(projected * support_mask, axis=1) / np.maximum(counts, 1)
            variance = np.sum((projected - mean[:, None]) ** 2 * support_mask, axis=1) / np.maximum(counts, 1)
            valid = ((counts >= cfg["normal_min_points"])
                     & (variance <= cfg["normal_max_thickness_m"] ** 2))
            selected = indices[valid]
            self.normals[selected] = normals[valid]
            self.direction_borrowed[selected] = True
            self.normal_valid[selected] = True
            self.normal_thickness[selected] = np.sqrt(np.maximum(variance[valid], 0.))
            self.normal_position_support[selected] = counts[valid]
            self.normal_radii[selected] = cfg["normal_max_radius_m"]
            self.line_points[selected] = self.anchor[selected] + normals[valid] * mean[valid, None]
            self.direction_supported_points += len(selected)

    def _local_support(self, targets, candidates, allowed):
        """Component containing each seed in its own radius-induced graph.

        A remote U-shaped bridge must not join two locally separate paint bands.
        Compact Python integer bitsets bound memory to the existing patch cap;
        only vertices inside each query circle may be traversed. This is exact
        connectivity, not a truncated flood-fill that could invent a thin line.
        """
        incomplete = np.flatnonzero(np.sum(allowed, axis=1) != self._component_sizes[self.components[targets]])
        if not len(incomplete):
            return allowed
        if self._adjacency_bits is not None:
            positions = np.full(len(self.anchor), -1, dtype=np.int64)
            positions[candidates] = np.arange(len(candidates))
            parents = self._forest_parents[candidates]
            edges = np.flatnonzero(parents >= 0)
            edges = edges[positions[parents[edges]] >= 0]
            parent_positions = positions[parents[edges]]
            counts = np.sum(allowed[incomplete], axis=1)
            edge_count = np.sum(allowed[incomplete[:, None], edges]
                                & allowed[incomplete[:, None], parent_positions], axis=1)
            incomplete = incomplete[edge_count != counts - 1]
            if not len(incomplete):
                return allowed
            # Live windows are <=4000 points: share their exact sparse graph
            # encoded as at most 2 MB of bitsets across every PCA radius/query.
            # Larger full-map indexes retain the bounded local-patch fallback.
            size = len(self.anchor)
            global_allowed = np.zeros((len(incomplete), size), dtype=bool)
            global_allowed[:, candidates] = allowed[incomplete]
            masks = np.packbits(global_allowed, axis=1, bitorder="little")
            for number, row in enumerate(incomplete):
                seed = int(targets[row])
                if not global_allowed[number, seed]:
                    allowed[row] = False
                    continue
                mask = int.from_bytes(masks[number].tobytes(), "little")
                reached = frontier = 1 << seed
                while frontier and reached != mask:
                    bit = frontier & -frontier
                    frontier ^= bit
                    added = self._adjacency_bits[bit.bit_length() - 1] & mask & ~reached
                    reached |= added
                    frontier |= added
                bits = np.unpackbits(np.frombuffer(reached.to_bytes((size + 7) // 8, "little"), dtype=np.uint8),
                                     bitorder="little")
                allowed[row] = bits[candidates]
            return allowed
        count = len(candidates)
        positions = {int(index): position for position, index in enumerate(candidates)}
        # Cheap exact certificate for ordinary continuous bands: an induced
        # subset of a spanning forest is connected iff it retains n-1 edges.
        # Only uncertified rows need flood-fill; this cannot admit a U bridge
        # outside the query circle, since both endpoints must be allowed.
        forest = [(position, positions[other]) for position, index in enumerate(candidates)
                  for other in self._forest_children[int(index)] if other in positions]
        if forest:
            edges = np.asarray(forest, dtype=np.int64)
            connected_count = np.sum(allowed[incomplete[:, None], edges[:, 0]]
                                     & allowed[incomplete[:, None], edges[:, 1]], axis=1)
            needed = np.sum(allowed[incomplete], axis=1) - 1
            incomplete = incomplete[connected_count != needed]
            if not len(incomplete):
                return allowed
        adjacency = []
        points = self.anchor[candidates]
        for start in range(0, count, 64):
            delta = points[start:start + 64, None, :] - points[None, :, :]
            connected = np.sum(delta * delta, axis=2) <= self.cfg["normal_connectivity_m"] ** 2
            adjacency.extend(int.from_bytes(row.tobytes(), "little") for row in
                             np.packbits(connected, axis=1, bitorder="little"))
        byte_count = (count + 7) // 8
        for row in incomplete:
            seed = positions.get(int(targets[row]))
            if seed is None or not allowed[row, seed]:
                allowed[row] = False
                continue
            mask = int.from_bytes(np.packbits(allowed[row], bitorder="little").tobytes(), "little")
            reached = frontier = 1 << seed
            while frontier and reached != mask:
                bit = frontier & -frontier
                frontier ^= bit
                added = adjacency[bit.bit_length() - 1] & mask & ~reached
                reached |= added
                frontier |= added
            allowed[row] = np.unpackbits(np.frombuffer(reached.to_bytes(byte_count, "little"), dtype=np.uint8),
                                         bitorder="little")[:count]
        return allowed


# The frozen hint method must recursively construct its own frozen oracle,
# not silently call the new production constructor.
_FrozenIndex = FrozenR4Index

GEOMETRY = ("anchor", "components", "_component_sizes", "_forest_parents",
            "normals", "line_points", "normal_valid", "normal_radii",
            "normal_thickness", "normal_position_support", "direction_borrowed",
            "computed_normals")


def exact_array(actual, expected):
    assert actual.dtype == expected.dtype
    assert actual.shape == expected.shape
    assert actual.tobytes() == expected.tobytes()


def assert_geometry_exact(actual, expected):
    for name in GEOMETRY:
        exact_array(getattr(actual, name), getattr(expected, name))
    assert actual._adjacency_bits == expected._adjacency_bits
    assert actual._forest_children == expected._forest_children
    assert actual.direction_supported_points == expected.direction_supported_points
    assert actual.patch_truncations == expected.patch_truncations
    assert actual.reused_geometry_points == expected.reused_geometry_points
    assert actual.thickness_p95_m == expected.thickness_p95_m
    if expected.query_nearest is None:
        assert actual.query_nearest is None
    else:
        exact_array(actual.query_nearest, expected.query_nearest)


def graph(points):
    value = object.__new__(production._FrozenIndex)
    value.anchor = np.asarray(points, dtype=np.float64)
    value.cfg = production._configuration(None)
    value.components = value._components()
    value._component_sizes = np.bincount(value.components, minlength=len(value.anchor))
    return value


def bfs_support(points, targets, candidates, allowed, radius):
    """Independent induced graph using ordinary Python sets, no index shortcuts."""
    cloud = points[candidates]
    delta = cloud[:, None, :] - cloud[None, :, :]
    adjacency = np.sum(delta * delta, axis=2) <= radius ** 2
    positions = {int(vertex): position for position, vertex in enumerate(candidates)}
    result = np.zeros_like(allowed)
    for row, target in enumerate(targets):
        seed = positions.get(int(target))
        if seed is None or not allowed[row, seed]:
            continue
        seen, pending = {seed}, [seed]
        while pending:
            current = pending.pop()
            for other in np.flatnonzero(adjacency[current] & allowed[row]):
                other = int(other)
                if other not in seen:
                    seen.add(other)
                    pending.append(other)
        result[row, list(seen)] = True
    return result, np.sum(result, axis=1, dtype=np.int64)


def assert_support_counts(value, targets, candidates, allowed, compare_frozen=True):
    targets, candidates = np.asarray(targets, dtype=np.int64), np.asarray(candidates, dtype=np.int64)
    allowed = np.asarray(allowed, dtype=bool)
    expected, expected_counts = bfs_support(value.anchor, targets, candidates, allowed,
                                             value.cfg["normal_connectivity_m"])
    observed, counts = value._local_support(targets, candidates, allowed.copy(), return_counts=True)
    assert observed.dtype == np.bool_
    assert counts.dtype == np.int64 and counts.shape == (len(targets),)
    exact_array(observed, expected)
    exact_array(counts, expected_counts)
    default = value._local_support(targets, candidates, allowed.copy())
    assert isinstance(default, np.ndarray)
    exact_array(default, observed)
    if compare_frozen:
        exact_array(FrozenR4Index._local_support(value, targets, candidates, allowed.copy()), expected)
    return observed, counts


@pytest.mark.parametrize("count", [5, 6, 7])
def test_whole_component_count_and_default_return(count):
    points = np.c_[np.arange(count) * .1, np.zeros(count)]
    index = graph(points)
    mask = np.ones((count, count), dtype=bool)
    _, counts = assert_support_counts(index, np.arange(count), np.arange(count), mask)
    exact_array(counts, np.full(count, count, dtype=np.int64))


def test_empty_target_rows_and_empty_candidates_have_final_zero_counts():
    value = graph(np.array([[0., 0.], [.1, 0.]]))
    assert_support_counts(value, [], [0, 1], np.empty((0, 2), dtype=bool))
    assert_support_counts(value, [0, 1], [], np.empty((2, 0), dtype=bool))
    assert_support_counts(value, [0, 1], [0, 1], np.zeros((2, 2), dtype=bool))


@pytest.mark.parametrize("missing", ["not_a_candidate", "candidate_but_not_allowed"])
def test_missing_seed_cannot_return_another_connected_component(missing):
    # The remaining three vertices are connected; this probes the forest
    # certificate as well as the final flood-fill missing-seed branch.
    value = graph(np.c_[np.arange(4) * .1, np.zeros(4)])
    candidates = np.arange(3 if missing == "not_a_candidate" else 4)
    allowed = np.ones((1, len(candidates)), dtype=bool)
    if missing == "candidate_but_not_allowed":
        allowed[0, 3] = False
    # This is an explicitly authorized safety correction, not numerical
    # equivalence to R4's erroneous early forest-certificate acceptance.
    old = FrozenR4Index._local_support(value, np.array([3]), candidates, allowed.copy())
    assert np.sum(old) == 3
    _, counts = assert_support_counts(value, [3], candidates, allowed, compare_frozen=False)
    exact_array(counts, np.array([0], dtype=np.int64))


def u_shape():
    x = np.arange(0., 3.001, .1)
    return np.vstack((np.c_[x, np.zeros(len(x))], np.c_[x, np.full(len(x), .4)],
                      [[3., .1], [3., .2], [3., .3]]))


@pytest.mark.parametrize("total", [65, 4000, 4001])
def test_u_bridge_final_counts_remove_disconnected_local_band(total):
    base = u_shape()
    assert len(base) == 65
    remote = np.c_[100. + np.arange(total - len(base)) * .4,
                   np.full(total - len(base), 20.)]
    points = np.vstack((base, remote))
    value = graph(points)
    assert (value._adjacency_bits is None) == (total > 4000)
    targets, candidates = np.array([0, 31]), np.arange(len(base))
    allowed = np.tile(base[:, 0] < .5, (2, 1))
    observed, counts = assert_support_counts(value, targets, candidates, allowed)
    exact_array(counts, np.array([5, 5], dtype=np.int64))
    for row, seed in enumerate(targets):
        assert np.all(base[observed[row], 1] == base[seed, 1])


@pytest.mark.parametrize("position", ["inside", "on", "outside"])
def test_normal_radius_nextafter_support_is_exact(position):
    radius = .45
    edge = {"inside": np.nextafter(radius, -np.inf), "on": radius,
            "outside": np.nextafter(radius, np.inf)}[position]
    points = np.c_[[0., .1, .2, .3, .4, edge], np.zeros(6)]
    value = graph(points)
    delta = points[None, :, :] - points[[0], None, :]
    allowed = np.sum(delta * delta, axis=2) <= radius ** 2
    _, counts = assert_support_counts(value, [0], np.arange(6), allowed)
    assert counts[0] == (5 if position == "outside" else 6)


def test_patch_truncation_uses_only_candidates_and_never_fabricates_a_seed():
    x, y = np.meshgrid(np.arange(25) * .02, np.arange(25) * .02)
    points = np.c_[x.ravel(), y.ravel()]
    cfg = production._configuration(None)
    value = production._FrozenIndex(points, cfg, normal_targets=np.empty(0, dtype=np.int64))
    candidates = value._patch((0, 0), cfg["normal_radius_m"])
    assert len(candidates) == cfg["max_patch_points"] == 512
    missing = np.setdiff1d(np.arange(len(points)), candidates)
    assert len(missing) >= 2
    targets = np.r_[missing[:2], candidates[:2]]
    assert not np.any(np.isin(targets[:2], candidates))
    delta = points[candidates][None, :, :] - points[targets, None, :]
    allowed = np.sum(delta * delta, axis=2) <= cfg["normal_radius_m"] ** 2
    observed, counts = assert_support_counts(value, targets, candidates, allowed)
    assert not np.any(observed[:2])
    exact_array(counts[:2], np.array([0, 0], dtype=np.int64))


def pca_fixture(kind):
    if kind in ("five", "six", "seven"):
        count = {"five": 5, "six": 6, "seven": 7}[kind]
        return np.c_[np.arange(count) * .08, np.zeros(count)]
    if kind == "mixed":
        return np.vstack((np.c_[np.arange(7) * .06, np.zeros(7)],
                          np.c_[np.arange(5) * .06, np.full(5, .3)]))
    if kind == "multiscale":
        return np.c_[np.arange(9) * .15, np.zeros(9)]
    if kind == "u":
        return u_shape()
    if kind == "isotropic":
        x, y = np.meshgrid(np.arange(4) * .08, np.arange(4) * .08)
        return np.c_[x.ravel(), y.ravel()]
    if kind == "truncated":
        x, y = np.meshgrid(np.arange(25) * .02, np.arange(25) * .02)
        return np.c_[x.ravel(), y.ravel()]
    if kind in ("inside", "on", "outside"):
        edge = {"inside": np.nextafter(.45, -np.inf), "on": .45,
                "outside": np.nextafter(.45, np.inf)}[kind]
        return np.c_[[0., .1, .2, .3, .4, edge], np.zeros(6)]
    raise ValueError(kind)


@pytest.mark.parametrize("kind", ["five", "six", "seven", "mixed", "multiscale", "u",
                                  "isotropic", "truncated", "inside", "on", "outside"])
def test_all_geometry_arrays_match_frozen_r4_pca(kind):
    points = pca_fixture(kind)
    before = points.tobytes()
    cfg = production._configuration(None)
    actual, expected = production._FrozenIndex(points, cfg), FrozenR4Index(points, cfg)
    assert_geometry_exact(actual, expected)
    assert points.tobytes() == before
    if kind == "five":
        assert not np.any(actual.normal_valid)
    if kind in ("six", "seven"):
        assert np.all(actual.normal_valid)
    if kind in ("multiscale", "outside"):
        assert np.any(actual.normal_radii > cfg["normal_radius_m"])


@pytest.mark.parametrize("total", [4000, 4001])
def test_large_graph_geometry_target_subset_matches_frozen_r4(total):
    base = pca_fixture("mixed")
    remote = np.c_[100. + np.arange(total - len(base)) * .4,
                   np.full(total - len(base), 20.)]
    points = np.vstack((base, remote))
    targets = np.arange(len(base))
    cfg = production._configuration(None)
    actual = production._FrozenIndex(points, cfg, normal_targets=targets)
    expected = FrozenR4Index(points, cfg, normal_targets=targets)
    assert_geometry_exact(actual, expected)


@pytest.mark.parametrize("count", [5, 6, 7])
@pytest.mark.parametrize("mode", ["all", "targets", "queries", "cache"])
def test_hint_and_cached_geometry_remain_exact_with_insufficient_rows(count, mode):
    angle = np.arange(count) * (2. * np.pi / count)
    # Isotropic confirmed samples need a hint; a separate valid lane keeps
    # enough/insufficient rows mixed within the same constructor.
    points = np.vstack((np.c_[.07 * np.cos(angle), .07 * np.sin(angle)],
                        np.c_[np.arange(7) * .08, np.full(7, .3)]))
    x = np.arange(-1., 1.001, .05)
    hint = np.vstack((np.c_[x, np.zeros(len(x))], np.c_[x, np.full(len(x), .3)]))
    cfg = production._configuration(None)
    actual_kwargs, expected_kwargs = {"direction_support": hint}, {"direction_support": hint}
    if mode == "targets":
        actual_kwargs["normal_targets"] = expected_kwargs["normal_targets"] = np.arange(count)
    elif mode == "queries":
        actual_kwargs["normal_queries"] = expected_kwargs["normal_queries"] = points[:count]
    elif mode == "cache":
        actual_kwargs["reuse_index"] = production._FrozenIndex(points.copy(), cfg)
        expected_kwargs["reuse_index"] = FrozenR4Index(points.copy(), cfg)
    actual = production._FrozenIndex(points.copy(), cfg, **actual_kwargs)
    expected = FrozenR4Index(points.copy(), cfg, **expected_kwargs)
    assert_geometry_exact(actual, expected)
    if count == 5:
        assert not np.any(actual.direction_borrowed[:count])


def test_frozen_oracle_does_not_call_new_count_return_path():
    import inspect
    assert "return_counts" not in inspect.getsource(FrozenR4Index.__init__)
    assert "return_counts" not in inspect.getsource(FrozenR4Index._local_support)
    assert "return_counts" not in inspect.getsource(FrozenR4Index._past_direction_support)
