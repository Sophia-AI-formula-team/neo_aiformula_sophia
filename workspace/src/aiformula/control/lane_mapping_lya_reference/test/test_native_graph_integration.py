"""Native graph integration against the unchanged Python geometry backend.

These are exact geometry/ownership tests, not performance or driving evidence.
Source-only pytest may skip native comparisons. LANE_REQUIRE_NATIVE=1 instead
fails when the extension is absent; importing the module alone is not a pass.
"""
import importlib
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from lane_mapping_lya_reference import mask_localization as module


ARRAYS = ("anchor", "components", "_forest_parents", "_component_sizes",
          "normals", "line_points", "normal_valid", "normal_radii",
          "normal_thickness", "normal_position_support", "direction_borrowed",
          "computed_normals")


@pytest.fixture(scope="module")
def native():
    if os.environ.get("LANE_REQUIRE_NATIVE") == "1":
        try:
            result = importlib.import_module("_lane_mapping_lya_native")
        except ImportError as error:
            pytest.fail("LANE_REQUIRE_NATIVE=1 but native import failed: {}".format(error))
    else:
        result = pytest.importorskip("_lane_mapping_lya_native")
    assert result.API_VERSION == 1
    assert result.RADIUS_GRAPH_API_VERSION == 1
    assert result.LOCAL_SUPPORT_API_VERSION == 1
    assert result.NEAREST_API_VERSION == 1
    assert result.CROSS_SECTION_WEIGHTS_API_VERSION == 1
    assert callable(result.radius_graph)
    assert callable(result.prepare_graph)
    assert callable(result.graph_forest)
    assert callable(result.connected_subsets)
    assert callable(result.local_support)
    assert callable(result.nearest_grouped)
    assert callable(result.cross_section_weights)
    return result


class NativeSpy:
    """Forward to the real extension; record calls without replacing results."""
    def __init__(self, native):
        self.native = native
        self.radius_calls = []
        self.prepare_calls = []
        self.graph_calls = []
        self.subset_calls = []
        self.subset_graphs = []
        self.local_calls = []
        self.nearest_calls = []
        self.weight_calls = []

    def __getattr__(self, name):
        return getattr(self.native, name)

    def radius_graph(self, points, keys, count, radius_squared):
        self.radius_calls.append((points, keys, count, radius_squared))
        return self.native.radius_graph(points, keys, count, radius_squared)

    def prepare_graph(self, packed, count):
        result = self.native.prepare_graph(packed, count)
        self.prepare_calls.append((count, len(packed), id(result)))
        return result

    def graph_forest(self, graph, count):
        self.graph_calls.append((count, type(graph).__name__, id(graph)))
        return self.native.graph_forest(graph, count)

    def connected_subsets(self, graph, count, masks, seeds):
        self.subset_calls.append((count, len(masks), tuple(seeds)))
        self.subset_graphs.append((count, type(graph).__name__, id(graph)))
        return self.native.connected_subsets(graph, count, masks, seeds)

    def local_support(self, graph, count, candidates, targets, allowed, parents):
        self.local_calls.append((count, type(graph).__name__, id(graph),
                                 candidates, targets, allowed, parents))
        return self.native.local_support(graph, count, candidates, targets, allowed, parents)

    def nearest_grouped(self, points, anchor, query_offsets, query_ids,
                        candidate_offsets, candidate_ids, allowed, radius_squared):
        args = (points, anchor, query_offsets, query_ids, candidate_offsets,
                candidate_ids, allowed, radius_squared)
        self.nearest_calls.append(args)
        return self.native.nearest_grouped(*args)

    def cross_section_weights(self, graph, count, points, normals, ids, half_width, radius_squared):
        self.weight_calls.append((count, type(graph).__name__, id(graph), points,
                                  normals, ids, half_width, radius_squared))
        return self.native.cross_section_weights(graph, count, points, normals, ids,
                                                 half_width, radius_squared)


def build(monkeypatch, backend, points, cfg=None, **kwargs):
    with monkeypatch.context() as patch:
        patch.setattr(module, "_GRAPH_NATIVE", backend)
        return module._FrozenIndex(points.copy(), module._configuration(cfg), **kwargs)


def invoke(monkeypatch, backend, function, *args, **kwargs):
    with monkeypatch.context() as patch:
        patch.setattr(module, "_GRAPH_NATIVE", backend)
        return function(*args, **kwargs)


def assert_bytes(actual, expected, name="array"):
    assert actual.shape == expected.shape, name
    assert actual.dtype == expected.dtype, name
    assert actual.tobytes() == expected.tobytes(), name


def assert_index_exact(actual, expected):
    for name in ARRAYS:
        assert_bytes(getattr(actual, name), getattr(expected, name), name)
    assert actual._adjacency_bits == expected._adjacency_bits
    assert actual._adjacency_packed == expected._adjacency_packed
    assert actual._forest_children == expected._forest_children
    assert actual.cells == expected.cells
    assert actual.patch_truncations == expected.patch_truncations
    assert actual.direction_supported_points == expected.direction_supported_points
    assert actual.thickness_p95_m == expected.thickness_p95_m


def assert_one_radius_call(spy, points, cfg):
    """The C path receives original native-width bytes and Python's threshold."""
    assert len(spy.radius_calls) == 1, "float64 geometry must not quietly use the NumPy radius fallback"
    point_bytes, key_bytes, count, threshold = spy.radius_calls[0]
    assert type(point_bytes) is bytes
    assert type(key_bytes) is bytes
    assert points.dtype == np.float64
    assert point_bytes == points.tobytes()
    keys = np.floor(points / cfg["normal_connectivity_m"]).astype(np.int64)
    assert key_bytes == keys.tobytes()
    assert count == len(points)
    assert type(threshold) is float
    # The original empty graph performs no radius-square arithmetic; preserve
    # that behavior even for a finite radius whose square would overflow.
    expected_threshold = float(cfg["normal_connectivity_m"] ** 2) if len(points) else 0.0
    assert threshold.hex() == expected_threshold.hex()


def independent_radius_oracle(points, keys, radius_squared):
    """All pairs, no production buckets/batching/native oracles.

    The explicit key check is important: the original five-cell enumeration
    permits the symmetric 3x3 key neighborhood, not arbitrary close pairs.
    Python integers avoid overflow at signed int64 key endpoints.
    """
    difference = points[:, None, :] - points[None, :, :]
    connected = np.sum(difference * difference, axis=2) <= radius_squared
    for left in range(len(points)):
        for right in range(len(points)):
            if (abs(int(keys[left, 0]) - int(keys[right, 0])) > 1 or
                    abs(int(keys[left, 1]) - int(keys[right, 1])) > 1):
                connected[left, right] = False
    return np.packbits(connected, axis=1, bitorder="little").tobytes()


def line(y=0., start=-2., stop=2., step=.1):
    x = np.arange(start, stop + step * .25, step)
    return np.column_stack((x, np.full(len(x), y)))


def rectangle(x0, x1, y0, y1, step=.1):
    x, y = np.meshgrid(np.arange(x0, x1 + step * .25, step),
                       np.arange(y0, y1 + step * .25, step))
    return np.column_stack((x.ravel(), y.ravel()))


def u_shape(bridge_x):
    return np.vstack((line(-.3, 0., 3.), line(.3, 0., 3.),
                      [[bridge_x, -.15], [bridge_x, 0.], [bridge_x, .15]]))


def scene(name):
    if name == "empty":
        return np.empty((0, 2), dtype=np.float64)
    if name == "singleton":
        return np.array([[.1, -.2]])
    if name == "thin":
        return np.vstack((line(-1.), line(1.)))
    if name == "filled_band":
        return rectangle(-2., 2., -.3, .3)
    if name == "clipped_band":
        return rectangle(0., .6, -.3, .3)
    if name == "near_u":
        return u_shape(.6)
    if name == "remote_u":
        return u_shape(3.)
    if name == "random":
        return np.random.RandomState(20261011).uniform(-2., 2., (173, 2))
    raise AssertionError(name)


def test_python_source_only_backend_remains_executable(monkeypatch):
    # This test deliberately has no native fixture and runs without a compiler.
    result = build(monkeypatch, None, line())
    assert np.all(result.normal_valid)
    assert np.all(result.components == 0)
    assert result.normal_position_support.min() >= result.cfg["normal_min_points"]


@pytest.mark.parametrize("name", ["empty", "singleton", "thin", "filled_band",
                                  "clipped_band", "near_u", "remote_u", "random"])
def test_all_index_arrays_are_bit_exact(monkeypatch, native, name):
    points = scene(name)
    before = points.tobytes()
    expected = build(monkeypatch, None, points)
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points)
    assert_index_exact(actual, expected)
    assert_one_radius_call(spy, points, actual.cfg)
    assert type(actual._native_graph).__name__ == "PyCapsule"
    assert spy.prepare_calls == [(len(points), len(actual._adjacency_packed),
                                   id(actual._native_graph))]
    assert spy.graph_calls == [(len(points), "PyCapsule", id(actual._native_graph))]
    assert points.tobytes() == before


@pytest.mark.parametrize("position", ["inside", "on", "outside"])
@pytest.mark.parametrize("origin", [0., -.22])
def test_native_graph_keeps_radius_boundary_and_bucket_order(monkeypatch, native, position, origin):
    distance = {"inside": np.nextafter(.22, 0.), "on": .22,
                "outside": np.nextafter(.22, np.inf)}[position]
    points = np.array([[origin, 0.], [origin + distance, 0.],
                       [origin, distance], [origin + distance, distance]])[::-1].copy()
    assert_index_exact(build(monkeypatch, native, points), build(monkeypatch, None, points))


@pytest.mark.parametrize("bridge_x", [.6, 3.])
def test_induced_support_calls_native_and_cannot_cross_removed_u_bridge(monkeypatch, native, bridge_x):
    points = u_shape(bridge_x)
    expected = build(monkeypatch, None, points, normal_targets=np.empty(0, dtype=np.int64))
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points, normal_targets=np.empty(0, dtype=np.int64))
    candidates = np.arange(len(points), dtype=np.int64)
    targets = np.array([0, len(line(-.3, 0., 3.))], dtype=np.int64)
    assert np.array_equal(points[targets, 0], [0., 0.])
    allowed = np.tile(points[:, 0] < .5, (len(targets), 1))
    observed, counts = invoke(monkeypatch, spy, actual._local_support,
                              targets, candidates, allowed.copy(), return_counts=True)
    baseline, baseline_counts = invoke(monkeypatch, None, expected._local_support,
                                       targets, candidates, allowed.copy(), return_counts=True)
    assert_bytes(observed, baseline)
    assert_bytes(counts, baseline_counts)
    assert len(spy.local_calls) == 1, "the whole native local-support path must actually execute"
    assert spy.local_calls[0][:3] == (len(points), "PyCapsule", id(actual._native_graph))
    assert not spy.subset_calls, "native local support must not fall through to Python mask preparation"
    assert len(spy.prepare_calls) == 1
    assert all(call == (len(points), "PyCapsule", id(actual._native_graph))
               for call in spy.subset_graphs)
    for row, seed in enumerate(targets):
        assert np.all(points[observed[row], 1] == points[seed, 1])


@pytest.mark.parametrize("count", [4000, 4001])
def test_graph_capacity_boundary_preserves_large_index_python_fallback(monkeypatch, native, count):
    points = np.column_stack((np.arange(count) * .31, np.zeros(count)))
    targets = np.empty(0, dtype=np.int64)
    baseline = build(monkeypatch, None, points, normal_targets=targets)
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points, normal_targets=targets)
    assert_index_exact(actual, baseline)
    if count == 4000:
        assert_one_radius_call(spy, points, actual.cfg)
        assert len(spy.graph_calls) == 1
        assert spy.prepare_calls == [(count, len(actual._adjacency_packed),
                                      id(actual._native_graph))]
    else:
        assert not spy.radius_calls
        assert not spy.graph_calls
        assert not spy.prepare_calls
        assert actual._adjacency_bits is None
        assert actual._adjacency_packed is None
        candidates = np.arange(64, dtype=np.int64)
        allowed = np.ones((1, 64), dtype=bool)
        observed = invoke(monkeypatch, spy, actual._local_support,
                          np.array([0]), candidates, allowed.copy())
        expected = invoke(monkeypatch, None, baseline._local_support,
                          np.array([0]), candidates, allowed.copy())
        assert_bytes(observed, expected)
        assert np.count_nonzero(observed) == 1
        assert not spy.subset_calls
        assert not spy.local_calls
        with pytest.raises(ValueError, match="bounded graph capacity"):
            invoke(monkeypatch, spy, module._line_weights, points[:1],
                   np.array([[0., 1.]]), actual, np.array([0]), actual.cfg)


def test_real_512_point_patch_and_missing_seed_are_bit_exact(monkeypatch, native):
    points = rectangle(-1., 1., -1., 1., step=.05)
    selected = np.array([0, len(points) // 2, len(points) - 1], dtype=np.int64)
    baseline = build(monkeypatch, None, points, normal_targets=selected)
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points, normal_targets=selected)
    assert actual.cfg["max_patch_points"] == 512
    assert_index_exact(actual, baseline)
    candidates = actual._patch((0, 0), actual.cfg["normal_max_radius_m"])
    assert_bytes(candidates, baseline._patch((0, 0), baseline.cfg["normal_max_radius_m"]))
    assert len(candidates) == 512
    missing = np.setdiff1d(np.arange(len(points)), candidates)[0]
    allowed = np.ones((1, len(candidates)), dtype=bool)
    before_calls = len(spy.local_calls)
    observed, counts = invoke(monkeypatch, spy, actual._local_support,
                              np.array([missing]), candidates, allowed.copy(), return_counts=True)
    expected, expected_counts = invoke(monkeypatch, None, baseline._local_support,
                                       np.array([missing]), candidates, allowed.copy(), return_counts=True)
    assert_bytes(observed, expected)
    assert_bytes(counts, expected_counts)
    assert not np.any(observed)
    assert counts.tolist() == [0]
    assert len(spy.local_calls) == before_calls + 1


@pytest.mark.parametrize("kind", ["unique", "duplicates", "shifted_seed", "near_u", "remote_u", "batch"])
def test_cross_section_weights_use_native_with_exact_multiplicity(monkeypatch, native, kind):
    points = np.column_stack((np.zeros(9), np.arange(9) * .1))
    ids = np.array([0, 4, 8], dtype=np.int64)
    if kind == "duplicates":
        ids = np.array([0, 4, 4, 8, 8, 8], dtype=np.int64)
    elif kind in ("near_u", "remote_u"):
        points = u_shape(.6 if kind == "near_u" else 3.)
        ids = np.array([0, 31], dtype=np.int64)
    elif kind == "batch":
        points = rectangle(-.3, .3, -.3, .3, step=.05)
        ids = np.arange(71, dtype=np.int64)
        ids[-2:] = ids[:2]
    normals = np.tile([0., 1.], (len(ids), 1))
    reliable = points[ids].copy()
    if kind == "shifted_seed":
        reliable[:, 0] += .049
    baseline = build(monkeypatch, None, points, normal_targets=np.empty(0, dtype=np.int64))
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points, normal_targets=np.empty(0, dtype=np.int64))
    observed = invoke(monkeypatch, spy, module._line_weights, reliable, normals, actual, ids, actual.cfg)
    expected = invoke(monkeypatch, None, module._line_weights, reliable, normals, baseline, ids, baseline.cfg)
    assert_bytes(observed, expected)
    assert spy.weight_calls == [(len(points), "PyCapsule", id(actual._native_graph),
                                points.tobytes(), normals.tobytes(), ids.tobytes(),
                                actual.cfg["scan_voxel_m"] * .5,
                                actual.cfg["normal_max_radius_m"] ** 2)]
    assert not spy.subset_calls, "the new kernel owns the complete density operation"
    assert len(spy.prepare_calls) == 1, "one immutable graph must serve every density batch"
    assert not spy.subset_graphs
    if kind == "duplicates":
        np.testing.assert_array_equal(observed, np.full(len(ids), 1. / len(ids)))
    if kind in ("near_u", "remote_u"):
        np.testing.assert_array_equal(observed, [1., 1.])


@pytest.mark.parametrize("edge", ["slab", "radius"])
@pytest.mark.parametrize("position", ["inside", "on", "outside"])
def test_density_boundary_decisions_remain_exact(monkeypatch, native, edge, position):
    boundary = .05 if edge == "slab" else 1.35
    distance = {"inside": np.nextafter(boundary, 0.), "on": boundary,
                "outside": np.nextafter(boundary, np.inf)}[position]
    points = (np.array([[0., 0.], [distance, .1]]) if edge == "slab" else
              np.column_stack((np.zeros(10), np.append(np.arange(9) * .15, distance))))
    ids = np.array([0, len(points) - 1], dtype=np.int64)
    normals = np.tile([0., 1.], (2, 1))
    results = []
    for backend in (None, native):
        index = build(monkeypatch, backend, points, normal_targets=np.empty(0, dtype=np.int64))
        results.append(invoke(monkeypatch, backend, module._line_weights,
                              points[ids], normals, index, ids, index.cfg))
    assert_bytes(results[1], results[0])
    np.testing.assert_array_equal(results[1], np.full(2, 1. if position == "outside" else .5))


@pytest.mark.parametrize("change", ["same", "move", "clip"])
@pytest.mark.parametrize("old_native", [False, True])
def test_cache_can_cross_backend_boundary_without_geometry_or_source_mutation(monkeypatch, native, change, old_native):
    points = np.vstack((line(-1.), line(1.), line(1., 15., 19.)))
    previous = build(monkeypatch, native if old_native else None, points)
    before = {name: getattr(previous, name).tobytes() for name in ARRAYS}
    changed = points.copy()
    if change == "move":
        changed[10, 1] += .12
    elif change == "clip":
        changed = changed[5:]
    new_backend = None if old_native else native
    cached = build(monkeypatch, new_backend, changed, reuse_index=previous)
    fresh = build(monkeypatch, new_backend, changed)
    assert_index_exact(cached, fresh)
    assert cached.reused_geometry_points > 0
    for name in ARRAYS:
        assert getattr(previous, name).tobytes() == before[name], name
    for name in ARRAYS[4:]:
        assert not np.shares_memory(getattr(cached, name), getattr(previous, name)), name


@pytest.mark.parametrize("replacement", ["horizontal", "vertical", "absent"])
@pytest.mark.parametrize("old_native", [False, True])
def test_direction_hint_and_changed_context_match_across_backends(monkeypatch, native, replacement, old_native):
    confirmed = rectangle(-.15, .15, -.15, .15, step=.05)
    horizontal = line(step=.05)
    previous = build(monkeypatch, native if old_native else None, confirmed,
                     direction_support=horizontal)
    assert np.all(previous.direction_borrowed)
    before = {name: getattr(previous, name).tobytes() for name in ARRAYS}
    support = {"horizontal": horizontal, "vertical": horizontal[:, ::-1].copy(), "absent": None}[replacement]
    new_backend = None if old_native else native
    cached = build(monkeypatch, new_backend, confirmed, reuse_index=previous, direction_support=support)
    fresh = build(monkeypatch, new_backend, confirmed, direction_support=support)
    oracle = build(monkeypatch, None, confirmed, direction_support=support)
    assert_index_exact(cached, fresh)
    assert_index_exact(fresh, oracle)
    assert cached.reused_geometry_points == 0, "borrowed direction is never cached as own geometry"
    if replacement == "absent":
        assert not np.any(cached.normal_valid)
    else:
        assert np.all(cached.direction_borrowed)
        # Position still comes only from the symmetric confirmed square.
        normal_axis = 1 if replacement == "horizontal" else 0
        assert np.max(np.abs(cached.line_points[:, normal_axis])) < 1e-12
    for name in ARRAYS:
        assert getattr(previous, name).tobytes() == before[name], name


@pytest.mark.parametrize("count", [0, 1, 7, 8, 9, 63, 64, 65, 129])
def test_production_radius_bytes_match_independent_oracle_and_python_graph(monkeypatch, native, count):
    rng = np.random.RandomState(730 + count)
    points = rng.uniform(-.7, .7, (count, 2)).astype(np.float64)
    if count > 8:
        points[::7] = points[0]  # exact duplicates and non-byte-aligned rows
    points = points[::-1].copy()
    before = points.tobytes()
    no_normals = np.empty(0, dtype=np.int64)
    expected = build(monkeypatch, None, points, normal_targets=no_normals)
    spy = NativeSpy(native)
    observed = build(monkeypatch, spy, points, normal_targets=no_normals)
    assert_one_radius_call(spy, points, observed.cfg)
    assert_index_exact(observed, expected)
    keys = np.floor(points / observed.cfg["normal_connectivity_m"]).astype(np.int64)
    assert observed._adjacency_packed == independent_radius_oracle(
        points, keys, observed.cfg["normal_connectivity_m"] ** 2)
    assert points.tobytes() == before


@pytest.mark.parametrize("position", ["inside", "on", "outside"])
@pytest.mark.parametrize("axis", [0, 1])
def test_radius_threshold_uses_original_square_and_two_term_sum(monkeypatch, native, position, axis):
    radius = .22
    value = {"inside": np.nextafter(radius, 0.), "on": radius,
             "outside": np.nextafter(radius, np.inf)}[position]
    endpoint = [0., 0.]
    endpoint[axis] = -value
    # These diagonal coordinates straddle the same rounded squared threshold;
    # the native kernel must not replace dx*dx + dy*dy with hypot/FMA/epsilon.
    points = np.array([[0., 0.], endpoint, [.132, .176],
                       [np.nextafter(.132, np.inf), .176],
                       [.132, np.nextafter(.176, 0.)]], dtype=np.float64)
    spy = NativeSpy(native)
    result = build(monkeypatch, spy, points, normal_targets=np.empty(0, dtype=np.int64))
    assert_one_radius_call(spy, points, result.cfg)
    keys = np.floor(points / radius).astype(np.int64)
    assert result._adjacency_packed == independent_radius_oracle(points, keys, radius ** 2)
    assert_index_exact(result, build(monkeypatch, None, points,
                                    normal_targets=np.empty(0, dtype=np.int64)))


def test_native_radius_keeps_key_neighborhood_even_for_identical_positions(native):
    low, high = -(1 << 63), (1 << 63) - 1
    keys = np.array([[low, low], [low + 1, low], [high, high], [high - 1, high],
                     [0, 0], [1, 1], [2, 0]], dtype=np.int64)
    points = np.zeros((len(keys), 2), dtype=np.float64)
    actual = native.radius_graph(points.tobytes(), keys.tobytes(), len(points), .22 ** 2)
    assert type(actual) is bytes
    assert actual == independent_radius_oracle(points, keys, .22 ** 2)


def test_integer_connectivity_configuration_preserves_python_geometry(monkeypatch, native):
    points = scene("thin")
    cfg = {"normal_connectivity_m": 1, "normal_radius_m": 1.0}
    baseline = build(monkeypatch, None, points, cfg=cfg)
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points, cfg=cfg)
    assert_one_radius_call(spy, points, actual.cfg)
    assert_index_exact(actual, baseline)


def bucket_capacity_points(kind, accepted):
    if kind == "single":
        return np.zeros((512 if accepted else 513, 2), dtype=np.float64)
    # The current cell plus its four preceding neighbor cells contains exactly
    # 2048 or 2049 candidates. Every individual bucket remains <=512 points.
    cells = [(0, 0), (-1, -1), (-1, 0), (-1, 1), (0, -1)]
    counts = [408 if accepted else 409, 410, 410, 410, 410]
    return np.vstack([np.tile((np.asarray(key, dtype=np.float64) + .25) * .22, (count, 1))
                      for key, count in zip(cells, counts)])


@pytest.mark.parametrize("kind", ["single", "five_neighbors"])
def test_native_radius_retains_inclusive_python_bucket_cap(monkeypatch, native, kind):
    points = bucket_capacity_points(kind, accepted=True)
    no_normals = np.empty(0, dtype=np.int64)
    expected = build(monkeypatch, None, points, normal_targets=no_normals)
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points, normal_targets=no_normals)
    assert_one_radius_call(spy, points, actual.cfg)
    assert_index_exact(actual, expected)


@pytest.mark.parametrize("kind", ["single", "five_neighbors"])
def test_python_bucket_capacity_rejects_before_any_native_graph_work(monkeypatch, native, kind):
    points = bucket_capacity_points(kind, accepted=False)
    spy = NativeSpy(native)
    for backend in (None, spy):
        with pytest.raises(ValueError, match="normal connectivity bucket capacity exceeded"):
            build(monkeypatch, backend, points, normal_targets=np.empty(0, dtype=np.int64))
    assert not spy.radius_calls
    assert not spy.prepare_calls
    assert not spy.graph_calls
    assert not spy.subset_calls


@pytest.mark.parametrize("dtype", [np.dtype("float32"), np.dtype("float16"),
                                  np.dtype("float64").newbyteorder("S")])
def test_non_native_float64_arrays_keep_numpy_distance_semantics(monkeypatch, native, dtype):
    points = np.array([[0., 0.], [.22, 0.], [.132, .176], [-.22, 0.],
                       [-.132, -.176], [.44, .22]], dtype=dtype)
    assert points.dtype != np.float64
    before = points.tobytes()
    no_normals = np.empty(0, dtype=np.int64)
    expected = build(monkeypatch, None, points, normal_targets=no_normals)
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points, normal_targets=no_normals)
    assert_index_exact(actual, expected)
    assert not spy.radius_calls, "do not silently cast different arithmetic to native float64"
    # Topology acceleration may still run after Python has computed its edges.
    assert len(spy.prepare_calls) == 1
    assert len(spy.graph_calls) == 1
    assert points.tobytes() == before


def test_radius_kernel_exception_cannot_quietly_become_python_success(monkeypatch, native):
    class RejectingRadius(NativeSpy):
        def radius_graph(self, points, keys, count, radius_squared):
            self.radius_calls.append((points, keys, count, radius_squared))
            raise RuntimeError("injected native radius failure")

    spy = RejectingRadius(native)
    with pytest.raises(RuntimeError, match="injected native radius failure"):
        build(monkeypatch, spy, line(), normal_targets=np.empty(0, dtype=np.int64))
    assert len(spy.radius_calls) == 1
    assert not spy.prepare_calls
    assert not spy.graph_calls


def test_3999_padding_and_direct_api_reject_4001(monkeypatch, native):
    count = 3999
    points = np.column_stack((np.arange(count) * .75, np.zeros(count)))
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points, normal_targets=np.empty(0, dtype=np.int64))
    assert_one_radius_call(spy, points, actual.cfg)
    row_bytes = (count + 7) // 8
    assert actual._adjacency_packed == b"".join((1 << i).to_bytes(row_bytes, "little") for i in range(count))
    # Existing 4000/4001 integration tests also compare full geometry and prove
    # that 4001 uses the unchanged Python path. The public C API itself rejects
    # this size even with otherwise correctly shaped byte buffers.
    larger = np.column_stack((np.arange(4001) * .75, np.zeros(4001)))
    keys = np.floor(larger / .22).astype(np.int64)
    with pytest.raises(ValueError):
        native.radius_graph(larger.tobytes(), keys.tobytes(), len(larger), .22 ** 2)


def independent_local_support(index, targets, candidates, allowed):
    """Plain per-row graph walk: no production forest or native subset helper.

    The radius graph itself is separately compared against the independent
    all-pairs oracle above. These cases use unique candidate IDs, matching the
    bounded patches produced by _FrozenIndex._patch.
    """
    result = np.zeros_like(allowed, dtype=bool)
    for row, seed in enumerate(targets):
        positions = {int(value): position for position, value in enumerate(candidates)}
        if int(seed) not in positions or not allowed[row, positions[int(seed)]]:
            continue
        permitted = {int(value) for value, enabled in zip(candidates, allowed[row]) if enabled}
        pending = [int(seed)]
        visited = {int(seed)}
        while pending:
            current = pending.pop()
            for other in permitted - visited:
                if index._adjacency_bits[current] & (1 << other):
                    visited.add(other)
                    pending.append(other)
        for value in visited:
            result[row, positions[value]] = True
    return result, np.sum(result, axis=1, dtype=np.int64)


def local_support_case(kind):
    points = scene("filled_band") if kind == "thick" else scene("thin")
    if kind in ("near_u", "remote_u", "clipped"):
        points = u_shape(.6 if kind == "near_u" else 3.)
    candidates = np.arange(len(points), dtype=np.int64)
    targets = np.array([0, len(points) // 2, len(points) - 1], dtype=np.int64)
    # Ordinary PCA starts with the target's global component. The U examples
    # below retain both branches globally, then remove the connecting bridge.
    allowed = np.ones((len(targets), len(candidates)), dtype=bool)
    if kind in ("near_u", "remote_u", "clipped"):
        targets = np.array([0, 31], dtype=np.int64)
        allowed = np.tile(points[:, 0] < .5, (2, 1))
        if kind == "clipped":
            candidates = candidates[1:]
            allowed = allowed[:, 1:]
    elif kind == "seed_forbidden":
        allowed[np.arange(len(targets)), targets] = False
    elif kind == "seed_absent":
        candidates = np.setdiff1d(candidates, targets).astype(np.int64)
        allowed = np.ones((len(targets), len(candidates)), dtype=bool)
    elif kind == "empty_rows":
        targets = np.empty(0, dtype=np.int64)
        allowed = np.empty((0, len(candidates)), dtype=bool)
    elif kind == "empty_candidates":
        candidates = np.empty(0, dtype=np.int64)
        allowed = np.empty((len(targets), 0), dtype=bool)
    elif kind in ("random", "reversed", "strided"):
        allowed = np.random.RandomState(126).uniform(size=allowed.shape) > .3
        allowed[np.arange(len(targets)), targets] = True
        if kind == "reversed":
            candidates = candidates[::-1].copy()
            allowed = allowed[:, ::-1].copy()
    return points, targets, candidates, allowed


@pytest.mark.parametrize("kind", ["whole", "thick", "near_u", "remote_u", "clipped",
                                  "seed_forbidden", "seed_absent", "empty_rows",
                                  "empty_candidates", "random", "reversed", "strided"])
@pytest.mark.parametrize("return_counts", [False, True])
def test_native_local_support_preserves_alias_bytes_and_independent_walk(
        monkeypatch, native, kind, return_counts):
    points, targets, candidates, allowed = local_support_case(kind)
    no_normals = np.empty(0, dtype=np.int64)
    baseline = build(monkeypatch, None, points, normal_targets=no_normals)
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points, normal_targets=no_normals)
    assert_index_exact(actual, baseline)
    # Keep the original contract's pre-filtered global-component eligibility;
    # connectivity must still be recomputed inside each induced local patch.
    allowed &= baseline.components[candidates][None, :] == baseline.components[targets][:, None]
    if kind == "strided":
        backing = np.ones((len(targets), 2 * len(candidates)), dtype=bool)
        backing[:, ::2] = allowed
        allowed = backing[:, ::2]
        assert not allowed.flags.c_contiguous
    initial = allowed.copy()
    target_bytes, candidate_bytes = targets.tobytes(), candidates.tobytes()
    graph_before = {name: getattr(actual, name).tobytes() for name in ARRAYS}
    expected, expected_counts = invoke(monkeypatch, None, baseline._local_support,
                                       targets, candidates, initial.copy(), return_counts=True)
    walked, walked_counts = independent_local_support(baseline, targets, candidates, initial)
    assert_bytes(expected, walked)
    assert_bytes(expected_counts, walked_counts)
    result = invoke(monkeypatch, spy, actual._local_support,
                    targets, candidates, allowed, return_counts=return_counts)
    observed = result[0] if return_counts else result
    assert observed is allowed, "the original caller-owned mask must be filled in place"
    assert_bytes(observed, expected)
    if return_counts:
        counts = result[1]
        assert_bytes(counts, expected_counts)
        assert counts.dtype == np.int64 and counts.flags.writeable
        assert not np.shares_memory(counts, allowed)
        if len(counts):
            counts[0] = -123  # owned copy, not a read-only bytes view or graph state
            assert_bytes(observed, expected)
    if kind == "strided":
        assert np.all(backing[:, 1::2]), "native fill must not touch the non-view columns"
    assert len(spy.local_calls) == 1
    assert spy.local_calls[0] == (len(points), "PyCapsule", id(actual._native_graph),
                                 candidate_bytes, target_bytes, initial.tobytes(),
                                 actual._forest_parents.tobytes())
    assert not spy.subset_calls, "the early native branch must own the entire support operation"
    assert len(spy.prepare_calls) == 1
    assert targets.tobytes() == target_bytes and candidates.tobytes() == candidate_bytes
    for name in ARRAYS:
        assert getattr(actual, name).tobytes() == graph_before[name], name


@pytest.mark.parametrize("ids_dtype", [np.dtype("int32"), np.dtype("int64").newbyteorder("S")])
def test_non_native_int64_local_ids_keep_python_fallback(monkeypatch, native, ids_dtype):
    points, targets, candidates, allowed = local_support_case("near_u")
    targets, candidates = targets.astype(ids_dtype), candidates.astype(ids_dtype)
    baseline = build(monkeypatch, None, points, normal_targets=np.empty(0, dtype=np.int64))
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points, normal_targets=np.empty(0, dtype=np.int64))
    expected, expected_counts = invoke(monkeypatch, None, baseline._local_support,
                                       targets, candidates, allowed.copy(), return_counts=True)
    input_mask = allowed.copy()
    observed, counts = invoke(monkeypatch, spy, actual._local_support,
                              targets, candidates, input_mask, return_counts=True)
    assert observed is input_mask
    assert_bytes(observed, expected)
    assert_bytes(counts, expected_counts)
    assert not spy.local_calls, "different integer byte order/width is not native int64"


def test_local_support_native_failure_cannot_quietly_fall_back(monkeypatch, native):
    class RejectingSupport(NativeSpy):
        def local_support(self, *args):
            self.local_calls.append(args)
            raise RuntimeError("injected native local-support failure")

    points = u_shape(3.)
    spy = RejectingSupport(native)
    actual = build(monkeypatch, spy, points, normal_targets=np.empty(0, dtype=np.int64))
    targets = np.array([0], dtype=np.int64)
    candidates = np.arange(len(points), dtype=np.int64)
    allowed = np.tile(points[:, 0] < .5, (1, 1))
    before = allowed.tobytes()
    with pytest.raises(RuntimeError, match="injected native local-support failure"):
        invoke(monkeypatch, spy, actual._local_support, targets, candidates, allowed)
    assert len(spy.local_calls) == 1
    assert not spy.subset_calls
    assert allowed.tobytes() == before


@pytest.mark.parametrize("use_native", [False, True])
def test_mixed_components_cannot_exploit_equal_component_size(monkeypatch, native, use_native):
    # Defensive validation added with the local-support integration: the old
    # internal Python fast path assumed callers already removed other global
    # components. Do not claim unchanged behavior for that invalid old input.
    points = np.array([[0., 0.], [.1, 0.], [2., 0.], [2.1, 0.]])
    backend = NativeSpy(native) if use_native else None
    index = build(monkeypatch, backend, points, normal_targets=np.empty(0, dtype=np.int64))
    targets = np.array([0], dtype=np.int64)
    candidates = np.arange(4, dtype=np.int64)
    allowed = np.array([[True, False, True, False]])
    assert index._component_sizes[index.components[0]] == allowed.sum() == 2
    expected, expected_counts = independent_local_support(index, targets, candidates, allowed)
    observed, counts = invoke(monkeypatch, backend, index._local_support,
                              targets, candidates, allowed, return_counts=True)
    assert observed is allowed
    assert_bytes(observed, expected)
    assert_bytes(counts, expected_counts)
    np.testing.assert_array_equal(observed, [[True, False, False, False]])
    np.testing.assert_array_equal(counts, [1])
    if use_native:
        assert len(backend.local_calls) == 1


API_VERSIONS = ("API_VERSION", "RADIUS_GRAPH_API_VERSION", "LOCAL_SUPPORT_API_VERSION",
                "NEAREST_API_VERSION", "CROSS_SECTION_WEIGHTS_API_VERSION")
API_FUNCTIONS = ("radius_graph", "prepare_graph", "graph_forest", "connected_subsets",
                 "local_support", "nearest_grouped", "cross_section_weights")


def load_with_api_stub(monkeypatch, attribute=None, missing=False):
    """Source-only import-contract test, never evidence of C execution."""
    stub = SimpleNamespace(**{name: 1 for name in API_VERSIONS})
    for name in API_FUNCTIONS:
        setattr(stub, name, lambda *args: pytest.fail("import must not execute geometry"))
    if attribute:
        if missing:
            delattr(stub, attribute)
        else:
            setattr(stub, attribute, 999 if attribute in API_VERSIONS else None)
    name = "lane_mapping_lya_reference._integration_api_probe"
    spec = importlib.util.spec_from_file_location(name, module.__file__)
    probe = importlib.util.module_from_spec(spec)
    with monkeypatch.context() as patch:
        patch.setitem(sys.modules, "_lane_mapping_lya_native", stub)
        patch.setitem(sys.modules, name, probe)
        spec.loader.exec_module(probe)
    return probe, stub


@pytest.mark.parametrize("attribute", API_VERSIONS + API_FUNCTIONS)
@pytest.mark.parametrize("missing", [False, True])
def test_source_only_native_api_mismatch_is_not_silent_fallback(monkeypatch, attribute, missing):
    with pytest.raises(RuntimeError, match="incompatible lane graph native extension"):
        load_with_api_stub(monkeypatch, attribute, missing)


def test_source_only_complete_api_stub_is_only_an_import_contract(monkeypatch):
    probe, stub = load_with_api_stub(monkeypatch)
    assert probe._GRAPH_NATIVE is stub


class RejectNewKernels:
    """Only a routing sentinel: no simulated numerical/native implementation."""
    def __init__(self):
        self.calls = []

    def nearest_grouped(self, *args):
        self.calls.append("nearest_grouped")
        raise RuntimeError("injected nearest failure")

    def cross_section_weights(self, *args):
        self.calls.append("cross_section_weights")
        raise RuntimeError("injected weights failure")


def test_source_only_query_ids_use_int64_even_when_flatnonzero_is_int32(monkeypatch):
    points = np.array([[.2, 0.], [-.1, 0.]])
    index = nearest_index(monkeypatch, points)
    original = np.flatnonzero
    seen = []

    def capture(*args):
        # Serialization-only stub, not a native execution or geometry oracle.
        seen.append(args)
        np.testing.assert_array_equal(np.frombuffer(args[3], dtype=np.int64), [1, 0])
        return np.full(2, -1, dtype=np.int64).tobytes()

    with monkeypatch.context() as patch:
        patch.setattr(np, "flatnonzero", lambda values: original(values).astype(np.int32))
        patch.setattr(module, "_GRAPH_NATIVE", SimpleNamespace(nearest_grouped=capture))
        np.testing.assert_array_equal(index.nearest(points), [-1, -1])
    assert len(seen) == 1


def nearest_index(monkeypatch, points, cfg=None):
    # Explicit synthetic normals isolate distance routing from PCA behavior;
    # all-index tests above independently cover real normal construction.
    index = build(monkeypatch, None, points, cfg, normal_targets=np.empty(0, dtype=np.int64))
    index.normal_valid = np.ones(len(points), dtype=bool)
    index.normals = np.tile([0., 1.], (len(points), 1))
    return index


@pytest.mark.parametrize("dtype", [np.dtype("float32"), np.dtype("float64").newbyteorder("S")])
@pytest.mark.parametrize("changed", ["queries", "anchor"])
def test_source_only_nearest_non_native_dtype_keeps_numpy(monkeypatch, dtype, changed):
    anchor = np.array([[0., 0.], [.2, 0.]])
    points = np.array([[.02, .03], [.19, 0.]])
    if changed == "anchor":
        anchor = anchor.astype(dtype)
    else:
        points = points.astype(dtype)
    index = nearest_index(monkeypatch, anchor)
    expected = invoke(monkeypatch, None, index.nearest, points)
    sentinel = RejectNewKernels()
    observed = invoke(monkeypatch, sentinel, index.nearest, points)
    assert_bytes(observed, expected)
    assert not sentinel.calls


@pytest.mark.parametrize("operation", ["nearest", "weights"])
def test_source_only_new_kernel_failure_must_propagate(monkeypatch, operation):
    points = np.array([[0., 0.], [0., .1]])
    index = nearest_index(monkeypatch, points)
    sentinel = RejectNewKernels()
    if operation == "nearest":
        call, args = index.nearest, (points,)
    else:
        # Sentinel capsule never reaches C; tests the dispatch/error boundary.
        index._native_graph = object()
        call = module._line_weights
        args = (points, np.tile([0., 1.], (2, 1)), index,
                np.arange(2, dtype=np.int64), index.cfg)
    with pytest.raises(RuntimeError, match="injected .* failure"):
        invoke(monkeypatch, sentinel, call, *args)
    assert sentinel.calls == (["nearest_grouped"] if operation == "nearest" else
                              ["cross_section_weights"])


@pytest.mark.parametrize("operation", ["nearest", "weights"])
def test_source_only_strict_underflow_keeps_original_numpy_error(monkeypatch, operation):
    points = np.array([[0., 0.], [1e-200, 0.]])
    index = nearest_index(monkeypatch, points)
    sentinel = RejectNewKernels()
    index._native_graph = object()
    with np.errstate(under="raise"):
        if operation == "nearest":
            call, args = index.nearest, (np.array([[.5e-200, 0.]]),)
        else:
            call = module._line_weights
            args = (points, np.tile([0., 1.], (2, 1)), index,
                    np.arange(2, dtype=np.int64), index.cfg)
        with pytest.raises(FloatingPointError, match="underflow"):
            invoke(monkeypatch, None, call, *args)
        with pytest.raises(FloatingPointError, match="underflow"):
            invoke(monkeypatch, sentinel, call, *args)
    assert not sentinel.calls, "native arithmetic must not hide NumPy's strict error policy"


def test_source_only_nearest_4001_queries_preserve_bounded_fallback(monkeypatch):
    index = nearest_index(monkeypatch, np.array([[0., 0.], [.2, 0.]]))
    queries = np.tile([.04, .01], (4001, 1))
    sentinel = RejectNewKernels()
    expected = invoke(monkeypatch, None, index.nearest, queries)
    observed = invoke(monkeypatch, sentinel, index.nearest, queries)
    assert_bytes(observed, expected)
    assert not sentinel.calls


def nearest_case(kind):
    anchor = np.array([[.1, .1], [-.1, .1], [.1, -.1], [1.1, .1], [1.1, -.1]])
    points = np.array([[1.1, 0.], [0., 0.], [.01, .02], [20., 20.]])
    cfg = None
    if kind == "empty_queries":
        points = np.empty((0, 2), dtype=np.float64)
    elif kind == "empty_anchor":
        anchor = np.empty((0, 2), dtype=np.float64)
    elif kind == "truncated":
        anchor = rectangle(-.4, .4, -.4, .4, step=.1)
        points = np.array([[0., 0.], [.1, 0.], [.3, .3]])
        cfg = {"max_patch_points": 8}
    elif kind == "strided":
        anchor = anchor[::-1]
        backing = np.full((len(points), 4), 123.)
        backing[:, ::2] = points
        points = backing[:, ::2]
        assert not points.flags.c_contiguous
    elif kind == "random":
        rng = np.random.RandomState(811)
        anchor = rng.uniform(-2., 2., (53, 2))
        points = rng.uniform(-3., 3., (19, 2))
    return anchor, points, cfg


class DotShapeRecorder(np.ndarray):
    """Record the actual @ batch shapes while retaining NumPy's arithmetic."""
    def __new__(cls, values, calls):
        result = np.asarray(values).view(cls)
        result.calls = calls
        return result

    def __array_finalize__(self, previous):
        self.calls = getattr(previous, "calls", None)

    def __matmul__(self, other):
        self.calls.append((self.shape, other.shape))
        return np.asarray(self) @ other


@pytest.mark.parametrize("kind", ["groups", "empty_queries", "empty_anchor", "all_invalid",
                                  "truncated", "strided", "random"])
@pytest.mark.parametrize("valid_only", [False, True])
@pytest.mark.parametrize("with_normals", [False, True])
def test_native_nearest_exact_groups_shapes_order_and_ownership(
        monkeypatch, native, kind, valid_only, with_normals):
    anchor, points, cfg = nearest_case(kind)
    baseline = nearest_index(monkeypatch, anchor, cfg)
    actual = nearest_index(monkeypatch, anchor, cfg)
    for index in (baseline, actual):
        index.normal_valid[::2] = False
        if kind == "all_invalid":
            index.normal_valid[:] = False
        index.normals[::2] *= -1.  # sign must not change compatibility
    expected_shapes, observed_shapes = [], []
    normals = np.tile([0., 1.], (len(points), 1))
    if len(normals) > 2:
        normals[2] = [1., 0.]  # no compatible candidate, not nearest by distance
    expected = invoke(monkeypatch, None, baseline.nearest, points,
                      DotShapeRecorder(normals, expected_shapes) if with_normals else None,
                      valid_only)
    before = {name: getattr(actual, name).tobytes() for name in ARRAYS}
    spy = NativeSpy(native)
    observed = invoke(monkeypatch, spy, actual.nearest, points,
                      DotShapeRecorder(normals, observed_shapes) if with_normals else None,
                      valid_only)
    assert_bytes(observed, expected)
    assert observed.dtype == np.int64 and observed.flags.writeable
    assert observed_shapes == expected_shapes
    assert actual.patch_truncations == baseline.patch_truncations
    if kind == "truncated":
        assert actual.patch_truncations > 0
    assert len(spy.nearest_calls) == 1, "native nearest must actually execute"
    pb, ab, qo, qi, co, ci, allowed, radius = spy.nearest_calls[0]
    assert pb == points.tobytes() and ab == actual.anchor.tobytes()
    keys = np.floor(points / baseline.cell).astype(np.int64)
    unique, inverse, _ = module._unique_cells(keys)
    query_ids, candidate_ids, query_offsets, candidate_offsets, masks = [], [], [0], [0], []
    for group, key in enumerate(unique):
        targets = np.flatnonzero(inverse == group)
        candidates = baseline._patch(tuple(int(value) for value in key), baseline.cfg["match_distance_m"])
        if valid_only:
            candidates = candidates[baseline.normal_valid[candidates]]
        query_ids.extend(targets)
        candidate_ids.extend(candidates)
        query_offsets.append(len(query_ids))
        candidate_offsets.append(len(candidate_ids))
        if with_normals and len(candidates):
            masks.append((np.abs(normals[targets] @ baseline.normals[candidates].T) >=
                          module.math.cos(baseline.cfg["normal_max_angle_rad"])).tobytes())
    # Explicit int64 serialization, never platform-sized int/intp.
    for raw, values in ((qo, query_offsets), (qi, query_ids),
                        (co, candidate_offsets), (ci, candidate_ids)):
        assert raw == np.asarray(values, dtype=np.int64).tobytes()
    assert allowed == (b"".join(masks) if with_normals else None)
    assert radius.hex() == float(baseline.cfg["match_distance_m"] ** 2).hex()
    for name in ARRAYS:
        assert getattr(actual, name).tobytes() == before[name], name
    if len(observed):
        observed[0] = -999
        again = invoke(monkeypatch, spy, actual.nearest, points, normals if with_normals else None, valid_only)
        assert_bytes(again, expected)


def test_native_nearest_tie_keeps_patch_order_not_global_id(monkeypatch, native):
    # ID 1 is in the lexicographically earlier cell and ties ID 0 exactly.
    points = np.array([[0., 0.]])
    index = nearest_index(monkeypatch, np.array([[.1, 0.], [-.1, 0.]]))
    spy = NativeSpy(native)
    expected = invoke(monkeypatch, None, index.nearest, points)
    observed = invoke(monkeypatch, spy, index.nearest, points)
    assert_bytes(observed, expected)
    np.testing.assert_array_equal(observed, [1])
    np.testing.assert_array_equal(np.frombuffer(spy.nearest_calls[0][5], dtype=np.int64), [1, 0])


@pytest.mark.parametrize("edge", ["distance", "normal_dot"])
@pytest.mark.parametrize("position", ["inside", "on", "outside"])
def test_native_nearest_inclusive_boundaries_preserve_numpy_decision(
        monkeypatch, native, edge, position):
    index = nearest_index(monkeypatch, np.array([[0., 0.]]))
    if edge == "distance":
        boundary = index.cfg["match_distance_m"]
        value = {"inside": np.nextafter(boundary, 0.), "on": boundary,
                 "outside": np.nextafter(boundary, np.inf)}[position]
        queries, normals = np.array([[value, 0.]]), None
    else:
        boundary = module.math.cos(index.cfg["normal_max_angle_rad"])
        value = {"inside": np.nextafter(boundary, np.inf), "on": boundary,
                 "outside": np.nextafter(boundary, 0.)}[position]
        queries, normals = np.array([[0., 0.]]), np.array([[0., value]])
    spy = NativeSpy(native)
    expected = invoke(monkeypatch, None, index.nearest, queries, normals)
    observed = invoke(monkeypatch, spy, index.nearest, queries, normals)
    assert_bytes(observed, expected)
    np.testing.assert_array_equal(observed, [-1 if position == "outside" else 0])
    assert len(spy.nearest_calls) == 1


@pytest.mark.parametrize("layout", ["strided", "reversed", "empty", "float32_normals",
                                    "swapped_normals", "int32_ids", "swapped_ids",
                                    "float32_cloud", "swapped_cloud", "large_normals"])
def test_native_weights_layout_dtype_empty_and_fallback(monkeypatch, native, layout):
    points = np.column_stack((np.zeros(9), np.arange(9) * .1))
    ids = np.array([0, 4, 4, 8], dtype=np.int64)
    normals = np.tile([0., 1.], (len(ids), 1))
    if layout == "strided":
        storage = np.zeros((len(ids), 4))
        storage[:, ::2] = normals
        normals = storage[:, ::2]
        ids = np.repeat(ids, 2)[::2]
        assert not ids.flags.c_contiguous and not normals.flags.c_contiguous
    elif layout == "reversed":
        ids, normals = ids[::-1], normals[::-1]
    elif layout == "empty":
        ids, normals = np.empty(0, dtype=np.int64), np.empty((0, 2))
    elif layout == "float32_normals":
        normals = normals.astype(np.float32)
    elif layout == "swapped_normals":
        normals = normals.astype(np.dtype("float64").newbyteorder("S"))
    elif layout == "int32_ids":
        ids = ids.astype(np.int32)
    elif layout == "swapped_ids":
        ids = ids.astype(np.dtype("int64").newbyteorder("S"))
    elif layout == "float32_cloud":
        points = points.astype(np.float32)
    elif layout == "swapped_cloud":
        points = points.astype(np.dtype("float64").newbyteorder("S"))
    elif layout == "large_normals":
        normals *= 3.
    baseline = build(monkeypatch, None, points, normal_targets=np.empty(0, dtype=np.int64))
    spy = NativeSpy(native)
    actual = build(monkeypatch, spy, points, normal_targets=np.empty(0, dtype=np.int64))
    originals = (points.tobytes(), normals.tobytes(), ids.tobytes())
    expected = invoke(monkeypatch, None, module._line_weights, points[ids], normals, baseline, ids, baseline.cfg)
    observed = invoke(monkeypatch, spy, module._line_weights, points[ids], normals, actual, ids, actual.cfg)
    assert_bytes(observed, expected)
    assert observed.flags.writeable
    assert originals == (points.tobytes(), normals.tobytes(), ids.tobytes())
    if layout in ("strided", "reversed"):
        assert len(spy.weight_calls) == 1
        assert spy.weight_calls[0][3:6] == originals
        assert not spy.subset_calls
    elif layout == "empty":
        assert not spy.weight_calls and not spy.subset_calls
    else:
        assert not spy.weight_calls, "wrong byte order/width must preserve NumPy arithmetic"
        assert len(spy.subset_calls) == 1, "integer-only connected fallback still executes"
