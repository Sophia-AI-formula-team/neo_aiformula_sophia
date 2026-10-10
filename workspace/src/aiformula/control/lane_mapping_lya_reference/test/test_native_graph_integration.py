"""Native graph integration against the unchanged Python geometry backend.

These are exact geometry/ownership tests, not performance or driving evidence.
Source-only pytest may skip native comparisons. LANE_REQUIRE_NATIVE=1 instead
fails when the extension is absent; importing the module alone is not a pass.
"""
import importlib
import os
from pathlib import Path
import sys

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
    assert callable(result.prepare_graph)
    assert callable(result.graph_forest)
    assert callable(result.connected_subsets)
    return result


class NativeSpy:
    """Forward to the real extension; record calls without replacing results."""
    def __init__(self, native):
        self.native = native
        self.prepare_calls = []
        self.graph_calls = []
        self.subset_calls = []
        self.subset_graphs = []

    def __getattr__(self, name):
        return getattr(self.native, name)

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
    assert spy.subset_calls, "forest shortcut must not make this native-flood test vacuous"
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
        assert len(spy.graph_calls) == 1
        assert spy.prepare_calls == [(count, len(actual._adjacency_packed),
                                      id(actual._native_graph))]
    else:
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
        with pytest.raises(ValueError, match="bounded graph capacity"):
            invoke(monkeypatch, spy, module._line_weights, points[:1],
                   np.array([[0., 1.]]), actual, np.array([0]), actual.cfg)


def test_real_512_point_patch_and_missing_seed_are_bit_exact(monkeypatch, native):
    points = rectangle(-1., 1., -1., 1., step=.05)
    selected = np.array([0, len(points) // 2, len(points) - 1], dtype=np.int64)
    baseline = build(monkeypatch, None, points, normal_targets=selected)
    actual = build(monkeypatch, native, points, normal_targets=selected)
    assert actual.cfg["max_patch_points"] == 512
    assert_index_exact(actual, baseline)
    candidates = actual._patch((0, 0), actual.cfg["normal_max_radius_m"])
    assert_bytes(candidates, baseline._patch((0, 0), baseline.cfg["normal_max_radius_m"]))
    assert len(candidates) == 512
    missing = np.setdiff1d(np.arange(len(points)), candidates)[0]
    allowed = np.ones((1, len(candidates)), dtype=bool)
    observed, counts = invoke(monkeypatch, native, actual._local_support,
                              np.array([missing]), candidates, allowed.copy(), return_counts=True)
    expected, expected_counts = invoke(monkeypatch, None, baseline._local_support,
                                       np.array([missing]), candidates, allowed.copy(), return_counts=True)
    assert_bytes(observed, expected)
    assert_bytes(counts, expected_counts)
    assert not np.any(observed)
    assert counts.tolist() == [0]


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
    assert len(spy.subset_calls) == (len(ids) + 63) // 64
    assert len(spy.prepare_calls) == 1, "one immutable graph must serve every density batch"
    assert all(call == (len(points), "PyCapsule", id(actual._native_graph))
               for call in spy.subset_graphs)
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
