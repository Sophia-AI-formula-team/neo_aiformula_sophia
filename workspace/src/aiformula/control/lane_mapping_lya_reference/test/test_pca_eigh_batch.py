"""Bounded batched eigh versus frozen R4 PCA, not a timing benchmark.

The fixed d1c5d33 oracle is defined in test_local_support_counts. Its actual
eigh inputs are recorded and filtered only by the unchanged minimum-count
gate and the explicitly repaired missing-seed safety precondition. The spy
always calls the real NumPy solver; no geometry or acceptance is substituted.
"""
from collections import Counter
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parent))

from lane_mapping_lya_reference import mask_localization as production
from test_local_support_counts import FrozenR4Index, GEOMETRY, assert_geometry_exact, exact_array


def observe_solver(monkeypatch, constructor, points, cfg, **kwargs):
    original = np.linalg.eigh
    observed = dict(batch_sizes=[], matrices=[], eigenvalues=[], eigenvectors=[], radii=[], raw_rows=0)
    frozen = constructor is FrozenR4Index

    def spy(matrix, *args, **keywords):
        assert matrix.ndim == 3 and matrix.shape[1:] == (2, 2)
        assert matrix.dtype == np.float64
        frame_values = sys._getframe(1).f_locals
        selected = np.ones(len(matrix), dtype=bool)
        if frozen:
            # Read the old algorithm's real intermediate arrays. This is not
            # a reimplementation of the new batching or covariance reduction.
            counts = frame_values["counts"]
            indices = frame_values["indices"]
            candidates = frame_values["candidate_indices"]
            support = frame_values["support"]
            assert counts.shape == (len(matrix),)
            seed_present = np.any(support & (indices[:, None] == candidates[None, :]), axis=1)
            selected = (counts >= cfg["normal_min_points"]) & seed_present
        else:
            assert 1 <= len(matrix) <= 256
        values, vectors = original(matrix, *args, **keywords)
        observed["batch_sizes"].append(len(matrix))
        observed["raw_rows"] += len(matrix)
        observed["matrices"].extend(row.tobytes() for row in matrix[selected])
        observed["eigenvalues"].extend(row.tobytes() for row in values[selected])
        observed["eigenvectors"].extend(row.tobytes() for row in vectors[selected])
        observed["radii"].extend([frame_values["radius"]] * int(np.sum(selected)))
        return values, vectors

    with monkeypatch.context() as patch:
        patch.setattr(np.linalg, "eigh", spy)
        index = constructor(points, cfg, **kwargs)
    return index, observed


def assert_solver_inputs_and_outputs(actual, expected):
    assert sum(actual["batch_sizes"]) == len(expected["matrices"])
    # Ordered comparisons also reject duplicated or silently reordered rows.
    assert actual["matrices"] == expected["matrices"]
    assert actual["eigenvalues"] == expected["eigenvalues"]
    assert actual["eigenvectors"] == expected["eigenvectors"]
    assert actual["radii"] == expected["radii"]
    assert Counter(actual["matrices"]) == Counter(expected["matrices"])


def compare(monkeypatch, points, config=None, actual_kwargs=None, expected_kwargs=None):
    cfg = production._configuration(config)
    before = points.tobytes()
    actual, measured = observe_solver(monkeypatch, production._FrozenIndex, points, cfg,
                                      **(actual_kwargs or {}))
    expected, oracle = observe_solver(monkeypatch, FrozenR4Index, points, cfg,
                                      **(expected_kwargs or actual_kwargs or {}))
    assert_geometry_exact(actual, expected)
    assert_solver_inputs_and_outputs(measured, oracle)
    assert points.tobytes() == before
    return actual, measured, oracle


@pytest.mark.parametrize("count", [255, 256, 257, 511, 512, 513])
def test_exact_256_matrix_boundaries_and_radius_tail_flush(monkeypatch, count):
    points = np.c_[np.arange(count) * .01, np.zeros(count)]
    actual, measured, oracle = compare(monkeypatch, points)
    assert np.all(actual.normal_valid)
    assert np.all(actual.normal_radii == actual.cfg["normal_radius_m"])
    assert measured["batch_sizes"] == [256] * (count // 256) + ([count % 256] if count % 256 else [])
    assert sum(measured["batch_sizes"]) == count
    assert len(measured["batch_sizes"]) < len(oracle["batch_sizes"])


def test_more_than_256_cells_are_combined_without_cross_cell_pca(monkeypatch):
    points = np.vstack([np.c_[base + np.arange(7) * .06, np.zeros(7)] for base in range(300)])
    actual, measured, oracle = compare(monkeypatch, points)
    assert len(actual.cells) >= 300
    assert np.all(actual.normal_valid)
    assert sum(measured["batch_sizes"]) == 2100
    assert measured["batch_sizes"] == [256] * 8 + [52]
    assert len(oracle["batch_sizes"]) >= 300


def fixture(kind):
    if kind == "line":
        x = np.arange(-3., 3.001, .08)
        return np.c_[x, .07 * x]
    if kind == "wide_band":
        x, y = np.meshgrid(np.arange(-2., 2.001, .1), np.arange(-.15, .151, .05))
        return np.c_[x.ravel(), y.ravel()]
    if kind == "near_degenerate":
        angle = np.arange(65) * (2. * np.pi / 65)
        return np.c_[.12 * np.cos(angle), .12 * (1. + 1e-12) * np.sin(angle)]
    if kind == "isotropic":
        x, y = np.meshgrid(np.arange(-.16, .161, .08), np.arange(-.16, .161, .08))
        return np.c_[x.ravel(), y.ravel()]
    if kind == "mixed_minimum_radius":
        return np.vstack((np.c_[np.arange(9) * .15, np.zeros(9)],
                          np.c_[np.arange(7) * .06, np.full(7, 3.)]))
    if kind == "mixed_insufficient":
        return np.vstack((np.c_[np.arange(7) * .06, np.zeros(7)],
                          np.c_[np.arange(5) * .06, np.full(5, .3)]))
    raise ValueError(kind)


@pytest.mark.parametrize("kind", ["line", "wide_band", "near_degenerate", "isotropic",
                                  "mixed_minimum_radius", "mixed_insufficient"])
def test_all_geometry_and_actual_covariances_are_byte_exact(monkeypatch, kind):
    actual, measured, oracle = compare(monkeypatch, fixture(kind))
    if kind == "mixed_minimum_radius":
        assert np.any(actual.normal_radii == actual.cfg["normal_radius_m"])
        assert np.any(actual.normal_radii > actual.cfg["normal_radius_m"])
    if kind == "mixed_insufficient":
        assert oracle["raw_rows"] > sum(measured["batch_sizes"])
        assert not np.any(actual.normal_valid[-5:])
    if kind in ("near_degenerate", "isotropic"):
        assert not np.any(actual.normal_valid)


@pytest.mark.parametrize("mode", ["hint", "targets", "queries", "cache", "changed_cache"])
def test_hint_query_selection_and_cache_do_not_change_batch_order_or_geometry(monkeypatch, mode):
    x = np.arange(-2., 2.001, .08)
    square = fixture("isotropic")
    points = np.vstack((square, np.c_[x, np.full(len(x), 3.)]))
    hint = np.vstack((np.c_[x, np.zeros(len(x))], np.c_[x, np.full(len(x), 3.)]))
    cfg = production._configuration(None)
    actual_kwargs, expected_kwargs = {"direction_support": hint}, {"direction_support": hint}
    if mode == "targets":
        actual_kwargs["normal_targets"] = expected_kwargs["normal_targets"] = np.arange(len(square))
    elif mode == "queries":
        actual_kwargs["normal_queries"] = expected_kwargs["normal_queries"] = points[:len(square)]
    elif mode in ("cache", "changed_cache"):
        actual_kwargs["reuse_index"] = production._FrozenIndex(points.copy(), cfg)
        expected_kwargs["reuse_index"] = FrozenR4Index(points.copy(), cfg)
        if mode == "changed_cache":
            points = points.copy()
            points[-4:, 1] += .025
    actual, _, _ = compare(monkeypatch, points, actual_kwargs=actual_kwargs,
                            expected_kwargs=expected_kwargs)
    assert np.any(actual.direction_borrowed[:len(square)])


def test_real_six_point_patch_excludes_seed_and_explicitly_repairs_old_r4(monkeypatch):
    # These are seven ordered samples within a single real hash cell. The six
    # nearest its centre exclude x=.49, but remain a connected six-point line.
    points = np.c_[.25 + np.arange(7) * .04, np.zeros(7)]
    cfg = production._configuration({"max_patch_points": 6})
    actual, measured = observe_solver(monkeypatch, production._FrozenIndex, points, cfg)
    old, oracle = observe_solver(monkeypatch, FrozenR4Index, points, cfg)
    candidates = actual._patch((0, 0), cfg["normal_radius_m"])
    exact_array(candidates, np.arange(6, dtype=np.int64))
    assert 6 not in candidates and actual.patch_truncations > 0
    assert old.normal_valid[6] and old.normal_position_support[6] == 6
    assert not actual.normal_valid[6] and actual.normal_position_support[6] == 0
    assert not actual.direction_borrowed[6]
    assert np.all(actual.normal_valid[:6])
    # This one known safety difference is explicit. Every unaffected sample
    # and every actually eligible covariance/solver result still stays exact.
    for name in GEOMETRY:
        exact_array(getattr(actual, name)[:6], getattr(old, name)[:6])
    assert_solver_inputs_and_outputs(measured, oracle)
    assert sum(measured["batch_sizes"]) == 6


@pytest.mark.parametrize("count", [0, 1, 5])
def test_no_solver_work_when_no_row_can_pass_minimum_count(monkeypatch, count):
    points = np.c_[np.arange(count) * .06, np.zeros(count)]
    _, measured, oracle = compare(monkeypatch, points)
    assert measured["batch_sizes"] == oracle["batch_sizes"] == []
