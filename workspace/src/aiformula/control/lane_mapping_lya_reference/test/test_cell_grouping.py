"""Small exact-oracle regressions for the integer cell grouping optimization.

These compare production helpers with the previous NumPy grouping and original
input-order accumulation. They do not benchmark runtime or change match gates.
"""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from lane_mapping_lya_reference.mask_localization import _unique_cells, _voxel_means


def assert_same_array_bytes(actual, expected):
    assert actual.shape == expected.shape
    assert actual.dtype == expected.dtype
    assert actual.tobytes(order="C") == expected.tobytes(order="C")


def assert_grouping_matches_numpy(keys):
    before = keys.copy()
    expected = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    actual = _unique_cells(keys)
    for observed, reference in zip(actual, expected):
        assert_same_array_bytes(observed, reference)
    assert_same_array_bytes(keys, before)
    unique, inverse, counts = actual
    np.testing.assert_array_equal(unique[inverse], keys)
    assert int(np.sum(counts)) == len(keys)
    if len(keys):
        np.testing.assert_array_equal(np.bincount(inverse), counts)


LIMITS = np.iinfo(np.int64)


@pytest.mark.parametrize("rows", [
    [],
    [[0, 0]],
    [[8, -3]] * 11,
    [[0, 3], [0, -3], [0, 0], [0, 3], [0, -3]],
    [[3, 0], [-3, 0], [0, 0], [3, 0]],
    [[1, -8], [-1, 8], [-1, -8], [1, 8], [0, 0], [-1, 8]],
    [[-9, -8], [-9, -7], [0, 0], [2, 5]],
    [[2, 5], [0, 0], [-9, -7], [-9, -8]],
    [[LIMITS.min, LIMITS.max], [LIMITS.max, LIMITS.min],
     [LIMITS.min, LIMITS.min], [LIMITS.max, LIMITS.max],
     [LIMITS.min, LIMITS.max], [0, -1], [-1, 0]],
])
def test_exact_unique_inverse_counts_for_boundary_cases(rows):
    assert_grouping_matches_numpy(np.asarray(rows, dtype=np.int64).reshape(-1, 2))


@pytest.mark.parametrize("seed", [7, 101, 20261011])
@pytest.mark.parametrize("spread", [3, 1_000_000])
def test_seeded_duplicate_and_sparse_cells_keep_numpy_lexicographic_order(seed, spread):
    generator = np.random.default_rng(seed)
    keys = generator.integers(-spread, spread + 1, size=(257, 2), dtype=np.int64)
    keys = np.vstack((keys, keys[::7], keys[::-19]))
    assert_grouping_matches_numpy(keys)


@pytest.mark.parametrize("layout", ["row_stride", "negative_stride", "column_stride", "fortran"])
def test_noncontiguous_integer_views_are_not_modified(layout):
    storage = np.random.default_rng(108).integers(-5, 6, size=(72, 4), dtype=np.int64)
    views = {
        "row_stride": storage[::2, :2],
        "negative_stride": storage[::-1, 1::-1],
        "column_stride": storage[:, ::2],
        "fortran": np.asfortranarray(storage[:, :2]),
    }
    keys = views[layout]
    assert not keys.flags.c_contiguous
    assert_grouping_matches_numpy(keys)


def test_readonly_input_and_outputs_do_not_share_storage():
    keys = np.array([[3, 2], [-1, 8], [3, 2]], dtype=np.int64)
    keys.flags.writeable = False
    assert_grouping_matches_numpy(keys)
    for result in _unique_cells(keys):
        assert not np.shares_memory(result, keys)


def numpy_voxel_oracle(points, resolution):
    if not len(points):
        return np.empty((0, 2), dtype=np.float64)
    keys = np.floor(points / resolution).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    sums = np.zeros((len(counts), 2), dtype=np.float64)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


@pytest.mark.parametrize("resolution", [.08, .1, .5])
def test_voxel_means_match_original_accumulation_bytes_near_cell_boundaries(resolution):
    centers = np.arange(-4., 5.) * resolution
    boundary = np.concatenate((np.nextafter(centers, -np.inf), centers,
                               np.nextafter(centers, np.inf)))
    points = np.column_stack((boundary, boundary[::-1]))
    points = np.vstack((points, points[::3], [[0., -0.], [-0., 0.]]))
    before = points.copy()
    assert_same_array_bytes(_voxel_means(points, resolution), numpy_voxel_oracle(points, resolution))
    assert_same_array_bytes(points, before)


@pytest.mark.parametrize("dtype", [np.float32, np.float64])
def test_random_noncontiguous_voxel_means_match_numpy_bytes(dtype):
    generator = np.random.default_rng(23)
    storage = generator.uniform(-10., 10., size=(411, 4)).astype(dtype)
    points = storage[::-2, ::2]
    assert not points.flags.c_contiguous
    assert_same_array_bytes(_voxel_means(points, .1), numpy_voxel_oracle(points, .1))


def test_mixed_magnitude_same_voxel_preserves_input_order_rounding():
    # Reordering sums within a voxel can lose/add representable low bits even
    # if the output cell order and counts are unchanged.
    points = np.array([[1e16, 1.], [1., 1e16], [1., 1.], [2., 2.],
                       [-1e16, -1.], [-1., -1e16], [-1., -1.]], dtype=np.float64)
    resolution = 2e16
    original = numpy_voxel_oracle(points, resolution)
    reversed_result = numpy_voxel_oracle(points[::-1], resolution)
    assert original.tobytes() != reversed_result.tobytes(), "Fixture must expose summation order"
    assert_same_array_bytes(_voxel_means(points, resolution), original)
    assert_same_array_bytes(_voxel_means(points[::-1], resolution), reversed_result)


def test_empty_voxel_mean_retains_original_float64_shape():
    points = np.empty((0, 2), dtype=np.float64)
    assert_same_array_bytes(_voxel_means(points, .1), numpy_voxel_oracle(points, .1))
