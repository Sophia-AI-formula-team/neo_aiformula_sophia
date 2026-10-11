"""Batch frame means must retain the literal per-group NumPy float bits.

The reference is the original ``np.mean(values[faces], axis=0)`` expression,
not another scatter reduction. No local recordings, Git history, native binary,
or diagnostic directory is required. Run on each supported NumPy platform:
reduction-order equality is not assumed to be a public NumPy API guarantee.
"""
import numpy as np
import pytest

from lane_mapping_lya_reference import paint_geometry as paint


def _literal(points, normals, member_groups):
    if not member_groups:
        return np.empty((0, 2), dtype=np.float64), np.empty((0, 2), dtype=np.float64)
    return (np.asarray([np.mean(points[faces], axis=0) for faces in member_groups]),
            np.asarray([np.mean(normals[faces], axis=0) for faces in member_groups]))


def _same_bytes(actual, expected):
    assert isinstance(actual, np.ndarray)
    assert actual.dtype == expected.dtype == np.dtype(np.float64)
    assert actual.shape == expected.shape
    assert actual.tobytes() == expected.tobytes()


def _check(points, normals, member_groups):
    point_before, normal_before = points.tobytes(), normals.tobytes()
    groups_before = [list(group) for group in member_groups]
    expected = _literal(points, normals, member_groups)
    actual = paint._group_frame_means(points, normals, member_groups)
    assert isinstance(actual, tuple) and len(actual) == 2
    for got, wanted in zip(actual, expected):
        _same_bytes(got, wanted)
    assert points.tobytes() == point_before
    assert normals.tobytes() == normal_before
    assert member_groups == groups_before
    return actual


@pytest.mark.parametrize("source_rows", [0, 3])
def test_zero_groups_return_two_empty_matrices(source_rows):
    points = np.zeros((source_rows, 2), dtype=np.float64)
    normals = np.zeros_like(points)
    _check(points, normals, [])


@pytest.mark.parametrize("layout", ["contiguous", "strided", "negative_strides"])
@pytest.mark.parametrize("shuffle_members", [False, True])
def test_all_lengths_and_source_gather_layouts(layout, shuffle_members):
    # The 512-point PCA window is NOT a bound on consensus group length.
    sizes = np.array(list(range(1, 513)) + [513, 1025, 4096], dtype=np.int64)
    ends = np.concatenate(([0], np.cumsum(sizes)))
    rng = np.random.default_rng(1947301)
    total = int(ends[-1])
    points = rng.uniform(-1.e7, 1.e7, (total, 2))
    angles = rng.uniform(-.2, .2, total)
    normals = np.column_stack((np.cos(angles), np.sin(angles)))
    if layout != "contiguous":
        point_storage, normal_storage = np.zeros((total*2, 6)), np.zeros((total*2, 6))
        point_storage[::2, ::3], normal_storage[::2, ::3] = points, normals
        points, normals = point_storage[::2, ::3], normal_storage[::2, ::3]
        assert not points.flags.c_contiguous and not normals.flags.c_contiguous
        if layout == "negative_strides":
            points, normals = points[::-1, ::-1], normals[::-1, ::-1]
    groups = [list(range(int(ends[i]), int(ends[i+1]))) for i in range(len(sizes))]
    if shuffle_members:
        groups = [rng.permutation(group).tolist() for group in groups]
    # Both paths must gather the identical ordered Nx2 rows from the source.
    _check(points, normals, groups)


def test_negative_zero_and_signed_cancellation_keep_literal_bits():
    points = np.array([[0., -0.], [-0., 0.], [-0., -0.], [0., 0.],
                       [1., -1.], [-1., 1.]], dtype=np.float64)
    normals = points[::-1].copy()
    groups = [[0], [1], [2], [3], [0, 1], [2, 2], [4, 5], [5, 4], list(range(6))]
    # Zero normals are deliberate arithmetic fixtures; normalization/geometry
    # admission remains the caller's job, not this private mean helper's job.
    _check(points, normals, groups)


def test_cancellation_uses_original_member_order_without_sorting():
    tiny = np.spacing(1.e7) / 4.
    pattern = np.array([[1.e7, -1.e7], [tiny, -tiny], [-1.e7, 1.e7],
                        [tiny, -tiny], [1.e-290, -1.e-290], [-1.e-290, 1.e-290]])
    points = np.tile(pattern, (700, 1))
    normals = points[::-1].copy()
    groups = [[0, 1, 2, 3], [0, 2, 1, 3], [3, 2, 1, 0],
              list(range(513)), list(range(1025))[::-1], list(range(4096))]
    _check(points, normals, groups)


def test_mixed_magnitude_finite_arithmetic_after_strided_gather():
    # Large magnitudes are a helper arithmetic case, not accepted map positions.
    smallest = np.nextafter(0., 1.)
    pattern = np.array([[1.e200, -1.e200], [-1.e200, 1.e200],
                        [1.e-200, -1.e-200], [smallest, -smallest], [-0., 0.]])
    storage = np.zeros((1040, 6))
    storage[::2, ::3] = pattern[np.arange(520) % len(pattern)]
    points = storage[::2, ::3]
    normals = points[::-1, ::-1]
    _check(points, normals, [[0], list(range(17)), list(range(512)), list(range(513))[::-1]])


def test_helper_does_not_normalize_or_alias_input_arrays():
    angles = np.array([-.1, .1, -.15, .15])
    points = np.arange(8, dtype=np.float64).reshape(4, 2)
    normals = np.column_stack((np.cos(angles), np.sin(angles)))
    points.flags.writeable = normals.flags.writeable = False
    point_mean, normal_mean = _check(points, normals, [[1, 0], [3, 2]])
    assert np.linalg.norm(normal_mean[0]) < 1.
    assert np.linalg.norm(normal_mean[1]) < 1.
    assert not np.shares_memory(point_mean, points)
    assert not np.shares_memory(normal_mean, normals)
    point_mean[0, 0] = 123.
    normal_mean[0, 0] = 123.
    _same_bytes(points, np.arange(8, dtype=np.float64).reshape(4, 2))
    _same_bytes(normals, np.column_stack((np.cos(angles), np.sin(angles))))
