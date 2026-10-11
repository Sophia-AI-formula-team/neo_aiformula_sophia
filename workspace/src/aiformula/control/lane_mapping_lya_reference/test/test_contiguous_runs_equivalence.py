"""Cell-key-only refactor against the literal R23 directed-run algorithm.

Oracle source: commit 764d860e64c5fe726fa03346f2142571fe1b2a99,
paint_geometry.py SHA256
7f8023c67829b2e4fbf33d636ee71aed30f72a7b4a8558af7f13a17f3539ce90.
The literal below deliberately retains the two int64 coordinate comparisons,
original matrix products, closed-seam merge, sort and member order. No Git,
diagnostic files, estimator, extraction, deadlines or native kernel is needed.

SimpleNamespace supplies exactly the fields consumed by this private pure
partitioner. Invalid/omitted faces here mean NOT eligible, not invented links.
This tests discrete output equivalence, not physical geometry quality.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from lane_mapping_lya_reference import paint_geometry as paint


def _literal_r23_runs(obs, eligible, cells, cosine):
    """Partition actual directed contours, never collect a cell's re-entry.

    Every face in a run is signed-compatible with every earlier face in that
    run. This avoids fixed angle-sector discontinuities without combining two
    oppositely directed sides. Invalid faces break runs even inside one cell.
    """
    admitted = np.zeros(len(obs.points), dtype=bool)
    admitted[eligible] = True
    all_cells = np.zeros((len(obs.points), 2), dtype=np.int64)
    all_cells[eligible] = cells
    visited = np.zeros(len(obs.points), dtype=bool)
    output = []
    normals = obs.model_normals
    for seed in list(np.flatnonzero(obs.previous < 0)) + list(range(len(obs.points))):
        if visited[seed]:
            continue
        path, current = [], int(seed)
        while current >= 0 and not visited[current]:
            visited[current] = True
            path.append(current)
            current = int(obs.next[current])
        parts, run = [], []
        for face in path:
            same = (bool(run) and admitted[face]
                    and all_cells[face, 0] == all_cells[run[0], 0]
                    and all_cells[face, 1] == all_cells[run[0], 1]
                    and np.all(normals[run] @ normals[face] >= cosine))
            if run and not same:
                parts.append(run)
                run = []
            if admitted[face]:
                run.append(face)
        if run:
            parts.append(run)
        # A closed contour's arbitrary iteration start is not a physical cut.
        # Merge across it only if the original faces are genuinely adjacent.
        if (len(parts) > 1 and obs.next[parts[-1][-1]] == parts[0][0]
                and all_cells[parts[-1][0], 0] == all_cells[parts[0][0], 0]
                and all_cells[parts[-1][0], 1] == all_cells[parts[0][0], 1]
                and np.all(normals[parts[-1]] @ normals[parts[0]].T >= cosine)):
            parts[0] = parts[-1] + parts[0]
            parts.pop()
        for faces in parts:
            first = faces[0]
            output.append((tuple(map(int, all_cells[first])), int(obs.contour_ids[first]), faces))
    return sorted(output, key=lambda item: (item[0], item[1], item[2][0]))


def _observation(normals, paths=None, closed=False, contour_ids=None):
    normals = np.asarray(normals, dtype=np.float64).reshape(-1, 2)
    count = len(normals)
    paths = [list(range(count))] if paths is None else paths
    closure = [closed] * len(paths) if isinstance(closed, bool) else closed
    previous = np.full(count, -1, dtype=np.int64)
    following = previous.copy()
    labels = np.full(count, -1, dtype=np.int64)
    for label, (path, loop) in enumerate(zip(paths, closure)):
        for face in path:
            assert labels[face] == -1
            labels[face] = label
        pairs = list(zip(path, path[1:]))
        if loop and path:
            pairs.append((path[-1], path[0]))
        for first, second in pairs:
            following[first], previous[second] = second, first
    assert np.all(labels >= 0) or count == 0
    if contour_ids is not None:
        labels = np.asarray(contour_ids, dtype=np.int64)
        assert labels.shape == (count,)
    return SimpleNamespace(points=np.zeros((count, 2)), model_normals=normals,
                           previous=previous, next=following, contour_ids=labels)


def _normals(degrees):
    angles = np.deg2rad(degrees)
    return np.column_stack((np.cos(angles), np.sin(angles)))


def _check(obs, eligible, cells, cosine=math.cos(math.pi / 6.)):
    eligible = np.asarray(eligible, dtype=np.int64)
    cells = np.asarray(cells, dtype=np.int64).reshape(-1, 2)
    inputs = (obs.points, obs.model_normals, obs.previous, obs.next,
              obs.contour_ids, eligible, cells)
    before = [(array.dtype.str, array.shape, array.strides, array.tobytes()) for array in inputs]
    expected = _literal_r23_runs(obs, eligible, cells, cosine)
    actual = paint._contiguous_cell_runs(obs, eligible, cells, cosine)
    assert actual == expected
    assert before == [(array.dtype.str, array.shape, array.strides, array.tobytes()) for array in inputs]
    # Equality alone must not conceal changed Python/public output types.
    for cell, contour, faces in actual:
        assert type(cell) is tuple and len(cell) == 2
        assert all(type(value) is int for value in cell)
        assert type(contour) is int and type(faces) is list
        assert all(type(face) is int for face in faces)
    assert sorted(face for _, _, faces in actual for face in faces) == sorted(set(eligible.tolist()))
    return actual


@pytest.mark.parametrize('count', [0, 7])
def test_empty_eligible_has_no_default_zero_cell_support(count):
    obs = _observation(np.tile([1., 0.], (count, 1)))
    assert _check(obs, [], []) == []


@pytest.mark.parametrize('closed', [False, True])
@pytest.mark.parametrize('eligible', [[], [0]])
def test_single_face_open_closed_and_omitted(closed, eligible):
    obs = _observation([[1., 0.]], closed=closed)
    result = _check(obs, eligible, [[-3, 8]] if eligible else [])
    assert result == ([((-3, 8), 0, [0])] if eligible else [])


@pytest.mark.parametrize('count', [2, 6, 33])
def test_all_compatible_run_preserves_every_member(count):
    obs = _observation(np.tile([1., -0.], (count, 1)))
    assert _check(obs, np.arange(count), np.tile([-7, 9], (count, 1))) == [
        ((-7, 9), 0, list(range(count)))]


@pytest.mark.parametrize('closed', [False, True])
@pytest.mark.parametrize('pattern', [
    [0, 0, 1, 0, 0],       # Cell re-entry is not globally merged.
    [0, 1, 0, 1, 0],
    [-1, -1, 2, 2, -1],
    [0, 0, 0, 0, 0],
])
def test_cell_changes_reentry_and_closed_seam(closed, pattern):
    obs = _observation(np.tile([0., 1.], (len(pattern), 1)), closed=closed)
    result = _check(obs, np.arange(len(pattern)), np.column_stack((pattern, np.zeros(len(pattern), dtype=np.int64))))
    if pattern == [0, 0, 1, 0, 0]:
        assert result == ([((0, 0), 0, [3, 4, 0, 1]), ((1, 0), 0, [2])] if closed else
                          [((0, 0), 0, [0, 1]), ((0, 0), 0, [3, 4]), ((1, 0), 0, [2])])


@pytest.mark.parametrize('closed', [False, True])
def test_ineligible_nan_faces_break_runs_without_becoming_support(closed):
    normals = np.tile([1., 0.], (7, 1))
    normals[[0, 3]] = np.nan
    obs = _observation(normals, closed=closed)
    assert _check(obs, [1, 2, 4, 5, 6], np.zeros((5, 2), dtype=np.int64)) == [
        ((0, 0), 0, [1, 2]), ((0, 0), 0, [4, 5, 6])]


def test_signed_normals_do_not_merge_opposite_sides():
    obs = _observation([[1., 0.], [1., 0.], [-1., -0.], [-1., 0.]])
    assert _check(obs, range(4), np.zeros((4, 2), dtype=np.int64)) == [
        ((0, 0), 0, [0, 1]), ((0, 0), 0, [2, 3])]


def test_all_prior_normals_not_only_adjacent_direction():
    obs = _observation(_normals([0., 20., 40.]))
    assert _check(obs, range(3), np.zeros((3, 2), dtype=np.int64)) == [
        ((0, 0), 0, [0, 1]), ((0, 0), 0, [2])]


def test_closed_seam_requires_all_cross_pairs_not_only_touching_faces():
    obs = _observation(_normals([0., 20., 90., -20., 0.]), closed=True)
    result = _check(obs, range(5), [[0, 0], [0, 0], [1, 0], [0, 0], [0, 0]])
    assert result == [((0, 0), 0, [0, 1]), ((0, 0), 0, [3, 4]), ((1, 0), 0, [2])]


@pytest.mark.parametrize('cosine', [.5, math.cos(math.pi / 6.)])
@pytest.mark.parametrize('step', [-1, 0, 1])
def test_exact_cosine_and_adjacent_binary64_values(cosine, step):
    x = cosine if step == 0 else np.nextafter(cosine, -np.inf if step < 0 else np.inf)
    # First normal makes the original two-coordinate product exactly x.
    obs = _observation([[1., 0.], [x, math.sqrt(1.-x*x)]])
    result = _check(obs, [0, 1], [[4, -8], [4, -8]], cosine)
    assert result == ([((4, -8), 0, [0]), ((4, -8), 0, [1])] if step < 0 else
                      [((4, -8), 0, [0, 1])])


@pytest.mark.parametrize('cosine', [0., .5, 1.])
def test_signed_zero_normals_are_not_rewritten(cosine):
    obs = _observation([[1., -0.], [1., 0.], [-0., 1.], [0., 1.]], closed=True)
    _check(obs, range(4), np.zeros((4, 2), dtype=np.int64), cosine)


@pytest.mark.parametrize('reverse', [False, True])
def test_duplicate_eligible_keeps_original_int64_scatter_before_key_conversion(reverse):
    eligible = np.asarray([0, 1, 2, 1, 0], dtype=np.int64)
    cells = np.asarray([[5, 0], [8, 0], [3, 0], [3, 0], [3, 0]], dtype=np.int64)
    if reverse:
        eligible, cells = eligible[::-1], cells[::-1]
    obs = _observation(np.tile([1., 0.], (3, 1)))
    _check(obs, eligible, cells)


def test_large_negative_int64_keys_cannot_round_trip_through_float():
    lo, hi = np.iinfo(np.int64).min, np.iinfo(np.int64).max
    cells = np.asarray([[lo, hi], [lo, hi], [2**53, 0], [2**53+1, 0],
                        [-2**53, 0], [-2**53-1, 0]], dtype=np.int64)
    obs = _observation(np.tile([1., 0.], (6, 1)))
    result = _check(obs, range(6), cells)
    assert len(result) == 5
    assert result[0] == ((int(lo), int(hi)), 0, [0, 1])


def test_disconnected_paths_are_not_merged_even_with_same_contour_id():
    obs = _observation(np.tile([1., 0.], (6, 1)), paths=[[4, 1, 3], [2, 0, 5]],
                       closed=[False, True], contour_ids=np.full(6, 7))
    result = _check(obs, range(6), np.zeros((6, 2), dtype=np.int64))
    assert result == [((0, 0), 7, [0, 5, 2]), ((0, 0), 7, [4, 1, 3])]


@pytest.mark.parametrize('read_only', [False, True])
def test_strided_normals_cells_and_eligible_keep_input_ownership(read_only):
    normal_backing = np.tile([1., -0., 7., 8.], (12, 1))
    obs = _observation(np.tile([1., 0.], (6, 1)), closed=True)
    obs.model_normals = normal_backing[::2, :2]
    cell_backing = np.tile([-5, 3, 91, 92], (12, 1)).astype(np.int64)
    cells = cell_backing[::2, :2]
    eligible = np.arange(12, dtype=np.int64)[::2] // 2
    # Preserve a non-contiguous eligible view too.
    storage = np.zeros(12, dtype=np.int64)
    storage[::2] = eligible
    eligible = storage[::2]
    if read_only:
        for array in (obs.model_normals, cells, eligible):
            array.flags.writeable = False
    _check(obs, eligible, cells)


@pytest.mark.parametrize('seed', range(6))
def test_bounded_deterministic_mixed_paths_against_literal(seed):
    rng = np.random.default_rng(seed)
    count = 48
    permutation = rng.permutation(count)
    paths = [part.tolist() for part in np.split(permutation, [11, 27])]
    normals = _normals(rng.choice([0., 10., 25., 45., 90., 180.], size=count))
    obs = _observation(normals, paths=paths, closed=[True, False, True])
    eligible = np.flatnonzero(rng.random(count) > .2)
    cells = rng.integers(-2, 3, size=(len(eligible), 2), dtype=np.int64)
    if len(eligible):
        # Deliberate duplicate writes with potentially different final cells.
        eligible = np.r_[eligible, eligible[:3]]
        cells = np.concatenate((cells, rng.integers(-2, 3, size=(min(3, len(cells)), 2), dtype=np.int64)))
    _check(obs, eligible, cells)
