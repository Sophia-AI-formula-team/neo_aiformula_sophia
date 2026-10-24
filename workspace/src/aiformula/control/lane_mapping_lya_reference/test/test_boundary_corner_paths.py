"""Independent directed-contour corner fixtures; no mask, history or solver.

Vertices below describe observed face endpoints, and each point is its face's
midpoint. Expected cuts come from the construction (not production PCA output).
The helper is allowed to remove contour links, never to reorder/drop faces or
invent links. Invalid faces stay present: later direction fitting performs the
contiguous stability stop, while this helper must not borrow evidence across it.
These are geometry tests, not runtime-budget or complete-lap evidence.
"""

import math

import numpy as np
import pytest

from lane_mapping_lya_reference import paint_geometry as geometry


def _faces(vertices):
    vertices = np.asarray(vertices, dtype=np.float64).reshape(-1, 2)
    segments = np.stack((vertices[:-1], vertices[1:]), axis=1)
    return segments.mean(axis=1), segments


def _polyline(corners, face_counts):
    """Return a connected piecewise-linear contour and exact corner links."""
    corners = np.asarray(corners, dtype=np.float64)
    assert len(corners) == len(face_counts) + 1
    vertices = [corners[0]]
    for first, last, count in zip(corners[:-1], corners[1:], face_counts):
        vertices.extend(first + (last - first) * fraction
                        for fraction in np.linspace(0., 1., count + 1)[1:])
    points, segments = _faces(vertices)
    offsets = np.cumsum(face_counts)
    cuts = {(int(offset - 1), int(offset)) for offset in offsets[:-1]}
    return points, segments, cuts


def _rotate(points, segments, degrees):
    theta = math.radians(degrees)
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]])
    translation = np.array([1.37, -.91])
    return points @ rotation.T + translation, segments @ rotation.T + translation


def _links(paths):
    answer = set()
    for path, closed in paths:
        ids = [int(item) for item in path]
        answer.update(zip(ids[:-1], ids[1:]))
        if closed and ids:
            answer.add((ids[-1], ids[0]))
    return answer


def _call(points, segments, paths=None, stable=None):
    """Check ownership/topology independently on every invocation."""
    if paths is None:
        paths = [(np.arange(len(points), dtype=np.int64), False)]
    if stable is None:
        stable = np.ones(len(points), dtype=bool)
    cfg = dict(geometry.DEFAULT_CONFIG)
    cfg_before = dict(cfg)
    arrays = [points, segments, stable] + [path for path, _ in paths]
    before = [(array.dtype.str, array.shape, array.tobytes()) for array in arrays]
    result = geometry._direction_paths(points, segments, stable, paths, cfg)
    assert isinstance(result, list)
    assert cfg == cfg_before
    assert before == [(array.dtype.str, array.shape, array.tobytes()) for array in arrays]
    expected_ids = sorted(int(item) for path, _ in paths for item in path)
    actual_ids = []
    for piece, closed in result:
        assert isinstance(piece, np.ndarray)
        assert piece.ndim == 1 and np.issubdtype(piece.dtype, np.integer)
        assert isinstance(closed, (bool, np.bool_))
        assert len(piece) > 0
        actual_ids.extend(int(item) for item in piece)
    # In particular, invalid faces cannot vanish or count twice at a cut.
    assert sorted(actual_ids) == expected_ids
    assert _links(result) <= _links(paths)
    return result, _links(paths) - _links(result)


def _literal_variances(points):
    """Analytic two-dimensional covariance eigenvalues, fixture sanity only."""
    n = len(points)
    mx = math.fsum(float(p[0]) for p in points) / n
    my = math.fsum(float(p[1]) for p in points) / n
    xx = math.fsum((float(p[0]) - mx) ** 2 for p in points) / n
    yy = math.fsum((float(p[1]) - my) ** 2 for p in points) / n
    xy = math.fsum((float(p[0]) - mx) * (float(p[1]) - my) for p in points) / n
    spread = math.hypot(xx - yy, 2. * xy)
    return (xx + yy - spread) * .5, (xx + yy + spread) * .5


def _assert_supported_side(points):
    # Fixed original gates, not lowered to make the corner fixtures detectable.
    assert len(points) == geometry.DEFAULT_CONFIG["normal_min_points"] == 6
    minor, major = _literal_variances(points)
    assert major > 1.e-5
    assert minor <= .30 ** 2
    assert minor <= .15 * major


def test_empty_contour_list_is_empty():
    result, cuts = _call(np.empty((0, 2)), np.empty((0, 2, 2)), [])
    assert result == [] and cuts == set()


@pytest.mark.parametrize("degrees", [0., 17., 63., 131., 180.])
@pytest.mark.parametrize("sampling", ["uniform", "strongly_nonuniform"])
def test_straight_faces_never_acquire_a_corner(degrees, sampling):
    if sampling == "uniform":
        distances = np.linspace(0., 2., 97)
    else:
        # A 1000:1 face-length ratio changes density, not the line direction.
        lengths = np.geomspace(.0001, .1, 96)
        distances = np.concatenate(([0.], np.cumsum(lengths)))
    points, segments = _faces(np.column_stack((distances, np.zeros(len(distances)))))
    points, segments = _rotate(points, segments, degrees)
    result, cuts = _call(points, segments)
    assert cuts == set()
    assert len(result) == 1 and not result[0][1]


@pytest.mark.parametrize("degrees", [0., 23., 77., 143.])
@pytest.mark.parametrize("sampling", ["uniform", "perspective_density"])
def test_fine_diagonal_staircase_does_not_cut_at_pixel_right_angles(degrees, sampling):
    # Every individual face turns by 90 degrees, but the sampled paint side
    # follows one diagonal. Slow sampling-density variation models perspective.
    lengths = (np.full(96, .01) if sampling == "uniform"
               else np.geomspace(.002, .06, 96))
    vertices = [np.zeros(2)]
    for length in lengths:
        vertices.append(vertices[-1] + [length, 0.])
        vertices.append(vertices[-1] + [0., length])
    points, segments = _faces(vertices)
    points, segments = _rotate(points, segments, degrees)
    result, cuts = _call(points, segments)
    assert cuts == set()
    assert len(result) == 1


@pytest.mark.parametrize("degrees", [0., 7., 45., 127.])
def test_closed_rectangle_has_exactly_four_observed_corner_cuts(degrees):
    counts = [32, 24, 32, 24]
    points, segments, expected = _polyline(
        [(0., 0.), (1.6, 0.), (1.6, 1.2), (0., 1.2), (0., 0.)], counts)
    expected.add((len(points) - 1, 0))
    points, segments = _rotate(points, segments, degrees)
    path = np.arange(len(points), dtype=np.int64)
    for left, right in expected:
        _assert_supported_side(points[(left - np.arange(5, -1, -1)) % len(points)])
        _assert_supported_side(points[(right + np.arange(6)) % len(points)])
    result, cuts = _call(points, segments, [(path, True)])
    assert cuts == expected
    assert len(result) == 4 and all(not closed for _, closed in result)


@pytest.mark.parametrize("shift", [0, 1, 17, 31, 32, 55, 87, 111])
def test_closed_rectangle_rotation_of_path_start_preserves_the_same_links(shift):
    points, segments, expected = _polyline(
        [(0., 0.), (1.6, 0.), (1.6, 1.2), (0., 1.2), (0., 0.)], [32, 24, 32, 24])
    expected.add((len(points) - 1, 0))
    points, segments = _rotate(points, segments, 29.)
    path = np.roll(np.arange(len(points), dtype=np.int64), shift)
    result, cuts = _call(points, segments, [(path, True)])
    assert cuts == expected
    assert len(result) == 4


@pytest.mark.parametrize("degrees", [0., 37., 119.])
def test_open_l_uses_two_independent_straight_sides(degrees):
    points, segments, expected = _polyline([(0., 0.), (1.2, 0.), (1.2, 1.2)], [24, 24])
    points, segments = _rotate(points, segments, degrees)
    _assert_supported_side(points[18:24])
    _assert_supported_side(points[24:30])
    result, cuts = _call(points, segments)
    assert cuts == expected == {(23, 24)}
    assert len(result) == 2 and all(not closed for _, closed in result)


@pytest.mark.parametrize("short_side", [1, 3, 5])
def test_open_short_cap_cannot_supply_six_faces_of_direction_evidence(short_side):
    points, segments, _ = _polyline([(0., 0.), (1.2, 0.), (1.2, .12)], [24, short_side])
    result, cuts = _call(points, segments)
    assert cuts == set()
    assert len(result) == 1


def test_tiny_l_cannot_invent_two_supported_sides():
    points, segments, _ = _polyline([(0., 0.), (.004, 0.), (.004, .004)], [4, 4])
    result, cuts = _call(points, segments)
    assert cuts == set()
    assert len(result) == 1


@pytest.mark.parametrize("gap_width", [1, 3, 8])
def test_j_cut_at_l_corner_prevents_borrowing_evidence_across_gap(gap_width):
    points, segments, _ = _polyline([(0., 0.), (1.2, 0.), (1.2, 1.2)], [24, 24])
    stable = np.ones(len(points), dtype=bool)
    start = 24 - gap_width // 2
    stable[start:start + gap_width] = False
    # The physical contour is unchanged. It is the evidence, not the path IDs,
    # that is missing, so no corner may be certified using either invalid face.
    result, cuts = _call(points, segments, stable=stable)
    assert cuts == set()
    assert len(result) == 1


def test_two_separate_paths_cannot_supply_each_others_missing_support():
    points, segments, _ = _polyline([(0., 0.), (.15, 0.), (.15, .15)], [5, 5])
    paths = [(np.arange(5, dtype=np.int64), False),
             (np.arange(5, 10, dtype=np.int64), False)]
    result, cuts = _call(points, segments, paths)
    assert cuts == set()
    assert len(result) == 2


@pytest.mark.parametrize("clockwise", [False, True])
@pytest.mark.parametrize("closed", [False, True])
def test_gradual_circle_is_not_a_sequence_of_hard_corners(clockwise, closed):
    angle = np.linspace(0., 2. * math.pi if closed else math.pi / 2., 361 if closed else 121)
    if clockwise:
        angle = -angle
    vertices = 8. * np.column_stack((np.cos(angle), np.sin(angle)))
    if closed:
        vertices[-1] = vertices[0]
    points, segments = _faces(vertices)
    path = np.arange(len(points), dtype=np.int64)
    result, cuts = _call(points, segments, [(path, closed)])
    # Six faces on either side span only a few degrees; curvature is not a
    # greater-than-30-degree discontinuity at a single observed link.
    assert cuts == set()
    assert len(result) == 1 and bool(result[0][1]) is closed


@pytest.mark.parametrize("degrees", [0., 61.])
def test_u_shaped_contour_keeps_nearby_arms_separate_and_cuts_real_corners(degrees):
    points, segments, expected = _polyline(
        [(0., 0.), (0., 1.), (.12, 1.), (.12, 0.)], [50, 12, 50])
    points, segments = _rotate(points, segments, degrees)
    _assert_supported_side(points[50:56])
    result, cuts = _call(points, segments)
    assert cuts == expected == {(49, 50), (61, 62)}
    assert len(result) == 3 and all(not closed for _, closed in result)
    # The arms are closer than the normal radius, but Euclidean proximity is
    # not a contour link and cannot join either arm directly to the other.
    assert all(not (set(map(int, ids)) & set(range(50))
                    and set(map(int, ids)) & set(range(62, 112)))
               for ids, _ in result)


def test_all_invalid_faces_are_retained_without_geometric_certification():
    points, segments, _ = _polyline([(0., 0.), (1.2, 0.), (1.2, 1.2)], [24, 24])
    result, cuts = _call(points, segments, stable=np.zeros(len(points), dtype=bool))
    assert cuts == set()
    assert len(result) == 1


@pytest.mark.parametrize('count', [24, 36, 48])
@pytest.mark.parametrize('degrees', [0., 37., 113.])
def test_coarse_smooth_circle_wide_pca_turn_is_not_a_vertex_corner(count, degrees):
    # At 24 faces the two six-face PCA directions differ by 90 degrees. This
    # does not put their intersection at the observed junction: the curve is
    # smooth, so angle threshold + peak selection alone is insufficient.
    angle = np.linspace(0., 2.*math.pi, count+1)
    vertices = 8.*np.column_stack((np.cos(angle), np.sin(angle)))
    vertices[-1] = vertices[0]
    points, segments = _faces(vertices)
    points, segments = _rotate(points, segments, degrees)
    result, cuts = _call(points, segments, [(np.arange(count), True)])
    assert not cuts
    assert len(result) == 1 and result[0][1]
