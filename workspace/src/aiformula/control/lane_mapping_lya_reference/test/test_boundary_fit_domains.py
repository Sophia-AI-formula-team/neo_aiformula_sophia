"""Independent finite-face-domain oracles, not a pose or mapping experiment.

R15 changes only where an unchanged fitted polynomial may be evaluated: the
original selected, connected faces, clipped to the original signed arc radius.
It does not turn the white/black bracket into tangent support, add samples to
the fit, connect a gap, or relax the unique-root/J/PCA/consensus gates. Literal
quadratic-fit arithmetic is still tested in test_boundary_extraction_equivalence.
"""
import math

import numpy as np
import pytest

from lane_mapping_lya_reference import paint_geometry as paint


def _case(vertices, fractions=None, *, origin=(0., 0.), normal=(0., 1.),
          scale=1., offsets=None, selected=None, radius=10.):
    vertices = np.asarray(vertices, dtype=np.float64)
    segments = np.stack((vertices[:-1], vertices[1:]), axis=1)
    count = len(segments)
    if fractions is None:
        fractions = np.full(count, .5)
    fractions = np.asarray(fractions, dtype=np.float64)
    centers = segments[:, 0] + fractions[:, None]*(segments[:, 1]-segments[:, 0])
    if offsets is None:
        lengths = np.linalg.norm(segments[:, 1]-segments[:, 0], axis=1)
        offsets = np.r_[0., np.cumsum(lengths[:-1])] + fractions*lengths
    return dict(origins=np.asarray([origin], dtype=np.float64),
                normals=np.asarray([normal], dtype=np.float64),
                scales=np.asarray([scale], dtype=np.float64),
                centers=centers[None], segments=segments[None],
                arc_offsets=np.asarray([offsets], dtype=np.float64),
                weights=np.asarray([np.ones(count, dtype=bool) if selected is None else selected]),
                radius=radius)


def _literal_face_oracle(case):
    """Scalar interval clipping along each original directed straight face.

    This uses face-length coordinates measured from endpoint zero, rather than
    the production batched centre-to-endpoint fractions. It never sees W/B.
    """
    result = np.full((len(case['origins']), 2), np.nan)
    radius = case['radius']
    for row, origin in enumerate(case['origins']):
        normal, scale = case['normals'][row], case['scales'][row]
        selected = np.flatnonzero(case['weights'][row])
        if not len(selected) or np.any(np.diff(selected) != 1):
            continue
        if (not np.isfinite(origin).all() or not np.isfinite(normal).all()
                or not np.isfinite(scale) or scale <= 0.):
            continue
        tangent = np.array([normal[1], -normal[0]])
        values = []
        for position, index in enumerate(selected):
            first, last = case['segments'][row, index]
            center = case['centers'][row, index]
            offset = case['arc_offsets'][row, index]
            if (not np.isfinite([first, last, center]).all()
                    or not np.isfinite(offset) or abs(offset) > radius):
                break
            if position and not np.array_equal(
                    case['segments'][row, selected[position-1], 1], first):
                break
            length = math.hypot(*(last-first))
            before = math.hypot(*(center-first))
            after = math.hypot(*(last-center))
            if length == 0. or before == 0. or after == 0.:
                break
            low = max(0., before-radius-offset)
            high = min(length, before+radius-offset)
            if low > high:
                break
            for distance in (low, high):
                if distance == 0.:
                    point = first
                elif distance == length:
                    point = last
                else:
                    point = first+(distance/length)*(last-first)
                values.append(float(np.dot(point-origin, tangent)/scale))
        else:
            result[row] = min(values), max(values)
    return result


def _call(case):
    before = {key: (value.shape, value.dtype.str, value.tobytes())
              for key, value in case.items() if isinstance(value, np.ndarray)}
    result = paint._finite_face_fit_bounds(**case)
    assert before == {key: (value.shape, value.dtype.str, value.tobytes())
                      for key, value in case.items() if isinstance(value, np.ndarray)}
    assert result.shape == (len(case['origins']), 2)
    assert result.dtype == np.float64
    return result


def _line_root(x, bounds):
    # A horizontal fitted line and an independently observed vertical W/B ray.
    return paint._quantized_curve_positions(
        np.array([[x, -.25]]), np.array([[x, .25]]), np.zeros((1, 2)),
        np.array([[0., 1.]]), np.zeros((1, 3)), np.ones(1),
        np.asarray(bounds), np.array([True]))


def test_original_face_extends_center_hull_but_wb_is_not_the_domain():
    case = _case([[-1., 0.], [0., 0.], [1., 0.]], offsets=[-.5, .5])
    bounds = _call(case)
    np.testing.assert_array_equal(bounds, [[-1., 1.]])
    old_center_bounds = np.array([[-.5, .5]])
    assert not _line_root(.75, old_center_bounds)[2][0]
    assert _line_root(.75, bounds)[2][0]
    # This root is still strictly inside W/B, but outside every observed face.
    point, _, valid = _line_root(1.25, bounds)
    assert not valid[0] and np.isnan(point).all()


@pytest.mark.parametrize('edge', [-1., 1.])
def test_closed_face_endpoint_does_not_admit_nextafter_extrapolation(edge):
    case = _case([[-1., 0.], [0., 0.], [1., 0.]], offsets=[-.5, .5])
    bounds = _call(case)
    assert _line_root(edge, bounds)[2][0]
    outside = np.nextafter(edge, -np.inf if edge < 0. else np.inf)
    assert not _line_root(outside, bounds)[2][0]


def test_endpoints_are_clipped_at_original_radius_not_whole_face_extrema():
    case = _case([[-2., 0.], [-1., 0.], [0., 0.], [1., 0.], [2., 0.]],
                 offsets=[-1.5, -.5, .5, 1.5], radius=1.75)
    np.testing.assert_array_equal(_call(case), [[-1.75, 1.75]])
    np.testing.assert_array_equal(_call(case), _literal_face_oracle(case))
    assert not _line_root(1.875, _call(case))[2][0]


def test_perspective_center_need_not_bisect_projected_face():
    # A projective image midpoint can be 1/4 of the way along its metric face.
    case = _case([[0., 0.], [4., 0.]], [.25], origin=(1., 0.),
                 offsets=[0.], radius=2.)
    np.testing.assert_array_equal(_call(case), [[-1., 2.]])
    np.testing.assert_array_equal(_call(case), _literal_face_oracle(case))
    assert not np.array_equal(_call(case), [[-2., 2.]])  # false midpoint assumption
    assert not np.array_equal(_call(case), [[-1., 3.]])  # unclipped endpoint hull


def test_selected_center_at_radius_has_only_inward_half_face_support():
    case = _case([[0., 0.], [1., 0.]], [.25], origin=(0., 0.),
                 offsets=[1.], radius=1.)
    np.testing.assert_array_equal(_call(case), [[0., .25]])


@pytest.mark.parametrize('bad', ['nan', 'far', 'zero'])
def test_nonselected_faces_cannot_supply_bounds_or_poison_selected_run(bad):
    case = _case([[0., 0.], [1., 0.], [2., 0.], [3., 0.], [4., 0.]],
                 selected=[False, True, True, False], offsets=[-.5, .5, 1.5, 2.5])
    for index in (0, 3):
        if bad == 'nan':
            case['segments'][0, index] = np.nan
            case['centers'][0, index] = np.nan
            case['arc_offsets'][0, index] = np.nan
        elif bad == 'far':
            case['segments'][0, index] += 1.e6
            case['centers'][0, index] += 1.e6
        else:
            case['segments'][0, index] = case['centers'][0, index]
    np.testing.assert_array_equal(_call(case), [[1., 3.]])


@pytest.mark.parametrize('kind', ['unselected_gap', 'endpoint_gap', 'nextafter_gap', 'reversed_face'])
def test_separate_runs_never_form_a_tangent_hull(kind):
    case = _case([[0., 0.], [1., 0.], [2., 0.], [3., 0.]], offsets=[-.5, .5, 1.5])
    if kind == 'unselected_gap':
        case['weights'][0, 1] = False
    elif kind == 'endpoint_gap':
        case['segments'][0, 1, 0, 0] += .25
    elif kind == 'nextafter_gap':
        case['segments'][0, 1, 0, 0] = np.nextafter(1., np.inf)
    else:
        case['segments'][0, 1] = case['segments'][0, 1, ::-1]
    assert np.isnan(_call(case)).all()


def test_partition_does_not_borrow_adjacent_unselected_faces():
    # Six original faces are available in this fit domain; a neighbouring
    # partition has six more. The helper does not alter the six-point gate.
    vertices = np.column_stack((np.arange(13, dtype=float), np.zeros(13)))
    case = _case(vertices, selected=[True]*6+[False]*6, radius=20.)
    np.testing.assert_array_equal(_call(case), [[0., 6.]])


def test_connected_closed_contour_window_can_cross_original_array_seam():
    vertices = np.array([[0., 0.], [2., 0.], [2., 2.], [0., 2.], [0., 0.]])
    whole = _case(vertices, offsets=[0., 2., 4., -2.])
    window = dict(whole)
    # Directed order is last face -> first -> second, not sorted face IDs.
    order = [3, 0, 1]
    for field in ('segments', 'centers', 'arc_offsets', 'weights'):
        window[field] = whole[field][:, order].copy()
    np.testing.assert_array_equal(_call(window), [[0., 2.]])
    np.testing.assert_array_equal(_call(window), _literal_face_oracle(window))
    # Changing the arbitrary seam of a fully selected loop changes no support.
    for shift in (1, 2, 3):
        moved = dict(whole)
        for field in ('segments', 'centers', 'arc_offsets', 'weights'):
            moved[field] = np.roll(whole[field], shift, axis=1)
        np.testing.assert_array_equal(_call(moved), _call(whole))


@pytest.mark.parametrize('angle', [0., .37, -1.2, math.pi/2.])
@pytest.mark.parametrize('translation', [(0., 0.), (13., -7.)])
def test_rigid_transform_preserves_chart_domain_and_scalar_oracle(angle, translation):
    case = _case([[-2., 0.], [-1., 0.], [0., 0.], [2., 0.]],
                 [.25, .75, .25], origin=(-.25, 0.), scale=2.,
                 offsets=[-1.5, 0., .75], radius=1.75)
    baseline = _literal_face_oracle(case)
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    translation = np.asarray(translation)
    for field in ('origins', 'centers', 'segments'):
        case[field] = case[field] @ rotation.T + translation
    case['normals'] = case['normals'] @ rotation.T
    actual = _call(case)
    np.testing.assert_allclose(actual, _literal_face_oracle(case), rtol=0., atol=5.e-15)
    np.testing.assert_allclose(actual, baseline, rtol=0., atol=5.e-15)


@pytest.mark.parametrize('kind', ['no_selected', 'nan_center', 'nan_endpoint', 'nan_arc',
                                  'outside_radius', 'nan_origin', 'nan_normal',
                                  'nan_scale', 'infinite_scale', 'zero_scale',
                                  'negative_scale', 'zero_face', 'collapsed_away_from_center'])
def test_invalid_selected_geometry_cannot_expand_center_domain(kind):
    case = _case([[-1., 0.], [0., 0.], [1., 0.]], offsets=[-.5, .5], radius=1.)
    if kind == 'no_selected': case['weights'][:] = False
    elif kind == 'nan_center': case['centers'][0, 0, 0] = np.nan
    elif kind == 'nan_endpoint': case['segments'][0, 0, 0, 0] = np.nan
    elif kind == 'nan_arc': case['arc_offsets'][0, 0] = np.nan
    elif kind == 'outside_radius': case['arc_offsets'][0, 0] = np.nextafter(-1., -np.inf)
    elif kind == 'nan_origin': case['origins'][0, 0] = np.nan
    elif kind == 'nan_normal': case['normals'][0, 0] = np.nan
    elif kind == 'nan_scale': case['scales'][0] = np.nan
    elif kind == 'infinite_scale': case['scales'][0] = np.inf
    elif kind == 'zero_scale': case['scales'][0] = 0.
    elif kind == 'negative_scale': case['scales'][0] = -1.
    else:
        # Use one face so connectivity cannot incidentally hide this rejection.
        for field in ('centers', 'segments', 'arc_offsets', 'weights'):
            case[field] = case[field][:, :1].copy()
        if kind == 'zero_face':
            case['segments'][0, 0] = case['centers'][0, 0]
        else:
            case['segments'][0, 0] = [-.75, 0.]
    assert np.isnan(_call(case)).all()


@pytest.mark.parametrize('rows, columns', [(0, 0), (0, 3), (2, 0)])
def test_empty_rows_or_empty_support_are_not_observed_domains(rows, columns):
    case = dict(origins=np.zeros((rows, 2)), normals=np.zeros((rows, 2)),
                scales=np.ones(rows), centers=np.zeros((rows, columns, 2)),
                segments=np.zeros((rows, columns, 2, 2)),
                arc_offsets=np.zeros((rows, columns)),
                weights=np.zeros((rows, columns), dtype=bool), radius=.45)
    assert np.isnan(_call(case)).all()


def test_direction_fit_values_counts_and_scales_do_not_change(monkeypatch):
    # A literal straight contour with dyadic coordinates avoids endpoint-rounding
    # ambiguity. No pose solver, mask generation, or map state is involved.
    x = np.arange(-8, 9, dtype=float)/32.
    points = np.column_stack((x, np.zeros(len(x))))
    segments = np.stack((points-[1./64., 0.], points+[1./64., 0.]), axis=1)
    white, black = points-[0., 1./64.], points+[0., 1./64.]
    polarity = np.tile([0, 1], (len(x), 1))
    stable = np.ones(len(x), dtype=bool)
    paths = [(np.arange(len(x)), False)]
    arguments = (points, segments, white, black, polarity, stable, paths, dict(paint.DEFAULT_CONFIG))
    actual = paint._directions(*arguments)
    with monkeypatch.context() as context:
        context.setattr(paint, '_finite_face_fit_bounds_indexed',
                        lambda origins, *args: np.full((len(origins), 2), np.nan))
        center_domain = paint._directions(*arguments)
    assert np.all(actual[1]) and np.all(actual[2] >= paint.DEFAULT_CONFIG['normal_min_points'])
    # Normal, validity, count, radius, coefficients and scale remain bit exact.
    for index in range(6):
        assert actual[index].dtype == center_domain[index].dtype
        assert actual[index].shape == center_domain[index].shape
        assert actual[index].tobytes() == center_domain[index].tobytes()
    assert np.any(actual[6][:, 0] < center_domain[6][:, 0])
    assert np.any(actual[6][:, 1] > center_domain[6][:, 1])
