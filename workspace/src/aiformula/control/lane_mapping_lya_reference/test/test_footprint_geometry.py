"""Independent exact-rational oracles. No estimator, renderer, or source edits."""

from fractions import Fraction as F
import math

import numpy as np
import pytest

from lane_mapping_lya_reference.footprint_geometry import (
    HORIZON, projector_homography_bounds, quad_j4_upper_bound,
    pixel_footprints, clip_local_lines,
)


def rational(value):
    if isinstance(value, F):
        return value
    return F.from_float(float(value))


def dot(row, values):
    return sum((rational(a)*b for a, b in zip(row, values)), F(0))


def exact_projection(h, point):
    z = [rational(point[0]), rational(point[1]), F(1)]
    denominator = dot(h[2], z)
    return tuple(dot(row, z)/denominator for row in h[:2])


def assert_global_certificate(h, q, upper):
    """Certify upper against exact global vertex-numerator bound, not a grid."""
    z = [[rational(p[0]), rational(p[1]), F(1)] for p in q]
    ds = [dot(h[2], p) for p in z]
    assert all(value < rational(HORIZON) for value in ds)
    for axis, sign in ((0, -1), (0, 1), (1, -1), (1, 1)):
        dd = sign*rational(h[2, axis])
        shifted = [d+dd for d in ds]
        assert all(value < rational(HORIZON) for value in shifted)
        norm_squares = []
        for p, d in zip(z, ds):
            numerator = [sign*rational(h[i, axis])*d-dot(h[i], p)*dd for i in range(2)]
            norm_squares.append(sum((v*v for v in numerator), F(0)))
        product = min(-d for d in ds)*min(-d for d in shifted)
        assert rational(upper)**2 * product**2 >= max(norm_squares)


def square(x=50., y=40., half=.5):
    return np.array([[x-half, y-half], [x+half, y-half],
                     [x+half, y+half], [x-half, y+half]])


@pytest.mark.parametrize('reverse', [False, True])
@pytest.mark.parametrize('h', [
    np.array([[.1, 0., 0.], [0., .1, 0.], [0., 0., -1.]]),
    np.array([[1., .25, -20.], [-.125, .75, 10.], [-.01, -.02, -1.]]),
    np.array([[1.e6, -2.e6, 3.e6], [2.e6, 3.e6, -4.e6], [-.1, -.2, -20.]]),
    np.array([[0., -0., 0.], [-0., 0., -0.], [-0., 0., -1.]]),
])
def test_global_fraction_certificate_and_interior_points(h, reverse):
    q = square()[::-1] if reverse else square()
    upper, valid = quad_j4_upper_bound(h, q[None], 120, 100)
    assert valid.tolist() == [True]
    assert_global_certificate(h, q, upper[0])
    # Independent sample check is supplementary; the exact certificate above
    # is for every point in the continuous quadrilateral, not these samples.
    for s in (0., .125, .5, .875, 1.):
        for t in (0., .25, .75, 1.):
            point = (1-s)*(1-t)*q[0]+s*(1-t)*q[1]+s*t*q[2]+(1-s)*t*q[3]
            base = exact_projection(h, point)
            for shift in ((-1., 0.), (1., 0.), (0., -1.), (0., 1.)):
                other = exact_projection(h, point+shift)
                assert sum(((a-b)**2 for a, b in zip(base, other)), F(0)) <= rational(upper[0])**2


def test_fixed_random_convex_footprints_exact_fraction_bound():
    rng = np.random.RandomState(20261011)
    for unused in range(16):
        theta = rng.uniform(-math.pi, math.pi)
        rotation = np.array([[math.cos(theta), -math.sin(theta)],
                             [math.sin(theta), math.cos(theta)]])
        q = (square(0., 0., .3) @ rotation.T)+[50., 40.]
        h = rng.uniform(-1., 1., (3, 3))
        h[2] = [-.001, -.002, -2.]
        upper, valid = quad_j4_upper_bound(h, q[None], 120, 100)
        assert valid[0]
        assert_global_certificate(h, q, upper[0])


@pytest.mark.parametrize('kind', ['base_horizon', 'shift_horizon', 'positive', 'fov',
                                  'touch_fov_uncertain', 'nan', 'bowtie', 'flat', 'overflow'])
def test_invalid_or_uncertain_geometry_fails_closed(kind):
    h = np.array([[.1, 0., 0.], [0., .1, 0.], [0., 0., -1.]])
    q = square()
    if kind == 'base_horizon':
        h[2] = [0., 0., HORIZON]
    elif kind == 'shift_horizon':
        h[2] = [0., 1., -41.25]  # Base max40.5 valid; +v crosses horizon.
    elif kind == 'positive':
        h[2, 2] = 1.
    elif kind == 'fov':
        q = square(.25, 40.)
    elif kind == 'touch_fov_uncertain':
        q = square(1., 40.)  # Shifted left edge exactly-.5 cannot be proven after outward addition.
    elif kind == 'nan':
        q[0, 0] = np.nan
    elif kind == 'bowtie':
        q[[1, 2]] = q[[2, 1]]
    elif kind == 'flat':
        q[:, 1] = 1.
    elif kind == 'overflow':
        h[0, 0] = np.finfo(float).max
    upper, valid = quad_j4_upper_bound(h, q[None], 120, 100)
    assert not valid[0] and np.isinf(upper[0])


def test_near_horizon_valid_when_provably_strict_and_cannot_normalize_scale():
    h = np.diag([1.e-10, 1.e-10, -2.e-9])
    upper, valid = quad_j4_upper_bound(h, square()[None], 120, 100)
    assert valid[0]
    assert_global_certificate(h, square(), upper[0])
    scaled, admitted = quad_j4_upper_bound(h*.25, square()[None], 120, 100)
    assert not admitted[0] and np.isinf(scaled[0])


def test_rounding_close_to_horizon_never_admits_invalid_exact_denominator():
    q = square()
    for value in (np.nextafter(HORIZON, -np.inf), HORIZON, np.nextafter(HORIZON, np.inf)):
        h = np.diag([1.e-10, 1.e-10, value])
        upper, valid = quad_j4_upper_bound(h, q[None], 120, 100)
        if valid[0]:
            assert rational(value) < rational(HORIZON)
            assert_global_certificate(h, q, upper[0])
        else:
            assert np.isinf(upper[0])


def test_subnormal_signedzero_mixed_rows_and_inputs_unchanged():
    tiny = np.nextafter(0., 1.)
    h = np.array([[tiny, -0., 0.], [0., tiny, -0.], [-0., 0., -1.]])
    q = np.stack((square(), square(.25, 40.)))
    before_h, before_q = h.tobytes(), q.tobytes()
    with np.errstate(all='raise'):
        upper, valid = quad_j4_upper_bound(h, q, 120, 100)
    assert valid.tolist() == [True, False]
    assert_global_certificate(h, q[0], upper[0])
    assert h.tobytes() == before_h and q.tobytes() == before_q
    assert np.geterr()['under'] != 'raise'  # Caller context restored after with.


def test_empty_and_noncontiguous_inputs():
    h = np.diag([.1, .1, -1.])
    upper, valid = quad_j4_upper_bound(h, np.empty((0, 4, 2)), 120, 100)
    assert upper.shape == valid.shape == (0,)
    quads = np.stack((square(), square(55., 45.), square(60., 50.)))[::2]
    upper, valid = quad_j4_upper_bound(h, quads, 120, 100)
    assert np.all(valid)
    for q, bound in zip(quads, upper):
        assert_global_certificate(h, q, bound)


@pytest.mark.parametrize('h,quad,width,height', [
    (np.eye(2), square()[None], 120, 100),
    (np.full((3, 3), np.inf), square()[None], 120, 100),
    (np.eye(3), square(), 120, 100),
    (np.eye(3), square()[None], True, 100),
    (np.eye(3), square()[None], 120.0, 100),
    (np.eye(3), square()[None], 1, 100),
    (np.eye(3), square()[None], 120, 2**31),
])
def test_malformed_api_inputs_rejected(h, quad, width, height):
    with pytest.raises(ValueError):
        quad_j4_upper_bound(h, quad, width, height)


def exact_projector_h(r, n, b, o, height):
    """Independent rational matrix algebra, no production interval functions."""
    d = [sum((rational(n[k])*rational(r[k, j]) for k in range(3)), F(0)) for j in range(3)]
    c = [sum((rational(b[i, k])*rational(o[k]) for k in range(3)), F(0)) for i in range(2)]
    p = [[sum((rational(b[i, k])*rational(r[k, j]) for k in range(3)), F(0))
          for j in range(3)] for i in range(2)]
    return np.array([[c[i]*d[j]-rational(height)*p[i][j] for j in range(3)]
                     for i in range(2)]+[d], dtype=object)


@pytest.mark.parametrize('kind', ['ordinary', 'strong_perspective', 'cancellation', 'signedzero'])
def test_constructed_intervals_enclose_exact_projector_algebra_and_global_bound(kind):
    r = np.array([[.003, 0., -.15], [0., .002, -.1], [-.0001, -.0002, -1.]])
    n = np.array([0., 0., 1.])
    b = np.array([[1., 0., 0.], [0., 1., 0.]])
    o = np.array([.2, -.1, .655])
    height = .648
    if kind == 'strong_perspective':
        r[2] = [0., .01, -.43]  # Shifted maximum v=41.5 remains below horizon.
    elif kind == 'cancellation':
        o[:2] = [1.e6, -1.e6]
        r[:2] += o[:2, None]*r[2]/height
    elif kind == 'signedzero':
        r[0, 1], n[0], b[1, 0] = -0., np.nextafter(0., 1.), -0.
    before = [value.tobytes() for value in (r, n, b, o)]
    interval = projector_homography_bounds(r, n, b, o, height)
    exact = exact_projector_h(r, n, b, o, height)
    for index in np.ndindex(3, 3):
        assert rational(interval[0][index]) <= exact[index] <= rational(interval[1][index])
    q = square()
    upper, valid = quad_j4_upper_bound(interval, q[None], 120, 100)
    assert valid[0]
    assert_global_certificate(exact, q, upper[0])
    assert before == [value.tobytes() for value in (r, n, b, o)]


def test_fixed_random_interval_coefficients_contain_all_exact_mathematical_values():
    rng = np.random.RandomState(44129)
    for unused in range(12):
        r, n, b, o = (rng.uniform(-2., 2., shape) for shape in ((3, 3), (3,), (2, 3), (3,)))
        height = float(rng.uniform(.01, 2.))
        lo, hi = projector_homography_bounds(r, n, b, o, height)
        exact = exact_projector_h(r, n, b, o, height)
        for index in np.ndindex(3, 3):
            assert rational(lo[index]) <= exact[index] <= rational(hi[index])


def test_point_interval_preserves_old_h_api_bits():
    h = np.array([[1., .25, -20.], [-.125, .75, 10.], [-.01, -.02, -1.]])
    quads = np.stack((square(), square(.25, 40.)))
    direct = quad_j4_upper_bound(h, quads, 120, 100)
    interval = quad_j4_upper_bound((h, h), quads, 120, 100)
    assert all(a.tobytes() == b.tobytes() for a, b in zip(direct, interval))


def test_interval_uncertainty_crossing_horizon_fails_closed_without_scale_change():
    lo = np.diag([1.e-10, 1.e-10, -2.e-9])
    hi = lo.copy()
    hi[2, 2] = HORIZON
    upper, valid = quad_j4_upper_bound((lo, hi), square()[None], 120, 100)
    assert not valid[0] and np.isinf(upper[0])


@pytest.mark.parametrize('kind', ['reversed', 'nonfinite', 'shape'])
def test_bad_interval_h_rejected(kind):
    lo = np.diag([.1, .1, -1.])
    hi = lo.copy()
    if kind == 'reversed':
        hi[0, 0] = np.nextafter(lo[0, 0], -np.inf)
    elif kind == 'nonfinite':
        hi[0, 0] = np.inf
    else:
        hi = np.eye(2)
    with pytest.raises(ValueError, match='homography'):
        quad_j4_upper_bound((lo, hi), square()[None], 120, 100)


@pytest.mark.parametrize('kind', ['ray_shape', 'normal_nan', 'basis_shape', 'origin_inf',
                                  'height_zero', 'height_nan', 'height_bool', 'overflow'])
def test_bad_projector_components_rejected_without_normalization(kind):
    r, n, b, o, h = np.eye(3), np.array([0., 0., 1.]), np.eye(3)[:2], np.array([0., 0., 1.]), 1.
    if kind == 'ray_shape':
        r = np.eye(2)
    elif kind == 'normal_nan':
        n[0] = np.nan
    elif kind == 'basis_shape':
        b = np.eye(3)
    elif kind == 'origin_inf':
        o[0] = np.inf
    elif kind == 'height_zero':
        h = 0.
    elif kind == 'height_nan':
        h = np.nan
    elif kind == 'height_bool':
        h = True
    else:
        r.fill(np.finfo(float).max)
        n.fill(np.finfo(float).max)
    with pytest.raises(ValueError):
        projector_homography_bounds(r, n, b, o, h)


def exact_cross(left, right):
    return left[0]*right[1]-left[1]*right[0]


def exact_line_quad_oracle(quad, root, normal):
    """Literal rational half-space intersection, no production clipping code."""
    vertices = [tuple(map(rational, point)) for point in quad]
    p = tuple(map(rational, root))
    tangent = (rational(normal[1]), -rational(normal[0]))
    area = sum((exact_cross(vertices[i], vertices[(i+1) % 4]) for i in range(4)), F(0))
    assert area != 0
    orientation = 1 if area > 0 else -1
    lower, upper = None, None
    halfspaces = []
    for start, stop in zip(vertices, vertices[1:]+vertices[:1]):
        edge = (stop[0]-start[0], stop[1]-start[1])
        offset = (p[0]-start[0], p[1]-start[1])
        a = orientation*exact_cross(edge, offset)
        b = orientation*exact_cross(edge, tangent)
        halfspaces.append((start, edge, orientation))
        if b == 0:
            if a < 0:
                return None
        elif b > 0:
            value = -a/b
            lower = value if lower is None else max(lower, value)
        else:
            value = -a/b
            upper = value if upper is None else min(upper, value)
    if lower is None or upper is None or lower >= upper:
        return None
    return lower, upper, tangent, halfspaces


def assert_segment_exactly_contained(quad, root, normal, segment):
    result = exact_line_quad_oracle(quad, root, normal)
    assert result is not None
    lower, upper, tangent, halfspaces = result
    tangent2 = sum((v*v for v in tangent), F(0))
    p = tuple(map(rational, root))
    endpoints = [tuple(map(rational, row)) for row in segment]
    parameters = []
    for endpoint in endpoints:
        for start, edge, orientation in halfspaces:
            offset = (endpoint[0]-start[0], endpoint[1]-start[1])
            assert orientation*exact_cross(edge, offset) >= 0, 'stored endpoint outside exact original quad'
        offset = (endpoint[0]-p[0], endpoint[1]-p[1])
        parameter = sum((a*b for a, b in zip(offset, tangent)), F(0))/tangent2
        assert lower <= parameter <= upper, 'stored endpoint parameter expands original line extent'
        parameters.append(parameter)
        # Stored endpoints include rounded multiply/add. Bound only their
        # numerical perpendicular residual, not a physical support tolerance.
        scale = max(F(1), *(abs(value) for value in (*p, *endpoint)))
        arithmetic_bound = F(16)*rational(np.finfo(float).eps)*scale*sum(map(abs, tangent))
        assert abs(exact_cross(offset, tangent)) <= arithmetic_bound
    assert parameters[0] <= 0 <= parameters[1]
    assert parameters[0] < parameters[1]
    length2 = sum(((b-a)**2 for a, b in zip(*endpoints)), F(0))
    assert length2 > 0
    assert length2 <= (upper-lower)**2*tangent2, 'stored segment longer than exact line/quad intersection'


@pytest.mark.parametrize('direction', [(1, 0), (-1, 0), (0, 1), (0, -1)])
def test_cardinal_footprints_and_true_cap_exact_extent(direction):
    white = np.array([[20, 20]], dtype=np.int64)
    black = white+direction
    before = white.tobytes(), black.tobytes()
    quads = pixel_footprints(white, black)
    d = np.asarray(direction, dtype=float)
    t = np.array([-d[1], d[0]])
    expected = np.array([white[0]-.5*t, black[0]-.5*t, black[0]+.5*t, white[0]+.5*t])
    assert quads.tobytes() == expected[None].tobytes()
    root = white.astype(float)+.25*d
    segments, valid = clip_local_lines(quads, root, d[None])
    assert valid.tolist() == [True]
    assert_segment_exactly_contained(quads[0], root[0], d, segments[0])
    # A true cap normal points across its white/black pair; no sideward
    # extrapolation is taken from a neighboring face or inferred paint width.
    assert np.all((segments[0]-root[0])@d == 0.)
    assert before == (white.tobytes(), black.tobytes())


@pytest.mark.parametrize('theta', [0., .23, -1.17, math.pi/2])
@pytest.mark.parametrize('clockwise', [False, True])
def test_rotated_slanted_lines_match_fraction_containment_and_length(theta, clockwise):
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]])
    q = np.array([[-.5, -.375], [.5, -.375], [.5, .375], [-.5, .375]]) @ rotation.T+[7.25, -3.5]
    if clockwise:
        q = q[::-1]
    root = np.array([.03125, -.0625])@rotation.T+[7.25, -3.5]
    normal = np.array([3./5., 4./5.])@rotation.T
    before = tuple(array.tobytes() for array in (q, root, normal))
    segments, valid = clip_local_lines(q[None], root[None], normal[None])
    assert valid[0]
    assert_segment_exactly_contained(q, root, normal, segments[0])
    assert before == tuple(array.tobytes() for array in (q, root, normal))


@pytest.mark.parametrize('clockwise', [False, True])
def test_strong_perspective_projected_footprint_exact_binary_geometry(clockwise):
    image_quad = pixel_footprints(np.array([[30, 20]]), np.array([[31, 20]]))[0]
    h = np.array([[.1, .03, -1.], [-.01, .12, .25], [-.001, -.007, -.1]])
    q = np.array([[float(value) for value in exact_projection(h, point)] for point in image_quad])
    if clockwise:
        q = q[::-1]
    root = np.array([float(value) for value in exact_projection(h, [30.4, 20.])])
    target = np.array([float(value) for value in exact_projection(h, [30.4, 20.25])])
    tangent = target-root
    normal = np.array([-tangent[1], tangent[0]])/np.linalg.norm(tangent)
    segments, valid = clip_local_lines(q[None], root[None], normal[None])
    assert valid[0]
    assert_segment_exactly_contained(q, root, normal, segments[0])
    bound, admitted = quad_j4_upper_bound(h, image_quad[None], 80, 60)
    assert admitted[0]
    assert_global_certificate(h, image_quad, bound[0])


@pytest.mark.parametrize('kind', ['outside', 'outside_ulp', 'edge_root', 'corner_root',
                                  'nan_root', 'bad_normal', 'zero_normal', 'nan_quad',
                                  'bowtie', 'flat', 'point_quad'])
def test_clip_degenerate_uncertain_or_invalid_rows_have_no_support(kind):
    q, root, normal = square(0., 0.), np.zeros(2), np.array([0., 1.])
    if kind == 'outside':
        root[0] = .6
    elif kind == 'outside_ulp':
        root[0] = np.nextafter(.5, np.inf)
    elif kind == 'edge_root':
        root[0] = .5
    elif kind == 'corner_root':
        root[:] = .5
    elif kind == 'nan_root':
        root[1] = np.nan
    elif kind == 'bad_normal':
        normal[1] = 1.01
    elif kind == 'zero_normal':
        normal[:] = 0.
    elif kind == 'nan_quad':
        q[0, 0] = np.nan
    elif kind == 'bowtie':
        q[[1, 2]] = q[[2, 1]]
    elif kind == 'flat':
        q[:, 1] = 0.
    elif kind == 'point_quad':
        q[:] = 0.
    segments, valid = clip_local_lines(q[None], root[None], normal[None])
    assert not valid[0] and np.all(np.isnan(segments[0]))


def test_clip_mixed_rows_empty_and_disconnected_footprints_do_not_form_hull():
    quads = np.stack((square(0., 0.), square(4., 0.), square(8., 0.)))
    roots = np.array([[0., 0.], [4., 0.], [np.nan, 0.]])
    normals = np.tile([0., 1.], (3, 1))
    segments, valid = clip_local_lines(quads, roots, normals)
    assert valid.tolist() == [True, True, False]
    for index in (0, 1):
        assert_segment_exactly_contained(quads[index], roots[index], normals[index], segments[index])
    assert np.max(segments[0, :, 0]) <= .5
    assert np.min(segments[1, :, 0]) >= 3.5
    empty, good = clip_local_lines(np.empty((0, 4, 2)), np.empty((0, 2)), np.empty((0, 2)))
    assert empty.shape == (0, 2, 2) and good.shape == (0,)


@pytest.mark.parametrize('white,black', [
    ([[1., 2.]], [[2., 2.]]),
    ([[True, False]], [[False, True]]),
    ([[-1, 2]], [[0, 2]]),
    ([[2**31, 2]], [[2**31-1, 2]]),
    ([[1, 2]], [[2, 3]]),
    ([[1, 2]], [[1, 2]]),
    ([[1, 2]], [[3, 2]]),
    ([1, 2], [2, 2]),
])
def test_footprint_malformed_or_noncardinal_pixels_rejected(white, black):
    with pytest.raises(ValueError):
        pixel_footprints(white, black)


def test_footprints_largest_accepted_integer_and_empty_are_exact():
    white = np.array([[2**31-2, 2**31-2]], dtype=np.uint64)
    black = white+np.array([[1, 0]], dtype=np.uint64)
    q = pixel_footprints(white, black)
    assert rational(q[0, 0, 0]) == F(2**31-2)
    assert rational(q[0, 0, 1]) == F(2**31-2)-F(1, 2)
    assert pixel_footprints(np.empty((0, 2), dtype=int), np.empty((0, 2), dtype=int)).shape == (0, 4, 2)


@pytest.mark.parametrize('quads,roots,normals', [
    (np.zeros((4, 2)), np.zeros((1, 2)), np.ones((1, 2))),
    (np.zeros((1, 4, 2)), np.zeros((2, 2)), np.ones((1, 2))),
    (np.zeros((1, 4, 2)), np.zeros((1, 2)), np.ones((2, 2))),
])
def test_clip_input_shapes_rejected(quads, roots, normals):
    with pytest.raises(ValueError):
        clip_local_lines(quads, roots, normals)


