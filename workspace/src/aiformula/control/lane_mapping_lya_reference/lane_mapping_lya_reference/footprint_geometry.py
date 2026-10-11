"""Whole-footprint J4 upper bound and finite local model clipping.

The exact model certified here is F(u,v) = H[:2] @ [u,v,1] /
(H[2] @ [u,v,1]), where each supplied binary64 H entry is exact. H[2]
MUST retain the original ray-plane denominator scale and sign. Arbitrarily
normalizing H changes the meaning of the unchanged -1e-9 horizon gate.

Alternatively, (Hlo, Hhi) encloses every coefficient of one exact mathematical
homography. projector_homography_bounds constructs such an enclosure from the
projector's ALREADY ROUNDED ray_transform, normal, basis, origin and height.
It covers algebraic coefficient composition, not prior inverse(K)/basis/height
calculation errors, physical calibration, linear shape accuracy, or the later
rounding of the old ray/divide/project arithmetic program. Inputs are copied;
no estimator state is read. Invalid rows return (infinity, False), not low J.

For each unit axis shift delta, the numerator of F(z+delta)-F(z) is an
affine vector. Its norm over a convex quadrilateral is bounded by the largest
vertex norm. Each negative affine denominator has its absolute minimum at a
vertex. Their independent positive lower bounds give a conservative quotient.
Every arithmetic operation below expands its computed binary64 endpoints.
This relies on ordinary IEEE-754 binary64 NumPy add/multiply/divide/sqrt,
without flush-to-zero or unsafe fast-math. There is no fixed physical epsilon.
"""

from numbers import Integral

import numpy as np


HORIZON = -1.e-9
SHIFTS = ((0, -1.), (0, 1.), (1, -1.), (1, 1.))


def _point(value):
    value = np.asarray(value, dtype=np.float64)
    return value, value


def _down(value):
    return np.nextafter(value, -np.inf)


def _up(value):
    return np.nextafter(value, np.inf)


def _add(left, right):
    return _down(left[0]+right[0]), _up(left[1]+right[1])


def _sub(left, right):
    return _down(left[0]-right[1]), _up(left[1]-right[0])


def _mul(left, right):
    products = np.stack(np.broadcast_arrays(left[0]*right[0], left[0]*right[1],
                                           left[1]*right[0], left[1]*right[1]))
    return _down(np.min(products, axis=0)), _up(np.max(products, axis=0))


def _divide_positive(numerator, denominator):
    """Caller separately proves 0 <= numerator and 0 < denominator."""
    return (_down(numerator[0]/denominator[1]),
            _up(numerator[1]/denominator[0]))


def _affine(coefficients, uv):
    # No generic dot/BLAS/FMA: two multiplications and two additions.
    lo, hi = coefficients
    return _add(_add(_mul((lo[0], hi[0]), (uv[0][..., 0], uv[1][..., 0])),
                     _mul((lo[1], hi[1]), (uv[0][..., 1], uv[1][..., 1]))),
                (lo[2], hi[2]))


def _dot3(left, right):
    """Interval dot of the last three coordinates, explicit original order."""
    result = _mul((left[0][..., 0], left[1][..., 0]),
                  (right[0][..., 0], right[1][..., 0]))
    for index in (1, 2):
        result = _add(result, _mul((left[0][..., index], left[1][..., index]),
                                   (right[0][..., index], right[1][..., index])))
    return result


def _sign(interval, sign):
    # Multiplication by exactly +/-1 is exact; reverse bounds on negation.
    return interval if sign == 1. else (-interval[1], -interval[0])


def projector_homography_bounds(ray_transform, normal, basis, origin, height):
    """Enclose exact algebraic H from the SAME already-rounded projector data.

    R is base_rotation @ inverse(K) as already computed by _projector, n is
    its already-selected normal, B its 2x3 ground basis, o camera origin and
    h the already-computed positive (n @ o - plane_offset). All supplied
    binary64 values are treated as exact here. No normalization is performed.

    d=n R, P=B R, c=B o; H[:2]=c[:,None] d[None,:]-h P, H[2]=d.
    This encloses that real algebra, including cancellation in composition.
    It does NOT yet bound the extra rounding of the old per-query projector.
    The latter is a separate implementation-level numerical error contract.
    """
    inputs = []
    for value, shape, name in ((ray_transform, (3, 3), 'ray_transform'),
                              (normal, (3,), 'normal'), (basis, (2, 3), 'basis'),
                              (origin, (3,), 'origin')):
        array = np.array(value, dtype=np.float64, copy=True)
        if array.shape != shape or not np.all(np.isfinite(array)):
            raise ValueError(name+' must have finite projector shape')
        inputs.append(array)
    scalar = np.asarray(height)
    if scalar.shape != () or scalar.dtype.kind not in 'fiu' or isinstance(height, (bool, np.bool_)):
        raise ValueError('height must be a finite positive scalar')
    height = float(height)
    if not np.isfinite(height) or height <= 0.:
        raise ValueError('height must be a finite positive scalar')
    r, n, b, o = inputs
    with np.errstate(over='ignore', under='ignore', divide='ignore', invalid='ignore'):
        denominator = _dot3(_point(r.T), _point(n))
        c = _dot3(_point(b), _point(o))
        projected_rows = [_dot3(_point(r.T), _point(row)) for row in b]
        p = tuple(np.stack([row[side] for row in projected_rows]) for side in (0, 1))
        numerator = _sub(_mul((c[0][:, None], c[1][:, None]),
                              (denominator[0][None, :], denominator[1][None, :])),
                         _mul(_point(height), p))
        result = tuple(np.vstack((numerator[side], denominator[side])) for side in (0, 1))
    if (not np.all(np.isfinite(result[0])) or not np.all(np.isfinite(result[1]))
            or np.any(result[0] > result[1])):
        raise ValueError('homography composition could not be finitely enclosed')
    return result


def _convex(quad):
    """Prove four cyclic edges have one strict turn sign; no ambiguous area."""
    edge = _sub(_point(np.roll(quad, -1, axis=1)), _point(quad))
    following = (np.roll(edge[0], -1, axis=1), np.roll(edge[1], -1, axis=1))
    cross = _sub(_mul((edge[0][..., 0], edge[1][..., 0]),
                      (following[0][..., 1], following[1][..., 1])),
                 _mul((edge[0][..., 1], edge[1][..., 1]),
                      (following[0][..., 0], following[1][..., 0])))
    finite = np.all(np.isfinite(cross[0]) & np.isfinite(cross[1]), axis=1)
    return finite & (np.all(cross[0] > 0., axis=1) | np.all(cross[1] < 0., axis=1))


def _inside(uv, width, height):
    return np.all((uv[0][..., 0] >= -.5) & (uv[1][..., 0] <= width-.5)
                  & (uv[0][..., 1] >= -.5) & (uv[1][..., 1] <= height-.5), axis=1)


def quad_j4_upper_bound(homography, quads, width, height):
    """Return upper[N], valid[N] for a cyclic image-space (N,4,2) footprint.

    homography can be a binary64 3x3 H or a tuple (Hlo, Hhi) of finite,
    coefficientwise ordered 3x3 bounds. The latter certifies every fixed H in
    the interval family, including the exact algebraic projector H.
    Both clockwise/counterclockwise convex quadrilaterals are accepted. All
    footprint points and all four shifted footprints must stay in the image;
    the original denominator must be provably < -1e-9 throughout each one.
    No normalization, clipping, epsilon enlargement, or invalid-row omission
    is performed. A valid upper may exceed the caller's unchanged J threshold.
    """
    for value in (width, height):
        if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral)
                or value < 2 or value > 2**31-1):
            raise ValueError('image dimensions must be integers in [2, 2**31-1]')
    if isinstance(homography, tuple) and len(homography) == 2:
        h = tuple(np.array(value, dtype=np.float64, copy=True) for value in homography)
    else:
        exact = np.array(homography, dtype=np.float64, copy=True)
        h = (exact, exact)
    q = np.array(quads, dtype=np.float64, copy=True)
    if (any(value.shape != (3, 3) or not np.all(np.isfinite(value)) for value in h)
            or np.any(h[0] > h[1])):
        raise ValueError('homography must be finite ordered binary64 3 by 3 bounds')
    if q.ndim != 3 or q.shape[1:] != (4, 2):
        raise ValueError('quads must have shape N by 4 by 2')
    upper = np.full(len(q), np.inf)
    valid = np.all(np.isfinite(q), axis=(1, 2))
    if len(q) == 0:
        return upper, valid
    # Invalid rows are still represented in the returned validity array. All
    # floating failures below are converted to rejection, not repaired values.
    with np.errstate(over='ignore', under='ignore', divide='ignore', invalid='ignore'):
        uv = _point(q)
        valid &= _convex(q) & _inside(uv, width, height)
        denominator = _affine((h[0][2], h[1][2]), uv)
        numerators = tuple(_affine((h[0][i], h[1][i]), uv) for i in (0, 1))
        valid &= np.all(np.isfinite(denominator[0]) & np.isfinite(denominator[1])
                        & (denominator[1] < HORIZON), axis=1)
        # Negation is exact; denominator.upper encloses the closest-to-zero
        # value. min is selection, not a new rounded arithmetic operation.
        minimum_d = np.min(-denominator[1], axis=1)
        bounds = []
        for axis, sign in SHIFTS:
            shift = np.zeros(2)
            shift[axis] = sign
            shifted_uv = _add(uv, _point(shift))
            valid &= _inside(shifted_uv, width, height)
            delta_d = _sign((h[0][2, axis], h[1][2, axis]), sign)
            shifted_d = _add(denominator, delta_d)
            valid &= np.all(np.isfinite(shifted_d[0]) & np.isfinite(shifted_d[1])
                            & (shifted_d[1] < HORIZON), axis=1)
            minimum_shifted = np.min(-shifted_d[1], axis=1)
            # The affine numerator's norm is convex. Bound EACH vertex's norm
            # from its component intervals, then select the largest vertex.
            magnitude = []
            for coordinate in range(2):
                delta_n = _sign((h[0][coordinate, axis], h[1][coordinate, axis]), sign)
                value = _sub(_mul(delta_n, denominator),
                             _mul(numerators[coordinate], delta_d))
                magnitude.append(np.maximum(np.abs(value[0]), np.abs(value[1])))
            squared = _add(_mul(_point(magnitude[0]), _point(magnitude[0])),
                           _mul(_point(magnitude[1]), _point(magnitude[1])))
            max_norm = np.max(_up(np.sqrt(squared[1])), axis=1)
            denominator_product = _mul(_point(minimum_d), _point(minimum_shifted))
            proven = ((minimum_d > 0.) & (minimum_shifted > 0.)
                      & np.isfinite(denominator_product[0])
                      & np.isfinite(denominator_product[1])
                      & (denominator_product[0] > 0.) & np.isfinite(max_norm))
            bound = _divide_positive(_point(max_norm), denominator_product)[1]
            valid &= proven & np.isfinite(bound) & (bound >= 0.)
            bounds.append(bound)
        proposed = np.max(np.stack(bounds), axis=0)
        upper[valid] = proposed[valid]
    return upper, valid


def pixel_footprints(white, black):
    """The original WB sampling cell, not a hull across neighboring faces.

    White/black centers differ by one cardinal pixel. Q spans their bracket
    and half a pixel each way along that face. Integer centers/half corners
    are exactly representable over the accepted image range.
    """
    w, b = np.asarray(white), np.asarray(black)
    if (w.ndim != 2 or w.shape[1:] != (2,) or b.shape != w.shape
            or w.dtype.kind not in 'iu' or b.dtype.kind not in 'iu'
            or np.any(w < 0) or np.any(b < 0)
            or np.any(w > 2**31-1) or np.any(b > 2**31-1)):
        raise ValueError('footprints require integer white/black pixel centers')
    w, b = w.astype(np.float64), b.astype(np.float64)
    d = b-w
    if np.any(np.sum(np.abs(d), axis=1) != 1.):
        raise ValueError('footprints require cardinal neighboring pixels')
    half_t = .5*np.column_stack((-d[:, 1], d[:, 0]))
    return np.stack((w-half_t, b-half_t, b+half_t, w+half_t), axis=1)


def _cross(left, right):
    return _sub(_mul((left[0][..., 0], left[1][..., 0]),
                     (right[0][..., 1], right[1][..., 1])),
                _mul((left[0][..., 1], left[1][..., 1]),
                     (right[0][..., 0], right[1][..., 0])))


def clip_local_lines(quads, roots, normals):
    """Clip one first-order boundary model to its own convex metric quad.

    This is a NEW local straight-line approximation inside ONE quantization
    footprint, not recovery of the quadratic curve or physical subpixel truth.
    The root and normal are already admitted by the original label/direction
    gates. No neighboring root, hull, topology link, or extra vote is made.
    All endpoint half-space checks use outward arithmetic on the supplied
    binary64 metric vertices. Uncertain/boundary roots fail closed. Projection
    calibration error and local curvature error are not certified here.
    """
    q = np.array(quads, dtype=np.float64, copy=True)
    p, n = np.asarray(roots, dtype=np.float64), np.asarray(normals, dtype=np.float64)
    if (q.ndim != 3 or q.shape[1:] != (4, 2) or p.shape != (len(q), 2)
            or n.shape != p.shape):
        raise ValueError('line clipping requires matching N by 4 by 2 / N by 2 arrays')
    segments = np.full((len(q), 2, 2), np.nan)
    good = (np.all(np.isfinite(q), axis=(1, 2)) & np.all(np.isfinite(p), axis=1)
            & np.all(np.isfinite(n), axis=1))
    if not len(q):
        return segments, good
    with np.errstate(over='ignore', under='ignore', divide='ignore', invalid='ignore'):
        good &= _convex(q) & (np.abs(np.linalg.norm(n, axis=1)-1.) <= 1.e-6)
        edges = _sub(_point(np.roll(q, -1, axis=1)), _point(q))
        turn = _cross((edges[0][:, 0], edges[1][:, 0]),
                      (edges[0][:, 1], edges[1][:, 1]))
        clockwise = turn[1] < 0.
        q[clockwise] = q[clockwise, ::-1]
        edges = _sub(_point(np.roll(q, -1, axis=1)), _point(q))
        tangent = np.column_stack((n[:, 1], -n[:, 0]))
        a = _cross(edges, _sub(_point(p[:, None, :]), _point(q)))
        b = _cross(edges, _point(tangent[:, None, :]))
        good &= np.all(np.isfinite(a[0]) & (a[0] > 0.)
                       & np.isfinite(b[0]) & np.isfinite(b[1]), axis=1)
        plus = np.min(np.where(b[0] < 0., _down(a[0]/(-b[0])), np.inf), axis=1)
        minus = np.min(np.where(b[1] > 0., _down(a[0]/b[1]), np.inf), axis=1)
        good &= np.isfinite(plus) & np.isfinite(minus) & (plus > 0.) & (minus > 0.)
        # The quotient bound is conservative in exact arithmetic. Certify the
        # actually stored rounded endpoints too; shrink only, never extrapolate.
        pending = np.flatnonzero(good)
        good[:] = False
        for attempt in range(8):
            if not len(pending):
                break
            endpoints = np.stack((p[pending]-minus[pending, None]*tangent[pending],
                                  p[pending]+plus[pending, None]*tangent[pending]), axis=1)
            offset = _sub(_point(endpoints[:, :, None, :]), _point(q[pending, None, :, :]))
            side = _cross((edges[0][pending, None, :, :], edges[1][pending, None, :, :]), offset)
            accepted = (np.all(np.isfinite(side[0]) & (side[0] >= 0.), axis=(1, 2))
                        & np.all(np.isfinite(endpoints), axis=(1, 2))
                        & np.any(endpoints[:, 0] != endpoints[:, 1], axis=1))
            chosen = pending[accepted]
            segments[chosen], good[chosen] = endpoints[accepted], True
            pending = pending[~accepted]
            if attempt < 3:
                plus[pending] = np.nextafter(plus[pending], 0.)
                minus[pending] = np.nextafter(minus[pending], 0.)
            else:
                plus[pending] *= .5
                minus[pending] *= .5
    return segments, good
