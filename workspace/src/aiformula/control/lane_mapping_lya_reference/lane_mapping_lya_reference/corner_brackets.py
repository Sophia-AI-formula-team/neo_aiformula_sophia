"""Strict label/footprint existence certificate for a proposed contour split.

This is NOT a corner measurement, fitted position, new map point, or permission
to extend observed support. A caller may use only certificate existence to
support splitting a previously admitted sharp-turn candidate. It must retain
its original per-side PCA, minimum-support, angle, J and observation gates.
There is no QP/extractor import and no label removal, slack or position clamp.

All geometric inequalities use exact rational values of the supplied binary64
inputs. Up to 28 pairs of the original eight half-space boundaries are tested.
A certificate point need not have a float64 representation: authoritative point
and footprint coordinates are immutable Fractions. Logging float conversions
must be marked approximate and must never become estimator observations.
"""
from dataclasses import dataclass
from fractions import Fraction
from itertools import combinations

import numpy as np


MAX_SIDE_FACES = 512
# Numerical validity only; no input is normalized and no half-space is widened.
# This allows a computed unit vector's arithmetic roundoff, not a geometric
# normal-angle or label tolerance. Degenerate bases are rejected conservatively.
_ROUND = Fraction(64, 2**52)
_HALF = Fraction(1, 2)


@dataclass(frozen=True)
class CornerBracketCertificate:
    """Exact existence witness, not a measured corner or additional map extent.

    point_xy and parameters_uv are two-Fraction tuples. offset_intervals holds
    two (lo, hi) Fraction tuples for the original signed side normals. The
    caller must not substitute approximate float copies into its observation.
    """
    point_xy: tuple
    parameters_uv: tuple
    offset_intervals: tuple


def _array(value, shape):
    array = np.asarray(value)
    if array.shape != shape or array.dtype.kind not in 'fiu':
        return None
    # Binary32/64 convert exactly. Do not silently round a wider float or a
    # nonrepresentable integer label before certifying the original inputs.
    if array.dtype.kind == 'f' and array.dtype.itemsize > 8:
        return None
    result = np.array(array, dtype=np.float64, copy=True)
    if not np.isfinite(result).all():
        return None
    if array.dtype.kind in 'iu' and any(int(x) != int(y)
                                      for x, y in zip(array.flat, result.flat)):
        return None
    return result


def _pair(values):
    return tuple(Fraction.from_float(float(value)) for value in values)


def _dot(a, b):
    return a[0]*b[0] + a[1]*b[1]


def _det(a, b):
    return a[0]*b[1] - a[1]*b[0]


def _independent(a, b):
    # Squared form avoids sqrt/float overflow and preserves the exact predicate.
    return _det(a, b)**2 > _ROUND**2 * _dot(a, a) * _dot(b, b)


def _world_point(vertex, axes, uv):
    return tuple(vertex[j] + uv[0]*axes[0][j] + uv[1]*axes[1][j]
                 for j in range(2))


def _original_certificate(point, vertex, axes, normals, whites, blacks):
    """Recheck original labels and independently invert original footprint."""
    delta = tuple(point[j]-vertex[j] for j in range(2))
    determinant = _det(axes[0], axes[1])
    if determinant == 0:
        return None
    # q - vertex = u * axes[0] + v * axes[1], not a rounded transformed matrix.
    uv = (_det(delta, axes[1])/determinant,
          _det(axes[0], delta)/determinant)
    if any(value < -_HALF or value > _HALF for value in uv):
        return None
    if _world_point(vertex, axes, uv) != point:
        return None
    for side in range(2):
        projected = _dot(normals[side], point)
        for white, black in zip(whites[side], blacks[side]):
            if not _dot(normals[side], white) <= projected <= _dot(normals[side], black):
                return None
    return uv


def _dot_enclosure(first, second):
    """Outward enclosure of the exact two-component binary64 dot product."""
    products = first*second
    lower = np.nextafter(products, -np.inf)
    upper = np.nextafter(products, np.inf)
    return (np.nextafter(lower[..., 0]+lower[..., 1], -np.inf),
            np.nextafter(upper[..., 0]+upper[..., 1], np.inf))


def _definitely_disjoint(vertex, axes, normals, whites, blacks):
    """Cheap negative certificate; uncertainty always falls through to exact.

    Necessary conditions only: each side's signed label band must be nonempty
    and meet the original half-face parallelogram. No approximate positive
    witness, polygon clipping, label slack or geometry tolerance is used. The
    exact solver below still decides every case not proved impossible here.
    Inputs are the unchanged validated binary64 arrays, before Fraction work.
    """
    with np.errstate(all='ignore'):
        white_lower, _ = _dot_enclosure(normals[:, None, :], whites)
        _, black_upper = _dot_enclosure(normals[:, None, :], blacks)
        origin_lower, origin_upper = _dot_enclosure(normals, vertex)
        axes_lower, axes_upper = _dot_enclosure(normals[:, None, :], axes[None, :, :])
        # q = vertex + u*axis0 + v*axis1, |u|, |v| <= 1/2. All
        # magnitudes/radii are upper bounds, giving an OUTER footprint range.
        magnitudes = np.maximum(np.abs(axes_lower), np.abs(axes_upper))
        radius = np.nextafter(.5*np.nextafter(magnitudes[:, 0]+magnitudes[:, 1], np.inf), np.inf)
        footprint_lower = np.nextafter(origin_lower-radius, -np.inf)
        footprint_upper = np.nextafter(origin_upper+radius, np.inf)
        lower = np.max(white_lower, axis=1)
        upper = np.min(black_upper, axis=1)
    if not all(np.all(np.isfinite(value)) for value in (
            lower, upper, footprint_lower, footprint_upper)):
        return False
    return bool(np.any((lower > upper) | (lower > footprint_upper) | (upper < footprint_lower)))


def certify_corner_brackets(vertex, face_axes, side_normals, whites, blacks):
    """Return an exact CornerBracketCertificate or None, without input writes.

    Shapes: vertex (2,), face_axes (2,2), side_normals (2,2), whites/blacks
    (2,k,2), 1 <= k <= 512. Each axes ROW is the original face delta:
    q = vertex + u*face_axes[0] + v*face_axes[1], u,v in [-.5,.5]. The signed,
    outward normals must already be unit vectors; no automatic sign/scale fix.
    A caller still enforces its original minimum of six PCA support faces.

    For each side ALL original labels require n.white <= n.q <= n.black.
    Intersect their exact offset bounds and the original half-face footprint.
    A bounded nonempty 2D polygon (including a line/point) has a vertex at two
    independent boundary lines, so exhaustive 8-choose-2 intersection testing
    suffices. Invalid/parallel/ill-conditioned inputs fail closed. Exact-return
    feasibility does not imply anything about the measurement model's truth.
    """
    try:
        raw_whites = np.asarray(whites)
        if (raw_whites.ndim != 3 or raw_whites.shape[0] != 2
                or raw_whites.shape[2] != 2
                or not 1 <= raw_whites.shape[1] <= MAX_SIDE_FACES):
            return None
        shape = raw_whites.shape
        vertex = _array(vertex, (2,))
        face_axes = _array(face_axes, (2, 2))
        side_normals = _array(side_normals, (2, 2))
        whites = _array(raw_whites, shape)
        blacks = _array(blacks, shape)
    except (TypeError, ValueError, OverflowError):
        return None
    if any(value is None for value in (vertex, face_axes, side_normals, whites, blacks)):
        return None
    if _definitely_disjoint(vertex, face_axes, side_normals, whites, blacks):
        return None
    vertex = _pair(vertex)
    axes = tuple(_pair(row) for row in face_axes)
    normals = tuple(_pair(row) for row in side_normals)
    whites = tuple(tuple(_pair(point) for point in side) for side in whites)
    blacks = tuple(tuple(_pair(point) for point in side) for side in blacks)
    if any(abs(_dot(normal, normal)-1) > _ROUND for normal in normals):
        return None
    if not (_independent(axes[0], axes[1]) and _independent(normals[0], normals[1])):
        return None
    intervals = tuple((max(_dot(normals[i], point) for point in whites[i]),
                       min(_dot(normals[i], point) for point in blacks[i]))
                      for i in range(2))
    if any(lo > hi for lo, hi in intervals):
        return None
    # A_i . [u,v] >= b_i. These four box and four label-band inequalities
    # are assembled with exact dots; no pre-rounded lo/hi or matrix is trusted.
    zero, one = Fraction(), Fraction(1)
    rows = [((one, zero), -_HALF), ((-one, zero), -_HALF),
            ((zero, one), -_HALF), ((zero, -one), -_HALF)]
    for normal, (lo, hi) in zip(normals, intervals):
        direction = (_dot(normal, axes[0]), _dot(normal, axes[1]))
        origin = _dot(normal, vertex)
        rows.append((direction, lo-origin))
        rows.append(((-direction[0], -direction[1]), origin-hi))
    for first, second in combinations(rows, 2):
        a, b = first
        c, d = second
        determinant = _det(a, c)
        if determinant == 0:
            continue
        uv = ((b*c[1]-a[1]*d)/determinant,
              (a[0]*d-b*c[0])/determinant)
        if any(_dot(row, uv) < bound for row, bound in rows):
            continue
        point = _world_point(vertex, axes, uv)
        verified_uv = _original_certificate(point, vertex, axes, normals, whites, blacks)
        if verified_uv is not None and verified_uv == uv:
            return CornerBracketCertificate(point, verified_uv, intervals)
    return None
