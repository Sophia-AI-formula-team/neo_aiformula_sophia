"""Independent R20 exact-body oracle for the R21 negative-only screen.

The literal source below is the complete R20 corner_brackets implementation:
commit 51890caaac3be996b6ffef53eee7c64cd8043859, source SHA256
2daa5e14a7a4f7455d455473de71ec1b50e356bb91b11f8d97afd3a725ab55b8.
It does not import production helpers or require a Git checkout/history at CI.
A negative screen must preserve exact feasibility, witness order and all input
admission. Fraction checks here are mathematical checks, not a runtime claim.
"""
from fractions import Fraction
import hashlib
import math
import sys
from types import ModuleType

import numpy as np
import pytest

from lane_mapping_lya_reference import corner_brackets as subject

OLD_EXACT_SOURCE = r'''
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
'''
# Git blob is LF; the independent oracle's source bytes must remain unchanged.
assert hashlib.sha256(OLD_EXACT_SOURCE.lstrip("\n").encode("utf8")).hexdigest() == (
    "2daa5e14a7a4f7455d455473de71ec1b50e356bb91b11f8d97afd3a725ab55b8")
_old = ModuleType("_test_r20_corner_brackets_exact")
sys.modules[_old.__name__] = _old
exec(compile(OLD_EXACT_SOURCE, "<literal R20 corner oracle>", "exec"), _old.__dict__)


def _case(k=6, lower=-.25, upper=.25, vertex=(0., 0.), axes=None, normals=None):
    white = np.zeros((2, k, 2), dtype=np.float64)
    black = np.zeros_like(white)
    for side in (0, 1):
        white[side, :, side] = lower
        black[side, :, side] = upper
        # Tangential labels are not extra normal constraints.
        white[side, :, 1-side] = np.linspace(-.375, .375, k)
        black[side, :, 1-side] = white[side, :, 1-side]
    return [np.array(vertex, dtype=np.float64),
            np.eye(2) if axes is None else np.array(axes, dtype=np.float64),
            np.eye(2) if normals is None else np.array(normals, dtype=np.float64),
            white, black]


def _fields(certificate):
    if certificate is None:
        return None
    return (certificate.point_xy, certificate.parameters_uv, certificate.offset_intervals)


def _f(value):
    return Fraction.from_float(float(value))


def _exact_dot(first, second):
    return _f(first[0])*_f(second[0]) + _f(first[1])*_f(second[1])


def _check_original_inequalities(certificate, arguments):
    if certificate is None:
        return
    vertex, axes, normals, whites, blacks = map(np.asarray, arguments)
    point, uv, intervals = _fields(certificate)
    assert all(type(x) is Fraction for x in point + uv)
    assert all(-Fraction(1, 2) <= x <= Fraction(1, 2) for x in uv)
    assert point == tuple(_f(vertex[j]) + uv[0]*_f(axes[0, j]) +
                          uv[1]*_f(axes[1, j]) for j in range(2))
    for side in (0, 1):
        projected = _f(normals[side, 0])*point[0] + _f(normals[side, 1])*point[1]
        lower = max(_exact_dot(normals[side], value) for value in whites[side])
        upper = min(_exact_dot(normals[side], value) for value in blacks[side])
        assert intervals[side] == (lower, upper)
        assert lower <= projected <= upper
        for white, black in zip(whites[side], blacks[side]):
            assert _exact_dot(normals[side], white) <= projected <= _exact_dot(normals[side], black)


def _snapshot(arguments):
    return [(value.dtype.str, value.shape, value.tobytes())
            if isinstance(value, np.ndarray) else None for value in arguments]


def _equivalent(arguments, *, expected="oracle"):
    before = _snapshot(arguments)
    with np.errstate(over="ignore", invalid="ignore", under="ignore", divide="ignore"):
        old = _old.certify_corner_brackets(*arguments)
        new = subject.certify_corner_brackets(*arguments)
    assert _snapshot(arguments) == before
    assert _fields(new) == _fields(old)  # Exact Fraction values/order; never allclose.
    _check_original_inequalities(new, arguments)
    if expected == "feasible":
        assert new is not None
    elif expected == "infeasible":
        assert new is None
    return new


@pytest.mark.parametrize("k", [1, 6, 31, 512])
def test_old_exact_witness_order_on_feasible_rectangle(k):
    result = _equivalent(_case(k), expected="feasible")
    assert result.point_xy == (Fraction(-1, 4), Fraction(-1, 4))


@pytest.mark.parametrize("sign", [-1., 1.])
def test_original_half_face_equality_is_closed_not_shrunk(sign):
    args = _case(lower=sign*.5, upper=sign*.5)
    result = _equivalent(args, expected="feasible")
    assert result.point_xy == (_f(sign*.5), _f(sign*.5))
    assert result.parameters_uv == result.point_xy


@pytest.mark.parametrize("sign", [-1., 1.])
def test_one_ulp_outside_half_face_remains_infeasible(sign):
    outside = np.nextafter(sign*.5, sign*np.inf)
    _equivalent(_case(lower=outside, upper=outside), expected="infeasible")


@pytest.mark.parametrize("side", [0, 1])
@pytest.mark.parametrize("location", [0., 1., -1., 2.**500])
def test_one_ulp_empty_side_band_matches_fraction_oracle(side, location):
    args = _case()
    args[3][side, :, side] = np.nextafter(location, np.inf)
    args[4][side, :, side] = location
    _equivalent(args, expected="infeasible")


@pytest.mark.parametrize("case", ["empty_band", "outside_footprint"])
def test_certain_impossibility_returns_before_fraction_conversion(case, monkeypatch):
    args = _case(lower=2., upper=1. if case == "empty_band" else 3.)
    assert _old.certify_corner_brackets(*args) is None
    def forbidden(_):
        raise AssertionError("a certifiably negative screen reached Fraction conversion")
    monkeypatch.setattr(subject, "_pair", forbidden)
    assert subject.certify_corner_brackets(*args) is None


def test_possible_screen_result_still_uses_original_exact_solver(monkeypatch):
    original = subject._pair
    calls = []
    def seen(value):
        calls.append(np.array(value, copy=True))
        return original(value)
    monkeypatch.setattr(subject, "_pair", seen)
    _equivalent(_case(), expected="feasible")
    assert calls


@pytest.mark.parametrize("position", range(5))
@pytest.mark.parametrize("dtype", [np.bool_, np.complex128, object, "U32"])
def test_invalid_dtypes_preserve_old_fail_closed_contract(position, dtype):
    args = _case()
    args[position] = args[position].astype(dtype)
    _equivalent(args, expected="infeasible")


@pytest.mark.parametrize("position", range(5))
@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_nonfinite_any_input_is_rejected_before_screen(position, bad):
    args = _case()
    args[position].flat[-1] = bad
    _equivalent(args, expected="infeasible")


@pytest.mark.parametrize("normals", [
    [[0., 0.], [0., 1.]], [[2., 0.], [0., 1.]],
    [[1., 0.], [1., 0.]], [[1., 0.], [-1., 0.]],
    [[1., 0.], [1., 2.**-50]],
])
def test_bad_unit_or_degenerate_normals_do_not_gain_admission(normals):
    _equivalent(_case(normals=normals), expected="infeasible")


@pytest.mark.parametrize("axes", [
    [[0., 0.], [0., 1.]], [[1., 0.], [2., 0.]],
    [[1., 0.], [1., 2.**-50]],
])
def test_degenerate_face_basis_never_gets_a_certificate(axes):
    _equivalent(_case(axes=axes), expected="infeasible")


@pytest.mark.parametrize("bad_shape", [(0, 6, 2), (2, 0, 2), (2, 513, 2), (2, 6, 3), (2, 2)])
def test_original_label_shape_and_capacity(bad_shape):
    args = _case()
    args[3] = np.zeros(bad_shape)
    _equivalent(args, expected="infeasible")


@pytest.mark.parametrize("position", [0, 1, 2, 3, 4])
def test_integer_rounding_is_not_silently_certified(position):
    args = _case()
    args[position] = np.zeros(args[position].shape, dtype=np.uint64)
    args[position].flat[0] = 2**53 + 1
    _equivalent(args, expected="infeasible")


@pytest.mark.parametrize("dtype", [np.float32, np.float64, np.int8, np.int64, np.uint64])
def test_supported_dtypes_keep_old_semantics(dtype):
    args = _case(lower=0., upper=0.)
    args = [value.astype(dtype) for value in args]
    _equivalent(args, expected="feasible")


def test_wider_float_is_rejected_where_it_exists():
    args = _case()
    args[0] = args[0].astype(np.longdouble)
    result = _equivalent(args)
    assert (result is None) == (np.dtype(np.longdouble).itemsize > 8)


@pytest.mark.parametrize("signed_zero", [0., -0.])
def test_signed_zero_input_and_exact_point_certificate(signed_zero):
    args = _case(lower=signed_zero, upper=signed_zero, vertex=(signed_zero, -signed_zero))
    args[1][0, 1] = signed_zero
    args[1][1, 0] = -signed_zero
    result = _equivalent(args, expected="feasible")
    assert result.point_xy == (Fraction(), Fraction())


@pytest.mark.parametrize("exponent", [-1074, -1073, -1022, -500, 500, 1023])
def test_dyadic_extreme_scale_preserves_exact_half_face(exponent):
    scale = math.ldexp(1., exponent)
    args = _case(lower=0., upper=0., axes=[[scale, 0.], [0., scale]])
    # Scale may be subnormal; a zero-width label still has the exact center.
    result = _equivalent(args, expected="feasible")
    assert result.point_xy == (Fraction(), Fraction())


@pytest.mark.parametrize("sign", [-1., 1.])
def test_subnormal_half_face_boundary_is_not_rounded_inwards(sign):
    tiny = float.fromhex("0x0.0000000000001p-1022")
    args = _case(lower=sign*tiny, upper=sign*tiny,
                 axes=[[2*tiny, 0.], [0., 2*tiny]])
    result = _equivalent(args, expected="feasible")
    assert result.parameters_uv == (_f(sign*.5), _f(sign*.5))


@pytest.mark.parametrize("coordinates", [(1.e308, 1.e308), (1.e308, -1.e308),
                                         (-1.e308, 1.e308), (-1.e308, -1.e308)])
def test_overflow_or_cancellation_in_screen_must_fall_back_not_reject(coordinates):
    root_half = math.sqrt(.5)
    args = _case(vertex=coordinates, normals=[[root_half, root_half], [-root_half, root_half]])
    # Every original label is exactly the footprint center. Float dot may
    # overflow, while the Fraction constraints have a valid center witness.
    args[3][:] = coordinates
    args[4][:] = coordinates
    result = _equivalent(args, expected="feasible")
    assert result.point_xy == tuple(_f(value) for value in coordinates)


def test_overflowing_face_corner_arithmetic_still_has_feasible_origin():
    huge = np.finfo(np.float64).max
    args = _case(lower=0., upper=0., axes=[[huge, huge], [huge, -huge]])
    _equivalent(args, expected="feasible")


@pytest.mark.parametrize("readonly", [False, True])
def test_strided_and_readonly_arrays_preserve_input_bytes(readonly):
    args = []
    for value in _case():
        buffer = np.empty(value.shape[:-1] + (4,), dtype=np.float64)
        buffer[..., ::2] = value
        buffer[..., 1::2] = 17.
        view = buffer[..., ::2]
        if readonly:
            view.setflags(write=False)
        args.append(view)
    _equivalent(args, expected="feasible")


@pytest.mark.parametrize("seed", list(range(24)))
def test_fixed_seed_random_certificate_is_identical(seed):
    rng = np.random.RandomState(seed)
    angle = rng.uniform(-math.pi, math.pi)
    normals = [[math.cos(angle), math.sin(angle)],
               [-math.sin(angle), math.cos(angle)]]
    exponent = int(rng.choice([-500, -30, 0, 30, 500]))
    scale = math.ldexp(1., exponent)
    vertex = rng.uniform(-2., 2., size=2)*scale
    axes = np.array([[1., .375], [-.25, 1.]])*scale
    k = int(rng.choice([1, 6, 17]))
    args = _case(k, vertex=vertex, axes=axes, normals=normals)
    if seed % 3 == 0:
        # Known feasible q == vertex, not an approximate normalized oracle.
        args[3][:] = vertex
        args[4][:] = vertex
        _equivalent(args, expected="feasible")
    else:
        args[3] = rng.uniform(-2., 2., size=(2, k, 2))*scale + vertex
        args[4] = rng.uniform(-2., 2., size=(2, k, 2))*scale + vertex
        _equivalent(args)


def test_redundant_label_permutation_keeps_original_exact_witness():
    args = _case(k=31)
    original = _fields(_equivalent(args, expected="feasible"))
    order = np.random.RandomState(191).permutation(31)
    args[3] = args[3][:, order, :]
    args[4] = args[4][:, order, :]
    assert _fields(_equivalent(args, expected="feasible")) == original


@pytest.mark.parametrize("case", ["ordinary", "empty", "signed_zero", "subnormal", "overflow", "cancellation"])
def test_public_certificate_preserves_strict_numpy_error_mode(case):
    args = _case()
    if case == "empty":
        args = _case(lower=2., upper=1.)
    elif case == "signed_zero":
        args = _case(lower=-0., upper=0., vertex=(-0., 0.))
    elif case == "subnormal":
        tiny = float.fromhex("0x0.0000000000001p-1022")
        args = _case(lower=tiny, upper=tiny, axes=[[2*tiny, 0.], [0., 2*tiny]])
    elif case in ("overflow", "cancellation"):
        root_half = math.sqrt(.5)
        vertex = [1.e308, 1.e308 if case == "overflow" else -1.e308]
        args = _case(vertex=vertex, normals=[[root_half, root_half], [-root_half, root_half]])
        args[3][:] = vertex
        args[4][:] = vertex
    before = _snapshot(args)
    with np.errstate(all="raise"):
        expected = _old.certify_corner_brackets(*args)
        actual = subject.certify_corner_brackets(*args)
        assert np.geterr() == dict(divide="raise", over="raise", under="raise", invalid="raise")
    assert _fields(actual) == _fields(expected)
    assert _snapshot(args) == before
    _check_original_inequalities(actual, args)


@pytest.mark.parametrize("first,second", [
    ((0., -0.), (-0., 0.)), ((1., 1.), (1., -1.)),
    ((1., 1.), (1.e308, -1.e308)), ((.6, .8), (1.e308, 1.e308)),
    ((1., 1.), (float.fromhex("0x0.0000000000001p-1022"), 0.)),
    ((.5, -.5), (float.fromhex("0x0.0000000000001p-1022"), 0.)),
    ((2.**500, 2.**-500), (2.**500, -2.**500)),
    ((np.finfo(np.float64).max, np.finfo(np.float64).max), (2., -2.)),
])
def test_direct_dot_enclosure_contains_exact_binary64_dot(first, second):
    exact = _exact_dot(first, second)
    with np.errstate(all="ignore"):
        lower, upper = subject._dot_enclosure(np.array(first), np.array(second))
    # NaN from inf-inf is explicitly indeterminate, never a negative proof.
    if not math.isnan(float(lower)):
        assert float(lower) == -math.inf or _f(lower) <= exact
    if not math.isnan(float(upper)):
        assert float(upper) == math.inf or exact <= _f(upper)


@pytest.mark.parametrize("seed", list(range(20)))
def test_screen_negative_implies_exact_problem_is_infeasible(seed):
    rng = np.random.RandomState(9100+seed)
    args = _case(k=6)
    args[0] = rng.uniform(-2., 2., 2)
    args[1] = rng.uniform(-2., 2., (2, 2))
    angle = rng.uniform(-math.pi, math.pi)
    args[2] = np.array([[math.cos(angle), math.sin(angle)],
                        [-math.sin(angle), math.cos(angle)]])
    args[3] = rng.uniform(-2., 2., (2, 6, 2))
    args[4] = rng.uniform(-2., 2., (2, 6, 2))
    before = _snapshot(args)
    if subject._definitely_disjoint(*args):
        assert _old.certify_corner_brackets(*args) is None
    assert _snapshot(args) == before
    _equivalent(args)

