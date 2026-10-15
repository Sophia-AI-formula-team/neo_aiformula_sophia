"""Certified chart-strip bounds for UNKNOWN competitors (development only).

An axis-aligned box is an enclosure, not the original region. A thin oblique
chart strip may have a very large AABB. This module supplies an additional
distance lower bound; it never confirms a curve, removes a hypothesis, merges
signatures, selects a farther candidate or changes any physical gate.

The certificate encloses BOTH exact inverse-chart source points and separately
rounded binary64 inverse-chart model points for every u,v,d in the supplied
intervals. Fraction arithmetic bounds each multiply/add/divide rounding error;
it is construction-time arithmetic, not a runtime-pose solver. No integration
hook is installed by importing this module.
"""
from dataclasses import dataclass, field
from fractions import Fraction
import math

import numpy as np

from .curve_rounding import step


_SEAL = object()
_EPS = Fraction(1, 2**52)
_TINY = Fraction(1, 2**1074)


def _pair(value, name):
    if (type(value) is not tuple or len(value) != 2
            or any(type(x) is not float or not math.isfinite(x) for x in value)):
        raise ValueError('expected finite builtin-float ' + name)
    return tuple(Fraction(x) for x in value)


def _interval(value, name):
    result = _pair(value, name)
    if result[0] > result[1]:
        raise ValueError('reversed ' + name)
    return result


def _outward(value, upper):
    result = float(value)
    if not math.isfinite(result):
        raise ArithmeticError('unrepresentable chart-strip bound')
    direction = math.inf if upper else -math.inf
    # Subnormal nextafter is intentional certificate construction, not sensor
    # underflow. Restore the caller's NumPy policy after this local operation.
    with np.errstate(all='ignore'):
        if ((Fraction(result) < value) if upper else (Fraction(result) > value)):
            result = step(result, direction)
        # One extra outward step covers equality/zero, not a physical epsilon.
        result = step(result, direction)
    if not math.isfinite(result):
        raise ArithmeticError('unrepresentable outward chart-strip bound')
    return result


def _environment():
    # Decline flushing, non-nearest arithmetic and unexpected excess precision.
    # All calculations after this probe are exact rational operations apart
    # from the explicitly outward conversion. No FP control register is changed.
    tiny = float.fromhex('0x0.0000000000001p-1022')
    one, half = 1., float.fromhex('0x1p-53')
    successor = float.fromhex('0x1.0000000000001p+0')
    return (one + half == one and successor + half == float.fromhex('0x1.0000000000002p+0')
            and tiny * one == tiny and (-tiny) * one == -tiny)


@dataclass(frozen=True)
class InverseChartSlab:
    origin: tuple
    tangent: tuple
    bounds: tuple
    norm_squared_upper: float
    _seal: object = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        if type(self) is not InverseChartSlab or self._seal is not _SEAL:
            raise ValueError('use inverse_chart_slab')


def _inverse_chart_data(origin, tangent, span, lateral, denominator):
    if not _environment():
        raise ArithmeticError('unsupported chart-strip arithmetic environment')
    o, t = _pair(origin, 'origin'), _pair(tangent, 'tangent')
    u, v = _interval(span, 'span'), _interval(lateral, 'lateral')
    d = _interval(denominator, 'denominator')
    q = t[0]*t[0] + t[1]*t[1]
    if q <= 0 or d[0] <= 0 or not d[0] <= q <= d[1]:
        raise ValueError('denominator must enclose positive exact squared norm')
    n = (-t[1], t[0])
    max_u, max_v = max(map(abs, u)), max(map(abs, v))
    errors = []
    for i in (0, 1):
        a, b = abs(t[i])*max_u, abs(n[i])*max_v
        ea, eb = _EPS*a + _TINY, _EPS*b + _TINY
        ec = ea + eb + _EPS*(a+b+ea+eb) + _TINY
        ed = ec/d[0] + _EPS*((a+b+ec)/d[0]) + _TINY
        ee = ed + _EPS*(abs(o[i])+(a+b)/d[0]+ed) + _TINY
        errors.append(ee)
    projection_error = sum(abs(t[i])*errors[i] for i in (0, 1))
    return u, d, q, projection_error


def inverse_chart_slab(origin, tangent, span, lateral, denominator):
    """Bound t dot (world-origin), including rounded inverse-chart evaluation.

    world[i] = origin[i] + (t[i]*u + n[i]*v)/d, n=(-t[1],t[0]).
    Each displayed multiply/add/divide is separately rounded, as in the scalar
    curve endpoint. Denominator must contain both the exact squared norm and
    any rounded denominator used by the caller. A rejected certificate requires
    the existing AABB-only decision; it does NOT permit ignoring that source.
    """
    u, d, q, projection_error = _inverse_chart_data(origin, tangent, span, lateral, denominator)
    # t dot n is EXACTLY zero over the real values of these binary64 entries.
    # Do not replace q/d by one: the model's denominator is rounded.
    values = tuple(q/di * ui for di in d for ui in u)
    lower = min(u[0], min(values)-projection_error)
    upper = max(u[1], max(values)+projection_error)
    bounds = (_outward(lower, False), _outward(upper, True))
    return InverseChartSlab(origin, tangent, bounds, _outward(q, True), _SEAL)


def _points_and_projection(points, origin, tangent):
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError('expected Nx2 points')
    with np.errstate(all='ignore'):
        lo = np.nextafter(points - np.asarray(origin), -np.inf)
        hi = np.nextafter(points - np.asarray(origin), np.inf)
        a, b = lo*np.asarray(tangent), hi*np.asarray(tangent)
        product_lo = np.nextafter(np.minimum(a, b), -np.inf)
        product_hi = np.nextafter(np.maximum(a, b), np.inf)
        projection_lo = np.nextafter(product_lo[:, 0]+product_lo[:, 1], -np.inf)
        projection_hi = np.nextafter(product_hi[:, 0]+product_hi[:, 1], np.inf)
    return points, projection_lo, projection_hi


def squared_distance_lower(points, slab):
    """Conservative Euclidean squared distance to one certified chart strip.

    Nonfinite/uncertain arithmetic returns zero, never an exclusion. The caller
    may combine this bound with an AABB bound by MAX and must retain the original
    candidate-distance upper bound and strict-greater exclusion comparison.
    """
    if type(slab) is not InverseChartSlab or slab._seal is not _SEAL:
        raise ValueError('expected prepared chart-strip certificate')
    points, projection_lo, projection_hi = _points_and_projection(points, slab.origin, slab.tangent)
    if not _environment():
        return np.zeros(len(points))
    with np.errstate(all='ignore'):
        gap = np.maximum(0., np.maximum(np.nextafter(slab.bounds[0]-projection_hi, -np.inf),
                                       np.nextafter(projection_lo-slab.bounds[1], -np.inf)))
        square = np.maximum(0., np.nextafter(gap*gap, -np.inf))
        result = np.maximum(0., np.nextafter(square/slab.norm_squared_upper, -np.inf))
    result[~np.isfinite(result) | ~np.isfinite(points).all(axis=1)] = 0.
    return result


@dataclass(frozen=True)
class InverseChartComplement:
    origin: tuple
    tangent: tuple
    interiors: tuple
    norm_squared_upper: float
    _seal: object = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        if type(self) is not InverseChartComplement or self._seal is not _SEAL:
            raise ValueError('use inverse_chart_complement')


def inverse_chart_complement(origin, tangent, span, lateral, denominator, covered):
    """Inner strips provably disjoint from EVERY original uncovered u region.

    Covered intervals must be the caller's complete certified finite domains,
    not a distance/range crop. The construction SHRINKS them for inverse-chart
    roundoff and denominator error. It never fills gaps or extends endpoints.
    Pending regions outside all covered intervals retain a distance bound, not
    a model. The caller must attach this only to the exact same key's complete
    complement, never to its whole UNKNOWN state or a competing signature.
    """
    u, d, q, error = _inverse_chart_data(origin, tangent, span, lateral, denominator)
    if type(covered) is not tuple:
        raise ValueError('expected immutable covered intervals')
    ratios = (q/d[1], q/d[0])
    result, previous = [], None
    for interval in covered:
        a, b = _interval(interval, 'covered interval')
        if not u[0] <= a < b <= u[1] or (previous is not None and a < previous):
            raise ValueError('covered intervals must be ordered/disjoint within parameter span')
        previous = b
        lower = max(ratio*a for ratio in ratios)+error
        upper = min(ratio*b for ratio in ratios)-error
        if lower < upper:
            safe = (_outward(lower, True), _outward(upper, False))
            if safe[0] < safe[1]:
                result.append(safe)
    return InverseChartComplement(origin, tangent, tuple(result), _outward(q, True), _SEAL)


def complement_squared_distance_lower(points, complement):
    """Distance lower bound to the complete UNKNOWN parameter complement.

    Zero on a gap, an endpoint, unknown arithmetic or incomplete safe interval.
    A positive result does not accept the nominal match: the original AABB,
    all other keys, normal/extent, topology and full foreground gates remain.
    """
    if type(complement) is not InverseChartComplement or complement._seal is not _SEAL:
        raise ValueError('expected prepared chart-complement certificate')
    points, lo, hi = _points_and_projection(points, complement.origin, complement.tangent)
    result = np.zeros(len(points))
    if not _environment():
        return result
    with np.errstate(all='ignore'):
        for lower, upper in complement.interiors:
            gap = np.maximum(0., np.minimum(np.nextafter(lo-lower, -np.inf),
                                            np.nextafter(upper-hi, -np.inf)))
            square = np.maximum(0., np.nextafter(gap*gap, -np.inf))
            candidate = np.maximum(0., np.nextafter(square/complement.norm_squared_upper, -np.inf))
            candidate[~np.isfinite(candidate)] = 0.
            result = np.maximum(result, candidate)
    result[~np.isfinite(points).all(axis=1)] = 0.
    return result
