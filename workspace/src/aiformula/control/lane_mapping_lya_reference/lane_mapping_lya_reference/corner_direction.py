"""Exact local competing-side test, not a replacement boundary estimator.

A fixed PCA partition is not evidence that its endpoint belongs to that side.
Seven original W/B pairs (other side's six plus the endpoint) may admit a
different signed direction. Test that family without line fitting, label slack,
slope sampling, metric padding, or changing the existing angular tolerance.
Only nominal binary64 coordinates are certified, not calibration or pose error.
"""
from dataclasses import dataclass
from fractions import Fraction as F
from math import isqrt


@dataclass(frozen=True)
class DirectionDecision:
    status: str
    slope: object = None
    offset: object = None


def _dot(a, b):
    return sum((x*y for x, y in zip(a, b)), F(0))


def _value(poly, x):
    a, b, c = poly
    return (a*x+b)*x+c


def _range(poly, lower, upper):
    values = [_value(poly, lower), _value(poly, upper)]
    a, b, _ = poly
    if a and lower < -b/(2*a) < upper:
        values.append(_value(poly, -b/(2*a)))
    return min(values), max(values)


def _sqrt_bounds(value):
    """Exact dyadic enclosure; precision uncertainty never authorizes a model."""
    if value < 0:
        raise ValueError('negative discriminant')
    scale = 1 << 96
    root = isqrt((value.numerator*scale*scale)//value.denominator)
    lower = F(root, scale)
    return lower, lower if lower*lower == value else F(root+1, scale)


def _roots(poly):
    a, b, c = poly
    if not a:
        return [] if not b else [(-c/b, -c/b)]
    discriminant = b*b-4*a*c
    if discriminant < 0:
        return []
    low, high = _sqrt_bounds(discriminant)
    answer = []
    for sign in (-1, 1):
        endpoints = ((-b+sign*low)/(2*a), (-b+sign*high)/(2*a))
        answer.append((min(endpoints), max(endpoints)))
    return answer


def competing_line_direction(white, black, other_normal, selected_normal, cosine):
    """Return clear/ambiguous/indeterminate for original k faces + seed.

    Last W/B pair is the seed. Labels require white <= line <= black, with a
    STRICT seed crossing. Other-normal cone uses the unchanged signed angle;
    ambiguity requires a normal strictly outside the selected-normal cone.
    Rational interval elimination and polynomial sign cells exhaust the slope
    family. Unresolved algebraic-root bands return indeterminate, never clear.
    Callers must first verify original support count/radius/polarity/J gates.
    """
    try:
        white, black = [tuple(tuple(F(float(x)) for x in point) for point in values)
                        for values in (white, black)]
        n, chosen = [tuple(F(float(x)) for x in normal) for normal in (other_normal, selected_normal)]
        c = F(float(cosine))
    except (TypeError, ValueError, OverflowError):
        return DirectionDecision('indeterminate')
    if (not 3 <= len(white) <= 513 or len(black) != len(white) or any(len(p) != 2 for p in white+black)
            or len(n) != 2 or len(chosen) != 2 or not 0 < c < 1
            or not _dot(n, n) or not _dot(chosen, chosen)):
        raise ValueError('competing direction requires k original WB pairs plus seed and two normals')
    # Identical signed cones are correlated constraints. Treating their same
    # algebraic boundary as two independent uncertainty bands would needlessly
    # reject a family that cannot contain a competing direction at all.
    if n[0]*chosen[1] == n[1]*chosen[0] and _dot(n, chosen) > 0:
        return DirectionDecision('clear')
    tangent = (n[1], -n[0])
    origin = white[-1]
    def chart(point):
        delta = tuple(x-y for x, y in zip(point, origin))
        return _dot(delta, tangent), _dot(delta, n)
    w, b = tuple(map(chart, white)), tuple(map(chart, black))
    # N(s)=n-s*t has positive dot with n and squared length |n|^2(1+s^2).
    # Outer finite bounds only; the exact cone polynomial below stays mandatory.
    _, limit = _sqrt_bounds(1/(c*c)-1)
    lower, upper, open_lower, open_upper = -limit, limit, False, False
    for i, (xw, yw) in enumerate(w):
        for j, (xb, yb) in enumerate(b):
            coefficient, rhs = xb-xw, yb-yw
            strict = i == len(w)-1 or j == len(b)-1
            if not coefficient:
                if rhs < 0 or (strict and rhs == 0):
                    return DirectionDecision('clear')
                continue
            bound = rhs/coefficient
            if coefficient > 0:
                if bound < upper:
                    upper, open_upper = bound, strict
                elif bound == upper:
                    open_upper |= strict
            else:
                if bound > lower:
                    lower, open_lower = bound, strict
                elif bound == lower:
                    open_lower |= strict
    if lower > upper or (lower == upper and (open_lower or open_upper)):
        return DirectionDecision('clear')
    scale = c*c*_dot(n, n)*_dot(chosen, chosen)
    d0, d1 = _dot(n, chosen), -_dot(tangent, chosen)
    chosen_dot = (F(0), d1, d0)
    chosen_cone = (d1*d1-scale, 2*d0*d1, d0*d0-scale)
    other_cone = (-c*c, F(0), 1-c*c)

    def witness(slope):
        if ((slope == lower and open_lower) or (slope == upper and open_upper)
                or _value(other_cone, slope) < 0
                or (_value(chosen_dot, slope) >= 0 and _value(chosen_cone, slope) >= 0)):
            return None
        whites = [y-slope*x for x, y in w]
        blacks = [y-slope*x for x, y in b]
        offset = (max(whites)+min(blacks))/2
        if (all(y <= offset for y in whites) and all(y >= offset for y in blacks)
                and whites[-1] < offset < blacks[-1]):
            return DirectionDecision('ambiguous', slope, offset)
        return None

    # Roots partition the feasible slope interval into constant-sign cells.
    # Exact rational samples prove a whole cell's predicates; they are not a
    # selected grid, optimizer initialization or a heuristic midpoint-only test.
    bands = [(lower, lower), (upper, upper)]
    for poly in (other_cone, chosen_cone, chosen_dot):
        bands.extend((max(lower, a), min(upper, z)) for a, z in _roots(poly)
                     if z >= lower and a <= upper)
    merged = []
    for a, z in sorted(bands):
        if merged and a <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(z, merged[-1][1]))
        else:
            merged.append((a, z))
    for a, z in merged:
        for slope in (a, (a+z)/2, z):
            found = witness(slope)
            if found is not None:
                return found
    for (_, a), (z, _) in zip(merged, merged[1:]):
        found = witness((a+z)/2)
        if found is not None:
            return found
    for a, z in merged:
        if a == z:
            continue  # Exact root/end point already tested above.
        if _range(other_cone, a, z)[1] < 0:
            continue
        if _range(chosen_dot, a, z)[0] >= 0 and _range(chosen_cone, a, z)[0] >= 0:
            continue
        return DirectionDecision('indeterminate')
    return DirectionDecision('clear')
