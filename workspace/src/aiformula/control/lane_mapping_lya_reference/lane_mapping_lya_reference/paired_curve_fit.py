"""Bounded two-primary-curve proposals; NOT extraction or map admission.

An original computational cut does not imply independently fitted quadratics
meet there. Hypotheses stay in the original half-face P and both finite charts.
The two coefficients free after fixing a latent joint are fitted against ALL
original centre and white/black occurrences. No support/radius/extent is changed.

Float64 active-set algebra is only a proposal generator, not an optimality or
feasibility proof. Returned coefficient packets are independently checked using
outward binary64 intervals, with exact nominal substitution for uncertain signs.
Rounding may separate the shared zero: a caller MUST separately certify the
actual rounded curves' joint and retain its root/angle/J/ownership gates.

This module is deliberately NOT enabled in extract_boundary_observation yet.
Iteration bounds are not a wall-time guarantee; no driving permission is implied.
"""
from dataclasses import dataclass
from fractions import Fraction as F
from itertools import combinations, product
import math

import numpy as np

from .footprint_geometry import _point, _add, _sub, _mul

MAX_FACES = 512
MAX_PIVOTS = 24
MAX_BITS = 16384


def _array(value, shape):
    raw = np.asarray(value)
    if (raw.shape != shape or raw.dtype.kind not in 'fiu'
            or raw.dtype.kind == 'f' and raw.dtype.itemsize > 8):
        raise ValueError('original binary64-compatible geometry shape required')
    result = np.array(raw, dtype=np.float64, copy=True)
    if not np.all(np.isfinite(result)):
        raise ValueError('nonfinite original geometry')
    if raw.dtype.kind in 'iu' and any(int(a) != int(b) for a, b in zip(raw.flat, result.flat)):
        raise ValueError('integer geometry would be rounded')
    return result


def _number(value):
    result = value if isinstance(value, F) else F.from_float(float(value))
    if result.numerator.bit_length() > MAX_BITS or result.denominator.bit_length() > MAX_BITS:
        raise ValueError('exact initializer arithmetic checkpoint')
    return result


def _pair(value):
    return tuple(_number(item) for item in value)


@dataclass(frozen=True)
class PrimaryPatch:
    """Copied numerical chart, not authentication of a contour owner.

    Caller must bind these fields and every support occurrence to the SAME
    actual primary producer. Equal counts cannot turn ribbon labels into this
    contract. from_arrays does not normalize normals or repair bad shapes.
    """
    origin: tuple
    normal: tuple
    scale: float
    bounds: tuple

    @classmethod
    def from_arrays(cls, origin, normal, scale, bounds):
        origin, normal, bounds = [_array(value, (2,)) for value in (origin, normal, bounds)]
        scalar = _array(scale, ())
        scale = float(scalar)
        if scale <= 0. or bounds[0] > bounds[1] or not np.any(normal != 0.):
            raise ValueError('invalid unchanged finite chart')
        return cls(tuple(origin), tuple(normal), scale, tuple(bounds))


def _chart_exact(model, point):
    ox, oy = _pair(model.origin)
    nx, ny = _pair(model.normal)
    dx, dy = point[0]-ox, point[1]-oy
    return (_number((ny*dx-nx*dy)/_number(model.scale)), _number(nx*dx+ny*dy))


def _inside(point, planes):
    return all(_number(a*point[0]+b*point[1]+c) >= 0 for a, b, c in planes)


def _locations(models, junction, axes):
    """Same five slots, exact intersection of original P and four chart slabs."""
    j, (a, b) = _pair(junction), tuple(_pair(row) for row in axes)
    determinant = _number(a[0]*b[1]-a[1]*b[0])
    if not determinant:
        raise ValueError('degenerate original cut axes')
    corners = [tuple(j[i]+u*a[i]/2+v*b[i]/2 for i in range(2))
               for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    sign = 1 if determinant > 0 else -1
    planes = []
    for p, q in zip(corners, corners[1:]+corners[:1]):
        dx, dy = q[0]-p[0], q[1]-p[1]
        planes.append((-sign*dy, sign*dx, sign*(dy*p[0]-dx*p[1])))
    for model in models:
        nx, ny = _pair(model.normal)
        ox, oy = _pair(model.origin)
        aa, bb = ny/_number(model.scale), -nx/_number(model.scale)
        offset = -aa*ox-bb*oy
        lo, hi = _pair(model.bounds)
        planes.extend(((aa, bb, offset-lo), (-aa, -bb, hi-offset)))
    planes = [tuple(_number(value) for value in row) for row in planes]
    vertices = set()
    for p, q in combinations(planes, 2):
        det = _number(p[0]*q[1]-p[1]*q[0])
        if det:
            point = (_number((p[1]*q[2]-p[2]*q[1])/det),
                     _number((p[2]*q[0]-p[0]*q[2])/det))
            if _inside(point, planes):
                vertices.add(point)
    if not vertices:
        return []
    vertices = sorted(vertices)
    def average(points):
        return tuple(_number(sum((p[i] for p in points), F(0))/len(points)) for i in range(2))
    center = average(vertices)
    coordinates = [((_number((p[0]-j[0])*b[1]-(p[1]-j[1])*b[0]))/determinant,
                    (_number(a[0]*(p[1]-j[1])-a[1]*(p[0]-j[0])))/determinant) for p in vertices]
    result = [center]
    for axis, largest in ((0, False), (1, False), (0, True), (1, True)):
        extremum = (max if largest else min)(p[axis] for p in coordinates)
        face = average([p for p, uv in zip(vertices, coordinates) if uv[axis] == extremum])
        result.append(average((center, face)))
    if not all(_inside(point, planes) for point in result):
        raise ValueError('full original-domain verification failed')
    return result


def _chart_float(model, points):
    delta = points-np.asarray(model.origin)
    nx, ny = model.normal
    return ((ny*delta[:, 0]-nx*delta[:, 1])/model.scale,
            nx*delta[:, 0]+ny*delta[:, 1])


def _chart_interval(model, points):
    dx = _sub(_point(points[:, 0]), _point(model.origin[0]))
    dy = _sub(_point(points[:, 1]), _point(model.origin[1]))
    nx, ny = model.normal
    numerator = _sub(_mul(_point(ny), dx), _mul(_point(nx), dy))
    x = (np.nextafter(numerator[0]/model.scale, -np.inf),
         np.nextafter(numerator[1]/model.scale, np.inf))
    y = _add(_mul(_point(nx), dx), _mul(_point(ny), dy))
    return x, y


def _labels(model, coefficients, whites, blacks, diagnostic=None):
    """All occurrences, including off-extent labels; nominal exact chart signs."""
    coefficients = np.asarray(coefficients, dtype=np.float64)
    if coefficients.shape != (3,) or not np.all(np.isfinite(coefficients)):
        return False
    with np.errstate(over='ignore', under='ignore', divide='ignore', invalid='ignore'):
        for role, points in ((-1, whites), (1, blacks)):
            x, y = _chart_interval(model, points)
            residual = _sub(_sub(_sub(y, _point(coefficients[0])),
                                _mul(_point(coefficients[1]), x)),
                            _mul(_point(coefficients[2]), _mul(x, x)))
            lo, hi = residual if role == 1 else (-residual[1], -residual[0])
            if np.any(hi < 0.):
                return False
            uncertain = ~np.isfinite(lo) | ~np.isfinite(hi) | (lo < 0.)
            for index in np.flatnonzero(uncertain):
                if diagnostic is not None:
                    diagnostic['exact_label_substitutions'] += 1
                xx, yy = _chart_exact(model, _pair(points[index]))
                aa, bb, cc = _pair3(coefficients)
                value = _number(role*(yy-aa-bb*xx-cc*xx*xx))
                if value < 0:
                    return False
    return True


def _pair3(values):
    return tuple(_number(value) for value in values)


def _definite_linear_violations(rows, beta, limits):
    """Generator-only roundoff handling, NEVER a label/admission predicate.

    A rounded dot product can report a positive residual at an exact active
    equality. Enclose the supplied float linear system and pivot only on a
    definitely positive residual. Uncertain rows are NOT certified feasible:
    every resulting beta64 still goes through the independent full original
    WB predicate and the caller's rounded-joint/old geometry gates. There is
    no residual epsilon, changed observation or relaxed final acceptance.
    """
    with np.errstate(over='ignore', under='ignore', divide='ignore', invalid='ignore'):
        residual = _sub(_add(_mul(_point(rows[:, 0]), _point(beta[0])),
                             _mul(_point(rows[:, 1]), _point(beta[1]))), _point(limits))
    if not (np.all(np.isfinite(residual)) and np.all(residual[0] <= residual[1])):
        raise ValueError('nonfinite proposal linear-residual enclosure')
    return np.flatnonzero(residual[0] > 0.)


def _restricted_proposal(gram, rhs, rows, limits, max_pivots, diagnostic=None):
    """Bounded float generator: no rounded linear-system feasibility claim."""
    def count(key):
        if diagnostic is not None:
            diagnostic[key] += 1
    def fail(reason):
        if diagnostic is not None:
            diagnostic['reason'] = reason
        return None
    eigenvalues = np.linalg.eigvalsh(gram)
    if (not np.all(np.isfinite(eigenvalues)) or eigenvalues[-1] <= 0.
            or eigenvalues[0] <= 64.*np.finfo(np.float64).eps*eigenvalues[-1]):
        return fail('reduced_rank_unknown')
    count('systems')
    current = np.linalg.solve(gram, rhs)
    active = ()
    for pivot in range(max_pivots+1):
        if not np.all(np.isfinite(current)):
            return fail('nonfinite_reduced_solution')
        count('full_scans')
        violated = _definite_linear_violations(rows, current, limits)
        if not len(violated):
            return current
        if pivot == max_pivots:
            return fail('pivot_budget_unknown')
        count('pivots')
        pool = tuple(sorted(set(active+(int(violated[0]),))))
        found = None
        for size in range(min(2, len(pool))+1):
            for basis in combinations(pool, size):
                matrix = np.zeros((2+size, 2+size))
                matrix[:2, :2] = gram
                if size:
                    matrix[:2, 2:] = rows[list(basis)].T
                    matrix[2:, :2] = rows[list(basis)]
                try:
                    count('systems')
                    answer = np.linalg.solve(matrix, np.r_[rhs, limits[list(basis)]])
                except np.linalg.LinAlgError:
                    continue
                beta, multipliers = answer[:2], answer[2:]
                if (not np.all(np.isfinite(answer)) or np.any(multipliers < 0.)
                        or len(_definite_linear_violations(rows[list(pool)], beta, limits[list(pool)]))):
                    continue
                found = beta, tuple(k for k, m in zip(basis, multipliers) if m > 0.)
                break
            if found is not None:
                break
        if found is None:
            return fail('restricted_float_pool_unknown')
        current, active = found


def _fit_at_q(model, centers, whites, blacks, q, max_pivots, diagnostic=None):
    if diagnostic is not None:
        diagnostic['reason'] = 'fit_not_completed'
    xq_exact, yq_exact = _chart_exact(model, q)
    if not _number(model.bounds[0]) <= xq_exact <= _number(model.bounds[1]):
        if diagnostic is not None:
            diagnostic['reason'] = 'outside_original_finite_chart'
        return None
    xq, yq = float(xq_exact), float(yq_exact)
    x, y = _chart_float(model, centers)
    design = np.column_stack((x-xq, x*x-xq*xq))
    target = y-yq
    gram, rhs = design.T @ design, design.T @ target
    labels = np.stack((whites, blacks), axis=1).reshape(-1, 2)
    xx, yy = _chart_float(model, labels)
    signs = np.tile((-1., 1.), len(whites))
    rows = signs[:, None]*np.column_stack((xx-xq, xx*xx-xq*xq))
    limits = signs*(yy-yq)
    if not all(np.all(np.isfinite(v)) for v in (gram, rhs, rows, limits)):
        if diagnostic is not None:
            diagnostic['reason'] = 'nonfinite_objective_or_labels'
        return None
    free = _restricted_proposal(gram, rhs, rows, limits, max_pivots, diagnostic)
    if free is None:
        return None
    b, c = free
    nearest = (yq-b*xq-c*xq*xq, b, c)
    # Foxy runs Python 3.8, before math.nextafter was available. NumPy is an
    # existing package dependency; use its IEEE binary64 adjacent values.
    adjacent = [(v, float(np.nextafter(v, -np.inf)), float(np.nextafter(v, np.inf)))
                for v in nearest]
    best, best_cost = None, None
    for coefficients in product(*adjacent):
        if diagnostic is not None:
            diagnostic['rounding_packets'] += 1
        if not _labels(model, coefficients, whites, blacks, diagnostic):
            continue
        a, b, c = coefficients
        cost = float(np.sum((y-a-b*x-c*x*x)**2))
        if math.isfinite(cost) and (best is None or cost < best_cost):
            best, best_cost = tuple(float(v) for v in coefficients), cost
    if diagnostic is not None:
        diagnostic['reason'] = None if best is not None else 'no_certified_27_packet'
    return best


def propose_pair(left, right, centers, whites, blacks, junction_xy, axes_2x2,
                 *, max_pivots=MAX_PIVOTS):
    """Pure proposal API. Caller supplies two authentic complete PRIMARY groups.

    No coefficients are adopted here. Every returned pair still needs an
    independent actual-rounded joint proof and ALL original downstream gates.
    Failed search is UNKNOWN, never physical infeasibility or a point veto.
    """
    result = {'status': 'unknown', 'candidates': [], 'map_writes': 0,
              'dropped_occurrences': 0, 'gate_changes': 0, 'domain_changes': 0,
              'joint_certified': False, 'search_failure_is_not_infeasibility': True,
              'objective_scope': 'float_nominal_saved_world_center_LS_proposal_not_original_FP_Gram_rhs',
              'label_contract': 'nominal_all_WB_polynomial_extension',
              'initializer_policy': 'five_original_P_intersect_original_finite_chart_locations_v2'}
    try:
        if type(max_pivots) is not int or not 0 <= max_pivots <= MAX_PIVOTS:
            raise ValueError('unchanged 24-pivot bound required')
        junction, axes = _array(junction_xy, (2,)), _array(axes_2x2, (2, 2))
        models = [PrimaryPatch.from_arrays(m.origin, m.normal, m.scale, m.bounds) for m in (left, right)]
        if not len(centers) == len(whites) == len(blacks) == 2:
            raise ValueError('two paired-primary groups required')
        groups = []
        for cs, ws, bs in zip(centers, whites, blacks):
            if not 6 <= len(cs) == len(ws) == len(bs) <= MAX_FACES:
                raise ValueError('full same-support six..512 occurrences required')
            groups.append(tuple(_array(v, (len(cs), 2)) for v in (cs, ws, bs)))
        locations = _locations(models, junction, axes)
        for q in locations:
            diagnostics = [dict(systems=0, pivots=0, full_scans=0, rounding_packets=0,
                                exact_label_substitutions=0, reason='not_started') for _ in models]
            coefficients = [None, None]
            item = {'latent_q_exact': tuple(str(v) for v in q),
                'latent_q_rounded': tuple(float(v) for v in q),
                'side_diagnostics': diagnostics,
                'coefficients': coefficients, 'both_beta64_labels_certified': False,
                'shared_q_after_rounding': 'unverified_requires_joint_certificate'}
            # Retain work counters even if a later operation raises. A partial
            # packet remains UNKNOWN and must never be used as a complete pair.
            result['candidates'].append(item)
            for side, (m, g, d) in enumerate(zip(models, groups, diagnostics)):
                coefficients[side] = _fit_at_q(m, *g, q, max_pivots, d)
            item['both_beta64_labels_certified'] = all(v is not None for v in coefficients)
        result['status'] = 'completed_proposal_diagnostic' if locations else 'unknown'
        if not locations:
            result['reason'] = 'no_original_domain_initializer_not_global_no'
    except (ValueError, TypeError, AttributeError, IndexError, ArithmeticError, np.linalg.LinAlgError) as error:
        result['reason'] = str(error)
    return result
