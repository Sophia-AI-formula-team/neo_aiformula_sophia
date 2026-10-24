"""One prescribed box: outward-binary64 joint-existence check, not admission.

Supplied binary64 chart/coefficients are exact nominal model data. This does
not bound physical calibration/pose error or prove unique topology. Failure is
UNKNOWN, not global NO. No radius/domain growth, clipping, residual epsilon,
support replacement, extraction or state update is performed.

Krawczyk strict inclusion is checked inside an inward enclosure of the
ORIGINAL exact box center+[-radius,radius]^2. F and J use outward arithmetic;
the larger numerical enclosure used to evaluate J must also fit entirely in
original P and both finite charts. The floating inverse is only a candidate
preconditioner; its nonsingularity is independently enclosed before use.
Ordinary IEEE binary64 without FTZ/unsafe fast-math is required, as in
footprint_geometry. Iteration limits are not a WCET guarantee.

This module is not enabled in extraction or a vehicle runtime yet.
"""
import numpy as np

from .footprint_geometry import _point, _add, _sub, _mul
from .paired_curve_fit import PrimaryPatch, _array


def _divide_scalar(interval, positive):
    # The denominator is an exact, strictly positive binary64 scalar.
    return (np.nextafter(interval[0]/positive, -np.inf),
            np.nextafter(interval[1]/positive, np.inf))


def _divide(interval, denominator):
    if denominator[0] <= 0. <= denominator[1]:
        raise ValueError('uncertified denominator sign')
    candidates = np.asarray((interval[0]/denominator[0], interval[0]/denominator[1],
                             interval[1]/denominator[0], interval[1]/denominator[1]))
    return np.nextafter(np.min(candidates), -np.inf), np.nextafter(np.max(candidates), np.inf)


def _chart(model, x, y):
    dx, dy = _sub(x, _point(model.origin[0])), _sub(y, _point(model.origin[1]))
    nx, ny = model.normal
    xx = _divide_scalar(_sub(_mul(_point(ny), dx), _mul(_point(nx), dy)), model.scale)
    yy = _add(_mul(_point(nx), dx), _mul(_point(ny), dy))
    return xx, yy


def _value(model, coefficients, x, y):
    xx, yy = _chart(model, x, y)
    a, b, c = coefficients
    return _sub(_sub(_sub(yy, _point(a)), _mul(_point(b), xx)),
                _mul(_point(c), _mul(xx, xx)))


def _gradient(model, coefficients, x, y):
    xx, _ = _chart(model, x, y)
    _, b, c = coefficients
    slope = _divide_scalar(_add(_point(b), _mul(_mul(_point(2.), _point(c)), xx)), model.scale)
    nx, ny = model.normal
    return (_sub(_point(nx), _mul(slope, _point(ny))),
            _add(_point(ny), _mul(slope, _point(nx))))


def _matvec(matrix, vector):
    return [_add(_mul(matrix[i][0], vector[0]), _mul(matrix[i][1], vector[1])) for i in range(2)]


def _finite(intervals):
    return all(np.isfinite(lo) and np.isfinite(hi) and lo <= hi for lo, hi in intervals)


def certify_pair(left, right, coefficients, junction_xy, axes_2x2, center_xy, radius):
    """Pure nominal existence predicate. All old per-seed gates still required."""
    result = {'status': 'unknown', 'box_count': 1, 'map_writes': 0, 'gate_changes': 0,
              'domain_changes': 0, 'claim': 'nominal_joint_existence_not_uniqueness_or_physical_truth'}
    try:
        models = [PrimaryPatch.from_arrays(m.origin, m.normal, m.scale, m.bounds) for m in (left, right)]
        beta = _array(coefficients, (2, 3))
        junction, axes = _array(junction_xy, (2,)), _array(axes_2x2, (2, 2))
        center, radius_array = _array(center_xy, (2,)), _array(radius, ())
        radius = float(radius_array)
        if radius <= 0.:
            raise ValueError('positive prescribed box radius required')
        with np.errstate(over='ignore', under='ignore', divide='ignore', invalid='ignore'):
            delta = (-radius, radius)
            outer = [_add(_point(value), delta) for value in center]
            inner = [(np.nextafter(value-radius, np.inf), np.nextafter(value+radius, -np.inf))
                     for value in center]
            if not _finite(outer+inner) or any(lo >= hi for lo, hi in inner):
                raise ValueError('original exact box has no certified strict interior')
            offset = [_sub(outer[i], _point(junction[i])) for i in range(2)]
            ax, ay = _point(axes[0, 0]), _point(axes[0, 1])
            bx, by = _point(axes[1, 0]), _point(axes[1, 1])
            determinant = _sub(_mul(ax, by), _mul(ay, bx))
            if not _finite([determinant]):
                raise ValueError('nonfinite original-P determinant enclosure')
            u = _divide(_sub(_mul(offset[0], by), _mul(offset[1], bx)), determinant)
            v = _divide(_sub(_mul(ax, offset[1]), _mul(ay, offset[0])), determinant)
            if not _finite([u, v]) or any(lo < -.5 or hi > .5 for lo, hi in (u, v)):
                raise ValueError('whole numerical box not inside original P')
            for model in models:
                xx, _ = _chart(model, *outer)
                if (not _finite([xx]) or xx[0] < model.bounds[0] or xx[1] > model.bounds[1]):
                    raise ValueError('whole numerical box not inside original finite chart')
            # Approximate inverse is never used as an exact inverse certificate.
            approximate_j = []
            for model, row in zip(models, beta):
                nx, ny = model.normal
                dx, dy = center-np.asarray(model.origin)
                x = (ny*dx-nx*dy)/model.scale
                slope = (row[1]+2.*row[2]*x)/model.scale
                approximate_j.append((nx-slope*ny, ny+slope*nx))
            approximate_j = np.asarray(approximate_j)
            if not np.all(np.isfinite(approximate_j)):
                raise ValueError('nonfinite preconditioner proposal')
            preconditioner = np.linalg.inv(approximate_j)
            if not np.all(np.isfinite(preconditioner)):
                raise ValueError('nonfinite preconditioner')
            c = [[_point(value) for value in row] for row in preconditioner]
            det_c = _sub(_mul(c[0][0], c[1][1]), _mul(c[0][1], c[1][0]))
            if not _finite([det_c]) or det_c[0] <= 0. <= det_c[1]:
                raise ValueError('preconditioner nonsingularity not certified')
            f = [_value(model, row, *map(_point, center)) for model, row in zip(models, beta)]
            jacobian = [_gradient(model, row, *outer) for model, row in zip(models, beta)]
            if not _finite(f+[entry for row in jacobian for entry in row]):
                raise ValueError('nonfinite model enclosure')
            cf = _matvec(c, f)
            residual_matrix = []
            for i in range(2):
                row = []
                for k in range(2):
                    cj = _add(_mul(c[i][0], jacobian[0][k]), _mul(c[i][1], jacobian[1][k]))
                    row.append(_sub(_point(float(i == k)), cj))
                residual_matrix.append(row)
            correction = _matvec(residual_matrix, [delta, delta])
            image = [_add(_sub(_point(center[i]), cf[i]), correction[i]) for i in range(2)]
            if not _finite(image) or any(not lo < k_lo <= k_hi < hi
                                       for (lo, hi), (k_lo, k_hi) in zip(inner, image)):
                raise ValueError('strict original-box inclusion not certified')
        result.update(status='yes', reason=None,
            krawczyk_image=[[float(lo), float(hi)] for lo, hi in image],
            original_box_inward_bounds=[[float(lo), float(hi)] for lo, hi in inner],
            model_evaluation_outward_bounds=[[float(lo), float(hi)] for lo, hi in outer],
            preconditioner=preconditioner.tolist(), preconditioner_det_enclosure=list(map(float, det_c)))
    except (ValueError, TypeError, AttributeError, IndexError, ArithmeticError, np.linalg.LinAlgError) as error:
        result['reason'] = str(error)
    return result
