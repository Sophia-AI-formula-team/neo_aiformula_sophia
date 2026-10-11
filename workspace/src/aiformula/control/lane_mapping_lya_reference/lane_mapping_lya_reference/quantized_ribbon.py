"""Bounded label-feasible quadratic proposal, independent of image topology.

A proposal is NOT a measured connection. Callers must retain actual faces,
their J/support gates and original adjacency. Black labels must not be removed
to make a ribbon fit. This module neither votes nor consumes history/pose.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LabelCurve:
    coefficients: object
    iterations: int
    reason: str
    minimum_margin: float = 0.


def polynomial_bounds(coefficients, x):
    """Outward Horner enclosure of the exact polynomial on binary64 inputs."""
    a, b, c = coefficients
    x = np.asarray(x, dtype=np.float64)
    with np.errstate(over='ignore', invalid='ignore', under='ignore'):
        first = c*x
        lo, hi = np.nextafter(first, -np.inf), np.nextafter(first, np.inf)
        lo = np.nextafter(lo+b, -np.inf)
        hi = np.nextafter(hi+b, np.inf)
        lower_product = np.where(x >= 0., lo, hi)*x
        upper_product = np.where(x >= 0., hi, lo)*x
        lo = np.nextafter(np.nextafter(lower_product, -np.inf)+a, -np.inf)
        hi = np.nextafter(np.nextafter(upper_product, np.inf)+a, np.inf)
    return lo, hi


def fit_label_curve(inside, outside, *, max_iterations=128):
    """Find h=p(z) with every inside h<p(z) and outside h>p(z).

    Inputs are all labels on one outward side in a fixed normalized chart.
    A four-variable log-barrier phase-I maximizes a common signed margin; a
    strictly feasible start always exists by making margin negative. Newton
    work is bounded, with no constraint removal or negative label tolerance.
    We return the first interval-certified positive-margin proposal, not a
    claim of an exact LP optimum. Infeasible/numerically difficult cases fail
    closed. Numerical backtracking is not a new geometric admission gate.
    """
    inside, outside = np.asarray(inside, dtype=np.float64), np.asarray(outside, dtype=np.float64)
    if (inside.ndim != 2 or outside.ndim != 2 or inside.shape[1:] != (2,)
            or outside.shape[1:] != (2,) or not len(inside) or not len(outside)
            or len(inside)+len(outside) > 200000
            or not np.all(np.isfinite(inside)) or not np.all(np.isfinite(outside))
            or np.max(np.abs(inside)) > 1.e7 or np.max(np.abs(outside)) > 1.e7
            or type(max_iterations) is not int or not 1 <= max_iterations <= 512):
        raise ValueError('invalid bounded label-curve input')
    labels = np.concatenate((inside, outside))
    z, h = labels.T
    scale = float(np.max(np.abs(h)))
    if scale == 0.:
        return LabelCurve(None, 0, 'zero_label_spread')
    sign = np.r_[-np.ones(len(inside)), np.ones(len(outside))]
    design = np.column_stack((np.ones(len(z)), z, z*z))
    matrix = np.column_stack((sign[:, None]*design, np.ones(len(z))))
    rhs = sign*h/scale
    value = np.array([0., 0., 0., float(np.min(rhs))-1.])
    weight = 1.
    objective = np.array([0., 0., 0., -1.])
    # Each iteration performs at most one Newton solve and 32 backtracks.
    for iteration in range(1, max_iterations+1):
        slack = rhs-matrix@value
        if not np.all(np.isfinite(slack)) or np.any(slack <= 0.):
            return LabelCurve(None, iteration, 'noninterior_iteration')
        coefficients = value[:3]*scale
        if value[3] > 0.:
            lo, hi = polynomial_bounds(coefficients, z)
            margin = np.r_[lo[:len(inside)]-h[:len(inside)],
                           h[len(inside):]-hi[len(inside):]]
            if np.all(np.isfinite(margin)) and np.all(margin > 0.):
                frozen = np.frombuffer(coefficients.tobytes(), dtype=np.float64)
                return LabelCurve(frozen, iteration, 'strict_label_proposal', float(np.min(margin)))
        inverse = 1./slack
        gradient = weight*objective+matrix.T@inverse
        scaled_rows = matrix*inverse[:, None]
        hessian = scaled_rows.T@scaled_rows
        if not np.all(np.isfinite(gradient)) or not np.all(np.isfinite(hessian)):
            return LabelCurve(None, iteration, 'nonfinite_newton_system')
        try:
            step = np.linalg.solve(hessian, -gradient)
        except np.linalg.LinAlgError:
            return LabelCurve(None, iteration, 'singular_newton_system')
        descent = float(gradient@step)
        if not np.all(np.isfinite(step)) or not np.isfinite(descent):
            return LabelCurve(None, iteration, 'nonfinite_newton_step')
        # Newton decrement is dimensionless; this changes only barrier search,
        # never the strict outward label certificate above.
        if -descent <= 1.e-8:
            weight *= 10.
            continue
        if descent >= 0.:
            return LabelCurve(None, iteration, 'non_descent_newton_step')
        old_cost = weight*float(objective@value)-float(np.sum(np.log(slack)))
        fraction, accepted = 1., False
        for _ in range(32):
            candidate = value+fraction*step
            candidate_slack = rhs-matrix@candidate
            if np.all(np.isfinite(candidate_slack)) and np.all(candidate_slack > 0.):
                cost = weight*float(objective@candidate)-float(np.sum(np.log(candidate_slack)))
                if np.isfinite(cost) and cost <= old_cost+.01*fraction*descent:
                    value, accepted = candidate, True
                    break
            fraction *= .5
        if not accepted:
            return LabelCurve(None, iteration, 'backtrack_limit')
    return LabelCurve(None, max_iterations, 'iteration_limit')
