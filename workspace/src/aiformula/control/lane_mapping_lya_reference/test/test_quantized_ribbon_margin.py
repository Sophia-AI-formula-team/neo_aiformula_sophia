"""Exact certificate tests independent of a particular NumPy/BLAS proposal.

The Newton solve is allowed to propose slightly different finite coefficients
on different supported platforms. Acceptance and the reported lower bound must
still enclose the exact binary64 polynomial, without tolerances or label slack.
"""
from fractions import Fraction

import numpy as np
import pytest

from lane_mapping_lya_reference import quantized_ribbon as ribbon


def _exact_value(coefficients, x):
    a, b, c = [Fraction.from_float(float(item)) for item in coefficients]
    x = Fraction.from_float(float(x))
    return a + b*x + c*x*x


def _assert_lower_certificate(result, inside, outside):
    assert result.reason == 'strict_label_proposal'
    margins = [_exact_value(result.coefficients, x)-Fraction.from_float(float(h))
               for x, h in inside]
    margins += [Fraction.from_float(float(h))-_exact_value(result.coefficients, x)
                for x, h in outside]
    assert all(margin > 0 for margin in margins)
    assert 0 < Fraction.from_float(result.minimum_margin) <= min(margins)


@pytest.mark.parametrize('sign', [-1., 1.])
@pytest.mark.parametrize('column', [0, 1, 2])
def test_small_solver_proposal_roundoff_cannot_overstate_label_margin(monkeypatch, sign, column):
    z = np.array([-1., -.75, -.25, 0., .125, .5, 1.])
    inside = np.column_stack((z, np.full(len(z), -.25)))
    outside = np.column_stack((z, np.full(len(z), .25)))
    original = np.linalg.solve
    calls = []

    def slightly_different_finite_proposal(matrix, rhs):
        step = original(matrix, rhs)
        # Fixed tiny dyadic perturbation to the proposal, not to the labels,
        # certificate, acceptance conditions or independently evaluated oracle.
        step[column] += sign * float.fromhex('0x1p-58')
        calls.append(step.copy())
        return step

    monkeypatch.setattr(ribbon.np.linalg, 'solve', slightly_different_finite_proposal)
    result = ribbon.fit_label_curve(inside, outside)
    assert calls
    _assert_lower_certificate(result, inside, outside)


def test_published_foxy_failure_is_strictly_greater_despite_same_float_display():
    # Exact fractions retained in all four R19 Foxy logs. This regression locks
    # the diagnosis, not an approximate printed coefficient or a BLAS version.
    reported = Fraction(9007199254740989, 36028797018963968)
    exact = Fraction(40564819207303327076670516933975,
                     162259276829213363391578010288128)
    assert float(reported) == float(exact)
    assert reported > exact
    assert Fraction.from_float(float(np.nextafter(float(reported), -np.inf))) < exact
