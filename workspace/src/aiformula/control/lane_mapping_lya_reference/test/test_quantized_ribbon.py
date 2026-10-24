"""Independent arithmetic and hard-label contracts, not geometry integration.

Fraction evaluates the exact polynomial of the supplied binary64 values.
Feasible examples require every original label, not proximity to a chosen
curve or a relaxed tolerance. No SciPy, image, map, engine or clock is used.
"""
from dataclasses import FrozenInstanceError
from fractions import Fraction
import math

import numpy as np
import pytest

from lane_mapping_lya_reference import quantized_ribbon as ribbon


def exact_polynomial(coefficients, value):
    a, b, c = (Fraction.from_float(float(item)) for item in coefficients)
    x = Fraction.from_float(float(value))
    return a + b*x + c*x*x


def assert_enclosure(coefficients, values):
    original_c = np.asarray(coefficients, dtype=np.float64).copy()
    original_x = np.asarray(values, dtype=np.float64).copy()
    lower, upper = ribbon.polynomial_bounds(coefficients, values)
    assert np.shape(lower) == original_x.shape == np.shape(upper)
    assert np.all(np.isfinite(lower)) and np.all(np.isfinite(upper))
    for x, lo, hi in zip(original_x.flat, np.asarray(lower).flat, np.asarray(upper).flat):
        exact = exact_polynomial(original_c, x)
        assert Fraction.from_float(float(lo)) <= exact
        assert exact <= Fraction.from_float(float(hi))
        assert lo <= hi
    assert np.asarray(coefficients, dtype=np.float64).tobytes() == original_c.tobytes()
    assert np.asarray(values, dtype=np.float64).tobytes() == original_x.tobytes()


@pytest.mark.parametrize("coefficients", [
    (0., 0., 0.), (-0., +0., -0.),
    (1., 2., 3.), (-1., 2., -3.), (1., -2., -3.), (-1., -2., 3.),
    (0.125, -0.5, 0.0625), (1.e7, -1.e7, 1.e7),
    (float.fromhex('0x0.0000000000001p-1022'), 0., 0.),
    (0., float.fromhex('0x0.0000000000001p-1022'),
     -float.fromhex('0x0.0000000000001p-1022')),
    (float.fromhex('0x1p-1022'), -float.fromhex('0x1p-1022'),
     float.fromhex('0x1p-1022')),
])
def test_horner_bounds_enclose_exact_binary64_degree_two_across_signs(coefficients):
    tiny = float.fromhex('0x0.0000000000001p-1022')
    values = np.array([-1.e7, -2., -1., -0.25, -tiny, -0., 0., tiny,
                       0.25, 1., 2., 1.e7])
    assert_enclosure(coefficients, values)


def test_horner_scalar_empty_and_strided_shapes():
    coefficients = np.array([0.125, -0.5, 0.0625])
    assert_enclosure(coefficients, np.asarray(-0.75))
    assert_enclosure(coefficients, np.empty((2, 0)))
    assert_enclosure(coefficients, np.linspace(-2., 2., 30).reshape(5, 6)[:, ::2])


def test_horner_fixed_seed_exact_random_polynomials():
    rng = np.random.RandomState(901732)
    for _ in range(96):
        # Independent dyadic and general binary64 values, including cancellation.
        coefficients = rng.uniform(-8., 8., 3)
        values = rng.uniform(-3., 3., 13)
        assert_enclosure(coefficients, values)
    for exponent in (-1020, -500, -20, 0, 20, 500):
        scale = math.ldexp(1., exponent)
        assert_enclosure((scale, -2.*scale, scale), np.array([-1., 0., 1., 2.]))


def labels(coefficients, margin, z=None):
    if z is None:
        z = np.array([-1., -0.75, -0.25, 0., 0.125, 0.5, 1.])
    values = np.array([float(exact_polynomial(coefficients, x)) for x in z])
    return np.column_stack((z, values-margin)), np.column_stack((z, values+margin))


def assert_hard_certificate(result, inside, outside):
    assert result.reason == 'strict_label_proposal'
    assert isinstance(result.coefficients, np.ndarray)
    assert result.coefficients.shape == (3,) and result.coefficients.dtype == np.float64
    assert np.all(np.isfinite(result.coefficients))
    assert 1 <= result.iterations <= 128
    exact_margins = []
    for x, height in inside:
        exact_margins.append(exact_polynomial(result.coefficients, x)
                             - Fraction.from_float(float(height)))
    for x, height in outside:
        exact_margins.append(Fraction.from_float(float(height))
                             - exact_polynomial(result.coefficients, x))
    assert all(margin > 0 for margin in exact_margins)
    assert 0. < result.minimum_margin
    assert Fraction.from_float(result.minimum_margin) <= min(exact_margins)


@pytest.mark.parametrize("coefficients,margin", [
    ((0., 0., 0.), 0.25), ((0.375, 0., 0.), 0.125), ((-0.375, 0., 0.), 0.125),
    ((0.125, 0.5, 0.), 0.0625), ((-0.125, -0.5, 0.), 0.0625),
    ((0.125, -0.25, 0.5), 0.0625), ((-0.125, 0.25, -0.5), 0.0625),
    ((0.125, 0.25, 0.0625), 2.**-12),
])
def test_feasible_constants_lines_and_curves_keep_every_hard_label(coefficients, margin):
    inside, outside = labels(coefficients, margin)
    before = inside.tobytes(), outside.tobytes()
    result = ribbon.fit_label_curve(inside, outside)
    assert_hard_certificate(result, inside, outside)
    assert before == (inside.tobytes(), outside.tobytes())


def test_label_order_and_repeated_labels_never_removed_to_pass():
    inside, outside = labels((0.125, -0.25, 0.5), 0.0625)
    inside = np.r_[inside[::-1], inside[2:4]]
    outside = np.r_[outside[[2, 0, 6, 1, 3, 5, 4]], outside[0:2]]
    assert_hard_certificate(ribbon.fit_label_curve(inside, outside), inside, outside)


@pytest.mark.parametrize("inside_height,outside_height", [(1., 0.), (0.25, 0.25)])
def test_same_x_conflicting_or_touching_labels_fail_closed(inside_height, outside_height):
    inside, outside = labels((0., 0., 0.), 1.)
    inside[3, 1], outside[3, 1] = inside_height, outside_height
    result = ribbon.fit_label_curve(inside, outside, max_iterations=64)
    assert result.coefficients is None
    assert result.reason != 'strict_label_proposal'
    assert 0 <= result.iterations <= 64
    assert result.minimum_margin == 0.


def test_zero_height_spread_is_not_a_strict_separator():
    inside = np.array([[-1., 0.], [0., 0.], [1., 0.]])
    result = ribbon.fit_label_curve(inside, inside.copy())
    assert (result.coefficients, result.iterations, result.reason) == (None, 0, 'zero_label_spread')


def test_iteration_limit_is_explicit_bounded_and_does_not_leak_last_iterate(monkeypatch):
    original = np.linalg.solve
    calls = []

    def counted(matrix, vector):
        calls.append((matrix.shape, vector.shape))
        return original(matrix, vector)

    monkeypatch.setattr(ribbon.np.linalg, 'solve', counted)
    inside, outside = labels((0.125, -0.25, 0.5), 0.0625)
    result = ribbon.fit_label_curve(inside, outside, max_iterations=1)
    assert calls == [((4, 4), (4,))]
    assert (result.coefficients, result.iterations, result.reason) == (None, 1, 'iteration_limit')
    assert result.minimum_margin == 0.


def test_singular_linear_algebra_fails_closed(monkeypatch):
    def singular(*args):
        raise np.linalg.LinAlgError('independent injected solver failure')

    monkeypatch.setattr(ribbon.np.linalg, 'solve', singular)
    inside, outside = labels((0., 0., 0.), 0.25)
    result = ribbon.fit_label_curve(inside, outside)
    assert (result.coefficients, result.iterations, result.reason) == (None, 1, 'singular_newton_system')


def test_coefficients_and_result_are_immutable_and_independent_of_input():
    inside, outside = labels((0.125, -0.25, 0.5), 0.0625)
    result = ribbon.fit_label_curve(inside, outside)
    assert_hard_certificate(result, inside, outside)
    before = result.coefficients.tobytes()
    assert not result.coefficients.flags.writeable
    with pytest.raises(ValueError):
        result.coefficients[0] = 10.
    with pytest.raises(ValueError):
        result.coefficients.setflags(write=True)
    with pytest.raises(FrozenInstanceError):
        result.iterations = 1
    inside[:] = 100.
    outside[:] = -100.
    assert result.coefficients.tobytes() == before


@pytest.mark.parametrize("which", ['inside', 'outside'])
@pytest.mark.parametrize("bad", [np.empty((0, 2)), np.empty((2, 0)),
                                 np.zeros(2), np.zeros((2, 3)), np.zeros((1, 2, 1))])
def test_empty_single_side_and_malformed_shapes_rejected(which, bad):
    inside, outside = labels((0., 0., 0.), 0.25)
    inputs = {'inside': inside, 'outside': outside}
    inputs[which] = bad
    with pytest.raises(ValueError, match='invalid bounded label-curve input'):
        ribbon.fit_label_curve(**inputs)


@pytest.mark.parametrize("which", ['inside', 'outside'])
@pytest.mark.parametrize("column", [0, 1])
@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf, np.nextafter(1.e7, np.inf),
                                 np.nextafter(-1.e7, -np.inf)])
def test_nonfinite_and_coordinate_capacity_rejected(which, column, bad):
    inside, outside = labels((0., 0., 0.), 0.25)
    inputs = {'inside': inside, 'outside': outside}
    inputs[which][2, column] = bad
    with pytest.raises(ValueError, match='invalid bounded label-curve input'):
        ribbon.fit_label_curve(**inputs)


@pytest.mark.parametrize("limit", [0, -1, 513, True, 1., np.int64(2), None, '128'])
def test_invalid_iteration_limit_rejected(limit):
    inside, outside = labels((0., 0., 0.), 0.25)
    with pytest.raises(ValueError, match='invalid bounded label-curve input'):
        ribbon.fit_label_curve(inside, outside, max_iterations=limit)


def test_more_than_200000_labels_rejected_before_newton(monkeypatch):
    def forbidden(*args):
        raise AssertionError('capacity rejection must precede Newton')

    monkeypatch.setattr(ribbon.np.linalg, 'solve', forbidden)
    inside = np.zeros((100001, 2))
    outside = np.ones((100000, 2))
    with pytest.raises(ValueError, match='invalid bounded label-curve input'):
        ribbon.fit_label_curve(inside, outside)
