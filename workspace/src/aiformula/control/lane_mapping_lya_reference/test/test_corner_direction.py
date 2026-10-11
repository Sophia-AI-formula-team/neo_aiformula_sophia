"""Independent exact competing-line witnesses; never an extractor or solver.

`clear` only excludes THIS linear family, not every quadratic boundary model.
All tests use explicit input labels. They do not alter support/radius/PCA/J
admission in the caller or claim calibration/physical-boundary accuracy.
"""
from dataclasses import FrozenInstanceError
from fractions import Fraction as F
import math

import numpy as np
import pytest

from lane_mapping_lya_reference.corner_direction import (
    competing_line_direction, _roots, _range, _sqrt_bounds, _value,
)


def labels(width=.125, count=6):
    x = np.arange(count, dtype=float)-count//2
    white = np.column_stack((x, np.full(count, -width)))
    black = np.column_stack((x, np.full(count, width)))
    return np.vstack((white, [0., -width])), np.vstack((black, [0., width]))


def singleton_labels(count=6):
    """Other labels pin y=0 exactly, but the seed strictly straddles it."""
    white, black = labels(count=count)
    white[:-1, 1] = black[:-1, 1] = 0.
    return white, black


def dot(first, second):
    return sum((a*b for a, b in zip(first, second)), F(0))


def fractions(values):
    return tuple(F(float(value)) for value in values)


def verify_witness(decision, white, black, normal, chosen, cosine):
    """Recheck original 2D label half-planes and cones, not helper internals."""
    assert decision.status == 'ambiguous'
    assert isinstance(decision.slope, F) and isinstance(decision.offset, F)
    n, selected = fractions(normal), fractions(chosen)
    c = F(float(cosine))
    tangent = (n[1], -n[0])
    direction = tuple(a-decision.slope*b for a, b in zip(n, tangent))
    origin = fractions(white[-1])
    whites = [dot(direction, tuple(a-b for a, b in zip(fractions(point), origin))) for point in white]
    blacks = [dot(direction, tuple(a-b for a, b in zip(fractions(point), origin))) for point in black]
    assert all(value <= decision.offset for value in whites)
    assert all(value >= decision.offset for value in blacks)
    assert whites[-1] < decision.offset < blacks[-1]
    own_dot, selected_dot = dot(direction, n), dot(direction, selected)
    assert own_dot > 0
    assert own_dot*own_dot >= c*c*dot(direction, direction)*dot(n, n)
    assert (selected_dot < 0 or
            selected_dot*selected_dot < c*c*dot(direction, direction)*dot(selected, selected))


def test_simple_witness_obeys_every_original_label_and_signed_cone():
    white, black = labels()
    result = competing_line_direction(white, black, [0., 1.], [1., 0.], .875)
    verify_witness(result, white, black, [0., 1.], [1., 0.], .875)
    with pytest.raises(FrozenInstanceError):
        result.status = 'clear'


@pytest.mark.parametrize('matrix', [
    [[1., 0.], [0., 1.]], [[0., -1.], [1., 0.]],
    [[-1., 0.], [0., -1.]], [[0., 1.], [-1., 0.]],
    [[-1., 0.], [0., 1.]], [[1., 0.], [0., -1.]],
    [[0., 1.], [1., 0.]], [[0., -1.], [-1., 0.]],
])
def test_dyadic_rigid_reflections_and_rotations_preserve_exact_witness(matrix):
    white, black = labels()
    rotation = np.asarray(matrix)
    translation = np.array([2.5, -3.25])
    white, black = white@rotation.T+translation, black@rotation.T+translation
    normal, chosen = rotation@np.array([0., 1.]), rotation@np.array([1., 0.])
    result = competing_line_direction(white, black, normal, chosen, .875)
    verify_witness(result, white, black, normal, chosen, .875)


@pytest.mark.parametrize('scale', [.125, 1., 8.])
def test_positive_uniform_scale_changes_no_feasibility_or_signed_direction(scale):
    white, black = labels()
    white, black = white*scale, black*scale
    result = competing_line_direction(white, black, [0., 1.], [1., 0.], .875)
    verify_witness(result, white, black, [0., 1.], [1., 0.], .875)


def test_reversed_other_face_order_does_not_move_or_relabel_the_seed():
    white, black = labels()
    order = [5, 4, 3, 2, 1, 0, 6]
    first = competing_line_direction(white, black, [0., 1.], [1., 0.], .875)
    second = competing_line_direction(white[order], black[order], [0., 1.], [1., 0.], .875)
    assert first == second
    verify_witness(second, white, black, [0., 1.], [1., 0.], .875)


def test_signed_opposite_selected_normal_is_not_abs_dot_compatible():
    white, black = singleton_labels()
    result = competing_line_direction(white, black, [0., 1.], [0., -1.], .875)
    assert result.slope == 0
    verify_witness(result, white, black, [0., 1.], [0., -1.], .875)


def test_reversing_white_black_without_normal_flip_cannot_create_a_witness():
    white, black = labels()
    result = competing_line_direction(black, white, [0., 1.], [1., 0.], .875)
    assert result.status == 'clear'
    assert result.slope is result.offset is None


def test_reversing_label_polarity_and_both_normals_is_consistent():
    white, black = labels()
    result = competing_line_direction(black, white, [0., -1.], [-1., 0.], .875)
    verify_witness(result, black, white, [0., -1.], [-1., 0.], .875)


def test_same_selected_and_other_cones_are_clear_even_with_wide_labels():
    white, black = labels(width=10.)
    assert competing_line_direction(white, black, [0., 1.], [0., 1.], .875).status == 'clear'


def test_singleton_family_is_not_lost_by_only_testing_open_slope_cells():
    white, black = singleton_labels()
    result = competing_line_direction(white, black, [0., 1.], [1., 0.], .875)
    assert result.slope == 0
    verify_witness(result, white, black, [0., 1.], [1., 0.], .875)


def test_narrow_normal_family_inside_selected_cone_is_clear():
    white, black = labels(width=2.**-35)
    assert competing_line_direction(white, black, [0., 1.], [.125, 1.], .875).status == 'clear'


def test_contradictory_other_labels_make_empty_family_without_dropping_conflict():
    white, black = labels()
    white[0], black[0] = [-3., 1.], [-3., -1.]
    assert competing_line_direction(white, black, [0., 1.], [1., 0.], .875).status == 'clear'


def test_seed_must_cross_strictly_not_merely_touch_the_only_feasible_line():
    white, black = singleton_labels()
    white[-1] = [0., 0.]
    assert competing_line_direction(white, black, [0., 1.], [1., 0.], .875).status == 'clear'
    white[-1], black[-1] = [0., -.125], [0., 0.]
    assert competing_line_direction(white, black, [0., 1.], [1., 0.], .875).status == 'clear'


def test_zero_width_seed_is_clear_even_when_all_closed_labels_are_feasible():
    white, black = labels()
    white[-1] = black[-1] = [0., 0.]
    assert competing_line_direction(white, black, [0., 1.], [1., 0.], .875).status == 'clear'


def test_adjacent_binary64_cosines_straddle_exact_selected_angle_without_epsilon():
    # Exact threshold is 4/5, NOT float(.8). The upper float lies above it;
    # its predecessor is below. A public exact nontrivial angular equality
    # is not fabricated by treating decimal .8 as a mathematical rational.
    white, black = singleton_labels()
    upper, lower = .8, float(np.nextafter(.8, -np.inf))
    assert F(lower) < F(4, 5) < F(upper)
    assert competing_line_direction(white, black, [0., 1.], [3., 4.], lower).status == 'clear'
    result = competing_line_direction(white, black, [0., 1.], [3., 4.], upper)
    verify_witness(result, white, black, [0., 1.], [3., 4.], upper)


def test_exact_rational_angle_boundary_is_in_closed_cone_not_outside():
    # The exact closed-cone polynomial used by the sign partition. Rational
    # equality is checked here directly; public inputs are binary64 samples.
    polynomial = (-F(9, 25), F(0), F(16, 25))
    for value in (-F(4, 3), F(4, 3)):
        assert _value(polynomial, value) == 0
    roots = sorted(_roots(polynomial))
    for (lower, upper), exact in zip(roots, (-F(4, 3), F(4, 3))):
        assert lower <= exact <= upper


def test_critical_algebraic_band_is_indeterminate_not_false_clear():
    # Competing interval at the cone boundary is narrower than the helper's
    # fixed 96-bit root enclosure. No label/normal tolerance is introduced.
    white, black = labels(width=10.)
    result = competing_line_direction(white, black, [0., 1.], [2.**-150, 1.],
                                      math.cos(math.pi/6.))
    assert result.status == 'indeterminate'
    assert result.slope is result.offset is None


@pytest.mark.parametrize('value', [F(0), F(1), F(2), F(1, 3), F(1, 2**250), F(2**100, 3)])
def test_root_enclosure_contains_exact_square_and_never_overclaims_precision(value):
    low, high = _sqrt_bounds(value)
    assert 0 <= low <= high
    assert low*low <= value <= high*high
    if low == high:
        assert low*low == value
    else:
        assert high-low == F(1, 2**96)


@pytest.mark.parametrize('polynomial,expected', [
    ((F(0), F(2), F(-3)), [(F(3, 2), F(3, 2))]),
    ((F(1), F(-2), F(1)), [(F(1), F(1)), (F(1), F(1))]),
    ((F(0), F(0), F(1)), []),
    ((F(1), F(0), F(1)), []),
])
def test_linear_constant_and_double_root_partition_cases(polynomial, expected):
    assert _roots(polynomial) == expected


def test_polynomial_range_checks_interior_extremum_not_only_endpoints():
    assert _range((F(1), F(0), F(-1)), F(-2), F(2)) == (-F(1), F(3))
    assert _range((-F(1), F(0), F(1)), F(-2), F(2)) == (-F(3), F(1))


@pytest.mark.parametrize('count', [2, 3, 9])
def test_original_caller_support_count_is_not_hardcoded_to_six(count):
    white, black = singleton_labels(count=count)
    result = competing_line_direction(white, black, [0., 1.], [1., 0.], .875)
    verify_witness(result, white, black, [0., 1.], [1., 0.], .875)


@pytest.mark.parametrize('count', [0, 1, 513])
def test_too_short_or_over_capacity_labels_are_rejected(count):
    white, black = labels(count=count)
    with pytest.raises(ValueError):
        competing_line_direction(white, black, [0., 1.], [1., 0.], .875)


@pytest.mark.parametrize('kind', ['unequal_lengths', 'coordinate_count', 'normal_size',
                                 'zero_other', 'zero_selected', 'cos_zero', 'cos_one', 'cos_negative'])
def test_malformed_finite_inputs_are_explicitly_rejected(kind):
    white, black = labels()
    normal, chosen, cosine = [0., 1.], [1., 0.], .875
    if kind == 'unequal_lengths':
        black = black[:-1]
    elif kind == 'coordinate_count':
        white = np.zeros((7, 3))
    elif kind == 'normal_size':
        normal = [0., 1., 0.]
    elif kind == 'zero_other':
        normal = [0., 0.]
    elif kind == 'zero_selected':
        chosen = [0., 0.]
    else:
        cosine = {'cos_zero': 0., 'cos_one': 1., 'cos_negative': -1.}[kind]
    with pytest.raises(ValueError):
        competing_line_direction(white, black, normal, chosen, cosine)


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -float('inf'), 'not_numeric', 10**1000])
def test_nonfinite_or_unconvertible_input_is_indeterminate_not_clear(invalid):
    white, black = labels()
    white = white.tolist()
    white[0][0] = invalid
    result = competing_line_direction(white, black, [0., 1.], [1., 0.], .875)
    assert result.status == 'indeterminate'
    assert result.slope is result.offset is None

