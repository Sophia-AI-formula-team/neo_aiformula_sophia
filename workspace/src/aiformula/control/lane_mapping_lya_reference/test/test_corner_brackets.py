"""Independent original-label oracles; no extractor, QP, map or pose solve."""
from dataclasses import FrozenInstanceError
from fractions import Fraction

import numpy as np
import pytest

from lane_mapping_lya_reference.corner_brackets import (
    MAX_SIDE_FACES, CornerBracketCertificate, certify_corner_brackets,
)


def _f(value):
    return Fraction.from_float(float(value))


def _bands(low=(-.2, -.2), high=(.2, .2), k=6):
    t = np.linspace(-1., 1., k)
    whites = np.array([np.column_stack((np.full(k, low[0]), t)),
                       np.column_stack((-t, np.full(k, low[1])))])
    blacks = np.array([np.column_stack((np.full(k, high[0]), t)),
                       np.column_stack((-t, np.full(k, high[1])))])
    return np.zeros(2), np.eye(2), np.eye(2), whites, blacks


def _verify(certificate, inputs):
    assert isinstance(certificate, CornerBracketCertificate)
    vertex, axes, normals, whites, blacks = map(np.asarray, inputs)
    q, uv = certificate.point_xy, certificate.parameters_uv
    assert isinstance(q, tuple) and isinstance(uv, tuple)
    assert all(isinstance(value, Fraction) for value in q+uv)
    assert all(-Fraction(1, 2) <= value <= Fraction(1, 2) for value in uv)
    for coordinate in range(2):
        assert q[coordinate] == (_f(vertex[coordinate])
                                + uv[0]*_f(axes[0, coordinate])
                                + uv[1]*_f(axes[1, coordinate]))
    # Do not trust precomputed/rounded intervals: every original label is checked.
    for side in range(2):
        offset = sum((_f(normals[side, j])*q[j] for j in range(2)), Fraction())
        lower, upper = [], []
        for white, black in zip(whites[side], blacks[side]):
            w = sum((_f(normals[side, j])*_f(white[j]) for j in range(2)), Fraction())
            b = sum((_f(normals[side, j])*_f(black[j]) for j in range(2)), Fraction())
            assert w <= offset <= b
            lower.append(w)
            upper.append(b)
        assert certificate.offset_intervals[side] == (max(lower), min(upper))


def test_tls_mean_intersection_outside_does_not_discard_feasible_original_labels():
    inputs = list(_bands(low=(.1, .1), high=(1.8, 1.8)))
    # Symmetric tangent samples give zero cross-covariance. Four wide brackets
    # shift each midpoint/TLS-mean offset >.5, but ALL labels allow [.1,.4].
    inputs[4][0, 2:4, 0] = .4
    inputs[4][1, 2:4, 1] = .4
    midpoints = (inputs[3]+inputs[4])*.5
    tls_offsets = np.array([np.mean(midpoints[0, :, 0]), np.mean(midpoints[1, :, 1])])
    assert np.all(tls_offsets > .5)
    result = certify_corner_brackets(*inputs)
    _verify(result, inputs)
    assert all(_f(.1) <= value <= _f(.4) for value in result.point_xy)


def test_two_consistent_bands_can_have_no_common_original_footprint():
    inputs = _bands(low=(.6, -.1), high=(.8, .1))
    assert certify_corner_brackets(*inputs) is None


def test_one_contradictory_label_is_not_dropped_or_averaged():
    inputs = list(_bands())
    inputs[3][0, -1, 0] = .3
    assert certify_corner_brackets(*inputs) is None


@pytest.mark.parametrize('outside', [.5+1.e-8, np.nextafter(.5, np.inf)])
def test_no_physical_or_ulp_expansion_of_half_face_footprint(outside):
    inputs = _bands(low=(outside, -.1), high=(.8, .1))
    assert certify_corner_brackets(*inputs) is None


def test_exact_half_pixel_point_intersection_is_inclusive():
    inputs = _bands(low=(.5, -.5), high=(.5, -.5))
    result = certify_corner_brackets(*inputs)
    _verify(result, inputs)
    assert result.point_xy == (Fraction(1, 2), Fraction(-1, 2))
    assert result.parameters_uv == result.point_xy


def test_exact_zero_width_band_with_nonrepresentable_footprint_parameter_is_a_valid_witness():
    # n0=(1,0), offset=1 and first face delta=3: u=1/3 cannot be binary64.
    inputs = list(_bands(low=(1., 0.), high=(1., 0.)))
    inputs[1] = np.array([[3., 0.], [0., 1.]])
    result = certify_corner_brackets(*inputs)
    _verify(result, inputs)
    assert result.parameters_uv == (Fraction(1, 3), Fraction())
    # The proof is Fraction-valued; logging an approximate u would not preserve it.
    assert _f(float(result.parameters_uv[0])) != result.parameters_uv[0]


def test_nonrepresentable_world_point_remains_an_exact_existence_certificate():
    inputs = list(_bands(low=(0., 0.), high=(0., 0.)))
    inputs[1] = 2.*np.eye(2)
    inputs[2][1] = [.6, .8]
    inputs[3][1] = [1., 0.]
    inputs[4][1] = [1., 0.]
    result = certify_corner_brackets(*inputs)
    _verify(result, inputs)
    assert result.point_xy == (Fraction(), _f(.6)/_f(.8))
    assert _f(float(result.point_xy[1])) != result.point_xy[1]


def test_exact_original_dot_detects_infeasibility_hidden_by_rounded_offset():
    inputs = list(_bands(low=(0., -.5), high=(1., .5)))
    inputs[2][0] = [1., -(2.**-54)]
    inputs[3][0] = [.5, -1.]
    inputs[4][0] = [1., 0.]
    assert float(inputs[2][0] @ inputs[3][0, 0]) == .5
    # Exact lower offset=.5+2^-54, max over the box=.5+2^-55: no feasible q.
    assert certify_corner_brackets(*inputs) is None


@pytest.mark.parametrize('angle', [0., .13, .7, 1.2, np.pi/2., 2.4, -1.3, np.pi])
@pytest.mark.parametrize('translation', [[0., 0.], [7.125, -3.75], [-100., 80.]])
def test_rotated_translated_original_data_stays_strictly_certified(angle, translation):
    inputs = _bands(low=(-.15, -.12), high=(.19, .17))
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    t = np.array(translation)
    vertex, axes, normals, whites, blacks = inputs
    transformed = (vertex @ rotation.T+t, axes @ rotation.T, normals @ rotation.T,
                   whites @ rotation.T+t, blacks @ rotation.T+t)
    _verify(certify_corner_brackets(*transformed), transformed)


def test_axis_rows_are_original_nonorthogonal_face_deltas_not_normalized_directions():
    inputs = list(_bands(low=(.4, .1), high=(.4, .1)))
    inputs[1] = np.array([[2., 0.], [.5, 1.]])
    result = certify_corner_brackets(*inputs)
    _verify(result, inputs)
    assert result.parameters_uv[1] == _f(.1)
    assert result.parameters_uv[0] == (_f(.4)-Fraction(1, 2)*_f(.1))/2


@pytest.mark.parametrize('field,value', [
    (1, [[1., 0.], [2., 0.]]),
    (1, [[1., 0.], [1., 1.e-16]]),
    (1, [[0., 0.], [0., 1.]]),
    (2, [[1., 0.], [1., 0.]]),
    (2, [[1., 0.], [1., 1.e-16]]),
    (2, [[2., 0.], [0., 1.]]),
    (2, [[0., 0.], [0., 1.]]),
    (2, [[1.000001, 0.], [0., 1.]]),
])
def test_nonunit_normals_and_degenerate_axes_or_normals_reject(field, value):
    inputs = list(_bands())
    inputs[field] = np.asarray(value)
    assert certify_corner_brackets(*inputs) is None


def test_wrong_normal_sign_is_not_silently_flipped():
    inputs = list(_bands())
    inputs[2][0] *= -1.
    assert certify_corner_brackets(*inputs) is None


@pytest.mark.parametrize('field', range(5))
@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
def test_nonfinite_any_original_input_rejects(field, bad):
    inputs = list(_bands())
    inputs[field].flat[-1] = bad
    assert certify_corner_brackets(*inputs) is None


@pytest.mark.parametrize('k', [0, MAX_SIDE_FACES+1])
def test_empty_or_over_capacity_side_is_not_truncated(k):
    inputs = _bands(k=k)
    assert certify_corner_brackets(*inputs) is None


def test_capacity_boundary_keeps_last_label():
    inputs = list(_bands(k=MAX_SIDE_FACES))
    _verify(certify_corner_brackets(*inputs), inputs)
    inputs[3][1, -1, 1] = .3
    assert certify_corner_brackets(*inputs) is None


@pytest.mark.parametrize('field,value', [
    (0, [0.]), (1, np.eye(3)), (2, np.eye(2, dtype=bool)),
    (3, np.zeros((2, 6, 3))), (4, np.zeros((2, 5, 2))),
    (3, np.zeros((2, 6, 2), dtype=complex)),
])
def test_bad_shapes_and_nonreal_types_reject(field, value):
    inputs = list(_bands())
    inputs[field] = value
    assert certify_corner_brackets(*inputs) is None


def test_unrepresentable_integer_is_not_rounded_before_certificate():
    inputs = list(_bands())
    inputs[0] = np.array([2**53+1, 0], dtype=np.int64)
    assert certify_corner_brackets(*inputs) is None


def test_strided_inputs_never_mutate_and_certificate_has_no_array_aliases():
    base = _bands()
    inputs = []
    for original in base:
        storage = np.empty(original.shape[:-1]+(original.shape[-1]*2,))
        storage[..., ::2] = original
        inputs.append(storage[..., ::2])
    before = [value.tobytes() for value in inputs]
    result = certify_corner_brackets(*inputs)
    _verify(result, inputs)
    assert [value.tobytes() for value in inputs] == before
    saved = result.point_xy, result.parameters_uv, result.offset_intervals
    for value in inputs:
        value[...] = 99.
    assert (result.point_xy, result.parameters_uv, result.offset_intervals) == saved
    with pytest.raises(FrozenInstanceError):
        result.point_xy = ()
    with pytest.raises(TypeError):
        result.parameters_uv[0] = Fraction()
