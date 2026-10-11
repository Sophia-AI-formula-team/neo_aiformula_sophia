"""Packed finite-face bounds versus the frozen padded R16 implementation.

Literal body copied from git 77f2f17 paint_geometry.py, identical to pre-packed
6bb0730232290a81579a7d2d0cc53e0d8d02077ae8bf0d1b0de6d569ff95ef85.
No runtime Git, private fixtures, native extension or pose solver is required.
The DDS fixture is the literal runtime_smoke image/K/T, not a new calibration.
"""
from dataclasses import fields

import numpy as np
import pytest

from lane_mapping_lya_reference import paint_geometry as paint
from lane_mapping_lya_reference.mapping_core import GroundLookup


def _literal_bounds(origins, normals, scales, centers, segments,
                    arc_offsets, weights, radius):
    # Frozen original expressions, including padding and reduction order.
    count = len(origins)
    answer = np.full((count, 2), np.nan)
    if not count or not weights.shape[1]:
        return answer
    selected = np.asarray(weights, dtype=bool)
    nonempty = np.any(selected, axis=1)
    first = np.argmax(selected, axis=1)
    last = selected.shape[1]-1-np.argmax(selected[:, ::-1], axis=1)
    contiguous = np.sum(selected, axis=1) == last-first+1
    adjacent = selected[:, :-1] & selected[:, 1:]
    linked = np.all(segments[:, :-1, 1] == segments[:, 1:, 0], axis=2)
    connected = np.all(~adjacent | linked, axis=1)
    finite = (np.all(np.isfinite(centers), axis=2)
              & np.all(np.isfinite(segments), axis=(2, 3))
              & np.isfinite(arc_offsets) & (np.abs(arc_offsets) <= radius))
    delta = segments-centers[:, :, None, :]
    lengths = np.linalg.norm(delta, axis=3)
    finite &= (np.all(lengths > 0., axis=2)
               & (np.linalg.norm(segments[:, :, 1]-segments[:, :, 0], axis=2) > 0.))
    good = (nonempty & contiguous & connected & np.all(~selected | finite, axis=1)
            & np.all(np.isfinite(origins), axis=1)
            & np.all(np.isfinite(normals), axis=1)
            & np.isfinite(scales) & (scales > 0.))
    if not np.any(good):
        return answer
    remaining = np.stack((radius+arc_offsets, radius-arc_offsets), axis=2)
    with np.errstate(invalid='ignore', divide='ignore', over='ignore'):
        fraction = np.minimum(1., remaining/lengths)
        clipped = centers[:, :, None, :] + fraction[:, :, :, None]*delta
        clipped = np.where((fraction == 1.)[:, :, :, None], segments, clipped)
        tangents = np.column_stack((normals[:, 1], -normals[:, 0]))
        values = np.sum((clipped-origins[:, None, None, :])
                        * tangents[:, None, None, :], axis=3)/scales[:, None, None]
    finite_projection = np.all(np.isfinite(values), axis=2)
    good &= np.all(~selected | finite_projection, axis=1)
    lower = np.min(np.where(selected[:, :, None], values, np.inf), axis=(1, 2))
    upper = np.max(np.where(selected[:, :, None], values, -np.inf), axis=(1, 2))
    answer[good, 0], answer[good, 1] = lower[good], upper[good]
    return answer


def _inputs(seed, count=8, width=17):
    rng = np.random.RandomState(seed)
    vertices = np.cumsum(rng.uniform([.001, -.003], [.008, .003],
                                    (count, width+1, 2)), axis=1)
    vertices += rng.uniform(-3., 3., (count, 1, 2))
    segments = np.stack((vertices[:, :-1], vertices[:, 1:]), axis=2)
    fractions = rng.uniform(.1, .9, (count, width, 1))
    centers = segments[:, :, 0] + fractions*(segments[:, :, 1]-segments[:, :, 0])
    angles = rng.uniform(-np.pi, np.pi, count)
    normals = np.column_stack((np.sin(angles), np.cos(angles)))
    offsets = np.broadcast_to((np.arange(width)-width//2)*.005, (count, width)).copy()
    weights = np.zeros((count, width), dtype=bool)
    for row in range(count):
        if row % 4 == 1:
            weights[row] = True
        elif row % 4 == 2:
            start = max(0, width//2-3)
            weights[row, start:min(width, start+7)] = True
        elif row % 4 == 3:
            weights[row] = rng.rand(width) > .3
    return dict(origins=rng.uniform(-2., 2., (count, 2)), normals=normals,
                scales=rng.uniform(.1, 2., count), centers=centers,
                segments=segments, arc_offsets=offsets, weights=weights, radius=.2)


def _assert_equivalent(case):
    before = {name: (value.dtype.str, value.shape, value.tobytes())
              for name, value in case.items() if isinstance(value, np.ndarray)}
    # Output equivalence under ordinary non-trapping IEEE arithmetic. This does
    # not assert identical warning counts from intentionally unused padding.
    with np.errstate(all='ignore'):
        expected = _literal_bounds(**case)
        actual = paint._finite_face_fit_bounds(**case)
    assert actual.dtype == expected.dtype == np.float64
    assert actual.shape == expected.shape
    np.testing.assert_array_equal(np.isnan(actual), np.isnan(expected))
    np.testing.assert_array_equal(np.isfinite(actual), np.isfinite(expected))
    finite = np.isfinite(expected)
    assert actual[finite].tobytes() == expected[finite].tobytes()
    assert before == {name: (value.dtype.str, value.shape, value.tobytes())
                      for name, value in case.items() if isinstance(value, np.ndarray)}
    return actual


@pytest.mark.parametrize('seed', range(12))
def test_random_original_row_order_selection_and_projection_bytes(seed):
    width = [1, 7, 33, 512][seed % 4]
    result = _assert_equivalent(_inputs(seed, width=width))
    assert np.isfinite(result).any()


@pytest.mark.parametrize('kind', [
    'disconnected', 'noncontiguous', 'unselected_nan', 'unselected_inf',
    'selected_nan', 'selected_inf', 'zero_face', 'endpoint_center',
    'bad_origin', 'bad_normal', 'zero_scale', 'infinite_scale',
    'tiny_scale', 'extreme_coordinates', 'outside_radius', 'radius_boundary',
])
def test_invalid_padding_disconnection_and_extreme_rows_match_literal(kind):
    case = _inputs(311, count=3, width=9)
    case['weights'][:] = False
    case['weights'][:, 3:6] = True
    if kind == 'disconnected':
        case['segments'][0, 4, 0, 0] = np.nextafter(case['segments'][0, 4, 0, 0], np.inf)
    elif kind == 'noncontiguous':
        case['weights'][0, 4] = False
    elif kind.startswith('unselected_'):
        value = np.nan if kind.endswith('nan') else np.inf
        case['centers'][:, :3] = value
        case['segments'][:, :3] = value
        case['arc_offsets'][:, :3] = value
    elif kind.startswith('selected_'):
        case['segments'][0, 4, 1, 0] = np.nan if kind.endswith('nan') else np.inf
    elif kind == 'zero_face':
        case['segments'][0, 4, 1] = case['segments'][0, 4, 0]
    elif kind == 'endpoint_center':
        case['centers'][0, 4] = case['segments'][0, 4, 0]
    elif kind == 'bad_origin':
        case['origins'][0, 0] = np.nan
    elif kind == 'bad_normal':
        case['normals'][0, 1] = np.inf
    elif kind == 'zero_scale':
        case['scales'][0] = 0.
    elif kind == 'infinite_scale':
        case['scales'][0] = np.inf
    elif kind == 'tiny_scale':
        case['scales'][0] = np.nextafter(0., 1.)
    elif kind == 'extreme_coordinates':
        case['segments'][0] *= 1.e200
        case['centers'][0] *= 1.e200
        case['origins'][0] *= 1.e200
    elif kind == 'outside_radius':
        case['arc_offsets'][0, 4] = np.nextafter(case['radius'], np.inf)
    else:
        case['arc_offsets'][0, 3:6] = [-case['radius'], 0., case['radius']]
    _assert_equivalent(case)


@pytest.mark.parametrize('width', [1, 2, 3, 8, 17, 64, 512])
def test_signed_zero_bounds_preserve_original_padded_reduction(width):
    case = _inputs(512, count=4, width=width)
    vertices = np.zeros((4, width+1, 2))
    vertices[:, :, 1] = np.arange(width+1)*.01
    vertices[:, ::2, 0] = -0.
    case['segments'] = np.stack((vertices[:, :-1], vertices[:, 1:]), axis=2)
    case['centers'] = np.mean(case['segments'], axis=2)
    case['normals'][:] = [0., 1.]
    case['origins'][:] = [0., 0.]
    case['origins'][1::2, 0] = -0.
    case['arc_offsets'][:] = 0.
    case['weights'][:] = False
    case['weights'][:, width//2] = True
    case['weights'][1] = True
    case['radius'] = 100.
    assert np.all(_assert_equivalent(case) == 0.)


@pytest.mark.parametrize('layout', ['strided', 'fortran'])
def test_array_layout_does_not_change_selected_face_values(layout):
    case = _inputs(92, width=33)
    for name, value in list(case.items()):
        if not isinstance(value, np.ndarray):
            continue
        if layout == 'fortran':
            case[name] = np.asfortranarray(value)
        else:
            backing = np.empty(value.shape[:-1] + (value.shape[-1]*2,), dtype=value.dtype)
            backing[..., ::2] = value
            case[name] = backing[..., ::2]
    _assert_equivalent(case)


@pytest.mark.parametrize('count,width', [(0, 17), (3, 0)])
def test_empty_rows_or_empty_window_match_literal(count, width):
    _assert_equivalent(_inputs(1, count=count, width=width))


def test_literal_dds_full_extractor_all_observation_fields_byte_exact(monkeypatch):
    pixels = bytearray(640*360)
    for row in range(210, 360):
        for side in (-1., 1.):
            column = int(round(320+side*1.2*(row-180)))
            for offset in (-1, 0, 1):
                if 0 <= column+offset < 640:
                    pixels[row*640+column+offset] = 255
    mask = np.frombuffer(pixels, dtype=np.uint8).reshape(360, 640)
    K = np.array([[254., 0., 320.], [0., 254., 180.], [0., 0., 1.]])
    T = np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.],
                  [0., -1., 0., 1.], [0., 0., 0., 1.]])
    lookup = GroundLookup(640, 360, K, T, .5)
    before = mask.tobytes(), K.tobytes(), T.tobytes()
    actual = paint.extract_boundary_observation(mask, lookup, (0., 0., 1.),
                                                1_000_000_000, threshold=127)
    calls = []
    def counted_literal(*args, **kwargs):
        calls.append(1)
        return _literal_bounds(*args, **kwargs)
    with monkeypatch.context() as patch:
        patch.setattr(paint, '_finite_face_fit_bounds', counted_literal)
        expected = paint.extract_boundary_observation(mask, lookup, (0., 0., 1.),
                                                      1_000_000_000, threshold=127)
    assert calls and np.count_nonzero(actual.model_valid) > 0
    for field in fields(actual):
        left, right = getattr(actual, field.name), getattr(expected, field.name)
        if isinstance(left, np.ndarray):
            assert left.shape == right.shape and left.dtype == right.dtype, field.name
            assert left.tobytes() == right.tobytes(), field.name
            assert not left.flags.writeable and not right.flags.writeable
        else:
            assert left == right, field.name
    assert before == (mask.tobytes(), K.tobytes(), T.tobytes())
