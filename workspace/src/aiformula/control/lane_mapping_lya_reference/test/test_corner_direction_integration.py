"""Endpoint ambiguity guard wiring, not a proof of physical corner identity.

Small analytic fixtures exercise admission prerequisites and extraction ordering.
The linear-family mathematics has its own independent Fraction tests. No private
diagnostic data, simulator calibration, map, truth pose, or solver is imported.
Controlled statuses below test wiring explicitly, not a fabricated certificate.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from lane_mapping_lya_reference import corner_direction as direction
from lane_mapping_lya_reference import paint_geometry as paint
from lane_mapping_lya_reference import ribbon_observation as ribbon


def _guard_fixture(k=6):
    """Exact dyadic contiguous raw faces, with two eligible cut endpoints."""
    n = 2*k
    endpoints = np.column_stack((np.arange(n+1)*.03125, np.zeros(n+1)))
    segments = np.stack((endpoints[:-1], endpoints[1:]), axis=1)
    points = segments.mean(axis=1)
    normal = np.tile([0., 1.], (n, 1))
    return dict(cuts=[(np.arange(n).reshape(2, k), np.array([[0., 1.], [0., 1.]]))],
        points=points, segments=segments,
        white_xy=points-np.array([0., .015625]),
        black_xy=points+np.array([0., .015625]),
        pixel_outward=np.tile([0, 1], (n, 1)),
        stable=np.ones(n, bool), radii=np.full(n, .45),
        model_normals=normal, model_valid=np.ones(n, bool),
        cfg=dict(paint.DEFAULT_CONFIG, normal_min_points=k))


def _fingerprint(fixture):
    arrays = [v for v in fixture.values() if isinstance(v, np.ndarray)]
    arrays += [v for cut in fixture['cuts'] for v in cut]
    return [(v.dtype.str, v.shape, v.tobytes()) for v in arrays]


def _record_status(monkeypatch, status='ambiguous', error=None):
    calls = []
    def fake(white, black, other, selected, cosine):
        calls.append(tuple(np.array(v, copy=True) if isinstance(v, np.ndarray) else v
                           for v in (white, black, other, selected, cosine)))
        if error is not None:
            raise error('test-only malformed family')
        return SimpleNamespace(status=status)
    monkeypatch.setattr(direction, 'competing_line_direction', fake)
    return calls


@pytest.mark.parametrize('status,bit', [
    ('clear', 0), ('ambiguous', paint.REASON_COMPETING_DIRECTION),
    ('indeterminate', paint.REASON_DIRECTION_UNCERTAIN)])
@pytest.mark.parametrize('k', [2, 6, 9])
def test_only_cut_endpoints_get_family_status_and_original_seed_is_last(monkeypatch, status, bit, k):
    fixture = _guard_fixture(k)
    before = _fingerprint(fixture)
    calls = _record_status(monkeypatch, status)
    actual = paint._competing_endpoint_directions(**fixture)
    expected = np.zeros(2*k, np.int64)
    expected[[k-1, k]] = bit
    np.testing.assert_array_equal(actual, expected)
    assert len(calls) == 2
    for call, other, seed in zip(calls, [np.arange(k, 2*k), np.arange(k)], [k-1, k]):
        labels = np.r_[other, seed]
        assert call[0].tobytes() == fixture['white_xy'][labels].tobytes()
        assert call[1].tobytes() == fixture['black_xy'][labels].tobytes()
        assert call[2].tobytes() == np.array([0., 1.]).tobytes()
        assert call[3].tobytes() == fixture['model_normals'][seed].tobytes()
        assert call[4] == math.cos(paint.CONSENSUS_DEFAULT_CONFIG['normal_max_angle_rad'])
    assert _fingerprint(fixture) == before


@pytest.mark.parametrize('blocker', [
    'no_selected_cut', 'wrong_support_count', 'unstable_seed', 'unstable_other',
    'opposite_pixel_polarity', 'raw_segment_gap', 'over_arc_radius',
    'nonfinite_arc', 'already_invalid_seed'])
def test_missing_original_prerequisite_never_calls_competing_family(monkeypatch, blocker):
    fixture = _guard_fixture()
    # Isolate the incoming endpoint. The other endpoint already has no model.
    fixture['model_valid'][6] = False
    if blocker == 'no_selected_cut':
        fixture['cuts'] = []
    elif blocker == 'wrong_support_count':
        fixture['cfg']['normal_min_points'] = 7
    elif blocker == 'unstable_seed':
        fixture['stable'][5] = False
    elif blocker == 'unstable_other':
        fixture['stable'][8] = False
    elif blocker == 'opposite_pixel_polarity':
        fixture['pixel_outward'][8] = [0, -1]
    elif blocker == 'raw_segment_gap':
        fixture['segments'][8, 0, 0] += .00390625
    elif blocker == 'over_arc_radius':
        fixture['radii'][5] = np.nextafter(.1875, 0.)
    elif blocker == 'nonfinite_arc':
        fixture['points'][8, 0] = np.nan
    elif blocker == 'already_invalid_seed':
        fixture['model_valid'][5] = False
    before = _fingerprint(fixture)
    calls = _record_status(monkeypatch)
    actual = paint._competing_endpoint_directions(**fixture)
    assert not calls
    assert not actual.any()
    assert _fingerprint(fixture) == before


def test_original_arc_radius_equality_is_not_rejected(monkeypatch):
    fixture = _guard_fixture()
    fixture['model_valid'][6] = False
    fixture['radii'][5] = .1875
    calls = _record_status(monkeypatch)
    actual = paint._competing_endpoint_directions(**fixture)
    assert len(calls) == 1
    assert actual[5] == paint.REASON_COMPETING_DIRECTION


def test_arc_distance_is_not_replaced_with_shorter_endpoint_distance(monkeypatch):
    fixture = _guard_fixture()
    fixture['model_valid'][6] = False
    # Only bend the metric path; it remains exactly connected. The original
    # accumulated face-centre arc exceeds .45 although its end distance does not.
    vertices = np.array([[0., 0.], [.03125, .125], [.0625, 0.], [.09375, .125],
                         [.125, 0.], [.15625, .125], [.1875, 0.], [.21875, .125],
                         [.25, 0.], [.28125, .125], [.3125, 0.], [.34375, .125], [.375, 0.]])
    fixture['segments'] = np.stack((vertices[:-1], vertices[1:]), axis=1)
    fixture['points'] = fixture['segments'].mean(axis=1)
    assert np.linalg.norm(fixture['points'][11]-fixture['points'][5]) < .45
    calls = _record_status(monkeypatch)
    actual = paint._competing_endpoint_directions(**fixture)
    assert not calls and not actual.any()


def test_value_error_is_explicit_uncertainty_not_clear(monkeypatch):
    fixture = _guard_fixture()
    _record_status(monkeypatch, error=ValueError)
    actual = paint._competing_endpoint_directions(**fixture)
    assert set(np.flatnonzero(actual)) == {5, 6}
    assert np.all(actual[[5, 6]] == paint.REASON_DIRECTION_UNCERTAIN)


def test_unexpected_programming_error_is_not_silently_swallowed(monkeypatch):
    _record_status(monkeypatch, error=RuntimeError)
    with pytest.raises(RuntimeError, match='test-only'):
        paint._competing_endpoint_directions(**_guard_fixture())


def _camera(size=96):
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1.
    return SimpleNamespace(width=size, height=size,
        camera_matrix=np.array([[50., 0., size/2.], [0., 50., size/2.], [0., 0., 1.]]),
        base_from_camera=transform, plane_offset_m=0.,
        max_sensitivity_m_per_px=paint.CONSENSUS_DEFAULT_CONFIG['max_sensitivity'])


@pytest.mark.parametrize('status,reason', [
    ('ambiguous', paint.REASON_COMPETING_DIRECTION),
    ('indeterminate', paint.REASON_DIRECTION_UNCERTAIN)])
def test_full_extraction_guard_after_proposals_preserves_raw_evidence(monkeypatch, status, reason):
    """Real mask/projection/PCA/topology; inject statuses only to test ordering."""
    mask = np.zeros((96, 96), np.uint8)
    mask[20:76, 36:60] = 255
    cuts = []
    original_directions = paint._directions
    def capture(*args, **kwargs):
        value = original_directions(*args, **kwargs)
        cuts.extend((ids.copy(), normals.copy()) for ids, normals in kwargs['cut_evidence'])
        return value
    monkeypatch.setattr(paint, '_directions', capture)
    _record_status(monkeypatch, 'clear')
    baseline = paint.extract_boundary_observation(mask, _camera(), (0., 0., 1.), 123)
    # Fail loudly if the analytic real-PCA fixture no longer contains a usable
    # selected endpoint. Do not synthesize a cut to rescue the test.
    eligible = [int(ids[0, -1]) for ids, _ in cuts
                if baseline.model_valid[int(ids[0, -1])]]
    assert eligible
    seed = eligible[0]
    original_proposals = ribbon.island_ribbon_proposals
    proposals_called = []
    def add_explicit_proposal(*args, **kwargs):
        result = list(original_proposals(*args, **kwargs))
        # Deliberately publish an admitted proposal at the actual selected
        # endpoint. A guard incorrectly placed before this stage is undone.
        result.append((np.array([seed]), baseline.normals[[seed]].copy(),
            baseline.model_points[[seed]].copy(), baseline.model_normals[[seed]].copy(),
            np.array([True]), baseline.normal_support_count[[seed]].copy(),
            baseline.normal_radius[[seed]].copy()))
        proposals_called.append(seed)
        return result
    monkeypatch.setattr(ribbon, 'island_ribbon_proposals', add_explicit_proposal)
    _record_status(monkeypatch, status)
    raw_mask = mask.tobytes()
    actual = paint.extract_boundary_observation(mask, _camera(), (0., 0., 1.), 123)
    assert proposals_called == [seed]
    assert not actual.model_valid[seed]
    assert not actual.model_reliable[seed]
    assert not actual.position_segment_valid[seed]
    assert not actual.position_segment_reliable[seed]
    assert actual.reason_codes[seed] & reason
    assert baseline.model_valid[seed]
    raw_fields = ('points', 'normals', 'segments', 'sensitivity', 'metric_valid', 'stable',
        'normal_valid', 'valid', 'reliable', 'pixel_segments', 'white_pixels', 'black_pixels',
        'previous', 'next', 'contour_ids', 'normal_support_count', 'normal_radius')
    for name in raw_fields:
        assert getattr(actual, name).tobytes() == getattr(baseline, name).tobytes(), name
    # Model coordinates/normals are audit data, not erased or silently replaced.
    for name in ('position_estimates', 'position_normals', 'position_sensitivity'):
        assert getattr(actual, name).tobytes() == getattr(baseline, name).tobytes(), name
    assert actual.source_stamp_ns == baseline.source_stamp_ns == 123
    assert mask.tobytes() == raw_mask

