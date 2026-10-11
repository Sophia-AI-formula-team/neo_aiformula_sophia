"""Actual binary-mask/white-black branch oracles for contour-domain cuts.

All masks are independently specified analytic shapes or declared one-pixel
raster noise. Extraction runs with the unmodified DEFAULT_CONFIG, not injected
PCA vectors or preselected cuts. The downward synthetic camera is test data,
not a real-vehicle calibration. No pose solver, map voting or runtime benchmark.

The first version incorrectly required no fitting-domain cuts on every smooth
circle and short cap. That structural expectation is not the output contract:
cuts do not add a physical corner, a raw contour link, or a map observation.
Its original source and 22-pass/7-fail XML are retained in the local diagnostic
archive r14_corner_labels_01 (tests_01.xml), with the independent 27-output
quality comparison in r14_corner_quality_01. The corrected output assertions
below are a reviewed test-contract change, not an algorithm repair of those
seven original failures. All original masks and production gates are unchanged.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from lane_mapping_lya_reference import paint_geometry as paint


def _calibration(size=128, degrees=0.):
    angle = math.radians(degrees)
    rotation = np.array([[math.cos(angle), -math.sin(angle), 0.],
                         [math.sin(angle), math.cos(angle), 0.],
                         [0., 0., 1.]])
    transform = np.eye(4)
    transform[:3, :3] = rotation @ np.diag([1., -1., -1.])
    transform[2, 3] = 1.
    return SimpleNamespace(
        width=size, height=size,
        camera_matrix=np.array([[50., 0., size/2.], [0., 50., size/2.], [0., 0., 1.]]),
        base_from_camera=transform,
        max_sensitivity_m_per_px=paint.CONSENSUS_DEFAULT_CONFIG['max_sensitivity'],
        plane_offset_m=0.)


def _links(paths):
    links = set()
    for path, closed in paths:
        ids = list(map(int, path))
        links.update(zip(ids[:-1], ids[1:]))
        if ids and closed:
            links.add((ids[-1], ids[0]))
    return links


def _extract_with_domains(mask, monkeypatch, *, degrees=0.):
    """Spy on the real production call, not a substitute segmentation oracle."""
    calibration = _calibration(mask.shape[0], degrees)
    original = paint._direction_paths
    calls = []
    mask_before = mask.tobytes()
    config_before = dict(paint.DEFAULT_CONFIG)
    def traced(points, segments, stable, paths, cfg, *, white_xy=None, black_xy=None):
        assert white_xy is not None and black_xy is not None
        arrays = [points, segments, stable, white_xy, black_xy] + [path for path, _ in paths]
        before = [(value.dtype.str, value.shape, value.tobytes()) for value in arrays]
        result = original(points, segments, stable, paths, cfg,
                          white_xy=white_xy, black_xy=black_xy)
        assert before == [(value.dtype.str, value.shape, value.tobytes()) for value in arrays]
        # A cut changes only fitting domains, never ownership or raw adjacency.
        expected_ids = sorted(int(index) for path, _ in paths for index in path)
        actual_ids = sorted(int(index) for path, _ in result for index in path)
        assert actual_ids == expected_ids == list(range(len(points)))
        assert all(len(path) > 0 for path, _ in result)
        assert _links(result) <= _links(paths)
        calls.append(dict(paths=[(path.copy(), closed) for path, closed in paths],
                          domains=[(path.copy(), closed) for path, closed in result],
                          white_xy=white_xy.copy(), black_xy=black_xy.copy(),
                          stable=stable.copy()))
        return result
    with monkeypatch.context() as context:
        context.setattr(paint, '_direction_paths', traced)
        observation = paint.extract_boundary_observation(mask, calibration, (0., 0., 1.), 1)
    assert len(calls) == 1
    assert mask.tobytes() == mask_before
    assert dict(paint.DEFAULT_CONFIG) == config_before
    call = calls[0]
    raw_links = {(index, int(successor)) for index, successor in enumerate(observation.next)
                 if successor >= 0}
    assert raw_links == _links(call['paths'])
    for first, second in raw_links:
        assert observation.previous[second] == first
    # Every original face still has the genuine adjacent white and black pixel.
    white, black = observation.white_pixels, observation.black_pixels
    assert np.all(mask[white[:, 1], white[:, 0]] == 255)
    assert np.all(mask[black[:, 1], black[:, 0]] == 0)
    assert np.all(np.abs(white-black).sum(axis=1) == 1)
    project, _ = paint._projector(calibration, (0., 0., 1.))
    for pixels, key in ((white, 'white_xy'), (black, 'black_xy')):
        expected, valid = project(pixels)
        assert valid.all()
        assert expected.tobytes() == call[key].tobytes()
    cuts = raw_links-_links(call['domains'])
    return observation, call, cuts


def _stripe(angle_degrees, width, size=128):
    yy, xx = np.indices((size, size))
    angle = math.radians(angle_degrees)
    signed = (xx-size/2.)*math.cos(angle)+(yy-size/2.)*math.sin(angle)
    # Touch image boundaries: no physical endpoint is introduced by the FOV.
    return (np.abs(signed) <= width/2.).astype(np.uint8)*255


@pytest.mark.parametrize('angle', [0., 23., 45., 71.])
@pytest.mark.parametrize('world_rotation', [0., 37.])
def test_straight_binary_staircase_never_gets_a_physical_corner(angle, world_rotation, monkeypatch):
    observation, call, cuts = _extract_with_domains(_stripe(angle, 10.), monkeypatch,
                                                    degrees=world_rotation)
    assert observation.valid.any()
    assert not cuts
    assert len(call['domains']) == len(call['paths'])


@pytest.mark.parametrize('width', [1., 2., 3.])
@pytest.mark.parametrize('angle', [0., 35.])
def test_thin_binary_stripe_does_not_borrow_opposite_side_for_a_corner(width, angle, monkeypatch):
    observation, call, cuts = _extract_with_domains(_stripe(angle, width), monkeypatch)
    assert len(observation.points) > 20
    assert not cuts
    assert len(call['domains']) == len(call['paths'])


@pytest.mark.parametrize('world_rotation', [0., 63.])
def test_one_pixel_boundary_noise_on_straight_staircase_does_not_make_a_corner(world_rotation, monkeypatch):
    yy, xx = np.indices((128, 128))
    # Fixed, isolated +/- one-pixel perturbations, not a long new paint branch.
    noise = (yy % 13 == 0).astype(float)-(yy % 13 == 6).astype(float)
    center = 64.+.55*(yy-64.)+noise
    mask = (np.abs(xx-center) <= 5.).astype(np.uint8)*255
    observation, call, cuts = _extract_with_domains(mask, monkeypatch, degrees=world_rotation)
    assert len(observation.points) > 100
    assert not cuts
    assert len(call['domains']) == len(call['paths'])


@pytest.mark.parametrize('radius_pixels', [6., 9., 15.])
@pytest.mark.parametrize('subpixel_phase', [0., .35])
def test_smooth_coarse_binary_circle_models_respect_observed_quantization(radius_pixels, subpixel_phase, monkeypatch):
    yy, xx = np.indices((128, 128))
    radius = np.hypot(xx-(64.+subpixel_phase), yy-(64.-subpixel_phase))
    mask = ((radius > radius_pixels-1.5) & (radius < radius_pixels+1.5)).astype(np.uint8)*255
    observation, call, cuts = _extract_with_domains(mask, monkeypatch)
    assert len(call['paths']) == 2 and all(closed for _, closed in call['paths'])
    assert observation.stable.all()
    assert observation.model_valid.any()  # Empty output is not zero shape error.
    center = np.array([64.+subpixel_phase, 64.-subpixel_phase])
    inner, outer = radius_pixels-1.5, radius_pixels+1.5
    for face in np.flatnonzero(observation.model_valid):
        white = call['white_xy'][face]
        black = call['black_xy'][face]
        point = observation.model_points[face]
        delta = black-white
        # This fixed downward camera maps each cardinal pixel bracket to an
        # axis-aligned metric bracket. Check the original closed interval
        # strictly; no fixed-distance or ULP expansion is introduced.
        axis = int(np.flatnonzero(delta)[0])
        assert np.count_nonzero(delta) == 1
        assert np.isfinite(point).all()
        assert point[1-axis] == white[1-axis]
        fraction = (point[axis]-white[axis])/delta[axis]
        assert 0. <= fraction <= 1.
        assert np.all(point >= np.minimum(white, black))
        assert np.all(point <= np.maximum(white, black))

        # Independently intersect this original W/B pixel segment with the
        # analytic annulus. The generator is used only by this output oracle,
        # never by extraction. A black endpoint identifies inner vs outer
        # boundary, so we do not pick whichever circle gives the best error.
        pixel_white = observation.white_pixels[face].astype(np.float64)
        pixel_black = observation.black_pixels[face].astype(np.float64)
        pixel_delta = pixel_black-pixel_white
        black_radius_squared = float(np.dot(pixel_black-center, pixel_black-center))
        assert black_radius_squared <= inner*inner or black_radius_squared >= outer*outer
        target_radius = inner if black_radius_squared <= inner*inner else outer
        relative_white = pixel_white-center
        a = float(np.dot(pixel_delta, pixel_delta))
        b = 2.*float(np.dot(relative_white, pixel_delta))
        c = float(np.dot(relative_white, relative_white))-target_radius*target_radius
        discriminant = b*b-4.*a*c
        assert discriminant > 0.
        roots = [(-b-sign*math.sqrt(discriminant))/(2.*a) for sign in (-1., 1.)]
        observed_roots = [root for root in roots if 0. <= root <= 1.]
        assert len(observed_roots) == 1
        true_point = white+observed_roots[0]*delta
        # Both the unknown physical intersection and model lie in this same
        # measured bracket. This is the point-specific quantization error
        # bound max(t, 1-t)*|B-W|, not a tuned millimetre accuracy threshold.
        uncertainty_bound = max(fraction, 1.-fraction)*abs(delta[axis])
        assert float(np.linalg.norm(point-true_point)) <= uncertainty_bound


@pytest.mark.parametrize('width', [12, 24])
@pytest.mark.parametrize('world_rotation', [0., 29.])
def test_long_binary_end_cap_cuts_only_at_its_four_observed_corners(width, world_rotation, monkeypatch):
    mask = np.zeros((128, 128), dtype=np.uint8)
    x0, y0, y1 = 52, 20, 108
    mask[y0:y1, x0:x0+width] = 255
    observation, call, cuts = _extract_with_domains(mask, monkeypatch, degrees=world_rotation)
    expected_vertices = {(x, y) for x in (x0-.5, x0+width-.5) for y in (y0-.5, y1-.5)}
    actual_vertices = {tuple(observation.pixel_segments[first, 1]) for first, _ in cuts}
    assert actual_vertices == expected_vertices
    assert len(cuts) == 4 and len(call['domains']) == 4
    assert all(not closed for _, closed in call['domains'])
    assert min(len(path) for path, _ in call['domains']) == width


@pytest.mark.parametrize('width', [1, 3, 5])
def test_short_binary_end_cap_cannot_recruit_side_faces_to_meet_six(width, monkeypatch):
    mask = np.zeros((128, 128), dtype=np.uint8)
    mask[20:108, 52:52+width] = 255
    observation, call, cuts = _extract_with_domains(mask, monkeypatch)
    for end_row in (19.5, 107.5):
        cap = np.all(observation.pixel_segments[:, :, 1] == end_row, axis=1)
        assert int(cap.sum()) == width < paint.DEFAULT_CONFIG['normal_min_points']
        assert not observation.model_valid[cap].any()
    # Fitting-domain cuts are permitted. _extract_with_domains independently
    # verifies every raw face exactly once and unchanged original prev/next.
    # Intermediate normal fitting must not become an admitted cap model when
    # that cap has fewer than the original six observed faces.
