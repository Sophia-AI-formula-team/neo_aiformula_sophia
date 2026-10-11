"""Raster-ribbon extraction contracts, not map or runtime acceptance.

The independent oracle enumerates only actual in-image white/black faces.
Turning off the new proposal isolates its effect on immutable raw topology.
No inference from a successful local label fit to an observed connection is
allowed. Small synthetic cameras are explicit analytic test fixtures.
"""
from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np
import pytest

from lane_mapping_lya_reference import paint_geometry as paint
from lane_mapping_lya_reference import ribbon_observation as ribbon


RAW_FIELDS = ('points', 'segments', 'sensitivity', 'metric_valid', 'stable',
              'pixel_segments', 'white_pixels', 'black_pixels', 'previous', 'next', 'contour_ids')


def camera(shape, scale=.03, maximum_j=.5):
    height, width = shape
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1.
    return SimpleNamespace(width=width, height=height,
        camera_matrix=np.array([[1./scale, 0., width/2.],
                                [0., 1./scale, height/2.], [0., 0., 1.]]),
        base_from_camera=transform, plane_offset_m=0., max_sensitivity_m_per_px=maximum_j)


def scalar_xy(uv, calibration):
    # Direct ray/plane intersection for the explicit level downlooking camera.
    inverse = np.linalg.inv(calibration.camera_matrix)
    ray = calibration.base_from_camera[:3, :3] @ (inverse @ np.r_[uv, 1.])
    origin = calibration.base_from_camera[:3, 3]
    distance = -origin[2]/ray[2]
    return (origin+distance*ray)[:2]


def literal_faces(mask):
    result = set()
    height, width = mask.shape
    for y in range(height):
        for x in range(width):
            for dx, dy in ((1, 0), (0, 1)):
                xx, yy = x+dx, y+dy
                if xx >= width or yy >= height or bool(mask[y, x]) == bool(mask[yy, xx]):
                    continue
                first, second = (x, y), (xx, yy)
                white, black = (first, second) if mask[y, x] else (second, first)
                result.add((white, black))
    return result


def pattern(name):
    mask = np.zeros((48, 48), dtype=np.uint8)
    if name == 'single_3x1':
        mask[22:25, 24] = 255
    elif name == 'short_cap':
        mask[8:40, 23:26] = 255
    elif name == 'island_ribbon':
        yy, xx = np.indices(mask.shape)
        mask[(np.abs(xx-(24.+.31*(yy-24.))) < .39) & (yy >= 6) & (yy < 42)] = 255
    elif name == 'black_gap':
        mask[6:22, 23:26] = 255
        mask[25:42, 23:26] = 255
    elif name == 'parallel':
        mask[6:42, 16:18] = 255
        mask[6:42, 22:24] = 255
    elif name == 'u_arms':
        mask[8:36, 17:20] = 255
        mask[8:36, 28:31] = 255
        mask[33:36, 17:31] = 255
    elif name == 'fov':
        mask[:38, 23:26] = 255
    elif name == 'full':
        mask[:] = 255
    elif name != 'empty':
        raise AssertionError(name)
    return mask


def extract(mask, calibration=None, stamp=1):
    return paint.extract_boundary_observation(mask, calibration or camera(mask.shape), (0., 0., 1.), stamp)


def check_faces_and_brackets(mask, observed, calibration):
    expected = literal_faces(mask)
    actual = set(zip(map(tuple, observed.white_pixels), map(tuple, observed.black_pixels)))
    assert actual == expected
    assert len(observed.points) == len(expected)
    for face in np.flatnonzero(observed.model_valid):
        white = scalar_xy(observed.white_pixels[face], calibration)
        black = scalar_xy(observed.black_pixels[face], calibration)
        crossing = black-white
        estimated = observed.model_points[face]
        fraction = float((estimated-white) @ crossing/(crossing @ crossing))
        assert -2.e-13 <= fraction <= 1.+2.e-13
        np.testing.assert_allclose(estimated, white+fraction*crossing, rtol=0, atol=2.e-13)
        assert observed.model_normals[face] @ crossing > 0.
        # The existing local model normal-turn gate is unchanged.
        assert observed.model_normals[face] @ observed.normals[face] >= math.cos(math.pi/6.)-2.e-14
        uv = np.array([estimated[0]*calibration.camera_matrix[0, 0]+calibration.width/2.,
                       -estimated[1]*calibration.camera_matrix[1, 1]+calibration.height/2.])
        base = scalar_xy(uv, calibration)
        independent_j = 0.
        for delta in ((-1., 0.), (1., 0.), (0., -1.), (0., 1.)):
            shifted = uv+delta
            assert -.5 <= shifted[0] <= calibration.width-.5
            assert -.5 <= shifted[1] <= calibration.height-.5
            independent_j = max(independent_j, float(np.linalg.norm(scalar_xy(shifted, calibration)-base)))
        assert independent_j <= calibration.max_sensitivity_m_per_px+2.e-13
        assert abs(observed.position_sensitivity[face]-independent_j) <= 2.e-13
        assert observed.normal_support_count[face] >= 6
    for face in range(len(observed.points)):
        for neighbour in (observed.previous[face], observed.next[face]):
            if neighbour >= 0:
                assert observed.contour_ids[neighbour] == observed.contour_ids[face]
                common = set(map(tuple, observed.pixel_segments[face])) & set(map(tuple, observed.pixel_segments[neighbour]))
                assert len(common) == 1


@pytest.mark.parametrize('name', ['single_3x1', 'short_cap', 'island_ribbon', 'black_gap',
                                  'parallel', 'u_arms', 'fov', 'empty', 'full'])
def test_raw_faces_and_topology_byte_exact_with_ribbon_disabled(monkeypatch, name):
    mask = pattern(name)
    before = mask.tobytes()
    observed = extract(mask)
    monkeypatch.setattr(ribbon, 'island_ribbon_proposals', lambda *args, **kwargs: [])
    legacy = extract(mask)
    for field in RAW_FIELDS:
        first, second = getattr(observed, field), getattr(legacy, field)
        assert first.dtype == second.dtype and first.shape == second.shape
        assert first.tobytes() == second.tobytes(), field
    assert mask.tobytes() == before
    check_faces_and_brackets(mask, observed, camera(mask.shape))


def test_single_three_by_one_island_cannot_fabricate_six_same_side_faces():
    observed = extract(pattern('single_3x1'))
    assert len(observed.points) == 8 and len(np.unique(observed.contour_ids)) == 1
    assert not observed.normal_valid.any()
    assert not observed.model_valid.any()


def test_multi_island_actual_proposal_does_not_join_islands(monkeypatch):
    mask = pattern('island_ribbon')
    actual = ribbon.island_ribbon_proposals
    calls = []

    def record(*args, **kwargs):
        result = actual(*args, **kwargs)
        calls.extend(result)
        return result

    monkeypatch.setattr(ribbon, 'island_ribbon_proposals', record)
    observed = extract(mask)
    assert len(np.unique(observed.contour_ids)) >= 3
    assert calls, 'positive fixture must actually exercise ribbon proposal'
    assert sum(np.count_nonzero(item[4]) for item in calls) > 0
    ids = np.concatenate([item[0] for item in calls])
    assert len(np.unique(ids)) == len(ids)
    check_faces_and_brackets(mask, observed, camera(mask.shape))


@pytest.mark.parametrize('name', ['black_gap', 'parallel', 'u_arms'])
def test_gap_and_separate_arms_keep_actual_black_labels_and_disconnected_topology(name):
    mask = pattern(name)
    observation = extract(mask)
    check_faces_and_brackets(mask, observation, camera(mask.shape))
    if name != 'u_arms':
        assert len(np.unique(observation.contour_ids)) == 2
    if name == 'black_gap':
        # No original face crosses the three fully black rows, and neither may
        # the proposal add a finite observed segment there.
        assert not np.any((observation.pixel_segments.mean(axis=1)[:, 1] > 22.)
                          & (observation.pixel_segments.mean(axis=1)[:, 1] < 24.))
    for white, black in zip(observation.white_pixels, observation.black_pixels):
        assert mask[tuple(white[::-1])] == 255
        assert mask[tuple(black[::-1])] == 0


def test_image_border_does_not_create_cap_or_recertify_missing_j4():
    mask = pattern('fov')
    observed = extract(mask)
    assert not np.any(np.all(observed.pixel_segments[:, :, 1] == -.5, axis=1))
    border = np.any(observed.pixel_segments[:, :, 1] == -.5, axis=1)
    assert border.any()
    assert np.all(observed.reason_codes[border] & paint.REASON_J4_UNOBSERVED)
    assert not observed.stable[border].any()
    assert not observed.model_valid[border].any()


def test_j_cut_retains_faces_without_manufacturing_cut_edges_or_direction():
    mask = pattern('island_ribbon')
    observed = extract(mask, camera(mask.shape, scale=.11, maximum_j=.1))
    assert len(observed.points) == len(literal_faces(mask))
    assert observed.metric_valid.all()
    assert not observed.stable.any() and not observed.model_valid.any()
    assert np.all(observed.reason_codes & paint.REASON_SENSITIVITY)


@pytest.mark.parametrize('count', [0, 1, 5, 6, 512, 513])
def test_side_pca_keeps_original_six_face_and_capacity_gate(count):
    x = np.linspace(-.15, .15, count)
    points = np.column_stack((x, np.zeros(count)))
    result = ribbon._pca(points, dict(paint.DEFAULT_CONFIG))
    assert (result is not None) == (6 <= count <= 512)


@pytest.mark.parametrize('normal_scale', [.02, .3, .6])
def test_pca_variance_and_thickness_are_actual_original_eigenvalue_gates(normal_scale):
    x = np.tile(np.array([-.4, -.2, .2, .4]), 2)
    y = np.repeat(np.array([-normal_scale, normal_scale]), 4)
    points = np.column_stack((x, y))
    eigenvalues = np.linalg.eigvalsh((points-points.mean(axis=0)).T @ (points-points.mean(axis=0))/len(points))
    expected = (eigenvalues[1] > 1.e-5 and eigenvalues[0] <= .3**2
                and eigenvalues[0] <= .15*eigenvalues[1])
    assert (ribbon._pca(points, dict(paint.DEFAULT_CONFIG)) is not None) == expected


def test_helper_inputs_unchanged_and_side_selection_uses_original_cosine(monkeypatch):
    mask = pattern('island_ribbon')
    original_pca, original_helper = ribbon._pca, ribbon.island_ribbon_proposals
    evidence = []

    def checked_helper(*args):
        inputs = [value for value in args if isinstance(value, np.ndarray)]
        frozen = [(value.dtype.str, value.shape, value.tobytes()) for value in inputs]
        assert args[-2] == math.cos(math.pi/6.)
        assert args[-3] == paint.DEFAULT_CONFIG
        result = original_helper(*args)
        assert frozen == [(value.dtype.str, value.shape, value.tobytes()) for value in inputs]
        evidence.append(result)
        return result

    def checked_pca(points, cfg):
        assert cfg['normal_min_points'] == 6
        assert cfg['normal_max_variance_ratio'] == .15
        assert cfg['normal_max_thickness_m'] == .30
        assert cfg['normal_radius_m'] == .45 and cfg['normal_max_radius_m'] == 1.35
        return original_pca(points, cfg)

    monkeypatch.setattr(ribbon, '_pca', checked_pca)
    monkeypatch.setattr(ribbon, 'island_ribbon_proposals', checked_helper)
    observed = extract(mask)
    assert evidence and evidence[0]
    assert observed.model_valid.any()


def test_one_observation_still_one_vote_and_original_five_frame_confirmation():
    observation = extract(pattern('island_ribbon'))
    assert observation.model_valid.any()
    consensus = paint.BoundaryConsensus()
    token = consensus.prepare(observation)
    assert token.updates
    assert all(record.votes == 1 and record.reliable_votes <= 1 for record in token.updates)
    assert all(edge.votes == 1 and edge.reliable_votes <= 1 for edge in token.edge_updates)
    assert consensus.confirmed_count == 0 and consensus.last_stamp_ns is None
    assert consensus.commit(token)
    assert consensus.confirmed_count == 0
    with pytest.raises(ValueError):
        consensus.prepare(observation)
    for stamp in range(2, 6):
        proposal = consensus.prepare(replace(observation, source_stamp_ns=stamp))
        assert all(record.votes <= stamp for record in proposal.updates)
        assert consensus.commit(proposal)
        if stamp < 5:
            assert consensus.confirmed_count == 0
    reference = consensus.snapshot()
    assert reference.votes.size > 0
    assert np.all(reference.votes == 5)
    assert np.all(reference.reliable_votes <= 5)


def test_new_model_arrays_are_bytes_backed_and_source_stamp_is_preserved():
    observation = extract(pattern('island_ribbon'), stamp=271)
    assert observation.source_stamp_ns == 271
    for name in ('position_estimates', 'position_normals', 'position_valid',
                 'position_sensitivity', 'position_reliable'):
        array = getattr(observation, name)
        assert not array.flags.writeable
        with pytest.raises(ValueError):
            array.setflags(write=True)


def proposal_inputs(monkeypatch):
    """Capture genuine pre-proposal extraction inputs, without a second model."""
    captured = []
    with monkeypatch.context() as patch:
        patch.setattr(ribbon, 'island_ribbon_proposals',
                      lambda *args: captured.append(args) or [])
        extract(pattern('island_ribbon'))
    assert len(captured) == 1
    return list(captured[0])


def test_original_direction_valid_face_is_never_proposed_or_overwritten(monkeypatch):
    args = proposal_inputs(monkeypatch)
    baseline = ribbon.island_ribbon_proposals(*args)
    accepted_ids = np.concatenate([item[0][item[4]] for item in baseline])
    assert len(accepted_ids) > 1, 'must exercise a real, accepted proposal'
    protected = int(accepted_ids[len(accepted_ids)//2])
    original_valid = args[10].copy()
    assert not original_valid[protected]
    original_valid[protected] = True
    args[10] = original_valid
    immutable_before = original_valid.tobytes()
    proposals = ribbon.island_ribbon_proposals(*args)
    assert proposals, 'protecting one face must not make this test vacuous'
    assert all(protected not in item[0] for item in proposals)
    assert original_valid.tobytes() == immutable_before


def test_each_target_keeps_all_actual_side_support_inside_its_own_radius(monkeypatch):
    args = proposal_inputs(monkeypatch)
    actual_pca, actual_intersect = ribbon._pca, args[-1]
    pca_calls, intersections = [], []

    def record_pca(points, cfg):
        fitted = actual_pca(points, cfg)
        pca_calls.append((points.copy(), fitted))
        return fitted

    def record_intersect(*values):
        # Each proposed patch has one white-label chart and two real side-PCAs.
        # Capturing the actual support is independent of the target-radius test.
        assert len(pca_calls) >= 3
        chart, low, high = pca_calls[-3:]
        assert all(item[1] is not None for item in (chart, low, high))
        side = (values[3] @ chart[1][2]) < 0.
        intersections.append((low[0], high[0], side.copy()))
        return actual_intersect(*values)

    monkeypatch.setattr(ribbon, '_pca', record_pca)
    args[-1] = record_intersect
    proposals = ribbon.island_ribbon_proposals(*args)
    assert len(proposals) == len(intersections) > 0
    accepted = 0
    diameter_exceeds_radius = False
    for proposal, (lower, upper, lower_side) in zip(proposals, intersections):
        ids, _, _, _, valid, counts, radius = proposal
        assert len(ids) == len(lower_side)
        for support in (lower, upper):
            diameter2 = np.sum((support[:, None, :]-support[None, :, :])**2, axis=2)
            diameter_exceeds_radius |= bool(np.any(diameter2 > radius*radius))
        for row, face in enumerate(ids):
            support = lower if lower_side[row] else upper
            assert counts[row] == len(support)
            if valid[row]:
                accepted += 1
                distance2 = np.sum((args[2][face]-support)**2, axis=1)
                assert np.all(distance2 <= radius*radius), (
                    int(face), radius, float(np.max(distance2)))
    assert accepted > 0
    assert diameter_exceeds_radius, 'fixture must expose the seed-disk 2r risk'


def test_fitted_normal_must_match_actual_side_pca_not_only_common_chart(monkeypatch):
    args = proposal_inputs(monkeypatch)
    baseline = ribbon.island_ribbon_proposals(*args)
    assert sum(np.count_nonzero(item[4]) for item in baseline) > 0
    actual_pca = ribbon._pca
    raw_face_rows = {tuple(row) for row in args[2]}
    angle = math.pi/3.
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    changed_side_fits = []

    def inconsistent_side_pca(points, cfg):
        fitted = actual_pca(points, cfg)
        # Explicit admission fault injection, not a claimed physical point
        # cloud: change only actual side-PCA orientation by 60 degrees. The
        # genuine white-center chart, raster labels and polynomial stay fixed.
        # Pixel centers and internal face centers are disjoint in this fixture.
        is_side = len(points) and all(tuple(row) in raw_face_rows for row in points)
        if fitted is not None and is_side:
            origin, tangent, normal = fitted
            changed_side_fits.append(normal.copy())
            return origin, rotation @ tangent, rotation @ normal
        return fitted

    monkeypatch.setattr(ribbon, '_pca', inconsistent_side_pca)
    proposals = ribbon.island_ribbon_proposals(*args)
    assert len(changed_side_fits) >= 2
    assert proposals, 'model/bracket proposal must reach the direction gate'
    assert not any(item[4].any() for item in proposals)
