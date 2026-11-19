import numpy as np

from concept_road.field_road import FieldRoad
from concept_road.vision import CX, CY, FOCAL, HEIGHT


def scene(*, left=-1.4, right=2.5, divider=.55, slope=0.,
          shoulder=(50, 94, 51), spill=0., road_color=(83, 88, 85)):
    v, u = np.mgrid[:360, :640]
    z = HEIGHT * FOCAL / np.maximum(1, v-CY)
    x = (u-CX)*z/FOCAL - slope*z
    on_ground = v > CY+3
    pavement = (x >= left) & (x <= right) & on_ground
    image = np.empty((360, 640, 3), np.uint8)
    image[:] = shoulder
    image[pavement] = road_color
    markings = (abs(x-left) <= .043) | (abs(x-right) <= .043) | (abs(x-divider) <= .043)
    markings &= on_ground
    image[markings] = (224, 225, 224)
    predicted = (x >= left-spill) & (x <= divider) & on_ground
    zz = np.r_[np.arange(-1.2, .95, .16), np.linspace(.95, 12., 45)]
    rows = np.c_[zz, np.full(len(zz), -8.), np.full(len(zz), 8.), np.zeros((len(zz), 2))]
    return image, predicted, markings, rows


def observe(observer, s, **kw):
    im, road_head, lane, rows = s
    return observer.update(im, road_head, lane, rows, [], kw.pop('speed', 0.), 0., kw.pop('dt', .2), **kw)


def limits(result, depth):
    z = result.road[:, 0]
    return np.interp(depth, z, result.road[:, 1]), np.interp(depth, z, result.road[:, 2])


def test_painted_outer_edge_corrects_b_spill_but_interior_paint_can_be_crossed():
    obs = observe(FieldRoad(), scene(spill=.8))
    left, right = limits(obs, 3.)
    assert -1.58 < left < -1.36
    assert 2.35 < right < 2.65
    assert obs.audit['exterior_rows'][0] >= 10
    assert obs.audit['visible_z'][0] == .95
    assert obs.reference is not None


def test_visible_asphalt_across_interior_marking_even_when_outer_edge_is_off_camera():
    obs = observe(FieldRoad(), scene(right=30.))
    assert limits(obs, 3.)[1] > 1.3
    assert any('paint_connected_asphalt' in row for row in obs.audit['observed_bounds'])


def test_no_road_head_does_not_grant_field_of_view_and_memory_expires():
    fresh = scene()
    empty = (fresh[0], np.zeros_like(fresh[1]), fresh[2], fresh[3])
    assert not len(observe(FieldRoad(), empty).road)
    observer = FieldRoad()
    observe(observer, fresh)
    old = observe(observer, empty, speed=.6)
    assert len(old.road) and old.audit['visible_z'] is None and old.audit['memory_rows'] > 0
    assert not len(observe(observer, empty, dt=.5).road)


def test_marked_non_green_shoulder_and_specular_glare():
    colored = observe(FieldRoad(), scene(shoulder=(49, 71, 120)))
    assert -1.6 < limits(colored, 3.)[0] < -1.3
    base = scene()
    base[0][:] = 252
    glare = observe(FieldRoad(), base)
    assert not len(glare.road)
    assert glare.audit['visible_z'] is None


def test_early_guidance_and_observed_frontier_before_heading_reaches_verge():
    obs = observe(FieldRoad(), scene(left=-.78, right=3.6, divider=1.7, slope=.30, spill=.55))
    left, right = limits(obs, 4.)
    assert .30 < left < .55 and right > left + 1.
    assert np.interp(4., obs.road[:, 0], obs.heading) > .12
    assert np.interp(4., obs.reference['points'][:, 0], obs.reference['points'][:, 1]) > .7


def test_hole_not_credited_as_current_or_cached_as_rear():
    base = scene()
    z = 1.95
    y = round(CY+HEIGHT*FOCAL/z)
    base[1][y-6:y+7] = False
    result = observe(FieldRoad(), base)
    assert result.audit['visible_z'][1] < 2.1
    assert result.road[0, 0] >= -.17
    assert result.audit['footprint_rows'] > 0


def test_near_field_inference_cannot_override_car_patch_or_grant_reverse_room():
    base = scene()
    base[3][base[3][:, 0] <= 0, 1:3] = [.31, .43]
    result = observe(FieldRoad(), base)
    assert result.road[0, 0] >= -.17
    assert limits(result, 0)[0] <= -.4 and limits(result, 0)[1] >= .4
    base[3][base[3][:, 0] <= 0, 1:3] = [-1.35, 1.1]
    meta = [{'z': z, 'source': 'historical_material_observation'} for z in base[3][:, 0] if z < .95]
    historical = observe(FieldRoad(), base, nearfield_rows=meta)
    assert historical.road[0, 0] <= -1.2
    assert np.isclose(limits(historical, -.6)[0], -1.35)
    assert historical.audit['memory_rows'] > 0


def test_an_unpainted_prediction_does_not_open_all_adjacent_gray_pixels():
    image, b, lane, rows = scene(right=30.)
    image[:] = (80, 81, 79)
    result = observe(FieldRoad(), (image, b, np.zeros_like(lane), rows))
    assert -.05 < limits(result, 3.)[1] - .55 < .05


def test_recover_split_nearest_slice_only_if_the_next_slice_supports_it():
    img, b, lane, rows = scene(left=-1.6, right=2.5, divider=1.5)
    # A dropout close to the bumper with positive B on both sides.
    b[340:, 255:405] = False
    result = observe(FieldRoad(), (img, b, lane, rows))
    left, right = limits(result, .95)
    assert left < -.5 and right > .5


def test_paint_gap_uses_material_transition_to_interpolate_measured_edge():
    img, b, lane, rows = scene(left=-.6, right=5., divider=3., slope=.25, spill=.7)
    z = HEIGHT*FOCAL / np.maximum(1, np.arange(360)-CY)
    band = (z > 4) & (z < 5.8)
    lane[band] = False
    result = observe(FieldRoad(), (img, b, lane, rows))
    assert abs(limits(result, 5.)[0] - .65) < .20
    assert any(row[1] == 'connected_material_edge' for row in result.audit['observed_bounds'])


def test_sensor_measurements_survive_empty_legacy_road_fit():
    image, predicted, lane, _ = scene()
    r = observe(FieldRoad(), (image, predicted, lane, np.empty((0, 5))))
    assert limits(r, 3.)[0] < -1. and limits(r, 3.)[1] > 2.
    assert r.road[0, 0] >= -.17


def test_uniform_green_false_positive_does_not_become_the_asphalt_reference():
    image, b, lane, rows = scene()
    image[:] = (45, 99, 48)
    b[:] = True
    lane[:] = False
    assert not len(observe(FieldRoad(), (image, b, lane, rows)).road)
