"""Cases where the B head divides the observed asphalt into two islands."""
import numpy as np
import pytest

from concept_road.field_scan import _section
from concept_road.field_road import ordered_marking_rows
from concept_road.vision import CX, FOCAL


def divided_scan():
    scale = FOCAL / 6.
    centers = np.array([180, 340, 500])
    order = (centers - CX) / scale
    road = np.zeros(640, bool)
    road[105:267] = True
    road[340:471] = True
    paint = np.zeros_like(road)
    for center in centers:
        paint[center - 2:center + 3] = True
    green = np.zeros(640)
    green[505:] = .18
    return road, paint, green, np.zeros(640), np.full(640, 85, np.uint8), order


def test_current_two_lane_material_reaches_real_outer_paint():
    road, paint, green, blue, light, order = divided_scan()
    before = _section(6., road, paint, green, blue, light, 210.)
    after = _section(6., road, paint, green, blue, light, 210., order)
    assert before[-1][1] == 'road_head'
    assert after[-1][1] == 'two_lane_material_edge'
    assert abs(after[0][1] - (order[-1] + .055)) < 1e-6
    assert after[1][1]
    assert before[2] == after[2]


@pytest.mark.parametrize('reason', ['turf', 'dark_seam', 'overexposure', 'extra_stripe',
                                    'no_neighbouring_B', 'near_outer_edge', 'no_order'])
def test_union_stops_at_a_real_barrier_or_missing_evidence(reason):
    road, paint, green, blue, light, order = divided_scan()
    if reason == 'turf':
        green[365:400] = .22
    elif reason == 'dark_seam':
        light[343:345] = 0
    elif reason == 'overexposure':
        light[365:395] = 251
    elif reason == 'extra_stripe':
        paint[408:412] = True
    elif reason == 'no_neighbouring_B':
        road[300:] = False
    elif reason == 'near_outer_edge':
        green[347:370] = .25
    elif reason == 'no_order':
        order = None
    result = _section(6., road, paint, green, blue, light, 210., order)
    assert result[-1][1] != 'two_lane_material_edge'
    assert result[0][1] < .65


def test_partial_only_proposes_end_of_measured_B_with_known_outer_stripe():
    road, paint, green, blue, light, order = divided_scan()
    green[:] = 0
    partial = np.full(2, np.nan)
    result = _section(6., road, paint, green, blue, light, 210., order, partial)
    assert result[-1][1] == 'road_head' and not result[1][1]
    assert np.isclose(partial[1], (470 - CX) * 6. / FOCAL)
    order[2] = np.nan
    partial[:] = np.nan
    _section(6., road, paint, green, blue, light, 210., order, partial)
    assert np.isnan(partial).all()


def test_markings_at_an_identified_turnout_need_an_edge_that_continues_along_main_road():
    depths = np.array([6., 9., 10.])
    opening = [{'rows': [[3.6, -4.1, -.55], [9., -4.1, -.55]]}]

    def lines(outer_end):
        return [dict(track=i, kind='solid', points=[[3.6, x], [6., x], [end, x]])
                for i, x, end in [(1, -4.35, outer_end), (2, -.55, 10.), (3, 3.25, 10.)]]

    # The grass beyond the mouth is observable while the main road continues.
    local = ordered_marking_rows(depths, [1, 2, 3], lines(9.1), opening)
    assert np.isnan(local[0, 0]) and np.isfinite(local[0, 1:]).all()
    # An ordinary legal adjacent lane is also labelled as an 'opening' by the
    # current detector; its third stripe continues past the end of the mouth.
    continuous = ordered_marking_rows(depths, [1, 2, 3], lines(10.), opening)
    assert np.isfinite(continuous[0]).all()
    # An observably terminating exterior cannot corroborate a lane on its own.
    unresolved = ordered_marking_rows(depths, [1, 2, 3], lines(8.8))
    assert np.isnan(unresolved[0, 0])
