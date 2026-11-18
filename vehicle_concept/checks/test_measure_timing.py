"""Offline records exercise source matching, duplicate commands and time-bin boundaries."""
from measure_timing import Measurement, NS, timestamp
import pytest


def test_source_pairing_and_clock_evidence():
    data = Measurement()
    t = 100 * NS
    m = NS
    for i in range(9):
        stamp = t + i * 66_666_667
        data.camera.observe(stamp, stamp+10_000, m+i*66_666_667)
        data.perception.observe(stamp, stamp+30_000, m+i*66_666_667+30_000)
    for i, status, skipped in ((0, 'ok', 0), (3, 'shadow', 2), (6, 'ok', 4)):
        stamp = t + i * 66_666_667
        sec, nano = divmod(stamp, NS)
        data.decision(dict(session='test', status=status, reason='stage', stamp=dict(sec=sec, nanosec=nano),
                           processing_ms=9., capture_age_s=.02, command={}, planning_hz=5.,
                           rate_skipped_total=skipped), stamp+20_000_000, m+i*66_666_667+20_000_000)
    data.twists.observe(t, t+30_000_000, m+30_000_000)
    newest = t+6*66_666_667
    data.twists.observe(newest, newest+210_000_000, m+610_000_002)
    data.twists.observe(newest, newest+220_000_000, m+620_000_002)
    data.twists.observe(t+66_666_667, newest+240_000_000, m+640_000_002)
    clock = dict(operator_confirmed_vehicle_clock=False, use_sim_time=False, clock_publishers=0,
                 zed_publisher_present=True, camera_publishers=['/aiformula_sensing/zed_node'])
    out = data.report(.9, 5, clock, m+900_000_000)
    assert out['auto_cmd']['duplicates'] == out['auto_cmd']['out_of_order'] == 1
    assert out['cmd_with_camera'] == 2 and out['cmd_without_camera'] == 0
    assert out['capture_to_twist_over_200ms'] == out['twist_intervals_over_200ms'] == 1
    assert out['capture_to_observed_twist_s']['p50'] == pytest.approx(.12)
    assert out['observed_twist_intervals_s']['maximum'] == pytest.approx(.580000002)
    assert out['rate_skipped_confirmed'] == 4
    assert out['statuses'] == {'ok': 2, 'shadow': 1}
    assert out['complete_periods'] == 4 and out['periods_without_new_twist'] == 2
    assert not out['clock']['eligible_for_vehicle_timing_evaluation']
    clock['operator_confirmed_vehicle_clock'] = True
    assert data.report(.9, 5, clock, m+900_000_000)['clock']['eligible_for_vehicle_timing_evaluation']
    clock['clock_publishers'] = 1
    assert not data.report(.9, 5, clock, m+900_000_000)['clock']['eligible_for_vehicle_timing_evaluation']


def test_unmeasured_fields_stay_unknown():
    data = Measurement()
    out = data.report(.1, 5, dict(operator_confirmed_vehicle_clock=False, use_sim_time=True,
                                  clock_publishers=1, zed_publisher_present=False), NS)
    for k in ('rate_skipped_confirmed', 'periods_without_new_twist', 'capture_to_observed_twist_s'):
        assert out[k] is None
    with pytest.raises(ValueError):
        timestamp(0, 0)
    with pytest.raises(ValueError):
        timestamp(1, NS)
