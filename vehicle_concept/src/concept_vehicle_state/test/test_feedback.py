from concept_vehicle_state.core import VehicleState


def test_can_order_causal_gyro_and_expiry():
    s = VehicleState()
    s.gyro(-.4, 1., 1., 2.)
    s.gyro(.8, 1.05, 1.05, 2.05)
    s.wheels([9, 0, 0, 0, 7, 0, 0, 0], 1.03, 1.06, 2.06, frame_id=1809)
    data = s.sample(1.07, 2.07)
    assert (data['leftRpm'], data['rightRpm'], data['gyro']) == (7, 9, -.4)
    assert data['t'] == 1.03 and not s.sample(1.5, 2.5)['valid']


def test_unknown_encoding_and_stale_measurement():
    s = VehicleState()
    s.gyro(1., 1., 1., 2.)
    s.wheels([9, 1, 0, 0, 7, 0, 0, 0], 1., 1., 2., frame_id=1809)
    assert s.sample(1., 2.)['leftRpm'] is None
    s.wheels([9, 0, 0, 0, 7, 0, 0, 0], 1.01, 2., 3., frame_id=1809)
    assert not s.sample(2., 3.)['valid']


def test_all_legal_low_byte_combinations_match_neo_order():
    from concept_vehicle_state.core import decode_honda_rpm
    for right in range(256):
        for left in range(256):
            raw = bytes((right, 0, 0, 0, left, 0, 0, 0))
            assert decode_honda_rpm(0x711, raw) == (left, right)
