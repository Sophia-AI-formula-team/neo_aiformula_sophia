from concept_vehicle_state.core import VehicleState


def test_can_order_causal_gyro_and_expiry():
    s = VehicleState()
    s.gyro(-.4, 1., 1., 2.)
    s.gyro(.8, 1.05, 1.05, 2.05)
    s.wheels([9, 0, 0, 0, 7, 0, 0, 0], 1.03, 1.06, 2.06, frame_id=1809)
    data = s.sample(1.07, 2.07)
    assert (data['leftRpm'], data['rightRpm'], data['gyro']) == (7, 9, -.4)
    assert data['t'] == 1.03 and not s.sample(1.5, 2.5)['valid']


def test_out_of_range_rpm_and_stale_measurement():
    s = VehicleState()
    s.gyro(1., 1., 1., 2.)
    s.wheels([255, 127, 0, 0, 7, 0, 0, 0], 1., 1., 2., frame_id=1809)
    assert s.sample(1., 2.)['leftRpm'] is None
    s.wheels([9, 0, 0, 0, 7, 0, 0, 0], 1.01, 2., 3., frame_id=1809)
    assert not s.sample(2., 3.)['valid']


def test_all_legal_low_byte_combinations_match_neo_order():
    from concept_vehicle_state.core import decode_honda_rpm
    for right in range(256):
        for left in range(256):
            raw = bytes((right, 0, 0, 0, left, 0, 0, 0))
            assert decode_honda_rpm(0x711, raw) == (left, right)


def test_signed_reverse_from_vehicle_recording():
    from concept_vehicle_state.core import decode_honda_rpm
    assert decode_honda_rpm(0x711, bytes.fromhex('bbffffffbdffffff')) == (-67, -69)
    assert decode_honda_rpm(0x711, bytes.fromhex('09000000d7ffffff')) == (-41, 9)
    s = VehicleState()
    s.gyro(.08, 1., 1., 2.)
    s.wheels(bytes.fromhex('d7ffffffd4ffffff'), 1., 1., 2., frame_id=0x711)
    data = s.sample(1.01, 2.01)
    assert data['valid'] and data['rpm_encoding'] == 'signed_le_int32'
    assert (data['leftRpm'], data['rightRpm']) == (-44, -41)


def test_recorded_high_word_and_four_mps_cruise_feedback():
    import struct
    import pytest
    from concept_vehicle_state.core import decode_honda_rpm

    from concept_vehicle_state.core import MAX_WHEEL_RPM
    assert MAX_WHEEL_RPM == 487
    for right, left in ((253, 202), (256, 202), (341, 258), (301, 301), (258, 341),
                        (272, 395), (395, 272), (0, -395), (-487, 487)):
        assert decode_honda_rpm(0x711, struct.pack('<ii', right, left)) == (left, right)
    for right, left in ((488, 0), (0, 488), (-488, 0), (0, -488)):
        with pytest.raises(ValueError, match='unsupported_can_rpm_range'):
            decode_honda_rpm(0x711, struct.pack('<ii', right, left))
