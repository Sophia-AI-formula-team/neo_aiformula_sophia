import json

import cv2
import numpy as np
import pytest

from concept_yolop.geometry import CalibrationReceipt, HONDA, REFERENCE_K, HEIGHT, load_projection


def test_honda_mount_projects_vehicle_centerline_using_left_camera_info(tmp_path):
    params = dict(profile=HONDA, optical_height_m=.55, image_topic='/camera',
                  frame_id='left', mount_id='sn', width=640, height=360,
                  projection=[245.857193, 0, 328.655579, 0, 0, 245.857193,
                              189.012695, 0, 0, 0, 1, 0])
    path = tmp_path / 'honda-ground.json'
    path.write_text(json.dumps(params))
    mount = load_projection(path)
    # Raw left-camera pixel independently projected from (right=0, forward=4 m).
    pixel = mount.warp @ [332.405, 227.675, 1.]
    expected = REFERENCE_K @ [0, HEIGHT, 4.]
    np.testing.assert_allclose(pixel[:2] / pixel[2], expected[:2] / expected[2], atol=.002)
    assert mount.profile == HONDA and mount.visible.shape == (360, 640)

from concept_yolop.calibrate import install_receipt


@pytest.mark.parametrize('measured_height', [None, 0.55])
def test_measured_intrinsics_pitch_and_independent_check(tmp_path, measured_height):
    k = np.array([[246., 0., 328.7], [0., 246., 189.], [0., 0., 1.]])
    rotation, _ = cv2.Rodrigues(np.array([.025, .014, -.008]))
    def measure(points):
        result = []
        for x, z in points:
            sight = k @ rotation @ np.array([x, measured_height or HEIGHT, z])
            result.append([*(sight[:2] / sight[2]), x, z])
        return result
    fit = measure([(-.8, 2.), (.8, 2.), (-1., 8.), (1., 8.)])
    held = measure([(-.7, 2.5), (.9, 2.6), (-1.1, 7.5), (1.2, 7.8)])
    record = dict(frame_id='cam', image_topic='/img', mount_id='test', width=640, height=360,
                  projection=np.c_[k, np.zeros(3)].ravel().tolist(), fit_marks=fit, ground_marks=held)
    if measured_height is not None:
        record['optical_height_m'] = measured_height
    path = tmp_path/'receipt.json'
    path.write_text(json.dumps(record))
    receipt = CalibrationReceipt(path)
    assert receipt.max_error < 1e-7
    u, v, x, z = held[0]
    marker = np.zeros((360,640,3), np.uint8)
    cv2.circle(marker, (int(round(u)), int(round(v))), 2, (255,255,255), -1)
    expected = REFERENCE_K @ [x, HEIGHT, z]
    px, py = np.rint(expected[:2]/expected[2]).astype(int)
    assert receipt.rectify(marker)[py,px,0] > 200
    assert not receipt.visible.all()
    mask = np.ones_like(receipt.visible)
    boxes = dict(boxes=np.zeros((0,4)), labels=np.zeros(0), scores=np.zeros(0))
    road, lane, _ = receipt.restrict_predictions(mask, mask, boxes)
    assert np.array_equal(road, receipt.visible) and np.array_equal(lane, road)
    record['ground_marks'] = fit
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='different physical'):
        CalibrationReceipt(path)
    held[-1][3] += 1.
    record['ground_marks'] = held
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='0.10 m'):
        CalibrationReceipt(path)


def test_level_camera_install_and_reject_invalid_replacement(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / 'config'))
    record = dict(frame_id='cam', image_topic='/img', mount_id='test', width=640, height=360,
                  projection=np.c_[REFERENCE_K, np.zeros(3)].ravel().tolist(), optical_height_m=.55)
    points = [(-.8, 2.), (.8, 2.), (-1., 8.), (1., 8.)]
    record['ground_marks'] = []
    for right, forward in points:
        pixel = REFERENCE_K @ [right, .55, forward]
        record['ground_marks'].append([*(pixel[:2] / pixel[2]), right, forward])
    source = tmp_path / 'source.json'
    original = json.dumps(record, indent=2).encode('utf8')
    source.write_bytes(original)
    destination, receipt = install_receipt(source)
    assert destination == tmp_path / 'config' / 'concept' / 'camera.json'
    assert receipt.max_error < 1e-7
    assert destination.read_bytes() == original

    for bad_height in (0., float('nan'), True, HEIGHT):
        record['optical_height_m'] = bad_height
        source.write_text(json.dumps(record))
        with pytest.raises(ValueError):
            install_receipt(source)
        assert destination.read_bytes() == original
    assert list(destination.parent.iterdir()) == [destination]
