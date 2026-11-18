import json

import cv2
import numpy as np
import pytest

from concept_yolop.geometry import CalibrationReceipt, REFERENCE_K, HEIGHT


def test_measured_intrinsics_pitch_and_independent_check(tmp_path):
    k = np.array([[246., 0., 328.7], [0., 246., 189.], [0., 0., 1.]])
    rotation, _ = cv2.Rodrigues(np.array([.025, .014, -.008]))
    def measure(points):
        result = []
        for x, z in points:
            sight = k @ rotation @ np.array([x, HEIGHT, z])
            result.append([*(sight[:2] / sight[2]), x, z])
        return result
    fit = measure([(-.8, 2.), (.8, 2.), (-1., 8.), (1., 8.)])
    held = measure([(-.7, 2.5), (.9, 2.6), (-1.1, 7.5), (1.2, 7.8)])
    record = dict(frame_id='cam', image_topic='/img', mount_id='test', width=640, height=360,
                  projection=np.c_[k, np.zeros(3)].ravel().tolist(), fit_marks=fit, ground_marks=held)
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
