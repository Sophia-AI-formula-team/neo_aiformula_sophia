import json
from types import SimpleNamespace as Obj

import numpy as np
import pytest
from concept_yolop.frame_codec import encode_frame, decode_frame, parse_frame_header, decode_frame_data
from concept_yolop.geometry import CalibrationReceipt, FX, FY, CX, CY, HEIGHT, UNVERIFIED
from concept_yolop.weights import import_weights


def test_registered_and_unregistered_view(tmp_path):
    marks = [[CX + FX*x/z, CY + FY*HEIGHT/z, x, z] for x, z in ((-.8,2.),(.8,2.),(-1.,8.),(1.,8.))]
    P = [FX,0.,CX,0.,0.,FY,CY,0.,0.,0.,1.,0.]
    info = Obj(header=Obj(frame_id='camera'), width=640, height=360, p=P)
    evidence = dict(image_topic='/camera', frame_id='camera', mount_id='test-install', width=640,
                    height=360, projection=P, ground_marks=marks)
    path = tmp_path/'optics.json'
    path.write_text(json.dumps(evidence))
    registered = CalibrationReceipt(path)
    assert registered.matches('/camera', info, info)
    assert not registered.matches('/another', info, info)
    assert not registered.matches('/camera', info, None)
    marks[-1][-1] += .2
    path.write_text(json.dumps(evidence))
    with pytest.raises(ValueError):
        CalibrationReceipt(path)


def test_frame_round_trip_and_model_hash():
    bgr = np.zeros((360,640,3),np.uint8)
    bgr[8:16,:,2] = 85
    road = bgr[:,:,2] == 85
    lane = ~road
    boxes = dict(boxes=np.empty((0,4),np.float32), labels=np.zeros(0,np.int64), scores=np.zeros(0,np.float32))
    s = encode_frame(7,11,'camera',bgr,road,lane,boxes)
    obj, stamp_ns, frame_id, geometry = parse_frame_header(s)
    assert (stamp_ns, frame_id, geometry) == (7000000011, 'camera', UNVERIFIED)
    split_result = decode_frame_data(obj)
    result = decode_frame(s)
    assert result['stamp_ns'] == 7000000011 and result['geometry_profile'] == UNVERIFIED
    for k,v in dict(bgr=bgr, road=road, lane=lane, **boxes).items():
        np.testing.assert_array_equal(result[k], v)
        np.testing.assert_array_equal(split_result[k], v)
    frame = json.loads(s)
    frame['bgr_png'] = ''
    parsed, _, _, _ = parse_frame_header(json.dumps(frame))
    with pytest.raises(ValueError):
        decode_frame_data(parsed)
    frame['model_sha256'] = 'bad'
    with pytest.raises(ValueError):
        decode_frame(json.dumps(frame))


def test_bad_weight_does_not_replace_file(tmp_path):
    old, incoming = tmp_path/'model.pth', tmp_path/'wrong.pth'
    old.write_bytes(b'existing local copy')
    incoming.write_bytes(b'wrong')
    with pytest.raises(ValueError):
        import_weights(incoming, old)
    assert old.read_bytes() == b'existing local copy'
