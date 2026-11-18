from hashlib import sha256
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from concept_drive.core import Drive
from concept_drive.transport import Feedback, native_sensor, ros_command


def test_five_archived_frames_through_real_perception_wire():
    from concept_yolop.frame_codec import decode_frame, encode_frame

    path = Path(__file__).parent / 'data' / 'moving-lap-start'
    ref = json.loads((path / 'expected.json').read_text())
    driver = Drive(3.4)
    cv2.setNumThreads(1)
    for row in ref:
        prefix = path / ('%05d' % row['frame'])
        image = cv2.imread(str(prefix) + '_rgb.png', cv2.IMREAD_COLOR)
        ns = round(row['sensor']['t'] * 1e9)
        sec, nano = divmod(ns, 1_000_000_000)
        with np.load(str(prefix) + '_perception.npz') as det:
            wire = encode_frame(sec, nano, 'camera', image, det['raw'], det['lane'], det, 'frozen-unity-v626')
        f = decode_frame(wire)
        sensor = dict(row['sensor'], gyro=-row['sensor']['gyro'])
        result = driver.step(f['bgr'], f['road'], f['lane'], f, native_sensor(sensor, f['stamp_ns']/1e9))
        assert (result['whitebox_index'], result['selected_index']) == (row['baseline'], row['selected'])
        assert [c['allowed'] for c in result['candidates']] == row['allowed']
        np.testing.assert_allclose([c['score'] for c in result['candidates']], row['score'], atol=1e-5, rtol=0)
        cmd = ros_command(result['command_native'])
        np.testing.assert_allclose([cmd['linear_x_mps'], -cmd['angular_z_radps']], row['control'], atol=1e-5, rtol=0)
        json.dumps(result, allow_nan=False)


def test_feedback_only_consumes_causal_valid_measurements():
    feedback = Feedback()
    for t in [4.0, 4.15]:
        feedback.add(dict(t=t, gyro=.4, leftRpm=4., rightRpm=5., wheelRadius=.127, source='test'))
    assert feedback.at(4.1,.15)['t'] == 4.
    with pytest.raises(ValueError):
        feedback.at(4.4,.15)
    feedback.add(dict(valid=False))
    with pytest.raises(ValueError):
        feedback.at(4.16,.15)

def test_installed_model_files_match_freeze_bytes():
    import concept_feasibility
    import concept_ranker
    import concept_road

    sources = json.loads((Path(__file__).resolve().parents[1] / 'PROVENANCE.json').read_text())['sources']
    originals = {
        'experiments/v626_time/model-h12.json': (concept_ranker, 'model-h12.json'),
        'experiments/v615_recovery/response61515.json': (concept_feasibility, 'response61515.json'),
        'experiments/v616_decision/reverse_response.json': (concept_feasibility, 'reverse_response.json'),
        'experiments/v617_regression/reverse_model617.json': (concept_feasibility, 'reverse_model617.json'),
        'research/learned_racing/unity_laps/v3_evidence/plant_fit.json': (concept_road, 'v3_evidence/plant_fit.json'),
    }
    for source, (module, relpath) in originals.items():
        path = Path(module.__file__).resolve().parent / relpath
        assert sha256(path.read_bytes()).hexdigest() == sources[source], str(path)
