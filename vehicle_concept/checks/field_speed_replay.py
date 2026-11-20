"""Time the road planner on identical Unity camera frames, wheel feedback and previous commands."""
import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np
from concept_drive.core import Drive
from concept_feasibility.response61515 import motor_profile


def prepare(scene, dest):
    from concept_yolop.geometry import load_projection
    from concept_yolop.inference import Predictor
    calibration = load_projection(Path('/opt/concept/unity/honda.json'))
    network = Predictor('/weights/B1.4.pth', 'cuda')
    dest.mkdir(parents=True, exist_ok=True)
    for i, line in enumerate((scene/'exchange.jsonl').read_text().splitlines()):
        row = json.loads(line)
        pixels = cv2.imread(str(scene/row['camera_file']))
        image, road, lane, boxes = network.predict(calibration.rectify(pixels))
        road, lane, boxes = calibration.restrict_predictions(road, lane, boxes)
        np.savez(dest/f'{i:03d}.npz', image=image, road=road, lane=lane,
                 source=json.dumps(row['frame']), **boxes)


def benchmark(data, mode, minimum, baseline, repeats):
    cv2.setNumThreads(1)
    frames = []
    for file in sorted(data.glob('*.npz')):
        with np.load(file) as a:
            f = json.loads(str(a['source']))
            state = dict(t=f['simulation_time_s'], leftRpm=f['left_rpm'], rightRpm=f['right_rpm'],
                         gyro=-f['gyro_ros_radps'], wheelRadius=f['wheel_radius_m'], field_ground_projection=True)
            frames.append((a['image'], a['road'], a['lane'], {k:a[k] for k in ('boxes','labels','scores')},
                           state, (f['applied_linear_mps'], -f['applied_angular_z_radps'])))
    assert frames
    samples, digests = [], []
    for repeat in range(repeats+1):
        driver = Drive(2., mode) if baseline else Drive(2., mode, minimum)
        digest = hashlib.sha256()
        for img, road, lane, det, sensor, applied in frames:
            with motor_profile(True, applied):
                start = time.perf_counter_ns()
                d = driver.step(img, road, lane, det, sensor)
                duration = (time.perf_counter_ns()-start)/1e6
            if repeat: samples.append(duration)
            evidence = [d[k] for k in ('road_rows','objects','lines','map','map_semantics','candidates',
                                       'selected_index','reason','command_native')]
            digest.update(json.dumps(evidence, sort_keys=True, allow_nan=False).encode())
        digests.append(digest.hexdigest())
    assert len(set(digests)) == 1
    return dict(mode=mode, frames=len(frames), repeats=repeats, median_ms=np.median(samples),
                p95_ms=np.percentile(samples,95), max_ms=max(samples), digest=digests[0], raw_ms=samples)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--prepare', action='store_true')
    p.add_argument('--baseline', action='store_true')
    p.add_argument('--minimum', type=float, default=.5)
    p.add_argument('--mode', default='pure_pursuit')
    p.add_argument('--repeats', type=int, default=5)
    a=p.parse_args()
    if a.prepare:
        prepare(a.directory, a.output)
    else:
        stats=benchmark(a.directory, a.mode, a.minimum, a.baseline, a.repeats)
        a.output.write_text(json.dumps(stats))
        print({k:v for k,v in stats.items() if k!='raw_ms'})
