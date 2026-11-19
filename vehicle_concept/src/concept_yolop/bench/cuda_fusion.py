"""Compare FP32 kernels on matched camera frames and, when available, recorded vehicle state."""
import argparse
import gc
import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from concept_yolop.geometry import load_projection
from concept_yolop.inference import Predictor


def samples(real, frozen, scenes, calibration):
    rows = []
    for tag, paths in [('real_camera', sorted(real.glob('frame_*.png'))),
                       ('moving_cone_fixture', sorted(frozen.glob('*_rgb.png')) if frozen else [])]:
        for p in paths:
            rows.append((tag, p.name, cv2.imread(str(p)), None))
    projection = load_projection(calibration) if scenes else None
    for scene in scenes:
        for line in (scene / 'exchange.jsonl').read_text().splitlines():
            row = json.loads(line)
            image = cv2.imread(str(scene / row['camera_file']))
            rows.append((scene.name, row['camera_file'], projection.rectify(image), row['frame']))
    if not rows or any(image is None or image.shape != (360, 640, 3) for _, _, image, _ in rows):
        raise ValueError('expected decoded 640x360 samples')
    return rows, projection


def drive_once(driver, image, road, lane, objects, frame):
    from concept_drive.transport import ros_command
    from concept_feasibility.response61515 import motor_profile
    sensor = dict(t=frame['simulation_time_s'], leftRpm=frame['left_rpm'], rightRpm=frame['right_rpm'],
                  wheelRadius=frame['wheel_radius_m'], gyro=-frame['gyro_ros_radps'], field_ground_projection=True)
    with motor_profile(True, (frame['applied_linear_mps'], -frame['applied_angular_z_radps'])):
        d = driver.step(image, road, lane, objects, sensor)
    return dict(road_rows=d['road_rows'], feasible_count=d['feasible_count'], selected=d['selected_index'],
                allowed=[c['allowed'] for c in d['candidates']], command=ros_command(d['command_native'], physical=True))


def summary(values):
    return dict(zip(('median', 'p95', 'max'), np.percentile(values, [50, 95, 100]).round(3).tolist()))


def compare(ref, actual):
    before_road, before_lane, before_det, before_drive = ref
    road, lane, det, drive = actual
    cones = (det['labels'] == 91) & (det['scores'] >= .5)
    before_cones = (before_det['labels'] == 91) & (before_det['scores'] >= .5)
    return dict(road_pixels=int(np.count_nonzero(road != before_road)),
                lane_pixels=int(np.count_nonzero(lane != before_lane)),
                lower_third_road_pixels=int(np.count_nonzero(road[240:] != before_road[240:])),
                detections=len(det['labels']), cone_count=int(cones.sum()),
                boxes_equal=all(np.array_equal(det[k], before_det[k]) for k in ('boxes', 'labels', 'scores')),
                cones_equal=all(np.array_equal(det[k][cones], before_det[k][before_cones])
                                for k in ('boxes', 'labels', 'scores')),
                road_rows_equal=drive['road_rows'] == before_drive['road_rows'] if drive else None,
                feasibility_equal=drive['allowed'] == before_drive['allowed'] if drive else None,
                selection_equal=drive['selected'] == before_drive['selected'] if drive else None,
                command_equal=drive['command'] == before_drive['command'] if drive else None)


def run(args):
    if args.scene:
        from concept_drive.core import Drive
    inputs, project = samples(args.real, args.frozen, args.scene, args.calibration)
    results = dict(device=torch.cuda.get_device_name(), torch=torch.__version__, cudnn=torch.backends.cudnn.version(),
                   before=args.before, after=args.after, timing_scope='decoded canonical BGR to both NumPy masks and detections',
                   rounds=[], comparisons=[])
    reference = []
    # Alternating block order, with only one model allocated at any time.
    for trial, mode in enumerate([args.before, args.after, args.after, args.before]):
        model = Predictor(args.weights, 'cuda', mode)
        torch.cuda.reset_peak_memory_stats()
        for _, _, image, _ in inputs[:4]:
            model.predict(image)
        drivers = {}
        times = []
        for i, (group, name, im, frame) in enumerate(inputs):
            start = time.perf_counter()
            _, road, lane, det = model.predict(im)
            times.append(1000 * (time.perf_counter() - start))
            if trial > 1:
                continue
            drive = None
            if frame is not None:
                if group not in drivers:
                    drivers[group] = Drive(2.)
                road, lane, det = project.restrict_predictions(road, lane, det)
                drive = drive_once(drivers[group], im, road, lane, det, frame)
            snapshot = (road, lane, det, drive)
            if trial == 0:
                reference.append(snapshot)
            else:
                results['comparisons'].append(dict(group=group, file=name, **compare(reference[i], snapshot)))
        results['rounds'].append(dict(setting=mode, channels=list(model.native_fusion_channels), frames=len(times),
                allocated_peak_MiB=round(torch.cuda.max_memory_allocated() / 2**20, 2),
                wall_ms=summary(times), per_frame_ms=np.round(times, 3).tolist()))
        print({k: v for k, v in results['rounds'][-1].items() if k != 'per_frame_ms'}, flush=True)
        del model, drivers
        gc.collect()
        torch.cuda.empty_cache()
        args.output.write_text(json.dumps(results, indent=2))
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--real', required=True, type=Path)
    parser.add_argument('--frozen', type=Path)
    parser.add_argument('--scene', action='append', type=Path, default=[])
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--before', default='stock')
    parser.add_argument('--after', default='auto')
    parser.add_argument('--output', type=Path, required=True)
    run(parser.parse_args())
