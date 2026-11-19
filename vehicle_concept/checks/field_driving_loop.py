"""Closed-loop field-policy check on a synthetic two-lane straight, without ROS.

The renderer alone knows the physical limits. Drive receives its usual field-camera
inputs and wheel/gyro feedback. The plant uses the fitted forward/yaw/braking
response and the gate's most recent request, so successive images follow the
chosen commands. This checks the driver with prescribed perception, not YOLOP.
"""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from concept_drive.core import Drive
from concept_feasibility.response61515 import motor_profile
from concept_feasibility.reverse_model617 import step as plant_step
from concept_road.onboard625 import HALF_PAINT, HALF_TRACK, HALF_TREAD
from concept_road.vision import CX, CY, FOCAL, HEIGHT


DT = .2
RIGHT_EDGE, LEFT_EDGE = 3.5, -3.5
EMPTY = dict(boxes=np.empty((0, 4)), scores=np.empty(0), labels=np.empty(0, int))
Y, U = np.indices((360, 640))
GROUND = Y > CY + 1
Z = np.where(GROUND, HEIGHT * FOCAL / np.maximum(Y - CY, 1.), 0.)
X = (U - CX) * Z / FOCAL


def camera(forward, right, heading):
    c, s = math.cos(heading), math.sin(heading)
    world_x = right + X * c + Z * s
    world_z = forward + Z * c - X * s
    pavement = GROUND & (world_x >= LEFT_EDGE) & (world_x <= RIGHT_EDGE)
    outer = np.minimum(abs(world_x - LEFT_EDGE), abs(world_x - RIGHT_EDGE)) <= HALF_PAINT
    divider = (abs(world_x) <= HALF_PAINT) & (world_z % 4. < 2.)
    lines = (outer | divider) & GROUND
    image = np.empty((360, 640, 3), np.uint8)
    image[:] = (33, 36, 39)
    image[GROUND] = (61, 76, 59)
    image[pavement] = (64, 65, 66)
    image[lines] = (193, 196, 196)
    own_lane = (world_x <= 0.) if right < 0. else (world_x >= 0.)
    seed = pavement & own_lane
    return image, seed, lines


def run(name, right, heading, initial_speed, seconds):
    drive = Drive(2.)
    forward, speed, gyro, released = 0., initial_speed, 0., 0.
    last = (initial_speed, 0.)
    rows = []
    for tick in range(int(seconds / DT)):
        image, seed, lines = camera(forward, right, heading)
        rpm = speed * 30. / (math.pi * .127)
        sensor = dict(t=tick * DT, wheelRadius=.127, leftRpm=rpm, rightRpm=rpm,
                      gyro=gyro, field_ground_projection=True, source='closed_loop_test')
        with motor_profile(True, last):
            result = drive.step(image, seed, lines, EMPTY, sensor)
            v, w = result['command_native']
            before = (forward, right)
            clearance = float('inf')
            for _ in range(4):
                new_v, new_w, released = (float(value) for value in plant_step(speed, gyro, v, w, .05, released))
                mid = heading + (gyro + new_w) * .0125
                distance = (speed + new_v) * .025
                forward += distance * math.cos(mid)
                right += distance * math.sin(mid)
                heading += (gyro + new_w) * .025
                speed, gyro = new_v, new_w
                inner_tread = (HALF_TRACK - HALF_TREAD) * abs(math.cos(heading))
                gap = min(right - LEFT_EDGE, RIGHT_EDGE - right) + HALF_PAINT - inner_tread
                clearance = min(clearance, gap)
            last = (float(v), float(w) if v != 0 else 0.)
        rows.append(dict(t=round((tick + 1) * DT, 3), forward=forward, right=right, clearance=clearance,
                         heading=heading, speed=speed, command=[float(v),float(w)],
                         moved=math.hypot(forward-before[0], right-before[1]), allowed=result['feasible_count'],
                         reason=result['reason'], launch=result.get('launch_decision'),
                         selected=result['selected_index'], edge_candidate=result.get('edge_candidate_index')))
    return dict(name=name, simulated_s=seconds, forward_m=forward, min_tread_margin_m=min(r['clearance'] for r in rows),
                stopped_s=DT * sum(abs(r['speed']) < .04 for r in rows), rows=rows)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('output', type=Path)
    ap.add_argument('--seconds', type=float, default=12.)
    ap.add_argument('--verify', action='store_true', help='Require 20 m progress, zero excursions and under 1 s stopped in each 12 s scene')
    args = ap.parse_args()
    cv2.setNumThreads(1)
    cases = [('straight_start', -1.75, 0., 0.),
             ('heading_toward_left_edge', -2.25, math.radians(-14.), 1.4),
             ('heading_toward_right_edge', 2.25, math.radians(14.), 1.4)]
    results = [run(*case, seconds=args.seconds) for case in cases]
    args.output.write_text(json.dumps(results, allow_nan=False), encoding='utf-8')
    for row in results:
        print(row['name'], 'progress %.2f m; minimum tyre margin %.3f m; stopped %.1f s' %
              (row['forward_m'], row['min_tread_margin_m'], row['stopped_s']))
    if args.verify and (args.seconds != 12. or any(row['forward_m'] < 20.
                        or row['min_tread_margin_m'] < 0 or row['stopped_s'] >= 1. for row in results)):
        raise SystemExit('field loop regression')
