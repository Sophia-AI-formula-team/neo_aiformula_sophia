#!/usr/bin/env python3
"""Synthetic completed-lap route building and causal fixed-reference tracking.

This is an ideal 2-D unicycle/controller demonstration, NOT perception, SLAM,
ROS transport, hardware-stop or vehicle validation. The completed first lap is
available before route construction; each second-lap update receives only the
current simulated pose. No future observations or teacher speed recording are
used by the runtime controller. All production safety limits stay unchanged.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
CONTROL = PACKAGE.parent
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(CONTROL / 'trajectory_follower'))
from trajectory_follower.lya_profile import REFERENCE_SPEED_MPS
from lane_mapping_lya_reference.controller import ClosedRouteController, ControlFault, Pose, wrap
from lane_mapping_lya_reference.route import RouteBuildError, build_route


def completed_lap(radius):
    """An explicit ideal first-lap fixture, not claimed as a built real map."""
    angle = np.linspace(0, 2 * math.pi, 801)
    trace = np.column_stack((radius * np.cos(angle), radius * np.sin(angle), angle + math.pi / 2))
    dense = np.linspace(0, 2 * math.pi, 4800, endpoint=False)
    lanes = np.vstack([np.column_stack((r * np.cos(dense), r * np.sin(dense)))
                       for r in (radius - 1.2, radius + 1.2)])
    return trace, lanes


def route_at(radius):
    return build_route(*completed_lap(radius), config={'reference_speed_mps': REFERENCE_SPEED_MPS})


def controller_for(route):
    # The production defaults retain lateral acceleration 0.35, acceleration
    # 0.5 and yaw acceleration 0.8. No gate is raised to make this demo pass.
    return ClosedRouteController(route, maximum_speed=REFERENCE_SPEED_MPS,
                                 reference_speed_mps=REFERENCE_SPEED_MPS)


def track(route, dt=0.05):
    controller = controller_for(route)
    first = route['route_samples'][0]
    x, y = first['x_m'] + 0.10, first['y_m']
    yaw, speed = first['yaw_rad'] + 0.04, 0.0
    start = 1_000_000_000
    controller.attach(Pose(x, y, yaw, speed, start, start))
    history = []
    deadline = math.ceil((controller.length / REFERENCE_SPEED_MPS * 1.5 + 10) / dt)
    for index in range(deadline):
        stamp = start + int(index * dt * 1e9)
        pose = Pose(x, y, yaw, speed, stamp, stamp)
        command = controller.update(pose, dt)
        metrics = controller.last_metrics
        history.append(dict(time_s=index * dt, x_m=x, y_m=y, yaw_rad=yaw,
            progress_m=metrics['progress_m'], cross_track_m=metrics['cross_track_m'],
            heading_error_rad=metrics['heading_error_rad'],
            reference_speed_mps=metrics['v_ref_mps'], speed_mps=command.speed,
            yaw_rate_rps=command.yaw_rate,
            commanded_lateral_acceleration_mps2=abs(command.speed * command.yaw_rate)))
        if controller.progress >= controller.length:
            return history, controller
        # Exact constant-command integration over this tick; next measurement
        # exists only after the current production command has been generated.
        next_yaw = yaw + command.yaw_rate * dt
        if abs(command.yaw_rate) < 1e-10:
            x += command.speed * math.cos(yaw) * dt
            y += command.speed * math.sin(yaw) * dt
        else:
            x += command.speed / command.yaw_rate * (math.sin(next_yaw) - math.sin(yaw))
            y -= command.speed / command.yaw_rate * (math.cos(next_yaw) - math.cos(yaw))
        yaw, speed = wrap(next_yaw), command.speed
    raise AssertionError('Synthetic route did not complete within its explicit timeout')


def rejection(radius):
    try:
        route = route_at(radius)
    except RouteBuildError as error:
        return dict(radius_m=radius, rejected=True, stage='route_builder', reason=str(error),
                    diagnostics=error.diagnostics)
    try:
        controller_for(route)
    except ControlFault as error:
        return dict(radius_m=radius, rejected=True, stage='controller_attachment', reason=str(error),
                    route_max_curvature_1pm=route['diagnostics']['max_curvature_1pm'],
                    required_lateral_acceleration_mps2=REFERENCE_SPEED_MPS ** 2 * route['diagnostics']['max_curvature_1pm'],
                    allowed_lateral_acceleration_mps2=0.35)
    raise AssertionError('Expected a tight-curve fixed-speed rejection at radius ' + str(radius))


def run_demo():
    started = time.perf_counter()
    radius = max(15.0, 5.0 * REFERENCE_SPEED_MPS ** 2)
    route = route_at(radius)
    history, controller = track(route)
    cte = np.array([row['cross_track_m'] for row in history])
    speed = np.array([row['speed_mps'] for row in history])
    steady = [row for row in history if row['time_s'] >= 10.0]
    steady_cte = np.array([row['cross_track_m'] for row in steady])
    tight = [rejection(REFERENCE_SPEED_MPS ** 2 / 0.8),
             rejection(REFERENCE_SPEED_MPS ** 2 / (4.0 / 9.0))]
    assert {row['reference_speed_mps'] for row in history} == {REFERENCE_SPEED_MPS}
    assert {row['speed_mps'] for row in route['route_samples']} == {REFERENCE_SPEED_MPS}
    assert np.max(cte) < controller.maximum_cross_track
    source_paths = [PACKAGE / 'lane_mapping_lya_reference/controller.py',
                    PACKAGE / 'lane_mapping_lya_reference/route.py',
                    CONTROL / 'trajectory_follower/trajectory_follower/lya_profile.py']
    report = dict(schema_version=1, passed=True, evidence_level='synthetic_offline_only',
        fixture=dict(radius_m=radius, lane_width_m=2.4, timestep_s=0.05,
                     initial_outward_offset_m=0.10, initial_heading_offset_rad=0.04),
        sources={str(path.relative_to(CONTROL)).replace('\\', '/'):
                 hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths},
        reference_speed_mps=REFERENCE_SPEED_MPS, speed_policy=route['speed_policy'],
        gates=dict(route_lateral_acceleration_mps2=route['config']['max_lateral_accel_mps2'],
                   controller_lateral_acceleration_mps2=controller.maximum_lateral_acceleration,
                   controller_acceleration_mps2=controller.maximum_acceleration,
                   controller_yaw_acceleration_rps2=controller.maximum_yaw_acceleration,
                   controller_maximum_yaw_rate_rps=controller.maximum_yaw_rate,
                   maximum_cross_track_m=controller.maximum_cross_track),
        route_diagnostics=route['diagnostics'],
        tracking=dict(completed_laps=int(controller.progress // controller.length),
                      simulated_duration_s=history[-1]['time_s'], samples=len(history),
                      cross_track_rmse_m=float(np.sqrt(np.mean(cte ** 2))),
                      cross_track_p95_m=float(np.quantile(cte, 0.95)),
                      cross_track_max_m=float(np.max(cte)),
                      final_cross_track_m=history[-1]['cross_track_m'],
                      steady_after_s=10.0,
                      steady_cross_track_rmse_m=float(np.sqrt(np.mean(steady_cte ** 2))),
                      steady_mean_command_speed_mps=float(np.mean([row['speed_mps'] for row in steady])),
                      commanded_speed_min_mps=float(np.min(speed)),
                      commanded_speed_max_mps=float(np.max(speed)),
                      maximum_commanded_lateral_acceleration_mps2=max(
                          row['commanded_lateral_acceleration_mps2'] for row in history)),
        tight_curve_rejections=tight, elapsed_wall_s=time.perf_counter() - started,
        limits=['Ideal noiseless unicycle and exact current pose, not vehicle accuracy',
                'Full synthetic completed first lap available before route construction',
                'No mask projection, online localization, sensor errors, DDS or hardware exercised',
                '{} m/s is the current LYA-profile reference; startup slew and feedback remain active'.format(REFERENCE_SPEED_MPS),
                '0.35 m/s^2 checks fixed-reference route feasibility, not every feedback command',
                'No future runtime observation or recorded teacher-speed replay'])
    return report, route, history


def svg_preview(report, route, history):
    radius = report['fixture']['radius_m']
    scale, origin_x, origin_y = 220.0 / radius, 275, 290
    def points(rows):
        return ' '.join('{:.2f},{:.2f}'.format(origin_x + row['x_m'] * scale,
                                             origin_y - row['y_m'] * scale) for row in rows)
    tracking = report['tracking']
    circles = ''.join('<circle cx="275" cy="290" r="{}" fill="none" stroke="#536276"/>'.format(r * scale)
                      for r in (radius - 1.2, radius + 1.2))
    return '''<svg xmlns="http://www.w3.org/2000/svg" width="930" height="570" viewBox="0 0 930 570">
<rect width="930" height="570" fill="#101923"/>
<g font-family="sans-serif" fill="#e6edf3">
<text x="28" y="34" font-size="21">Synthetic fixed-reference tracking | NOT vehicle evidence</text>
{}<polyline points="{}" fill="none" stroke="#47b7ee" stroke-width="3"/>
<polyline points="{}" fill="none" stroke="#ffce67" stroke-width="1.3"/>
<text x="560" y="108" font-size="22">{} m/s LYA-profile reference</text>
<text x="560" y="146">{} m radius | 2.4 m lane width</text>
<text x="560" y="176">Blue: completed-map route</text>
<text x="560" y="202">Yellow: current-feedback simulation</text>
<text x="560" y="252">Completed laps: {}</text>
<text x="560" y="278">Cross-track RMSE: {:.4f} m</text>
<text x="560" y="304">Maximum cross-track: {:.4f} m</text>
<text x="560" y="330">Steady command speed: {:.4f} m/s</text>
<text x="560" y="380">{} m radius: route rejected</text>
<text x="560" y="406">{} m radius: controller rejected</text>
<text x="560" y="446">Original 0.60 / 0.35 m/s2 gates retained.</text>
<text x="28" y="548" font-size="13">Ideal kinematics, exact current pose; no mask/SLAM/ROS/hardware test. Scale: {} px/m.</text>
</g></svg>'''.format(circles, points(route['route_samples']), points(history),
                      REFERENCE_SPEED_MPS, radius,
                      tracking['completed_laps'], tracking['cross_track_rmse_m'],
                      tracking['cross_track_max_m'], tracking['steady_mean_command_speed_mps'],
                      report['tight_curve_rejections'][0]['radius_m'],
                      report['tight_curve_rejections'][1]['radius_m'], scale)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New result directory; never overwritten')
    args = parser.parse_args(argv)
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error('Refusing to overwrite existing evidence directory: ' + str(output))
    report, route, history = run_demo()
    output.mkdir(parents=True, exist_ok=False)
    with (output / 'metrics.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')
    with (output / 'tracking.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(history[0]))
        writer.writeheader()
        writer.writerows(history)
    with (output / 'preview.svg').open('x', encoding='utf-8') as stream:
        stream.write(svg_preview(report, route, history))
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
