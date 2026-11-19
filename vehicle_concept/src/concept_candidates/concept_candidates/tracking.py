"""Tracking laws used inside the candidate rollouts, before feasibility and ranking.

The 11/19 installed neo controller uses ROS left-positive angles. Concept plans
in forward/right coordinates. Convert at the mathematical boundary, including
the installed steering bias and sine guards, not after the path is checked.
"""
import math

import numpy as np


MODES = ('pure_pursuit', 'lya')


def validate_mode(mode):
    if mode not in MODES:
        raise ValueError("tracking_controller must be pure_pursuit or lya")
    return mode


def first_lya_depth(near, far):
    """The car samples points 0, 2, 4 on its 0.5 m forward-depth grid."""
    if near is None or far is None:
        return None
    first = math.ceil(max(0., near) / .5) * .5
    return first if first + 2. <= far else None


def _lateral_at(depth, look, aa, bb, slope):
    q = np.minimum(depth, look)
    return aa*q**3 + bb*q**2 + np.maximum(depth-look, 0)*slope


def neo_lya(distance, odom_dx, alpha, beta, omega_t, reference_speed):
    """Installed connected_omegat_global.py, SHA256 76cc9931… (ROS convention)."""
    sa = np.sin(alpha)
    sb = np.sin(beta)
    sa = np.where(np.abs(sa) < 1e-6, 1e-6, sa)
    sb = np.where(np.abs(sb) < 1e-6, 1e-6, sb)
    half_sine = np.sin(2*alpha)/2
    r = distance + .001
    v = (reference_speed*np.cos(beta) + .0017*odom_dx)*np.cos(alpha)
    w = (-.04 + .37*sa + (.8/sa)*(sa/(.8*r) + sb/(40*r))*
         (half_sine*np.cos(beta) - sb)*reference_speed
         - (omega_t*sb/40)*(.8/sa) + (.8/sa)*.0017*half_sine*(sa/.8 + sb/40))
    return np.clip(v, .8, 4.), np.clip(w, -.45, .45)


def tracking_request(z, x, heading, speed, yaw, commands, slope, aa, bb, look, params,
                     first_depth, capture=None):
    """First iteration optionally retains the exact points used before motor constraints."""
    if params.get('tracking_controller', 'pure_pursuit') == 'pure_pursuit':
        pursuit = z + np.clip(1. + .7*speed, 1.2, 3.)
        reference = _lateral_at(pursuit, look, aa, bb, slope)
        dz, dx = pursuit-z, reference-x
        lateral = dx*np.cos(heading) - dz*np.sin(heading)
        desired = (params['steer_gain']*np.maximum(speed, .5)*2*lateral /
                   (dz*dz+dx*dx+.01) - params['yaw_damping']*yaw)
        if capture is not None:
            capture.update(points=((pursuit, reference),), requested=desired, vref=commands)
        return commands, desired

    if first_depth is None:
        # No valid current ABC triple. A failed observation may still use
        # an independently checked recovery, but never an assumed straight ABC.
        return np.zeros_like(commands), np.zeros_like(commands)
    az = z + first_depth
    ax = _lateral_at(az, look, aa, bb, slope)
    bx = _lateral_at(az+1., look, aa, bb, slope)
    cx = _lateral_at(az+2., look, aa, bb, slope)
    # ROS Pose2D uses y left. A is the target, AB its orientation; the
    # archived withoutkalman_0312 supplies (heading_BC-heading_AB)/0.1.
    ab = np.arctan2(ax-bx, 1.)
    bc = np.arctan2(bx-cx, 1.)
    omega_t = (bc-ab)/.1
    theta = np.arctan2(x-ax, az-z)
    alpha, beta = theta+heading, theta-ab
    alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
    beta = np.arctan2(np.sin(beta), np.cos(beta))
    # Installed linear feedback uses odometry-axis dx, not Euclidean r.
    # Only the wheel/gyro-integrated rotation since local start is needed.
    odom_yaw = params['lya_odom_heading_right_rad']
    odom_dx = np.cos(odom_yaw)*(az-z) - np.sin(odom_yaw)*(ax-x)
    v, w = neo_lya(np.hypot(az-z, ax-x), odom_dx, alpha, beta, omega_t, commands)
    # Neo's 0.8 m/s floor belongs to free-running LYA. Concept's candidate
    # remains a hard speed cap, including explicit zero and slow launch slots.
    drive = np.minimum(commands, v)
    if capture is not None:
        capture.update(points=((az, ax), (az+1., bx), (az+2., cx)), requested=-w, vref=commands,
                       heading=-ab, omega=-omega_t)
    return drive, -w


def controller_record(mode, control=None, owner='unavailable', frame=None, index=None):
    record = dict(mode=mode, owner=owner, active=False, points_zx_m=[], target_zx_m=None,
                  requested_yaw_right_radps=None, checked_yaw_right_radps=None, checked_speed_mps=None,
                  reference_speed_mps=None, target_heading_right_rad=None, reference_yaw_right_radps=None)
    if control is not None:
        record.update(checked_speed_mps=float(control[0]), checked_yaw_right_radps=float(control[1]))
    if owner != 'candidate' or frame is None or index is None:
        return record
    data, indices = frame
    j = indices[index]
    if 'edge_override' in data and data['edge_override'][j]:
        record['owner'] = 'edge_recovery'
        return record
    if 'points' not in data:
        record['owner'] = 'unavailable'
        return record
    points = [[float(z[j]), float(x[j])] for z, x in data['points']]
    record.update(active=True, points_zx_m=points, target_zx_m=points[0],
                  requested_yaw_right_radps=float(data['requested'][j]), reference_speed_mps=float(data['vref'][j]))
    if 'heading' in data:
        record.update(target_heading_right_rad=float(data['heading'][j]), reference_yaw_right_radps=float(data['omega'][j]))
    return record
