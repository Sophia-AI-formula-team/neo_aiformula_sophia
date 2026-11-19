"""A forward trajectory proposal relative to a locally observed exterior edge."""
import numpy as np

from concept_candidates.field_controls import (
    DIFFERENTIAL_SLEW, OUTWARD_RELEASE_SLEW, FORWARD_ACCEL, HALF_TREAD, TRACKING_HEADROOM,
    WHEEL_SPEED_FRACTION, forward_control,
)
from concept_feasibility.response61515 import FIELD_YAW_TAU_S, motor_steering
from concept_road.onboard625 import HALF_TRACK

MARGIN_M = .5  # Desired turn-away space, in addition to wheel-centre offset.
EDGE_SOURCES = ('paint_material_edge', 'connected_material_edge', 'two_lane_material_edge')
EDGE_RESIDUAL_M = .15


def _supported_lines(z, x, side):
    """Fit measured local stretches. Competing stretches are compared by the caller."""
    if len(z) < 5 or z[0] > 3. or np.ptp(z) < 1.:
        return []
    # An isolated alternate marking can spoil a whole scan. Its neighbours
    # must jointly support interpolation. Never bridge two missing scanlines.
    linear = x[:-2] + (x[2:]-x[:-2])*(z[1:-1]-z[:-2])/(z[2:]-z[:-2])
    back, ahead = x[1:-1]-x[:-2], x[1:-1]-x[2:]
    spike = ((abs(x[1:-1]-linear) > 2*EDGE_RESIDUAL_M) &
             (back*ahead > 0) & (np.minimum(abs(back), abs(ahead)) > EDGE_RESIDUAL_M) &
             (z[2:]-z[:-2] <= .52))
    indices = np.flatnonzero(~np.r_[False, spike, False])
    z, x = z[indices], x[indices]
    if len(z) < 5:
        return []
    # Test the whole coherent stretch and its subintervals in one small array
    # (at most 21 measured sections out to 6 m). A real bend farther up the
    # road does not invalidate its near tangent.
    lo, hi = np.triu_indices(len(z), k=4)
    gaps = np.r_[0, np.cumsum(np.diff(z) > .52)]
    count = hi-lo+1
    eligible = ((z[lo] <= 3.) & (z[hi]-z[lo] >= 1.) & (gaps[hi] == gaps[lo]) &
                (count >= .75*(indices[hi]-indices[lo]+1)))
    lo, hi, count = lo[eligible], hi[eligible], count[eligible]
    if not len(lo):
        return []
    zs = np.arange(len(z))
    mask = (zs >= lo[:, None]) & (zs <= hi[:, None])
    mean_z, mean_x = (mask @ z)/count, (mask @ x)/count
    dz = (z-mean_z[:, None])*mask
    slope = np.sum(dz*(x-mean_x[:, None]), axis=1)/np.sum(dz*dz, axis=1)
    intercept = mean_x-slope*mean_z
    residual = np.max(abs(x - (slope[:, None]*z + intercept[:, None]))*mask, axis=1)
    valid = np.flatnonzero((side*intercept > 0) & (residual <= EDGE_RESIDUAL_M))
    # Keep genuine competitors for the ambiguity test.
    return [(int(count[j]), float(z[lo[j]]), float(z[hi[j]]), float(slope[j]), float(intercept[j]))
            for j in valid]


def _unambiguous_line(lines):
    if not lines:
        return None
    # Prefer the longest measured consensus, and the closer one on a tie.
    best = max(lines, key=lambda v: (v[0], -v[1], v[2]-v[1]))
    n, start, end, slope, intercept = best
    for count, lo, hi, other_slope, other_intercept in lines:
        if count < .8*n:
            continue
        left, right = max(start, lo), min(end, hi)
        if right < left:
            # Check continuity where two supported stretches would meet.
            left = right = .5*(left+right)
        error = max(abs((slope-other_slope)*z + intercept-other_intercept) for z in (left, right))
        if error > 2*EDGE_RESIDUAL_M:
            return None
    return slope, intercept, end


def fit_edge(road, observed_bounds, *, include_extent=False):
    """Return (side, tangent angle, signed distance) in the current camera frame.

    A negative side is the left exterior. A road mask or a lane divider alone
    cannot supply an exterior. Use only a contiguous, locally straight
    stretch of measured verge. The caller retains the actual road for scoring.
    """
    road = np.asarray(road)
    if len(road) < 3 or not len(observed_bounds):
        return None
    fits = []
    for side, col in ((-1, 1), (1, 2)):
        depths = np.array([z for z, left, right in observed_bounds
                           if (left, right)[col-1] in EDGE_SOURCES
                           and max(.0, road[0, 0]-.006) < z <= min(6., road[-1, 0]+.006)])
        if len(depths) < 5:
            continue
        # Bounds are logged at 1 cm precision on the observer's 25 cm grid.
        groups = np.split(depths, np.flatnonzero(np.diff(depths) > .52)+1)
        lines = []
        for z in groups:
            x = np.interp(z, road[:, 0], road[:, col])
            lines.extend(_supported_lines(z, x, side))
        line = _unambiguous_line(lines)
        if line is not None:
            slope, intercept, end = line
            angle = np.arctan(slope)
            distance = intercept*np.cos(angle)
            fit = (side, float(angle), float(distance))
            fits.append(fit + (float(end),) if include_extent else fit)
    return min(fits, key=lambda edge: abs(edge[2])) if fits else None


def corroborated_previous_edge(edge, road, observed_bounds, speed, yaw, dt):
    """Reuse one previous fit only where this photo shows the same material edge."""
    if edge is None or not 0. < dt <= .4 or len(road) < 3:
        return None
    side, old_angle, old_distance, old_end = edge
    col = 1 if side < 0 else 2
    turn = yaw*dt
    dz = speed*dt*np.sinc(turn/np.pi)
    dx = .5*speed*dt*turn*np.sinc(turn/(2*np.pi))**2
    angle = old_angle - turn
    # n = (-sin(angle), cos(angle)) in (forward, right) coordinates.
    distance = old_distance + np.sin(old_angle)*dz - np.cos(old_angle)*dx
    end_x = (old_distance + old_end*np.sin(old_angle))/np.cos(old_angle)
    end = (old_end-dz)*np.cos(turn) + (end_x-dx)*np.sin(turn)
    if side*distance <= 0.:
        return None
    depths = np.array([row[0] for row in observed_bounds if row[col] in EDGE_SOURCES
                       and road[0, 0] <= row[0] <= min(end, road[-1, 0])])
    if len(depths) < 5:
        return None
    x = np.interp(depths, road[:, 0], road[:, col])
    error = abs(np.cos(angle)*x - np.sin(angle)*depths - distance)
    supported = depths[error <= EDGE_RESIDUAL_M]
    runs = np.split(supported, np.flatnonzero(np.diff(supported) > .52)+1)
    ends = [group[-1] for group in runs if len(group) >= 5 and group[-1]-group[0] >= 1.]
    if not ends:
        return None
    return side, angle, distance, max(ends)


def edge_preview(edge, z, x, heading, yaw, previous):
    """Compute the same measured-edge test used by the rollout and the ROS trace."""
    side, angle, normal_distance, end = edge
    z, x, heading, yaw = map(np.asarray, (z, x, heading, yaw))
    distance = side*(normal_distance + np.sin(angle)*z - np.cos(angle)*x)
    # A previously applied outward differential takes finite time to unwind.
    # Include its triangular heading sweep as well as the measured gyro tail.
    previous = np.asarray(previous)
    outward_steer = np.where(side*previous > 0., previous, 0.)
    release_time = abs(motor_steering(-outward_steer, physical=True))/OUTWARD_RELEASE_SLEW
    predicted_heading = heading + FIELD_YAW_TAU_S*yaw + .5*outward_steer*release_time - angle
    outward = side*np.sin(predicted_heading)
    reach = np.maximum(0., end-z)
    reserve = HALF_TRACK + MARGIN_M
    predicted_margin = distance - reach*outward
    conflict = (reach > .25) & (outward > 0.) & (predicted_margin < reserve)
    urgency = np.where(conflict, np.clip((reserve-predicted_margin) / MARGIN_M, 0., 1.), 0.)
    return predicted_heading, distance, reach, predicted_margin, conflict, urgency


def preview_audit(edge, source, yaw, previous=0.):
    if edge is None:
        return dict(source='no_supported_exterior', side=None, triggered=False)
    angle_error, distance, reach, margin, conflict, urgency = edge_preview(edge, 0., 0., 0., yaw, previous)
    side, angle, normal_distance, end = edge
    return dict(source=source, side='left' if side < 0 else 'right', triggered=bool(conflict),
                normal_distance_m=float(normal_distance), tangent_right_rad=float(angle),
                end_forward_m=float(end), gyro_right_radps=float(yaw), previous_yaw_right_radps=float(previous),
                predicted_relative_heading_rad=float(angle_error),
                projected_center_clearance_m=float(margin), desired_center_clearance_m=HALF_TRACK+MARGIN_M,
                urgency=float(urgency))


def preview_control(edge, desired, z, x, heading, speed, yaw, previous, drive, dt, previous_drive=None):
    """Turn back while the currently observed verge is still reachable.

    The measured gyro continues rotating the body while the motor changes
    response. Project that heading only as far as this camera's edge support.
    If continuing on this heading consumes the turn-away space, unwind the
    actual outward differential before building the inward turn. The grip
    and wheel-speed limits apply during both the response and the rollout.
    """
    if edge is None or len(edge) < 4:
        return forward_control(desired, drive, speed, previous, dt, previous_drive=previous_drive)
    side = edge[0]
    predicted_heading, distance, _, _, conflict, _ = edge_preview(edge, z, x, heading, yaw, previous)
    reserve = HALF_TRACK + MARGIN_M
    # Turning parallel removes the closing velocity. A near wheel also needs
    # inward displacement to recover the available turn-away margin.
    inside = np.maximum(0., 2.*side*predicted_heading + .35*np.maximum(0., reserve-distance))
    desired = np.where(conflict, -side*np.maximum(-side*desired, inside), desired)
    release = conflict & (side*np.asarray(previous) > 0.)
    return forward_control(desired, drive, speed, previous, dt, release_outward=release, previous_drive=previous_drive)


def edge_control(edge, z, x, heading, speed, yaw, applied_drive, applied_steer, dt, pace):
    """One proposal step; positive steer is clockwise. No direct motor output."""
    side, angle, normal_distance = edge[:3]
    relative_heading = np.asarray(heading)-angle
    distance = side*(normal_distance + np.sin(angle)*z - np.cos(angle)*x)
    reserve = HALF_TRACK + MARGIN_M
    desired = -2.*relative_heading - .8*yaw - side*.35*np.maximum(0., reserve-distance)
    outward = side*np.sin(relative_heading)
    # When projected toward the nearby exterior, use just enough forward
    # drive to allow one more step of differential motion. Creeping at a
    # fixed very low speed would clamp the differential and lose the turn.
    dt_step = min(dt, .2)
    applied_diff = np.abs(motor_steering(-np.asarray(applied_steer), physical=True))
    turning_drive = HALF_TREAD/WHEEL_SPEED_FRACTION*(applied_diff + DIFFERENTIAL_SLEW*dt_step)
    approaching = (outward > .09) & (distance < reserve + 5.*outward)
    goal = np.where(approaching, np.minimum(pace, turning_drive), pace)
    drive = np.minimum(goal, np.minimum(applied_drive + FORWARD_ACCEL*dt_step,
                                        np.maximum(speed, 0.) + TRACKING_HEADROOM))
    return preview_control(edge, desired, z, x, heading, speed, yaw, applied_steer, drive, dt,
                           np.minimum(applied_drive, np.minimum(pace, np.maximum(speed, 0.) + TRACKING_HEADROOM)))
