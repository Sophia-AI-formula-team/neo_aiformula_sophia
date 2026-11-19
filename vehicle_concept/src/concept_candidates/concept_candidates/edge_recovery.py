"""A forward trajectory proposal relative to a locally observed exterior edge."""
import numpy as np

from concept_candidates.field_controls import (
    DIFFERENTIAL_SLEW, FORWARD_ACCEL, HALF_TREAD, TRACKING_HEADROOM,
    WHEEL_SPEED_FRACTION, forward_steering,
)
from concept_feasibility.response61515 import FIELD_YAW_TAU_S, motor_steering
from concept_road.onboard625 import HALF_TRACK

MARGIN_M = .5  # Desired turn-away space, in addition to wheel-centre offset.
EDGE_SOURCES = ('paint_material_edge', 'connected_material_edge')
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


def preview_steering(edge, desired, z, x, heading, speed, yaw, previous, drive, dt):
    """Turn back while the currently observed verge is still reachable.

    The measured gyro continues rotating the body while the motor changes
    response. Project that heading only as far as this camera's edge support.
    If continuing on this heading consumes the turn-away space, release the
    outward wheel differential and request an inward turn. The ordinary grip
    and wheel-speed limits apply during both the response and the rollout.
    """
    if edge is None or len(edge) < 4:
        return forward_steering(desired, drive, speed, previous, dt)
    side, angle, normal_distance, end = edge
    z, x, heading, speed, yaw = map(np.asarray, (z, x, heading, speed, yaw))
    distance = side*(normal_distance + np.sin(angle)*z - np.cos(angle)*x)
    predicted_heading = heading + FIELD_YAW_TAU_S*yaw - angle
    outward = side*np.sin(predicted_heading)
    reach = np.maximum(0., end-z)
    reserve = HALF_TRACK + MARGIN_M
    predicted_margin = distance - reach*outward
    conflict = (reach > .25) & (outward > 0.) & (predicted_margin < reserve)
    urgency = np.where(conflict, np.clip((reserve-predicted_margin) / MARGIN_M, 0., 1.), 0.)
    # Turning parallel removes the closing velocity. A near wheel also needs
    # inward displacement to recover the available turn-away margin.
    inside = np.maximum(0., 2.*side*predicted_heading + .35*np.maximum(0., reserve-distance))
    desired = np.where(conflict, -side*np.maximum(-side*desired, inside), desired)
    previous = np.where(conflict & (side*np.asarray(previous) > 0.), 0., previous)
    return forward_steering(desired, drive, speed, previous, dt, edge_urgency=urgency)


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
    steer = preview_steering(edge, desired, z, x, heading, speed, yaw, applied_steer, drive, dt)
    return drive, steer
