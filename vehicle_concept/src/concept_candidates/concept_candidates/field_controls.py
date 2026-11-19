"""Field candidate limits, applied before any path or braking branch is evaluated."""
import numpy as np

from concept_feasibility.response61515 import motor_steering, physical_applied_command

# Initial commissioning limits. The turn ceiling is below the ±0.7 rad/s
# manual plateaus recorded by VectorNav on 2026-11-19.
FORWARD_ACCEL = 1.0             # m/s², command ramp
TRACKING_HEADROOM = .5          # m/s ahead of measured wheel speed
MAX_YAW = .65                   # body rad/s
LATERAL_ACCEL = 1.0             # m/s²
DIFFERENTIAL_SLEW = 1.5         # rad/s² at the legacy differential input
HALF_TREAD = .30                # metres
WHEEL_SPEED_FRACTION = .5       # |turn wheel speed| / straight wheel speed


def forward_ceiling(measured, dt, pace):
    applied = max(0., physical_applied_command()[0])
    return min(pace, applied + FORWARD_ACCEL * min(dt, .2),
               max(0., measured) + TRACKING_HEADROOM)


def forward_steering(desired, drive, measured, previous, dt):
    """Choose a real motor input whose actual wheel differential rises smoothly.

    The stock inverse sigmoid has disjoint fitted branches at zero. Project
    in its differential coordinates, then map back to the unchanged converter.
    All inputs/outputs at the planner boundary are clockwise-positive.
    """
    drive = np.asarray(drive, dtype=float)
    limit = np.minimum(MAX_YAW, LATERAL_ACCEL / np.maximum(.01, np.maximum(measured, drive)))
    ros = -np.clip(desired, -limit, limit)
    requested = motor_steering(ros, physical=True)
    # A small request below the fitted branch's intercept can invert the
    # wheel differential. Select straight for an unrepresentable request.
    requested = np.where(requested * ros > 0., requested, 0.)
    last = motor_steering(-np.asarray(previous, dtype=float), physical=True)
    delta = DIFFERENTIAL_SLEW * min(dt, .2)
    wheel_limit = drive * WHEEL_SPEED_FRACTION / HALF_TREAD
    raw = np.clip(np.clip(requested, last-delta, last+delta), -wheel_limit, wheel_limit)
    # Additionally respect the body-yaw ceiling when speed changes.
    raw = np.clip(raw, motor_steering(-limit, physical=True), motor_steering(limit, physical=True))
    pos = raw >= 0
    L = np.where(pos, 1.53908994, 1.68283261)
    k = np.where(pos, 3.15496243, 2.91627673)
    offset = np.where(pos, 3.36664054, -3.22124235)
    bias = np.where(pos, -.0621119, 1.6954783)
    command = L / (1. + np.exp(-k*(raw-offset))) - bias
    return np.where((drive > 0.) & (np.abs(raw) > 1e-9), -command, 0.)
