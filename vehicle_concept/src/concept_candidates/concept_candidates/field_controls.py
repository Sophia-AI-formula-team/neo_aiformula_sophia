"""Field candidate limits, applied before any path or braking branch is evaluated."""
import numpy as np

from concept_feasibility.response61515 import motor_steering, physical_applied_command

# Initial commissioning limits. The turn ceiling is below the ±0.7 rad/s
# manual plateaus recorded by VectorNav on 2026-11-19.
FORWARD_ACCEL = 1.0             # m/s², command ramp
TRACKING_HEADROOM = .5          # m/s ahead of measured wheel speed
MAX_YAW = .65                   # body rad/s
LATERAL_ACCEL = 1.0             # m/s²
# 16:35 Nov 19, forward/manual and valid 0x711 wheel feedback. Among
# 2,406 overlapping intervals of 0.20 to 0.24 s, the 95th percentile change
# in right-minus-left speed was 30 RPM. The stock converter gives about
# 59 RPM of wheel differential per unit here (0.5 per 0.2 s).
OUTWARD_RELEASE_SLEW = 2.5
DIFFERENTIAL_SLEW = 1.5         # rad/s² at the legacy differential input
HALF_TREAD = .30                # metres
WHEEL_SPEED_FRACTION = .5       # |turn wheel speed| / straight wheel speed


def forward_ceiling(measured, dt, pace):
    applied = max(0., physical_applied_command()[0])
    return min(pace, applied + FORWARD_ACCEL * min(dt, .2),
               max(0., measured) + TRACKING_HEADROOM)


def forward_control(desired, drive, measured, previous, dt, *, release_outward=None, previous_drive=None):
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
    if release_outward is not None:
        # Only move faster while withdrawing a previous outward differential.
        # After zero the ordinary inward build-up limit applies.
        relaxed = OUTWARD_RELEASE_SLEW * min(dt, .2)
        release = np.minimum(relaxed, abs(last)) + np.maximum(0., delta*(1.-abs(last)/relaxed))
        delta = np.where(release_outward, release, delta)
    wheel_limit = drive * WHEEL_SPEED_FRACTION / HALF_TREAD
    if previous_drive is not None:
        # The target combination may need less wheel differential simply
        # because it asks for less forward speed. Limit that desired pair
        # before applying the differential ramp. While the wheels unwind,
        # keep just enough common speed; an explicit zero still brakes now.
        requested = np.clip(requested, -wheel_limit, wheel_limit)
        raw = np.clip(requested, last-delta, last+delta)
        floor = np.minimum(previous_drive, np.abs(raw)*HALF_TREAD/WHEEL_SPEED_FRACTION)
        drive = np.where(drive > 0., np.maximum(drive, floor), 0.)
        wheel_limit = drive*WHEEL_SPEED_FRACTION/HALF_TREAD
    else:
        raw = np.clip(requested, last-delta, last+delta)
    raw = np.clip(raw, -wheel_limit, wheel_limit)
    # Additionally respect the body-yaw ceiling when speed changes.
    raw = np.clip(raw, motor_steering(-limit, physical=True), motor_steering(limit, physical=True))
    pos = raw >= 0
    L = np.where(pos, 1.53908994, 1.68283261)
    k = np.where(pos, 3.15496243, 2.91627673)
    offset = np.where(pos, 3.36664054, -3.22124235)
    bias = np.where(pos, -.0621119, 1.6954783)
    command = L / (1. + np.exp(-k*(raw-offset))) - bias
    return drive, np.where((drive > 0.) & (np.abs(raw) > 1e-9), -command, 0.)


def forward_steering(desired, drive, measured, previous, dt, *, release_outward=None):
    return forward_control(desired, drive, measured, previous, dt, release_outward=release_outward)[1]
