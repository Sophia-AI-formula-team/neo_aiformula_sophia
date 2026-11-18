"""Runtime source extracted from the accepted V6.26 freeze: research/learned_racing/unity_laps/v3_model.py. """

from dataclasses import dataclass

import numpy as np

BODY_HALF_WIDTH = .38

BODY_FRONT, BODY_REAR = .28, .88

DT = .20

DEFAULT_PARAMETERS = dict(pace=4.45, lookahead=1.1, steer_gain=2.0,
                         yaw_damping=.65, slew=1.8, clearance=.18,
                         uncertainty=1.8, smoothness=.12, response=.35)

@dataclass(frozen=True)
class Observation:
    # Rows: forward distance, visible left/right bounds, lateral uncertainty.
    road: np.ndarray
    # Rows: forward, right, estimated forward/right velocity, radius, uncertainty.
    objects: np.ndarray
    speed: float
    yaw: float
    dt: float = DT
