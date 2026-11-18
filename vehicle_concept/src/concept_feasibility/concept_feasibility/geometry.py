"""Runtime source extracted from the accepted V6.26 freeze: experiments/v614_driving/geometry.py. """

import numpy as np
from concept_road.onboard625 import HALF_TRACK, HALF_TREAD, tread_clearance

try:
    from . import _tread
except ImportError:  # Unbuilt source checkout.
    _tread = None

_OFFSETS = np.concatenate([side + np.linspace(-HALF_TREAD, HALF_TREAD, 5) for side in (-HALF_TRACK, HALF_TRACK)])


def road_clearance(road, z, x, heading, uncertainty_scale=1.):
    if _tread is None or np.ndim(z) != 1 or np.shape(z) != np.shape(x) or np.shape(x) != np.shape(heading):
        return tread_clearance(road, z, x, heading, uncertainty_scale)
    return _tread.clearance(road, z, x, heading, uncertainty_scale, _OFFSETS)
