"""Scalar outward steps with the original NumPy boundary/error behavior.

Only ordinary builtin binary64 values strictly inside the normal finite range
use the stdlib operation. Neither endpoint can then underflow or overflow.
Zeros, subnormals, exceptional/wider values, changed NumPy bindings and Python
3.8 without math.nextafter take the literal original operation.
"""
import math
import sys

import numpy as np


_NUMPY_NEXTAFTER = np.nextafter
_NUMPY_FLOAT64 = np.float64
_MATH_NEXTAFTER = getattr(math, "nextafter", None)
_MINIMUM = sys.float_info.min * 2.
_MAXIMUM = sys.float_info.max / 2.


def step(value, direction):
    if (_MATH_NEXTAFTER is not None
            and np.nextafter is _NUMPY_NEXTAFTER and np.float64 is _NUMPY_FLOAT64
            and type(value) is float and type(direction) is float
            and direction in (-math.inf, math.inf)
            and _MINIMUM < abs(value) < _MAXIMUM):
        return _MATH_NEXTAFTER(value, direction)
    return float(np.nextafter(np.float64(value), direction))
