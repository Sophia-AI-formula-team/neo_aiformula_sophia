"""Compare exact result bits, warnings and errors with the original operation."""
import math
import struct
import warnings
from unittest.mock import patch

import numpy as np
import pytest

from lane_mapping_lya_reference import curve_rounding as candidate


def literal(value, direction):
    return float(np.nextafter(np.float64(value), direction))


def outcome(function, value, direction):
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        try:
            result = ("value", struct.pack("d", function(value, direction)))
        except Exception as error:
            result = (type(error), str(error))
    return result, [(item.category, str(item.message)) for item in caught]


@pytest.mark.parametrize("policy", ["ignore", "warn", "raise"])
def test_binary64_extremes_and_normal_samples(policy):
    bits = [0, 1, 2, 0x000fffffffffffff, 0x0010000000000000,
            0x0020000000000000, 0x3ff0000000000000, 0x7fefffffffffffff,
            0x7ff0000000000000, 0x7ff8000000000042]
    rng = np.random.default_rng(17331)
    bits.extend(map(int, rng.integers(0x0010000000000000,
        0x7fefffffffffffff, size=1024, dtype=np.uint64)))
    with np.errstate(all=policy):
        for raw in bits:
            for sign in (0, 1 << 63):
                value = struct.unpack("d", struct.pack("Q", raw | sign))[0]
                for direction in (-math.inf, math.inf):
                    assert outcome(candidate.step, value, direction) == outcome(literal, value, direction)


@pytest.mark.parametrize("value", [np.float64(1.), np.float32(1.), np.longdouble(1.),
                                   1, True, "1.0", None, complex(1., 1.)])
def test_nonbuiltin_inputs_use_literal_conversion(value):
    assert outcome(candidate.step, value, math.inf) == outcome(literal, value, math.inf)


@pytest.mark.parametrize("direction", [0., -0., 1., float("nan"), np.float32(np.inf)])
def test_other_directions_keep_original_behavior(direction):
    assert outcome(candidate.step, 1., direction) == outcome(literal, 1., direction)


def test_python38_fallback_and_rebound_numpy_operation():
    with patch.object(candidate, "_MATH_NEXTAFTER", None):
        assert outcome(candidate.step, 1., math.inf) == outcome(literal, 1., math.inf)
    original = np.nextafter
    with patch.object(np, "nextafter", wraps=original) as recorder:
        candidate.step(1., math.inf)
        assert recorder.call_count == 1


def test_standard_normal_path_does_not_call_numpy_ufunc():
    if candidate._MATH_NEXTAFTER is None:
        pytest.skip("stdlib nextafter unavailable; original NumPy path required")
    with patch.object(candidate, "_MATH_NEXTAFTER", wraps=candidate._MATH_NEXTAFTER) as recorder:
        candidate.step(1., math.inf)
        assert recorder.call_count == 1
