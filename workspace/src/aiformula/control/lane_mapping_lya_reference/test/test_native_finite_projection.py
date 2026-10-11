"""Bit-exact native finite-projection contract and whole-domain integration.

The public Python projection_bounds is the independent arithmetic oracle: it
does not call the private native dispatcher. No estimator, recorded data,
relaxed gates or test clock is used. An absent/new-API-incompatible binary is
an explicit skip in source-only runs and a collection failure in native CI.
"""
import math
import os
from pathlib import Path
import random
import struct
import sys
from unittest.mock import patch

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
try:
    import _lane_mapping_lya_native as native
except ImportError:
    if os.environ.get("LANE_REQUIRE_NATIVE") == "1":
        raise
    pytest.skip("native extension is not built", allow_module_level=True)

if (getattr(native, "FINITE_PROJECTION_BOUNDS_API_VERSION", None) != 1
        or not callable(getattr(native, "finite_projection_bounds", None))):
    if os.environ.get("LANE_REQUIRE_NATIVE") == "1":
        raise RuntimeError("required native finite_projection_bounds API version 1 is not installed")
    pytest.skip("installed native extension lacks finite_projection_bounds API version 1",
                allow_module_level=True)

from lane_mapping_lya_reference import finite_support as finite


COSINE = math.cos(math.pi / 6.)
SMALLEST = float.fromhex("0x0.0000000000001p-1022")
MIN_NORMAL = float.fromhex("0x1.0000000000000p-1022")
MAXIMUM = sys.float_info.max


def packed(values):
    return struct.pack("=" + "d" * len(values), *values)


def six_arguments(point, origin, tangent):
    # Deliberately spell out the documented interleaved scalar protocol.
    return (float(point[0]), float(origin[0]), float(tangent[0]),
            float(point[1]), float(origin[1]), float(tangent[1]))


def arithmetic_outcome(function, *args):
    try:
        value = function(*args)
    except (ValueError, OverflowError, TypeError) as error:
        return ("error", type(error), error.args)
    assert type(value) is tuple and len(value) == 2
    assert all(type(item) is float for item in value)
    return ("value", packed(value))


def compare_projection(point, origin, tangent):
    before = (packed(point), packed(origin), packed(tangent))
    expected = arithmetic_outcome(finite.projection_bounds, point, origin, tangent)
    actual = arithmetic_outcome(native.finite_projection_bounds,
                                *six_arguments(point, origin, tangent))
    assert actual == expected
    assert before == (packed(point), packed(origin), packed(tangent))
    return expected


def state_bytes(state):
    if state is None:
        return b"none"
    result = packed(state.origin + state.tangent)
    result += struct.pack("=qq", state.source_stamp_ns, len(state.intervals))
    for item in state.intervals:
        result += struct.pack("=ddqqq", item.lower, item.upper, item.frame_votes,
                              item.reliable_votes, item.last_stamp_ns)
    return result


def export_bytes(values):
    result = []
    for value in values:
        assert isinstance(value, np.ndarray)
        assert not value.flags.writeable
        with pytest.raises(ValueError):
            value.setflags(write=True)
        result.append((value.dtype.str, value.shape, value.tobytes()))
    return tuple(result)


def frame_arrays(intervals, origin=(0., 0.), angle=0., reliable=True):
    tangent = np.array((math.cos(angle), math.sin(angle)), dtype=np.float64)
    normal = np.array((-tangent[1], tangent[0]), dtype=np.float64)
    coordinates = np.asarray(intervals, dtype=np.float64).reshape(-1, 2)
    segments = np.asarray(origin, dtype=np.float64) + coordinates[:, :, None] * tangent
    normals = np.broadcast_to(normal, segments.shape).copy()
    flags = (np.full(len(segments), reliable, dtype=bool)
             if isinstance(reliable, bool) else np.asarray(reliable, dtype=bool))
    return segments, normals, flags, normal


def advance(state, intervals, *, stamp, origin=(0., 0.), angle=0.,
            reliable=True, ttl=0, capacity=200000):
    segments, normals, flags, normal = frame_arrays(intervals, origin, angle, reliable)
    before = (segments.tobytes(), normals.tobytes(), flags.tobytes(), state_bytes(state))
    result = finite.advance_domain(
        state, origin=origin, current_frame_normal=normal,
        segments=segments, normals=normals, reliable=flags, stamp_ns=stamp,
        minimum_frame_votes=5, minimum_reliable_votes=1, candidate_ttl_ns=ttl,
        normal_cosine=COSINE, max_intervals=capacity)
    assert before == (segments.tobytes(), normals.tobytes(), flags.tobytes(), state_bytes(state))
    return result, normal


class RecordingKernel:
    def __init__(self):
        self.arguments = []

    def __call__(self, *values):
        assert len(values) == 6 and all(type(value) is float for value in values)
        self.arguments.append(packed(values))
        return native.finite_projection_bounds(*values)


def test_required_api_is_real_and_private_dispatch_is_selected():
    assert native.FINITE_PROJECTION_BOUNDS_API_VERSION == 1
    assert finite._FINITE_NATIVE is native
    assert finite._NATIVE_PROJECTION is native.finite_projection_bounds
    assert compare_projection((2., 3.), (1., 1.), (1., 0.))[0] == "value"


@pytest.mark.parametrize("point,origin,tangent", [
    ((0., 0.), (0., 0.), (1., 0.)),
    ((-0., -0.), (+0., +0.), (-0., +0.)),
    ((+0., -0.), (-0., +0.), (-1., -0.)),
    ((3., -2.), (1., 4.), (1., 0.)),
    ((3., -2.), (1., 4.), (0., -1.)),
    ((3., -2.), (1., 4.), (-1., -0.)),
    ((1., 1.), (0., 0.), (1., -1.)),
    ((2.**52 + 1., -(2.**52)), (2.**52, -(2.**52) + 1.), (.5, .5)),
    ((1.e7, -1.e7), (np.nextafter(1.e7, -math.inf),
                          np.nextafter(-1.e7, math.inf)), (.6, -.8)),
    ((SMALLEST, -SMALLEST), (0., 0.), (1., 1.)),
    ((MIN_NORMAL, -MIN_NORMAL), (SMALLEST, -SMALLEST), (.5, -.5)),
    ((SMALLEST, SMALLEST), (0., 0.), (SMALLEST, -SMALLEST)),
    ((-SMALLEST, -SMALLEST), (0., 0.), (-0., +0.)),
    ((1., -1.), (0., 0.), (SMALLEST, SMALLEST)),
    ((MAXIMUM / 16., -MAXIMUM / 16.), (0., 0.), (.25, .25)),
    ((-MAXIMUM / 16., MAXIMUM / 16.), (0., 0.), (-.25, -.25)),
], ids=["zero", "all-signed-zero", "signed-zero-axis", "x-axis", "negative-y-axis",
        "negative-x-axis", "cancel", "large-translation-cancel", "world-ulp",
        "subnormal-point", "normal-subnormal-border", "underflow-product",
        "negative-underflow-zero", "subnormal-tangent", "large-finite", "large-negative"])
def test_edge_cases_match_public_python_bytes(point, origin, tangent):
    compare_projection(point, origin, tangent)


@pytest.mark.parametrize("point,origin,tangent", [
    ((MAXIMUM, 0.), (-MAXIMUM, 0.), (1., 0.)),
    ((MAXIMUM, 0.), (0., 0.), (1., 0.)),
    ((MAXIMUM, 0.), (0., 0.), (0., 0.)),
    ((MAXIMUM / 4., 0.), (0., 0.), (8., 0.)),
    ((MAXIMUM / 2., MAXIMUM / 2.), (0., 0.), (1., 1.)),
    ((-MAXIMUM, 0.), (0., 0.), (-1., 0.)),
], ids=["subtraction-overflow", "outward-overflow", "zero-tangent-still-subtracts",
        "product-overflow", "sum-overflow", "negative-outward-overflow"])
def test_finite_arithmetic_overflow_has_same_error(point, origin, tangent):
    outcome = compare_projection(point, origin, tangent)
    assert outcome[0] == "error" and outcome[1] is ValueError


def test_all_signed_zero_combinations_are_byte_exact():
    for bits in range(64):
        values = [math.copysign(0., -1. if bits & (1 << index) else 1.) for index in range(6)]
        compare_projection((values[0], values[3]), (values[1], values[4]),
                           (values[2], values[5]))


def test_fixed_seed_finite_ieee_patterns_match_success_or_failure_exactly():
    rng = random.Random(90218411)
    successes = errors = 0
    for _ in range(768):
        scalars = []
        while len(scalars) < 6:
            value = struct.unpack("=d", struct.pack("=Q", rng.getrandbits(64)))[0]
            if math.isfinite(value):
                scalars.append(value)
        outcome = compare_projection((scalars[0], scalars[3]),
                                     (scalars[1], scalars[4]), (scalars[2], scalars[5]))
        successes += outcome[0] == "value"
        errors += outcome[0] == "error"
    assert successes > 0 and errors > 0


def test_fixed_seed_realistic_coordinates_and_near_cancellation():
    rng = random.Random(571204)
    for _ in range(256):
        origin = tuple(rng.uniform(-1.e7, 1.e7) for _ in range(2))
        point = tuple(origin[index] + rng.uniform(-1., 1.) for index in range(2))
        angle = rng.uniform(-math.pi, math.pi)
        compare_projection(point, origin, (math.cos(angle), math.sin(angle)))
    for exponent in (-1020, -500, -20, 0, 20, 500, 1000):
        scale = math.ldexp(1., exponent)
        for direction in (-math.inf, math.inf):
            compare_projection((scale, np.nextafter(scale, direction)),
                               (0., 0.), (1., -1.))


@pytest.mark.parametrize("position", range(6))
@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_nonfinite_native_arguments_rejected_even_zero_other_term(position, bad):
    values = [0., 0., 0., 0., 0., 0.]
    values[position] = bad
    with pytest.raises(ValueError, match="finite projection requires finite inputs"):
        native.finite_projection_bounds(*values)


@pytest.mark.parametrize("size", [0, 1, 5, 7])
def test_wrong_arity_rejected(size):
    with pytest.raises(TypeError):
        native.finite_projection_bounds(*([0.] * size))


@pytest.mark.parametrize("bad", [None, "0", b"0", object(), complex(1., 0.)])
def test_non_numeric_scalar_rejected(bad):
    with pytest.raises(TypeError):
        native.finite_projection_bounds(bad, 0., 1., 0., 0., 0.)


def test_unrepresentable_python_integer_rejected():
    with pytest.raises(OverflowError):
        native.finite_projection_bounds(10**10000, 0., 1., 0., 0., 0.)


def test_public_python_oracle_does_not_dispatch_native():
    def forbidden(*args):
        raise AssertionError("public independent oracle called native dispatcher")
    expected = arithmetic_outcome(finite.projection_bounds, (1., 2.), (0., 0.), (.6, .8))
    with patch.object(finite, "_NATIVE_PROJECTION", forbidden):
        assert arithmetic_outcome(finite.projection_bounds, (1., 2.), (0., 0.), (.6, .8)) == expected


def test_private_dispatch_calls_native_and_none_uses_public_oracle():
    kernel = RecordingKernel()
    point, origin, tangent = (1.25, -2.5), (.125, -.25), (.6, .8)
    with patch.object(finite, "_NATIVE_PROJECTION", kernel):
        actual = finite._validated_projection_bounds(point, origin, tangent)
    assert kernel.arguments == [packed(six_arguments(point, origin, tangent))]
    with patch.object(finite, "_NATIVE_PROJECTION", None):
        expected = finite._validated_projection_bounds(point, origin, tangent)
    assert packed(actual) == packed(expected)


def sequence(kernel, origin, angle):
    # Explicit true joints, reversal/duplicates, one gap, a trusted subinterval,
    # repeated frames, an empty frame and TTL expiry. No spatial padding.
    frames = [
        (1, ((0., .5), (.5, 1.), (2., 2.5)), [False, False, False]),
        (2, ((0., .5), (.5, 1.), (.5, 0.), (2., 2.5)), [True, False, False, True]),
        (3, ((0., 1.), (2., 2.5)), [True, True]),
        (4, ((0., 1.), (2., 2.5)), [True, True]),
        (5, ((0., 1.), (2., 2.5)), [True, True]),
        (6, ((0., 1.), (2., 2.5), (3., 3.5)), [True, True, False]),
        (7, ((0., 1.), (2., 2.5)), [False, False]),
        (20, (), []),
    ]
    result, state, nonempty_exports = [], None, 0
    with patch.object(finite, "_NATIVE_PROJECTION", kernel):
        for stamp, segments, reliable in frames:
            state, normal = advance(state, segments, stamp=stamp, origin=origin,
                                    angle=angle, reliable=reliable, ttl=10)
            # Export from a shifted/rotated current model, not old bounds about
            # its moving mean. Both modes must retain the same creation chart.
            export_normal = (-math.sin(angle + .002), math.cos(angle + .002))
            export_point = np.asarray(origin) + np.asarray(normal) * .002
            exported = finite.certified_segments(state, export_point, export_normal, COSINE, 5, 1)
            nonempty_exports += bool(len(exported[0]))
            result.append((state_bytes(state), export_bytes(exported)))
    assert nonempty_exports >= 3
    return result


@pytest.mark.parametrize("origin,angle", [
    ((0., 0.), 0.), ((123.5, -67.25), .37), ((-9000., 1234.), 1.9),
    ((1.e6, -1.e6), -.71),
])
def test_whole_advance_domain_and_export_match_bytes(origin, angle):
    kernel = RecordingKernel()
    expected = sequence(None, origin, angle)
    actual = sequence(kernel, origin, angle)
    assert actual == expected
    assert len(kernel.arguments) > 0


def test_whole_empty_and_opposite_normals_do_not_quietly_invoke_kernel():
    kernel = RecordingKernel()
    with patch.object(finite, "_NATIVE_PROJECTION", kernel):
        state, _ = advance(None, (), stamp=1)
        before = state_bytes(state)
        segments, normals, reliable, _ = frame_arrays(((0., 1.),))
        # The creation chart remains fixed; truly opposite observations reject.
        result = finite.advance_domain(
            state, origin=(0., 0.), current_frame_normal=(0., 1.),
            segments=segments, normals=-normals, reliable=reliable, stamp_ns=2,
            minimum_frame_votes=5, minimum_reliable_votes=1, candidate_ttl_ns=0,
            normal_cosine=COSINE, max_intervals=100)
    assert state_bytes(state) == before
    assert result.intervals == ()
    assert kernel.arguments == []


def test_capacity_rejection_has_same_message_and_preserves_previous_state():
    old, _ = advance(None, ((0., 1.),), stamp=1)
    before = state_bytes(old)
    outcomes = []
    kernel = RecordingKernel()
    for selected in (None, kernel):
        with patch.object(finite, "_NATIVE_PROJECTION", selected):
            with pytest.raises(ValueError) as caught:
                advance(old, ((2., 3.), (4., 5.)), stamp=2, capacity=1)
            outcomes.append((type(caught.value), caught.value.args))
        assert state_bytes(old) == before
    assert outcomes[0] == outcomes[1]
    assert kernel.arguments


def test_duplicate_source_rejection_precedes_kernel_and_preserves_state():
    old, _ = advance(None, ((0., 1.),), stamp=10)
    before = state_bytes(old)
    kernel = RecordingKernel()
    with patch.object(finite, "_NATIVE_PROJECTION", kernel):
        with pytest.raises(ValueError, match="source stamp must strictly increase"):
            advance(old, ((0., 1.),), stamp=10)
    assert not kernel.arguments
    assert state_bytes(old) == before


def test_whole_sequence_is_independent_of_numpy_error_mode():
    kernel = RecordingKernel()
    with np.errstate(all="raise"):
        expected = sequence(None, (0., 0.), 0.)
        actual = sequence(kernel, (0., 0.), 0.)
    assert actual == expected
    assert kernel.arguments

