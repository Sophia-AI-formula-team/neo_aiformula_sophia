"""Independent IEEE oracle for both R17 scalar paths; no engine or recording.

The bit-step fallback is exercised directly even when this interpreter provides
math.nextafter. This does not claim to execute Foxy's Python 3.8 interpreter.
"""
import ast
from dataclasses import fields, is_dataclass
import math
from pathlib import Path
import random
import runpy
import struct
import unittest
from unittest.mock import patch

import numpy as np

from lane_mapping_lya_reference import finite_support as fs


def from_bits(value):
    return struct.unpack('>d', struct.pack('>Q', value))[0]


def bits(value):
    return struct.pack('>d', float(value))


def numpy_oracle(value, direction):
    # Special-value overflow/invalid flags are expected for this scalar oracle.
    with np.errstate(all='ignore'):
        return float(np.nextafter(np.float64(value), np.float64(direction)))


def original_numpy_scalar(value, direction):
    # Exactly the replaced helper's arithmetic and error-state policy.
    with np.errstate(under='ignore'):
        return float(np.nextafter(np.float64(value), np.float64(direction)))


def bit_identity(value):
    if isinstance(value, np.ndarray):
        return (value.dtype.str, value.shape, value.tobytes(order='C'))
    if is_dataclass(value):
        return tuple((item.name, bit_identity(getattr(value, item.name))) for item in fields(value))
    if isinstance(value, (tuple, list)):
        return tuple(bit_identity(item) for item in value)
    if isinstance(value, (float, np.floating)):
        return bits(value)
    return value


SPECIAL_BITS = (
    0x0000000000000000, 0x8000000000000000,
    0x0000000000000001, 0x8000000000000001,
    0x0000000000000002, 0x8000000000000002,
    0x000FFFFFFFFFFFFF, 0x800FFFFFFFFFFFFF,
    0x0010000000000000, 0x8010000000000000,
    0x0010000000000001, 0x8010000000000001,
    0x3FEFFFFFFFFFFFFF, 0xBFEFFFFFFFFFFFFF,
    0x3FF0000000000000, 0xBFF0000000000000,
    0x3FF0000000000001, 0xBFF0000000000001,
    0x7FEFFFFFFFFFFFFE, 0xFFEFFFFFFFFFFFFE,
    0x7FEFFFFFFFFFFFFF, 0xFFEFFFFFFFFFFFFF,
    0x7FF0000000000000, 0xFFF0000000000000,
    0x7FF8000000000000, 0xFFF8000000000000,
    0x7FF0000000000001, 0xFFF0000000000001,
    0x7FFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF,
)
SPECIAL = tuple(from_bits(item) for item in SPECIAL_BITS)


class ScalarNextafterTests(unittest.TestCase):
    def assert_same(self, function, value, direction):
        expected = numpy_oracle(value, direction)
        actual = function(value, direction)
        if math.isnan(expected):
            self.assertTrue(math.isnan(actual))
        else:
            self.assertEqual(bits(actual), bits(expected),
                'value=%s direction=%s' % (bits(value).hex(), bits(direction).hex()))

    def test_fallback_all_special_value_pairs(self):
        for value in SPECIAL:
            for direction in SPECIAL:
                self.assert_same(fs._nextafter_binary64, value, direction)

    def test_chosen_all_special_value_pairs(self):
        for value in SPECIAL:
            for direction in SPECIAL:
                self.assert_same(fs._nextafter, value, direction)

    def test_fallback_deterministic_random_binary64_pairs(self):
        rng = random.Random(0x5A17F00D)
        for _ in range(4096):
            self.assert_same(fs._nextafter_binary64,
                             from_bits(rng.getrandbits(64)), from_bits(rng.getrandbits(64)))

    def test_chosen_deterministic_random_binary64_pairs(self):
        rng = random.Random(0x5A17F00D)
        for _ in range(4096):
            self.assert_same(fs._nextafter,
                             from_bits(rng.getrandbits(64)), from_bits(rng.getrandbits(64)))

    def test_exponent_boundaries_both_signs_and_directions(self):
        for exponent in range(0, 2047):
            for sign in (0, 1 << 63):
                base = sign | (exponent << 52)
                for value_bits in (base, base | ((1 << 52)-1)):
                    value = from_bits(value_bits)
                    for direction in (-math.inf, math.inf):
                        self.assert_same(fs._nextafter_binary64, value, direction)
                        self.assert_same(fs._nextafter, value, direction)

    def test_equal_values_return_destination_signed_zero(self):
        for value, direction in ((0., -0.), (-0., 0.), (-0., -0.), (0., 0.),
                                 (1., 1.), (-1., -1.), (math.inf, math.inf)):
            for function in (fs._nextafter_binary64, fs._nextafter):
                self.assertEqual(bits(function(value, direction)), bits(direction))

    def test_fallback_does_not_use_numpy_error_state(self):
        prior = np.seterr(all='raise')
        try:
            for value in SPECIAL:
                for direction in (-math.inf, -0., 0., math.inf, float('nan')):
                    self.assert_same(fs._nextafter_binary64, value, direction)
                    self.assertEqual(np.geterr(), dict.fromkeys(prior, 'raise'))
            rng = random.Random(1217)
            for _ in range(512):
                self.assert_same(fs._nextafter_binary64,
                                 from_bits(rng.getrandbits(64)), from_bits(rng.getrandbits(64)))
        finally:
            np.seterr(**prior)

    def test_guard_selects_platform_callable_or_fallback(self):
        self.assertIs(fs._nextafter, getattr(math, 'nextafter', fs._nextafter_binary64))

    def test_import_without_stdlib_nextafter_selects_fallback(self):
        # Simulate that one missing API, not an actual Python 3.8 runtime.
        with patch.dict(vars(math)):
            vars(math).pop('nextafter', None)
            namespace = runpy.run_path(fs.__file__, run_name='_finite_nextafter_compat_probe')
        self.assertIs(namespace['_nextafter'], namespace['_nextafter_binary64'])
        self.assert_same(namespace['_nextafter'], -0., math.inf)

    def test_python38_parse_without_new_syntax_or_direct_missing_api(self):
        for path in (Path(fs.__file__), Path(__file__)):
            tree = ast.parse(path.read_text(encoding='utf8'), feature_version=(3, 8))
            for node in ast.walk(tree):
                if isinstance(node, ast.Call):
                    self.assertFalse(any(item.arg == 'slots' for item in node.keywords))
                if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
                    self.assertNotIn((node.value.id, node.attr), (('math', 'nextafter'), ('math', 'ulp')))


def projection_inputs():
    tiny = from_bits(1)
    cases = [((tiny, -tiny), (0., 0.), (.6, .8)),
             ((-0., 0.), (0., -0.), (0., -1.)),
             ((1.e7, 1.e7), (1.e7-1.e-7, 1.e7-2.e-7), (.6, -.8)),
             ((.2, .01), (.20000000000000004, .01), (1., 0.))]
    rng = random.Random(71017)
    for _ in range(128):
        angle = rng.uniform(-math.pi, math.pi)
        cases.append((tuple(rng.uniform(-1.e7, 1.e7) for _ in range(2)),
                      tuple(rng.uniform(-1.e7, 1.e7) for _ in range(2)),
                      (math.cos(angle), math.sin(angle))))
    return cases


def state_history(origin, normal):
    origin = np.asarray(origin, dtype=np.float64)
    normal = np.asarray(normal, dtype=np.float64)
    tangent = np.array([normal[1], -normal[0]])
    state, histories = None, []
    observations = [((-.4, -.1), (.1, .5))]*5 + [((-.3, -.1),), (), ((.1, .6),)]
    for index, intervals in enumerate(observations):
        pairs = np.asarray(intervals, dtype=np.float64).reshape(-1, 2)
        segments = origin + pairs[:, :, None]*tangent
        stamp = index+1 if index < 6 else 20+index
        state = fs.advance_domain(state, origin=origin + index*.002*tangent,
            current_frame_normal=normal, segments=segments,
            normals=np.tile(normal, (len(segments), 2, 1)),
            reliable=np.full(len(segments), index == 2 or index >= 5, dtype=bool),
            stamp_ns=stamp, minimum_frame_votes=5, minimum_reliable_votes=1,
            candidate_ttl_ns=10, normal_cosine=math.cos(math.pi/6.), max_intervals=1000)
        exported = fs.certified_segments(state, origin + .02*normal, normal,
                                         math.cos(math.pi/6.), 5, 1)
        histories.append((state, exported))
    return histories


class PipelineEquivalenceTests(unittest.TestCase):
    def test_fallback_projection_bounds_matches_original_numpy_bits(self):
        cases = projection_inputs()
        with patch.object(fs, '_nextafter', original_numpy_scalar):
            expected = [bit_identity(fs.projection_bounds(*item)) for item in cases]
        with patch.object(fs, '_nextafter', fs._nextafter_binary64):
            actual = [bit_identity(fs.projection_bounds(*item)) for item in cases]
        self.assertEqual(actual, expected)

    def test_chosen_projection_bounds_matches_original_numpy_bits(self):
        cases = projection_inputs()
        actual = [bit_identity(fs.projection_bounds(*item)) for item in cases]
        with patch.object(fs, '_nextafter', original_numpy_scalar):
            expected = [bit_identity(fs.projection_bounds(*item)) for item in cases]
        self.assertEqual(actual, expected)

    def test_fallback_state_history_and_export_matches_original_numpy_bits(self):
        for origin, normal in (((0., 0.), (0., 1.)), ((.2, .01), (0., -1.)),
                               ((123., -57.), (.6, .8)), ((1.e6, -1.e6), (-.8, .6))):
            with patch.object(fs, '_nextafter', original_numpy_scalar):
                expected = state_history(origin, normal)
            with patch.object(fs, '_nextafter', fs._nextafter_binary64):
                actual = state_history(origin, normal)
            self.assertEqual(bit_identity(actual), bit_identity(expected))
            self.assertGreater(len(actual[4][1][0]), 0)

    def test_chosen_state_history_and_export_matches_original_numpy_bits(self):
        actual = state_history((.2, .01), (.6, .8))
        with patch.object(fs, '_nextafter', original_numpy_scalar):
            expected = state_history((.2, .01), (.6, .8))
        self.assertEqual(bit_identity(actual), bit_identity(expected))

    def test_fallback_projection_and_state_under_strict_numpy_errstate(self):
        with patch.object(fs, '_nextafter', original_numpy_scalar):
            expected = bit_identity(state_history((123., -57.), (.6, .8)))
        with patch.object(fs, '_nextafter', fs._nextafter_binary64), np.errstate(all='raise'):
            actual = bit_identity(state_history((123., -57.), (.6, .8)))
            tiny = from_bits(1)
            fs.projection_bounds((tiny, -tiny), (0., 0.), (.6, .8))
        self.assertEqual(actual, expected)

    def test_outward_failures_remain_failures_with_both_implementations(self):
        for function in (original_numpy_scalar, fs._nextafter_binary64):
            for value in (math.inf, -math.inf, float('nan')):
                with patch.object(fs, '_nextafter', function):
                    with self.assertRaises(ValueError):
                        fs._outward(value)

    def test_isolated_unrepresentable_export_rejection_unchanged(self):
        state = fs.DomainState((0., .01), (1., 0.),
            (fs.Interval(.009999999999999997, .010000000000000004, 5, 1, 5),), 5)
        messages = []
        for function in (original_numpy_scalar, fs._nextafter_binary64, fs._nextafter):
            with patch.object(fs, '_nextafter', function):
                with self.assertRaises(ValueError) as error:
                    fs.certified_segments(state, (.001666666666666667, .01),
                                          (0., 1.), math.cos(math.pi/6.), 5, 1)
                messages.append(str(error.exception))
        self.assertEqual(messages, [messages[0]]*3)


if __name__ == '__main__':
    unittest.main()
