"""Exact scalar-validation contract against the independent R17 literal oracle.

The oracle helpers below retain the code at immutable R17 commit
b8482aa37935ff6ba268de6939ca6982681e0e0f. Runtime helpers come from the actual
package, not an in-test fast-path substitute. These are bounded correctness
checks, not a performance benchmark or engine/runtime acceptance test.
"""
import enum
import math
import numbers
import struct
import unittest
import warnings
from decimal import Decimal
from fractions import Fraction

import numpy as np

MAX_STAMP = 2**63 - 1
MAX_VOTES = 2**31 - 1


# Literal prior implementation; kept independent of the candidate functions.
def old_integer(value, name, minimum=1, maximum=MAX_STAMP):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Integral):
        raise ValueError(name + ' must be an integer')
    value = int(value)
    if not minimum <= value <= maximum:
        raise ValueError(name + ' outside supported range')
    return value


def old_number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real):
        raise ValueError(name + ' must be a real number')
    try:
        result = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError(name + ' cannot be represented as float64') from error
    if not math.isfinite(result):
        raise ValueError(name + ' must be finite')
    return result


from lane_mapping_lya_reference.finite_support import (
    _integer as checked_integer, _number as checked_number,
)


class PlainInt(int):
    pass


class PlainFloat(float):
    pass


class AlteredInt(int):
    def __int__(self):
        return 7


class AlteredFloat(float):
    def __float__(self):
        return -0.0


class RaiseInt(int):
    def __int__(self):
        raise OverflowError("int conversion marker")


class RaiseFloat(float):
    def __float__(self):
        raise ValueError("float conversion marker")


class InvalidFloatReturn(float):
    def __float__(self):
        return "not float"


class IntEnum(enum.IntEnum):
    ZERO = 0
    ONE = 1


class FloatingDuck:
    def __float__(self):
        return 1.0


class VirtualReal:
    def __init__(self, result=1.0, exception=None):
        self.result = result
        self.exception = exception
        self.calls = 0

    def __float__(self):
        self.calls += 1
        if self.exception is not None:
            raise self.exception
        return self.result


# Existing contract uses numbers.Real, including registered virtual subclasses.
numbers.Real.register(VirtualReal)


def exception_signature(error, depth=0):
    if error is None or depth > 3:
        return None
    return (
        type(error),
        str(error),
        bool(error.__suppress_context__),
        exception_signature(error.__cause__, depth + 1),
        exception_signature(error.__context__, depth + 1),
    )


def outcome(function, value, *args, **kwargs):
    with warnings.catch_warnings(record=True) as seen:
        warnings.simplefilter("always")
        try:
            result = function(value, *args, **kwargs)
            if type(result) is float:
                answer = ("return", float, struct.pack(">d", result))
            elif type(result) is int:
                answer = ("return", int, result)
            else:
                answer = ("unexpected_return_type", type(result), repr(result))
        except BaseException as error:
            answer = ("raise", exception_signature(error))
    return answer, tuple((item.category, str(item.message)) for item in seen)


class ScalarValidationTests(unittest.TestCase):
    def compare_integer(self, value, name="sample", **bounds):
        self.assertEqual(
            outcome(old_integer, value, name, **bounds),
            outcome(checked_integer, value, name, **bounds),
        )

    def compare_number(self, value, name="sample"):
        self.assertEqual(
            outcome(old_number, value, name),
            outcome(checked_number, value, name),
        )

    def test_builtin_integer_exact_boundary_and_output_type(self):
        for minimum, maximum in ((1, MAX_STAMP), (0, MAX_VOTES), (-7, 9), (5, 5), (5, 4)):
            for value in (minimum-1, minimum, minimum+1, maximum-1, maximum, maximum+1):
                with self.subTest(minimum=minimum, maximum=maximum, value=value):
                    self.compare_integer(value, minimum=minimum, maximum=maximum)
        self.assertIs(type(checked_integer(1, "x")), int)

    def test_huge_integer_integer_path_checks_range_without_float_conversion(self):
        for value in (10**10000, -(10**10000)):
            self.compare_integer(value)
            self.compare_integer(value, minimum=-(10**10001), maximum=10**10001)

    def test_bool_rejected_not_accidentally_integer_fast_path(self):
        for value in (True, False, np.bool_(True), np.bool_(False)):
            self.compare_integer(value, minimum=0)
            self.compare_number(value)
            with self.assertRaisesRegex(ValueError, "^x must be an integer$"):
                checked_integer(value, "x", minimum=0)
            with self.assertRaisesRegex(ValueError, "^x must be a real number$"):
                checked_number(value, "x")

    def test_numpy_integer_signed_unsigned_and_range(self):
        for dtype in (np.int8, np.int16, np.int32, np.int64,
                      np.uint8, np.uint16, np.uint32, np.uint64):
            info = np.iinfo(dtype)
            for value in (dtype(info.min), dtype(0), dtype(1), dtype(info.max)):
                self.compare_integer(value, minimum=0)
                self.compare_number(value)
        self.assertIs(type(checked_integer(np.int64(5), "x")), int)

    def test_numpy_float_scalar_types_are_not_builtin_only(self):
        for dtype in (np.float16, np.float32, np.float64, np.longdouble):
            for value in (dtype(0), dtype(-0.0), dtype(0.5),
                          dtype("nan"), dtype("inf"), dtype("-inf")):
                self.compare_number(value)
                self.compare_integer(value)

    def test_numpy_zero_dimensional_arrays_preserve_existing_rejection(self):
        for value in (np.array(1), np.array(1.0), np.array(True),
                      np.array([1.0]), np.array([], dtype=float)):
            self.compare_integer(value)
            self.compare_number(value)

    def test_fraction_is_real_but_not_integral_even_when_denominator_one(self):
        for value in (Fraction(1, 1), Fraction(1, 3), Fraction(-7, 2),
                      Fraction(0, 1), Fraction(10**10000, 3)):
            self.compare_integer(value)
            self.compare_number(value)
        self.assertIs(type(checked_number(Fraction(1, 3), "x")), float)

    def test_decimal_and_float_duck_remain_non_real(self):
        for value in (Decimal("1"), Decimal("NaN"), FloatingDuck(),
                      complex(1, 0), "1", b"1", None, object()):
            self.compare_integer(value)
            self.compare_number(value)

    def test_integer_subclass_conversion_and_exceptions_are_not_bypassed(self):
        for value in (PlainInt(3), AlteredInt(3), RaiseInt(3), IntEnum.ONE, IntEnum.ZERO):
            self.compare_integer(value)
            self.compare_number(value)
        self.assertEqual(checked_integer(AlteredInt(3), "x"), 7)
        with self.assertRaisesRegex(OverflowError, "^int conversion marker$"):
            checked_integer(RaiseInt(3), "x")

    def test_float_subclass_conversion_and_exception_chain_preserved(self):
        for value in (PlainFloat(3.0), AlteredFloat(3.0), RaiseFloat(3.0),
                      InvalidFloatReturn(3.0)):
            self.compare_number(value)
            self.compare_integer(value)
        self.assertEqual(struct.pack(">d", checked_number(AlteredFloat(3.0), "x")),
                         struct.pack(">d", -0.0))
        with self.assertRaises(ValueError) as caught:
            checked_number(RaiseFloat(3.0), "x")
        self.assertEqual(str(caught.exception), "x cannot be represented as float64")
        self.assertIs(type(caught.exception.__cause__), ValueError)
        self.assertEqual(str(caught.exception.__cause__), "float conversion marker")

    def test_virtual_real_conversion_occurs_once_on_each_slow_path(self):
        for result in (0.0, -0.0, 1.25, float("nan"), float("inf"), "bad"):
            first, second = VirtualReal(result), VirtualReal(result)
            self.assertEqual(outcome(old_number, first, "x"),
                             outcome(checked_number, second, "x"))
            self.assertEqual(first.calls, 1)
            self.assertEqual(second.calls, 1)

    def test_virtual_real_value_overflow_and_other_errors_preserve_chain(self):
        for exception_class in (ValueError, OverflowError, TypeError, RuntimeError):
            first = VirtualReal(exception=exception_class("marker"))
            second = VirtualReal(exception=exception_class("marker"))
            self.assertEqual(outcome(old_number, first, "x"),
                             outcome(checked_number, second, "x"))
            self.assertEqual((first.calls, second.calls), (1, 1))

    def test_builtin_float_extremes_and_signed_zero_are_bit_exact(self):
        values = (
            0.0, -0.0, 1.0, -1.0, 1.e7, -1.e7,
            float.fromhex("0x0.0000000000001p-1022"),
            -float.fromhex("0x0.0000000000001p-1022"),
            float.fromhex("0x1.fffffffffffffp+1023"),
            -float.fromhex("0x1.fffffffffffffp+1023"),
            float.fromhex("0x1.0000000000001p+0"),
        )
        for value in values:
            self.compare_number(value)
            self.assertEqual(struct.pack(">d", value),
                             struct.pack(">d", checked_number(value, "x")))
            self.assertIs(type(checked_number(value, "x")), float)

    def test_nan_payload_sign_and_infinities_still_rejected(self):
        for bits in (0x7ff0000000000001, 0x7ff8000000000000,
                     0xfff8000000000001, 0x7ff0000000000000,
                     0xfff0000000000000):
            value = struct.unpack(">d", struct.pack(">Q", bits))[0]
            self.compare_number(value)
            with self.assertRaisesRegex(ValueError, "^x must be finite$"):
                checked_number(value, "x")

    def test_number_huge_int_preserves_value_error_and_overflow_cause(self):
        for value in (10**10000, -(10**10000)):
            self.compare_number(value)
            with self.assertRaises(ValueError) as caught:
                checked_number(value, "x")
            self.assertEqual(str(caught.exception), "x cannot be represented as float64")
            self.assertIs(type(caught.exception.__cause__), OverflowError)

    def test_names_and_bound_parameter_errors_not_hidden_by_fast_path(self):
        for name in ("", "motion.stamp", 12, None):
            for value in (0, 1, True, "bad"):
                self.compare_integer(value, name)
            for value in (1.0, float("inf"), True, "bad"):
                self.compare_number(value, name)
        for bounds in ({"minimum": None}, {"maximum": "3"},
                       {"minimum": float("nan")}, {"maximum": float("inf")},
                       {"minimum": np.int64(0), "maximum": np.int64(2)},
                       {"minimum": Fraction(0), "maximum": Fraction(2)}):
            for value in (0, 1, 3, np.int64(1)):
                self.compare_integer(value, **bounds)

    def test_finite_coordinate_limit_remains_caller_policy_not_scalar_rejection(self):
        # _number itself is finite-only; Interval/_vector enforce coordinate range.
        for value in (1.e7, -1.e7, 4.e7, -4.e7, 4.e7 + 1.0, 1.e300):
            self.compare_number(value)
            self.assertEqual(checked_number(value, "x"), value)

    def test_bounded_deterministic_ieee_patterns_no_timing_measurement(self):
        # A small arithmetic-contract set, not a benchmark or runtime proof.
        bits = 0x123456789abcdef0
        for _ in range(256):
            bits = (bits * 6364136223846793005 + 1442695040888963407) & ((1 << 64) - 1)
            value = struct.unpack(">d", struct.pack(">Q", bits))[0]
            self.compare_number(value)

    def test_interval_like_assembly_preserves_endpoints_counts_and_stamp(self):
        def assembly(integer, number, lower, upper, votes, reliable, stamp):
            lower = number(lower, "interval.lower")
            upper = number(upper, "interval.upper")
            if not lower < upper or max(abs(lower), abs(upper)) > 4.e7:
                raise ValueError("interval requires bounded positive extent")
            votes = integer(votes, "interval.frame_votes", maximum=MAX_VOTES)
            reliable = integer(reliable, "interval.reliable_votes", minimum=0, maximum=votes)
            stamp = integer(stamp, "interval.last_stamp_ns")
            return struct.pack(">dd", lower, upper), votes, reliable, stamp
        cases = [
            (-0.0, 1.0, 5, 1, 5),
            (np.float64(-1), Fraction(1, 3), np.int64(5), np.int32(1), MAX_STAMP),
            (0.0, 0.0, 5, 1, 5),
            (0.0, 1.0, True, 1, 5),
            (0.0, 1.0, 5, 6, 5),
            (0.0, float("inf"), 5, 1, 5),
            (0.0, 1.0, 5, 1, MAX_STAMP + 1),
        ]
        for case in cases:
            results = []
            for integer, number in ((old_integer, old_number),
                                    (checked_integer, checked_number)):
                try:
                    results.append(("return", assembly(integer, number, *case)))
                except BaseException as error:
                    results.append(("raise", exception_signature(error)))
            self.assertEqual(results[0], results[1])


if __name__ == "__main__":
    unittest.main(verbosity=2)



