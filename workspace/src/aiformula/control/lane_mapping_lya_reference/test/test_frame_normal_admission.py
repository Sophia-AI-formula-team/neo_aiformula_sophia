"""Portable tests of ACTUAL production normal admission; prepared, NOT RUN.

The native recorder is an artificial call-boundary fixture, not geometry.
All algorithms come from the package finite_support.py, never D candidates.
Fixture identities are established during source exec, not patched afterward.
"""
import itertools
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np


FINITE_PATH = (Path(__file__).resolve().parents[1] /
               "lane_mapping_lya_reference" / "finite_support.py")
_IDS = itertools.count()
_REAL_DOT = np.dot
POLICY = dict(divide="warn", over="warn", under="ignore", invalid="warn")


def load_finite(native):
    """Load the actual portable production source with its native fixture first."""
    path = FINITE_PATH.resolve(strict=True)
    if path.name != "finite_support.py":
        raise RuntimeError("unexpected production source filename")
    name = "_frame_normal_production_{}".format(next(_IDS))
    module = types.ModuleType(name)
    module.__file__ = str(path)
    sys.modules[name] = module
    fixture = types.ModuleType("_lane_mapping_lya_native")
    fixture.FINITE_CURRENT_UNION_API_VERSION = 1
    fixture.finite_current_union = native
    try:
        with patch.dict(sys.modules, {"_lane_mapping_lya_native": fixture}):
            exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module


class ProductionAPI:
    """Only forward the legacy test call shape to real production functions."""
    def __init__(self, finite):
        self.finite = finite

    def prepare_frame(self, finite, batch, normals, cosine):
        if finite is not self.finite:
            raise AssertionError("wrong production module")
        return finite._prepare_normal_admission(batch, normals, cosine)

    def current_union(self, finite, plan, batch, index, *args):
        if finite is not self.finite:
            raise AssertionError("wrong production module")
        return finite._current_union_admitted(plan, batch, index, *args)


class NativeFailure(RuntimeError):
    pass


class NativeRecorder:
    def __init__(self):
        self.calls = []
        self.fail = False

    def __call__(self, *args):
        self.calls.append(args)
        if self.fail:
            raise NativeFailure("original native sentinel")
        return []  # No geometric result is supplied/claimed by this fixture.


def direction(x, sign=1.):
    x = float(x)
    return (x, math.copysign(math.sqrt(max(0., 1. - x*x)), sign))


def make_batch(finite, pairs, lengths, reliable_bytes=None):
    normals = np.array(pairs, dtype=np.float64).reshape(-1, 2, 2)
    count = len(normals)
    segments = np.array([[(float(i), 0.), (float(i+1), 0.)] for i in range(count)],
                        dtype=np.float64).reshape(-1, 2, 2)
    reliable = (np.ones(count, dtype=np.bool_) if reliable_bytes is None else
                np.frombuffer(reliable_bytes, dtype=np.bool_))
    offsets = np.r_[np.int64(0), np.cumsum(lengths, dtype=np.int64)]
    return finite._validate_frame_batch(segments, normals, reliable, offsets)


def group_args(batch, index, normal, cosine=.5):
    first, stop = int(batch.offsets[index]), int(batch.offsets[index+1])
    tangent = (float(normal[1]), -float(normal[0]))
    return (batch.segments[first:stop], batch.normals[first:stop], batch.reliable[first:stop],
            (0.125, -0.25), tangent, cosine)


def literal_bits(normals, normal, cosine):
    result = bytearray(len(normals))
    for index, directions in enumerate(normals):
        if (float(_REAL_DOT(directions[0], normal)) < cosine or
                float(_REAL_DOT(directions[1], normal)) < cosine):
            continue
        result[index] = 1
    return bytes(result)


class FrameAdmissionTests(unittest.TestCase):
    def setUp(self):
        old = np.seterr(**POLICY)
        self.addCleanup(np.seterr, **old)
        self.native = NativeRecorder()
        self.bind_finite()

    def bind_finite(self):
        # Also used under a dot spy: identities must bind during actual exec.
        self.finite = load_finite(self.native)
        self.helper = ProductionAPI(self.finite)
        self.addCleanup(sys.modules.pop, self.finite.__name__, None)
        self.assertIs(self.finite._ADMISSION_ORIGINAL_NATIVE, self.native)
        self.assertIs(self.finite._ADMISSION_ORIGINAL_UNION, self.finite._current_union)
        self.assertIs(self.finite._ADMISSION_ORIGINAL_PYTHON, self.finite._current_union_python)

    def plan(self, batch, normals, cosine=.5, helper=None):
        helper = self.helper if helper is None else helper
        return helper.prepare_frame(self.finite, batch, np.array(normals, dtype=np.float64), cosine)

    def compare_group(self, batch, plan, index, normal, cosine=.5):
        args = group_args(batch, index, normal, cosine)
        before = tuple(array.tobytes() for array in (batch.segments, batch.normals, batch.reliable, batch.offsets))
        original = self.finite._current_union(*args)
        expected = self.native.calls[-1]
        proposed = self.helper.current_union(self.finite, plan, batch, index, *args)
        self.assertEqual(proposed, original)
        self.assertEqual(self.native.calls[-1], expected)
        self.assertEqual(expected[1], literal_bits(args[1], np.array(normal), cosine))
        self.assertEqual(before, tuple(array.tobytes() for array in
                         (batch.segments, batch.normals, batch.reliable, batch.offsets)))

    def test_01_mixed_original_creation_normals_all_native_arguments(self):
        normals = [(1., 0.), (0., 1.), (-1., 0.), (0., -1.)]
        pairs = [(normal, normal) for normal in normals for unused in range(3)]
        batch = make_batch(self.finite, pairs, [3, 3, 3, 3])
        plan = self.plan(batch, normals)
        self.assertIsNotNone(plan)
        for index, normal in enumerate(normals):
            self.compare_group(batch, plan, index, normal)

    def test_02_small_and_empty_groups_not_dropped(self):
        pairs = [((1., 0.), (1., 0.)) for unused in range(8)]
        lengths = [0, 1, 1, 0, 2, 1, 1, 2]
        normals = [(1., 0.)] * len(lengths)
        batch = make_batch(self.finite, pairs, lengths)
        plan = self.plan(batch, normals)
        self.assertIsNotNone(plan)
        for index, normal in enumerate(normals):
            self.compare_group(batch, plan, index, normal)
        self.assertEqual(len(plan.tags), 8)

    def test_03_uncertain_dots_lazy_and_original_or_sequence(self):
        below = float(np.nextafter(.5, -np.inf))
        x_pairs = [(1., 1.), (.5, 1.), (below, 1.), (1., below),
                   (0., .5), (.5, 0.), (.5, .5), (0., 0.)]
        pairs = [(direction(a, 1.), direction(b, -1.)) for a, b in x_pairs]
        calls = []

        def delegate(left, right):
            calls.append(tuple(map(float, left)))
            return _REAL_DOT(left, right)

        with patch.object(np, "dot", delegate):
            self.bind_finite()
            batch = make_batch(self.finite, pairs, [4, 4])
            helper = self.helper
            plan = self.plan(batch, [(1., 0.)] * 2, helper=helper)
            self.assertIsNotNone(plan)
            self.assertEqual(calls, [])
            helper.current_union(self.finite, plan, batch, 0, *group_args(batch, 0, (1., 0.)))
            expected_first = [(1, 0), (1, 1), (2, 0), (3, 0), (3, 1)]
            self.assertEqual(calls, [tuple(pairs[row][endpoint]) for row, endpoint in expected_first])
            helper.current_union(self.finite, plan, batch, 1, *group_args(batch, 1, (1., 0.)))
            self.assertEqual(calls, [tuple(pairs[row][endpoint]) for row, endpoint in
                                    expected_first + [(6, 0), (6, 1)]])

    def test_04_earlier_native_error_prevents_later_uncertain_dots(self):
        pairs = [((1., 0.), (1., 0.))] * 4 + [(direction(.5), direction(.5, -1.))] * 4
        calls = []

        def delegate(left, right):
            calls.append(tuple(map(float, left)))
            return _REAL_DOT(left, right)

        with patch.object(np, "dot", delegate):
            self.bind_finite()
            batch = make_batch(self.finite, pairs, [4, 4])
            helper = self.helper
            plan = self.plan(batch, [(1., 0.)] * 2, helper=helper)
            self.assertEqual(calls, [])
            self.native.fail = True
            with self.assertRaisesRegex(NativeFailure, "original native sentinel"):
                helper.current_union(self.finite, plan, batch, 0, *group_args(batch, 0, (1., 0.)))
            self.assertEqual(calls, [])
            self.assertEqual(len(self.native.calls), 1)
        # This tests lazy native error order, not an unimplemented advance hook.

    def test_05_preflight_ineligible_is_none_and_original_full_fallback(self):
        batch = make_batch(self.finite, [((1., 0.), (1., 0.))] * 8, [8])
        normals = np.array([(1., 0.)])
        for bad in (normals.astype(np.float32), normals[:, ::-1], np.array([1., 0.]),
                    np.array([(float("nan"), 0.)]), np.array([(2.1, 0.)])):
            self.assertIsNone(self.helper.prepare_frame(self.finite, batch, bad, .5))
        for cosine in (True, 1, np.float64(.5), 0., float("nan"), float("inf")):
            self.assertIsNone(self.helper.prepare_frame(self.finite, batch, normals, cosine))
        self.assertIsNone(self.helper.prepare_frame(self.finite, object(), normals, .5))
        # The diagnostic-only source_sha argument is not a production API.
        # Its assertion is replaced by REAL module/kernel identity tests below.
        self.compare_group(batch, None, 0, (1., 0.))

    def test_06_batch_view_and_chart_mismatch_full_current_alias(self):
        batch = make_batch(self.finite, [((1., 0.), (1., 0.))] * 8, [4, 4])
        plan = self.plan(batch, [(1., 0.)] * 2)
        original = self.finite._current_union
        calls = []

        def spy(*args):
            calls.append(args)
            return original(*args)

        with patch.object(self.finite, "_current_union", spy):
            self.helper.current_union(self.finite, plan, batch, 0, *group_args(batch, 0, (1., 0.)))
        self.assertEqual(len(calls), 1)
        other = make_batch(self.finite, [((1., 0.), (1., 0.))] * 8, [4, 4])
        self.compare_group(other, plan, 0, (1., 0.))
        # Fresh identical bytes/views and differing actual creation normal are
        # not silently accepted as the plan's authenticated group.
        args = list(group_args(batch, 0, (1., 0.)))
        args[0] = args[0].copy()
        expected = self.finite._current_union(*args)
        expected_native = self.native.calls[-1]
        self.assertEqual(self.helper.current_union(self.finite, plan, batch, 0, *args), expected)
        self.assertEqual(self.native.calls[-1], expected_native)
        self.compare_group(batch, plan, 0, (-1., 0.))

    def test_07_noncanonical_bool_bytes_and_multiplicity_preserved(self):
        pairs = [((1., 0.), (1., 0.))] * 8
        batch = make_batch(self.finite, pairs, [4, 4], bytes((0, 2, 255, 1, 2, 0, 255, 1)))
        plan = self.plan(batch, [(1., 0.)] * 2)
        for index in (0, 1):
            self.compare_group(batch, plan, index, (1., 0.))
        self.assertEqual(self.native.calls[-1][2], bytes((1, 0, 1, 1)))
        self.assertEqual(len(plan.tags), 8)  # Repeated rows are not deduplicated.

    def test_08_mid_frame_dot_and_strict_policy_use_complete_old_route(self):
        batch = make_batch(self.finite, [((1., 0.), (1., 0.))] * 8, [4, 4])
        plan = self.plan(batch, [(1., 0.)] * 2)
        calls = []

        def rejecting(left, right):
            calls.append(tuple(map(float, left)))
            return 0.

        with patch.object(np, "dot", rejecting):
            self.helper.current_union(self.finite, plan, batch, 1, *group_args(batch, 1, (1., 0.)))
        self.assertEqual(calls, [(1., 0.)] * 4)
        self.assertEqual(self.native.calls[-1][1], bytes(4))
        with np.errstate(under="raise"):
            expected = self.finite._current_union(*group_args(batch, 0, (1., 0.)))
            proposed = self.helper.current_union(self.finite, plan, batch, 0, *group_args(batch, 0, (1., 0.)))
            self.assertEqual(proposed, expected)
            self.assertIsNone(self.plan(batch, [(1., 0.)] * 2))

    def test_09_near_ulps_and_closed_cosine_actual_scalar_bits(self):
        for cosine in (1.e-6, .5, 1.):
            below = float(np.nextafter(cosine, -np.inf))
            above = min(1., float(np.nextafter(cosine, np.inf)))
            values = (cosine, below, above, 0., 1., -1., below, cosine)
            pairs = [(direction(value), direction(value, -1.)) for value in values]
            batch = make_batch(self.finite, pairs, [1] * 8)
            plan = self.plan(batch, [(1., 0.)] * 8, cosine)
            self.assertIsNotNone(plan)
            for index in range(8):
                self.compare_group(batch, plan, index, (1., 0.), cosine)

    def test_10_signed_zero_and_unsupported_tangent_preserve_original_conversion(self):
        batch = make_batch(self.finite, [((1., 0.), (1., 0.))] * 8, [8])
        plan = self.plan(batch, [(1., -0.)])
        self.compare_group(batch, plan, 0, (1., -0.))
        self.compare_group(batch, plan, 0, (1., 0.))

        class Scalar:
            def __init__(self, value):
                self.value = value
                self.conversions = 0

            def __neg__(self):
                return -self.value

            def __float__(self):
                self.conversions += 1
                return float(self.value)

        scalar = Scalar(0.)
        args = list(group_args(batch, 0, (1., 0.)))
        args[4] = (scalar, -1.)
        self.helper.current_union(self.finite, plan, batch, 0, *args)
        # Original normal conversion and native float(tangent[0]) each occur;
        # the optional helper must not convert the custom scalar an extra time.
        self.assertEqual(scalar.conversions, 2)

    def test_11_production_identity_guards_and_current_fallback(self):
        batch = make_batch(self.finite, [((1., 0.), (1., 0.))] * 8, [8])
        normals = np.array([(1., 0.)])
        plan = self.plan(batch, [(1., 0.)])
        self.assertIsNotNone(plan)
        for name in ("_current_union", "_current_union_python", "_NATIVE_CURRENT_UNION"):
            original = getattr(self.finite, name)
            calls = []
            def spy(*args, **kwargs):
                calls.append((args, kwargs))
                return original(*args, **kwargs)
            with patch.object(self.finite, name, spy):
                self.assertIsNone(self.helper.prepare_frame(self.finite, batch, normals, .5))
                args = group_args(batch, 0, (1., 0.))
                actual = self.helper.current_union(self.finite, plan, batch, 0, *args)
                self.assertEqual(actual, [])
                if name in ("_current_union", "_NATIVE_CURRENT_UNION"):
                    self.assertEqual(len(calls), 1)
        with patch.object(self.finite, "_NATIVE_CURRENT_UNION", None):
            self.assertIsNone(self.helper.prepare_frame(self.finite, batch, normals, .5))

    def test_12_prior_scalar_and_state_errors_precede_later_unknown_dot_native(self):
        calls = []
        def delegate(left, right):
            calls.append(tuple(map(float, left)))
            return _REAL_DOT(left, right)
        with patch.object(np, "dot", delegate):
            self.bind_finite()
            pairs = [(direction(.5), direction(.5, -1.))] * 8
            batch = make_batch(self.finite, pairs, [4, 4])
            plan = self.plan(batch, [(1., 0.)] * 2)
            self.assertIsNotNone(plan)
            self.assertEqual(plan.tags, bytes([2] * 8))
            defaults = dict(batch=batch, origin=(0., 0.), current_frame_normal=(1., 0.),
                stamp_ns=11, minimum_frame_votes=5, minimum_reliable_votes=1,
                candidate_ttl_ns=0, normal_cosine=.5, max_intervals=8,
                normal_admission=plan)
            wrong = [
                (dict(origin=(True, 0.)), None, "origin"),
                (dict(current_frame_normal=(2., 0.)), None, "unit"),
                (dict(stamp_ns=True), None, "stamp_ns"),
                (dict(minimum_frame_votes=0), None, "minimum_frame_votes"),
                (dict(minimum_reliable_votes=6), None, "minimum_reliable_votes"),
                (dict(candidate_ttl_ns=-1), None, "candidate_ttl_ns"),
                (dict(normal_cosine=0.), None, "normal_cosine"),
                (dict(max_intervals=0), None, "max_intervals"),
                ({}, object(), "previous must"),
                ({}, self.finite.DomainState((0., 0.), (0., -1.), (), 11), "strictly increase"),
                ({}, self.finite.DomainState((0., 0.), (0., -1.),
                    (self.finite.Interval(0., 1., 6, 1, 10),), 10), "counters"),
                (dict(max_intervals=1), self.finite.DomainState((0., 0.), (0., -1.),
                    (self.finite.Interval(0., 1., 1, 0, 10),
                     self.finite.Interval(2., 3., 1, 0, 10)), 10), "capacity"),
            ]
            for changes, previous, message in wrong:
                with self.subTest(message=message, changes=changes):
                    calls.clear()
                    self.native.calls.clear()
                    kwargs = dict(defaults, **changes)
                    with self.assertRaisesRegex(ValueError, message):
                        for index in (0, 1):
                            self.finite._advance_domain_validated(
                                previous if index == 0 else None, group_index=index, **kwargs)
                    self.assertEqual(calls, [])
                    self.assertEqual(self.native.calls, [])
            # An eligible plan must NOT be consumed at the default/public entry.
            # No native geometric result is supplied by this test fixture.
            self.assertEqual(calls, [])
            self.assertEqual(self.native.calls, [])

    def test_13_public_and_default_validated_keep_original_union_route(self):
        batch = make_batch(self.finite, [((1., 0.), (1., 0.))] * 8, [8])
        args = group_args(batch, 0, (1., 0.))
        params = dict(origin=(0., 0.), current_frame_normal=(1., 0.), stamp_ns=1,
            minimum_frame_votes=5, minimum_reliable_votes=1,
            candidate_ttl_ns=0, normal_cosine=.5, max_intervals=8)
        original = self.finite._current_union
        calls = []
        def forward(*values):
            calls.append(values)
            return original(*values)
        with patch.object(self.finite, "_current_union", forward), patch.object(
                self.finite, "_current_union_admitted", side_effect=AssertionError("unexpected admission consumer")):
            self.finite.advance_domain(None, segments=args[0], normals=args[1], reliable=args[2], **params)
            self.finite._advance_domain_validated(None, batch=batch, group_index=0, **params)
        self.assertEqual(len(calls), 2)
        self.assertEqual(len(self.native.calls), 2)
        self.assertEqual(self.native.calls[0], self.native.calls[1])


if __name__ == "__main__":
    unittest.main()
