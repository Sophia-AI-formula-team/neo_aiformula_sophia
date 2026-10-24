"""Independent boundary fixtures for generator-only linear roundoff handling.

These fixtures are nominal binary64/Fraction checks, not vehicle, original
rounded-fit-program, owner or actual joint proofs. An empty definite-violation
set is deliberately shown NOT to certify feasibility. No legacy test is
imported, skipped, relaxed or used to manufacture this oracle.
"""
import ast
from fractions import Fraction
import inspect
from itertools import product
import unittest
from unittest import mock

import numpy as np

from lane_mapping_lya_reference import paired_curve_fit as subject


def rational(value):
    return Fraction.from_float(float(value))


def linear_residuals_direct(rows, beta, limits):
    # Exact arithmetic on the SUPPLIED rounded rows, not a production chart.
    b0, b1 = (rational(v) for v in beta)
    return [rational(row[0])*b0+rational(row[1])*b1-rational(limit)
            for row, limit in zip(rows, limits)]


def world_label_direct(model, coefficients, point, role):
    # Independent substitution of the ORIGINAL supplied world point.
    ox, oy = (rational(v) for v in model.origin)
    nx, ny = (rational(v) for v in model.normal)
    dx, dy = rational(point[0])-ox, rational(point[1])-oy
    x = (ny*dx-nx*dy)/rational(model.scale)
    y = nx*dx+ny*dy
    a, b, c = (rational(v) for v in coefficients)
    return role*(y-a-b*x-c*x*x)


def array_identity(array):
    return (array.dtype.str, array.shape, array.strides,
            array.tobytes(order="C"), bool(array.flags.writeable))


def work_counters():
    return dict(systems=0, pivots=0, full_scans=0, rounding_packets=0,
                exact_label_substitutions=0, reason=None)


def six_point_pair():
    # Two orthogonal original primary charts, no label reconstruction by helper.
    tangent = np.array([-1., -.75, -.5, .5, .75, 1.])
    models = [subject.PrimaryPatch.from_arrays((0., 0.), (1., 0.), 1., (-1., 1.)),
              subject.PrimaryPatch.from_arrays((0., 0.), (0., -1.), 1., (-1., 1.))]
    centers = [np.column_stack((np.zeros(6), -tangent)),
               np.column_stack((-tangent, np.zeros(6)))]
    whites = [centers[0]-np.array([.25, 0.]), centers[1]-np.array([0., -.25])]
    blacks = [centers[0]+np.array([.25, 0.]), centers[1]+np.array([0., -.25])]
    return models, centers, whites, blacks, np.zeros(2), np.eye(2)


class PairedCurveRoundoffTests(unittest.TestCase):
    def test_definite_members_have_independent_fraction_positive_residuals(self):
        rows = np.array([[.5, .25], [-.5, .25], [1., 0.], [1., 1.], [-1., -1.]])
        beta = np.array([2., 4.])
        limits = np.array([1., 2., 0., 5., -6.])
        exact = linear_residuals_direct(rows, beta, limits)
        self.assertEqual(exact, [Fraction(1), Fraction(-2), Fraction(2),
                                 Fraction(1), Fraction(0)])
        found = subject._definite_linear_violations(rows, beta, limits)
        np.testing.assert_array_equal(found, [0, 2, 3])
        self.assertEqual(list(found), [i for i, value in enumerate(exact) if value > 0])
        self.assertTrue(all(exact[int(i)] > 0 for i in found))

    def test_closed_equalities_signed_zero_and_empty_rows_are_not_pivots(self):
        rows = np.array([[0., -0.], [.5, -.25], [1., -1.]])
        beta = np.array([2., 4.])
        limits = np.array([-0., 0., -2.])
        self.assertEqual(linear_residuals_direct(rows, beta, limits), [Fraction(0)]*3)
        self.assertEqual(subject._definite_linear_violations(rows, beta, limits).size, 0)
        empty = subject._definite_linear_violations(np.empty((0, 2)), beta, np.empty(0))
        self.assertEqual(empty.shape, (0,))
        self.assertEqual(empty.dtype.kind, "i")

    def test_positive_one_ulp_and_cancellation_can_be_uncertain_not_feasible(self):
        below_one = float(np.nextafter(1., -np.inf))
        smallest = float(np.nextafter(0., np.inf))
        fixtures = [
            (np.array([[1., 0.]]), np.array([1., 0.]), np.array([below_one]),
             Fraction(1, 2**53)),
            (np.array([[float(2**53), 1.]]), np.array([1., 1.]),
             np.array([float(2**53)]), Fraction(1)),
            (np.array([[smallest, 0.]]), np.array([1., 0.]),
             np.array([0.]), Fraction(1, 2**1074)),
        ]
        for rows, beta, limits, expected in fixtures:
            with self.subTest(expected=expected):
                exact = linear_residuals_direct(rows, beta, limits)
                self.assertEqual(exact, [expected])
                self.assertGreater(exact[0], 0)
                self.assertEqual(subject._definite_linear_violations(rows, beta, limits).size, 0)
                # Empty generator pivots must never be asserted as exact feasible.
                self.assertFalse(all(value <= 0 for value in exact))

    def test_power_of_two_scaling_has_no_fixed_absolute_epsilon(self):
        for exponent in (-500, 0, 500):
            with self.subTest(exponent=exponent):
                scale = float(np.ldexp(1., exponent))
                rows = np.array([[scale, 0.], [scale, 0.], [-scale, 0.]])
                beta = np.array([1., 1.])
                limits = np.array([0., 2.*scale, 0.])
                exact = linear_residuals_direct(rows, beta, limits)
                self.assertEqual(exact, [rational(scale), -rational(scale), -rational(scale)])
                np.testing.assert_array_equal(subject._definite_linear_violations(rows, beta, limits), [0])

    def test_readonly_strided_duplicate_rows_preserve_order_and_input_bytes(self):
        storage = np.array([[1., 0.], [77., 88.], [1., 0.], [77., 88.],
                            [0., 1.], [77., 88.], [1., 0.], [77., 88.]])
        limit_storage = np.array([.5, 99., 1., 99., 1.5, 99., .5, 99.])
        beta_storage = np.array([1., 99., 2., 99.])
        rows, limits, beta = storage[::2], limit_storage[::2], beta_storage[::2]
        for array in (rows, limits, beta):
            array.flags.writeable = False
        self.assertFalse(rows.flags.c_contiguous)
        inputs = (storage, limit_storage, beta_storage, rows, limits, beta)
        before = [array_identity(array) for array in inputs]
        exact = linear_residuals_direct(rows, beta, limits)
        self.assertEqual(exact, [Fraction(1, 2), Fraction(0), Fraction(1, 2), Fraction(1, 2)])
        np.testing.assert_array_equal(subject._definite_linear_violations(rows, beta, limits), [0, 2, 3])
        self.assertEqual([array_identity(array) for array in inputs], before)
        self.assertEqual(len(rows), 4)  # Repeated original rows were not deduplicated.

    def test_nonfinite_and_overflow_enclosures_raise_not_silent_acceptance(self):
        huge = np.finfo(np.float64).max
        fixtures = [
            (np.array([[huge, huge]]), np.array([2., -2.]), np.array([0.])),
            (np.array([[huge, huge]]), np.array([1., 1.]), np.array([0.])),
            (np.array([[-huge, 0.]]), np.array([1., 0.]), np.array([huge])),
            (np.array([[np.nan, 0.]]), np.array([1., 0.]), np.array([0.])),
            (np.array([[1., 0.]]), np.array([np.inf, 0.]), np.array([0.])),
            (np.array([[1., 0.]]), np.array([1., 0.]), np.array([-np.inf])),
        ]
        # Even an EXACTLY ZERO final scalar can have overflowing intermediates.
        self.assertEqual(linear_residuals_direct(*fixtures[0]), [Fraction(0)])
        for index, (rows, beta, limits) in enumerate(fixtures):
            with self.subTest(index=index), np.errstate(all="raise"):
                with self.assertRaisesRegex(ValueError, "nonfinite proposal linear-residual enclosure"):
                    subject._definite_linear_violations(rows, beta, limits)

    def test_zero_pivot_uncertain_generator_is_not_a_white_black_certificate(self):
        below_one = float(np.nextafter(1., -np.inf))
        rows = np.tile(np.array([[-1., -1.], [1., 1.]]), (6, 1))
        limits = np.full(12, 2.)
        limits[-1] = below_one
        diagnostic = work_counters()
        free = subject._restricted_proposal(np.eye(2), np.array([1., 0.]),
                                            rows, limits, 0, diagnostic)
        np.testing.assert_array_equal(free, [1., 0.])
        self.assertEqual(diagnostic["systems"], 1)
        self.assertEqual(diagnostic["pivots"], 0)
        self.assertEqual(diagnostic["full_scans"], 1)
        self.assertGreater(linear_residuals_direct(rows, free, limits)[-1], 0)
        model = subject.PrimaryPatch.from_arrays((0., 0.), (1., 0.), 1., (-.5, .5))
        whites = np.tile(np.array([-2., -1.]), (6, 1))
        blacks = np.tile(np.array([2., -1.]), (6, 1))
        blacks[-1, 0] = below_one
        beta = (0., float(free[0]), float(free[1]))
        self.assertEqual(world_label_direct(model, beta, blacks[-1], 1), -Fraction(1, 2**53))
        self.assertTrue(all(world_label_direct(model, beta, point, -1) >= 0 for point in whites))
        self.assertFalse(subject._labels(model, beta, whites, blacks, diagnostic))
        self.assertGreater(diagnostic["exact_label_substitutions"], 0)
        # All six labels, including off-extent repeated ones, still matter.
        self.assertEqual(len(whites), 6)
        self.assertEqual(len(blacks), 6)

    def test_all_27_packets_keep_full_off_extent_duplicate_label_guard(self):
        model = subject.PrimaryPatch.from_arrays((0., 0.), (1., 0.), 1., (-.5, .5))
        tangent = np.array([-1., -.75, -.5, .5, .75, 1.])
        centers = np.column_stack((np.zeros(6), -tangent))
        whites = np.tile(np.array([-2., -1.]), (6, 1))
        blacks = np.tile(np.array([2., -1.]), (6, 1))
        bad = 1.
        for _ in range(8):
            bad = float(np.nextafter(bad, -np.inf))
        blacks[-1] = (bad, -1.)
        # The unmodified last original occurrence is outside the finite extent.
        self.assertGreater(Fraction(1), rational(model.bounds[1]))
        adjacent = [(v, float(np.nextafter(v, -np.inf)), float(np.nextafter(v, np.inf)))
                    for v in (0., 1., 0.)]
        packets = list(product(*adjacent))
        self.assertEqual(len(packets), 27)
        self.assertTrue(all(world_label_direct(model, beta, blacks[-1], 1) < 0 for beta in packets))
        arrays = (centers, whites, blacks)
        before = [array_identity(array) for array in arrays]
        guarded = []
        original_labels = subject._labels

        def guard(original_model, coefficients, original_whites, original_blacks, diagnostic):
            self.assertIs(original_model, model)
            self.assertEqual(array_identity(original_whites), before[1])
            self.assertEqual(array_identity(original_blacks), before[2])
            guarded.append(tuple(coefficients))
            return original_labels(original_model, coefficients, original_whites, original_blacks, diagnostic)

        diagnostic = work_counters()
        with mock.patch.object(subject, "_restricted_proposal", return_value=np.array([1., 0.])):
            with mock.patch.object(subject, "_labels", side_effect=guard):
                beta = subject._fit_at_q(model, centers, whites, blacks,
                                         (Fraction(0), Fraction(0)), 0, diagnostic)
        self.assertIsNone(beta)
        self.assertEqual(guarded, packets)
        self.assertEqual(diagnostic["rounding_packets"], 27)
        self.assertEqual(diagnostic["reason"], "no_certified_27_packet")
        self.assertEqual([array_identity(array) for array in arrays], before)

    def test_definite_violation_still_enforces_zero_pivot_budget_unknown(self):
        rows = np.array([[1., 0.]])
        limits = np.array([.5])
        diagnostic = work_counters()
        self.assertEqual(linear_residuals_direct(rows, [1., 0.], limits), [Fraction(1, 2)])
        value = subject._restricted_proposal(np.eye(2), np.array([1., 0.]),
                                             rows, limits, 0, diagnostic)
        self.assertIsNone(value)
        self.assertEqual(diagnostic["reason"], "pivot_budget_unknown")
        self.assertEqual(diagnostic["systems"], 1)
        self.assertEqual(diagnostic["pivots"], 0)
        self.assertEqual(diagnostic["full_scans"], 1)

    def test_public_enclosure_error_keeps_partial_unknown_and_no_adoption(self):
        models, centers, whites, blacks, junction, axes = six_point_pair()
        arrays = (*centers, *whites, *blacks, junction, axes)
        before = [array_identity(array) for array in arrays]
        with mock.patch.object(subject, "_definite_linear_violations",
                               side_effect=ValueError("nonfinite proposal linear-residual enclosure")):
            result = subject.propose_pair(*models, centers, whites, blacks, junction, axes)
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["reason"], "nonfinite proposal linear-residual enclosure")
        self.assertTrue(result["search_failure_is_not_infeasibility"])
        self.assertFalse(result["joint_certified"])
        for field in ("map_writes", "dropped_occurrences", "gate_changes", "domain_changes"):
            self.assertEqual(result[field], 0)
        self.assertNotIn("global_no", result)
        self.assertEqual(len(result["candidates"]), 1)
        item = result["candidates"][0]
        self.assertEqual(item["latent_q_exact"], ("0", "0"))
        self.assertEqual(item["coefficients"], [None, None])
        self.assertFalse(item["both_beta64_labels_certified"])
        self.assertEqual(item["shared_q_after_rounding"], "unverified_requires_joint_certificate")
        self.assertEqual(item["side_diagnostics"][0]["systems"], 1)
        self.assertEqual(item["side_diagnostics"][0]["full_scans"], 1)
        self.assertEqual(item["side_diagnostics"][0]["reason"], "fit_not_completed")
        self.assertEqual(item["side_diagnostics"][1]["reason"], "not_started")
        self.assertEqual([array_identity(array) for array in arrays], before)

    def test_predicate_is_generator_only_and_python38_compatible(self):
        tree = ast.parse(inspect.getsource(subject))
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)]
        uses = {}
        for function in functions:
            count = sum(isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                        and node.func.id == "_definite_linear_violations"
                        for node in ast.walk(function))
            if count:
                uses[function.name] = count
        self.assertEqual(uses, {"_restricted_proposal": 2})
        self.assertEqual((subject.MAX_FACES, subject.MAX_PIVOTS, subject.MAX_BITS),
                         (512, 24, 16384))
        for source in (inspect.getsource(subject), inspect.getsource(inspect.getmodule(rational))):
            local_tree = ast.parse(source)
            for node in ast.walk(local_tree):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                    self.assertFalse(isinstance(node.func.value, ast.Name)
                                     and node.func.value.id == "math"
                                     and node.func.attr == "nextafter")


if __name__ == "__main__":
    unittest.main()
