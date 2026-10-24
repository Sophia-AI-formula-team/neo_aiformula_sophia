"""Independent hand/Fraction fixtures for bounded paired-curve PROPOSALS.

Prepared statically; no test outcome is asserted by this file. The oracle below
does not call the helper's chart, half-plane, location, label or fit routines.
It checks nominal supplied binary64 world-coordinate labels, NOT the original
rounded production fit program, physical geometry, owner identity or a joint.
"""
import ast
from fractions import Fraction
import inspect
import math
import unittest
from unittest import mock

import numpy as np

from lane_mapping_lya_reference import paired_curve_fit as subject


def exact(value):
    return value if isinstance(value, Fraction) else Fraction.from_float(float(value))


def chart_direct(model, point):
    ox, oy = (exact(v) for v in model.origin)
    nx, ny = (exact(v) for v in model.normal)
    dx, dy = exact(point[0])-ox, exact(point[1])-oy
    return (ny*dx-nx*dy)/exact(model.scale), nx*dx+ny*dy


def label_direct(model, beta, point, role):
    x, y = chart_direct(model, point)
    a, b, c = (exact(v) for v in beta)
    return role*(y-a-b*x-c*x*x)


def all_labels_direct(model, beta, white, black):
    # Every occurrence, not unique points or the finite-chart subset.
    return (all(label_direct(model, beta, p, -1) >= 0 for p in white)
            and all(label_direct(model, beta, p, 1) >= 0 for p in black))


def original_p_direct(point, junction, axes):
    # Solve the ORIGINAL two-axis coordinates, not subject's plane expansion.
    px, py = (exact(v) for v in point)
    jx, jy = (exact(v) for v in junction)
    (ax, ay), (bx, by) = [tuple(exact(v) for v in row) for row in axes]
    determinant = ax*by-ay*bx
    if determinant == 0:
        return False
    u = ((px-jx)*by-(py-jy)*bx)/determinant
    v = (ax*(py-jy)-ay*(px-jx))/determinant
    return -Fraction(1, 2) <= u <= Fraction(1, 2) and -Fraction(1, 2) <= v <= Fraction(1, 2)


def finite_chart_direct(model, point):
    x, _ = chart_direct(model, point)
    return exact(model.bounds[0]) <= x <= exact(model.bounds[1])


def descriptor(array):
    return (array.dtype.str, array.shape, array.tobytes(order="C"),
            array.strides, bool(array.flags.writeable))


def diagnostic():
    return dict(systems=0, pivots=0, full_scans=0, rounding_packets=0,
                exact_label_substitutions=0, reason=None)


def straight_pair(n=6, *, bounds=((-1., 1.), (-1., 1.))):
    # L: F_L=X, local tangent x=-Y; R: F_R=-Y, tangent x=-X.
    # Six dyadic tangent positions give H=diag(29/8, 353/128) at q=(0,0).
    t = np.resize(np.array([-1., -.75, -.5, .5, .75, 1.]), n)
    left = subject.PrimaryPatch.from_arrays((0., 0.), (1., 0.), 1., bounds[0])
    right = subject.PrimaryPatch.from_arrays((0., 0.), (0., -1.), 1., bounds[1])
    centers = [np.column_stack((np.zeros(n), -t)),
               np.column_stack((-t, np.zeros(n)))]
    white = [centers[0]-np.array([.25, 0.]), centers[1]-np.array([0., -.25])]
    black = [centers[0]+np.array([.25, 0.]), centers[1]+np.array([0., -.25])]
    return [[left, right], centers, white, black, np.zeros(2), np.eye(2)]


def call_pair(data, **options):
    models, centers, white, black, junction, axes = data
    return subject.propose_pair(*models, centers, white, black, junction, axes, **options)


def kkt_direct(gram, rhs, rows, limits, beta, basis, multipliers):
    h = [[exact(v) for v in row] for row in gram]
    r = [exact(v) for v in rhs]
    a = [[exact(v) for v in row] for row in rows]
    b = [exact(v) for v in limits]
    z = [exact(v) for v in beta]
    lm = [exact(v) for v in multipliers]
    return (all(sum(ai[j]*z[j] for j in range(2)) <= bi for ai, bi in zip(a, b))
            and all(v >= 0 for v in lm)
            and all(sum(h[i][j]*z[j] for j in range(2))
                    +sum(l*a[k][i] for k, l in zip(basis, lm)) == r[i] for i in range(2))
            and all(sum(a[k][j]*z[j] for j in range(2)) == b[k] for k in basis))


class PairedCurveProposalTests(unittest.TestCase):
    def assert_no_adoption(self, result):
        self.assertFalse(result["joint_certified"])
        for key in ("map_writes", "dropped_occurrences", "gate_changes", "domain_changes"):
            self.assertEqual(result[key], 0)
        self.assertTrue(result["search_failure_is_not_infeasibility"])
        self.assertEqual(result["label_contract"], "nominal_all_WB_polynomial_extension")
        self.assertNotIn("owner_certified", result)
        self.assertNotIn("immutable", result)
        for item in result["candidates"]:
            self.assertEqual(item["shared_q_after_rounding"], "unverified_requires_joint_certificate")

    def assert_candidate_oracles(self, result, data):
        self.assert_no_adoption(result)
        models, _, white, black, junction, axes = data
        for item in result["candidates"]:
            q = tuple(Fraction(v) for v in item["latent_q_exact"])
            self.assertTrue(original_p_direct(q, junction, axes))
            self.assertTrue(all(finite_chart_direct(m, q) for m in models))
            self.assertEqual(item["both_beta64_labels_certified"],
                             all(beta is not None for beta in item["coefficients"]))
            for side, beta in enumerate(item["coefficients"]):
                if beta is not None:
                    self.assertTrue(all(math.isfinite(v) for v in beta))
                    self.assertTrue(all_labels_direct(models[side], beta, white[side], black[side]))
            for d in item["side_diagnostics"]:
                self.assertLessEqual(d["pivots"], 24)
                self.assertLessEqual(d["systems"], 1+7*24)
                self.assertLessEqual(d["full_scans"], 25)
                self.assertLessEqual(d["rounding_packets"], 27)
                self.assertLessEqual(d["exact_label_substitutions"], 27*2*len(white[0]))

    def test_baseline_hand_zero_coefficients_and_all_original_labels(self):
        data = straight_pair()
        result = call_pair(data)
        self.assertEqual(result["status"], "completed_proposal_diagnostic")
        self.assertEqual(len(result["candidates"]), 5)
        item = result["candidates"][0]
        self.assertEqual(item["latent_q_exact"], ("0", "0"))
        self.assertTrue(item["both_beta64_labels_certified"])
        for beta in item["coefficients"]:
            np.testing.assert_array_equal(beta, (0., 0., 0.))
        self.assert_candidate_oracles(result, data)

    def test_hand_original_p_five_locations_not_nearest_lane_guess(self):
        models, _, _, _, junction, axes = straight_pair()
        actual = subject._locations(models, junction, axes)
        expected = [(Fraction(0), Fraction(0)), (Fraction(-1, 4), Fraction(0)),
                    (Fraction(0), Fraction(-1, 4)), (Fraction(1, 4), Fraction(0)),
                    (Fraction(0), Fraction(1, 4))]
        self.assertEqual(actual, expected)
        self.assertTrue(all(original_p_direct(q, junction, axes) for q in actual))

    def test_hand_clipped_finite_charts_shift_all_five_original_slots(self):
        data = straight_pair(bounds=((-0.25, 0.25), (-0.125, 0.375)))
        models, _, _, _, junction, axes = data
        actual = subject._locations(models, junction, axes)
        expected = [(Fraction(-1, 8), Fraction(0)), (Fraction(-1, 4), Fraction(0)),
                    (Fraction(-1, 8), Fraction(-1, 8)), (Fraction(0), Fraction(0)),
                    (Fraction(-1, 8), Fraction(1, 8))]
        self.assertEqual(actual, expected)
        self.assertTrue(all(original_p_direct(q, junction, axes)
                            and all(finite_chart_direct(m, q) for m in models) for q in actual))
        self.assert_candidate_oracles(call_pair(data), data)

    def test_negative_original_axis_determinant_preserves_closed_p(self):
        data = straight_pair()
        data[-1] = np.array([[0., 1.], [1., 0.]])
        q = subject._locations(data[0], data[-2], data[-1])
        self.assertEqual(q, [(Fraction(0), Fraction(0)), (Fraction(0), Fraction(-1, 4)),
                             (Fraction(-1, 4), Fraction(0)), (Fraction(0), Fraction(1, 4)),
                             (Fraction(1, 4), Fraction(0))])
        self.assert_candidate_oracles(call_pair(data), data)

    def test_sheared_original_axes_no_rectangular_domain_substitution(self):
        data = straight_pair(bounds=((-2., 2.), (-2., 2.)))
        data[-1] = np.array([[1., 0.], [1., 1.]])
        q = subject._locations(data[0], data[-2], data[-1])
        self.assertEqual(q, [(Fraction(0), Fraction(0)), (Fraction(-1, 4), Fraction(0)),
                             (Fraction(-1, 4), Fraction(-1, 4)), (Fraction(1, 4), Fraction(0)),
                             (Fraction(1, 4), Fraction(1, 4))])
        self.assert_candidate_oracles(call_pair(data), data)

    def test_zero_width_closed_chart_keeps_five_duplicate_slots(self):
        data = straight_pair(bounds=((0., 0.), (0., 0.)))
        q = subject._locations(data[0], data[-2], data[-1])
        self.assertEqual(q, [(Fraction(0), Fraction(0))]*5)
        self.assert_candidate_oracles(call_pair(data), data)

    def test_empty_original_domain_is_unknown_not_global_no(self):
        data = straight_pair(bounds=((2., 3.), (-1., 1.)))
        result = call_pair(data)
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["candidates"], [])
        self.assertEqual(result["reason"], "no_original_domain_initializer_not_global_no")
        self.assert_no_adoption(result)

    def test_degenerate_original_axes_rejected_without_p_repair(self):
        data = straight_pair()
        data[-1] = np.array([[1., 0.], [2., 0.]])
        result = call_pair(data)
        self.assertEqual(result["status"], "unknown")
        self.assertIn("degenerate original cut axes", result["reason"])
        self.assert_no_adoption(result)

    def test_manual_spd_unconstrained_stationary_point(self):
        h = np.array([[2., 1.], [1., 2.]])
        rhs, rows, limits = np.array([3., 3.]), np.empty((0, 2)), np.empty(0)
        d = diagnostic()
        beta = subject._restricted_proposal(h, rhs, rows, limits, 24, d)
        np.testing.assert_array_equal(beta, (1., 1.))
        self.assertTrue(kkt_direct(h, rhs, rows, limits, beta, (), ()))
        self.assertEqual((d["systems"], d["pivots"], d["full_scans"]), (1, 0, 1))

    def test_manual_spd_single_active_kkt_with_nonidentity_h(self):
        h, rhs = np.diag([2., 4.]), np.array([4., 12.])
        rows, limits = np.array([[1., 0.]]), np.array([1.])
        d = diagnostic()
        beta = subject._restricted_proposal(h, rhs, rows, limits, 24, d)
        np.testing.assert_array_equal(beta, (1., 3.))
        self.assertTrue(kkt_direct(h, rhs, rows, limits, beta, (0,), (2.,)))
        self.assertEqual((d["systems"], d["pivots"], d["full_scans"]), (3, 1, 2))

    def test_manual_two_active_constraints_and_ordered_system_counter(self):
        h, rhs, rows, limits = np.eye(2), np.array([2., 3.]), np.eye(2), np.ones(2)
        d = diagnostic()
        beta = subject._restricted_proposal(h, rhs, rows, limits, 24, d)
        np.testing.assert_array_equal(beta, (1., 1.))
        self.assertTrue(kkt_direct(h, rhs, rows, limits, beta, (0, 1), (1., 2.)))
        self.assertEqual((d["systems"], d["pivots"], d["full_scans"]), (7, 2, 3))

    def test_manual_three_member_pool_drops_old_active_constraints(self):
        h, rhs = np.eye(2), np.array([2., 2.])
        rows, limits = np.array([[1., 0.], [0., 1.], [1., 1.]]), np.array([1., 1., 1.5])
        d = diagnostic()
        beta = subject._restricted_proposal(h, rhs, rows, limits, 24, d)
        np.testing.assert_array_equal(beta, (.75, .75))
        self.assertTrue(kkt_direct(h, rhs, rows, limits, beta, (2,), (1.25,)))
        self.assertEqual((d["systems"], d["pivots"], d["full_scans"]), (11, 3, 4))
        self.assertLessEqual(d["systems"], 169)

    def test_zero_pivot_violated_search_is_unknown_not_false_feasible(self):
        d = diagnostic()
        beta = subject._restricted_proposal(np.eye(2), np.array([2., 3.]),
                                            np.array([[1., 0.]]), np.array([1.]), 0, d)
        self.assertIsNone(beta)
        self.assertEqual(d["reason"], "pivot_budget_unknown")
        self.assertEqual((d["systems"], d["pivots"], d["full_scans"]), (1, 0, 1))

    def test_one_pivot_two_active_case_retains_unknown_budget_reason(self):
        d = diagnostic()
        beta = subject._restricted_proposal(np.eye(2), np.array([2., 3.]),
                                            np.eye(2), np.ones(2), 1, d)
        self.assertIsNone(beta)
        self.assertEqual(d["reason"], "pivot_budget_unknown")
        self.assertEqual((d["systems"], d["pivots"], d["full_scans"]), (3, 1, 2))

    def test_inconsistent_constraints_return_pool_unknown_not_infeasibility_proof(self):
        d = diagnostic()
        beta = subject._restricted_proposal(np.eye(2), np.array([2., 0.]),
                                            np.array([[1., 0.], [-1., 0.]]),
                                            np.array([0., -1.]), 24, d)
        self.assertIsNone(beta)
        self.assertEqual(d["reason"], "restricted_float_pool_unknown")
        self.assertEqual((d["systems"], d["pivots"], d["full_scans"]), (7, 2, 2))

    def test_failed_kkt_system_attempt_is_counted_not_hidden(self):
        original = np.linalg.solve
        calls = []
        def fail_active(matrix, rhs):
            calls.append(matrix.shape)
            if len(matrix) > 2:
                raise np.linalg.LinAlgError("synthetic singular active system")
            return original(matrix, rhs)
        d = diagnostic()
        with mock.patch.object(subject.np.linalg, "solve", side_effect=fail_active):
            beta = subject._restricted_proposal(np.eye(2), np.array([2., 3.]),
                                                np.array([[1., 0.]]), np.array([1.]), 24, d)
        self.assertIsNone(beta)
        self.assertEqual(len(calls), 3)
        self.assertEqual(d["systems"], 3)
        self.assertEqual(d["reason"], "restricted_float_pool_unknown")

    def test_rank_deficient_reduced_system_has_no_invented_ridge(self):
        d = diagnostic()
        beta = subject._restricted_proposal(np.array([[1., 0.], [0., 0.]]),
                                            np.zeros(2), np.empty((0, 2)), np.empty(0), 24, d)
        self.assertIsNone(beta)
        self.assertEqual(d["reason"], "reduced_rank_unknown")
        self.assertEqual(d["systems"], 0)

    def test_negative_reduced_eigenvalue_is_unknown(self):
        d = diagnostic()
        beta = subject._restricted_proposal(np.diag([1., -1.]), np.zeros(2),
                                            np.empty((0, 2)), np.empty(0), 24, d)
        self.assertIsNone(beta)
        self.assertEqual(d["reason"], "reduced_rank_unknown")

    def test_hand_label_orientation_white_negative_black_positive(self):
        model = subject.PrimaryPatch.from_arrays((0., 0.), (0., 1.), 1., (-1., 1.))
        white, black = np.array([[0., -.25]]), np.array([[0., .25]])
        beta = (0., 0., 0.)
        self.assertTrue(all_labels_direct(model, beta, white, black))
        self.assertTrue(subject._labels(model, beta, white, black))
        self.assertFalse(all_labels_direct(model, beta, black, white))
        self.assertFalse(subject._labels(model, beta, black, white))

    def test_closed_zero_residual_uses_exact_substitution_not_epsilon(self):
        model = subject.PrimaryPatch.from_arrays((0., 0.), (0., 1.), 1., (-1., 1.))
        t = np.array([0., 1., 2., -1.])
        beta = (1., .5, .25)
        points = np.column_stack((t, 1.+.5*t+.25*t*t))
        d = diagnostic()
        self.assertTrue(all_labels_direct(model, beta, points, points))
        self.assertTrue(subject._labels(model, beta, points, points, d))
        self.assertGreaterEqual(d["exact_label_substitutions"], 8)

    def test_one_ulp_white_violation_cannot_be_certified(self):
        model = subject.PrimaryPatch.from_arrays((0., 0.), (0., 1.), 1., (-1., 1.))
        white, black = np.array([[0., float(np.nextafter(1., np.inf))]]), np.array([[0., 1.]])
        d = diagnostic()
        self.assertFalse(all_labels_direct(model, (1., 0., 0.), white, black))
        self.assertFalse(subject._labels(model, (1., 0., 0.), white, black, d))
        self.assertGreaterEqual(d["exact_label_substitutions"], 1)

    def test_off_extent_bad_label_is_not_clipped_or_deduplicated(self):
        model = subject.PrimaryPatch.from_arrays((0., 0.), (0., 1.), 1., (-.25, .25))
        white = np.array([[0., -.25], [2., 1.], [2., 1.]])
        black = np.array([[0., .25], [2., 2.], [2., 2.]])
        self.assertFalse(finite_chart_direct(model, white[1]))
        self.assertFalse(all_labels_direct(model, (0., 0., 0.), white, black))
        self.assertFalse(subject._labels(model, (0., 0., 0.), white, black))

    def test_interval_overflow_falls_back_to_exact_nominal_zero_polynomial(self):
        smallest = float(np.nextafter(0., np.inf))
        model = subject.PrimaryPatch.from_arrays((0., 0.), (0., 1.), smallest, (-1., 1.))
        white, black = np.array([[-1., -.25]]), np.array([[-1., .25]])
        d = diagnostic()
        self.assertTrue(all_labels_direct(model, (0., 0., 0.), white, black))
        self.assertTrue(subject._labels(model, (0., 0., 0.), white, black, d))
        self.assertEqual(d["exact_label_substitutions"], 2)

    def test_interval_overflow_does_not_hide_tiny_quadratic_sign_violation(self):
        smallest = float(np.nextafter(0., np.inf))
        model = subject.PrimaryPatch.from_arrays((0., 0.), (0., 1.), smallest, (-1., 1.))
        white, black = np.array([[-1., -.25]]), np.array([[-1., .25]])
        beta = (0., 0., smallest)
        self.assertFalse(all_labels_direct(model, beta, white, black))
        self.assertFalse(subject._labels(model, beta, white, black, diagnostic()))

    def test_nonfinite_coefficient_packet_is_never_label_certified(self):
        model = straight_pair()[0][0]
        for beta in ((math.nan, 0., 0.), (0., math.inf, 0.), (0., 0., -math.inf)):
            with self.subTest(beta=beta):
                self.assertFalse(subject._labels(model, beta, np.zeros((1, 2)), np.zeros((1, 2))))

    def test_fit_at_center_checks_all27_adjacent_packets_but_no_joint(self):
        data = straight_pair()
        model, cs, ws, bs = data[0][0], data[1][0], data[2][0], data[3][0]
        d = diagnostic()
        beta = subject._fit_at_q(model, cs, ws, bs, (Fraction(0), Fraction(0)), 24, d)
        np.testing.assert_array_equal(beta, (0., 0., 0.))
        self.assertTrue(all_labels_direct(model, beta, ws, bs))
        self.assertEqual((d["systems"], d["pivots"], d["full_scans"], d["rounding_packets"]),
                         (1, 0, 1, 27))
        self.assertIsNone(d["reason"])

    def test_fit_outside_original_finite_chart_stops_before_system(self):
        model = subject.PrimaryPatch.from_arrays((0., 0.), (0., 1.), 1., (-.25, .25))
        _, cs, ws, bs, _, _ = straight_pair()
        d = diagnostic()
        self.assertIsNone(subject._fit_at_q(model, cs[0], ws[0], bs[0],
                                           (Fraction(1, 2), Fraction(0)), 24, d))
        self.assertEqual(d["reason"], "outside_original_finite_chart")
        self.assertEqual(d["systems"], 0)

    def test_nonfinite_float_objective_is_unknown_without_certified_packet(self):
        model = subject.PrimaryPatch.from_arrays((0., 0.), (0., 1.), 1., (-1., 1.))
        cs = np.column_stack((np.full(6, 1.e200), np.zeros(6)))
        ws, bs = cs-np.array([0., .25]), cs+np.array([0., .25])
        d = diagnostic()
        with np.errstate(all="ignore"):
            answer = subject._fit_at_q(model, cs, ws, bs, (Fraction(0), Fraction(0)), 24, d)
        self.assertIsNone(answer)
        self.assertEqual(d["reason"], "nonfinite_objective_or_labels")
        self.assertEqual(d["systems"], 0)
        self.assertEqual(d["rounding_packets"], 0)

    def test_no_certified_rounding_packet_retains27_attempts_and_unknown_reason(self):
        data = straight_pair()
        d = diagnostic()
        with mock.patch.object(subject, "_labels", return_value=False):
            beta = subject._fit_at_q(data[0][0], data[1][0], data[2][0], data[3][0],
                                     (Fraction(0), Fraction(0)), 24, d)
        self.assertIsNone(beta)
        self.assertEqual(d["rounding_packets"], 27)
        self.assertEqual(d["reason"], "no_certified_27_packet")

    def test_whole_support_duplicate_order_and_copy_contract(self):
        data = straight_pair(n=7)
        permutation = [6, 2, 0, 3, 5, 1, 4]  # repeats original first point, not unique support.
        for group in data[1:4]:
            for side in range(2):
                group[side] = group[side][permutation]
        expected = [[descriptor(group[side]) for group in data[1:4]] for side in range(2)]
        before = [descriptor(v) for group in data[1:4] for v in group]
        calls, original = [], subject._fit_at_q
        def traced(model, cs, ws, bs, q, pivots, d=None):
            calls.append([descriptor(v) for v in (cs, ws, bs)])
            return original(model, cs, ws, bs, q, pivots, d)
        with mock.patch.object(subject, "_fit_at_q", side_effect=traced):
            result = call_pair(data)
        self.assertEqual(calls, expected*5)
        self.assertEqual(before, [descriptor(v) for group in data[1:4] for v in group])
        self.assert_candidate_oracles(result, data)

    def test_all_off_extent_support_labels_still_enter_actual_proposal(self):
        data = straight_pair(bounds=((-0.25, 0.25), (-0.25, 0.25)))
        self.assertTrue(all(not finite_chart_direct(data[0][0], p) for p in data[2][0]))
        result = call_pair(data)
        self.assert_candidate_oracles(result, data)
        self.assertTrue(result["candidates"][0]["both_beta64_labels_certified"])

    def test_contradictory_off_extent_support_prevents_both_label_flag(self):
        data = straight_pair(bounds=((-0.25, 0.25), (-0.25, 0.25)))
        data[2][0][0, 0] = 1.  # white requires f>=1 at same t whose black requires f<=.25.
        result = call_pair(data)
        self.assertEqual(result["status"], "completed_proposal_diagnostic")
        self.assertTrue(all(item["coefficients"][0] is None for item in result["candidates"]))
        self.assertTrue(all(not item["both_beta64_labels_certified"] for item in result["candidates"]))
        self.assert_no_adoption(result)

    def test_wrong_normal_sign_cannot_turn_reversed_wb_order_into_certificate(self):
        data = straight_pair()
        data[0][0] = subject.PrimaryPatch.from_arrays((0., 0.), (-1., 0.), 1., (-1., 1.))
        result = call_pair(data)
        self.assertTrue(all(item["coefficients"][0] is None for item in result["candidates"]))
        self.assert_no_adoption(result)

    def test_nonunit_numeric_chart_is_preserved_not_normalized(self):
        model = subject.PrimaryPatch.from_arrays((.125, -.25), (2., 0.), 2., (-1., 1.))
        self.assertEqual(model.normal, (2., 0.))
        self.assertEqual(model.scale, 2.)
        self.assertEqual(chart_direct(model, (.375, -.75)), (Fraction(1, 2), Fraction(1, 2)))

    def test_readonly_noncontiguous_inputs_remain_byte_unchanged(self):
        data = straight_pair()
        for group in data[1:4]:
            for side in range(2):
                padded = np.zeros((6, 4))
                padded[:, ::2] = group[side]
                group[side] = padded[:, ::2]
                group[side].setflags(write=False)
        before = [descriptor(v) for group in data[1:4] for v in group]
        result = call_pair(data)
        self.assertEqual(before, [descriptor(v) for group in data[1:4] for v in group])
        self.assert_candidate_oracles(result, data)

    def test_result_is_mutable_diagnostic_not_immutable_or_adopted_map(self):
        result = call_pair(straight_pair())
        self.assert_no_adoption(result)
        result["candidates"].append({"local_probe": True})
        self.assertEqual(result["candidates"][-1], {"local_probe": True})
        result["local_note"] = "test owns this returned mutable dictionary"
        self.assertNotIn("immutable", result)

    def test_initial_solve_exception_preserves_appended_partial_counters(self):
        with mock.patch.object(subject.np.linalg, "solve",
                               side_effect=np.linalg.LinAlgError("synthetic initial solve")):
            result = call_pair(straight_pair())
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(len(result["candidates"]), 1)
        item = result["candidates"][0]
        self.assertEqual(item["coefficients"], [None, None])
        self.assertEqual(item["side_diagnostics"][0]["systems"], 1)
        self.assertEqual(item["side_diagnostics"][1]["systems"], 0)
        self.assertFalse(item["both_beta64_labels_certified"])
        self.assert_no_adoption(result)

    def test_later_side_exception_keeps_first_packet_without_complete_pair_claim(self):
        calls, original = [], subject._fit_at_q
        def stop_second(model, cs, ws, bs, q, pivots, d=None):
            calls.append(q)
            if len(calls) == 2:
                d["systems"] += 1
                d["reason"] = "synthetic side checkpoint"
                raise ValueError("synthetic side checkpoint")
            return original(model, cs, ws, bs, q, pivots, d)
        with mock.patch.object(subject, "_fit_at_q", side_effect=stop_second):
            result = call_pair(straight_pair())
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["reason"], "synthetic side checkpoint")
        self.assertEqual(len(result["candidates"]), 1)
        item = result["candidates"][0]
        self.assertIsNotNone(item["coefficients"][0])
        self.assertIsNone(item["coefficients"][1])
        self.assertEqual(item["side_diagnostics"][1]["systems"], 1)
        self.assertEqual(item["side_diagnostics"][1]["reason"], "synthetic side checkpoint")
        self.assertFalse(item["both_beta64_labels_certified"])
        self.assert_no_adoption(result)

    def test_five_occurrences_rejected_without_padding_to_six(self):
        result = call_pair(straight_pair(n=5))
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["candidates"], [])
        self.assertIn("six..512", result["reason"])
        self.assert_no_adoption(result)

    def test_513_occurrences_rejected_without_truncation(self):
        result = call_pair(straight_pair(n=513))
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["candidates"], [])
        self.assertIn("six..512", result["reason"])
        self.assert_no_adoption(result)

    def test_512_occurrences_are_passed_complete_without_unique_deduplication(self):
        data = straight_pair(n=512)
        counts, original = [], subject._fit_at_q
        def traced(model, cs, ws, bs, q, pivots, d=None):
            counts.append((len(cs), len(ws), len(bs)))
            return original(model, cs, ws, bs, q, pivots, d)
        with mock.patch.object(subject, "_fit_at_q", side_effect=traced):
            result = call_pair(data)
        self.assertEqual(result["status"], "completed_proposal_diagnostic")
        self.assertEqual(counts, [(512, 512, 512)]*10)
        self.assert_no_adoption(result)

    def test_ribbon_double_band_counts_rejected_not_called_primary_by_count(self):
        data = straight_pair()
        data[2][0] = np.repeat(data[2][0], 2, axis=0)
        data[3][0] = np.repeat(data[3][0], 2, axis=0)
        result = call_pair(data)
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["candidates"], [])
        self.assertIn("same-support", result["reason"])
        self.assert_no_adoption(result)

    def test_numeric_equal_count_cannot_authenticate_ribbon_or_actual_owner(self):
        # Only numeric records exist in this API; a caller must verify role/owner.
        result = call_pair(straight_pair())
        self.assertNotIn("owner_certified", result)
        self.assertNotIn("primary_source_authenticated", result)
        self.assert_no_adoption(result)

    def test_not_exactly_two_groups_rejected(self):
        data = straight_pair()
        data[1].pop()
        result = call_pair(data)
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["candidates"], [])
        self.assertIn("two paired-primary", result["reason"])

    def test_wrong_point_shape_rejected_without_reshape(self):
        data = straight_pair()
        data[1][0] = np.zeros((6, 3))
        result = call_pair(data)
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["candidates"], [])

    def test_complex_geometry_rejected_not_real_part_only(self):
        data = straight_pair()
        data[1][0] = data[1][0].astype(np.complex128)
        result = call_pair(data)
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["candidates"], [])

    def test_boolean_geometry_rejected_not_converted_to_positions(self):
        data = straight_pair()
        data[1][0] = np.zeros((6, 2), dtype=bool)
        result = call_pair(data)
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["candidates"], [])

    def test_nonfinite_geometry_rejected_before_locations_or_solves(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                data = straight_pair()
                data[2][0][0, 1] = value
                result = call_pair(data)
                self.assertEqual(result["status"], "unknown")
                self.assertEqual(result["candidates"], [])

    def test_integer_geometry_rounding_rejected_not_silently_changed(self):
        data = straight_pair()
        data[1][0] = np.zeros((6, 2), dtype=np.int64)
        data[1][0][0, 0] = (1 << 53)+1
        result = call_pair(data)
        self.assertEqual(result["status"], "unknown")
        self.assertIn("integer geometry would be rounded", result["reason"])

    def test_float32_binary_values_are_exactly_widened(self):
        actual = subject._array(np.array([[.125, -.25]], dtype=np.float32), (1, 2))
        self.assertEqual(actual.dtype, np.dtype(np.float64))
        np.testing.assert_array_equal(actual, [[.125, -.25]])

    def test_invalid_chart_shape_scale_and_zero_normal_rejected(self):
        for origin, normal, scale, bounds in (
                ((0.,), (1., 0.), 1., (-1., 1.)),
                ((0., 0.), (0., 0.), 1., (-1., 1.)),
                ((0., 0.), (1., 0.), 0., (-1., 1.)),
                ((0., 0.), (1., 0.), -1., (-1., 1.)),
                ((0., 0.), (1., 0.), True, (-1., 1.)),
                ((0., 0.), (1., 0.), 1., (1., -1.))):
            with self.subTest(origin=origin, normal=normal, scale=scale, bounds=bounds):
                with self.assertRaises(ValueError):
                    subject.PrimaryPatch.from_arrays(origin, normal, scale, bounds)

    def test_direct_malformed_dataclass_is_revalidated_by_public_api(self):
        data = straight_pair()
        data[0][0] = subject.PrimaryPatch((0., 0.), (0., 0.), 1., (-1., 1.))
        result = call_pair(data)
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["candidates"], [])
        self.assert_no_adoption(result)

    def test_invalid_pivot_types_or_bounds_rejected_before_search(self):
        for budget in (-1, 25, True, 1., np.int64(1)):
            with self.subTest(budget=budget):
                result = call_pair(straight_pair(), max_pivots=budget)
                self.assertEqual(result["status"], "unknown")
                self.assertEqual(result["candidates"], [])
                self.assertIn("24-pivot", result["reason"])

    def test_zero_pivot_feasible_center_remains_a_label_only_proposal(self):
        data = straight_pair()
        result = call_pair(data, max_pivots=0)
        item = result["candidates"][0]
        self.assertTrue(item["both_beta64_labels_certified"])
        for d in item["side_diagnostics"]:
            self.assertEqual(d["pivots"], 0)
            self.assertLessEqual(d["systems"], 1)
            self.assertLessEqual(d["full_scans"], 1)
        self.assert_candidate_oracles(result, data)

    def test_all_rank_unknown_slots_do_not_claim_global_infeasibility(self):
        data = straight_pair()
        for group in data[1:4]:
            for side in range(2):
                group[side][:] = group[side][0]
        result = call_pair(data)
        self.assertEqual(result["status"], "completed_proposal_diagnostic")
        self.assertTrue(all(not p["both_beta64_labels_certified"] for p in result["candidates"]))
        self.assertTrue(all(d["reason"] == "reduced_rank_unknown"
                            for p in result["candidates"] for d in p["side_diagnostics"]))
        self.assert_no_adoption(result)

    def test_exact_arithmetic_checkpoint_boundary_is_not_a_physical_no(self):
        self.assertEqual(subject.MAX_BITS, 16384)
        self.assertEqual(subject._number(Fraction((1 << 16384)-1)), Fraction((1 << 16384)-1))
        self.assertEqual(subject._number(Fraction(1, 1 << 16383)), Fraction(1, 1 << 16383))
        with self.assertRaises(ValueError):
            subject._number(Fraction(1 << 16384))
        with self.assertRaises(ValueError):
            subject._number(Fraction(1, 1 << 16384))

    def test_search_caps_are_original_declared_constants_not_new_safety_gates(self):
        self.assertEqual((subject.MAX_FACES, subject.MAX_PIVOTS), (512, 24))
        result = call_pair(straight_pair())
        self.assertEqual(result["objective_scope"],
                         "float_nominal_saved_world_center_LS_proposal_not_original_FP_Gram_rhs")
        self.assert_no_adoption(result)

    def test_python38_sources_do_not_call_missing_math_neighbor_api(self):
        # Source inspection occurs only in the future approved unit process.
        # This is not a import-time Python-version skip or a fallback fitter.
        for source in (inspect.getsource(subject), inspect.getsource(inspect.getmodule(exact))):
            tree = ast.parse(source)
            missing = [node for node in ast.walk(tree)
                       if isinstance(node, ast.Attribute) and node.attr == "nextafter"
                       and isinstance(node.value, ast.Name) and node.value.id == "math"]
            self.assertEqual(missing, [])


if __name__ == "__main__":
    unittest.main()

