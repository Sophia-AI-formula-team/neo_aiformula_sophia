"""Independent nominal one-box joint oracles; prepared, not executed here.

Portable Python3.8/NumPy/unittest source. No private diagnostic paths, extractor,
ROS, native fitter, real observations or existing test helpers are imported.
Own exact saved-world polynomial and exact central-difference derivatives supply
the oracle; no output of the module's F/gradient implementation is an expected
value. Every returned YES is checked by a separate exact original-box K image.
A synthetic proof is not admission, topology/uniqueness, physical accuracy or
runtime/two-lap evidence. Root owns later full-read/pinned bounded execution.
"""
from dataclasses import dataclass, replace
from fractions import Fraction as F
from itertools import product
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
from lane_mapping_lya_reference import paired_curve_joint as joint


@dataclass(frozen=True)
class ManualPatch:
    origin: tuple = (0., 0.)
    normal: tuple = (0., 1.)
    scale: float = 1.
    bounds: tuple = (-2., 2.)


def rat(value):
    return value if isinstance(value, F) else F.from_float(float(value))


def point(q):
    return tuple(rat(v) for v in q)


def original_corners(center, radius):
    center, radius = point(center), rat(radius)
    return tuple((center[0] + a * radius, center[1] + b * radius)
                 for a, b in product((-1, 1), repeat=2))


def direct_chart(model, q):
    """Own saved-normal/world substitution, not the module's chart function."""
    ox, oy = point(model.origin)
    nx, ny = point(model.normal)
    dx, dy = q[0] - ox, q[1] - oy
    return (ny * dx - nx * dy) / rat(model.scale), nx * dx + ny * dy


def direct_polynomial(model, coefficients, q):
    x, y = direct_chart(model, q)
    a, b, c = tuple(rat(v) for v in coefficients)
    return y - a - b * x - c * x * x


def exact_gradient(model, coefficients, q):
    """Quadratic's exact derivative via symmetric difference with step ONE."""
    result = []
    for axis in range(2):
        positive, negative = list(q), list(q)
        positive[axis] += 1
        negative[axis] -= 1
        result.append((direct_polynomial(model, coefficients, positive) -
                       direct_polynomial(model, coefficients, negative)) / 2)
    return tuple(result)


def iv_add(a, b):
    return a[0] + b[0], a[1] + b[1]


def iv_sub(a, b):
    return a[0] - b[1], a[1] - b[0]


def iv_mul(a, b):
    values = [x * y for x in a for y in b]
    return min(values), max(values)


def iv_point(v):
    return v, v


def exact_krawczyk_image(models, coefficients, center, radius, c):
    """Exact oracle with affine derivative extrema at ORIGINAL box corners.

    C is the reported saved binary64 matrix interpreted exactly, not an
    assumed exact inverse. No NumPy linear algebra or module helpers are used.
    """
    center, radius = point(center), rat(radius)
    corners = original_corners(center, radius)
    c = tuple(tuple(rat(v) for v in row) for row in c)
    values = [direct_polynomial(m, beta, center) for m, beta in zip(models, coefficients)]
    jacobian = []
    for m, beta in zip(models, coefficients):
        gradients = [exact_gradient(m, beta, q) for q in corners]
        jacobian.append(tuple((min(g[k] for g in gradients), max(g[k] for g in gradients))
                              for k in range(2)))
    residual = []
    for i in range(2):
        row = []
        for k in range(2):
            cj = iv_add(iv_mul(iv_point(c[i][0]), jacobian[0][k]),
                        iv_mul(iv_point(c[i][1]), jacobian[1][k]))
            row.append(iv_sub(iv_point(F(i == k)), cj))
        residual.append(row)
    base = [center[i] - sum((c[i][k] * values[k] for k in range(2)), F(0)) for i in range(2)]
    delta = (-radius, radius)
    return tuple(iv_add(iv_point(base[i]), iv_add(iv_mul(residual[i][0], delta),
                                                iv_mul(residual[i][1], delta)))
                 for i in range(2))


def original_uv(q, junction, axes):
    j, (a, b) = point(junction), tuple(point(row) for row in axes)
    determinant = a[0] * b[1] - a[1] * b[0]
    dx, dy = q[0] - j[0], q[1] - j[1]
    return ((dx * b[1] - dy * b[0]) / determinant,
            (a[0] * dy - a[1] * dx) / determinant)


def snapshot(value):
    """Bytes include signed zeros; capture shape/stride/readonly state too."""
    if isinstance(value, np.ndarray):
        return (value.dtype.str, value.shape, value.strides, value.flags.writeable,
                value.tobytes(order="A"))
    if isinstance(value, ManualPatch):
        return tuple(snapshot(v) for v in (value.origin, value.normal, value.scale, value.bounds))
    if isinstance(value, (list, tuple)):
        return tuple(snapshot(v) for v in value)
    if isinstance(value, (float, np.floating)):
        return np.asarray(value, dtype=np.float64).tobytes()
    return value


def affine_models(origin=(0., 0.)):
    return ManualPatch(origin=origin, normal=(1., 0.)), ManualPatch(origin=origin, normal=(0., 1.))


ZERO_BETA = ((0., 0., 0.), (0., 0., 0.))
DEFAULT_AXES = ((2., 0.), (0., 2.))


class PairedCurveJointTests(unittest.TestCase):
    def invoke(self, left, right, beta=ZERO_BETA, junction=(0., 0.), axes=DEFAULT_AXES,
               center=(0., 0.), radius=.125):
        inputs = (left, right, beta, junction, axes, center, radius)
        before = snapshot(inputs)
        result = joint.certify_pair(*inputs)
        self.assertEqual(before, snapshot(inputs))
        self.assertIn(result["status"], ("yes", "unknown"))
        self.assertEqual(result["box_count"], 1)
        self.assertEqual(result["map_writes"], 0)
        self.assertEqual(result["gate_changes"], 0)
        self.assertEqual(result["domain_changes"], 0)
        self.assertEqual(result["claim"], "nominal_joint_existence_not_uniqueness_or_physical_truth")
        if result["status"] == "yes":
            self.check_yes((left, right), beta, junction, axes, center, radius, result)
        return result

    def check_yes(self, models, beta, junction, axes, center, radius, result):
        center, radius = point(center), rat(radius)
        inner = tuple(tuple(rat(v) for v in row) for row in result["original_box_inward_bounds"])
        outer = tuple(tuple(rat(v) for v in row) for row in result["model_evaluation_outward_bounds"])
        image = tuple(tuple(rat(v) for v in row) for row in result["krawczyk_image"])
        c = tuple(tuple(rat(v) for v in row) for row in result["preconditioner"])
        det = c[0][0] * c[1][1] - c[0][1] * c[1][0]
        det_interval = tuple(rat(v) for v in result["preconditioner_det_enclosure"])
        self.assertNotEqual(det, 0)
        self.assertLessEqual(det_interval[0], det)
        self.assertGreaterEqual(det_interval[1], det)
        self.assertTrue(det_interval[1] < 0 or det_interval[0] > 0)
        exact_image = exact_krawczyk_image(models, beta, center, radius, c)
        for i in range(2):
            lo, hi = center[i] - radius, center[i] + radius
            self.assertLessEqual(outer[i][0], lo)
            self.assertGreaterEqual(outer[i][1], hi)
            self.assertLessEqual(lo, inner[i][0])
            self.assertLessEqual(inner[i][1], hi)
            self.assertLess(inner[i][0], image[i][0])
            self.assertLessEqual(image[i][0], exact_image[i][0])
            self.assertGreaterEqual(image[i][1], exact_image[i][1])
            self.assertLess(image[i][1], inner[i][1])
            self.assertLess(lo, exact_image[i][0])
            self.assertLess(exact_image[i][1], hi)
        for q in product(*outer):
            u, v = original_uv(q, junction, axes)
            self.assertTrue(-F(1, 2) <= u <= F(1, 2))
            self.assertTrue(-F(1, 2) <= v <= F(1, 2))
            for model in models:
                x, _ = direct_chart(model, q)
                self.assertTrue(rat(model.bounds[0]) <= x <= rat(model.bounds[1]))

    def yes(self, *args, **kwargs):
        result = self.invoke(*args, **kwargs)
        self.assertEqual(result["status"], "yes", result.get("reason"))
        return result

    def unknown(self, *args, **kwargs):
        result = self.invoke(*args, **kwargs)
        self.assertEqual(result["status"], "unknown")
        self.assertTrue(result.get("reason"))
        return result

    def test_literal_affine_root_origin_exact_k_oracle(self):
        models = affine_models()
        self.assertEqual(tuple(direct_polynomial(m, b, (F(0), F(0)))
                               for m, b in zip(models, ZERO_BETA)), (F(0), F(0)))
        self.yes(*models)

    def test_literal_translated_affine_root(self):
        root = (.25, -.375)
        models = affine_models(root)
        self.yes(*models, junction=root, center=root)
        self.assertEqual(tuple(direct_polynomial(m, b, point(root))
                               for m, b in zip(models, ZERO_BETA)), (F(0), F(0)))

    def test_center_is_not_forced_to_be_a_root(self):
        models = affine_models()
        center = (.015625, -.03125)
        self.assertTrue(all(direct_polynomial(m, b, point(center)) != 0
                            for m, b in zip(models, ZERO_BETA)))
        self.yes(*models, center=center)

    def test_one_curved_quadratic_literal_rational_root(self):
        left, right = ManualPatch(normal=(0., 1.)), ManualPatch(normal=(1., 0.))
        beta = ((0., 0., 1.), (.5, 0., 0.))
        root = (.5, .25)  # y=x^2 and x=1/2.
        self.assertEqual(tuple(direct_polynomial(m, b, point(root))
                               for m, b in zip((left, right), beta)), (F(0), F(0)))
        self.yes(left, right, beta, axes=((4., 0.), (0., 4.)), center=root, radius=1./128.)

    def test_two_curved_models_nonzero_center_residual_exact_k_proof(self):
        models = (ManualPatch(normal=(0., 1.)), ManualPatch(normal=(1., 0.)))
        beta = ((0., 0., 1.), (.5, 0., .25))
        center = (.52, .27)  # Not claimed as the irrational common root.
        self.assertTrue(all(direct_polynomial(m, b, point(center)) != 0
                            for m, b in zip(models, beta)))
        self.yes(*models, beta=beta, axes=((4., 0.), (0., 4.)), center=center, radius=.05)

    def test_tiny_residual_parallel_displaced_lines_have_no_joint(self):
        left = right = ManualPatch(normal=(1., 0.))
        offset = math.ldexp(1., -45)
        beta = ((0., 0., 0.), (offset, 0., 0.))
        # For every exact q, F_right-F_left=-offset, hence no common zero.
        for q in ((F(0), F(0)), (F(1, 4), -F(1, 8))):
            self.assertEqual(direct_polynomial(right, beta[1], q) -
                             direct_polynomial(left, beta[0], q), -rat(offset))
        self.unknown(left, right, beta)

    def test_tangent_true_root_is_unknown_not_false_no(self):
        models = (ManualPatch(normal=(0., 1.)), ManualPatch(normal=(0., 1.)))
        beta = ((0., 0., 1.), (0., 0., 0.))
        self.assertEqual(exact_gradient(models[0], beta[0], (F(0), F(0))),
                         exact_gradient(models[1], beta[1], (F(0), F(0))))
        self.unknown(*models, beta=beta)

    def test_coincident_curves_do_not_prove_unique_joint(self):
        model = ManualPatch(normal=(0., 1.))
        beta = ((0., 0., 1.), (0., 0., 1.))
        self.unknown(model, model, beta)

    def test_wrong_root_box_never_passes_small_residual_shortcut(self):
        models = tuple(replace(m, bounds=(-3., 3.)) for m in affine_models())
        self.unknown(*models, junction=(1., 1.), center=(1., 1.), radius=.125)

    def test_root_at_original_box_boundary_cannot_be_strict(self):
        models = affine_models()
        result = self.unknown(*models, center=(.125, 0.), radius=.125)
        self.assertIn("strict original-box inclusion", result["reason"])

    def test_near_parallel_but_certifiable_affine_pair(self):
        delta = math.ldexp(1., -20)
        models = (ManualPatch(normal=(1., 0.)), ManualPatch(normal=(1., delta)))
        self.yes(*models)

    def test_extreme_near_parallel_pair_is_conservative_unknown(self):
        delta = math.ldexp(1., -1022)
        models = (ManualPatch(normal=(1., 0.)), ManualPatch(normal=(1., delta)))
        self.unknown(*models)

    def test_tiny_well_conditioned_normal_scale_can_certify(self):
        tiny = math.ldexp(1., -500)
        models = (ManualPatch(normal=(tiny, 0.)), ManualPatch(normal=(0., tiny)))
        self.yes(*models)

    def test_subnormal_normal_cannot_create_infinite_inverse_yes(self):
        tiny = float(np.nextafter(0., 1.))
        models = (ManualPatch(normal=(tiny, 0.)), ManualPatch(normal=(0., tiny)))
        self.unknown(*models)

    def test_smallest_positive_radius_has_no_certified_strict_interior(self):
        models = affine_models()
        result = self.unknown(*models, radius=float(np.nextafter(0., 1.)))
        self.assertIn("no certified strict interior", result["reason"])

    def test_huge_translated_finite_root_is_checked_without_recentering_inputs(self):
        root = (math.ldexp(1., 500), -math.ldexp(1., 500))
        radius, extent, axis = math.ldexp(1., 460), math.ldexp(1., 462), math.ldexp(1., 464)
        models = tuple(replace(m, bounds=(-extent, extent)) for m in affine_models(root))
        self.yes(*models, junction=root, axes=((axis, 0.), (0., axis)), center=root, radius=radius)

    def test_overflowing_original_P_determinant_is_unknown(self):
        huge = math.ldexp(1., 1020)
        result = self.unknown(*affine_models(), axes=((huge, 0.), (0., huge)))
        self.assertIn("determinant enclosure", result["reason"])

    def test_huge_normal_preconditioner_det_underflow_is_not_yes(self):
        huge = math.ldexp(1., 1000)
        models = (ManualPatch(normal=(huge, 0.)), ManualPatch(normal=(0., huge)))
        self.unknown(*models)

    def test_entire_outer_box_must_fit_original_P_no_clamp_or_growth(self):
        models = affine_models()
        for half_axis in (.0625, .125):
            with self.subTest(half_axis=half_axis):
                # .125 touches exact P; the larger numerical outer box still fails.
                result = self.unknown(*models, axes=((2.*half_axis, 0.), (0., 2.*half_axis)))
                self.assertIn("inside original P", result["reason"])

    def test_entire_outer_box_must_fit_both_original_finite_charts(self):
        for bound in (.0625, .125):
            with self.subTest(bound=bound):
                left, right = affine_models()
                left = replace(left, bounds=(-bound, bound))
                result = self.unknown(left, right)
                self.assertIn("original finite chart", result["reason"])

    def test_original_P_uncertain_denominator_sign_never_divides_through_pole(self):
        other = float(np.nextafter(1., np.inf))
        axes = ((1., 1.), (1., other))
        radius = math.ldexp(1., -60)
        # Exact P is nondegenerate and contains this tiny box; outward determinant
        # uncertainty includes zero. Do not loosen it because a root is visible.
        for q in original_corners((0., 0.), radius):
            uv = original_uv(q, (0., 0.), axes)
            self.assertTrue(all(-F(1, 2) <= v <= F(1, 2) for v in uv))
        result = self.unknown(*affine_models(), axes=axes, radius=radius)
        self.assertIn("denominator sign", result["reason"])

    def test_original_P_reflection_and_skew_are_not_normalized(self):
        for axes in (((-2., 0.), (1., 2.)), ((2., 1.), (1., 3.))):
            with self.subTest(axes=axes):
                self.yes(*affine_models(), axes=axes)

    def test_nonunit_saved_normals_and_unequal_scales(self):
        models = (ManualPatch(normal=(3., 0.), scale=2.),
                  ManualPatch(normal=(0., -4.), scale=.5))
        self.yes(*models)

    def test_scalar_and_geometry_shapes_are_exactly_required(self):
        left, right = affine_models()
        cases = (
            {"radius": [0.125]}, {"radius": np.array([[.125]])},
            {"center": [[0., 0.]]}, {"junction": [[0., 0.]]},
            {"axes": [[1., 0., 0.], [0., 1., 0.]]},
            {"beta": [0., 0., 0.]}, {"beta": np.zeros((2, 3, 1))},
        )
        for overrides in cases:
            with self.subTest(keys=tuple(overrides)):
                self.unknown(left, right, **overrides)
        for bad in (replace(left, bounds=(-1., 1., 2.)), replace(left, scale=[1.]),
                    replace(left, normal=(1.,)), replace(left, origin=(0.,))):
            with self.subTest(model=bad):
                self.unknown(bad, right)

    def test_invalid_nonreal_nonfinite_boolean_integerloss_inputs_are_unknown(self):
        left, right = affine_models()
        cases = (
            {"radius": True}, {"radius": 0.}, {"radius": -1.},
            {"radius": float("nan")}, {"radius": float("inf")},
            {"beta": np.zeros((2, 3), dtype=complex)}, {"beta": np.zeros((2, 3), dtype=bool)},
            {"center": [float("nan"), 0.]}, {"junction": [float("inf"), 0.]},
            {"axes": [[0., 0.], [0., 0.]]},
            {"axes": [[1., 0.], [2., 0.]]},
            {"center": np.array([2**53+1, 0], dtype=np.int64)},
            {"center": np.array(["0", "0"])},
            {"center": np.array([0, 0], dtype=object)},
        )
        for overrides in cases:
            with self.subTest(keys=tuple(overrides)):
                self.unknown(left, right, **overrides)
        for bad in (replace(left, normal=(0., 0.)), replace(left, scale=0.),
                    replace(left, scale=-1.), replace(left, scale=float("nan")),
                    replace(left, bounds=(1., -1.)), replace(left, bounds=(-1., float("inf")))):
            with self.subTest(model=bad):
                self.unknown(bad, right)

    def test_point_F_and_gradient_enclosures_use_independent_fraction_derivatives(self):
        tiny = float(np.nextafter(0., 1.))
        samples = (
            (ManualPatch(), (.25, -.5, 2.), (.375, -.125)),
            (ManualPatch(origin=(.125, -.25), normal=(3., -2.), scale=.5),
             (-.125, .0625, -.25), (.5, .375)),
            (ManualPatch(normal=(1., 0.)), (tiny, 0., 0.), (16.*tiny, 3.*tiny)),
            (ManualPatch(normal=(math.ldexp(1., -1022), 0.)), (0., 0., 0.),
             (math.ldexp(1., -46), 0.)),
        )
        for m, beta, raw in samples:
            with self.subTest(model=m, beta=beta):
                q = point(raw)
                value = direct_polynomial(m, beta, q)
                gradient = exact_gradient(m, beta, q)
                bounds = joint._value(m, beta, (raw[0], raw[0]), (raw[1], raw[1]))
                self.assertLessEqual(rat(bounds[0]), value)
                self.assertGreaterEqual(rat(bounds[1]), value)
                gradient_bounds = joint._gradient(m, beta, (raw[0], raw[0]), (raw[1], raw[1]))
                for interval, exact in zip(gradient_bounds, gradient):
                    self.assertLessEqual(rat(interval[0]), exact)
                    self.assertGreaterEqual(rat(interval[1]), exact)

    def test_singular_saved_preconditioner_is_independently_rejected(self):
        with patch.object(joint.np.linalg, "inv", return_value=np.array([[1., 2.], [2., 4.]])):
            result = self.unknown(*affine_models())
        self.assertIn("preconditioner nonsingularity", result["reason"])

    def test_noninverse_nonsingular_preconditioner_can_prove_existence(self):
        c = .875 * np.eye(2)  # Not the inverse of the identity Jacobian.
        with patch.object(joint.np.linalg, "inv", return_value=c):
            result = self.yes(*affine_models())
        self.assertEqual(result["preconditioner"], c.tolist())
        self.assertNotEqual(rat(result["preconditioner"][0][0]), F(1))

    def test_readonly_noncontiguous_and_signedzero_inputs_are_immutable(self):
        left = ManualPatch(origin=(-0., 0.), normal=(1., -0.))
        right = ManualPatch(origin=(-0., 0.), normal=(-0., 1.))
        storage = np.zeros((2, 6))
        storage[:, 1::2] = 99.
        beta = storage[:, ::2]
        beta[0, 0] = -0.
        beta.setflags(write=False)
        axes = (2.*np.eye(2)).T
        axes.setflags(write=False)
        center_storage = np.array([-0., 17., 0., 19.])
        center = center_storage[::2]
        center.setflags(write=False)
        radius = np.asarray(.125)
        radius.setflags(write=False)
        self.yes(left, right, beta, axes=axes, center=center, radius=radius)

    def test_swapping_side_order_does_not_create_or_destroy_literal_root(self):
        models = (ManualPatch(normal=(0., 1.)), ManualPatch(normal=(1., 0.)))
        beta = ((0., 0., 1.), (.5, 0., 0.))
        first = self.yes(*models, beta=beta, axes=((4., 0.), (0., 4.)), center=(.5, .25), radius=.01)
        second = self.yes(*reversed(models), beta=tuple(reversed(beta)),
                          axes=((4., 0.), (0., 4.)), center=(.5, .25), radius=.01)
        self.assertEqual(first["status"], second["status"])


if __name__ == "__main__":
    unittest.main()

