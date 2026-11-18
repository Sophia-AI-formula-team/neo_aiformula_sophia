"""Geometric hypotheses and point-to-curve registration regressions."""
import math
import time
import unittest
from unittest.mock import patch

import numpy as np

import test_route_fusion as fixtures
from semantic_planner_core.geometric_alignment import (
    CurveProblem, PointHash, geometric_seeds, refine,
)
from semantic_planner_core.rough_route import RouteReference
from semantic_planner_core.route_fusion import build_fused_route as production_fuse


def build_fused_route(samples, official, **options):
    # Node (8, 0) lies on the synthetic horizontal segment: the supplied
    # branch points into that synthetic loop and is perpendicular to it.
    if official.points[0] == (8, 0):
        options.setdefault('branch_connections', {official.node_ids[0]: [('test_inner', (8., 3.))]})
        return production_fuse(samples, official, **options)
    return fixtures.build_fused_route(samples, official, **options)


class GeometricAlignmentTests(unittest.TestCase):
    # The mapped points lie on straight sides, not at the track's corners.
    outline = [(0, 0), (30, 0), (30, 7), (12, 7), (12, 25), (0, 25)]
    nodes = [(8, 0), (30, 3), (20, 7), (12, 19), (7, 25), (0, 8)]

    def test_hash_checks_actual_distance_across_cell_boundaries(self):
        lookup = PointHash(np.asarray([(0.99, 0.0), (-2.0, 1.0)]), 1.0)
        self.assertTrue(lookup.supports(np.asarray((1.01, 0.0)), 0.03))
        self.assertFalse(lookup.supports(np.asarray((1.01, 0.0)), 0.01))
        self.assertFalse(lookup.supports(np.asarray((10.0, 10.0)), 0.5))

    def test_pair_and_triangle_hypotheses_create_diverse_candidates(self):
        reference = RouteReference(fixtures.record(self.outline, step=0.2),
                                   spacing_m=0.2, smooth_window_m=0.0)
        problem = CurveProblem(reference, fixtures.route(self.nodes), 4.0, 2.0)
        seeds, diagnostics = geometric_seeds(problem, time.monotonic() + 20.0)
        self.assertFalse(diagnostics["search_budget_exhausted"])
        self.assertGreater(diagnostics["pair_hypotheses"], 100)
        self.assertGreaterEqual(diagnostics["triangle_verified_hypotheses"], len(seeds))
        self.assertGreater(len(seeds), 2)
        self.assertLessEqual(len(seeds), 64)
        clouds = [problem.design @ seed for seed in seeds]
        self.assertGreater(max(float(np.linalg.norm(clouds[0] - cloud)) for cloud in clouds[1:]), 5.0)

    def test_refinement_uses_monotone_point_curve_objective(self):
        reference = RouteReference(fixtures.record(self.outline, step=0.2),
                                   spacing_m=0.2, smooth_window_m=0.0)
        problem = CurveProblem(reference, fixtures.route(self.nodes), 4.0, 2.0)
        seed = problem.parameters(1.01, 0.04, (0.4, -0.2))
        candidate = refine(problem, seed, 1, time.monotonic() + 20.0)
        self.assertIsNotNone(candidate)
        history = candidate["objective_history"]
        self.assertGreater(len(history), 1)
        self.assertTrue(all(after <= before + 1e-12 for before, after in zip(history, history[1:])))
        self.assertLess(history[-1], history[0])
        self.assertTrue(candidate["converged"], candidate["termination"])

    def test_nodes_on_straights_match_under_transform_start_and_direction(self):
        for angle, scale, shift, reverse in ((0.0, 1.0, 23, False),
                                             (0.61, 1.2, 183, False),
                                             (-1.25, 0.8, 61, True)):
            with self.subTest(angle=angle, scale=scale, shift=shift, reverse=reverse):
                c, s = math.cos(angle), math.sin(angle)
                def transform(point):
                    x, y = point
                    return (scale * (c*x-s*y) + 101.0, scale * (s*x+c*y) - 52.0)
                measured = [transform(point) for point in self.outline]
                samples = fixtures.record(measured, step=0.18, reverse=reverse, shift=shift)
                result = build_fused_route(samples, fixtures.route(self.nodes), shape_weight=0.0)
                self.assertTrue(result.status["available"], result.status)
                self.assertTrue(result.alignment["best_converged"])
                self.assertTrue(result.alignment["best_objective_monotone"])
                self.assertEqual(result.alignment["direction"], -1 if reverse else 1)
                self.assertLess(result.alignment["rmse_m"], 0.05)
                self.assertFalse(result.alignment["search_exhaustive"])
                for index, point in enumerate(self.nodes):
                    self.assertLess(math.dist(result.node_positions[str(index)], transform(point)), 0.2)

    def test_symmetric_curve_reports_ambiguity_when_lowest_score_is_selected(self):
        square = [(0, 0), (20, 0), (20, 20), (0, 20)]
        result = build_fused_route(fixtures.record(square), fixtures.route(square))
        # Current policy intentionally picks the lowest eligible score even
        # when alternatives are equally good; ambiguity remains observable.
        self.assertTrue(result.status["available"])
        self.assertFalse(result.alignment["candidate_gap_enforced"])
        self.assertFalse(result.alignment["global_transform_available"])
        self.assertFalse(result.alignment["node_correspondence_available"])
        self.assertGreater(result.alignment["alternative_node_spread_m"], 10.0)

    def test_expired_search_returns_odom_reference_only(self):
        with patch("semantic_planner_core.geometric_alignment.MAX_SECONDS", 0.0):
            result = build_fused_route(fixtures.record(self.outline), fixtures.route(self.nodes))
        self.assertFalse(result.status["available"])
        self.assertEqual(result.status["reason"], "solver_budget_exhausted")
        self.assertTrue(result.alignment["search_budget_exhausted"])
        self.assertGreater(result.reference.length_m, 0.0)
        self.assertEqual(result.node_positions, {})

    def test_low_iteration_budget_cannot_silently_accept_unfinished_candidates(self):
        with patch("semantic_planner_core.geometric_alignment.MAX_ITERATIONS", 1):
            # Default argument binding needs an explicit wrapper for this budget.
            import semantic_planner_core.geometric_alignment as alignment
            original = alignment.refine
            def truncated(problem, seed, direction, deadline):
                return original(problem, seed, direction, deadline, max_iterations=1)
            with patch.object(alignment, "refine", side_effect=truncated):
                result = build_fused_route(fixtures.record(self.outline), fixtures.route(self.nodes))
        self.assertFalse(result.status["available"])
        self.assertEqual(result.status["reason"], "solver_not_converged")
        self.assertGreater(result.alignment["unresolved_candidate_count"], 0)
        self.assertEqual(result.node_positions, {})


if __name__ == "__main__":
    unittest.main()
