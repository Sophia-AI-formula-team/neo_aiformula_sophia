"""Synthetic counterexamples only. These tests have NOT been run at authoring.

Put the existing semantic_planner_core package and this directory on PYTHONPATH.
The real core's RouteReference/OfficialRoute are used, without modifying them.
No saved Unity coordinates, masks, model, simulator, or fit replay is involved.
"""

import copy
from dataclasses import replace
import math
import unittest

from semantic_planner_core.rough_route import RouteReference, RouteSample
from semantic_planner_core.rough_topology import OfficialRoute
from semantic_planner_core.traffic_route import TopologyGraph

import junction_association as ja


CONTEXT = ja.ObservationContext(7, "synthetic-camera-v1", "test_odom", 0.0, 100.0, True)
OLD_FAILURE = {"available": False, "reason": "rmse_exceeds_limit", "rmse_m": 3.40446314}


def reference(points):
    return RouteReference([RouteSample(x, y, 0.0) for x, y in points],
                          spacing_m=0.25, smooth_window_m=0.0)


def observed(identifier, anchor, bearing, index, uncertainty=0.02):
    supports = tuple(ja.CaptureSupport(
        sequence=index * 10 + frame + 1,
        capture_time_s=1.0 + index * 15.0 + frame * 0.1,
        received_time_s=1.05 + index * 15.0 + frame * 0.1,
        pose_time_s=1.0 + index * 15.0 + frame * 0.1,
        epoch=CONTEXT.epoch, calibration_id=CONTEXT.calibration_id, frame_id=CONTEXT.frame_id,
        anchor_xy=anchor, branch_bearing_rad=bearing,
        positive_left_pixels=20, positive_right_pixels=19,
    ) for frame in range(3))
    return ja.JunctionObservation(identifier, anchor, bearing, uncertainty, math.radians(2), supports)


def fixture(reverse=False, official_scale=1.0):
    # All FIVE positive junction observations lie on STRAIGHTS. The driven
    # reference has only FOUR geometric corners, none used as a junction seed.
    corners = [(0.0, 0.0), (40.0, 0.0), (40.0, 30.0), (0.0, 30.0)]
    anchors = [(7.0, 0.0), (29.0, 0.0), (40.0, 11.0), (27.0, 30.0), (0.0, 19.0)]
    bearings = [math.pi / 2, math.pi / 2, math.pi, -math.pi / 2, 0.0]
    ids = ["official_" + str(index) for index in range(5)]
    angle, scale, tx, ty = 0.37, 1.3 * official_scale, 12.0, -8.0
    def transform(point):
        x, y = point
        return (scale * (math.cos(angle) * x - math.sin(angle) * y) + tx,
                scale * (math.sin(angle) * x + math.cos(angle) * y) + ty)
    official_points = [transform(point) for point in anchors]
    official = OfficialRoute(ids + ids[:1], official_points + official_points[:1])
    priors = {identifier: bearing + angle for identifier, bearing in zip(ids, bearings)}
    ordering = list(range(5)) if not reverse else list(reversed(range(5)))
    observations = [observed("anonymous_" + str(index), anchors[index], bearings[index], arrival)
                    for arrival, index in enumerate(ordering)]
    if reverse:
        corners = corners[:1] + list(reversed(corners[1:]))
    return observations, official, reference(corners), priors


def run_case(data, **kwargs):
    observations, official, ref, priors = data
    options = dict(context=CONTEXT, official_branch_bearings=priors,
                   original_whole_lap_status=OLD_FAILURE, enabled=True)
    options.update(kwargs)
    return ja.associate_junctions(observations, official, ref, **options)


def edit_support(data, observation_index=0, support_index=0, **changes):
    observations = list(data[0])
    observation = observations[observation_index]
    supports = list(observation.supports)
    supports[support_index] = replace(supports[support_index], **changes)
    observations[observation_index] = replace(observation, supports=tuple(supports))
    return (observations,) + data[1:]


def move_observation(data, index, point):
    observations = list(data[0])
    old = observations[index]
    observations[index] = replace(old, anchor_xy=point,
                                 supports=tuple(replace(support, anchor_xy=point) for support in old.supports))
    return (observations,) + data[1:]


class JunctionAssociationTests(unittest.TestCase):
    def test_default_disabled_does_not_read_geometry_or_context(self):
        class Poison:
            def __getattribute__(self, key):
                raise AssertionError("disabled path accessed geometry")
        result = ja.associate_junctions(Poison(), Poison(), Poison(), context=Poison(),
                                       official_branch_bearings=Poison(), original_whole_lap_status=OLD_FAILURE)
        self.assertEqual(result["reason"], "disabled")
        self.assertEqual(result["candidates"], [])

    def test_only_exact_boolean_enables(self):
        for value in (1, "true", [], None):
            with self.subTest(value=value):
                self.assertEqual(run_case(fixture(), enabled=value)["reason"], "enabled_must_be_boolean")

    def test_five_straight_junctions_not_four_bends(self):
        data = fixture()
        result = run_case(data)
        self.assertTrue(result["available"], result)
        self.assertEqual(result["candidate_count"], 10)
        self.assertEqual({(c["direction"], c["shift"]) for c in result["candidates"]},
                         {(d, s) for d in (1, -1) for s in range(5)})
        self.assertEqual(result["match"]["direction"], 1)
        self.assertLess(result["match"]["transform"]["rmse_m"], 1e-10)
        for node in result["match"]["nodes"]:
            progress = node["reference_progress_m"]
            before, centre, after = [data[2].point_at(progress + offset) for offset in (-0.4, 0.0, 0.4)]
            incoming = math.atan2(centre[1] - before[1], centre[0] - before[0])
            outgoing = math.atan2(after[1] - centre[1], after[0] - centre[0])
            self.assertLess(abs(ja._wrap(outgoing - incoming)), 1e-8)

    def test_reverse_cycle_is_explicitly_enumerated(self):
        result = run_case(fixture(reverse=True))
        self.assertTrue(result["available"], result)
        self.assertEqual(result["match"]["direction"], -1)
        for node in result["match"]["nodes"]:
            self.assertEqual(node["official_id"].split("_")[-1], node["observation_id"].split("_")[-1])

    def test_official_uniform_scale_does_not_change_measured_residual(self):
        original, scaled = run_case(fixture()), run_case(fixture(official_scale=2.0))
        self.assertTrue(original["available"] and scaled["available"])
        self.assertAlmostEqual(original["match"]["transform"]["rmse_m"],
                               scaled["match"]["transform"]["rmse_m"], places=10)
        self.assertAlmostEqual(original["ambiguity_gap_m"], scaled["ambiguity_gap_m"], places=9)

    def test_inputs_reference_and_original_failure_are_preserved(self):
        data = fixture()
        before = copy.deepcopy((data[0], vars(data[1]), vars(data[2]), data[3], OLD_FAILURE))
        result = run_case(data)
        self.assertTrue(result["available"])
        self.assertEqual(before, (data[0], vars(data[1]), vars(data[2]), data[3], OLD_FAILURE))
        self.assertEqual(result["original_whole_lap_status"], OLD_FAILURE)
        self.assertFalse(result["original_whole_lap_status"]["available"])
        self.assertFalse(result["reference_modified"])
        self.assertFalse(result["internal_connectivity_observed"])
        self.assertFalse(result["detour_authorized"])
        result["original_whole_lap_status"]["reason"] = "edited-result-only"
        self.assertEqual(OLD_FAILURE["reason"], "rmse_exceeds_limit")

    def test_same_outer_outline_different_straight_opening_is_not_same_evidence(self):
        data = fixture()
        moved = move_observation(data, 1, (11.0, 0.0))
        self.assertEqual(vars(data[2]), vars(moved[2]))
        baseline, changed = run_case(data), run_case(moved)
        self.assertTrue(baseline["available"])
        self.assertFalse(changed["available"])
        changed_identity = next(c for c in changed["candidates"] if c["direction"] == 1 and c["shift"] == 0)
        self.assertGreater(changed_identity["rmse_m"], 3.0)

    def test_five_equal_degree_symmetric_mouths_remain_ambiguous(self):
        points = [(20 * math.cos(i * 2 * math.pi / 5), 20 * math.sin(i * 2 * math.pi / 5)) for i in range(5)]
        ids = [str(i) for i in range(5)]
        bearings = [math.atan2(-y, -x) for x, y in points]
        observations = [observed("track" + str(i), point, bearings[i], i) for i, point in enumerate(points)]
        data = (observations, OfficialRoute(ids + ids[:1], points + points[:1]), reference(points), dict(zip(ids, bearings)))
        result = run_case(data)
        self.assertFalse(result["available"])
        self.assertEqual(result["reason"], "ambiguous_junction_correspondence")
        self.assertEqual(result["candidate_count"], 10)

    def test_near_symmetric_case_is_not_resolved_by_float_tie_break(self):
        points = [(20 * math.cos(i * 2 * math.pi / 5), 20 * math.sin(i * 2 * math.pi / 5)) for i in range(5)]
        points[0] = (points[0][0] + 0.01, points[0][1])
        ids = [str(i) for i in range(5)]
        bearings = [math.atan2(-y, -x) for x, y in points]
        data = ([observed(str(i), point, bearings[i], i) for i, point in enumerate(points)],
                OfficialRoute(ids + ids[:1], points + points[:1]), reference(points), dict(zip(ids, bearings)))
        result = run_case(data)
        self.assertEqual(result["reason"], "ambiguous_junction_correspondence")
        self.assertLess(result["ambiguity_gap_m"], result["required_gap_m"])

    def test_missing_extra_or_bend_only_observations_cannot_seed_a_match(self):
        data = fixture()
        for observations in ([], data[0][:-1], data[0] + [data[0][0]]):
            with self.subTest(count=len(observations)):
                result = run_case((observations,) + data[1:])
                self.assertEqual(result["reason"], "requires_exactly_one_observation_per_outer_junction")

    def test_gap_crosswalk_or_main_lane_counts_are_not_positive_branch_evidence(self):
        # This validates the *declared* positive-evidence contract; only the
        # upstream image observer can verify what those pixels actually depict.
        for changes in ({"positive_left_pixels": 0}, {"positive_right_pixels": 0}, {"truncated": True}):
            with self.subTest(changes=changes):
                result = run_case(edit_support(fixture(), **changes))
                self.assertEqual(result["reason"], "missing_positive_unclipped_branch_boundaries")

    def test_single_frame_or_replayed_frames_cannot_make_confirmation(self):
        data = fixture()
        observations = list(data[0])
        observations[0] = replace(observations[0], supports=observations[0].supports[:1])
        self.assertEqual(run_case((observations,) + data[1:])["reason"], "insufficient_new_positive_frames")
        replayed = edit_support(data, support_index=1, sequence=1)
        self.assertEqual(run_case(replayed)["reason"], "duplicate_or_out_of_order_capture")

    def test_duplicated_track_ids_are_rejected(self):
        data = fixture()
        observations = list(data[0])
        observations[1] = replace(observations[1], observation_id=observations[0].observation_id)
        self.assertEqual(run_case((observations,) + data[1:])["reason"], "duplicate_or_invalid_observation_id")

    def test_epoch_calibration_and_frame_must_match(self):
        for changes in ({"epoch": 8}, {"calibration_id": "another-camera"}, {"frame_id": "world"}):
            with self.subTest(changes=changes):
                self.assertEqual(run_case(edit_support(fixture(), **changes))["reason"], "capture_identity_mismatch")

    def test_current_tick_pose_cannot_replace_capture_time_pose(self):
        self.assertEqual(run_case(edit_support(fixture(), pose_time_s=1.05))["reason"],
                         "capture_pose_or_ingress_time_invalid")

    def test_stale_future_or_pre_lap_capture_rejected(self):
        for changes in ({"received_time_s": 1.6}, {"received_time_s": 0.9},
                        {"capture_time_s": -1.0, "received_time_s": -0.9, "pose_time_s": -1.0}):
            with self.subTest(changes=changes):
                self.assertEqual(run_case(edit_support(fixture(), **changes))["reason"],
                                 "capture_pose_or_ingress_time_invalid")

    def test_historical_first_lap_anchors_are_not_reaged_as_live_commands(self):
        # Support at t=1 is legitimate at lap-end t=100 because original
        # capture/ingress age was checked. Association does not emit a cmd/ABC.
        result = run_case(fixture())
        self.assertTrue(result["available"])
        self.assertFalse(any(key in result for key in ("cmd", "v", "w", "abc", "traffic_light")))

    def test_unfinished_lap_is_unavailable(self):
        self.assertEqual(run_case(fixture(), context=replace(CONTEXT, completed_lap=False))["reason"],
                         "unverified_lap_identity")

    def test_nonfinite_observation_and_boolean_numeric_are_rejected(self):
        for value in (math.nan, math.inf, True):
            with self.subTest(value=value):
                self.assertFalse(run_case(edit_support(fixture(), branch_bearing_rad=value))["available"])

    def test_support_geometry_must_agree_with_its_anchor(self):
        self.assertEqual(run_case(edit_support(fixture(), anchor_xy=(8.0, 0.0)))["reason"],
                         "inconsistent_observation_support")

    def test_position_uncertainty_cannot_be_inflated_to_accept_bad_geometry(self):
        data = fixture()
        observations = list(data[0])
        observations[0] = replace(observations[0], position_uncertainty_m=4.0)
        self.assertEqual(run_case((observations,) + data[1:])["reason"], "observation_uncertainty_exceeds_contract")

    def test_projection_is_metric_and_does_not_move_reference_to_anchor(self):
        data = move_observation(fixture(), 0, (7.0, -1.1))
        self.assertEqual(run_case(data)["reason"], "junction_projection_too_far")

    def test_nearest_untrusted_neighbourhood_is_not_skipped_for_another_branch(self):
        data = fixture()
        ref = data[2]
        near = min(range(len(ref.points)), key=lambda i: math.hypot(ref.points[i][0] - 7.4, ref.points[i][1]))
        ref.trusted[near] = False
        self.assertEqual(run_case(data)["reason"], "untrusted_junction_neighbourhood")

    def test_crossing_route_projection_is_ambiguous_even_if_nearest_is_exact(self):
        data = move_observation(fixture(), 0, (5.0, 5.0))
        crossing = reference([(0.0, 0.0), (10.0, 10.0), (0.0, 10.0), (10.0, 0.0)])
        self.assertEqual(run_case((data[0], data[1], crossing, data[3]))["reason"], "ambiguous_reference_projection")

    def test_two_anonymous_ids_cannot_claim_one_reference_station(self):
        data = move_observation(fixture(), 1, (7.0, 0.0))
        self.assertEqual(run_case(data)["reason"], "non_distinct_junction_projections")

    def test_chronological_observations_cannot_be_sorted_into_a_fake_lap(self):
        data = fixture()
        observations = list(data[0])
        observations[1], observations[2] = (
            observed("anonymous_2", observations[2].anchor_xy, observations[2].branch_bearing_rad, 1),
            observed("anonymous_1", observations[1].anchor_xy, observations[1].branch_bearing_rad, 2),
        )
        self.assertEqual(run_case((observations,) + data[1:])["reason"], "observed_order_is_not_one_reference_cycle")

    def test_wrong_branch_direction_vetoes_geometrically_good_match(self):
        data = fixture()
        priors = dict(data[3])
        priors["official_0"] += math.pi / 2
        result = run_case(data, official_branch_bearings=priors)
        self.assertEqual(result["reason"], "best_correspondence_failed_gates")
        best = next(c for c in result["candidates"] if c["direction"] == 1 and c["shift"] == 0)
        self.assertIn("branch_bearing_inconsistent", best["rejections"])

    def test_bearing_filter_cannot_hide_a_nearly_equal_geometric_competitor(self):
        points = [(20 * math.cos(i * 2 * math.pi / 5), 20 * math.sin(i * 2 * math.pi / 5)) for i in range(5)]
        ids = [str(i) for i in range(5)]
        bearings = [math.atan2(-y, -x) for x, y in points]
        priors = dict(zip(ids, bearings))
        priors["0"] += 0.2
        data = ([observed(str(i), point, bearings[i], i) for i, point in enumerate(points)],
                OfficialRoute(ids + ids[:1], points + points[:1]), reference(points), priors)
        self.assertEqual(run_case(data)["reason"], "ambiguous_junction_correspondence")

    def test_individual_node_limit_is_not_hidden_by_low_average(self):
        data = move_observation(fixture(), 1, (29.1, 0.0))
        result = run_case(data, limits=replace(ja.AssociationLimits(), max_node_residual_m=0.025))
        self.assertEqual(result["reason"], "best_correspondence_failed_gates")
        best = next(c for c in result["candidates"] if c["direction"] == 1 and c["shift"] == 0)
        self.assertIn("individual_node_residual_exceeds_limit", best["rejections"])

    def test_unsafe_threshold_relaxation_is_not_an_api_escape(self):
        for changes in ({"max_projection_m": 2.1}, {"max_rmse_m": 3.1},
                        {"max_node_residual_m": 3.1}, {"ambiguity_margin_m": 0.0},
                        {"min_support_frames": 2}, {"max_branch_error_rad": math.pi},
                        {"max_ingress_age_s": 0.6}, {"max_pose_skew_s": 0.1},
                        {"max_support_span_s": 2.0}, {"remote_progress_m": 100.0},
                        {"projection_ambiguity_m": 0.01}, {"trusted_half_window_m": 0.1},
                        {"min_anchor_separation_m": 0.01}, {"max_official_per_odom_scale": 100.0}):
            with self.subTest(changes=changes):
                self.assertFalse(run_case(fixture(), limits=replace(ja.AssociationLimits(), **changes))["available"])

    def test_branch_prior_helper_checks_original_nodes_and_adjacency(self):
        data = fixture()
        official, priors = data[1], data[3]
        identifiers, points = official.node_ids[:-1], official.points[:-1]
        nodes = dict(zip(identifiers, points))
        adjacency = {identifier: {} for identifier in identifiers}
        def edge(a, b):
            weight = math.hypot(nodes[a][0] - nodes[b][0], nodes[a][1] - nodes[b][1])
            adjacency[a][b] = adjacency[b][a] = weight
        for index, identifier in enumerate(identifiers):
            branch = "internal_" + str(index)
            bearing = priors[identifier]
            nodes[branch] = (nodes[identifier][0] + 5 * math.cos(bearing), nodes[identifier][1] + 5 * math.sin(bearing))
            adjacency[branch] = {}
            edge(identifier, branch)
            edge(identifier, identifiers[index - 1])
        graph = TopologyGraph(nodes, adjacency, [official.node_ids])
        actual = ja.branch_bearings_from_graph(official, graph)
        for identifier in identifiers:
            self.assertLess(abs(ja._wrap(actual[identifier] - priors[identifier])), 1e-10)
        graph.nodes[identifiers[0]] = (999.0, 999.0)
        with self.assertRaisesRegex(ValueError, "official_graph_coordinate_mismatch"):
            ja.branch_bearings_from_graph(official, graph)


if __name__ == "__main__":
    unittest.main()
