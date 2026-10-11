"""Synthetic contract tests using the actual core RouteReference/TopologyGraph.

No simulator, image, model, CAD, control or native ROS acceptance is claimed.
Set CALIBRATED_TEST_CORE to the semantic_planner_core package root if needed.
"""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest

CORE = Path(os.environ.get("CALIBRATED_TEST_CORE", (
    "E:/Mess/Projects/Programming/aiformula/workspace/semantic_stop_20261012/repo/"
    "workspace/src/aiformula/perception/semantic_planning_system/semantic_planner_core")))
sys.path.insert(0, str(CORE))
from semantic_planner_core.rough_route import RouteReference, RouteSample
from semantic_planner_core.traffic_route import TopologyGraph
from calibrated_topology import AXES, SCHEMA, load_calibrated_topology


def dense(start, end):
    count = max(1, int(math.ceil(math.dist(start, end) / 0.5)))
    return [[start[0] + (end[0] - start[0]) * step / count,
             start[1] + (end[1] - start[1]) * step / count]
            for step in range(count + 1)]


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.osm = Path(self.temp.name) / "official.osm"
        self.nodes = {"a": [2, 0], "b": [10, 2], "c": [8, 10],
                      "d": [0, 8], "m": [5, 5]}
        node_xml = "".join('<node id="{}"><tag k="local_x" v="{}"/>'
                           '<tag k="local_y" v="{}"/></node>'.format(key, *value)
                           for key, value in self.nodes.items())
        self.osm.write_text('<osm>' + node_xml
                            + '<way id="1"><nd ref="a"/><nd ref="b"/><nd ref="c"/>'
                              '<nd ref="d"/><nd ref="a"/></way>'
                              '<way id="2"><nd ref="b"/><nd ref="m"/><nd ref="c"/></way>'
                              '</osm>', encoding="utf-8")
        self.reference = self.make_reference([(0, 0), (10, 0), (10, 10), (0, 10)])
        self.config = {"schema": SCHEMA, "calibration_id": "synthetic-square-v1",
                       "frame": "odom", "axes": AXES,
                       "osm_sha256": hashlib.sha256(self.osm.read_bytes()).hexdigest(),
                       "outer_node_ids": ["a", "b", "c", "d", "a"],
                       "direction": 1, "logical_nodes": copy.deepcopy(self.nodes),
                       "edges": [self.edge("b", "m"), self.edge("m", "c")]}

    @staticmethod
    def make_reference(points):
        return RouteReference([RouteSample(x, y, 0.0) for x, y in points],
                              spacing_m=0.25, smooth_window_m=0.0, max_detour_gap_m=0.0)

    def edge(self, start, end, bidirectional=False):
        return {"node_ids": [start, end], "points": dense(self.nodes[start], self.nodes[end]),
                "bidirectional": bidirectional}

    def load(self, config=None, reference=None):
        return load_calibrated_topology(self.config if config is None else config,
                                       self.reference if reference is None else reference,
                                       self.osm, enabled=True)

    def assert_failure(self, config=None, reference=None, reason=None):
        result = self.load(config, reference)
        self.assertFalse(result.available, result.status)
        self.assertIsNone(result.graph)
        self.assertEqual(result.node_positions, {})
        if reason:
            self.assertIn(reason, result.status["reason"])
        return result

    def test_default_disabled_does_not_read_or_change_inputs(self):
        before = copy.deepcopy(self.reference.__dict__)
        result = load_calibrated_topology("nonexistent.json", self.reference, "no.osm")
        self.assertEqual(result.status, {"available": False, "reason": "disabled"})
        self.assertEqual(before, self.reference.__dict__)
        for flag in (1, "true", None):
            result = load_calibrated_topology(None, None, None, enabled=flag)
            self.assertEqual(result.status["reason"], "enabled_must_be_bool")

    def test_actual_core_dijkstra_then_separate_dense_expansion(self):
        before_reference = copy.deepcopy(self.reference.__dict__)
        before_config = copy.deepcopy(self.config)
        result = self.load()
        self.assertTrue(result.available, result.status)
        original = TopologyGraph.from_osm(self.osm).alternative_route(
            "b", "c", outer_node_ids=["a", "b", "c", "d"])
        route = result.graph.alternative_route("b", "c", outer_node_ids=["a", "b", "c", "d"])
        self.assertEqual(route.node_ids, original.node_ids)
        self.assertEqual(route.node_ids, ["b", "m", "c"])
        self.assertGreater(len(route.points), len(route.node_ids))
        self.assertEqual(route.points[0], (10.0, 2.0))
        self.assertEqual(route.points[-1], (8.0, 10.0))
        self.assertEqual(route.points.count((5.0, 5.0)), 1)
        self.assertEqual(result.direction, 1)
        self.assertFalse(result.status["automatic_similarity_supplied"])
        self.assertNotIn("scale", result.status)
        self.assertEqual(before_reference, self.reference.__dict__)
        self.assertEqual(before_config, self.config)

    def test_real_reference_reverse_infers_negative_direction(self):
        reverse = self.make_reference([(0, 10), (10, 10), (10, 0), (0, 0)])
        self.assert_failure(reference=reverse, reason="configured_direction")
        self.config["direction"] = -1
        result = self.load(reference=reverse)
        self.assertTrue(result.available, result.status)
        self.assertEqual(result.direction, -1)
        self.assertAlmostEqual(sum(result.status["cyclic_gaps_m"]), reverse.length_m)

    def test_noncyclic_portal_identity_is_rejected(self):
        self.config["logical_nodes"]["b"], self.config["logical_nodes"]["c"] = (
            self.config["logical_nodes"]["c"], self.config["logical_nodes"]["b"])
        self.assert_failure(reason="cyclic_order")

    def test_collapsed_portals_are_rejected(self):
        self.config["logical_nodes"]["b"] = [2, 0]
        self.assert_failure(reason="one_distinct_cycle")

    def test_neighboring_exclusion_rejects_even_exact_trusted_projection(self):
        index = self.reference.points.index((2.5, 0.0))
        self.reference.trusted[index] = False
        self.assert_failure(reason="untrusted_neighborhood")

    def test_spatially_near_separate_branches_are_ambiguous(self):
        narrow = self.make_reference([(0, 0), (10, 0), (10, 1), (0, 1)])
        self.config["logical_nodes"]["a"] = [2, 0.5]
        self.assert_failure(reference=narrow, reason="ambiguous")

    def test_untrusted_second_branch_does_not_hide_ambiguity(self):
        narrow = self.make_reference([(0, 0), (10, 0), (10, 1), (0, 1)])
        for index, point in enumerate(narrow.points):
            if point[1] == 1.0:
                narrow.trusted[index] = False
        self.config["logical_nodes"]["a"] = [2, 0.5]
        self.assert_failure(reference=narrow, reason="ambiguous")

    def test_projection_limit_has_hard_two_metre_ceiling(self):
        self.config["max_projection_distance_m"] = 2.01
        self.assert_failure(reason="above maximum")
        self.config["max_projection_distance_m"] = 1.0
        self.config["logical_nodes"]["a"] = [2, -1.01]
        self.assert_failure(reason="outside_limit")

    def test_prior_and_projected_coordinates_remain_distinct(self):
        self.config["logical_nodes"]["b"] = [10.5, 2]
        self.config["edges"][0]["points"] = dense((10.5, 2), (5, 5))
        result = self.load()
        self.assertTrue(result.available, result.status)
        self.assertEqual(result.prior_nodes["b"], (10.5, 2.0))
        self.assertEqual(result.node_positions["b"], (10.0, 2.0))
        self.assertEqual(result.graph.nodes["b"], (10.0, 2.0))
        route = result.graph.alternative_route("b", "c", outer_node_ids=["a", "b", "c", "d"])
        self.assertEqual(route.points[0], (10.5, 2.0))

    def test_two_metre_projection_limit_does_not_relax_edge_endpoint_limit(self):
        self.config["max_projection_distance_m"] = 2.0
        self.config["logical_nodes"]["b"] = [11.25, 2]
        self.config["edges"][0]["points"] = dense((11.25, 2), (5, 5))
        result = self.load()
        self.assertTrue(result.available, result.status)
        self.assertIsNone(result.graph.alternative_route(
            "b", "c", outer_node_ids=["a", "b", "c", "d"]))
        self.assertEqual(result.graph.last_route_status["reason"],
                         "edge_endpoint_far_from_projection")

    def test_missing_selected_geometry_never_returns_official_chord(self):
        self.config["edges"].pop()
        result = self.load()
        self.assertTrue(result.available, result.status)
        self.assertIsNone(result.graph.alternative_route(
            "b", "c", outer_node_ids=["a", "b", "c", "d"]))
        self.assertEqual(result.graph.last_route_status["reason"], "missing_directed_edge_geometry")
        self.assertEqual(result.graph.last_route_status["edge"], ["m", "c"])

    def test_unused_edges_may_be_missing_but_no_outer_fallback(self):
        result = self.load()
        self.assertIsNone(result.graph.alternative_route(
            "a", "b", require_internal=False, outer_node_ids=["a", "b", "c", "d"]))
        self.assertEqual(result.graph.last_route_status["reason"], "missing_directed_edge_geometry")

    def test_reverse_geometry_requires_explicit_direction_permission(self):
        result = self.load()
        self.assertIsNone(result.graph.alternative_route(
            "c", "b", outer_node_ids=["a", "b", "c", "d"]))
        for edge in self.config["edges"]:
            edge["bidirectional"] = True
        result = self.load()
        forward = result.graph.alternative_route("b", "c", outer_node_ids=["a", "b", "c", "d"])
        reverse = result.graph.alternative_route("c", "b", outer_node_ids=["a", "b", "c", "d"])
        self.assertEqual(reverse.node_ids, list(reversed(forward.node_ids)))
        self.assertEqual(reverse.points, list(reversed(forward.points)))

    def test_finite_strict_schema_frame_hash_and_ids(self):
        cases = [("frame", "map"), ("axes", "x=unity_x,y=unity_z"),
                 ("osm_sha256", "0" * 64), ("direction", True),
                 ("trusted_neighborhood_m", 0), ("max_projection_distance_m", float("nan"))]
        for field, value in cases:
            with self.subTest(field=field):
                candidate = copy.deepcopy(self.config)
                candidate[field] = value
                self.assert_failure(candidate)
        candidate = copy.deepcopy(self.config)
        candidate["logical_nodes"]["typo"] = [1, 2]
        self.assert_failure(candidate, reason="exactly")
        candidate = copy.deepcopy(self.config)
        candidate["logical_nodes"]["a"] = [True, 0]
        self.assert_failure(candidate, reason="finite JSON number")
        candidate = copy.deepcopy(self.config)
        candidate["logical_nodes"]["a"] = [float("inf"), 0]
        self.assert_failure(candidate)

    def test_bad_adjacency_endpoints_duplicate_sparse_geometry(self):
        cases = []
        value = copy.deepcopy(self.config)
        value["edges"][0]["node_ids"] = ["a", "c"]
        cases.append(value)
        value = copy.deepcopy(self.config)
        value["edges"][0]["points"][0][0] += 0.1
        cases.append(value)
        value = copy.deepcopy(self.config)
        value["edges"].append(copy.deepcopy(value["edges"][0]))
        cases.append(value)
        value = copy.deepcopy(self.config)
        value["edges"][0]["points"] = [self.nodes["b"], self.nodes["m"]]
        cases.append(value)
        value = copy.deepcopy(self.config)
        value["edges"][0]["bidirectional"] = 1
        cases.append(value)
        for index, candidate in enumerate(cases):
            with self.subTest(case=index):
                self.assert_failure(candidate)

    def test_duplicate_json_fields_fail_and_valid_file_loads(self):
        path = Path(self.temp.name) / "calibration.json"
        path.write_text('{"schema":"x","schema":"y"}', encoding="utf-8")
        self.assert_failure(path, reason="Duplicate JSON key")
        path.write_text(json.dumps(self.config), encoding="utf-8")
        self.assertTrue(self.load(path).available)

    def test_original_reference_is_required_and_corruption_is_rejected(self):
        self.assert_failure(reference=object(), reason="actual core RouteReference")
        self.reference.cumulative_m[-1] += 1.0
        self.assert_failure(reason="length mismatch")


if __name__ == "__main__":
    unittest.main(verbosity=2)
