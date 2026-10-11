import math
from pathlib import Path
import tempfile
import unittest

from semantic_planner_core.traffic_route import DetourTracker, OpenTopoRoute, TopologyGraph, validated_alignment


OSM = """<osm>
  <node id="000" ><tag k="local_x" v="0"/><tag k="local_y" v="0"/></node>
  <node id="101" ><tag k="local_x" v="5"/><tag k="local_y" v="0"/></node>
  <node id="102" ><tag k="local_x" v="5"/><tag k="local_y" v="5"/></node>
  <node id="103" ><tag k="local_x" v="8"/><tag k="local_y" v="5"/></node>
  <node id="999" ><tag k="local_x" v="9"/><tag k="local_y" v="9"/></node>
  <way id="1"><nd ref="000"/><nd ref="101"/><nd ref="102"/></way>
  <way id="2"><nd ref="102"/><nd ref="103"/></way>
</osm>"""


class OpenTopoRouteTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / "map.osm"
        self.path.write_text(OSM, encoding="utf-8")

    def tearDown(self):
        self.temp.cleanup()

    def test_preserves_explicit_order_and_string_ids(self):
        route = OpenTopoRoute.from_osm(self.path, ["103", "102", "101", "000"])
        self.assertEqual(route.node_ids[-1], "000")
        self.assertEqual(route.points, [(8, 5), (5, 5), (5, 0), (0, 0)])

    def test_no_guessed_route_or_invented_edges(self):
        for identifiers in ([], None, "000,101", ["000"], ["000", "999"],
                            ["000", "102"], ["missing", "101"], ["000", "101", "000"]):
            with self.subTest(identifiers=identifiers), self.assertRaises(ValueError):
                OpenTopoRoute.from_osm(self.path, identifiers)

    def test_rejects_nonfinite_map_and_missing_edges(self):
        self.path.write_text(OSM.replace('v="8"', 'v="nan"'), encoding="utf-8")
        with self.assertRaises(ValueError):
            OpenTopoRoute.from_osm(self.path, ["000", "101"])
        self.path.write_text(OSM.replace('ref="103"', 'ref="missing"'), encoding="utf-8")
        with self.assertRaises(ValueError):
            OpenTopoRoute.from_osm(self.path, ["000", "101"])


class AlignmentTests(unittest.TestCase):
    def diagnostic(self):
        return dict(scale=2.0, rotation_rad=math.pi / 2.0,
                    translation_x=10.0, translation_y=-4.0, rmse_m=0.2,
                    transform_direction="recorded_odom_to_official_xy", diagnostic_only=True)

    def test_inverse_transform_includes_scale_rotation_and_translation(self):
        transform = validated_alignment(self.diagnostic(), 0.5, 0.5, 3.0)
        # odom (3,4) -> official (2,2) at scale=2, rotation=90deg, t=(10,-4).
        x, y = transform.point(2.0, 2.0)
        self.assertAlmostEqual(x, 3.0)
        self.assertAlmostEqual(y, 4.0)
        converted = transform.points([(10.0, -4.0), (2.0, 2.0)])
        self.assertEqual(converted[0], (0.0, 0.0))
        self.assertAlmostEqual(converted[1][1], 4.0)

    def test_rejects_bad_quality_missing_fields_or_wrong_direction(self):
        for changes in (dict(rmse_m=2.0), dict(rmse_m=-1), dict(scale=0),
                        dict(scale=9.0), dict(rotation_rad=float("nan")),
                        dict(translation_y=float("inf")), dict(available=False),
                        dict(transform_direction="official_to_odom")):
            diagnostic = self.diagnostic()
            diagnostic.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validated_alignment(diagnostic, 0.5, 0.5, 3.0)
        diagnostic = self.diagnostic()
        del diagnostic["rmse_m"]
        with self.assertRaises(ValueError):
            validated_alignment(diagnostic, 0.5, 0.5, 3.0)
        for limits in ((float("nan"), 0.5, 3), (1, 0, 3), (1, 2, 1)):
            with self.assertRaises(ValueError):
                validated_alignment(self.diagnostic(), *limits)


class TopologyGraphTests(unittest.TestCase):
    def make_graph(self, inner=True, spur=False):
        nodes = dict(a=(0.0, 0.0), b=(4.0, 0.0), c=(4.0, 4.0), d=(0.0, 4.0))
        edges = [("a", "b"), ("b", "c"), ("c", "d"), ("d", "a")]
        if inner:
            nodes.update(m=(2.0, 1.0), n=(2.0, -10.0))
            edges += [("a", "m"), ("m", "b"), ("a", "n"), ("n", "b")]
        if spur:
            nodes["spur"] = (-1.0, 0.0)
            edges.append(("a", "spur"))
        adjacency = {node: {} for node in nodes}
        for a, b in edges:
            adjacency[a][b] = adjacency[b][a] = math.dist(nodes[a], nodes[b])
        return TopologyGraph(nodes, adjacency, [["a", "b", "c", "d", "a"]])

    def test_shortest_internal_bypass_removes_edge_in_both_directions(self):
        graph = self.make_graph()
        outer = ["a", "b", "c", "d", "a"]
        self.assertEqual(graph.alternative_route("a", "b", outer_node_ids=outer).node_ids, ["a", "m", "b"])
        self.assertEqual(graph.alternative_route("b", "a", outer_node_ids=outer).node_ids, ["b", "m", "a"])
        # Blocking is local to this search, not a persistent graph mutation.
        self.assertIn("b", graph.adjacency["a"])
        self.assertIn("a", graph.adjacency["b"])

    def test_internal_is_preferred_even_when_outer_bypass_is_shorter(self):
        graph = self.make_graph()
        graph.adjacency["a"]["m"] = graph.adjacency["m"]["a"] = 20.0
        graph.adjacency["m"]["b"] = graph.adjacency["b"]["m"] = 20.0
        route = graph.alternative_route("a", "b", require_internal=False,
                                        outer_node_ids=["a", "b", "c", "d"])
        self.assertEqual(route.node_ids, ["a", "n", "b"])

    def test_missing_internal_route_waits_or_explicitly_falls_back(self):
        graph = self.make_graph(inner=False)
        outer = ["a", "b", "c", "d"]
        self.assertIsNone(graph.alternative_route("a", "b", outer_node_ids=outer))
        fallback = graph.alternative_route("a", "b", False, outer)
        self.assertEqual(fallback.node_ids, ["a", "d", "c", "b"])
        self.assertEqual(graph.alternative_route("a", "b", False).node_ids, fallback.node_ids)

    def test_internal_dead_end_cannot_fake_bypass_by_returning_to_same_node(self):
        graph = self.make_graph(inner=False, spur=True)
        self.assertIsNone(graph.alternative_route("a", "b", outer_node_ids=["a", "b", "c", "d"]))

    def test_missing_route_and_invalid_block_are_explicit(self):
        graph = self.make_graph(inner=False)
        with self.assertRaises(ValueError):
            graph.alternative_route("a", "b")
        with self.assertRaises(ValueError):
            graph.alternative_route("a", "c", outer_node_ids=["a", "b", "c", "d"])
        with self.assertRaises(ValueError):
            graph.alternative_route("missing", "b", outer_node_ids=["a"])
        del graph.adjacency["a"]["d"]
        del graph.adjacency["d"]["a"]
        self.assertIsNone(graph.alternative_route("a", "b", False))

    def test_osm_preserves_isolated_nodes_and_has_no_automatic_route(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.osm"
            path.write_text(OSM, encoding="utf-8")
            graph = TopologyGraph.from_osm(path)
        self.assertEqual(graph.adjacency["999"], {})
        self.assertEqual(graph.closed_ways, [])
        self.assertIsNone(graph.alternative_route("000", "101", False))


class DetourTrackerTests(unittest.TestCase):
    def tracker(self, **kwargs):
        return DetourTracker([(0, 0), (5, 0), (5, 5)], waypoint_radius_m=0.2,
                             max_cross_track_m=1.0, **kwargs)

    def test_requires_entry_and_does_not_attach_to_later_segment(self):
        tracker = self.tracker()
        result = tracker.update(5, 3, math.pi / 2)
        self.assertEqual(result.reason, "entry_not_reached")
        self.assertFalse(tracker.started)
        self.assertEqual(tracker.segment_index, 0)
        result = tracker.update(0.1, 0, 0)
        self.assertTrue(result.available)
        self.assertEqual(result.points, [(0.5, 0.0), (1.0, 0.0), (1.5, 0.0)])

    def test_near_corner_does_not_skip_ahead_and_keeps_distinct_targets(self):
        tracker = self.tracker()
        tracker.update(0, 0, 0)
        result = tracker.update(4.5, 0, 0)
        self.assertEqual(result.segment_index, 0)
        self.assertTrue(result.available)
        self.assertTrue(all(abs(y) < 1e-9 for _, y in result.points))
        self.assertLessEqual(result.points[-1][0], 0.5)
        self.assertTrue(all(math.dist(a, b) > 1e-6 for a, b in zip(result.points, result.points[1:])))
        result = tracker.update(5, 0, math.pi / 2)
        self.assertEqual(result.segment_index, 1)
        self.assertTrue(result.available)
        self.assertAlmostEqual(result.points[0][0], 0.5)
        self.assertAlmostEqual(result.points[0][1], 0.0)

    def test_cross_track_and_behind_vehicle_hold_all_state(self):
        tracker = self.tracker()
        tracker.update(0, 0, 0)
        tracker.update(1, 0, 0)
        state = (tracker.started, tracker.segment_index, tracker.progress_m)
        self.assertEqual(tracker.update(1, 2, 0).reason, "cross_track_exceeded")
        self.assertEqual(tracker.update(2, 0, math.pi).reason, "target_behind_vehicle")
        self.assertEqual((tracker.started, tracker.segment_index, tracker.progress_m), state)
        self.assertEqual(tracker.update(float("nan"), 0, 0).reason, "invalid_pose")

    def test_invalid_next_segment_target_does_not_commit_corner_transition(self):
        tracker = self.tracker()
        tracker.update(0, 0, 0)
        previous = tracker.progress_m
        result = tracker.update(5, 0, -math.pi / 2)
        self.assertEqual(result.reason, "target_behind_vehicle")
        self.assertEqual(tracker.segment_index, 0)
        self.assertEqual(tracker.progress_m, previous)
        self.assertEqual(tracker.update(5, 0, math.pi / 2).segment_index, 1)

    def test_end_is_done_without_repeated_endpoint_triplet(self):
        tracker = self.tracker()
        tracker.update(0, 0, 0)
        tracker.update(5, 0, math.pi / 2)
        result = tracker.update(5, 4.9, math.pi / 2)
        self.assertTrue(result.done)
        self.assertFalse(result.available)
        self.assertEqual(result.points, [])
        self.assertAlmostEqual(result.progress_m, 10.0)
        self.assertTrue(tracker.update(5, 5, 0).done)
        tracker.reset()
        self.assertFalse(tracker.started)
        self.assertEqual(tracker.update(5, 5, 0).reason, "entry_not_reached")

    def test_entry_radius_is_independent_and_progress_does_not_jump_at_crossing(self):
        tracker = DetourTracker([(0, 0), (4, 4), (0, 4), (4, 0)],
                                waypoint_radius_m=0.2, entry_radius_m=0.5)
        self.assertTrue(tracker.update(0.3, 0.1, math.pi / 4).available)
        result = tracker.update(2, 2, math.pi / 4)
        self.assertEqual(result.segment_index, 0)
        self.assertAlmostEqual(result.progress_m, math.sqrt(8))
        self.assertEqual(tracker.update(2.0, 2.0, math.pi / 4).segment_index, 0)

    def test_rejects_bad_geometry_and_limits(self):
        for points in ([], [(0, 0)], [(0, 0), (0, 0)], [(0, 0), (float("inf"), 1)]):
            with self.assertRaises(ValueError):
                DetourTracker(points)
        for kwargs in (dict(spacing_m=0), dict(lookahead_m=-1), dict(entry_radius_m=float("nan"))):
            with self.assertRaises(ValueError):
                DetourTracker([(0, 0), (1, 0)], **kwargs)


if __name__ == "__main__":
    unittest.main()
