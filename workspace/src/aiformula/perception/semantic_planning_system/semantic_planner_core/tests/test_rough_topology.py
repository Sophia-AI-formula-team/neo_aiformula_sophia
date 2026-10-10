"""Run with unittest; no ROS, numpy, or pytest dependency is required."""

import math
from pathlib import Path
import tempfile
import unittest

from semantic_planner_core.rough_topology import OfficialRoute


class OfficialRouteTests(unittest.TestCase):
    @staticmethod
    def official_map_path():
        packaged = (Path(__file__).resolve().parents[2] / "semantic_local_planner_ros"
                    / "config" / "aiformula_topomap.osm")
        if packaged.is_file():
            return packaged
        for parent in Path(__file__).resolve().parents:
            candidate = parent / "docs" / "topological_backup_research" / "aiformula_topomap.osm"
            if candidate.is_file():
                return candidate
        raise AssertionError("The supplied official OSM fixture is missing")

    def load_xml(self, content, node_ids=None):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.osm"
            path.write_text(content, encoding="utf-8")
            return OfficialRoute.from_osm(path, node_ids)

    def test_supplied_map_default_is_first_closed_way_not_isolated_origin(self):
        route = OfficialRoute.from_osm(self.official_map_path())
        self.assertEqual(route.node_ids, ["10000000000", "10000000001", "10000000002",
                                          "10000000003", "10000000004", "10000000000"])
        self.assertNotIn("00000000000", route.node_ids)
        self.assertAlmostEqual(route.chord_length_m, 215.73, delta=0.02)
        self.assertEqual(route.points[0], (0.144056, -3.517467))

    def test_explicit_route_can_join_edges_from_different_ways(self):
        ids = ["10000000000", "10000000001", "10000000005", "10000000000"]
        route = OfficialRoute.from_osm(self.official_map_path(), ids)
        self.assertEqual(route.node_ids, ids)

    def test_explicit_empty_route_uses_example_default(self):
        self.assertEqual(OfficialRoute.from_osm(self.official_map_path(), []).node_ids,
                         OfficialRoute.from_osm(self.official_map_path()).node_ids)

    def test_no_invented_edges_or_implicit_closure(self):
        invalid = [
            ["00000000000", "10000000000", "10000000001", "00000000000"],
            ["10000000000", "10000000002", "10000000001", "10000000000"],
            ["10000000000", "10000000001", "10000000005"],
        ]
        for route in invalid:
            with self.subTest(route=route), self.assertRaises(ValueError):
                OfficialRoute.from_osm(self.official_map_path(), route)

    def test_bad_xml_nonfinite_coordinates_and_open_map_rejected(self):
        invalid_xml = [
            "<osm>",
            "<something_else/>",
            '<osm><node id="1"><tag k="local_x" v="nan"/><tag k="local_y" v="0"/></node></osm>',
            '<osm><node id="1"><tag k="local_x" v="0"/></node></osm>',
            '<osm><way><nd ref="missing"/></way></osm>',
            '<osm><node id="1"><tag k="local_x" v="0"/><tag k="local_y" v="0"/></node></osm>',
        ]
        for content in invalid_xml:
            with self.subTest(xml=content), self.assertRaises(ValueError):
                self.load_xml(content)

    def test_route_ids_must_be_a_sequence_of_strings(self):
        for invalid in ("10000000000", [1, 2, 3, 1]):
            with self.subTest(value=invalid), self.assertRaises(ValueError):
                OfficialRoute.from_osm(self.official_map_path(), invalid)

    def test_similarity_fit_recovers_known_transform_without_mutating_path(self):
        route = OfficialRoute(["a", "b", "c", "d", "a"],
                              [(0.0, 0.0), (8.0, 0.0), (10.0, 5.0), (0.0, 4.0), (0.0, 0.0)])
        scale, angle, tx, ty = 1.7, 0.6, 12.0, -7.0
        cosine, sine = math.cos(angle), math.sin(angle)
        # Invert desired official = scale * R * odom + translation.
        recorded = [((cosine * (x - tx) + sine * (y - ty)) / scale,
                     (-sine * (x - tx) + cosine * (y - ty)) / scale)
                    for x, y in route.points]
        original = list(recorded)
        result = route.summarize_alignment(recorded)
        self.assertAlmostEqual(result["scale"], scale, places=10)
        self.assertAlmostEqual(result["rotation_rad"], angle, places=10)
        self.assertAlmostEqual(result["translation_x"], tx, places=10)
        self.assertAlmostEqual(result["translation_y"], ty, places=10)
        self.assertLess(result["rmse_m"], 1e-10)
        self.assertTrue(result["diagnostic_only"])
        self.assertEqual(recorded, original)

    def test_alignment_handles_repeated_samples_and_reports_closing_gap(self):
        route = OfficialRoute(["a", "b", "c", "a"], [(0, 0), (3, 0), (1, 2), (0, 0)])
        result = route.summarize_alignment([(0, 0), (0, 0), (3, 0), (1, 2)])
        self.assertAlmostEqual(result["closure_gap_m"], math.sqrt(5.0))
        self.assertAlmostEqual(result["scale"], 1.0)
        self.assertLess(result["rmse_m"], 1e-10)

    def test_alignment_rejects_invalid_odom(self):
        route = OfficialRoute.from_osm(self.official_map_path())
        for invalid in ([], [(0, 0)] * 4, [(0, 0), (1, 0), (math.nan, 1)], [(0,), (1, 0), (2, 2)]):
            with self.subTest(points=invalid), self.assertRaises(ValueError):
                route.summarize_alignment(invalid)

    def test_fraction_labels_wrap_without_connecting_origin(self):
        route = OfficialRoute.from_osm(self.official_map_path())
        self.assertEqual(route.edge_at_fraction(0.0), ("10000000000", "10000000001"))
        self.assertEqual(route.edge_at_fraction(1.0), route.edge_at_fraction(0.0))
        self.assertEqual(route.edge_at_fraction(0.999), ("10000000004", "10000000000"))
        with self.assertRaises(ValueError):
            route.edge_at_fraction(math.inf)


if __name__ == "__main__":
    unittest.main()
