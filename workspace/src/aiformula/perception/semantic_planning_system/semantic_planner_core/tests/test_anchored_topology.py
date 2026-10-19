import copy
import math
import unittest

from semantic_planner_core.anchored_topology import anchored_graph
from semantic_planner_core.traffic_route import OfficialToOdom, TopologyGraph


IDENTITY = OfficialToOdom(1.0, 0.0, 0.0, 0.0, 0.0)


def graph_with_edges(nodes, edges, closed_ways=None):
    adjacency = {identifier: {} for identifier in nodes}
    for start, end in edges:
        adjacency[start][end] = adjacency[end][start] = math.dist(nodes[start], nodes[end])
    return TopologyGraph(dict(nodes), adjacency, closed_ways or [])


class AnchoredTopologyTests(unittest.TestCase):
    def graph(self):
        nodes = dict(a=(0.0, 0.0), b=(4.0, 0.0), c=(4.0, 4.0), d=(0.0, 4.0),
                     inner=(2.0, 1.0), isolated=(8.0, -6.0))
        return graph_with_edges(nodes, [("a", "b"), ("b", "c"), ("c", "d"), ("d", "a"),
                                        ("a", "inner"), ("inner", "b")],
                                [["a", "b", "c", "d", "a"]])

    def test_boundary_nodes_match_exactly_and_internal_nodes_follow(self):
        graph = self.graph()
        anchors = dict(a=(0.0, 1.0), b=(6.0, 1.0), c=(6.0, 5.0), d=(0.0, 5.0))
        result = anchored_graph(graph, anchors, IDENTITY)
        for identifier, point in anchors.items():
            self.assertEqual(result.nodes[identifier], point)
        self.assertEqual(result.nodes["inner"], (3.0, 2.0))
        self.assertEqual(result.nodes["isolated"], graph.nodes["isolated"])

    def test_displacement_uses_inverse_original_edge_length(self):
        graph = graph_with_edges(dict(a=(0, 0), inner=(1, 0), b=(4, 0)),
                                 [("a", "inner"), ("inner", "b")])
        result = anchored_graph(graph, dict(a=(0, 2), b=(4, 0)), IDENTITY)
        self.assertAlmostEqual(result.nodes["inner"][0], 1.0)
        self.assertAlmostEqual(result.nodes["inner"][1], 1.5)

    def test_coupled_internal_nodes_solve_displacement_together(self):
        graph = graph_with_edges(dict(a=(0, 0), m=(1, 0), n=(2, 0), b=(3, 0)),
                                 [("a", "m"), ("m", "n"), ("n", "b")])
        result = anchored_graph(graph, dict(a=(0, 0), b=(3, 3)), IDENTITY)
        self.assertAlmostEqual(result.nodes["m"][1], 1.0)
        self.assertAlmostEqual(result.nodes["n"][1], 2.0)

    def test_rotated_scaled_translated_baseline_and_zero_displacement(self):
        graph = self.graph()
        transform = OfficialToOdom(2.0, math.pi / 2, 10.0, -4.0, 0.1)
        base = {identifier: transform.point(*point) for identifier, point in graph.nodes.items()}
        anchors = {identifier: base[identifier] for identifier in ("a", "b", "c", "d")}
        result = anchored_graph(graph, anchors, transform)
        self.assertEqual(result.nodes, base)
        self.assertAlmostEqual(result.adjacency["a"]["b"], 2.0)
        self.assertAlmostEqual(result.nodes["a"][0], 2.0)
        self.assertAlmostEqual(result.nodes["a"][1], 5.0)

    def test_one_anchor_translates_its_whole_component_only(self):
        graph = self.graph()
        result = anchored_graph(graph, {"a": (3.0, -2.0)}, IDENTITY)
        for identifier, point in graph.nodes.items():
            expected = point if identifier == "isolated" else (point[0] + 3.0, point[1] - 2.0)
            self.assertEqual(result.nodes[identifier], expected)

    def test_unanchored_connected_component_keeps_transformed_geometry(self):
        graph = self.graph()
        graph.nodes.update(other_a=(10.0, 10.0), other_b=(12.0, 10.0))
        graph.adjacency.update(other_a={"other_b": 2.0}, other_b={"other_a": 2.0})
        transform = OfficialToOdom(0.5, 0.0, 1.0, 2.0, 0.0)
        result = anchored_graph(graph, {"a": (0.0, 0.0)}, transform)
        for identifier in ("other_a", "other_b", "isolated"):
            self.assertEqual(result.nodes[identifier], transform.point(*graph.nodes[identifier]))
        self.assertEqual(result.adjacency["other_a"]["other_b"], 4.0)

    def test_edges_are_unchanged_and_costs_use_final_odom_metres(self):
        graph = self.graph()
        anchors = dict(a=(0.0, 0.0), b=(8.0, 0.0), c=(8.0, 6.0), d=(0.0, 6.0))
        result = anchored_graph(graph, anchors, IDENTITY)
        for start in graph.nodes:
            self.assertEqual(set(result.adjacency[start]), set(graph.adjacency[start]))
            for end in graph.adjacency[start]:
                self.assertEqual(result.adjacency[start][end], math.dist(result.nodes[start], result.nodes[end]))
        self.assertEqual(result.adjacency["a"]["b"], 8.0)
        self.assertEqual(result.closed_ways, graph.closed_ways)

    def test_detour_starts_and_ends_at_exact_fused_route_anchors(self):
        graph = self.graph()
        anchors = dict(a=(0.1, 0.2), b=(4.5, 0.3), c=(4.4, 4.2), d=(0.2, 4.1))
        result = anchored_graph(graph, anchors, IDENTITY)
        route = result.alternative_route("a", "b", outer_node_ids=["a", "b", "c", "d"])
        self.assertEqual(route.node_ids, ["a", "inner", "b"])
        self.assertEqual(route.points[0], anchors["a"])
        self.assertEqual(route.points[-1], anchors["b"])
        self.assertEqual(route.points[1], result.nodes["inner"])

    def test_source_graph_and_anchors_are_not_mutated_or_aliased(self):
        graph = self.graph()
        original = copy.deepcopy(graph)
        anchors = {"a": [1.0, 0.0]}
        result = anchored_graph(graph, anchors, IDENTITY)
        self.assertEqual(graph, original)
        self.assertEqual(anchors, {"a": [1.0, 0.0]})
        result.nodes["a"] = (99.0, 99.0)
        result.adjacency["a"].clear()
        result.closed_ways[0].clear()
        self.assertEqual(graph, original)

    def test_rejects_empty_unknown_nonfinite_and_malformed_anchors(self):
        for anchors in ({}, {"missing": (0, 0)}, {"a": (float("nan"), 0)},
                        {"a": (0, float("inf"))}, {"a": (0,)}, {"a": None},
                        {"a": "12"}, {"a": {"x": 1, "y": 2}}, None):
            with self.subTest(anchors=anchors), self.assertRaises(ValueError):
                anchored_graph(self.graph(), anchors, IDENTITY)

    def test_rejects_invalid_transform(self):
        for transform in (None, OfficialToOdom(0, 0, 0, 0, 0),
                          OfficialToOdom(-1, 0, 0, 0, 0),
                          OfficialToOdom(1, float("nan"), 0, 0, 0),
                          OfficialToOdom(1, 0, 0, 0, -1)):
            with self.subTest(transform=transform), self.assertRaises(ValueError):
                anchored_graph(self.graph(), {"a": (0, 0)}, transform)

    def test_rejects_incomplete_asymmetric_or_invalid_graph(self):
        mutations = [lambda graph: graph.adjacency.pop("a"),
                     lambda graph: graph.adjacency["a"].pop("b"),
                     lambda graph: graph.adjacency["a"].update(missing=1.0),
                     lambda graph: graph.adjacency["a"].update(a=1.0),
                     lambda graph: graph.adjacency["a"].update(b=-1.0),
                     lambda graph: graph.adjacency["a"].update(b=float("nan")),
                     lambda graph: graph.adjacency["a"].update(b=8.0),
                     lambda graph: graph.nodes.update(b=graph.nodes["a"]),
                     lambda graph: graph.nodes.update(a=(float("inf"), 0)),
                     lambda graph: graph.closed_ways.append(["a", "c", "d", "a"])]
        for index, mutate in enumerate(mutations):
            graph = self.graph()
            mutate(graph)
            with self.subTest(index=index), self.assertRaises(ValueError):
                anchored_graph(graph, {"a": (0, 0)}, IDENTITY)

    def test_rejects_edges_collapsed_by_boundary_deformation(self):
        with self.assertRaises(ValueError):
            anchored_graph(self.graph(), {"a": (0, 0), "b": (0, 0)}, IDENTITY)


if __name__ == "__main__":
    unittest.main()
