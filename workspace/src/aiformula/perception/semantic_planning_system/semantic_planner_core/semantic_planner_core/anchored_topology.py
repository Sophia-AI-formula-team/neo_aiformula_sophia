"""Place official graph nodes in the measured, fused-route odometry frame.

Outer nodes are exact route anchors. Interior geometry remains an estimate:
the official-to-odom similarity supplies its initial shape, and a harmonic
displacement extends the outer corrections through the existing graph edges.
This does not add connections or claim to survey unseen internal roads.
"""

from __future__ import annotations

import math
from collections.abc import Mapping

import numpy as np

from .traffic_route import OfficialToOdom, Point, TopologyGraph


_EPS = 1e-6


def _point(value) -> Point:
    try:
        if isinstance(value, (str, bytes)) or len(value) != 2:
            raise ValueError("Graph coordinates need two components")
        result = (float(value[0]), float(value[1]))
    except (TypeError, IndexError, KeyError, ValueError, OverflowError) as exc:
        raise ValueError("Graph coordinates must be finite pairs") from exc
    if not all(math.isfinite(coordinate) for coordinate in result):
        raise ValueError("Graph coordinates must be finite pairs")
    return result


def _checked_graph(graph: TopologyGraph) -> tuple[dict[str, Point], dict[str, dict[str, float]]]:
    """Validate graph integrity and derive weights from official geometry."""
    if (not isinstance(graph, TopologyGraph) or not isinstance(graph.nodes, Mapping)
            or not graph.nodes or not isinstance(graph.adjacency, Mapping)
            or set(graph.adjacency) != set(graph.nodes)):
        raise ValueError("Graph needs nodes and a complete adjacency mapping")
    if any(not isinstance(identifier, str) or not identifier for identifier in graph.nodes):
        raise ValueError("Node IDs must be nonempty strings")
    nodes = {identifier: _point(point) for identifier, point in graph.nodes.items()}
    weights = {identifier: {} for identifier in nodes}
    for start, neighbours in graph.adjacency.items():
        if not isinstance(neighbours, Mapping):
            raise ValueError("Graph adjacency entries must be mappings")
        for end, cost in neighbours.items():
            if end not in nodes or start == end:
                raise ValueError("Graph edge must join two known, distinct nodes")
            reverse = graph.adjacency[end]
            if not isinstance(reverse, Mapping) or start not in reverse:
                raise ValueError("Graph edges must be undirected")
            try:
                cost, reverse_cost = float(cost), float(reverse[start])
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError("Graph edge costs must be finite and positive") from exc
            if (not math.isfinite(cost) or not math.isfinite(reverse_cost)
                    or cost <= 0.0 or reverse_cost <= 0.0
                    or not math.isclose(cost, reverse_cost, rel_tol=1e-9, abs_tol=1e-9)):
                raise ValueError("Graph edge costs must be finite, positive and symmetric")
            length = math.dist(nodes[start], nodes[end])
            if not math.isfinite(length) or length <= _EPS:
                raise ValueError("Official graph edges must have nonzero finite lengths")
            weights[start][end] = 1.0 / length
    if not isinstance(graph.closed_ways, (list, tuple)):
        raise ValueError("Graph closed ways must be a sequence")
    for way in graph.closed_ways:
        if (not isinstance(way, (list, tuple)) or len(way) < 4
                or any(not isinstance(identifier, str) or identifier not in nodes for identifier in way)
                or way[0] != way[-1] or len(set(way[:-1])) < 3
                or any(end not in weights[start] for start, end in zip(way, way[1:]))):
            raise ValueError("Closed way must follow known graph edges and return to its start")
    return nodes, weights


def anchored_graph(graph: TopologyGraph, outer_positions: dict[str, Point],
                   official_to_odom: OfficialToOdom) -> TopologyGraph:
    """Return an independent graph in odom units with exact outer anchors.

    For each component, interior displacement is the inverse-edge-length
    weighted average of neighbouring displacements. Components without an
    anchor retain their initial similarity-transformed positions. A component
    with one anchor translates rigidly. All edge costs are recalculated as
    Euclidean distances between the final odom positions.

    Partial boundary sets are supported, but unknown anchors, malformed graph
    structure, non-finite inputs and collapsed edges raise ``ValueError``.
    The source graph, its adjacency and its closed ways are never modified.
    """
    nodes, weights = _checked_graph(graph)
    if (not isinstance(outer_positions, Mapping) or not outer_positions
            or any(identifier not in nodes for identifier in outer_positions)):
        raise ValueError("At least one anchor with a known graph node ID is required")
    anchors = {identifier: _point(point) for identifier, point in outer_positions.items()}
    if not isinstance(official_to_odom, OfficialToOdom):
        raise ValueError("A checked official-to-odom transformation is required")
    try:
        transform_values = tuple(float(getattr(official_to_odom, name)) for name in
                                 ("scale", "rotation_rad", "translation_x", "translation_y", "rmse_m"))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("Frame transformation must be finite") from exc
    if (not all(math.isfinite(value) for value in transform_values)
            or transform_values[0] <= 0.0 or transform_values[-1] < 0.0):
        raise ValueError("Frame transformation needs finite values and a positive scale")
    transform = OfficialToOdom(*transform_values)
    base = {identifier: transform.point(*point) for identifier, point in nodes.items()}
    result = dict(base)
    result.update(anchors)
    unseen = set(nodes)
    for first in nodes:
        if first not in unseen:
            continue
        component, pending = [], [first]
        unseen.remove(first)
        while pending:
            current = pending.pop()
            component.append(current)
            for neighbour in weights[current]:
                if neighbour in unseen:
                    unseen.remove(neighbour)
                    pending.append(neighbour)
        boundary = [identifier for identifier in component if identifier in anchors]
        free = [identifier for identifier in component if identifier not in anchors]
        if not boundary or not free:
            continue
        displacement = {identifier: (anchors[identifier][0] - base[identifier][0],
                                     anchors[identifier][1] - base[identifier][1])
                        for identifier in boundary}
        if any(not all(math.isfinite(value) for value in delta) for delta in displacement.values()):
            raise ValueError("Anchor displacement must be finite")
        if all(delta == (0.0, 0.0) for delta in displacement.values()):
            continue
        if len(boundary) == 1:
            dx, dy = displacement[boundary[0]]
            for identifier in free:
                result[identifier] = (base[identifier][0] + dx, base[identifier][1] + dy)
            continue
        indices = {identifier: index for index, identifier in enumerate(free)}
        matrix = np.zeros((len(free), len(free)), dtype=float)
        rhs = np.zeros((len(free), 2), dtype=float)
        for identifier, row in indices.items():
            for neighbour, weight in weights[identifier].items():
                matrix[row, row] += weight
                if neighbour in indices:
                    matrix[row, indices[neighbour]] -= weight
                else:
                    rhs[row] += weight * np.asarray(displacement[neighbour])
        try:
            solved = np.linalg.solve(matrix, rhs)
        except np.linalg.LinAlgError as exc:
            raise ValueError("Cannot interpolate graph-node displacements") from exc
        if not np.all(np.isfinite(solved)):
            raise ValueError("Graph deformation produced non-finite displacements")
        for identifier, row in indices.items():
            result[identifier] = (base[identifier][0] + float(solved[row, 0]),
                                  base[identifier][1] + float(solved[row, 1]))
    result = {identifier: _point(point) for identifier, point in result.items()}
    adjacency = {identifier: {} for identifier in nodes}
    for start, neighbours in weights.items():
        for end in neighbours:
            length = math.dist(result[start], result[end])
            if not math.isfinite(length) or length <= _EPS:
                raise ValueError("Anchored graph edges must have nonzero finite lengths")
            adjacency[start][end] = length
    return TopologyGraph(result, adjacency, [list(way) for way in graph.closed_ways])
