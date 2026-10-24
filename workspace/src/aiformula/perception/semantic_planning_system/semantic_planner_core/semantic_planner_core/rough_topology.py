"""Small helpers for the organiser's sparse topological map.

The straight segments in this file are connectivity/chord geometry, not measured
lane centre lines.  The default route is the first closed OSM way, as an example
route only: XML order does not declare the race route or permitted direction.
The odometry alignment below is a diagnostic with assumed start/progress
correspondence.  It is neither localisation nor a correction of the driven path.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Dict, FrozenSet, List, Optional, Sequence, Set, Tuple, Union
import xml.etree.ElementTree as ET


Point = Tuple[float, float]
_EPSILON = 1e-12


def _finite_points(values: Sequence[Point]) -> List[Point]:
    points: List[Point] = []
    for value in values:
        try:
            if len(value) != 2:
                raise ValueError("Each point must contain x and y only")
            point = (float(value[0]), float(value[1]))
        except (TypeError, IndexError, OverflowError) as exc:
            raise ValueError("Each point must contain finite numeric x and y") from exc
        if not all(math.isfinite(coordinate) for coordinate in point):
            raise ValueError("Point coordinates must be finite")
        points.append(point)
    return points


def _cumulative_lengths(points: Sequence[Point]) -> List[float]:
    cumulative = [0.0]
    for start, end in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.hypot(end[0] - start[0], end[1] - start[1]))
    if not math.isfinite(cumulative[-1]):
        raise ValueError("Path length must be finite")
    return cumulative


def _sample_closed(points: Sequence[Point], count: int) -> List[Point]:
    """Sample equal arc-length fractions without duplicating the end point."""
    cumulative = _cumulative_lengths(points)
    if cumulative[-1] <= _EPSILON:
        raise ValueError("A path with nonzero length is required")
    samples: List[Point] = []
    for index in range(count):
        distance = cumulative[-1] * index / count
        edge = min(bisect.bisect_right(cumulative, distance) - 1, len(points) - 2)
        edge_length = cumulative[edge + 1] - cumulative[edge]
        # bisect_right skips zero-length runs, including repeated odom samples.
        if edge_length <= _EPSILON:
            samples.append(points[edge])
            continue
        fraction = (distance - cumulative[edge]) / edge_length
        start, end = points[edge], points[edge + 1]
        samples.append((start[0] + fraction * (end[0] - start[0]),
                        start[1] + fraction * (end[1] - start[1])))
    return samples


@dataclass
class OfficialRoute:
    """A closed sequence of connected official nodes, preserving string IDs."""

    node_ids: List[str]
    points: List[Point]
    chord_length_m: float = field(init=False)
    _cumulative_m: List[float] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.node_ids = list(self.node_ids)
        self.points = _finite_points(self.points)
        if len(self.node_ids) != len(self.points):
            raise ValueError("Each route node must have one point")
        if (len(self.node_ids) < 4 or self.node_ids[0] != self.node_ids[-1]
                or len(set(self.node_ids[:-1])) < 3):
            raise ValueError("Route must be closed and contain at least three distinct nodes")
        if self.points[0] != self.points[-1]:
            raise ValueError("Closed route endpoint coordinates must agree")
        self._cumulative_m = _cumulative_lengths(self.points)
        if any(end - start <= _EPSILON
               for start, end in zip(self._cumulative_m, self._cumulative_m[1:])):
            raise ValueError("Route edges must have nonzero length")
        self.chord_length_m = self._cumulative_m[-1]

    @classmethod
    def from_osm(
        cls, path: Union[str, Path], node_ids: Optional[Sequence[str]] = None,
    ) -> "OfficialRoute":
        """Read local_x/local_y and undirected adjacency from an official OSM.

        A configured route must explicitly repeat its first node at the end.
        With no configured IDs, select the first closed way as an example.
        No edge is invented to connect an isolated origin or close an open way.
        """
        try:
            root = ET.parse(path).getroot()
        except (ET.ParseError, OSError) as exc:
            raise ValueError(f"Cannot read official OSM: {exc}") from exc
        if root.tag != "osm":
            raise ValueError("Expected an OSM XML root")

        nodes: Dict[str, Point] = {}
        for element in root.findall("node"):
            identifier = element.get("id")
            if not identifier or identifier in nodes:
                raise ValueError("OSM node IDs must be present and unique")
            tags = {tag.get("k"): tag.get("v") for tag in element.findall("tag")}
            try:
                coordinate = (float(tags["local_x"]), float(tags["local_y"]))
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                raise ValueError(f"Node {identifier} needs numeric local_x/local_y") from exc
            nodes[identifier] = _finite_points([coordinate])[0]

        edges: Set[FrozenSet[str]] = set()
        closed_ways: List[List[str]] = []
        for way in root.findall("way"):
            references = [item.get("ref") for item in way.findall("nd")]
            if any(reference not in nodes for reference in references):
                raise ValueError("An OSM way refers to an unknown or missing node")
            # References are known strings after the membership validation.
            route_ids = [str(reference) for reference in references]
            for start, end in zip(route_ids, route_ids[1:]):
                if start == end:
                    raise ValueError("An OSM edge cannot join a node to itself")
                edges.add(frozenset((start, end)))
            if len(route_ids) >= 4 and route_ids[0] == route_ids[-1]:
                closed_ways.append(route_ids)

        if isinstance(node_ids, (str, bytes)):
            raise ValueError("route node_ids must be a sequence of string IDs")
        selected = list(node_ids) if node_ids is not None else []
        if not selected:
            if not closed_ways:
                raise ValueError("No closed OSM way; configure an explicit closed route")
            selected = closed_ways[0]
        if any(not isinstance(identifier, str) or identifier not in nodes for identifier in selected):
            raise ValueError("Configured route contains an unknown node ID")
        if len(selected) < 4 or selected[0] != selected[-1]:
            raise ValueError("Configured route must explicitly repeat its first node at the end")
        for start, end in zip(selected, selected[1:]):
            if frozenset((start, end)) not in edges:
                raise ValueError(f"No official edge connects {start} to {end}")
        return cls(selected, [nodes[identifier] for identifier in selected])

    def edge_at_fraction(self, fraction: float) -> Tuple[str, str]:
        """Return a rough chord-progress edge label, never a localisation fix.

        Fractions wrap each lap; exactly 1.0 labels the first edge of the next
        lap.  At an interior node the outgoing edge is returned.
        """
        if not math.isfinite(fraction):
            raise ValueError("Route fraction must be finite")
        distance = (fraction % 1.0) * self.chord_length_m
        edge = min(bisect.bisect_right(self._cumulative_m, distance) - 1,
                   len(self.node_ids) - 2)
        return self.node_ids[edge], self.node_ids[edge + 1]

    def summarize_alignment(self, recorded_points: Sequence[Point]) -> dict:
        """Fit an orientation-preserving similarity from odom to official XY.

        Equal normalised arc length is assumed to denote corresponding progress,
        with the same start node and travel direction.  Different curve lengths,
        detours, or a different start invalidate that assumption.  No start phase
        search, SLAM, drift correction, or reliable branch inference is performed.
        A closing segment is added when odom does not exactly repeat the first
        sample; closure_gap_m lets the caller reject an inadequately closed lap.
        """
        measured = _finite_points(recorded_points)
        if len(measured) < 3 or len(set(measured)) < 3:
            raise ValueError("At least three distinct recorded points are required")
        closure_gap = math.hypot(measured[-1][0] - measured[0][0],
                                 measured[-1][1] - measured[0][1])
        if measured[-1] != measured[0]:
            measured.append(measured[0])
        sample_count = 256
        source = _sample_closed(measured, sample_count)
        target = _sample_closed(self.points, sample_count)
        source_mean = tuple(math.fsum(point[axis] for point in source) / sample_count
                            for axis in range(2))
        target_mean = tuple(math.fsum(point[axis] for point in target) / sample_count
                            for axis in range(2))
        source_centred = [(x - source_mean[0], y - source_mean[1]) for x, y in source]
        target_centred = [(x - target_mean[0], y - target_mean[1]) for x, y in target]
        denominator = math.fsum(x * x + y * y for x, y in source_centred)
        if not math.isfinite(denominator) or denominator <= _EPSILON:
            raise ValueError("Recorded path is too small or too large for alignment")
        a = math.fsum(x * u + y * v for (x, y), (u, v)
                      in zip(source_centred, target_centred)) / denominator
        b = math.fsum(x * v - y * u for (x, y), (u, v)
                      in zip(source_centred, target_centred)) / denominator
        scale = math.hypot(a, b)
        if not math.isfinite(scale) or scale <= _EPSILON:
            raise ValueError("No useful orientation-preserving similarity fit")
        translation_x = target_mean[0] - a * source_mean[0] + b * source_mean[1]
        translation_y = target_mean[1] - b * source_mean[0] - a * source_mean[1]
        squared_error = math.fsum(
            (a * x - b * y + translation_x - u) ** 2
            + (b * x + a * y + translation_y - v) ** 2
            for (x, y), (u, v) in zip(source, target)
        )
        result = {
            "scale": scale,
            "rotation_rad": math.atan2(b, a),
            "translation_x": translation_x,
            "translation_y": translation_y,
            "rmse_m": math.sqrt(squared_error / sample_count),
            "closure_gap_m": closure_gap,
            "recorded_closed_length_m": _cumulative_lengths(measured)[-1],
            "official_chord_length_m": self.chord_length_m,
            "sample_count": sample_count,
            "transform_direction": "recorded_odom_to_official_xy",
            "diagnostic_only": True,
            "assumption": "same start and direction; equal normalized arc-length progress",
        }
        if any(isinstance(value, float) and not math.isfinite(value) for value in result.values()):
            raise ValueError("Alignment produced non-finite values")
        return result
