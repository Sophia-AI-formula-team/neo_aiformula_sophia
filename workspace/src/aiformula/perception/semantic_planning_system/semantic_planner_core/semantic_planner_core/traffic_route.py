"""Explicit official-route loading, checked frame conversion and local tracking.

Official edges are sparse chords, not surveyed lane centres. These helpers do
not infer a permitted diversion, validate road clearance, or correct odometry.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
from itertools import count
import math
from pathlib import Path
from typing import Sequence
import xml.etree.ElementTree as ET
from typing import Tuple


Point = Tuple[float, float]
_EPS = 1e-6


def _points(values: Sequence[Point]) -> list[Point]:
    result = []
    try:
        for value in values:
            if len(value) != 2:
                raise ValueError("Expected two coordinates per point")
            point = (float(value[0]), float(value[1]))
            if not all(math.isfinite(coordinate) for coordinate in point):
                raise ValueError("Route coordinates must be finite")
            result.append(point)
    except (TypeError, IndexError, OverflowError) as exc:
        raise ValueError("Expected finite coordinate pairs") from exc
    return result


@dataclass
class OpenTopoRoute:
    """A user-specified open sequence of official, connected node IDs."""

    node_ids: list[str]
    points: list[Point]

    def __post_init__(self) -> None:
        self.node_ids = list(self.node_ids)
        self.points = _points(self.points)
        if (len(self.node_ids) < 2 or len(self.node_ids) != len(self.points)
                or self.node_ids[0] == self.node_ids[-1]):
            raise ValueError("An explicit open route with at least two nodes is required")
        if any(not isinstance(identifier, str) or not identifier for identifier in self.node_ids):
            raise ValueError("Node IDs must be nonempty strings")
        if any(math.dist(start, end) <= _EPS for start, end in zip(self.points, self.points[1:])):
            raise ValueError("Consecutive route nodes must have distinct coordinates")

    @classmethod
    def from_osm(cls, path: str | Path, node_ids: Sequence[str]) -> "OpenTopoRoute":
        """Validate explicit order against undirected official OSM adjacency.

        Empty configuration is an error. No default route, signal position,
        missing edge, or travel permission is inferred from XML order.
        """
        if isinstance(node_ids, (str, bytes)) or not node_ids:
            raise ValueError("Configure an explicit ordered list of route node IDs")
        return TopologyGraph.from_osm(path).route(node_ids)


@dataclass
class TopologyGraph:
    """The supplied undirected graph; edge direction is not inferred."""

    nodes: dict[str, Point]
    adjacency: dict[str, dict[str, float]]
    closed_ways: list[list[str]]

    @classmethod
    def from_osm(cls, path: str | Path) -> "TopologyGraph":
        try:
            root = ET.parse(path).getroot()
        except (ET.ParseError, OSError) as exc:
            raise ValueError(f"Cannot read official OSM: {exc}") from exc
        if root.tag != "osm":
            raise ValueError("Expected an OSM XML root")
        nodes = {}
        for element in root.findall("node"):
            identifier = element.get("id")
            if not identifier or identifier in nodes:
                raise ValueError("Official node IDs must be unique and present")
            tags = {item.get("k"): item.get("v") for item in element.findall("tag")}
            try:
                coordinate = (float(tags["local_x"]), float(tags["local_y"]))
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                raise ValueError(f"Node {identifier} needs finite local_x/local_y") from exc
            nodes[identifier] = _points([coordinate])[0]
        adjacency = {identifier: {} for identifier in nodes}
        closed_ways = []
        for way in root.findall("way"):
            references = [element.get("ref") for element in way.findall("nd")]
            if any(reference not in nodes for reference in references):
                raise ValueError("Official edge refers to an unknown node")
            for start, end in zip(references, references[1:]):
                if start == end:
                    raise ValueError("An official edge cannot join a node to itself")
                length = math.dist(nodes[start], nodes[end])
                if not math.isfinite(length) or length <= _EPS:
                    raise ValueError("Official edges need finite nonzero lengths")
                adjacency[start][end] = length
                adjacency[end][start] = length
            if len(references) >= 4 and references[0] == references[-1]:
                closed_ways.append(references)
        return cls(nodes, adjacency, closed_ways)

    def route(self, node_ids: Sequence[str]) -> OpenTopoRoute:
        if isinstance(node_ids, (str, bytes)) or not node_ids:
            raise ValueError("An ordered list of route node IDs is required")
        selected = list(node_ids)
        if any(not isinstance(identifier, str) or identifier not in self.nodes for identifier in selected):
            raise ValueError("Configured route contains an unknown node")
        for start, end in zip(selected, selected[1:]):
            if end not in self.adjacency[start]:
                raise ValueError(f"No official edge connects {start} to {end}")
        return OpenTopoRoute(selected, [self.nodes[identifier] for identifier in selected])

    def alternative_route(self, start_id: str, blocked_next_id: str,
                          require_internal: bool = True,
                          outer_node_ids: Sequence[str] | None = None) -> OpenTopoRoute | None:
        """Remove the blocked edge both ways and find a short open bypass.

        With ``require_internal``, a bypass must visit a node outside the
        caller's outer route. Otherwise internal bypasses are preferred when an
        outer route is supplied, with any remaining bypass as fallback. The
        graph is unchanged. Positive-length Dijkstra search retains visited
        nodes in its state so a spur-and-return cannot fake an internal bypass.
        This intentionally targets the small supplied topological graph.
        """
        if start_id not in self.nodes or blocked_next_id not in self.adjacency[start_id]:
            raise ValueError("The blocked segment must be an official adjacent edge")
        if isinstance(outer_node_ids, (str, bytes)):
            raise ValueError("Outer node IDs must be a sequence, not a string")
        outer = set(outer_node_ids or [])
        if any(identifier not in self.nodes for identifier in outer):
            raise ValueError("Outer route contains an unknown node")
        if require_internal and not outer:
            raise ValueError("An outer route is required to identify internal nodes")
        blocked = frozenset((start_id, blocked_next_id))

        def search(needs_internal: bool) -> list[str] | None:
            serial = count()
            initial_internal = bool(outer) and start_id not in outer
            heap = [(0.0, next(serial), (start_id,), frozenset((start_id,)), initial_internal)]
            best = {(start_id, frozenset((start_id,)), initial_internal): 0.0}
            while heap:
                cost, _, path, visited, has_internal = heapq.heappop(heap)
                current = path[-1]
                if cost > best[(current, visited, has_internal)]:
                    continue
                if current == blocked_next_id:
                    if has_internal or not needs_internal:
                        return list(path)
                    continue
                for neighbour, distance in self.adjacency[current].items():
                    if neighbour in visited or frozenset((current, neighbour)) == blocked:
                        continue
                    next_visited = visited | {neighbour}
                    next_internal = has_internal or (bool(outer) and neighbour not in outer)
                    next_cost = cost + distance
                    state = (neighbour, next_visited, next_internal)
                    if next_cost >= best.get(state, math.inf):
                        continue
                    best[state] = next_cost
                    heapq.heappush(heap, (next_cost, next(serial), path + (neighbour,),
                                          next_visited, next_internal))
            return None

        selected = search(True) if outer else None
        if selected is None and not require_internal:
            selected = search(False)
        return self.route(selected) if selected is not None else None


@dataclass(frozen=True)
class OfficialToOdom:
    """Inverse of the checked odom-to-official similarity fit."""

    scale: float
    rotation_rad: float
    translation_x: float
    translation_y: float
    rmse_m: float

    def point(self, x: float, y: float) -> Point:
        """Convert official XY to the odometry frame used in the first lap."""
        source_x, source_y = _points([(x, y)])[0]
        dx, dy = source_x - self.translation_x, source_y - self.translation_y
        c, s = math.cos(self.rotation_rad), math.sin(self.rotation_rad)
        result = ((c * dx + s * dy) / self.scale, (-s * dx + c * dy) / self.scale)
        if not all(math.isfinite(value) for value in result):
            raise ValueError("Frame conversion produced non-finite coordinates")
        return result

    def points(self, values: Sequence[Point]) -> list[Point]:
        return [self.point(x, y) for x, y in _points(values)]


def validated_alignment(diagnostic_dict: dict, max_rmse_m: float,
                        min_scale: float, max_scale: float) -> OfficialToOdom:
    """Check numeric fit quality and provide its inverse transformation.

    The caller MUST separately establish that the recorded lap and official
    route have a reliable start correspondence and travel direction, which can
    be obtained by automatic phase/direction fitting rather than a fixed start.
    Small residuals cannot establish that correspondence or make sparse chords
    safe driving paths. ``rmse_m`` is measured in official-map coordinates.
    """
    if (not all(math.isfinite(value) for value in (max_rmse_m, min_scale, max_scale))
            or max_rmse_m < 0.0 or min_scale <= 0.0 or max_scale < min_scale):
        raise ValueError("Invalid alignment acceptance limits")
    if not isinstance(diagnostic_dict, dict) or diagnostic_dict.get("available") is False:
        raise ValueError("No usable alignment diagnostic")
    if diagnostic_dict.get("transform_direction") != "recorded_odom_to_official_xy":
        raise ValueError("Alignment transform direction is missing or unsupported")
    names = ("scale", "rotation_rad", "translation_x", "translation_y", "rmse_m")
    try:
        values = {name: float(diagnostic_dict[name]) for name in names}
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError("Alignment needs complete finite transform and error fields") from exc
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("Alignment fields must be finite")
    if not min_scale <= values["scale"] <= max_scale:
        raise ValueError("Alignment scale is outside the configured interval")
    if not 0.0 <= values["rmse_m"] <= max_rmse_m:
        raise ValueError("Alignment residual exceeds the configured limit")
    return OfficialToOdom(**values)


@dataclass(frozen=True)
class DetourResult:
    points: list[Point]
    done: bool
    reason: str
    segment_index: int
    progress_m: float

    @property
    def available(self) -> bool:
        return not self.done and len(self.points) == 3


class DetourTracker:
    """Follow one open polyline in order without smoothing its corners.

    Start is allowed only within ``entry_radius_m`` of the first point. Targets
    stay on the current segment until its endpoint is within waypoint radius;
    at most one segment transition is committed per update. An unavailable
    result leaves tracker state unchanged. The caller decides whether to stop.
    """

    def __init__(self, points: Sequence[Point], waypoint_radius_m: float = 1.0,
                 max_cross_track_m: float = 3.0, lookahead_m: float = 0.5,
                 spacing_m: float = 0.5, entry_radius_m: float | None = None) -> None:
        self.points = _points(points)
        if len(self.points) < 2:
            raise ValueError("A detour needs at least two points")
        if entry_radius_m is None:
            entry_radius_m = waypoint_radius_m
        limits = (waypoint_radius_m, max_cross_track_m, lookahead_m, spacing_m, entry_radius_m)
        if not all(math.isfinite(value) and value > 0.0 for value in limits):
            raise ValueError("Tracker distances must be finite and positive")
        self.waypoint_radius_m = waypoint_radius_m
        self.max_cross_track_m = max_cross_track_m
        self.lookahead_m = lookahead_m
        self.spacing_m = spacing_m
        self.entry_radius_m = entry_radius_m
        self.cumulative_m = [0.0]
        for start, end in zip(self.points, self.points[1:]):
            length = math.dist(start, end)
            if not math.isfinite(length) or length <= _EPS:
                raise ValueError("Detour segments must have nonzero finite length")
            self.cumulative_m.append(self.cumulative_m[-1] + length)
        self.length_m = self.cumulative_m[-1]
        if not math.isfinite(self.length_m):
            raise ValueError("Detour length must be finite")
        self.reset()

    def reset(self) -> None:
        self.started = False
        self.done = False
        self.segment_index = 0
        self.progress_m = 0.0

    def _unavailable(self, reason: str) -> DetourResult:
        return DetourResult([], False, reason, self.segment_index, self.progress_m)

    def _projection(self, position: Point, index: int) -> tuple[float, float]:
        start, end = self.points[index:index + 2]
        length = self.cumulative_m[index + 1] - self.cumulative_m[index]
        dx, dy = end[0] - start[0], end[1] - start[1]
        along = (position[0] - start[0]) * (dx / length) + (position[1] - start[1]) * (dy / length)
        fraction = max(0.0, min(1.0, along / length))
        projected = (start[0] + fraction * dx, start[1] + fraction * dy)
        return self.cumulative_m[index] + fraction * length, math.dist(position, projected)

    def update(self, x: float, y: float, yaw: float) -> DetourResult:
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return self._unavailable("invalid_pose")
        if self.done:
            return DetourResult([], True, "done", self.segment_index, self.progress_m)
        position = (x, y)
        if not self.started and math.dist(position, self.points[0]) > self.entry_radius_m:
            return self._unavailable("entry_not_reached")
        index = self.segment_index
        projected, cross_track = self._projection(position, index)
        if cross_track > self.max_cross_track_m:
            return self._unavailable("cross_track_exceeded")
        if math.dist(position, self.points[index + 1]) <= self.waypoint_radius_m:
            if index == len(self.points) - 2:
                self.started = self.done = True
                self.progress_m = self.length_m
                return DetourResult([], True, "done", index, self.progress_m)
            index += 1
            projected, cross_track = self._projection(position, index)
            if cross_track > self.max_cross_track_m:
                return self._unavailable("cross_track_exceeded")
        progress = max(self.progress_m, projected)
        remaining = self.cumulative_m[index + 1] - progress
        if remaining <= 3.0 * _EPS:
            return self._unavailable("insufficient_forward_route")
        # Compress the preview near an endpoint while keeping ABC distinct.
        first = min(self.lookahead_m, remaining / 3.0)
        second = min(first + self.spacing_m, (first + remaining) / 2.0)
        third = min(second + self.spacing_m, remaining)
        start, end = self.points[index:index + 2]
        length = self.cumulative_m[index + 1] - self.cumulative_m[index]
        c, s = math.cos(yaw), math.sin(yaw)
        targets = []
        for offset in (first, second, third):
            fraction = (progress + offset - self.cumulative_m[index]) / length
            dx = start[0] + fraction * (end[0] - start[0]) - x
            dy = start[1] + fraction * (end[1] - start[1]) - y
            target = (c * dx + s * dy, -s * dx + c * dy)
            if not all(math.isfinite(value) for value in target):
                return self._unavailable("invalid_target")
            if target[0] < -_EPS:
                return self._unavailable("target_behind_vehicle")
            targets.append(target)
        if any(math.dist(a, b) <= _EPS for a, b in zip(targets, targets[1:])):
            return self._unavailable("insufficient_forward_route")
        self.started = True
        self.segment_index = index
        self.progress_m = progress
        return DetourResult(targets, False, "tracking", index, progress)
