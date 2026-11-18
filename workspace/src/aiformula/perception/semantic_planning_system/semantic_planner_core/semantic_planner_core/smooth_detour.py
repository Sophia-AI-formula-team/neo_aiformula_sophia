"""Cubic clamped uniform B-spline turns and ordered open-path tracking.

The official graph only supplies junction chords; neither these curves nor a
curvature limit establishes that a curve stays on the road or avoids obstacles.
All inputs and output points use metres in the same continuous odom frame.
"""
from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]
EPS = 1e-7


def _distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _unit(a: Point, b: Point) -> Point:
    distance = _distance(a, b)
    if distance < EPS:
        raise ValueError('duplicate_route_points')
    return ((b[0] - a[0]) / distance, (b[1] - a[1]) / distance)


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _bspline_point(controls: Sequence[Point], t: float) -> Point:
    """Evaluate a six-control cubic uniform B-spline, clamped at each end."""
    if t >= 1.0:
        return controls[-1]
    knots = (0., 0., 0., 0., 1. / 3., 2. / 3., 1., 1., 1., 1.)
    k = max(3, min(5, bisect_right(knots, max(0., t)) - 1))
    work = [list(controls[k - 3 + j]) for j in range(4)]
    for level in range(1, 4):
        for j in range(3, level - 1, -1):
            i = k - 3 + j
            denominator = knots[i + 4 - level] - knots[i]
            alpha = (t - knots[i]) / denominator if denominator > EPS else 0.0
            for axis in (0, 1):
                work[j][axis] = (1.0 - alpha) * work[j - 1][axis] + alpha * work[j][axis]
    return (work[3][0], work[3][1])


def _curvature(a: Point, b: Point, c: Point) -> float:
    ab, bc, ac = _distance(a, b), _distance(b, c), _distance(a, c)
    if min(ab, bc, ac) < EPS:
        return 0.0
    cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
    return 2.0 * abs(cross) / (ab * bc * ac)


def _turn(start: Point, initial: Point, end: Point, final: Point,
          spacing: float, tangent_factor: float) -> List[Point]:
    chord = _distance(start, end)
    if chord < EPS:
        raise ValueError('turn_has_zero_length')
    handle = chord * tangent_factor
    controls = [start,
                (start[0] + initial[0] * handle, start[1] + initial[1] * handle),
                (start[0] + initial[0] * handle * 2, start[1] + initial[1] * handle * 2),
                (end[0] - final[0] * handle * 2, end[1] - final[1] * handle * 2),
                (end[0] - final[0] * handle, end[1] - final[1] * handle), end]
    polygon = sum(_distance(a, b) for a, b in zip(controls, controls[1:]))
    count = max(35, int(math.ceil(polygon / spacing)) * 2)
    return [_bspline_point(controls, i / count) for i in range(count + 1)]


def _append_line(points: List[Point], end: Point, spacing: float) -> None:
    length = _distance(points[-1], end)
    if length < EPS:
        return
    start = points[-1]
    count = max(1, int(math.ceil(length / spacing)))
    points.extend((start[0] + (end[0] - start[0]) * i / count,
                   start[1] + (end[1] - start[1]) * i / count)
                  for i in range(1, count + 1))


def _append_curve(points: List[Point], curve: Sequence[Point]) -> None:
    if _distance(points[-1], curve[0]) > 0.001:
        raise ValueError('curve_not_connected')
    for point in curve[1:]:
        if _distance(points[-1], point) > EPS:
            points.append(point)


def _reference_tangent(reference, progress: float) -> Point:
    a = reference.point_at(progress - 0.3)
    b = reference.point_at(progress + 0.3)
    if not a[2] or not b[2]:
        raise ValueError('untrusted_exit_reference')
    return _unit(a[:2], b[:2])


@dataclass
class SmoothPath:
    points: List[Point]
    cumulative_m: List[float]
    join_progress_m: float
    max_curvature: float
    trim_m: float


def build_smooth_detour(reference, route: Sequence[Point], exit_progress: float,
                        pose: Tuple[float, float, float], min_radius_m: float,
                        max_trim_m: float, spacing_m: float = 0.15,
                        post_exit_m: float = 6.0) -> SmoothPath:
    """Connect the current pose to all topo chords, then to the outer reference.

    Junction offsets and the six-control B-spline handles are searched within
    bounded limits. Every spline is sampled for its geometric curvature. A
    ValueError means no candidate passed; it must not be silently overridden.
    The radius is a *configuration assumption*, not a measured vehicle limit.
    """
    if (len(route) < 3 or len(pose) != 3 or
            not all(math.isfinite(v) for v in pose + (exit_progress, min_radius_m,
                                                       max_trim_m, spacing_m, post_exit_m)) or
            min(min_radius_m, max_trim_m, spacing_m, post_exit_m) <= 0.0):
        raise ValueError('invalid_smooth_parameters')
    route = [(float(p[0]), float(p[1])) for p in route]
    if not all(math.isfinite(v) for p in route for v in p):
        raise ValueError('invalid_topology_coordinates')
    lengths = [_distance(a, b) for a, b in zip(route, route[1:])]
    if min(lengths) < 1.0:
        raise ValueError('topology_edge_too_short')
    max_trim = min(max_trim_m, min(lengths) * 0.45)
    # Prefer a broad turn and search downwards only when required by the graph.
    trims = [max_trim * ratio for ratio in (1.0, 0.85, 0.7, 0.55)]
    pose_xy = (pose[0], pose[1])
    initial = (math.cos(pose[2]), math.sin(pose[2]))
    last_reason = 'no_curvature_feasible_spline'
    for trim in trims:
        if trim < max(1.0, min_radius_m * 0.5):
            continue
        try:
            directions = [_unit(a, b) for a, b in zip(route, route[1:])]
            entry_end = (route[0][0] + trim * directions[0][0],
                         route[0][1] + trim * directions[0][1])
            if ((entry_end[0] - pose_xy[0]) * initial[0] +
                    (entry_end[1] - pose_xy[1]) * initial[1]) <= 0.3:
                raise ValueError('entry_is_behind_vehicle')
            exit_point = reference.point_at(exit_progress + trim)
            if not exit_point[2] or not reference._interval_trusted(exit_progress, exit_progress + trim + post_exit_m):
                raise ValueError('untrusted_exit_reference')
            exit_xy = exit_point[:2]
            exit_heading = _reference_tangent(reference, exit_progress + trim)
            for factor in (0.18, 0.12, 0.24, 0.09, 0.30):
                points = [pose_xy]
                _append_curve(points, _turn(pose_xy, initial, entry_end, directions[0], spacing_m, factor))
                for index in range(1, len(route) - 1):
                    before, after = directions[index - 1], directions[index]
                    turn_start = (route[index][0] - trim * before[0], route[index][1] - trim * before[1])
                    turn_end = (route[index][0] + trim * after[0], route[index][1] + trim * after[1])
                    _append_line(points, turn_start, spacing_m)
                    _append_curve(points, _turn(turn_start, before, turn_end, after, spacing_m, factor))
                before = directions[-1]
                exit_start = (route[-1][0] - trim * before[0], route[-1][1] - trim * before[1])
                _append_line(points, exit_start, spacing_m)
                _append_curve(points, _turn(exit_start, before, exit_xy, exit_heading, spacing_m, factor))
                # Complete the return curve first, then follow the learned
                # outer reference for two metres before handing back to vision.
                join_index = None
                steps = max(1, int(math.ceil(post_exit_m / spacing_m)))
                for i in range(1, steps + 1):
                    target = reference.point_at(exit_progress + trim + post_exit_m * i / steps)
                    if not target[2]:
                        raise ValueError('untrusted_exit_reference')
                    if _distance(points[-1], target[:2]) > EPS:
                        points.append(target[:2])
                    if join_index is None and post_exit_m * i / steps >= 2.0:
                        join_index = len(points) - 1
                if join_index is None:
                    raise ValueError('insufficient_exit_stabilization')
                # Reject cusps and excessive curvature on the complete sampled
                # route, including the straight/curve and curve/outer joins.
                maximum = max(_curvature(a, b, c) for a, b, c in zip(points, points[1:], points[2:]))
                if not math.isfinite(maximum) or maximum > 1.0 / min_radius_m + 1e-3:
                    continue
                if any(_distance(a, b) < EPS for a, b in zip(points, points[1:])):
                    continue
                cumulative = [0.0]
                for a, b in zip(points, points[1:]):
                    cumulative.append(cumulative[-1] + _distance(a, b))
                return SmoothPath(points, cumulative, cumulative[join_index], maximum, trim)
        except ValueError as exc:
            last_reason = str(exc)
    raise ValueError(last_reason)


@dataclass
class SmoothResult:
    points: List[Point]
    done: bool
    reason: str
    progress_m: float
    segment_index: int

    @property
    def available(self) -> bool:
        return not self.done and len(self.points) == 3


class SmoothDetourTracker:
    """Project locally onto one ordered path; publish three vehicle-frame points."""
    def __init__(self, path: SmoothPath, max_cross_track_m: float,
                 lookahead_m: float, spacing_m: float, max_target_angle_deg: float,
                 finish_tolerance_m: float = 0.7):
        if not all(math.isfinite(v) and v > 0.0 for v in
                   (max_cross_track_m, lookahead_m, spacing_m, max_target_angle_deg, finish_tolerance_m)):
            raise ValueError('invalid_tracker_limits')
        if max_target_angle_deg >= 90.0:
            raise ValueError('target_angle_must_be_less_than_90')
        self.path = path
        self.points = path.points
        self.progress_m = 0.0
        self.segment_index = 0
        self.started = False
        self.last_pose = None
        self.max_cross_track_m = max_cross_track_m
        self.lookahead_m = lookahead_m
        self.spacing_m = spacing_m
        self.max_target_angle = math.radians(max_target_angle_deg)
        self.finish_tolerance_m = finish_tolerance_m

    def _point_at(self, progress: float) -> Point:
        s = max(0.0, min(progress, self.path.cumulative_m[-1]))
        index = min(bisect_right(self.path.cumulative_m, s) - 1, len(self.points) - 2)
        a, b = self.points[index:index + 2]
        length = self.path.cumulative_m[index + 1] - self.path.cumulative_m[index]
        f = (s - self.path.cumulative_m[index]) / length if length > EPS else 0.0
        return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))

    def update(self, x: float, y: float, yaw: float) -> SmoothResult:
        def unavailable(reason):
            return SmoothResult([], False, reason, self.progress_m, self.segment_index)
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return unavailable('invalid_pose')
        displacement = _distance((x, y), self.last_pose) if self.last_pose else 0.0
        lower = max(0.0, self.progress_m - 0.5)
        upper = min(self.path.cumulative_m[-1], self.progress_m + max(3.0, displacement * 2 + 1.0))
        best = None
        for index in range(max(0, bisect_right(self.path.cumulative_m, lower) - 1), len(self.points) - 1):
            start_s, end_s = self.path.cumulative_m[index:index + 2]
            if start_s > upper:
                break
            a, b = self.points[index:index + 2]
            length = end_s - start_s
            direction = math.atan2(b[1] - a[1], b[0] - a[0])
            if abs(_angle_diff(direction, yaw)) > math.radians(80.0):
                continue
            fraction = ((x - a[0]) * (b[0] - a[0]) + (y - a[1]) * (b[1] - a[1])) / (length * length)
            lo = max(0., (lower - start_s) / length)
            hi = min(1., (upper - start_s) / length)
            if lo > hi:
                continue
            fraction = max(lo, min(hi, fraction))
            candidate = (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]))
            distance = _distance((x, y), candidate)
            progress = start_s + fraction * length
            key = (distance, abs(progress - self.progress_m))
            if best is None or key < best[0]:
                best = (key, progress, index)
        if best is None or best[0][0] > self.max_cross_track_m:
            return unavailable('cross_track_or_heading_exceeded')
        progress, index = max(self.progress_m, best[1]), best[2]
        if progress >= self.path.join_progress_m:
            here = self._point_at(progress)
            ahead = self._point_at(min(progress + 0.5, self.path.cumulative_m[-1]))
            heading = math.atan2(ahead[1] - here[1], ahead[0] - here[0])
            if (best[0][0] <= self.finish_tolerance_m and
                    abs(_angle_diff(heading, yaw)) <= math.radians(20.0)):
                self.progress_m, self.segment_index = progress, index
                return SmoothResult([], True, 'done', progress, index)
        if progress + self.lookahead_m + 2.0 * self.spacing_m > self.path.cumulative_m[-1]:
            return unavailable('insufficient_forward_route')
        c, s = math.cos(yaw), math.sin(yaw)
        targets = []
        for offset in (self.lookahead_m, self.lookahead_m + self.spacing_m,
                       self.lookahead_m + 2.0 * self.spacing_m):
            px, py = self._point_at(progress + offset)
            dx, dy = px - x, py - y
            target = (c * dx + s * dy, -s * dx + c * dy)
            if target[0] <= EPS or abs(math.atan2(target[1], target[0])) > self.max_target_angle:
                return unavailable('target_angle_exceeded')
            targets.append(target)
        self.started = True
        self.last_pose = (x, y)
        self.progress_m, self.segment_index = progress, index
        return SmoothResult(targets, False, 'tracking', progress, index)
