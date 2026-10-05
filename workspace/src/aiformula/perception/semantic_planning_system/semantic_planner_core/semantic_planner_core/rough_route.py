"""Small, ROS-independent route reference and visual heading supervisor.

All positions are metres in one continuous odometry frame. Angles are radians.
The caller must establish that the autonomous recording is one completed lap;
this module neither detects lap completion nor corrects odometry drift.
"""

from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class RouteSample:
    x: float
    y: float
    yaw: float
    excluded: bool = False


class RouteReference:
    """A closed, lightly smoothed first-lap trace with protected exclusions.

    ``points`` omits a repeated endpoint; ``cumulative_m`` has one additional
    entry, the full length including the closing segment. Short excluded runs
    are straightened for the coarse drawing but remain unavailable to steering.
    Smoothing cannot cross an excluded run. The recording must mark avoidance
    and branch manoeuvres excluded when their geometry should be protected.
    """

    def __init__(
        self,
        samples: List[RouteSample],
        spacing_m: float = 0.2,
        smooth_window_m: float = 1.0,
        max_detour_gap_m: float = 2.0,
    ) -> None:
        values = (spacing_m, smooth_window_m, max_detour_gap_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Route parameters must be finite")
        if spacing_m <= 0.0 or smooth_window_m < 0.0 or max_detour_gap_m < 0.0:
            raise ValueError("Spacing must be positive; windows must be nonnegative")
        clean: List[RouteSample] = []
        for sample in samples:
            if not all(math.isfinite(value) for value in (sample.x, sample.y, sample.yaw)):
                raise ValueError("Route samples must be finite")
            if clean and math.hypot(sample.x - clean[-1].x, sample.y - clean[-1].y) < 1e-8:
                previous = clean[-1]
                clean[-1] = RouteSample(previous.x, previous.y, sample.yaw,
                                        previous.excluded or sample.excluded)
            else:
                clean.append(sample)
        if len(clean) > 1 and math.hypot(clean[-1].x - clean[0].x, clean[-1].y - clean[0].y) < 1e-8:
            first = clean[0]
            clean[0] = RouteSample(first.x, first.y, first.yaw,
                                   first.excluded or clean[-1].excluded)
            clean.pop()
        if len(clean) < 3:
            raise ValueError("A closed reference needs at least three distinct samples")

        raw = [(sample.x, sample.y) for sample in clean]
        good = [not sample.excluded for sample in clean]
        self._straighten_short_exclusions(raw, good, max_detour_gap_m)
        raw_cumulative = self._cumulative(raw)
        raw_length = raw_cumulative[-1]
        if raw_length < spacing_m * 2.0:
            raise ValueError("Recorded route is too short for the requested spacing")
        count = max(3, int(math.ceil(raw_length / spacing_m)))
        step = raw_length / count
        points = []
        trusted = []
        for index in range(count):
            distance = index * step
            segment = min(bisect_right(raw_cumulative, distance) - 1, len(raw) - 1)
            following = (segment + 1) % len(raw)
            segment_length = raw_cumulative[segment + 1] - raw_cumulative[segment]
            fraction = (distance - raw_cumulative[segment]) / segment_length
            points.append((raw[segment][0] + fraction * (raw[following][0] - raw[segment][0]),
                           raw[segment][1] + fraction * (raw[following][1] - raw[segment][1])))
            trusted.append(good[segment] and good[following])

        # A local circular average treats the lap seam just like any other
        # section. Keep sharp corners and all exclusion boundaries unchanged.
        radius = min((count - 1) // 2, int(smooth_window_m / (2.0 * step)))
        smoothed = list(points)
        if radius > 0:
            for index in range(count):
                neighbours = [(index + offset) % count for offset in range(-radius, radius + 1)]
                if not all(trusted[item] for item in neighbours):
                    continue
                before = points[neighbours[0]]
                centre = points[index]
                after = points[neighbours[-1]]
                incoming = math.atan2(centre[1] - before[1], centre[0] - before[0])
                outgoing = math.atan2(after[1] - centre[1], after[0] - centre[0])
                if abs(_wrap(outgoing - incoming)) > math.radians(55.0):
                    continue
                smoothed[index] = (
                    sum(points[item][0] for item in neighbours) / len(neighbours),
                    sum(points[item][1] for item in neighbours) / len(neighbours),
                )
        self.points: List[Tuple[float, float]] = smoothed
        self.trusted: List[bool] = trusted
        self.cumulative_m = self._cumulative(self.points)
        self.length_m = self.cumulative_m[-1]
        # Keep modulo/projection well-defined even for a degenerate result of
        # smoothing or extreme parameter scales. The caller can keep learning.
        if not math.isfinite(self.length_m) or self.length_m <= max(1e-8, spacing_m * 0.01):
            raise ValueError("Smoothed reference has no usable route length")
        self.spacing_m = step

    @staticmethod
    def _cumulative(points: Sequence[Tuple[float, float]]) -> List[float]:
        cumulative = [0.0]
        for index, point in enumerate(points):
            following = points[(index + 1) % len(points)]
            cumulative.append(cumulative[-1] + math.hypot(following[0] - point[0], following[1] - point[1]))
        return cumulative

    @staticmethod
    def _straighten_short_exclusions(points, trusted, maximum):
        if not any(trusted) or all(trusted) or maximum <= 0.0:
            return
        size = len(points)
        anchor = trusted.index(True)
        offset = 1
        while offset < size:
            if trusted[(anchor + offset) % size]:
                offset += 1
                continue
            left_offset = offset - 1
            while offset < size and not trusted[(anchor + offset) % size]:
                offset += 1
            indices = [(anchor + item) % size for item in range(left_offset, offset + 1)]
            distances = [0.0]
            for previous, current in zip(indices, indices[1:]):
                distances.append(distances[-1] + math.hypot(points[current][0] - points[previous][0],
                                                          points[current][1] - points[previous][1]))
            if distances[-1] <= maximum and distances[-1] > 1e-8:
                start, end = points[indices[0]], points[indices[-1]]
                for item, distance in zip(indices[1:-1], distances[1:-1]):
                    fraction = distance / distances[-1]
                    points[item] = (start[0] + fraction * (end[0] - start[0]),
                                    start[1] + fraction * (end[1] - start[1]))

    def point_at(self, s: float) -> Tuple[float, float, bool]:
        """Interpolate at wrapped or unwrapped along-route distance."""
        if not math.isfinite(s):
            raise ValueError("Route progress must be finite")
        distance = s % self.length_m
        index = min(bisect_right(self.cumulative_m, distance) - 1, len(self.points) - 1)
        following = (index + 1) % len(self.points)
        segment_length = self.cumulative_m[index + 1] - self.cumulative_m[index]
        fraction = (distance - self.cumulative_m[index]) / segment_length if segment_length > 1e-10 else 0.0
        start, end = self.points[index], self.points[following]
        return (start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
                self.trusted[index] and self.trusted[following])

    def match(self, x: float, y: float, previous_s: Optional[float], travel_delta: float,
              search_window_m: float, max_distance_m: float) -> Optional[float]:
        """Project near predicted progress; never fall back to global nearest.

        Initialise ``previous_s`` to zero (or one lap length) at the confirmed
        start crossing. The returned progress is unwrapped across lap seams.
        Matching may traverse exclusions, but they cannot supply steering.
        """
        if previous_s is None:
            return None
        if not all(math.isfinite(value) for value in
                   (x, y, previous_s, travel_delta, search_window_m, max_distance_m)):
            return None
        if travel_delta < 0.0 or search_window_m < 0.0 or max_distance_m < 0.0:
            return None
        predicted = previous_s + travel_delta
        lower, upper = predicted - search_window_m, predicted + search_window_m
        best = None
        best_key = (math.inf, math.inf)
        for index, start in enumerate(self.points):
            end = self.points[(index + 1) % len(self.points)]
            begin, finish = self.cumulative_m[index:index + 2]
            length = finish - begin
            if length <= 1e-10:
                continue
            first_lap = math.ceil((lower - finish) / self.length_m)
            last_lap = math.floor((upper - begin) / self.length_m)
            for lap in range(first_lap, last_lap + 1):
                absolute_begin = begin + lap * self.length_m
                fraction_min = max(0.0, (lower - absolute_begin) / length)
                fraction_max = min(1.0, (upper - absolute_begin) / length)
                if fraction_min > fraction_max:
                    continue
                dx, dy = end[0] - start[0], end[1] - start[1]
                fraction = ((x - start[0]) * dx + (y - start[1]) * dy) / (length * length)
                fraction = max(fraction_min, min(fraction_max, fraction))
                distance_sq = (x - start[0] - fraction * dx) ** 2 + (y - start[1] - fraction * dy) ** 2
                progress = absolute_begin + fraction * length
                key = (round(distance_sq, 12), abs(progress - predicted))
                if key < best_key:
                    best_key, best = key, progress
        if best is None or best_key[0] > max_distance_m * max_distance_m:
            return None
        return best

    def _interval_trusted(self, start: float, end: float) -> bool:
        if end <= start or end - start >= self.length_m:
            return False
        for index in range(len(self.points)):
            if self.trusted[index] and self.trusted[(index + 1) % len(self.points)]:
                continue
            begin, finish = self.cumulative_m[index:index + 2]
            first_lap = math.ceil((start - finish) / self.length_m)
            last_lap = math.floor((end - begin) / self.length_m)
            if first_lap <= last_lap:
                return False
        return True

    def direction_candidates(self, x: float, y: float, yaw: float, s: float,
                             lookahead_m: float, progress_tolerance_m: float) -> List[float]:
        """Vehicle-relative bearings to trusted targets in a short progress band.

        Bearings originate at the actual vehicle position, so a lateral offset
        also produces a gentle return direction. Candidates behind the vehicle
        and paths crossing an excluded section are discarded.
        """
        if not all(math.isfinite(value) for value in
                   (x, y, yaw, s, lookahead_m, progress_tolerance_m)):
            return []
        if lookahead_m <= 0.0 or progress_tolerance_m < 0.0:
            return []
        minimum = max(self.spacing_m * 0.5, lookahead_m - progress_tolerance_m)
        maximum = lookahead_m + progress_tolerance_m
        if minimum > maximum or maximum >= self.length_m * 0.5:
            return []
        steps = min(30, max(1, int(math.ceil((maximum - minimum) / self.spacing_m))))
        candidates = []
        for index in range(steps + 1):
            target_s = s + minimum + (maximum - minimum) * index / steps
            target_x, target_y, trusted = self.point_at(target_s)
            if not trusted or not self._interval_trusted(s, target_s):
                continue
            dx, dy = target_x - x, target_y - y
            if math.hypot(dx, dy) < 1e-8:
                continue
            angle = _wrap(math.atan2(dy, dx) - yaw)
            if abs(angle) < math.pi * 0.5:
                if not candidates or abs(_wrap(angle - candidates[-1])) > 1e-8:
                    candidates.append(angle)
        return candidates


class HeadingGuard:
    """Return a bounded rotation of current visual points, never an accumulator."""

    def __init__(self, angle_tolerance_rad: float, persistence_frames: int,
                 max_correction_rad: float, max_step_rad: float) -> None:
        values = (angle_tolerance_rad, max_correction_rad, max_step_rad)
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("Heading limits must be finite and nonnegative")
        if angle_tolerance_rad > math.pi or max_correction_rad > math.pi:
            raise ValueError("Heading limits cannot exceed pi")
        if persistence_frames < 1 or int(persistence_frames) != persistence_frames:
            raise ValueError("Persistence must be a positive integer")
        self.angle_tolerance_rad = angle_tolerance_rad
        self.persistence_frames = int(persistence_frames)
        self.max_correction_rad = max_correction_rad
        self.max_step_rad = max_step_rad
        self.reset()

    def reset(self) -> None:
        self.violation_count = 0
        self.reason = "reset"
        self._correction = 0.0
        self._sign = 0

    def update(self, visual_angle: float, map_angles: Sequence[float], enabled: bool = True) -> float:
        if not enabled:
            self.reset()
            self.reason = "disabled"
            return 0.0
        valid_angles = [value for value in map_angles if math.isfinite(value)]
        if not math.isfinite(visual_angle) or not valid_angles:
            self.reset()
            self.reason = "unavailable"
            return 0.0
        error = min((_wrap(angle - visual_angle) for angle in valid_angles), key=abs)
        if abs(error) <= self.angle_tolerance_rad + 1e-12:
            self.reset()
            self.reason = "within_tolerance"
            return 0.0
        sign = 1 if error > 0.0 else -1
        if sign != self._sign:
            self.violation_count = 0
            self._correction = 0.0
        self._sign = sign
        self.violation_count = min(self.violation_count + 1, self.persistence_frames)
        if self.violation_count < self.persistence_frames:
            self.reason = "pending"
            return 0.0
        target = sign * min(self.max_correction_rad, abs(error) - self.angle_tolerance_rad)
        delta = max(-self.max_step_rad, min(self.max_step_rad, target - self._correction))
        self._correction += delta
        self.reason = "correcting"
        return self._correction
