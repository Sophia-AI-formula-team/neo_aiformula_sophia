"""Fail-closed, offline route extraction from an ordered first lap and lane map.

The map is evidence, not a free-space or obstacle map.  A successful result is a
candidate route requiring inspection before an independently armed follower may
use it.  No ROS dependency is needed; NumPy is the only external dependency.
"""

import math
from collections.abc import Mapping

import numpy as np


DEFAULT_CONFIG = {
    "sample_spacing_m": 0.20,
    "min_lap_length_m": 15.0,
    "max_closure_distance_m": 1.5,
    "max_closure_heading_rad": math.radians(35.0),
    "max_trajectory_step_m": 2.0,
    "min_lane_width_m": 1.0,
    "max_lane_width_m": 5.0,
    "max_side_distance_m": 4.0,
    "longitudinal_window_m": 0.35,
    "min_edge_offset_m": 0.20,
    "edge_cluster_gap_m": 0.30,
    "min_edge_points": 3,
    "min_support_fraction": 0.98,
    "max_unsupported_gap_m": 0.40,
    "vehicle_half_width_m": 0.30,
    "safety_margin_m": 0.20,
    "smoothing_iterations": 12,
    "max_curvature_1pm": 0.80,
    "max_speed_mps": 1.0,
    "max_lateral_accel_mps2": 0.60,
    "max_accel_mps2": 0.40,
    "max_decel_mps2": 0.50,
    "max_route_samples": 5000,
    "max_input_points": 1000000,
}


class RouteBuildError(ValueError):
    """No safe candidate can be produced; diagnostics explain the failed gate."""

    def __init__(self, message, diagnostics=None):
        super().__init__(message)
        self.diagnostics = dict(diagnostics or {})
        self.diagnostics["valid"] = False
        self.diagnostics["rejection_reason"] = str(message)


def _config(config):
    result = dict(DEFAULT_CONFIG)
    if config is not None:
        if not isinstance(config, Mapping):
            raise RouteBuildError("Route configuration must be a mapping")
        unknown = set(config) - set(result)
        if unknown:
            raise RouteBuildError("Unknown route settings: " + ", ".join(sorted(map(str, unknown))))
        result.update(config)
    integer_keys = {"min_edge_points", "smoothing_iterations", "max_route_samples", "max_input_points"}
    for key, value in result.items():
        if isinstance(value, bool) or not isinstance(value, (int, float, np.number)):
            raise RouteBuildError("Route setting must be numeric: " + key)
        if not math.isfinite(float(value)) or float(value) <= 0:
            raise RouteBuildError("Route setting must be finite and positive: " + key)
        if key in integer_keys:
            if int(value) != value:
                raise RouteBuildError("Route setting must be an integer: " + key)
            result[key] = int(value)
        else:
            result[key] = float(value)
    if result["min_support_fraction"] > 1.0:
        raise RouteBuildError("min_support_fraction must not exceed one")
    if result["max_closure_heading_rad"] > math.pi:
        raise RouteBuildError("max_closure_heading_rad must not exceed pi")
    if result["min_lane_width_m"] >= result["max_lane_width_m"]:
        raise RouteBuildError("Lane width interval is empty")
    clearance = result["vehicle_half_width_m"] + result["safety_margin_m"]
    if 2.0 * clearance > result["min_lane_width_m"]:
        raise RouteBuildError("Minimum lane width cannot contain the vehicle and safety margin")
    if result["max_route_samples"] < 8:
        raise RouteBuildError("max_route_samples must be at least eight")
    return result


def _points(value, columns, label, limit):
    try:
        points = np.asarray(value, dtype=np.float64)
    except (ValueError, TypeError) as exc:
        raise RouteBuildError(label + " is not a numeric array") from exc
    if points.ndim != 2 or points.shape[1] != columns or len(points) < 8:
        raise RouteBuildError(label + " must have shape N x " + str(columns) + " with N >= 8")
    if len(points) > limit:
        raise RouteBuildError(label + " exceeds the configured point limit")
    if not np.all(np.isfinite(points)):
        raise RouteBuildError(label + " contains non-finite values")
    return points


def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _check_intersections(points, label, diagnostics):
    """Reject nonadjacent intersections, including touches and overlapping runs."""
    following = np.roll(points, -1, axis=0)
    count = len(points)
    for index in range(count - 2):
        indices = np.arange(index + 2, count)
        if index == 0:
            indices = indices[indices != count - 1]
        if not len(indices):
            continue
        a, b = points[index], following[index]
        c, d = points[indices], following[indices]
        lower = np.maximum(np.minimum(a, b), np.minimum(c, d))
        upper = np.minimum(np.maximum(a, b), np.maximum(c, d))
        overlap = np.all(lower <= upper + 1e-9, axis=1)
        ab_c = _cross(b - a, c - a)
        ab_d = _cross(b - a, d - a)
        cd_a = _cross(d - c, a - c)
        cd_b = _cross(d - c, b - c)
        hits = overlap & (ab_c * ab_d <= 1e-12) & (cd_a * cd_b <= 1e-12)
        if np.any(hits):
            raise RouteBuildError(label + " self-intersects; route order is ambiguous", diagnostics)


def _resample_closed(points, spacing, max_samples):
    lengths = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    if np.any(lengths < 1e-8):
        raise RouteBuildError("Closed route has a zero-length segment")
    total = float(np.sum(lengths))
    count = max(8, int(math.ceil(total / spacing)))
    if count > max_samples:
        raise RouteBuildError("Route exceeds max_route_samples; check track scale")
    distance = np.arange(count, dtype=np.float64) * (total / count)
    knots = np.concatenate(([0.0], np.cumsum(lengths)))
    closed = np.vstack((points, points[:1]))
    samples = np.column_stack([
        np.interp(distance, knots, closed[:, axis]) for axis in range(2)
    ])
    return samples, distance, total


def _normals(points):
    tangent = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)
    norm = np.linalg.norm(tangent, axis=1)
    if np.any(norm < 1e-7):
        raise RouteBuildError("Route contains a direction reversal or unresolved cusp")
    tangent /= norm[:, None]
    return np.column_stack((-tangent[:, 1], tangent[:, 0]))


class _LaneIndex:
    def __init__(self, points, cell_size):
        self.points = points
        self.cell_size = cell_size
        self.cells = {}
        keys = np.floor(points / cell_size).astype(np.int64)
        for index, key in enumerate(keys):
            self.cells.setdefault((int(key[0]), int(key[1])), []).append(index)

    def nearby(self, point, radius):
        low = np.floor((point - radius) / self.cell_size).astype(np.int64)
        high = np.floor((point + radius) / self.cell_size).astype(np.int64)
        indices = []
        for x in range(int(low[0]), int(high[0]) + 1):
            for y in range(int(low[1]), int(high[1]) + 1):
                indices.extend(self.cells.get((x, y), ()))
        return self.points[indices]


def _nearest_edge(offsets, cfg):
    # Never skip a supported near edge and accidentally select the next lane.
    values = np.sort(offsets[(offsets > 1e-6) &
                             (offsets <= cfg["max_side_distance_m"])])
    if not len(values):
        return float("nan")
    groups = np.split(values, np.flatnonzero(np.diff(values) > cfg["edge_cluster_gap_m"]) + 1)
    for group in groups:
        if len(group) >= cfg["min_edge_points"]:
            return float(np.median(group))
    return float("nan")


def _max_missing_run(valid):
    if np.all(valid):
        return 0
    if not np.any(valid):
        return len(valid)
    start = int(np.flatnonzero(valid)[0])
    run = longest = 0
    for value in np.roll(valid, -start):
        run = 0 if value else run + 1
        longest = max(longest, run)
    return longest


def _corridor(points, normals, lane_index, cfg, diagnostics, stage):
    edges = np.full((len(points), 2), np.nan, dtype=np.float64)
    radius = math.hypot(cfg["max_side_distance_m"], cfg["longitudinal_window_m"])
    for index, (point, normal) in enumerate(zip(points, normals)):
        candidates = lane_index.nearby(point, radius) - point
        tangent = np.array([normal[1], -normal[0]])
        candidates = candidates[np.abs(candidates @ tangent) <= cfg["longitudinal_window_m"]]
        offsets = candidates @ normal
        edges[index] = (_nearest_edge(offsets, cfg), _nearest_edge(-offsets, cfg))
    finite = np.all(np.isfinite(edges), axis=1)
    if np.any(edges[np.isfinite(edges)] < cfg["min_edge_offset_m"]):
        raise RouteBuildError("Route passes too close to an observed lane edge", diagnostics)
    width = np.sum(edges, axis=1)
    plausible = (width >= cfg["min_lane_width_m"]) & (width <= cfg["max_lane_width_m"])
    # An observed but implausible width is evidence against the route, not a gap.
    if np.any(finite & ~plausible):
        diagnostics[stage + "_invalid_width_count"] = int(np.sum(finite & ~plausible))
        raise RouteBuildError("Observed lane width is outside the allowed interval", diagnostics)
    fraction = float(np.mean(finite))
    gaps = _max_missing_run(finite)
    lengths = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    gap_m = float(gaps * np.max(lengths))
    diagnostics[stage + "_support_fraction"] = fraction
    diagnostics[stage + "_max_unsupported_gap_m"] = gap_m
    diagnostics[stage + "_interpolated_sections"] = int(np.sum(~finite))
    if fraction < cfg["min_support_fraction"] or gap_m > cfg["max_unsupported_gap_m"] + 1e-8:
        raise RouteBuildError("Lane evidence is missing over an unsupported route section", diagnostics)
    if not np.all(finite):
        # Only explicitly bounded, sub-metre voxel gaps may be bridged; record them.
        valid_indices = np.flatnonzero(finite)
        extended = np.concatenate((valid_indices - len(points), valid_indices,
                                   valid_indices + len(points)))
        for side in range(2):
            edges[:, side] = np.interp(np.arange(len(points)), extended,
                                      np.tile(edges[finite, side], 3))
    return edges


def _curvature(points):
    previous = points - np.roll(points, 1, axis=0)
    following = np.roll(points, -1, axis=0) - points
    denominator = (np.linalg.norm(previous, axis=1) * np.linalg.norm(following, axis=1) *
                   np.linalg.norm(previous + following, axis=1))
    if np.any(denominator < 1e-10):
        raise RouteBuildError("Route curvature is undefined at a cusp")
    return 2.0 * _cross(previous, following) / denominator


def _speed_profile(points, curvature, cfg):
    speeds = np.minimum(cfg["max_speed_mps"], np.sqrt(
        cfg["max_lateral_accel_mps2"] / np.maximum(np.abs(curvature), 1e-9)))
    segment_length = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    # Circular passes propagate the slowest bend through the start/end seam too.
    for _ in range(3):
        for index in range(len(speeds)):
            following = (index + 1) % len(speeds)
            speeds[following] = min(speeds[following], math.sqrt(
                speeds[index] ** 2 + 2.0 * cfg["max_accel_mps2"] * segment_length[index]))
        for index in range(len(speeds) - 1, -1, -1):
            following = (index + 1) % len(speeds)
            speeds[index] = min(speeds[index], math.sqrt(
                speeds[following] ** 2 + 2.0 * cfg["max_decel_mps2"] * segment_length[index]))
    return speeds


def build_route(trajectory, lane_points, config=None):
    """Build a candidate closed reference, or raise :class:`RouteBuildError`.

    ``trajectory`` is ordered N x 3 ``[x_m, y_m, yaw_rad]`` in the map frame;
    ``lane_points`` is the confirmed N x 2 lane map in that *same* frame.
    The first lap has ended before this function is called.  The result contains
    no duplicate endpoint: followers must wrap the final segment to sample zero.
    A successful geometry check does not verify localization or obstacle safety.
    """
    cfg = _config(config)
    trace = _points(trajectory, 3, "trajectory", cfg["max_input_points"])
    lanes = _points(lane_points, 2, "lane_points", cfg["max_input_points"])
    lanes = np.unique(lanes, axis=0)
    increments = np.linalg.norm(np.diff(trace[:, :2], axis=0), axis=1)
    diagnostics = {
        "valid": False,
        "input_trajectory_points": int(len(trace)),
        "input_lane_points": int(len(lanes)),
        "travelled_distance_m": float(np.sum(increments)),
        "closure_distance_m": float(np.linalg.norm(trace[-1, :2] - trace[0, :2])),
        "closure_heading_error_rad": float(abs(math.atan2(
            math.sin(trace[-1, 2] - trace[0, 2]), math.cos(trace[-1, 2] - trace[0, 2])))),
    }
    if np.max(increments) > cfg["max_trajectory_step_m"]:
        raise RouteBuildError("Trajectory has a localization jump or an unobserved step", diagnostics)
    if diagnostics["travelled_distance_m"] < cfg["min_lap_length_m"]:
        raise RouteBuildError("First lap is shorter than min_lap_length_m", diagnostics)
    if diagnostics["closure_distance_m"] > cfg["max_closure_distance_m"]:
        raise RouteBuildError("First lap did not return close enough to its start", diagnostics)
    if diagnostics["closure_heading_error_rad"] > cfg["max_closure_heading_rad"]:
        raise RouteBuildError("First lap did not return with a compatible heading", diagnostics)
    keep = np.concatenate(([True], increments > 1e-5))
    prior = trace[keep, :2]
    while len(prior) > 3 and np.linalg.norm(prior[-1] - prior[0]) < 1e-5:
        prior = prior[:-1]
    prior, _, prior_length = _resample_closed(prior, cfg["sample_spacing_m"], cfg["max_route_samples"])
    _check_intersections(prior, "First lap", diagnostics)
    normals = _normals(prior)
    lane_index = _LaneIndex(lanes, cfg["max_side_distance_m"])
    edges = _corridor(prior, normals, lane_index, cfg, diagnostics, "prior")
    clearance = cfg["vehicle_half_width_m"] + cfg["safety_margin_m"]
    lower = -edges[:, 1] + clearance
    upper = edges[:, 0] - clearance
    if np.any(lower > upper):
        raise RouteBuildError("Observed corridor cannot fit the vehicle with safety margin", diagnostics)
    offset = 0.5 * (edges[:, 0] - edges[:, 1])
    raw = prior + normals * offset[:, None]
    route = raw.copy()
    for _ in range(cfg["smoothing_iterations"]):
        smooth = (np.roll(route, 1, axis=0) + 2.0 * route + np.roll(route, -1, axis=0)) / 4.0
        target = 0.15 * raw + 0.85 * smooth
        lateral = np.sum((target - prior) * normals, axis=1)
        route = prior + normals * np.clip(lateral, lower, upper)[:, None]
    route, distance, length = _resample_closed(route, cfg["sample_spacing_m"], cfg["max_route_samples"])
    _check_intersections(route, "Smoothed route", diagnostics)
    final_normals = _normals(route)
    final_edges = _corridor(route, final_normals, lane_index, cfg, diagnostics, "final")
    if np.any(final_edges < clearance - 1e-6):
        raise RouteBuildError("Smoothed route violates the measured safety clearance", diagnostics)
    curvature = _curvature(route)
    maximum_curvature = float(np.max(np.abs(curvature)))
    diagnostics["max_curvature_1pm"] = maximum_curvature
    if maximum_curvature > cfg["max_curvature_1pm"]:
        raise RouteBuildError("Candidate route contains a turn sharper than the configured limit", diagnostics)
    speeds = _speed_profile(route, curvature, cfg)
    yaw = np.arctan2(-final_normals[:, 0], final_normals[:, 1])
    # s_m follows the actual polyline exported to the controller, not the pre-fit curve.
    segment_lengths = np.linalg.norm(np.roll(route, -1, axis=0) - route, axis=1)
    distance = np.concatenate(([0.0], np.cumsum(segment_lengths[:-1])))
    length = float(np.sum(segment_lengths))
    diagnostics.update({
        "valid": True,
        "route_length_m": length,
        "prior_length_m": float(prior_length),
        "route_sample_count": int(len(route)),
        "min_clearance_m": float(np.min(final_edges)),
        "min_speed_mps": float(np.min(speeds)),
        "max_speed_mps": float(np.max(speeds)),
        "requires_operator_review": True,
        "obstacle_safety_verified": False,
        "localization_accuracy_verified": False,
    })
    samples = [{
        "s_m": float(distance[index]),
        "x_m": float(point[0]),
        "y_m": float(point[1]),
        "yaw_rad": float(yaw[index]),
        "curvature_1pm": float(curvature[index]),
        "speed_mps": float(speeds[index]),
        "left_clearance_m": float(final_edges[index, 0]),
        "right_clearance_m": float(final_edges[index, 1]),
    } for index, point in enumerate(route)]
    return {"schema_version": 1, "closed": True, "route_samples": samples,
            "diagnostics": diagnostics, "config": cfg}
