from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class PlannerConfig:
    vehicle_width_m: float
    clearance_m: float = 0.08
    lookahead_m: float = 4.5
    min_lookahead_m: float = 2.0
    max_lane_gap_m: float = 2.1
    offset_step_m: float = 0.15
    max_offset_m: float = 1.05
    max_curvature_inv_m: float = 2.0
    length_weight: float = 1.0
    smoothness_weight: float = 0.25
    reference_weight: float = 0.10

    def validate(self):
        if not all(np.isfinite(value) for value in vars(self).values()):
            raise ValueError("nonfinite_config")
        if self.vehicle_width_m <= 0:
            raise ValueError("vehicle_width_required")
        if (self.vehicle_width_m > 3 or not 0 <= self.clearance_m <= 1
                or not 0.5 <= self.min_lookahead_m <= self.lookahead_m <= 12
                or not 0 <= self.max_lane_gap_m <= 4
                or not 0.05 <= self.offset_step_m <= self.max_offset_m <= 3
                or not 0 < self.max_curvature_inv_m <= 10
                or min(self.length_weight, self.smoothness_weight, self.reference_weight) < 0
                or self.length_weight + self.smoothness_weight <= 0):
            raise ValueError("invalid_planner_config")


@dataclass
class PlanResult:
    reason: str
    path: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    reference_uv: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    center_uv: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    allowed: np.ndarray = field(default_factory=lambda: np.empty((0, 0), dtype=bool))
    obstacles: np.ndarray = field(default_factory=lambda: np.empty((0, 0), dtype=bool))
    band: np.ndarray = field(default_factory=lambda: np.empty((0, 0), dtype=np.uint8))
    metrics: dict = field(default_factory=dict)


def _ordered_points(points):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    points = points[np.all(np.isfinite(points), axis=1) & (points[:, 0] >= 0)]
    points = points[np.argsort(points[:, 0], kind="stable")]
    _, unique = np.unique(points[:, 0], return_index=True)
    return points[unique]


def _reference_y(center, x):
    # Connect the observed centerline to the current vehicle origin and heading.
    knots = np.vstack(((0.0, 0.0), center[center[:, 0] > 1e-6]))
    slopes = np.gradient(knots[:, 1], knots[:, 0])
    slopes[0] = 0.0
    segment = np.clip(np.searchsorted(knots[:, 0], x, side="right") - 1, 0, len(knots) - 2)
    distance = knots[segment + 1, 0] - knots[segment, 0]
    t = (x - knots[segment, 0]) / distance
    return ((2*t**3 - 3*t**2 + 1) * knots[segment, 1]
            + (t**3 - 2*t**2 + t) * distance * slopes[segment]
            + (-2*t**3 + 3*t**2) * knots[segment + 1, 1]
            + (t**3 - t**2) * distance * slopes[segment + 1])


def _project_lanes(center, left, right, horizon, camera):
    x = np.linspace(0.0, horizon, int(np.ceil(horizon / .025)) + 1)
    lane_y = (_reference_y(center, x),
              np.interp(x, left[:, 0], left[:, 1]),
              np.interp(x, right[:, 0], right[:, 1]))
    lines = []
    for y in lane_y:
        uv, depth = camera.ground_to_image(np.column_stack((x, y)))
        visible = (depth > 0) & np.all(np.isfinite(uv), axis=1)
        image_line = uv[visible][::-1]
        if len(image_line) < 2 or np.any(np.diff(image_line[:, 1]) <= 0):
            raise ValueError("lane_projection_not_monotonic")
        lines.append(image_line)
    first = max(int(np.ceil(camera.height * camera.roi_top_ratio)),
                int(np.ceil(max(line[0, 1] for line in lines))))
    last = min(camera.height - 1, int(np.floor(min(line[-1, 1] for line in lines))))
    rows = np.arange(last, first - 1, -1)
    return rows, [np.interp(rows, line[:, 1], line[:, 0]) for line in lines]


def _smooth_step(progress, start, end):
    t = np.clip((progress - start) / (end - start), 0.0, 1.0)
    return t**3 * (10.0 - 15.0*t + 6.0*t**2)


def _candidate_centers(reference, station, scale, config):
    yield reference
    progress = (station - station[0]) / (station[-1] - station[0])
    offsets = int(np.floor(config.max_offset_m / config.offset_step_m + 1e-9))
    for step in range(1, offsets + 1):
        offset_px = step * config.offset_step_m / scale
        for start, end in ((0., .32), (0., .48), (0., .68), (.17, .57), (.35, .75)):
            departure = _smooth_step(progress, start, end)
            for sign in (-1, 1):
                yield reference + sign * offset_px * departure
                if end < .65:
                    rejoin = _smooth_step(progress, max(.63, end + .1), 1.)
                    yield reference + sign * offset_px * (departure - rejoin)


def _shape(candidate, station, scale, sample, image_center):
    # Use pixel-to-metre scales for the score while checking occupancy on native pixels.
    lateral = (candidate - image_center) * scale
    uniform = np.interp(sample, station, lateral)
    slope = np.gradient(uniform, sample, edge_order=2)
    curvature = np.gradient(slope, sample, edge_order=2) / (1.0 + slope**2)**1.5
    lengths = np.hypot(np.diff(sample), np.diff(uniform))
    smoothness = np.sum((curvature[:-1]**2 + curvature[1:]**2) * lengths / 2)
    return (np.interp(station, sample, slope), float(np.max(np.abs(curvature))),
            float(np.sum(lengths)), float(smoothness))


def plan(center, left, right, obstacle_mask, config, camera):
    config.validate()
    shape = (camera.height, camera.width)
    obstacle_mask = np.asarray(obstacle_mask, dtype=bool)
    if obstacle_mask.shape != shape:
        raise ValueError("camera_and_mask_size_mismatch")
    allowed = np.zeros(shape, dtype=bool)
    result = PlanResult("no_feasible_candidate", allowed=allowed, obstacles=obstacle_mask,
                        band=np.zeros(shape, dtype=np.uint8))
    center, left, right = (_ordered_points(points) for points in (center, left, right))
    if np.count_nonzero(center[:, 0] > 1e-6) < 2:
        result.reason = "missing_centerline"
        return result
    if len(left) < 2 or len(right) < 2:
        result.reason = "missing_lane_boundary"
        return result
    near_gap = float(max(center[0, 0], left[0, 0], right[0, 0]))
    result.metrics["near_lane_gap_m"] = near_gap
    if near_gap > config.max_lane_gap_m:
        result.reason = "lane_points_start_too_far"
        return result
    horizon = min(config.lookahead_m, center[-1, 0], left[-1, 0], right[-1, 0])
    if horizon < config.min_lookahead_m:
        result.reason = "insufficient_lane_horizon"
        return result
    rows, (reference, lane_left, lane_right) = _project_lanes(center, left, right, horizon, camera)
    if len(rows) < 3:
        result.reason = "insufficient_image_rows"
        return result
    if np.any(lane_left >= lane_right):
        result.reason = "crossed_lane_boundaries"
        return result
    lo = np.maximum(0, np.ceil(lane_left)).astype(int)
    hi = np.minimum(camera.width - 1, np.floor(lane_right)).astype(int)
    columns = np.arange(camera.width)
    allowed[rows] = (columns >= lo[:, None]) & (columns <= hi[:, None])
    result.reference_uv = np.column_stack((reference, rows))
    result.metrics.update(image_rows=len(rows), far_row=int(rows[-1]), near_row=int(rows[0]),
                          obstacle_pixels=int(np.count_nonzero(obstacle_mask[rows])),
                          rejected={}, candidates_checked=0)
    station, scale, min_scale = camera.image_row_calibration(rows)
    if np.any(~np.isfinite(station)) or np.any(np.diff(station) <= 0) or np.any(min_scale <= 0):
        raise ValueError("invalid_image_row_calibration")
    sample = np.linspace(station[0], station[-1], max(3, int(np.ceil(np.ptp(station) / .08)) + 1))
    prefix = np.pad(np.cumsum(obstacle_mask[rows], axis=1), ((0, 0), (1, 0)))
    row_index = np.arange(len(rows))
    half_width = (config.vehicle_width_m / 2 + config.clearance_m) / min_scale
    best = None
    best_score = float("inf")
    for index, candidate in enumerate(_candidate_centers(reference, station, scale, config)):
        result.metrics["candidates_checked"] += 1
        slope, curve, length, smoothness = _shape(candidate, station, scale, sample,
                                                  (camera.width - 1) / 2)
        # A slanted strip of fixed transverse width occupies more columns on an image row.
        reach = half_width * np.hypot(1.0, slope)
        start = np.floor(candidate - reach).astype(int)
        end = np.ceil(candidate + reach).astype(int)
        if curve > config.max_curvature_inv_m:
            rejection = "curvature_limit"
        elif np.any(start < lo) or np.any(end > hi):
            rejection = "outside_allowed_image"
        elif np.any(start[1:] > end[:-1]) or np.any(start[:-1] > end[1:]):
            rejection = "disconnected_band"
        elif np.any(prefix[row_index, end + 1] != prefix[row_index, start]):
            rejection = "obstacle_pixels"
        else:
            rejection = ""
        if rejection:
            counts = result.metrics["rejected"]
            counts[rejection] = counts.get(rejection, 0) + 1
            continue
        deviation = float(np.mean(np.abs(candidate - reference) * scale))
        score = (config.length_weight * length + config.smoothness_weight * smoothness
                 + config.reference_weight * deviation)
        if score < best_score:
            best_score = score
            best = (candidate, start, end)
            result.reason = "keep_reference" if index == 0 else "detour"
            result.metrics.update(scored_length_m=length, smoothness=smoothness, score=score,
                                  mean_reference_deviation_m=deviation, max_curvature_inv_m=curve)
        if index == 0:
            break
    if best is None:
        return result

    candidate, start, end = best
    result.center_uv = np.column_stack((candidate, rows))
    # Convert only the selected centerline, after evaluating the full image-space band.
    ground = camera.image_to_ground(result.center_uv)
    if np.any(~np.isfinite(ground)) or np.any(np.diff(ground[:, 0]) <= 0):
        raise ValueError("selected_centerline_not_monotonic")
    tangents = np.gradient(ground, axis=0)
    yaw = np.arctan2(tangents[:, 1], tangents[:, 0])
    result.path = np.column_stack((ground, yaw))
    result.band[rows] = 255 * ((columns >= start[:, None]) & (columns <= end[:, None]))
    result.metrics.update(length_m=float(np.sum(np.linalg.norm(np.diff(ground, axis=0), axis=1))),
                          band_pixels=int(np.count_nonzero(result.band)),
                          near_band_width_px=int(end[0] - start[0] + 1),
                          far_band_width_px=int(end[-1] - start[-1] + 1))
    return result


def render_overlay(result, background=None):
    shape = (*result.band.shape, 3)
    if background is None or background.shape != shape:
        image = np.full(shape, 35, dtype=np.uint8)
    else:
        image = background.copy()
    for area, color, opacity in (
        (result.allowed, (64, 134, 220), .17),
        (result.obstacles, (249, 90, 54), .53),
        (result.band > 0, (25, 219, 170), .55),
    ):
        image[area] = (1.0 - opacity) * image[area] + opacity * np.asarray(color)
    for line, rgb, dashed in (
        (result.reference_uv, (113, 169, 255), True),
        (result.center_uv, (224, 255, 226), False),
    ):
        for i, (u, v) in enumerate(np.rint(line).astype(int)):
            if 0 <= u < image.shape[1] and (not dashed or i % 10 < 6):
                image[v, max(0, u - 1):u + 2] = rgb
    return image
