"""Automatically align closed odometry and official loops without reflections.

This is a geometric similarity fit, not SLAM or a road-clearance check. Start
phase and travel direction are inferred only when competing fits are separated
well enough. Sparse official chords and odometry drift still require field
validation before using the result to steer a vehicle.
"""
from __future__ import annotations

import bisect
import math
from typing import List, Sequence, Tuple

from .rough_topology import _cumulative_lengths, _finite_points, _sample_closed


Point = Tuple[float, float]
_PHASE_SEPARATION = 0.125


def _number(name: str, value) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _closed(values: Sequence[Point]) -> Tuple[List[Point], float]:
    try:
        points = _finite_points(values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("A loop needs finite coordinate pairs") from exc
    if len(points) < 3 or len(set(points)) < 3:
        raise ValueError("A loop needs at least three distinct points")
    gap = math.dist(points[-1], points[0])
    if not math.isfinite(gap):
        raise ValueError("Loop closure distance must be finite")
    if points[-1] != points[0]:
        points.append(points[0])
    return points, gap


def _centre(points: Sequence[Point]) -> Tuple[Point, List[Point], float]:
    count = len(points)
    try:
        mean = (math.fsum(x / count for x, _ in points), math.fsum(y / count for _, y in points))
        centred = [(x - mean[0], y - mean[1]) for x, y in points]
        energy = math.fsum(x * x + y * y for x, y in centred)
    except (ValueError, OverflowError) as exc:
        raise ValueError("Loop coordinates exceed numeric alignment limits") from exc
    if not math.isfinite(energy) or energy <= 1e-12:
        raise ValueError("Loop is too small or too large for alignment")
    return mean, centred, energy


def _phase_distance(first: float, second: float) -> float:
    return abs((first - second + 0.5) % 1.0 - 0.5)


def fit_loop_alignment(
    recorded_points: Sequence[Point],
    official_points: Sequence[Point],
    sample_count: int = 256,
    max_rmse_m: float = 3.0,
    min_scale: float = 0.5,
    max_scale: float = 2.0,
    ambiguity_margin_m: float = 0.3,
) -> dict:
    """Fit ``official = scale * R * recorded + translation`` with det(R)=+1.

    Both inputs are treated as closed loops; a repeated endpoint is optional.
    ``phase_fraction`` locates the first recorded point along the official loop.
    Recorded progress ``f`` maps to ``(phase_fraction + direction * f) % 1``.
    Both travel directions and all sampled cyclic offsets are searched, followed
    by local continuous phase refinement. Same-direction phases less than 1/8 of
    a lap apart are neighbours, not independent ambiguity candidates. Opposite
    travel directions always compete. A circle therefore remains ambiguous.

    Invalid inputs/configuration raise ValueError. A finite fit which fails
    scale, residual or ambiguity checks returns available=False with its reason.
    Error distances are in official-map units, assumed metres. The returned
    transform fields are compatible with traffic_route.validated_alignment.
    """
    if isinstance(sample_count, bool) or not isinstance(sample_count, int) or not 16 <= sample_count <= 1024:
        raise ValueError("sample_count must be an integer between 16 and 1024")
    maximum_error = _number("max_rmse_m", max_rmse_m)
    minimum_scale = _number("min_scale", min_scale)
    maximum_scale = _number("max_scale", max_scale)
    ambiguity_margin = _number("ambiguity_margin_m", ambiguity_margin_m)
    if maximum_error < 0 or minimum_scale <= 0 or maximum_scale < minimum_scale or ambiguity_margin < 0:
        raise ValueError("Invalid alignment acceptance limits")
    recorded, closure_gap = _closed(recorded_points)
    official, _ = _closed(official_points)
    recorded_length = _cumulative_lengths(recorded)[-1]
    cumulative = _cumulative_lengths(official)
    official_length = cumulative[-1]
    if min(recorded_length, official_length) <= 1e-12:
        raise ValueError("Closed loops must have nonzero length")
    source = _sample_closed(recorded, sample_count)
    source_mean, source_centred, denominator = _centre(source)
    official_samples = _sample_closed(official, sample_count)
    _centre(official_samples)  # Reject numeric degeneracy before searching.

    def point_at(fraction: float) -> Point:
        distance = (fraction % 1.0) * official_length
        index = min(bisect.bisect_right(cumulative, distance) - 1, len(official) - 2)
        length = cumulative[index + 1] - cumulative[index]
        if length <= 1e-12:
            return official[index]
        ratio = (distance - cumulative[index]) / length
        first, second = official[index], official[index + 1]
        return (first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]))

    def fit(phase: float, direction: int, target=None) -> dict:
        if target is None:
            target = [point_at(phase + direction * index / sample_count) for index in range(sample_count)]
        target_mean, target_centred, _ = _centre(target)
        try:
            a = math.fsum(x * u + y * v for (x, y), (u, v) in zip(source_centred, target_centred)) / denominator
            b = math.fsum(x * v - y * u for (x, y), (u, v) in zip(source_centred, target_centred)) / denominator
            error = math.fsum((a * x - b * y - u) ** 2 + (b * x + a * y - v) ** 2
                              for (x, y), (u, v) in zip(source_centred, target_centred))
            result = {
                "scale": math.hypot(a, b),
                "rotation_rad": math.atan2(b, a),
                "translation_x": target_mean[0] - a * source_mean[0] + b * source_mean[1],
                "translation_y": target_mean[1] - b * source_mean[0] - a * source_mean[1],
                "rmse_m": math.sqrt(error / sample_count),
                "phase_fraction": phase % 1.0,
                "direction": direction,
            }
        except (ValueError, OverflowError) as exc:
            raise ValueError("Alignment arithmetic exceeded finite limits") from exc
        if not all(math.isfinite(value) for value in result.values()):
            raise ValueError("Alignment produced non-finite values")
        return result

    def refine(candidate: dict, lower: float, upper: float) -> dict:
        # Golden-section minimisation stays within one phase-grid neighbourhood.
        ratio = (math.sqrt(5.0) - 1.0) / 2.0
        left = upper - ratio * (upper - lower)
        right = lower + ratio * (upper - lower)
        first, second = fit(left, candidate["direction"]), fit(right, candidate["direction"])
        best = candidate
        for _ in range(24):
            best = min((best, first, second), key=lambda value: value["rmse_m"])
            if first["rmse_m"] < second["rmse_m"]:
                upper, right, second = right, left, first
                left = upper - ratio * (upper - lower)
                first = fit(left, candidate["direction"])
            else:
                lower, left, first = left, right, second
                right = lower + ratio * (upper - lower)
                second = fit(right, candidate["direction"])
        return min((best, first, second), key=lambda value: value["rmse_m"])

    candidates = []
    step = 1.0 / sample_count
    for direction in (1, -1):
        grid = [fit(index * step, direction,
                    [official_samples[(index + direction * j) % sample_count] for j in range(sample_count)])
                for index in range(sample_count)]
        candidates.extend(grid)
        for index, candidate in enumerate(grid):
            before, after = grid[index - 1]["rmse_m"], grid[(index + 1) % sample_count]["rmse_m"]
            error = candidate["rmse_m"]
            if error <= min(before, after) and error + 1e-10 < max(before, after):
                phase = candidate["phase_fraction"]
                candidates.append(refine(candidate, phase - step, phase + step))

    def alternative_to(best: dict) -> dict:
        return min((candidate for candidate in candidates
                    if candidate["direction"] != best["direction"] or
                    _phase_distance(candidate["phase_fraction"], best["phase_fraction"]) >= _PHASE_SEPARATION - 1e-12),
                   key=lambda value: value["rmse_m"])

    best = min(candidates, key=lambda value: value["rmse_m"])
    # Also refine a constrained ambiguity candidate at the exclusion boundary.
    for _ in range(2):
        alternative = alternative_to(best)
        phase = alternative["phase_fraction"]
        lower, upper = phase - step, phase + step
        if alternative["direction"] == best["direction"]:
            delta = (phase - best["phase_fraction"] + 0.5) % 1.0 - 0.5
            centre = best["phase_fraction"] + delta
            lower, upper = centre - step, centre + step
            if delta >= 0:
                lower = max(lower, best["phase_fraction"] + _PHASE_SEPARATION)
            else:
                upper = min(upper, best["phase_fraction"] - _PHASE_SEPARATION)
        candidates.append(refine(alternative, lower, upper))
        best = min(candidates, key=lambda value: value["rmse_m"])
    alternative = alternative_to(best)
    gap = max(0.0, alternative["rmse_m"] - best["rmse_m"])
    reason = "accepted"
    if not minimum_scale <= best["scale"] <= maximum_scale:
        reason = "scale_out_of_range"
    elif best["rmse_m"] > maximum_error:
        reason = "rmse_exceeds_limit"
    elif gap <= ambiguity_margin + 1e-12:
        reason = "ambiguous_correspondence"
    return {
        **best,
        "available": reason == "accepted",
        "reason": reason,
        "alternative_rmse_m": alternative["rmse_m"],
        "alternative_phase_fraction": alternative["phase_fraction"],
        "alternative_direction": alternative["direction"],
        "ambiguity_gap_m": gap,
        "ambiguity_phase_separation_fraction": _PHASE_SEPARATION,
        "closure_gap_m": closure_gap,
        "recorded_closed_length_m": recorded_length,
        "official_chord_length_m": official_length,
        "sample_count": sample_count,
        "transform_direction": "recorded_odom_to_official_xy",
        "automatic_estimate": True,
        "quality_gated": True,
        "field_validation_required": True,
        "assumption": "corresponding closed loops with similar normalized arc-length progress; no odometry drift correction",
    }


def official_fraction_from_recorded(alignment: dict, recorded_fraction: float) -> float:
    """Convert recorded lap progress using an accepted phase/direction fit."""
    if not isinstance(alignment, dict) or alignment.get("available") is not True:
        raise ValueError("An accepted loop alignment is required")
    if alignment.get("transform_direction") != "recorded_odom_to_official_xy":
        raise ValueError("Unsupported alignment transform direction")
    direction = alignment.get("direction")
    if isinstance(direction, bool) or direction not in (-1, 1):
        raise ValueError("Alignment direction must be +1 or -1")
    phase = _number("phase_fraction", alignment.get("phase_fraction"))
    if not 0.0 <= phase < 1.0:
        raise ValueError("Alignment phase must be within [0, 1)")
    fraction = _number("recorded_fraction", recorded_fraction)
    return (phase + direction * fraction) % 1.0
