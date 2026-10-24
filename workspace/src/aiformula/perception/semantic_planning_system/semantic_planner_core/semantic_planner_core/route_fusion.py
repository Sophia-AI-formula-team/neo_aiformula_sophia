"""Fuse a measured lap with sparse outer-loop shape, in odometry metres.

The official map supplies ordered turn features and edge directions. It never
supplies the size of the driven route: measured node spacing and local curves
remain the metric reference. An ambiguous fit or an unobserved corner produces
an explicitly unavailable odom-only drawing, not a usable fused navigation map.
"""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .rough_route import RouteReference, RouteSample
from .rough_topology import OfficialRoute
from .topology_alignment import fit_loop_alignment


Point = Tuple[float, float]


@dataclass
class FusionResult:
    reference: RouteReference
    alignment: Optional[Dict]
    status: dict
    node_positions: Dict[str, Point]
    node_progress: Dict[str, float]


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _turn(before: Point, centre: Point, after: Point) -> float:
    return _wrap(math.atan2(after[1] - centre[1], after[0] - centre[0])
                 - math.atan2(centre[1] - before[1], centre[0] - before[0]))


def _fit_trace(reference: RouteReference, max_gap_m: float) -> Tuple[List[Point], Optional[str]]:
    """Only short, bounded gaps may be bridged for initial shape matching.

    Real excluded observations never steer the fit. The returned bridge is
    used for correspondence only; the reference's trust mask remains intact.
    """
    points = list(reference.points)
    trusted = reference.trusted
    if all(trusted):
        return points, None
    if not any(trusted):
        return points, "no_trusted_recording"
    size = len(points)
    anchor = trusted.index(True)
    offset, total = 1, 0.0
    while offset < size:
        if trusted[(anchor + offset) % size]:
            offset += 1
            continue
        left = offset - 1
        while offset < size and not trusted[(anchor + offset) % size]:
            offset += 1
        indices = [(anchor + value) % size for value in range(left, offset + 1)]
        distances = [0.0]
        for previous, current in zip(indices, indices[1:]):
            distances.append(distances[-1] + math.dist(points[previous], points[current]))
        total += distances[-1]
        # Resampling marks either endpoint of an excluded segment untrusted.
        # Two sample spacings account only for this conservative mask growth.
        if distances[-1] > max_gap_m + 2.0 * reference.spacing_m + 1e-8:
            return points, "excluded_gap_too_long"
        start, end = points[indices[0]], points[indices[-1]]
        for index, distance in zip(indices[1:-1], distances[1:-1]):
            fraction = distance / distances[-1]
            points[index] = (start[0] + fraction * (end[0] - start[0]),
                             start[1] + fraction * (end[1] - start[1]))
    if total > 0.15 * reference.length_m:
        return points, "insufficient_trusted_coverage"
    return points, None


def _node_anchors(reference: RouteReference, official: OfficialRoute, alignment: dict,
                  search_m: float, window_m: float) -> Tuple[Dict[str, float], Optional[str]]:
    """Refine phase seeds to measured signed turns, not official edge lengths."""
    count = len(official.node_ids) - 1
    direction, phase = alignment["direction"], alignment["phase_fraction"]
    result = {}
    # A seed may drift within a segment when individual official lengths differ.
    # The finite search neighbourhood prevents jumps to a distant similar bend.
    search = min(search_m, reference.length_m * 0.10)
    window = min(window_m, reference.length_m / (4.0 * count))
    step = min(reference.spacing_m / 2.0, max(window / 12.0, 0.02))
    for index, identifier in enumerate(official.node_ids[:-1]):
        expected = direction * _turn(official.points[(index - 1) % count],
                                     official.points[index], official.points[(index + 1) % count])
        # A node on a straight is not identifiable from this sensor combination.
        if abs(expected) < math.radians(8.0):
            return {}, "node_has_no_observable_turn"
        fraction = official._cumulative_m[index] / official.chord_length_m
        seed = (direction * (fraction - phase)) % 1.0 * reference.length_m
        candidates = []
        offsets = max(1, int(math.ceil(search / step)))
        for offset in range(-offsets, offsets + 1):
            distance = seed + search * offset / offsets
            if not reference._interval_trusted(distance - window, distance + window):
                continue
            before = reference.point_at(distance - window)[:2]
            centre = reference.point_at(distance)[:2]
            after = reference.point_at(distance + window)[:2]
            if min(math.dist(before, centre), math.dist(centre, after)) < window * 0.2:
                continue
            observed = _turn(before, centre, after)
            if observed * expected <= 0.0 or abs(observed) < max(0.10, abs(expected) * 0.25):
                continue
            # Corner strength matters most. A weak phase preference breaks ties
            # on rounded features without treating normalized distance as truth.
            cost = (observed - expected) ** 2 + 0.015 * ((distance - seed) / max(search, step)) ** 2
            candidates.append((cost, distance % reference.length_m, observed))
        if not candidates:
            return {}, "node_turn_unobserved_or_excluded"
        _, progress, observed = min(candidates)
        if abs(observed - expected) > math.radians(60.0):
            return {}, "node_turn_inconsistent"
        result[identifier] = progress

    # All turns must appear exactly once in official travel order. In
    # particular, two nodes cannot collapse to the same detected corner.
    ids = official.node_ids[:-1]
    ordered = ids if direction == 1 else [ids[0]] + list(reversed(ids[1:]))
    gaps = [(result[ordered[(i + 1) % count]] - result[identifier]) % reference.length_m
            for i, identifier in enumerate(ordered)]
    if (abs(sum(gaps) - reference.length_m) > reference.spacing_m
            or min(gaps) < max(reference.spacing_m * 2.0, window)):
        return {}, "node_order_or_separation_invalid"
    return result, None


def _shape_displacements(reference: RouteReference, official: OfficialRoute, alignment: dict,
                         anchors: Dict[str, float], weight: float, maximum: float) -> Dict[str, Point]:
    """Small least-squares direction corrections with measured length anchors.

    Official edge lengths are never used. The desired edge vector has the
    measured chord length. A stronger longitudinal constraint keeps each
    measured segment's size; positional anchors prevent translation or drift.
    """
    ids = sorted(anchors, key=anchors.get)
    count = len(ids)
    measured = np.asarray([reference.point_at(anchors[identifier])[:2] for identifier in ids])
    official_by_id = dict(zip(official.node_ids[:-1], official.points[:-1]))
    rows, targets = [], []
    for index in range(count * 2):
        row = np.zeros(count * 2)
        row[index] = 1.0
        rows.append(row)
        targets.append(0.0)
    # A similarity fit over unequal normalized segment lengths can invent a
    # small rotation. Re-estimate orientation from the measured node-to-node
    # *unit* directions after turn matching, so official lengths have no vote.
    direction_pairs = []
    for index, identifier in enumerate(ids):
        following = (index + 1) % count
        measured_edge = measured[following] - measured[index]
        official_edge = np.asarray(official_by_id[ids[following]]) - np.asarray(official_by_id[identifier])
        length = float(np.linalg.norm(measured_edge))
        if not math.isfinite(length) or length <= max(1e-6, reference.spacing_m):
            raise ValueError("degenerate_measured_node_chord")
        direction_pairs.append((measured_edge / length, official_edge / np.linalg.norm(official_edge)))
    a = sum(float(np.dot(first, second)) for first, second in direction_pairs)
    b = sum(float(first[0] * second[1] - first[1] * second[0]) for first, second in direction_pairs)
    if math.hypot(a, b) < 0.5 * count:
        raise ValueError("inconsistent_measured_edge_directions")
    node_rotation = math.atan2(b, a)
    alignment["node_rotation_rad"] = node_rotation
    c, s = math.cos(node_rotation), math.sin(node_rotation)
    strength = math.sqrt(weight / max(1.0 - weight, 1e-9))
    for index, identifier in enumerate(ids):
        following = (index + 1) % count
        ox, oy = official_by_id[identifier]
        nx, ny = official_by_id[ids[following]]
        norm = math.hypot(nx - ox, ny - oy)
        desired_direction = np.asarray([(c * (nx - ox) + s * (ny - oy)) / norm,
                                        (-s * (nx - ox) + c * (ny - oy)) / norm])
        measured_edge = measured[following] - measured[index]
        length = float(np.linalg.norm(measured_edge))
        residual = length * desired_direction - measured_edge
        for axis in range(2):
            row = np.zeros(count * 2)
            row[following * 2 + axis] = strength
            row[index * 2 + axis] = -strength
            rows.append(row)
            targets.append(strength * float(residual[axis]))
        # Stronger than the shape term: approximately preserve measured chord
        # length, while allowing a small lateral correction of the outline.
        row = np.zeros(count * 2)
        measured_direction = measured_edge / length
        row[following * 2:following * 2 + 2] = strength * 4.0 * measured_direction
        row[index * 2:index * 2 + 2] = -strength * 4.0 * measured_direction
        rows.append(row)
        targets.append(0.0)
    delta = np.linalg.lstsq(np.asarray(rows), np.asarray(targets), rcond=None)[0].reshape((-1, 2))
    if not np.isfinite(delta).all():
        raise ValueError("nonfinite_shape_solution")
    largest = max(float(np.linalg.norm(value)) for value in delta)
    if largest > maximum:
        delta *= maximum / largest
    return {identifier: (float(value[0]), float(value[1])) for identifier, value in zip(ids, delta)}


def build_fused_route(
    samples, official: OfficialRoute, *, spacing_m: float = 0.2,
    smooth_window_m: float = 1.0, max_detour_gap_m: float = 2.0,
    max_rmse_m: float = 3.0, ambiguity_margin_m: float = 0.3,
    node_search_m: float = 4.0, feature_window_m: float = 2.0,
    shape_weight: float = 0.25, max_displacement_m: float = 0.75,
) -> FusionResult:
    """Match turns, fuse the outer shape, then smooth in the odometry frame.

    Uniform official-map scale/translation cannot change the result. Different
    official segment lengths only affect initial correspondence seeds; each
    accepted node is relocated using a local observed-turn search. Arbitrarily
    wrong topology or large local scale errors are rejected rather than guessed.

    ``alignment`` retains the original odom-to-official similarity transform,
    but its quality distances are in normalized odometry metres. Consumers must
    use ``node_positions``/``node_progress`` for final junctions, not global lap
    fractions or the similarity transform's original node locations.
    """
    for name, value in (("node_search_m", node_search_m), ("feature_window_m", feature_window_m),
                        ("max_displacement_m", max_displacement_m), ("shape_weight", shape_weight),
                        ("max_rmse_m", max_rmse_m), ("ambiguity_margin_m", ambiguity_margin_m)):
        if isinstance(value, bool) or not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and nonnegative")
    if min(node_search_m, feature_window_m) <= 0.0 or shape_weight >= 1.0:
        raise ValueError("Positive feature windows and shape_weight < 1 are required")
    if len(set(official.node_ids[:-1])) != len(official.node_ids) - 1:
        raise ValueError("Fusion requires an outer loop without repeated interior nodes")
    samples = list(samples)
    # Preparation has no final smoothing. It removes repeats, bridges only
    # permitted short gaps and creates a uniform turn-feature sampling grid.
    prepared = RouteReference(samples, spacing_m, 0.0, max_detour_gap_m)
    base_status = {"available": False, "mode": "odom_only", "metric_source": "odometry",
                   "recorded_length_m": prepared.length_m,
                   "official_size_used": False, "field_validation_required": True}

    def fallback(reason, alignment=None):
        reference = RouteReference(samples, spacing_m, smooth_window_m, max_detour_gap_m)
        return FusionResult(reference, alignment,
                            {**base_status, "reason": reason, "final_length_m": reference.length_m}, {}, {})

    fit_points, reason = _fit_trace(prepared, max_detour_gap_m)
    if reason:
        return fallback(reason)
    origin = official.points[0]
    normalization = prepared.length_m / official.chord_length_m
    normalized = [((x - origin[0]) * normalization, (y - origin[1]) * normalization)
                  for x, y in official.points]
    try:
        normalized_fit = fit_loop_alignment(fit_points, normalized, sample_count=128,
                                            max_rmse_m=max_rmse_m, min_scale=0.1, max_scale=10.0,
                                            ambiguity_margin_m=ambiguity_margin_m)
    except (ValueError, OverflowError) as exc:
        return fallback("alignment_failed: " + str(exc))
    alignment = {**normalized_fit,
                 "scale": normalized_fit["scale"] / normalization,
                 "translation_x": normalized_fit["translation_x"] / normalization + origin[0],
                 "translation_y": normalized_fit["translation_y"] / normalization + origin[1],
                 "official_chord_length_m": official.chord_length_m,
                 "normalization_official_to_odom": normalization,
                 "error_units": "odom_metres",
                 "assumption": "similar outer shape; turn features refine local node correspondence"}
    if not alignment["available"]:
        return fallback(alignment["reason"], alignment)
    anchors, reason = _node_anchors(prepared, official, alignment, node_search_m, feature_window_m)
    if reason:
        return fallback(reason, alignment)
    try:
        displacements = _shape_displacements(prepared, official, alignment, anchors,
                                             shape_weight, max_displacement_m)
    except (ValueError, np.linalg.LinAlgError) as exc:
        return fallback("shape_fusion_failed: " + str(exc), alignment)
    ordered = sorted(anchors, key=anchors.get)
    progress = [anchors[identifier] for identifier in ordered]

    def displacement_at(distance):
        left_index = (bisect_right(progress, distance) - 1) % len(ordered)
        right_index = (left_index + 1) % len(ordered)
        left, right = progress[left_index], progress[right_index]
        span = (right - left) % prepared.length_m
        fraction = ((distance - left) % prepared.length_m) / span
        # Zero derivative at observed nodes avoids adding small heading kinks.
        fraction = fraction * fraction * (3.0 - 2.0 * fraction)
        first, second = displacements[ordered[left_index]], displacements[ordered[right_index]]
        return (first[0] + fraction * (second[0] - first[0]),
                first[1] + fraction * (second[1] - first[1]))

    deformed = []
    for index, (x, y) in enumerate(prepared.points):
        dx, dy = displacement_at(prepared.cumulative_m[index])
        deformed.append(RouteSample(x + dx, y + dy, 0.0, excluded=not prepared.trusted[index]))
    # This is the sole final smoothing, after shape fusion. Do not straighten
    # exclusions again: they have already been processed, and stay untrusted.
    reference = RouteReference(deformed, spacing_m, smooth_window_m, 0.0)
    deformed_cumulative = RouteReference._cumulative([(sample.x, sample.y) for sample in deformed])
    node_positions, node_progress = {}, {}
    for identifier, distance in anchors.items():
        edge = min(bisect_right(prepared.cumulative_m, distance) - 1, len(prepared.points) - 1)
        fraction = ((distance - prepared.cumulative_m[edge])
                    / (prepared.cumulative_m[edge + 1] - prepared.cumulative_m[edge]))
        before_smoothing_s = deformed_cumulative[edge] + fraction * (deformed_cumulative[edge + 1] - deformed_cumulative[edge])
        # RouteReference samples raw arc length uniformly, then smooths those
        # points without reordering them. Map through their preserved indices
        # to account for every earlier shortening, including at the lap seam.
        source_step = deformed_cumulative[-1] / len(reference.points)
        output_index = min(int(before_smoothing_s / source_step), len(reference.points) - 1)
        output_fraction = (before_smoothing_s - output_index * source_step) / source_step
        expected_s = (reference.cumulative_m[output_index] + output_fraction
                      * (reference.cumulative_m[output_index + 1] - reference.cumulative_m[output_index]))
        x, y = prepared.point_at(distance)[:2]
        dx, dy = displacements[identifier]
        # Project the node onto the actual final curve in a bounded local
        # neighbourhood; smoothing changes lengths, so the old fraction is not
        # the node's final progress. No global nearest-node jump is permitted.
        position = ordered.index(identifier)
        before_gap = (distance - progress[position - 1]) % prepared.length_m
        after_gap = (progress[(position + 1) % len(progress)] - distance) % prepared.length_m
        local_window = min(smooth_window_m + 2.0 * spacing_m, 0.25 * min(before_gap, after_gap))
        matched = reference.match(x + dx, y + dy, expected_s, 0.0, local_window,
                                  max_displacement_m + smooth_window_m + 2.0 * spacing_m)
        if matched is None or not reference.point_at(matched)[2]:
            return fallback("final_node_projection_unavailable", alignment)
        node_progress[identifier] = matched % reference.length_m
        node_positions[identifier] = reference.point_at(matched)[:2]
    final_ids = sorted(node_progress, key=node_progress.get)
    # The seam may move slightly, so compare cyclic order rather than first ID.
    if any(final_ids[(final_ids.index(identifier) + 1) % len(final_ids)] != ordered[(i + 1) % len(ordered)]
           for i, identifier in enumerate(ordered)):
        return fallback("final_node_order_invalid", alignment)
    maximum = max(math.hypot(*value) for value in displacements.values())
    status = {**base_status, "available": True, "mode": "fused", "reason": "accepted",
              "final_length_m": reference.length_m, "node_count": len(node_positions),
              "max_node_displacement_m": maximum, "shape_weight": shape_weight,
              "direction": alignment["direction"],
              "node_source": "observed_turns_then_projection_onto_final_reference"}
    return FusionResult(reference, alignment, status, node_positions, node_progress)
