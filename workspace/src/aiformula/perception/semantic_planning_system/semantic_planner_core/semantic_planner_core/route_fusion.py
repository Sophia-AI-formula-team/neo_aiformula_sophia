"""Fuse a measured odometry lap with sparse official outer-node geometry.

The official closed way is an ordered constellation of outer junctions.
Node-to-node chords and relative angles constrain alignment; they are not the
driven lane centreline or equal arc-length samples. The measured odometry route
remains the metric navigation reference.
"""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .rough_route import RouteReference, RouteSample
from .rough_topology import OfficialRoute


Point = Tuple[float, float]
_MIN_MAP_TO_ODOM_SCALE = 0.05
_MAX_MAP_TO_ODOM_SCALE = 20.0


@dataclass
class FusionResult:
    reference: RouteReference
    alignment: Optional[Dict]
    status: dict
    node_positions: Dict[str, Point]
    node_progress: Dict[str, float]


def build_odom_reference(samples, spacing_m: float = 0.2,
                         smooth_window_m: float = 1.0) -> RouteReference:
    """Build the lap reference with all excluded gaps straight and untrusted."""
    samples = list(samples)
    if len(samples) < 3:
        raise ValueError("A closed reference needs at least three samples")
    # A finite maximum larger than the entire recorded lap makes the existing
    # gap straightener cover every excluded run without changing trust labels.
    coordinates = [(sample.x, sample.y) for sample in samples]
    if not all(math.isfinite(value) for point in coordinates for value in point):
        raise ValueError("Route samples must be finite")
    # The final segment is an interpolation back to the first sample. When
    # that gap is larger than ordinary sampling, it has not been observed and
    # must not be used for steering or as a trusted topo correspondence.
    if math.dist(coordinates[0], coordinates[-1]) > max(2.0 * spacing_m, 1e-6):
        last = samples[-1]
        samples[-1] = RouteSample(last.x, last.y, last.yaw, excluded=last.excluded, closure=True)
    total = math.fsum(math.dist(first, second) for first, second in
                      zip(coordinates, coordinates[1:] + coordinates[:1]))
    if not math.isfinite(total):
        raise ValueError("Recorded route length must be finite")
    return RouteReference(samples, spacing_m, smooth_window_m,
                          max(total + spacing_m, spacing_m))


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _signed_area(points: np.ndarray) -> float:
    following = np.roll(points, -1, axis=0)
    return 0.5 * float(np.sum(points[:, 0] * following[:, 1]
                              - following[:, 0] * points[:, 1]))


def _fit_trace(reference: RouteReference, max_gap_m: float,
               max_closure_gap_m: float = 4.0) -> Tuple[List[Point], Optional[str]]:
    """Reject excessive excluded coverage before matching the outer nodes."""
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
        # An exclusion touching OA/invalid data is never treated as a pure
        # closure seam, even when the two runs meet at the lap boundary.
        pure_seam = (any(reference.closure_mask[i] for i in indices)
                     and not any(reference.excluded_mask[i] for i in indices))
        limit = max_closure_gap_m if pure_seam else max_gap_m
        if distances[-1] > limit + 2.0 * reference.spacing_m + 1e-8:
            return points, "excluded_gap_too_long"
        start, end = points[indices[0]], points[indices[-1]]
        for index, distance in zip(indices[1:-1], distances[1:-1]):
            fraction = distance / distances[-1]
            points[index] = (start[0] + fraction * (end[0] - start[0]),
                             start[1] + fraction * (end[1] - start[1]))
    if total > 0.15 * reference.length_m:
        return points, "insufficient_trusted_coverage"
    return points, None


def _projection_data(reference: RouteReference):
    points = np.asarray(reference.points, dtype=float)
    following = np.roll(points, -1, axis=0)
    vectors = following - points
    lengths_sq = np.einsum("ij,ij->i", vectors, vectors)
    trusted = np.asarray(reference.trusted, dtype=bool)
    valid = trusted & np.roll(trusted, -1) & (lengths_sq > 1e-12)
    indices = np.flatnonzero(valid)
    if len(indices) == 0:
        raise ValueError("no_trusted_route_for_node_alignment")
    return (points[indices], vectors[indices], lengths_sq[indices],
            np.sqrt(lengths_sq[indices]),
            np.asarray(reference.cumulative_m[:-1], dtype=float)[indices])


def _project_point(point: np.ndarray, reference: RouteReference, data):
    starts, vectors, lengths_sq, lengths, cumulative = data
    offsets = point - starts
    fractions = np.einsum("ij,ij->i", offsets, vectors) / lengths_sq
    fractions = np.clip(fractions, 0.0, 1.0)
    projected = starts + fractions[:, None] * vectors
    residuals = point - projected
    distances_sq = np.einsum("ij,ij->i", residuals, residuals)
    best = int(np.argmin(distances_sq))
    progress = (cumulative[best] + fractions[best] * lengths[best]) % reference.length_m
    return projected[best], float(progress), math.sqrt(max(0.0, float(distances_sq[best])))


def _project_nodes(reference: RouteReference, map_points: np.ndarray, scale: float,
                   rotation: float, translation: Point, data):
    cosine, sine = math.cos(rotation), math.sin(rotation)
    matrix = np.asarray([[cosine, -sine], [sine, cosine]], dtype=float)
    transformed = scale * (map_points @ matrix.T) + np.asarray(translation, dtype=float)
    projected, progress, distances = [], [], []
    for point in transformed:
        location, along, distance = _project_point(point, reference, data)
        projected.append(location)
        progress.append(along)
        distances.append(distance)
    return (np.asarray(projected, dtype=float), np.asarray(progress, dtype=float),
            np.asarray(distances, dtype=float))


def _joint_project_nodes(reference: RouteReference, map_points: np.ndarray, scale: float,
                         rotation: float, translation: Point, data, direction: int,
                         node_search_m: float, feature_window_m: float,
                         candidate_count: int = 6, forbidden_node: Optional[int] = None,
                         forbidden_progress: float = 0.0, forbidden_radius_m: float = 0.0,
                         node_only: bool = False):
    """Choose cyclically ordered node projections jointly from several options.

    Only trusted route segments generate observations. The optimiser keeps
    separated alternatives along the route and applies the same chord/angle
    checks to the entire group. It never treats a chord as travelled distance.
    """
    starts, vectors, lengths_sq, lengths, cumulative = data
    c, s = math.cos(rotation), math.sin(rotation)
    matrix = np.asarray([[c, -s], [s, c]], dtype=float)
    transformed = scale * (map_points @ matrix.T) + np.asarray(translation, dtype=float)
    options = []
    separation = max(2.0 * reference.spacing_m, min(1.0, 0.25 * node_search_m))
    for node_index, point in enumerate(transformed):
        fractions = np.clip(np.einsum("ij,ij->i", point - starts, vectors) / lengths_sq, 0.0, 1.0)
        projected = starts + fractions[:, None] * vectors
        delta = point - projected
        distances_sq = np.einsum("ij,ij->i", delta, delta)
        progress = (cumulative + fractions * lengths) % reference.length_m
        selected = []
        for index in np.argsort(distances_sq):
            along = float(progress[index])
            if (node_index == forbidden_node
                    and _phase_distance(along, forbidden_progress, reference.length_m) <= forbidden_radius_m):
                continue
            if any(_phase_distance(along, item[1], reference.length_m) < separation
                   for item in selected):
                continue
            selected.append((projected[index], along,
                             math.sqrt(max(0.0, float(distances_sq[index])))))
            if len(selected) >= candidate_count:
                break
        if not selected:
            return None
        options.append(selected)

    n = len(map_points)
    order = list(range(n)) if direction > 0 else [0] + list(range(n - 1, 0, -1))

    def huber_square(value, transition):
        value, transition = abs(float(value)), max(float(transition), 1e-6)
        return value * value if value <= transition else 2.0 * transition * value - transition * transition

    def edge_cost(from_id, to_id, first, second):
        if node_only:
            return 0.0
        expected = transformed[to_id] - transformed[from_id]
        observed = second[0] - first[0]
        expected_length = math.hypot(float(expected[0]), float(expected[1]))
        observed_length = math.hypot(float(observed[0]), float(observed[1]))
        if min(expected_length, observed_length) <= 1e-6:
            return math.inf
        error = _wrap(math.atan2(observed[1], observed[0])
                      - math.atan2(expected[1], expected[0]))
        length_error = observed_length - expected_length
        return ((0.2 * feature_window_m * error) ** 2
                + 0.01 * huber_square(length_error, max(node_search_m, feature_window_m)))

    def minimum_gap(from_id, to_id):
        chord = scale * math.dist(map_points[to_id], map_points[from_id])
        return max(2.0 * reference.spacing_m, chord - 2.0 * node_search_m)

    # Edge costs depend on two options, not on which initial node was chosen.
    # Cache them once so the cyclic dynamic program does not repeat the work.
    edge_tables = []
    minimum_gaps = []
    for index, from_id in enumerate(order):
        to_id = order[(index + 1) % n]
        minimum_gaps.append(minimum_gap(from_id, to_id))
        edge_tables.append([[edge_cost(from_id, to_id, first, second)
                             for second in options[to_id]] for first in options[from_id]])

    winner = None
    for first_index, first in enumerate(options[order[0]]):
        # Each state stores the chosen candidate indices and unwrapped progress.
        states = [(huber_square(first[2], feature_window_m), [first_index], 0.0, first_index)]
        for index in range(1, n):
            current_id = order[index]
            gap_min = minimum_gaps[index - 1]
            next_states = []
            for current_index, current in enumerate(options[current_id]):
                along = (current[1] - first[1]) % reference.length_m
                best = None
                for cost, chosen, previous_along, previous_index in states:
                    if along - previous_along < gap_min - 1e-6:
                        continue
                    value = (cost + huber_square(current[2], feature_window_m)
                             + edge_tables[index - 1][previous_index][current_index])
                    if best is None or value < best[0]:
                        best = (value, chosen + [current_index], along, current_index)
                if best is not None and math.isfinite(best[0]):
                    next_states.append(best)
            states = next_states
            if not states:
                break
        for cost, chosen, along, last_index in states:
            if len(chosen) != n or reference.length_m - along < minimum_gaps[-1] - 1e-6:
                continue
            total = cost + edge_tables[-1][last_index][first_index]
            if math.isfinite(total) and (winner is None or total < winner[0]):
                winner = (total, chosen)
    if winner is None:
        return None
    selected = [None] * n
    for index, option_index in zip(order, winner[1]):
        selected[index] = options[index][option_index]
    return (np.asarray([item[0] for item in selected], dtype=float),
            np.asarray([item[1] for item in selected], dtype=float),
            np.asarray([item[2] for item in selected], dtype=float))


def _coarse_curve(reference: RouteReference, step_m: float):
    count = max(48, int(math.ceil(reference.length_m / step_m)))
    progress, points = [], []
    # Sample from a canonical physical location so moving the recording seam
    # does not shift the coarse search grid around the same closed lap.
    trusted_indices = [index for index, good in enumerate(reference.trusted) if good]
    if not trusted_indices:
        raise ValueError("insufficient_trusted_route_for_node_alignment")
    anchor_index = min(trusted_indices, key=lambda index: reference.points[index])
    anchor_progress = reference.cumulative_m[anchor_index]
    for index in range(count):
        distance = (anchor_progress + reference.length_m * index / count) % reference.length_m
        x, y, trusted = reference.point_at(distance)
        if trusted:
            progress.append(distance)
            points.append((x, y))
    if len(points) < 12:
        raise ValueError("insufficient_trusted_route_for_node_alignment")
    return np.asarray(points, dtype=float), np.asarray(progress, dtype=float)


def _route_order(node_ids: List[str], direction: int) -> List[str]:
    if direction > 0:
        return list(node_ids)
    return [node_ids[0]] + list(reversed(node_ids[1:]))


def _progress_gaps(progress_by_id: Dict[str, float], ordered_ids: List[str],
                   map_points_by_id: Dict[str, Point], scale: float,
                   route_length: float, spacing_m: float,
                   node_search_m: float) -> Optional[List[float]]:
    gaps = []
    for index, current in enumerate(ordered_ids):
        following = ordered_ids[(index + 1) % len(ordered_ids)]
        gap = (progress_by_id[following] - progress_by_id[current]) % route_length
        chord = scale * math.dist(map_points_by_id[current], map_points_by_id[following])
        minimum = max(2.0 * spacing_m, chord - 2.0 * node_search_m)
        if gap < minimum - 1e-6:
            return None
        gaps.append(gap)
    if abs(math.fsum(gaps) - route_length) > max(1e-6, 2.0 * spacing_m):
        return None
    return gaps


def _huber_rms(values: np.ndarray, transition: float) -> float:
    transition = max(float(transition), 1e-6)
    absolute = np.abs(values)
    loss = np.where(absolute <= transition, absolute * absolute,
                    2.0 * transition * absolute - transition * transition)
    return math.sqrt(max(0.0, float(np.mean(loss))))


def _edge_shape_errors(node_ids: List[str], direction: int,
                       points_by_id: Dict[str, Point],
                       projected_by_id: Dict[str, np.ndarray],
                       scale: float, rotation: float):
    ordered = _route_order(node_ids, direction)
    cosine, sine = math.cos(rotation), math.sin(rotation)
    errors, length_errors = [], []
    for index, current in enumerate(ordered):
        following = ordered[(index + 1) % len(ordered)]
        map_delta = np.asarray(points_by_id[following]) - np.asarray(points_by_id[current])
        expected = scale * np.asarray([
            cosine * map_delta[0] - sine * map_delta[1],
            sine * map_delta[0] + cosine * map_delta[1],
        ])
        measured = projected_by_id[following] - projected_by_id[current]
        expected_length = float(np.linalg.norm(expected))
        measured_length = float(np.linalg.norm(measured))
        if min(expected_length, measured_length) <= 1e-6:
            return None, None
        angle_error = abs(_wrap(math.atan2(measured[1], measured[0])
                                - math.atan2(expected[1], expected[0])))
        errors.append(angle_error)
        length_errors.append(measured_length - expected_length)
    return np.asarray(errors, dtype=float), np.asarray(length_errors, dtype=float)


def _candidate_metrics(reference: RouteReference, node_ids: List[str],
                       map_points: np.ndarray, scale: float, rotation: float,
                       translation: Point, direction: int, coarse_points: np.ndarray,
                       coarse_progress: np.ndarray, node_search_m: float,
                       feature_window_m: float, allow_unordered: bool = False):
    cosine, sine = math.cos(rotation), math.sin(rotation)
    matrix = np.asarray([[cosine, -sine], [sine, cosine]], dtype=float)
    transformed = scale * (map_points @ matrix.T) + np.asarray(translation, dtype=float)
    offsets = transformed[:, None, :] - coarse_points[None, :, :]
    distances_sq = np.einsum("ijk,ijk->ij", offsets, offsets)
    nearest = np.argmin(distances_sq, axis=1)
    projected = coarse_points[nearest]
    progress = coarse_progress[nearest]
    distances = np.sqrt(np.maximum(0.0, distances_sq[np.arange(len(nearest)), nearest]))
    progress_by_id = dict(zip(node_ids, [float(value) for value in progress]))
    points_by_id = dict(zip(node_ids, [tuple(point) for point in map_points]))
    projected_by_id = dict(zip(node_ids, [point for point in projected]))
    ordered = _route_order(node_ids, direction)
    if allow_unordered:
        gaps = None
        angle_errors = np.zeros(len(node_ids), dtype=float)
        length_errors = np.zeros(len(node_ids), dtype=float)
    else:
        gaps = _progress_gaps(progress_by_id, ordered, points_by_id, scale,
                              reference.length_m, reference.spacing_m, node_search_m)
        if gaps is None:
            return None
        angle_errors, length_errors = _edge_shape_errors(
            node_ids, direction, points_by_id, projected_by_id, scale, rotation)
        if angle_errors is None:
            return None
    node_rmse = math.sqrt(float(np.mean(distances * distances)))
    node_score = _huber_rms(distances, feature_window_m)
    angle_rmse = math.sqrt(float(np.mean(angle_errors * angle_errors)))
    length_score = _huber_rms(length_errors, max(node_search_m, feature_window_m))
    # The coarse pass may have selected a wrong nearest segment. Keep its
    # transform as a seed; the joint ordered pass chooses the associations.
    score = (node_score if allow_unordered else
             node_score + 0.2 * feature_window_m * angle_rmse + 0.1 * length_score)
    return {
        "scale_map_to_odom": float(scale),
        "rotation_map_to_odom_rad": float(rotation),
        "translation_map_to_odom": (float(translation[0]), float(translation[1])),
        "direction": int(direction),
        "node_ids": list(node_ids),
        "node_progress": progress_by_id,
        "node_positions": {identifier: np.asarray(projected_by_id[identifier], dtype=float)
                           for identifier in node_ids},
        "node_distances": {identifier: float(distance)
                           for identifier, distance in zip(node_ids, distances)},
        "node_rmse_m": node_rmse,
        "node_max_distance_m": float(np.max(distances)),
        "edge_angle_rmse_rad": angle_rmse,
        "edge_angle_max_rad": float(np.max(angle_errors)),
        "edge_chord_rmse_m": math.sqrt(float(np.mean(length_errors * length_errors))),
        "score": float(score),
        "phase_progress_m": float(progress_by_id[node_ids[0]]),
        "progress_gaps_m": gaps,
    }


def _fit_similarity(map_points: np.ndarray, odom_points: np.ndarray,
                    weights: np.ndarray):
    weight_sum = float(np.sum(weights))
    if weight_sum <= 1e-12:
        raise ValueError("no_weighted_node_correspondences")
    map_mean = np.sum(map_points * weights[:, None], axis=0) / weight_sum
    odom_mean = np.sum(odom_points * weights[:, None], axis=0) / weight_sum
    map_centred = map_points - map_mean
    odom_centred = odom_points - odom_mean
    dot = float(np.sum(weights * np.einsum("ij,ij->i", map_centred, odom_centred)))
    cross = float(np.sum(weights * (map_centred[:, 0] * odom_centred[:, 1]
                                    - map_centred[:, 1] * odom_centred[:, 0])))
    denominator = float(np.sum(weights * np.einsum("ij,ij->i", map_centred, map_centred)))
    magnitude = math.hypot(dot, cross)
    if denominator <= 1e-10 or magnitude <= 1e-10:
        raise ValueError("degenerate_outer_node_constellation")
    rotation = math.atan2(cross, dot)
    scale = magnitude / denominator
    cosine, sine = math.cos(rotation), math.sin(rotation)
    rotated_mean = np.asarray([cosine * map_mean[0] - sine * map_mean[1],
                               sine * map_mean[0] + cosine * map_mean[1]])
    translation = odom_mean - scale * rotated_mean
    return float(scale), rotation, (float(translation[0]), float(translation[1]))


def _refine_candidate(reference: RouteReference, node_ids: List[str],
                      map_points: np.ndarray, seed: dict, direction: int,
                      node_search_m: float, feature_window_m: float):
    data = _projection_data(reference)
    scale = seed["scale_map_to_odom"]
    rotation = seed["rotation_map_to_odom_rad"]
    translation = seed["translation_map_to_odom"]
    points_by_id = dict(zip(node_ids, [tuple(point) for point in map_points]))
    for _ in range(15):
        joint = _joint_project_nodes(reference, map_points, scale, rotation, translation,
                                     data, direction, node_search_m, feature_window_m)
        if joint is None:
            return None
        projected, progress, distances = joint
        progress_by_id = dict(zip(node_ids, [float(value) for value in progress]))
        ordered = _route_order(node_ids, direction)
        if _progress_gaps(progress_by_id, ordered, points_by_id, scale,
                          reference.length_m, reference.spacing_m, node_search_m) is None:
            return None
        transition = max(feature_window_m, 0.05)
        weights = np.ones(len(distances), dtype=float)
        large = distances > transition
        weights[large] = transition / distances[large]
        try:
            new_scale, new_rotation, new_translation = _fit_similarity(
                map_points, projected, weights)
        except ValueError:
            return None
        if not _MIN_MAP_TO_ODOM_SCALE <= new_scale <= _MAX_MAP_TO_ODOM_SCALE:
            return None
        centre = np.mean(map_points, axis=0)
        old_centre = (scale * np.asarray([math.cos(rotation) * centre[0] - math.sin(rotation) * centre[1],
                                          math.sin(rotation) * centre[0] + math.cos(rotation) * centre[1]])
                      + np.asarray(translation))
        new_centre = (new_scale * np.asarray([math.cos(new_rotation) * centre[0] - math.sin(new_rotation) * centre[1],
                                              math.sin(new_rotation) * centre[0] + math.cos(new_rotation) * centre[1]])
                      + np.asarray(new_translation))
        change = (abs(_wrap(new_rotation - rotation))
                  + abs(math.log(new_scale / scale))
                  + float(np.linalg.norm(new_centre - old_centre)))
        scale, rotation, translation = new_scale, new_rotation, new_translation
        if change < 1e-5:
            break

    joint = _joint_project_nodes(reference, map_points, scale, rotation, translation,
                                 data, direction, node_search_m, feature_window_m)
    if joint is None:
        return None
    projected, progress, distances = joint
    return _candidate_from_projection(reference, node_ids, map_points, scale, rotation,
                                      translation, direction, node_search_m, feature_window_m,
                                      projected, progress, distances)


def _candidate_from_projection(reference: RouteReference, node_ids: List[str],
                               map_points: np.ndarray, scale: float, rotation: float,
                               translation: Point, direction: int, node_search_m: float,
                               feature_window_m: float, projected: np.ndarray,
                               progress: np.ndarray, distances: np.ndarray):
    points_by_id = dict(zip(node_ids, [tuple(point) for point in map_points]))
    progress_by_id = dict(zip(node_ids, [float(value) for value in progress]))
    ordered = _route_order(node_ids, direction)
    if _progress_gaps(progress_by_id, ordered, points_by_id, scale,
                      reference.length_m, reference.spacing_m, node_search_m) is None:
        return None
    projected_by_id = dict(zip(node_ids, [point for point in projected]))
    angle_errors, length_errors = _edge_shape_errors(
        node_ids, direction, points_by_id, projected_by_id, scale, rotation)
    if angle_errors is None:
        return None
    node_rmse = math.sqrt(float(np.mean(distances * distances)))
    node_score = _huber_rms(distances, feature_window_m)
    angle_rmse = math.sqrt(float(np.mean(angle_errors * angle_errors)))
    length_score = _huber_rms(length_errors, max(node_search_m, feature_window_m))
    score = node_score + 0.2 * feature_window_m * angle_rmse + 0.1 * length_score
    return {
        "scale_map_to_odom": float(scale),
        "rotation_map_to_odom_rad": float(rotation),
        "translation_map_to_odom": (float(translation[0]), float(translation[1])),
        "direction": int(direction),
        "node_ids": list(node_ids),
        "node_progress": progress_by_id,
        "node_positions": {identifier: np.asarray(projected_by_id[identifier], dtype=float)
                           for identifier in node_ids},
        "node_distances": {identifier: float(distance)
                           for identifier, distance in zip(node_ids, distances)},
        "node_rmse_m": node_rmse,
        "node_max_distance_m": float(np.max(distances)),
        "edge_angle_rmse_rad": angle_rmse,
        "edge_angle_max_rad": float(np.max(angle_errors)),
        "edge_chord_rmse_m": math.sqrt(float(np.mean(length_errors * length_errors))),
        "score": float(score),
        "phase_progress_m": float(progress_by_id[node_ids[0]]),
    }


def _phase_distance(first: float, second: float, length: float) -> float:
    return abs((first - second + 0.5 * length) % length - 0.5 * length)


def _same_basin(first: dict, second: dict, route_length: float,
                node_search_m: float) -> bool:
    if first["direction"] != second["direction"]:
        return False
    phase_close = _phase_distance(first["phase_progress_m"],
                                  second["phase_progress_m"], route_length)
    rotation_close = abs(_wrap(first["rotation_map_to_odom_rad"]
                               - second["rotation_map_to_odom_rad"])) < math.radians(12.0)
    # Raw translation depends on the arbitrary origin and units of map_points.
    # Compare the resulting node associations in the odom frame instead.
    node_ids = first["node_ids"]
    node_delta_rms = math.sqrt(math.fsum(
        math.dist(first["node_positions"][identifier],
                  second["node_positions"][identifier]) ** 2
        for identifier in node_ids
    ) / len(node_ids))
    positions_close = node_delta_rms < max(2.0 * node_search_m, 4.0)
    scale_close = abs(first["scale_map_to_odom"] / second["scale_map_to_odom"] - 1.0) < 0.15
    return phase_close < max(2.0 * node_search_m, 4.0) and rotation_close and positions_close and scale_close


def _mapped_point(candidate: dict, point: Point) -> Point:
    angle = candidate["rotation_map_to_odom_rad"]
    scale = candidate["scale_map_to_odom"]
    tx, ty = candidate["translation_map_to_odom"]
    c, s = math.cos(angle), math.sin(angle)
    return (scale * (c * point[0] - s * point[1]) + tx,
            scale * (s * point[0] + c * point[1]) + ty)


def _candidate_disagreement(first: dict, second: dict, comparison_points) -> float:
    distances = [math.dist(first["node_positions"][identifier],
                           second["node_positions"][identifier])
                 for identifier in first["node_ids"]]
    distances.extend(math.dist(_mapped_point(first, point), _mapped_point(second, point))
                     for point in comparison_points)
    return max(distances, default=0.0)


def _to_alignment(candidate: dict, reference: RouteReference, closure_gap: float,
                  official: OfficialRoute, alternative: Optional[dict],
                  max_rmse_m: float, ambiguity_margin_m: float,
                  node_search_m: float):
    map_to_odom_scale = candidate["scale_map_to_odom"]
    map_to_odom_rotation = candidate["rotation_map_to_odom_rad"]
    tx, ty = candidate["translation_map_to_odom"]
    odom_to_map_scale = 1.0 / map_to_odom_scale
    inverse_cosine, inverse_sine = math.cos(map_to_odom_rotation), math.sin(map_to_odom_rotation)
    inverse_tx = inverse_cosine * tx + inverse_sine * ty
    inverse_ty = -inverse_sine * tx + inverse_cosine * ty
    node_zero_progress = candidate["node_progress"][official.node_ids[0]]
    phase = (-candidate["direction"] * node_zero_progress / reference.length_m) % 1.0
    gap = (max(0.0, alternative["score"] - candidate["score"])
           if alternative is not None else None)
    reason = "accepted"
    if candidate["node_rmse_m"] > max_rmse_m:
        reason = "rmse_exceeds_limit"
    elif candidate["node_max_distance_m"] > node_search_m:
        reason = "node_offset_exceeds_search"
    elif candidate["edge_angle_max_rad"] > math.radians(60.0):
        reason = "outer_edge_direction_inconsistent"
    elif alternative is not None and gap <= ambiguity_margin_m + 1e-12:
        reason = "ambiguous_correspondence"
    return {
        "scale": odom_to_map_scale,
        "rotation_rad": _wrap(-map_to_odom_rotation),
        "translation_x": -odom_to_map_scale * inverse_tx,
        "translation_y": -odom_to_map_scale * inverse_ty,
        "rmse_m": candidate["node_rmse_m"],
        "phase_fraction": phase,
        "direction": candidate["direction"],
        "available": reason == "accepted",
        "reason": reason,
        "alternative_rmse_m": alternative["node_rmse_m"] if alternative is not None else None,
        "alternative_phase_fraction": (
            (-alternative["direction"] * alternative["node_progress"][official.node_ids[0]]
             / reference.length_m) % 1.0 if alternative is not None else None
        ),
        "alternative_direction": alternative["direction"] if alternative is not None else None,
        "ambiguity_gap_m": gap,
        "closure_gap_m": closure_gap,
        "recorded_closed_length_m": reference.length_m,
        "official_chord_length_m": official.chord_length_m,
        "sample_count": len(official.node_ids) - 1,
        "transform_direction": "recorded_odom_to_official_xy",
        "matching_method": "ordered_outer_nodes_to_curve",
        "matching_scale_map_to_odom": map_to_odom_scale,
        "node_max_distance_m": candidate["node_max_distance_m"],
        "edge_angle_rmse_deg": math.degrees(candidate["edge_angle_rmse_rad"]),
        "edge_angle_max_deg": math.degrees(candidate["edge_angle_max_rad"]),
        "edge_chord_rmse_m": candidate["edge_chord_rmse_m"],
        "ambiguity_margin_m": ambiguity_margin_m,
        "field_validation_required": True,
        "assumption": "outer topo nodes lie near the recorded loop; node chords constrain direction, not travelled arc length",
    }


def _match_outer_nodes(reference: RouteReference, official: OfficialRoute,
                       node_search_m: float, feature_window_m: float,
                       max_rmse_m: float, ambiguity_margin_m: float,
                       closure_gap: float, comparison_points=(),
                       ambiguity_position_m: float = 1.0, branch_connections=None,
                       branch_angle_min_deg=80.0, branch_angle_max_deg=95.0):
    from .geometric_alignment import match_outer_nodes
    return match_outer_nodes(reference, official, node_search_m, feature_window_m,
                             max_rmse_m, ambiguity_margin_m, closure_gap,
                             comparison_points, ambiguity_position_m, branch_connections,
                             branch_angle_min_deg, branch_angle_max_deg)


def _shape_displacements(reference: RouteReference, official: OfficialRoute, alignment: dict,
                         anchors: Dict[str, float], weight: float, maximum: float,
                         direction: int = 1) -> Dict[str, Point]:
    """Apply small edge-direction corrections while retaining measured chords."""
    ids = sorted(anchors, key=anchors.get)
    count = len(ids)
    measured = np.asarray([reference.point_at(anchors[identifier])[:2] for identifier in ids])
    official_order = official.node_ids[:-1]
    official_by_id = dict(zip(official_order, official.points[:-1]))
    for index, identifier in enumerate(ids):
        expected = official_order[(official_order.index(identifier) + direction) % count]
        if ids[(index + 1) % count] != expected:
            raise ValueError("node_order_or_separation_invalid")
    rows, targets = [], []
    for index in range(count * 2):
        row = np.zeros(count * 2)
        row[index] = 1.0
        rows.append(row)
        targets.append(0.0)
    direction_pairs = []
    for index, identifier in enumerate(ids):
        following = ids[(index + 1) % count]
        measured_edge = (measured[index + 1] - measured[index]
                         if index < count - 1 else measured[0] - measured[index])
        official_edge = np.asarray(official_by_id[following]) - np.asarray(official_by_id[identifier])
        length = float(np.linalg.norm(measured_edge))
        official_length = float(np.linalg.norm(official_edge))
        if (not math.isfinite(length) or length <= max(1e-6, reference.spacing_m)
                or official_length <= 1e-6):
            raise ValueError("degenerate_measured_node_chord")
        direction_pairs.append((measured_edge / length, official_edge / official_length))
    a = sum(float(np.dot(first, second)) for first, second in direction_pairs)
    b = sum(float(first[0] * second[1] - first[1] * second[0])
            for first, second in direction_pairs)
    if math.hypot(a, b) < 0.5 * count:
        raise ValueError("inconsistent_measured_edge_directions")
    node_rotation = math.atan2(b, a)
    alignment["node_rotation_rad"] = node_rotation
    cosine, sine = math.cos(node_rotation), math.sin(node_rotation)
    strength = math.sqrt(weight / max(1.0 - weight, 1e-9))
    for index, identifier in enumerate(ids):
        following = ids[(index + 1) % count]
        ox, oy = official_by_id[identifier]
        nx, ny = official_by_id[following]
        norm = math.hypot(nx - ox, ny - oy)
        desired_direction = np.asarray([(cosine * (nx - ox) + sine * (ny - oy)) / norm,
                                        (-sine * (nx - ox) + cosine * (ny - oy)) / norm])
        measured_edge = (measured[index + 1] - measured[index]
                         if index < count - 1 else measured[0] - measured[index])
        length = float(np.linalg.norm(measured_edge))
        residual = length * desired_direction - measured_edge
        following_index = (index + 1) % count
        for axis in range(2):
            row = np.zeros(count * 2)
            row[following_index * 2 + axis] = strength
            row[index * 2 + axis] = -strength
            rows.append(row)
            targets.append(strength * float(residual[axis]))
        row = np.zeros(count * 2)
        row[following_index * 2:following_index * 2 + 2] = strength * 4.0 * measured_edge / length
        row[index * 2:index * 2 + 2] = -strength * 4.0 * measured_edge / length
        rows.append(row)
        targets.append(0.0)
    delta = np.linalg.lstsq(np.asarray(rows), np.asarray(targets), rcond=None)[0].reshape((-1, 2))
    if not np.isfinite(delta).all():
        raise ValueError("nonfinite_shape_solution")
    largest = max(float(np.linalg.norm(value)) for value in delta)
    if largest > maximum:
        delta *= maximum / largest
    return {identifier: (float(value[0]), float(value[1]))
            for identifier, value in zip(ids, delta)}


def build_fused_route(
    samples, official: OfficialRoute, *, spacing_m: float = 0.2,
    smooth_window_m: float = 1.0, max_detour_gap_m: float = 2.0,
    max_rmse_m: float = 3.0, ambiguity_margin_m: float = 0.3,
    node_search_m: float = 4.0, feature_window_m: float = 2.0,
    shape_weight: float = 0.25, max_displacement_m: float = 0.75,
    comparison_points=None, ambiguity_position_m: float = 1.0, branch_connections=None,
    branch_angle_min_deg=80.0, branch_angle_max_deg=95.0,
    max_closure_gap_m=4.0,
) -> FusionResult:
    """Register ordered outer topo nodes to the measured closed Odom curve.

    The registration searches cyclic start phase and travel direction using
    the node constellation. Odom arc length is not equated with summed topo
    chord lengths. Topo coordinates may be scaled into the Odom frame, while
    the returned driving route keeps the measured Odom scale and curvature.
    """
    for name, value in (("spacing_m", spacing_m), ("smooth_window_m", smooth_window_m),
                        ("max_detour_gap_m", max_detour_gap_m),
                        ("node_search_m", node_search_m), ("feature_window_m", feature_window_m),
                        ("max_displacement_m", max_displacement_m), ("shape_weight", shape_weight),
                        ("max_rmse_m", max_rmse_m), ("ambiguity_margin_m", ambiguity_margin_m),
                        ("ambiguity_position_m", ambiguity_position_m)):
        if isinstance(value, bool) or not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and nonnegative")
    if min(spacing_m, node_search_m, feature_window_m, ambiguity_position_m) <= 0.0 or shape_weight >= 1.0:
        raise ValueError("Positive feature windows and shape_weight < 1 are required")
    if (isinstance(branch_angle_min_deg, bool) or isinstance(branch_angle_max_deg, bool)
            or not math.isfinite(branch_angle_min_deg) or not math.isfinite(branch_angle_max_deg)
            or not 0.0 <= branch_angle_min_deg <= branch_angle_max_deg <= 180.0):
        raise ValueError("Branch-angle range must be finite and within 0..180 degrees")
    try:
        comparison_points = [] if comparison_points is None else list(comparison_points)
        comparison_points = [tuple(float(value) for value in point) for point in comparison_points]
        if any(len(point) != 2 or not all(math.isfinite(value) for value in point)
               for point in comparison_points):
            raise ValueError("Comparison points must be finite pairs")
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("Comparison points must be finite pairs") from exc
    if len(set(official.node_ids[:-1])) != len(official.node_ids) - 1:
        raise ValueError("Fusion requires an outer loop without repeated interior nodes")
    samples = list(samples)
    closure_gap = (math.dist((samples[0].x, samples[0].y),
                             (samples[-1].x, samples[-1].y)) if len(samples) > 1 else math.inf)
    prepared = build_odom_reference(samples, spacing_m, 0.0)
    base_status = {"available": False, "mode": "odom_only", "metric_source": "odometry",
                   "recorded_length_m": prepared.length_m,
                   "official_size_used": False, "field_validation_required": True}

    fitted_samples = None

    def fallback(reason, alignment=None):
        source = fitted_samples if fitted_samples is not None else samples
        reference = build_odom_reference(source, spacing_m, smooth_window_m)
        return FusionResult(reference, alignment,
                            {**base_status, "reason": reason, "final_length_m": reference.length_m}, {}, {})

    if not math.isfinite(max_closure_gap_m) or max_closure_gap_m < 0.0:
        raise ValueError("max_closure_gap_m must be finite and nonnegative")
    fitted_points, coverage_reason = _fit_trace(prepared, max_detour_gap_m, max_closure_gap_m)
    if coverage_reason:
        return fallback(coverage_reason)
    if any(math.dist(fitted, current) > 1e-8
           for fitted, current in zip(fitted_points, prepared.points)):
        # Keep the fitted exclusion gaps as untrusted. Rebuild cumulative
        # distance on the straightened trace before projection and smoothing.
        fitted_samples = [RouteSample(x, y, 0.0, excluded=excluded, closure=closure)
                          for (x, y), excluded, closure in zip(
                              fitted_points, prepared.excluded_mask, prepared.closure_mask)]
        prepared = RouteReference(fitted_samples, spacing_m, 0.0, 0.0)
    try:
        alignment, anchors, reason = _match_outer_nodes(
            prepared, official, node_search_m, feature_window_m,
            max_rmse_m, ambiguity_margin_m, closure_gap, comparison_points,
            ambiguity_position_m,
            branch_connections,
            branch_angle_min_deg, branch_angle_max_deg,
        )
    except (ValueError, OverflowError, np.linalg.LinAlgError) as exc:
        return fallback("node_alignment_failed: " + str(exc))
    if alignment is None or not alignment.get("available"):
        return fallback(reason, alignment)
    try:
        displacements = _shape_displacements(
            prepared, official, alignment, anchors, shape_weight,
            max_displacement_m, direction=int(alignment["direction"]),
        )
    except (ValueError, np.linalg.LinAlgError) as exc:
        return fallback("shape_fusion_failed: " + str(exc), alignment)
    ordered = sorted(anchors, key=anchors.get)
    progress = [anchors[identifier] for identifier in ordered]

    def displacement_at(distance):
        left_index = (bisect_right(progress, distance) - 1) % len(ordered)
        right_index = (left_index + 1) % len(ordered)
        left, right = progress[left_index], progress[right_index]
        span = (right - left) % prepared.length_m
        if span <= 1e-9:
            return (0.0, 0.0)
        fraction = ((distance - left) % prepared.length_m) / span
        fraction = fraction * fraction * (3.0 - 2.0 * fraction)
        first, second = displacements[ordered[left_index]], displacements[ordered[right_index]]
        return (first[0] + fraction * (second[0] - first[0]),
                first[1] + fraction * (second[1] - first[1]))

    deformed = []
    for index, (x, y) in enumerate(prepared.points):
        dx, dy = displacement_at(prepared.cumulative_m[index])
        deformed.append(RouteSample(x + dx, y + dy, 0.0,
                                    excluded=prepared.excluded_mask[index],
                                    closure=prepared.closure_mask[index]))
    if any(sample.excluded or sample.closure for sample in deformed):
        # Interpolating shape displacements can bend an excluded straight gap.
        # Straighten it again after deformation without marking it trusted.
        deformed_points = [(sample.x, sample.y) for sample in deformed]
        trusted = [not (sample.excluded or sample.closure) for sample in deformed]
        RouteReference._straighten_short_exclusions(deformed_points, trusted, math.inf)
        deformed = [RouteSample(x, y, 0.0, excluded=sample.excluded, closure=sample.closure)
                    for (x, y), sample in zip(deformed_points, deformed)]
    reference = RouteReference(deformed, spacing_m, smooth_window_m, 0.0)
    deformed_cumulative = RouteReference._cumulative([(sample.x, sample.y) for sample in deformed])
    node_positions, node_progress = {}, {}
    for identifier, distance in anchors.items():
        edge = min(bisect_right(prepared.cumulative_m, distance) - 1, len(prepared.points) - 1)
        fraction = ((distance - prepared.cumulative_m[edge])
                    / (prepared.cumulative_m[edge + 1] - prepared.cumulative_m[edge]))
        before_smoothing_s = (deformed_cumulative[edge]
                              + fraction * (deformed_cumulative[edge + 1] - deformed_cumulative[edge]))
        source_step = deformed_cumulative[-1] / len(reference.points)
        output_index = min(int(before_smoothing_s / source_step), len(reference.points) - 1)
        output_fraction = (before_smoothing_s - output_index * source_step) / source_step
        expected_s = (reference.cumulative_m[output_index]
                      + output_fraction * (reference.cumulative_m[output_index + 1]
                                           - reference.cumulative_m[output_index]))
        x, y = prepared.point_at(distance)[:2]
        dx, dy = displacements[identifier]
        position = ordered.index(identifier)
        before_gap = (distance - progress[position - 1]) % prepared.length_m
        after_gap = (progress[(position + 1) % len(progress)] - distance) % prepared.length_m
        local_window = min(smooth_window_m + 2.0 * spacing_m,
                           0.25 * min(before_gap, after_gap))
        matched = reference.match(
            x + dx, y + dy, expected_s, 0.0, local_window,
            max_displacement_m + smooth_window_m + 2.0 * spacing_m,
        )
        if matched is None or not reference.point_at(matched)[2]:
            return fallback("final_node_projection_unavailable", alignment)
        node_progress[identifier] = matched % reference.length_m
        node_positions[identifier] = reference.point_at(matched)[:2]
    final_ids = sorted(node_progress, key=node_progress.get)
    if any(final_ids[(final_ids.index(identifier) + 1) % len(final_ids)]
           != ordered[(index + 1) % len(ordered)]
           for index, identifier in enumerate(ordered)):
        return fallback("final_node_order_invalid", alignment)
    maximum = max(math.hypot(*value) for value in displacements.values())
    status = {**base_status, "available": True, "mode": "fused", "reason": "accepted",
              "final_length_m": reference.length_m, "node_count": len(node_positions),
              "max_node_displacement_m": maximum, "shape_weight": shape_weight,
              "direction": alignment["direction"],
              "node_source": alignment["matching_method"],
              "matching_method": alignment["matching_method"]}
    return FusionResult(reference, alignment, status, node_positions, node_progress)
