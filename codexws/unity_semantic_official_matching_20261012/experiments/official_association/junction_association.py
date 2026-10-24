"""Disabled, ROS-independent candidate for observed junction association.

No route deformation, graph construction, traffic decision, or command output.
This consumes a vetted positive-evidence ledger; pixel claims are NOT proved by
this module. See DESIGN.md before attempting integration. No active core imports
are needed: OfficialRoute and RouteReference are accepted by their public data.
"""

from dataclasses import dataclass
import copy
import math
from typing import Any, Dict, Mapping, Sequence, Tuple


Point = Tuple[float, float]
_EPS = 1e-8


@dataclass(frozen=True)
class CaptureSupport:
    """One actually new capture, with geometry in its capture-time odom frame.

    Pixel counts refer to BOTH observed boundaries of the SAME side branch,
    not the two main-lane boundaries. Counts alone cannot establish that claim.
    All times use the same source clock; receipt is the original ingress time.
    """

    sequence: int
    capture_time_s: float
    received_time_s: float
    pose_time_s: float
    epoch: int
    calibration_id: str
    frame_id: str
    anchor_xy: Point
    branch_bearing_rad: float
    positive_left_pixels: int
    positive_right_pixels: int
    truncated: bool = False


@dataclass(frozen=True)
class JunctionObservation:
    """An anonymous, independently tracked branch junction, never an OSM ID.

    anchor_xy is the measured junction station on the driven main corridor,
    not the outer lip of a road opening or a nearby bend. Its extraction and
    uncertainty must be justified by actual positive boundary observations.
    Bearing is directed OUT of the main corridor into the observed branch.
    """

    observation_id: str
    anchor_xy: Point
    branch_bearing_rad: float
    position_uncertainty_m: float
    bearing_uncertainty_rad: float
    supports: Tuple[CaptureSupport, ...]


@dataclass(frozen=True)
class ObservationContext:
    """Caller-verified completed-lap ledger identity, not an inferred closure."""

    epoch: int
    calibration_id: str
    frame_id: str
    lap_start_time_s: float
    lap_end_time_s: float
    completed_lap: bool


@dataclass(frozen=True)
class AssociationLimits:
    # These are candidate contracts, NOT tuned or validated on real junctions.
    max_projection_m: float = 1.0
    projection_ambiguity_m: float = 0.25
    remote_progress_m: float = 2.0
    trusted_half_window_m: float = 0.5
    min_anchor_separation_m: float = 2.0
    max_ingress_age_s: float = 0.5
    max_support_span_s: float = 1.5
    max_pose_skew_s: float = 1e-6
    min_support_frames: int = 3
    max_position_uncertainty_m: float = 0.5
    max_bearing_uncertainty_rad: float = math.radians(15.0)
    max_rmse_m: float = 3.0
    max_node_residual_m: float = 3.0
    ambiguity_margin_m: float = 0.3
    max_branch_error_rad: float = math.radians(30.0)
    min_official_per_odom_scale: float = 0.1
    max_official_per_odom_scale: float = 10.0


def _finite(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("non_numeric_value")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("non_finite_value")
    return result


def _point(value: Any) -> Point:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("invalid_point")
    return (_finite(value[0]), _finite(value[1]))


def _integer(value: Any, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise ValueError("invalid_integer")
    return value


def _wrap(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _distance(a: Point, b: Point) -> float:
    result = math.hypot(a[0] - b[0], a[1] - b[1])
    if not math.isfinite(result):
        raise ValueError("geometry_overflow")
    return result


def _validate_limits(limits: AssociationLimits) -> None:
    for name, value in vars(limits).items():
        if name == "min_support_frames":
            _integer(value, 3)
        elif _finite(value) <= 0.0:
            raise ValueError("nonpositive_limit:" + name)
    if (limits.max_projection_m > 2.0 or limits.max_rmse_m > 3.0
            or limits.max_node_residual_m > 3.0
            or limits.ambiguity_margin_m < 0.3
            or limits.projection_ambiguity_m < 0.25
            or limits.remote_progress_m > 2.0
            or limits.trusted_half_window_m < 0.5
            or limits.min_anchor_separation_m < 2.0
            or limits.max_ingress_age_s > 0.5
            or limits.max_support_span_s > 1.5
            or limits.max_pose_skew_s > 1e-6
            or limits.max_position_uncertainty_m > 0.5
            or limits.max_branch_error_rad > math.radians(30.0)
            or limits.max_bearing_uncertainty_rad > math.radians(15.0)
            or limits.remote_progress_m < 2.0 * limits.trusted_half_window_m
            or limits.min_official_per_odom_scale < 0.1
            or limits.max_official_per_odom_scale > 10.0
            or limits.min_official_per_odom_scale >= limits.max_official_per_odom_scale):
        raise ValueError("unsafe_or_inconsistent_limits")


def _official_data(official: Any) -> Tuple[list, list]:
    identifiers = list(official.node_ids)
    points = [_point(value) for value in official.points]
    if (len(identifiers) < 4 or len(points) != len(identifiers)
            or identifiers[0] != identifiers[-1] or points[0] != points[-1]
            or any(not isinstance(item, str) or not item for item in identifiers)
            or len(set(identifiers[:-1])) != len(identifiers) - 1
            or len(set(points[:-1])) != len(points) - 1):
        raise ValueError("invalid_official_cycle")
    return identifiers[:-1], points[:-1]


def branch_bearings_from_graph(official: Any, graph: Any) -> Dict[str, float]:
    """Read each single non-outer OSM edge as a *prior*, never sensor evidence.

    Degree alone is deliberately not an association feature. A curved internal
    road need not share its chord's entry tangent; this conservative candidate
    can therefore reject a real match. No unseen internal connectivity is used.
    Caller must load both objects from the same hash-checked original OSM.
    """

    identifiers, points = _official_data(official)
    outer = set(identifiers)
    output = {}
    for index, identifier in enumerate(identifiers):
        if _point(graph.nodes[identifier]) != points[index]:
            raise ValueError("official_graph_coordinate_mismatch")
        neighbours = graph.adjacency[identifier]
        for other in (identifiers[index - 1], identifiers[(index + 1) % len(identifiers)]):
            if (other not in neighbours or identifier not in graph.adjacency[other]
                    or _finite(neighbours[other]) <= 0.0):
                raise ValueError("official_cycle_edge_missing")
        branches = [other for other in neighbours if other not in outer]
        if len(neighbours) != 3 or len(branches) != 1:
            raise ValueError("requires_single_observable_non_outer_branch")
        other = branches[0]
        if identifier not in graph.adjacency[other] or _finite(neighbours[other]) <= 0.0:
            raise ValueError("official_branch_edge_invalid")
        destination = _point(graph.nodes[other])
        if _distance(destination, points[index]) <= _EPS:
            raise ValueError("degenerate_official_branch")
        output[identifier] = math.atan2(destination[1] - points[index][1],
                                       destination[0] - points[index][0])
    return output


def _reference_data(reference: Any) -> Tuple[list, list, list, float]:
    points = [_point(value) for value in reference.points]
    trusted = list(reference.trusted)
    cumulative = [_finite(value) for value in reference.cumulative_m]
    length = _finite(reference.length_m)
    if (len(points) < 3 or len(trusted) != len(points)
            or any(type(value) is not bool for value in trusted)
            or len(cumulative) != len(points) + 1 or cumulative[0] != 0.0
            or length <= _EPS or abs(cumulative[-1] - length) > 1e-7 * max(1.0, length)):
        raise ValueError("invalid_reference")
    for index, start in enumerate(points):
        segment = cumulative[index + 1] - cumulative[index]
        if (segment <= _EPS or abs(segment - _distance(start, points[(index + 1) % len(points)]))
                > 1e-7 * max(1.0, segment)):
            raise ValueError("invalid_reference_segment")
    return points, trusted, cumulative, length


def _project(anchor: Point, uncertainty: float, data: tuple,
             limits: AssociationLimits) -> dict:
    points, trusted, cumulative, length = data
    choices = []
    for index, start in enumerate(points):
        end = points[(index + 1) % len(points)]
        dx, dy = end[0] - start[0], end[1] - start[1]
        segment = cumulative[index + 1] - cumulative[index]
        fraction = max(0.0, min(1.0, ((anchor[0] - start[0]) * dx
                                    + (anchor[1] - start[1]) * dy) / (segment * segment)))
        projected = (start[0] + fraction * dx, start[1] + fraction * dy)
        choices.append((_distance(anchor, projected),
                        (cumulative[index] + fraction * segment) % length, projected))
    best = min(choices, key=lambda item: (item[0], item[1]))
    if best[0] + uncertainty > limits.max_projection_m:
        raise ValueError("junction_projection_too_far")
    for distance, progress, _ in choices:
        delta = abs(progress - best[1])
        separation = min(delta, length - delta)
        if (separation >= limits.remote_progress_m
                and distance <= best[0] + limits.projection_ambiguity_m + 2.0 * uncertainty):
            raise ValueError("ambiguous_reference_projection")
    half_window = max(limits.trusted_half_window_m, uncertainty)
    if length <= 2.0 * half_window:
        raise ValueError("reference_shorter_than_trust_window")
    lower, upper = best[1] - half_window, best[1] + half_window
    for index in range(len(points)):
        if trusted[index] and trusted[(index + 1) % len(points)]:
            continue
        begin, end = cumulative[index:index + 2]
        if math.ceil((lower - end) / length) <= math.floor((upper - begin) / length):
            raise ValueError("untrusted_junction_neighbourhood")
    return {"point": best[2], "progress_m": best[1], "distance_m": best[0]}


def _observations(observations: Sequence[JunctionObservation], context: ObservationContext,
                  data: tuple, limits: AssociationLimits) -> list:
    _integer(context.epoch)
    if (context.completed_lap is not True or not isinstance(context.calibration_id, str)
            or not context.calibration_id or not isinstance(context.frame_id, str)
            or not context.frame_id):
        raise ValueError("unverified_lap_identity")
    start, end = _finite(context.lap_start_time_s), _finite(context.lap_end_time_s)
    if start < 0.0 or end <= start:
        raise ValueError("invalid_lap_time_bounds")
    used_ids, used_sequences, used_stamps = set(), set(), set()
    records = []
    for observation in observations:
        identifier = observation.observation_id
        if not isinstance(identifier, str) or not identifier or identifier in used_ids:
            raise ValueError("duplicate_or_invalid_observation_id")
        used_ids.add(identifier)
        anchor = _point(observation.anchor_xy)
        bearing = _finite(observation.branch_bearing_rad)
        uncertainty = _finite(observation.position_uncertainty_m)
        angular = _finite(observation.bearing_uncertainty_rad)
        if (not 0.0 <= uncertainty <= limits.max_position_uncertainty_m
                or not 0.0 <= angular <= limits.max_bearing_uncertainty_rad):
            raise ValueError("observation_uncertainty_exceeds_contract")
        supports = list(observation.supports)
        if len(supports) < limits.min_support_frames:
            raise ValueError("insufficient_new_positive_frames")
        previous_sequence, previous_time = -1, -math.inf
        times = []
        for support in supports:
            sequence = _integer(support.sequence)
            capture, received, pose = (_finite(support.capture_time_s),
                                       _finite(support.received_time_s), _finite(support.pose_time_s))
            if (sequence <= previous_sequence or capture <= previous_time
                    or sequence in used_sequences or capture in used_stamps):
                raise ValueError("duplicate_or_out_of_order_capture")
            if (type(support.epoch) is not int or support.epoch != context.epoch
                    or support.calibration_id != context.calibration_id
                    or support.frame_id != context.frame_id):
                raise ValueError("capture_identity_mismatch")
            if (not start <= capture <= end or not 0.0 <= received - capture <= limits.max_ingress_age_s
                    or abs(pose - capture) > limits.max_pose_skew_s):
                raise ValueError("capture_pose_or_ingress_time_invalid")
            if (support.truncated is not False or _integer(support.positive_left_pixels) == 0
                    or _integer(support.positive_right_pixels) == 0):
                raise ValueError("missing_positive_unclipped_branch_boundaries")
            if (_distance(_point(support.anchor_xy), anchor) > uncertainty + _EPS
                    or abs(_wrap(_finite(support.branch_bearing_rad) - bearing)) > angular + _EPS):
                raise ValueError("inconsistent_observation_support")
            used_sequences.add(sequence)
            used_stamps.add(capture)
            previous_sequence, previous_time = sequence, capture
            times.append(capture)
        if times[-1] - times[0] > limits.max_support_span_s:
            raise ValueError("observation_support_window_too_long")
        records.append({"observation": observation, "anchor": anchor, "bearing": bearing,
                        "uncertainty": uncertainty, "angular_uncertainty": angular,
                        "start_time": times[0], "end_time": times[-1],
                        "projection": _project(anchor, uncertainty, data, limits)})
    chronological = sorted(records, key=lambda item: item["start_time"])
    if any(b["start_time"] <= a["end_time"] for a, b in zip(chronological, chronological[1:])):
        raise ValueError("overlapping_junction_tracks_not_supported")
    # Positive captures must themselves describe one ordered traversal, not a
    # geometry-sorted permutation assembled from different visits or laps.
    if any(b["observation"].supports[0].sequence <= a["observation"].supports[-1].sequence
           for a, b in zip(chronological, chronological[1:])):
        raise ValueError("global_capture_sequence_not_monotonic")
    length = data[3]
    advances = [(b["projection"]["progress_m"] - a["projection"]["progress_m"]) % length
                for a, b in zip(chronological, chronological[1:] + chronological[:1])]
    if any(delta < limits.min_anchor_separation_m for delta in advances):
        raise ValueError("non_distinct_junction_projections")
    if abs(math.fsum(advances) - length) > 1e-7 * max(1.0, length):
        raise ValueError("observed_order_is_not_one_reference_cycle")
    return sorted(records, key=lambda item: item["projection"]["progress_m"])


def _fit(source: list, target: list) -> dict:
    """Fit official -> measured by least squares IN MEASURED METRES."""

    count = len(source)
    mean_s = tuple(math.fsum(p[axis] for p in source) / count for axis in range(2))
    mean_t = tuple(math.fsum(p[axis] for p in target) / count for axis in range(2))
    centred_s = [(x - mean_s[0], y - mean_s[1]) for x, y in source]
    centred_t = [(x - mean_t[0], y - mean_t[1]) for x, y in target]
    for points in (centred_s, centred_t):
        xx, yy = math.fsum(x * x for x, _ in points), math.fsum(y * y for _, y in points)
        xy = math.fsum(x * y for x, y in points)
        if not math.isfinite(xx + yy) or xx + yy <= _EPS or xx * yy - xy * xy <= 1e-8 * (xx + yy) ** 2:
            raise ValueError("degenerate_junction_geometry")
    denominator = math.fsum(x * x + y * y for x, y in centred_s)
    a = math.fsum(x * u + y * v for (x, y), (u, v) in zip(centred_s, centred_t)) / denominator
    b = math.fsum(x * v - y * u for (x, y), (u, v) in zip(centred_s, centred_t)) / denominator
    scale = math.hypot(a, b)
    if not math.isfinite(scale) or scale <= _EPS:
        raise ValueError("degenerate_similarity")
    tx, ty = mean_t[0] - a * mean_s[0] + b * mean_s[1], mean_t[1] - b * mean_s[0] - a * mean_s[1]
    residuals = [_distance((a * x - b * y + tx, b * x + a * y + ty), point)
                 for (x, y), point in zip(source, target)]
    return {"scale": scale, "rotation_rad": math.atan2(b, a), "translation_x": tx,
            "translation_y": ty, "transform_direction": "official_xy_to_recorded_odom",
            "residuals_m": residuals, "rmse_m": math.sqrt(math.fsum(r * r for r in residuals) / count),
            "error_units": "measured_odom_metres", "allows_reflection": False}


def associate_junctions(
    observations: Sequence[JunctionObservation], official: Any, reference: Any, *,
    context: ObservationContext, official_branch_bearings: Mapping[str, float],
    original_whole_lap_status: Mapping[str, Any], enabled: bool = False,
    limits: AssociationLimits = AssociationLimits(),
) -> dict:
    """Return a separate association hypothesis; never upgrade old fusion.

    Exactly N independently observed junctions are required for N outer nodes.
    Every cyclic shift in BOTH directions is scored. Bearing is a veto, not a
    mechanism for hiding a close geometric competitor. Inputs are never edited.
    No hypothesis authorises detour entry or supplies unseen internal geometry.
    """

    result = {"available": False, "reason": "disabled", "method": "observed_junction_correspondence_v0",
              "original_whole_lap_status": copy.deepcopy(dict(original_whole_lap_status)),
              "candidates": [], "match": None, "reference_modified": False,
              "internal_connectivity_observed": False, "detour_authorized": False}
    if type(enabled) is not bool:
        result["reason"] = "enabled_must_be_boolean"
        return result
    if not enabled:
        return result
    try:
        _validate_limits(limits)
        identifiers, official_points = _official_data(official)
        if len(observations) != len(identifiers):
            raise ValueError("requires_exactly_one_observation_per_outer_junction")
        if set(official_branch_bearings) != set(identifiers):
            raise ValueError("official_branch_prior_missing_or_extra")
        priors = {key: _finite(value) for key, value in official_branch_bearings.items()}
        records = _observations(observations, context, _reference_data(reference), limits)
        uncertainty = max(record["uncertainty"] for record in records)
        count = len(records)
        for direction in (1, -1):
            for shift in range(count):
                indices = [(shift + direction * index) % count for index in range(count)]
                paired_ids = [identifiers[index] for index in indices]
                candidate = {"shift": shift, "direction": direction, "official_ids": paired_ids,
                             "rmse_m": None, "eligible": False, "rejections": []}
                try:
                    fitted = _fit([official_points[index] for index in indices],
                                  [record["anchor"] for record in records])
                    branch_errors = [abs(_wrap(priors[identifier] + fitted["rotation_rad"] - record["bearing"]))
                                     for identifier, record in zip(paired_ids, records)]
                    candidate.update({"transform": fitted, "rmse_m": fitted["rmse_m"],
                                      "branch_errors_rad": branch_errors,
                                      "branch_model": "non_outer_osm_chord_prior_not_observed_connectivity"})
                    rejections = candidate["rejections"]
                    official_per_odom = 1.0 / fitted["scale"]
                    if not limits.min_official_per_odom_scale <= official_per_odom <= limits.max_official_per_odom_scale:
                        rejections.append("scale_out_of_bounds")
                    if fitted["rmse_m"] + uncertainty > limits.max_rmse_m:
                        rejections.append("node_rmse_exceeds_limit")
                    if any(value + record["uncertainty"] > limits.max_node_residual_m
                           for value, record in zip(fitted["residuals_m"], records)):
                        rejections.append("individual_node_residual_exceeds_limit")
                    if any(error + record["angular_uncertainty"] > limits.max_branch_error_rad
                           for error, record in zip(branch_errors, records)):
                        rejections.append("branch_bearing_inconsistent")
                    candidate["eligible"] = not rejections
                except (ValueError, OverflowError) as exc:
                    candidate["rejections"].append(str(exc))
                result["candidates"].append(candidate)
        # Compare every finite geometric hypothesis, including a competitor
        # failing a chord-bearing or scale gate. We cannot turn fragile sparse
        # priors into certainty by silently deleting a nearly equal competitor.
        ranked = sorted((item for item in result["candidates"] if item["rmse_m"] is not None),
                        key=lambda item: (item["rmse_m"], item["direction"], item["shift"]))
        if len(ranked) < 2:
            raise ValueError("insufficient_nondegenerate_candidates")
        best, runner_up = ranked[:2]
        gap = runner_up["rmse_m"] - best["rmse_m"]
        required_gap = limits.ambiguity_margin_m + 2.0 * uncertainty
        result.update({"best_candidate": {"shift": best["shift"], "direction": best["direction"]},
                       "runner_up_rmse_m": runner_up["rmse_m"], "ambiguity_gap_m": gap,
                       "required_gap_m": required_gap, "candidate_count": len(result["candidates"])})
        if gap <= required_gap:
            raise ValueError("ambiguous_junction_correspondence")
        if not best["eligible"]:
            raise ValueError("best_correspondence_failed_gates")
        matches = [{"official_id": identifier, "observation_id": record["observation"].observation_id,
                    "observed_anchor_xy": record["anchor"], "reference_anchor_xy": record["projection"]["point"],
                    "reference_progress_m": record["projection"]["progress_m"],
                    "projection_distance_m": record["projection"]["distance_m"],
                    "position_uncertainty_m": record["uncertainty"]}
                   for identifier, record in zip(best["official_ids"], records)]
        result.update({"available": True, "reason": "unique_observed_junction_correspondence",
                       "match": {"direction": best["direction"], "transform": best["transform"], "nodes": matches}})
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
        result["reason"] = str(exc)
    return result
