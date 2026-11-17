"""Geometric-basis hypotheses followed by continuous point-to-curve fitting.

Only current-lap outer-curve samples and official node coordinates are used.
The hash verifies a third point of a basis triangle; all remaining nodes then
vote for that similarity hypothesis. This is a bounded, specialised geometric
hashing search, not an exhaustive registration or a junction detector.
"""
from __future__ import annotations

import math
import time
from itertools import combinations

import numpy as np

from . import route_fusion as rf


MAX_SEEDS = 64
MAX_ITERATIONS = 60
MAX_SECONDS = 12.0
VERSION = "geometric_seam_v3"
MIN_BRANCH_ANGLE_DEG = 80.0
MAX_BRANCH_ANGLE_DEG = 95.0
BRANCH_ANGLE_EXEMPT_EDGES = frozenset({("10000000001", "10000000005")})
BRANCH_TANGENT_WINDOW_M = 2.0


def _loss(distances, transition):
    """The same radial Huber objective is used in every fitting stage."""
    distances = np.asarray(distances)
    return np.where(distances <= transition, distances * distances,
                    2.0 * transition * distances - transition * transition)


def _design(points):
    result = np.zeros((len(points), 2, 4))
    result[:, 0, 0], result[:, 0, 1] = points[:, 0], -points[:, 1]
    result[:, 1, 0], result[:, 1, 1] = points[:, 1], points[:, 0]
    result[:, 0, 2], result[:, 1, 3] = 1.0, 1.0
    return result


class CurveProblem:
    def __init__(self, reference, official, node_search_m, transition):
        self.reference = reference
        self.ids = list(official.node_ids[:-1])
        self.points = np.asarray(official.points[:-1], dtype=float)
        self.centre = self.points.mean(axis=0)
        self.radius = float(np.sqrt(np.mean(np.sum((self.points - self.centre) ** 2, axis=1))))
        if self.radius < 1e-8:
            raise ValueError("degenerate_outer_node_constellation")
        self.normalised = (self.points - self.centre) / self.radius
        self.design = _design(self.normalised)
        self.data = rf._projection_data(reference)
        self.search = node_search_m
        self.transition = max(transition, 1e-6)

    def parameters(self, scale, angle, translation):
        a, b = scale * math.cos(angle), scale * math.sin(angle)
        x, y = self.centre
        return np.asarray([a * self.radius, b * self.radius,
                           a * x - b * y + translation[0],
                           b * x + a * y + translation[1]])

    def transform(self, parameters):
        a, b, x, y = parameters
        scale = math.hypot(a, b) / self.radius
        angle = math.atan2(b, a)
        cx, cy = self.centre
        translation = (x - (a * cx - b * cy) / self.radius,
                       y - (b * cx + a * cy) / self.radius)
        return scale, angle, translation

    def valid_order(self, progress, scale, direction):
        return rf._progress_gaps(dict(zip(self.ids, progress)),
                                 rf._route_order(self.ids, direction),
                                 dict(zip(self.ids, self.points)), scale,
                                 self.reference.length_m, self.reference.spacing_m,
                                 self.search) is not None

    def project(self, parameters, direction):
        scale, angle, translation = self.transform(parameters)
        if not rf._MIN_MAP_TO_ODOM_SCALE <= scale <= rf._MAX_MAP_TO_ODOM_SCALE:
            return None
        predicted = self.design @ parameters
        starts, vectors, square, lengths, cumulative = self.data
        fraction = np.einsum("nki,ki->nk", predicted[:, None, :] - starts, vectors) / square
        clipped = np.clip(fraction, 0.0, 1.0)
        projected = starts + clipped[:, :, None] * vectors
        residual = predicted[:, None, :] - projected
        distances_sq = np.sum(residual * residual, axis=2)
        indices = np.argmin(distances_sq, axis=1)
        row = np.arange(len(self.ids))
        positions = projected[row, indices]
        progress = (cumulative[indices] + clipped[row, indices] * lengths[indices]) % self.reference.length_m
        distances = np.sqrt(distances_sq[row, indices])
        if not self.valid_order(progress, scale, direction):
            joint = rf._joint_project_nodes(
                self.reference, self.points, scale, angle, translation, self.data,
                direction, self.search, self.transition, candidate_count=8,
                node_only=True)
            if joint is None:
                return None
            positions, progress, distances = joint
            # Recover each selected segment for its continuous projection Jacobian.
            arc_distance = np.abs((progress[:, None] - cumulative + self.reference.length_m / 2.0)
                                  % self.reference.length_m - self.reference.length_m / 2.0)
            containing = (progress[:, None] - cumulative) % self.reference.length_m
            arc_distance = np.where(containing <= lengths + 1e-8, 0.0, arc_distance)
            indices = np.argmin(arc_distance, axis=1)
        errors = predicted - positions
        tangent = vectors[indices] / lengths[indices, None]
        raw_fraction = np.einsum("ni,ni->n", predicted - starts[indices], vectors[indices]) / square[indices]
        interior = (raw_fraction > 1e-8) & (raw_fraction < 1.0 - 1e-8)
        projector = np.tile(np.eye(2), (len(self.ids), 1, 1))
        projector[interior] -= tangent[interior, :, None] * tangent[interior, None, :]
        jacobian = projector @ self.design
        objective = float(np.mean(_loss(distances, self.transition)))
        return positions, progress, distances, errors, jacobian, objective

    def candidate(self, parameters, direction, projected):
        scale, angle, translation = self.transform(parameters)
        result = rf._candidate_from_projection(
            self.reference, self.ids, self.points, scale, angle, translation,
            direction, self.search, self.transition, *projected[:3])
        if result is not None:
            result["score"] = math.sqrt(max(0.0, projected[5]))
            result["objective_m2"] = projected[5]
        return result


class PointHash:
    """Spatial lookup for a third point predicted in a geometric pair basis."""
    def __init__(self, points, cell):
        self.points = points
        self.cell = cell
        self.bins = {}
        for index, point in enumerate(points):
            self.bins.setdefault(tuple(np.floor(point / cell).astype(int)), []).append(index)

    def supports(self, point, radius):
        key = np.floor(point / self.cell).astype(int)
        reach = int(math.ceil(radius / self.cell))
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                indices = self.bins.get((key[0] + dx, key[1] + dy), ())
                if indices and np.any(np.sum((self.points[list(indices)] - point) ** 2, axis=1) <= radius ** 2):
                    return True
        return False


def geometric_seeds(problem, deadline, limit=MAX_SEEDS):
    """Pair-defined similarities, hashed triangle support, then all-node votes."""
    step = max(2.0, problem.reference.length_m / 120.0)
    curve, _ = rf._coarse_curve(problem.reference, step)
    curve_radius = float(np.sqrt(np.mean(np.sum((curve - curve.mean(axis=0)) ** 2, axis=1))))
    scale_centre = curve_radius / problem.radius
    pairs = sorted(combinations(range(len(problem.ids)), 2),
                   key=lambda pair: -float(np.sum((problem.points[pair[1]] - problem.points[pair[0]]) ** 2)))[:3]
    lookup = PointHash(curve, step)
    target_a, target_b = np.where(~np.eye(len(curve), dtype=bool))
    target_delta = curve[target_b] - curve[target_a]
    target_distance = np.linalg.norm(target_delta, axis=1)
    raw = []
    enumerated = 0
    # This limits hypothesis generation, not accepted scale. It replaces the
    # old 0.70..1.30 radius-ratio seed grid with a wider pair-induced search.
    for first, second in pairs:
        delta = problem.points[second] - problem.points[first]
        distance = float(np.linalg.norm(delta))
        if distance < 1e-8:
            continue
        scale = target_distance / distance
        valid = ((scale >= max(rf._MIN_MAP_TO_ODOM_SCALE, 0.55 * scale_centre))
                 & (scale <= min(rf._MAX_MAP_TO_ODOM_SCALE, 1.8 * scale_centre)))
        selected = np.flatnonzero(valid)
        for offset in range(0, len(selected), 256):
            if time.monotonic() > deadline:
                return [], {"search_budget_exhausted": True, "pair_hypotheses": enumerated}
            indices = selected[offset:offset + 256]
            a = (target_delta[indices, 0] * delta[0] + target_delta[indices, 1] * delta[1]) / (distance * distance)
            b = (target_delta[indices, 1] * delta[0] - target_delta[indices, 0] * delta[1]) / (distance * distance)
            tx = curve[target_a[indices], 0] - a * problem.points[first, 0] + b * problem.points[first, 1]
            ty = curve[target_a[indices], 1] - b * problem.points[first, 0] - a * problem.points[first, 1]
            predicted = np.stack((a[:, None] * problem.points[:, 0] - b[:, None] * problem.points[:, 1] + tx[:, None],
                                  b[:, None] * problem.points[:, 0] + a[:, None] * problem.points[:, 1] + ty[:, None]), axis=2)
            nearest_sq = np.min(np.sum((predicted[:, :, None, :] - curve) ** 2, axis=3), axis=2)
            scores = np.mean(_loss(np.sqrt(nearest_sq), problem.transition), axis=1)
            enumerated += len(indices)
            for k in range(len(indices)):
                raw.append((float(scores[k]), float(a[k]), float(b[k]), float(tx[k]), float(ty[k]), first, second))
    raw.sort(key=lambda item: item[:5])
    seeds, clouds = [], []
    verified = 0
    for score, a, b, tx, ty, first, second in raw:
        if time.monotonic() > deadline:
            return seeds, {"search_budget_exhausted": True, "pair_hypotheses": enumerated}
        delta = problem.points[second] - problem.points[first]
        areas = np.abs(delta[0] * (problem.points[:, 1] - problem.points[first, 1])
                       - delta[1] * (problem.points[:, 0] - problem.points[first, 0]))
        third = int(np.argmax(areas))
        if areas[third] < 1e-8:
            continue
        cloud = np.column_stack((a * problem.points[:, 0] - b * problem.points[:, 1] + tx,
                                 b * problem.points[:, 0] + a * problem.points[:, 1] + ty))
        if not lookup.supports(cloud[third], max(2.0 * step, problem.search)):
            continue
        verified += 1
        # Seed spacing only manages compute. Final ambiguity compares every
        # refined result by mapped positions, never by this coarse grouping.
        if any(np.sqrt(np.mean(np.sum((cloud - previous) ** 2, axis=1))) < 1.5 * step for previous in clouds):
            continue
        clouds.append(cloud)
        seeds.append(problem.parameters(math.hypot(a, b), math.atan2(b, a), (tx, ty)))
        if len(seeds) >= limit:
            break
    return seeds, {"search_budget_exhausted": False, "pair_hypotheses": enumerated,
                   "triangle_verified_hypotheses": verified, "seed_count": len(seeds),
                   "coarse_spacing_m": step, "seed_radius_ratio_range": [0.55, 1.8]}


def refine(problem, initial, direction, deadline, max_iterations=MAX_ITERATIONS):
    """Monotone robust LM on continuous segment distance, with ordered lookup."""
    parameters = np.asarray(initial, dtype=float).copy()
    current = problem.project(parameters, direction)
    if current is None:
        return None
    damping = 1e-4
    history = [current[5]]
    converged, termination = False, "iteration_budget"
    last_step = None
    iteration = 0
    for iteration in range(1, max_iterations + 1):
        if time.monotonic() > deadline:
            termination = "time_budget"
            break
        distances, errors, jacobian = current[2], current[3], current[4]
        weights = np.minimum(1.0, problem.transition / np.maximum(distances, 1e-12))
        j = (jacobian * np.sqrt(weights)[:, None, None]).reshape((-1, 4))
        r = (errors * np.sqrt(weights)[:, None]).reshape(-1)
        gradient = j.T @ r / len(problem.ids)
        if float(np.linalg.norm(gradient, ord=np.inf)) < 1e-7:
            converged, termination = True, "gradient"
            break
        hessian = j.T @ j / len(problem.ids)
        accepted = False
        for _ in range(12):
            step = np.linalg.solve(hessian + damping * np.diag(np.maximum(np.diag(hessian), 1e-3)), -gradient)
            trial = problem.project(parameters + step, direction)
            if trial is not None and trial[5] < current[5] - 1e-14:
                last_step = float(np.max(np.linalg.norm(problem.design @ step, axis=1)))
                parameters += step
                improvement = current[5] - trial[5]
                current = trial
                history.append(current[5])
                damping = max(1e-9, damping / 3.0)
                accepted = True
                if last_step < 1e-5 and improvement < 1e-9:
                    converged, termination = True, "step_and_objective"
                break
            damping *= 10.0
        if converged:
            break
        if not accepted:
            # A rejected step is not proof of convergence at a segment switch.
            termination = "no_descent"
            break
    result = problem.candidate(parameters, direction, current)
    if result is not None:
        result.update(converged=converged, termination=termination,
                      iterations=iteration, last_step_m=last_step,
                      objective_history=history, candidate_source="geometric_basis_refined")
    return result


def _transform_spread(first, second, points):
    return max(math.dist(rf._mapped_point(first, point), rf._mapped_point(second, point))
               for point in points)


def _summary(candidate, best, comparison):
    return {"score_m": candidate["score"], "rmse_m": candidate["node_rmse_m"],
            "direction": candidate["direction"], "scale_map_to_odom": candidate["scale_map_to_odom"],
            "rotation_map_to_odom_rad": candidate["rotation_map_to_odom_rad"],
            "translation_map_to_odom": list(candidate["translation_map_to_odom"]),
            "source": candidate.get("candidate_source"), "converged": candidate.get("converged"),
            "iterations": candidate.get("iterations"), "termination": candidate.get("termination"),
            "global_disagreement_m": _transform_spread(best, candidate, comparison),
            "node_positions_odom": {key: value.tolist() for key, value in candidate["node_positions"].items()},
            "node_progress_m": dict(candidate["node_progress"])}


def angle_in_range(angle, minimum, maximum):
    return minimum - 1e-9 <= angle <= maximum + 1e-9


def _pure_closure_segments(reference):
    """Find complete untrusted runs caused solely by the closure gap."""
    cached = getattr(reference, "_pure_closure_segments", None)
    if cached is not None:
        return cached
    n = len(reference.points)
    bad = [not (reference.trusted[i] and reference.trusted[(i + 1) % n])
           for i in range(n)]
    pure = set()
    if any(bad) and not all(bad):
        anchor = bad.index(False)
        position = 1
        while position < n:
            index = (anchor + position) % n
            if not bad[index]:
                position += 1
                continue
            run = []
            while position < n and bad[(anchor + position) % n]:
                run.append((anchor + position) % n)
                position += 1
            endpoints = set(run) | {(i + 1) % n for i in run}
            if (any(reference.closure_mask[i] for i in endpoints)
                    and not any(reference.excluded_mask[i] for i in endpoints)):
                pure.update(run)
    reference._pure_closure_segments = pure
    return pure


def _branch_tangent(reference, progress, window):
    """Use a full tangent, or a contiguous trusted side of a pure seam.

    OA and other excluded data are never admitted through the seam fallback.
    The candidate itself and its chosen side must be trusted.
    """
    if reference._interval_trusted(progress - window, progress + window):
        before = reference.point_at(progress - window)
        after = reference.point_at(progress + window)
        return np.asarray(after[:2]) - np.asarray(before[:2]), "two_sided"
    if not hasattr(reference, "closure_mask") or not hasattr(reference, "excluded_mask"):
        return None, None
    if not reference.point_at(progress)[2]:
        return None, None
    start, end = progress - window, progress + window
    seam_seen = False
    size = len(reference.points)
    pure_segments = _pure_closure_segments(reference)
    for index in range(size):
        following = (index + 1) % size
        if reference.trusted[index] and reference.trusted[following]:
            continue
        begin, finish = reference.cumulative_m[index:index + 2]
        first_lap = math.ceil((start - finish) / reference.length_m)
        last_lap = math.floor((end - begin) / reference.length_m)
        if first_lap > last_lap:
            continue
        if index not in pure_segments:
            return None, None
        seam_seen = True
    if not seam_seen:
        return None, None
    # _interval_trusted treats a segment touching an endpoint as intersecting.
    # Probe just inside the chosen side, never the gap, to avoid rejecting a
    # trusted candidate that lies exactly at the boundary.
    epsilon = min(reference.spacing_m * 0.05, window * 0.01)
    if reference._interval_trusted(progress + epsilon, progress + window):
        before = np.asarray(reference.point_at(progress + epsilon)[:2])
        return np.asarray(reference.point_at(progress + window)[:2]) - before, "seam_forward"
    if reference._interval_trusted(progress - window, progress - epsilon):
        after = np.asarray(reference.point_at(progress - epsilon)[:2])
        return after - np.asarray(reference.point_at(progress - window)[:2]), "seam_backward"
    return None, None


def branch_angles(candidate, reference, branch_connections):
    """Evaluate every provided official inner chord against the local tangent.

    A missing/untrusted full tangent interval yields None, never an implicit
    pass. The chord only approximates the local branch direction.
    """
    result = []
    for ident, neighbours in branch_connections.items():
        neighbours = [(inner, point) for inner, point in neighbours
                      if (ident, inner) not in BRANCH_ANGLE_EXEMPT_EDGES]
        if not neighbours:
            continue
        progress = candidate["node_progress"][ident]
        window = BRANCH_TANGENT_WINDOW_M
        tangent, tangent_source = _branch_tangent(reference, progress, window)
        if tangent is None:
            return None
        # Normalize direction to official outer-way order, independent of
        # whether this lap was driven in the reverse direction.
        tangent = candidate["direction"] * tangent
        point = np.asarray(candidate["node_positions"][ident])
        for neighbour, official_point in neighbours:
            branch = point - np.asarray(rf._mapped_point(candidate, official_point))
            denom = float(np.linalg.norm(tangent) * np.linalg.norm(branch))
            if not math.isfinite(denom) or denom <= 1e-9:
                return None
            cosine = max(-1.0, min(1.0, float(np.dot(tangent, branch)) / denom))
            result.append({"outer_node_id": ident, "inner_node_id": neighbour,
                           "angle_deg": math.degrees(math.acos(cosine)),
                           "tangent_source": tangent_source})
    return result or None


def match_outer_nodes(reference, official, node_search_m, feature_window_m,
                      max_rmse_m, ambiguity_margin_m, closure_gap,
                      comparison_points=(), ambiguity_position_m=1.0, branch_connections=None,
                      branch_angle_min_deg=MIN_BRANCH_ANGLE_DEG, branch_angle_max_deg=MAX_BRANCH_ANGLE_DEG):
    start = time.monotonic()
    deadline = start + MAX_SECONDS
    problem = CurveProblem(reference, official, node_search_m, feature_window_m)
    if not isinstance(branch_connections, dict) or not branch_connections:
        return {"available": False, "reason": "branch_connections_missing", "matching_method": VERSION}, {}, "branch_connections_missing"
    try:
        cleaned_connections = {}
        for ident, neighbours in branch_connections.items():
            if ident not in problem.ids or not isinstance(neighbours, (tuple, list)) or not neighbours:
                raise ValueError("invalid outer branch")
            cleaned_connections[ident] = []
            for neighbour, point in neighbours:
                if not isinstance(neighbour, str) or not neighbour or neighbour in problem.ids or len(point) != 2 or not all(math.isfinite(float(v)) for v in point):
                    raise ValueError("invalid inner branch")
                cleaned_connections[ident].append((neighbour, tuple(float(v) for v in point)))
        branch_connections = cleaned_connections
    except (TypeError, ValueError, OverflowError):
        return {"available": False, "reason": "branch_connections_invalid", "matching_method": VERSION}, {}, "branch_connections_invalid"
    area = rf._signed_area(problem.points)
    track_area = rf._signed_area(np.asarray(reference.points))
    if abs(area) < 1e-8 or abs(track_area) < 1e-8:
        return None, {}, "closed_loop_orientation_ambiguous"
    direction = 1 if area * track_area > 0.0 else -1
    seeds, diagnostics = geometric_seeds(problem, deadline)
    refined = []
    for initial in seeds:
        for candidate_direction in (direction, -direction):
            if time.monotonic() > deadline:
                diagnostics["search_budget_exhausted"] = True
                break
            candidate = refine(problem, initial, candidate_direction, deadline)
            if candidate is not None:
                refined.append(candidate)
                if candidate["termination"] == "time_budget":
                    diagnostics["search_budget_exhausted"] = True
                    break
        if diagnostics["search_budget_exhausted"]:
            break
    if not refined:
        diagnostics.update(available=False, reason="solver_budget_exhausted" if diagnostics["search_budget_exhausted"] else "no_geometric_candidate",
                           matching_method=VERSION, search_exhaustive=False, solver_elapsed_sec=time.monotonic()-start)
        return diagnostics, {}, diagnostics["reason"]
    refined.sort(key=lambda item: (item["score"], not item["converged"]))
    def eligible(candidate):
        return (candidate["node_rmse_m"] <= max_rmse_m
                and candidate["node_max_distance_m"] <= node_search_m
                and candidate["edge_angle_max_rad"] <= math.radians(60.0))
    angular_pass, angular_unknown = [], []
    for candidate in refined:
        if time.monotonic() > deadline:
            diagnostics["search_budget_exhausted"] = True
            break
        if not eligible(candidate):
            continue
        angles = branch_angles(candidate, reference, branch_connections)
        if angles is None:
            angular_unknown.append(candidate)
        else:
            candidate["branch_angles"] = angles
            if all(angle_in_range(item["angle_deg"], branch_angle_min_deg, branch_angle_max_deg) for item in angles):
                angular_pass.append(candidate)
    if not angular_pass:
        reason = ("solver_budget_exhausted" if diagnostics["search_budget_exhausted"] else
                  "branch_angle_unavailable" if angular_unknown else "branch_angle_no_candidate")
        diagnostics.update(available=False, reason=reason, matching_method=VERSION,
                           branch_angle_threshold_deg=branch_angle_min_deg,
                           branch_angle_max_deg=branch_angle_max_deg,
                           branch_angle_exempt_edges=[list(x) for x in sorted(BRANCH_ANGLE_EXEMPT_EDGES)],
                           branch_angle_unknown_count=len(angular_unknown), search_exhaustive=False,
                           solver_elapsed_sec=time.monotonic()-start)
        return diagnostics, {}, reason
    # refined is already ordered by the original point-to-curve objective.
    best = angular_pass[0]
    points = list(problem.points) + list(comparison_points)
    near = [item for item in angular_pass if item["score"] <= best["score"] + ambiguity_margin_m + 1e-12]
    global_alternative = next((item for item in near if item is not best and
                               (item["direction"] != best["direction"] or
                                _transform_spread(best, item, points) > ambiguity_position_m)), None)
    # Evaluate local correspondence precision without reclassifying a fixed
    # transform probe as an independently converged global placement.
    local_probes = []
    for index, identifier in enumerate(problem.ids):
        if time.monotonic() > deadline:
            diagnostics["search_budget_exhausted"] = True
            break
        joint = rf._joint_project_nodes(reference, problem.points, best["scale_map_to_odom"],
                                       best["rotation_map_to_odom_rad"], best["translation_map_to_odom"],
                                       problem.data, best["direction"], node_search_m, feature_window_m,
                                       candidate_count=8, forbidden_node=index,
                                       forbidden_progress=best["node_progress"][identifier],
                                       forbidden_radius_m=ambiguity_position_m, node_only=True)
        if joint is None:
            continue
        candidate = rf._candidate_from_projection(reference, problem.ids, problem.points,
                                                   best["scale_map_to_odom"], best["rotation_map_to_odom_rad"],
                                                   best["translation_map_to_odom"], best["direction"],
                                                   node_search_m, feature_window_m, *joint)
        if candidate is not None:
            candidate["score"] = rf._huber_rms(joint[2], feature_window_m)
            candidate["candidate_source"] = "fixed_transform_probe"
            local_probes.append(candidate)
    near_local = [item for item in local_probes if eligible(item) and item["score"] <= best["score"] + ambiguity_margin_m + 1e-12]
    local_alternative = next((item for item in sorted(near_local, key=lambda item: item["score"])
                              if rf._candidate_disagreement(best, item, comparison_points) > ambiguity_position_m), None)
    # This release follows the requested lowest-score selection. It requires
    # the selected candidate to converge; other competitive unfinished seeds
    # are reported separately rather than treated as an ambiguity rejection.
    unresolved = [best] if not best["converged"] else []
    unknown_better = [item for item in angular_unknown if item["score"] < best["score"] - 1e-12]
    alternative = global_alternative or local_alternative
    alignment = rf._to_alignment(best, reference, closure_gap, official, alternative,
                                 max_rmse_m, ambiguity_margin_m, node_search_m)
    quality = eligible(best)
    if not quality:
        reason = alignment["reason"]
    elif diagnostics["search_budget_exhausted"]:
        reason = "solver_budget_exhausted"
    elif unresolved:
        reason = "solver_not_converged"
    elif unknown_better:
        reason = "branch_angle_unavailable"
    else:
        reason = "accepted"
    envelope = {}
    for identifier in problem.ids:
        envelope[identifier] = max([0.0] + [rf._phase_distance(item["node_progress"][identifier], best["node_progress"][identifier], reference.length_m)
                                             for item in near + near_local])
    diagnostic_candidates = [best]
    for candidate in [global_alternative, local_alternative] + near + refined:
        if candidate is not None and all(candidate is not item for item in diagnostic_candidates):
            diagnostic_candidates.append(candidate)
        if len(diagnostic_candidates) >= 6:
            break
    alignment.update(diagnostics)
    alignment.update(available=reason == "accepted", reason=reason, matching_method=VERSION,
                     objective="radial_huber_point_to_curve_rms", search_exhaustive=False,
                     solver_elapsed_sec=time.monotonic()-start, solver_max_seconds=MAX_SECONDS,
                     best_converged=best["converged"], best_iterations=best["iterations"],
                     best_termination=best["termination"], best_last_step_m=best["last_step_m"],
                     best_objective_monotone=all(b <= a + 1e-12 for a, b in zip(best["objective_history"], best["objective_history"][1:])),
                     unresolved_candidate_count=len(unresolved),
                     convergence_check_scope="selected_candidate_only",
                     competitive_nonconverged_count=sum(not item["converged"] for item in near),
                     branch_angle_threshold_deg=branch_angle_min_deg,
                     branch_angle_max_deg=branch_angle_max_deg,
                     branch_angle_exempt_edges=[list(x) for x in sorted(BRANCH_ANGLE_EXEMPT_EDGES)],
                     branch_angle_vector="inner_to_outer_vs_odom_tangent_along_official_way_order",
                     branch_tangent_window_m=BRANCH_TANGENT_WINDOW_M,
                     branch_angles=best["branch_angles"],
                     branch_angle_pass_count=len(angular_pass),
                     branch_angle_unknown_count=len(angular_unknown),
                     branch_angle_unknown_better_count=len(unknown_better),
                     candidate_gap_enforced=False,
                     selection_policy="lowest_point_curve_score_after_branch_angle_screen",
                     branch_angle_assumption="official junction-to-interior chord approximates local branch direction; field validation required",
                     global_transform_available=quality and not diagnostics["search_budget_exhausted"] and not unresolved and global_alternative is None,
                     node_correspondence_available=quality and not diagnostics["search_budget_exhausted"] and not unresolved and global_alternative is None and local_alternative is None,
                     node_progress_spread_m=envelope, uncertainty_kind="sampled_candidate_envelope_not_probability",
                     candidate_count=len(refined)+len(local_probes), comparison_point_count=len(comparison_points),
                     same_transform_alternatives_checked=True, ambiguity_position_m=ambiguity_position_m,
                     alternative_node_spread_m=rf._candidate_disagreement(best, alternative, comparison_points) if alternative is not None else None,
                     diagnostic_candidates=[_summary(item, best, points) for item in diagnostic_candidates])
    return alignment, dict(best["node_progress"]), reason
