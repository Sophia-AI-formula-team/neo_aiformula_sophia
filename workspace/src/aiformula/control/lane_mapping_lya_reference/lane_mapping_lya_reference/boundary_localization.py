"""Paint-boundary observations with the existing bounded pose solver.

The filled map is not a zero-width line. Only measured, signed physical mask
transitions constrain pose here. Current foreground evidence still supplies
the COMPLETE original overlap denominator; an unobserved side is not erased.
No sensor, motion model, pose gate or solver time budget is changed.
"""
import math
from types import MappingProxyType
import numpy as np

from .mask_localization import LaneMapLocalizer, _configuration, _rotation, _unique_cells
from .paint_geometry import BoundaryObservation, BoundaryReference


def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _endpoint_dot_uncertainty(points, vertices, delta, starts, ends):
    """Float64 arithmetic uncertainty, NOT a physical support tolerance.

    Bound coordinate subtraction and the two-product dot including cancellation.
    This only classifies an already selected chord's endpoint; its distance and
    raw projection fraction stay unchanged. Larger world coordinates widen the
    numerical uncertainty, not the sensor's claimed accuracy.
    """
    unit = np.finfo(np.float64).eps/2.
    with np.errstate(over="ignore", invalid="ignore"):
        offset = points-vertices
        error_offset = unit/(1.-unit)*(np.abs(points)+np.abs(vertices))
        error_delta = unit/(1.-unit)*(np.abs(starts)+np.abs(ends))
        products = offset*delta
        bound = np.sum(np.abs(delta)*error_offset + np.abs(offset)*error_delta
                       + error_offset*error_delta, axis=1)
        bound += (3.*unit)/(1.-3.*unit)*np.sum(np.abs(products), axis=1)
        value = np.sum(products, axis=1)
    finite = np.isfinite(bound) & np.isfinite(value)
    return value, finite & (np.abs(value) <= bound), finite


def assign_foreground_mass(points, normals, weights, observation, config):
    """First actual crossing on each side, independent of the past target.

    Intersect a foreground feature's cross-section with ALL metric pixel
    faces before checking reliability/direction. A nearer invalid boundary
    cannot be jumped over in favour of a convenient farther white stripe.
    Both half-masses remain in the caller's original denominator even when
    the corresponding transition is missing, ambiguous or uncertified.
    """
    count = len(observation.points)
    model_points, model_valid = observation.model_points, observation.model_valid
    model_reliable = observation.model_reliable
    model_normals = observation.model_normals
    mass = np.zeros(count, dtype=np.float64)
    good_metric = np.flatnonzero(observation.metric_valid)
    metrics = {"boundary_original_foreground_mass": float(np.sum(weights)),
               "boundary_missing_mass": 0., "boundary_uncertified_mass": 0.,
               "boundary_ambiguous_mass": 0.}
    if not len(good_metric):
        metrics["boundary_missing_mass"] = float(np.sum(weights))
        return mass, metrics
    segments = observation.segments[good_metric]
    edge = segments[:, 1] - segments[:, 0]
    cosine = math.cos(config["normal_max_angle_rad"])
    for start in range(0, len(points), 32):
        stop = min(len(points), start + 32)
        direction = normals[start:stop]
        delta = segments[None, :, 0] - points[start:stop, None, :]
        determinant = _cross(direction[:, None, :], edge[None, :, :])
        usable = np.abs(determinant) > 1e-12
        distance = np.full(determinant.shape, np.nan)
        along = np.full(determinant.shape, np.nan)
        np.divide(_cross(delta, edge[None, :, :]), determinant, out=distance, where=usable)
        np.divide(_cross(delta, direction[:, None, :]), determinant, out=along, where=usable)
        intersects = usable & (along >= 0.) & (along <= 1.)
        for sign in (-1., 1.):
            outward_distance = distance * sign
            allowed = intersects & (outward_distance >= 0.)
            ranked = np.where(allowed, outward_distance, np.inf)
            nearest = np.argmin(ranked, axis=1)
            best = ranked[np.arange(stop-start), nearest]
            for row, candidate in enumerate(nearest):
                half = float(weights[start+row]) * .5
                if not math.isfinite(best[row]):
                    metrics["boundary_missing_mass"] += half
                    continue
                chosen = int(good_metric[candidate])
                # A ray landing exactly on a vertex can meet two faces. Accept
                # only the same contour and compatible outward directions;
                # corners/diagonal contacts cannot arbitrarily choose a side.
                ties = good_metric[np.flatnonzero(np.abs(ranked[row]-best[row]) <= 1e-10)]
                if (len(ties) > 1 and (
                        np.any(observation.contour_ids[ties] != observation.contour_ids[chosen])
                        or np.any(model_normals[ties] @ model_normals[chosen] < cosine))):
                    metrics["boundary_ambiguous_mass"] += half
                    continue
                if (not model_reliable[chosen] or not model_valid[chosen]
                        or np.linalg.norm(model_points[chosen]) > config["max_scan_range_m"]
                        or model_normals[chosen] @ (sign*direction[row]) < cosine):
                    metrics["boundary_uncertified_mass"] += half
                    continue
                mass[chosen] += half
    metrics["boundary_assigned_mass"] = float(np.sum(mass))
    metrics["boundary_unassigned_mass"] = float(np.sum(weights)-np.sum(mass))
    return mass, metrics


def _scan_representatives(observation, mass, config):
    """Distinct thin edges, never generic XY voxel means across paint width."""
    selected = np.flatnonzero(mass > 0.)
    groups = {}
    model_points, model_normals = observation.model_points, observation.model_normals
    cosine = math.cos(config["normal_max_angle_rad"])
    for index in selected:
        key = tuple(np.floor(model_points[index]/config["scan_voxel_m"]).astype(np.int64))
        # Same contour may turn through a corner within one cell; its signed
        # direction is an additional condition, not a modulo-pi normal bin.
        bucket = groups.setdefault((key, int(observation.contour_ids[index])), [])
        for group in bucket:
            if np.all(model_normals[group] @ model_normals[index] >= cosine):
                group.append(int(index))
                break
        else:
            bucket.append([int(index)])
    representatives, weights = [], []
    for (cell, _), bucket in sorted(groups.items()):
        for group in bucket:
            centre = (np.asarray(cell)+.5)*config["scan_voxel_m"]
            distance = np.sum((model_points[group]-centre)**2, axis=1)
            representatives.append(group[int(np.argmin(distance))])
            weights.append(float(np.sum(mass[group])))
    indices = np.asarray(representatives, dtype=np.int64)
    return model_points[indices], model_normals[indices], np.asarray(weights)


class _FrozenBoundaryGraph:
    """Finite chords share confirmed nodes; no extrapolated line intersections.

    A chord is a piecewise-linear consensus model backed by repeated observed
    adjacency, not a claim that its exact straight line appeared in one image.
    Internal smooth vertices use one fixed observed normal, never a tangential
    point-to-point constraint. True ends and ambiguous junctions stay unmatched.
    """
    def __init__(self, reference, config):
        self.reference, self.config = reference, config
        self.edges = reference.directed_edges
        self.graph_primitive_count = len(self.edges)
        self.primitive_nodes = self.edges
        self.starts = reference.points_xy[self.edges[:, 0]]
        self.ends = reference.points_xy[self.edges[:, 1]]
        if reference.certified_model_segments is not None:
            # These are finite model primitives, NOT new topology edges/nodes.
            # Graph first preserves its exact-distance tie priority.
            self.starts = np.concatenate((self.starts, reference.certified_model_segments[:, 0]))
            self.ends = np.concatenate((self.ends, reference.certified_model_segments[:, 1]))
            self.primitive_nodes = np.concatenate((self.edges,
                np.repeat(reference.certified_model_owner[:, None], 2, axis=1)))
        self.delta = self.ends-self.starts
        self.length_squared = np.sum(self.delta*self.delta, axis=1)
        self.nonzero = self.length_squared > 0.
        self.normals = np.zeros_like(self.delta)
        self.normals[self.nonzero] = np.column_stack((self.delta[self.nonzero, 1],
            -self.delta[self.nonzero, 0]))/np.sqrt(self.length_squared[self.nonzero, None])
        observed = reference.normals[self.primitive_nodes]
        flip = np.sum(self.normals*np.sum(observed, axis=1), axis=1) < 0.
        self.normals[flip] *= -1.
        cosine = math.cos(config["normal_max_angle_rad"])
        self.direction_valid = self.nonzero & np.all(
            np.sum(observed*self.normals[:, None, :], axis=2) >= cosine, axis=1)
        self.lower, self.upper = np.minimum(self.starts, self.ends), np.maximum(self.starts, self.ends)
        count = len(reference.points_xy)
        incoming = np.bincount(self.edges[:, 1], minlength=count)
        outgoing = np.bincount(self.edges[:, 0], minlength=count)
        self.degree = incoming+outgoing
        self.smooth_joint = ((incoming == 1) & (outgoing == 1)
            & (reference.full_in_degree == 1) & (reference.full_out_degree == 1))
        bad = np.zeros(count, dtype=bool)
        # A node's measured normal is fixed before seeing this query. Both
        # incident chords must agree; query-dependent normal averaging would
        # allow a corner to choose whichever direction admits the observation.
        for side in (0, 1):
            np.logical_or.at(bad, self.edges[:, side], ~self.direction_valid[:self.graph_primitive_count])
        self.smooth_joint &= ~bad
        incoming_normal, outgoing_normal = np.zeros((count, 2)), np.zeros((count, 2))
        np.add.at(incoming_normal, self.edges[:, 1], self.normals[:self.graph_primitive_count])
        np.add.at(outgoing_normal, self.edges[:, 0], self.normals[:self.graph_primitive_count])
        self.smooth_joint &= np.sum(incoming_normal*outgoing_normal, axis=1) >= cosine
        # Outward-normal sign correction must not hide a directed hairpin.
        # The incoming and outgoing contour tangents must continue in the
        # same direction, using the existing angular gate (never abs(dot)).
        tangents = np.zeros_like(self.delta)
        tangents[self.nonzero] = self.delta[self.nonzero]/np.sqrt(
            self.length_squared[self.nonzero, None])
        incoming_tangent, outgoing_tangent = np.zeros((count, 2)), np.zeros((count, 2))
        np.add.at(incoming_tangent, self.edges[:, 1], tangents[:self.graph_primitive_count])
        np.add.at(outgoing_tangent, self.edges[:, 0], tangents[:self.graph_primitive_count])
        self.smooth_joint &= np.sum(incoming_tangent*outgoing_tangent, axis=1) >= cosine
        if reference.certified_model_segments is not None:
            owners = reference.certified_model_owner
            # Retain ambiguous geometry in nearest competition; rejecting it
            # must not expose a farther certificate or graph edge as fallback.
            self.direction_valid[self.graph_primitive_count:] &= (
                (reference.full_in_degree[owners] <= 1) & (reference.full_out_degree[owners] <= 1))

    def query(self, points, normals):
        count = len(points)
        owners = np.full(count, -1, dtype=np.int64)
        targets = np.zeros((count, 2))
        directions = np.zeros((count, 2))
        covered = np.zeros(count, dtype=bool)
        if not len(self.starts) or not count:
            return owners, targets, directions, covered
        config = self.config
        radius = config["match_distance_m"]
        cosine = math.cos(config["normal_max_angle_rad"])
        keys = np.floor(points/config["hash_cell_m"]).astype(np.int64)
        unique, inverse, _ = _unique_cells(keys)
        for group in range(len(unique)):
            rows = np.flatnonzero(inverse == group)
            lower, upper = np.min(points[rows], axis=0)-radius, np.max(points[rows], axis=0)+radius
            candidates = np.flatnonzero(self.nonzero & np.all(self.upper >= lower, axis=1)
                & np.all(self.lower <= upper, axis=1))
            if not len(candidates) or len(np.unique(self.primitive_nodes[candidates])) > config["max_patch_points"]:
                continue
            best_distance = np.full(len(rows), np.inf)
            best_edge = np.full(len(rows), -1, dtype=np.int64)
            best_alpha = np.zeros(len(rows))
            for start in range(0, len(candidates), 512):
                ids = candidates[start:start+512]
                offset = points[rows, None, :]-self.starts[ids][None, :, :]
                alpha = np.sum(offset*self.delta[ids][None, :, :], axis=2)/self.length_squared[ids]
                clamped = np.clip(alpha, 0., 1.)
                difference = offset-clamped[:, :, None]*self.delta[ids][None, :, :]
                squared = np.sum(difference*difference, axis=2)
                squared[normals[rows] @ self.normals[ids].T < cosine] = np.inf
                choice = np.argmin(squared, axis=1)
                values = squared[np.arange(len(rows)), choice]
                better = values < best_distance
                best_distance[better] = values[better]
                best_edge[better] = ids[choice[better]]
                best_alpha[better] = alpha[np.arange(len(rows))[better], choice[better]]
            eligible = best_distance <= radius**2
            rows, ids, alpha = rows[eligible], best_edge[eligible], best_alpha[eligible]
            if not len(rows):
                continue
            fraction = np.clip(alpha, 0., 1.)
            nodes = self.primitive_nodes[ids, (fraction >= .5).astype(np.int64)]
            graph_primitive = ids < self.graph_primitive_count
            smooth = graph_primitive & self.smooth_joint[nodes]
            owners[rows] = nodes
            targets[rows] = self.starts[ids] + fraction[:, None]*self.delta[ids]
            directions[rows] = self.normals[ids]
            covered[rows] = self.direction_valid[ids] & (alpha >= 0.) & (alpha <= 1.)
            start_dot, near_start, start_valid = _endpoint_dot_uncertainty(points[rows], self.starts[ids],
                self.delta[ids], self.starts[ids], self.ends[ids])
            end_dot, near_end, end_valid = _endpoint_dot_uncertainty(points[rows], self.ends[ids],
                self.delta[ids], self.starts[ids], self.ends[ids])
            ambiguous_ends = near_start & near_end
            at_vertex = (fraction == 0.) | (fraction == 1.) | near_start | near_end
            joint = at_vertex & smooth & ~ambiguous_ends
            if np.any(joint):
                selected_rows, selected_nodes = rows[joint], nodes[joint]
                targets[selected_rows] = self.reference.points_xy[selected_nodes]
                directions[selected_rows] = self.reference.normals[selected_nodes]
                covered[selected_rows] = np.sum(normals[selected_rows]*directions[selected_rows], axis=1) >= cosine
            # Nearest-only: a rejected terminal/ambiguous joint or direction
            # must not search for a farther segment with more convenient support.
            full_degree = self.reference.full_in_degree[nodes] + self.reference.full_out_degree[nodes]
            ambiguous_vertex = graph_primitive & at_vertex & (full_degree > 1) & ~smooth
            covered[rows[ambiguous_vertex]] = False
            # Never add support by snapping terminal extrapolation. Preserve
            # the old computed-exact-zero closed endpoint convention; it is
            # not a proof of an exact-real dot product after cancellation.
            uncertain_terminal = (~smooth
                & ((near_start & (start_dot != 0.)) | (near_end & (end_dot != 0.))))
            covered[rows[ambiguous_ends | uncertain_terminal | ~start_valid | ~end_valid]] = False
        return owners, targets, directions, covered


class FrozenBoundaryIndex:
    """Signed nearest finite mean-line geometry, with no support extrapolation."""
    def __init__(self, reference, config):
        if not isinstance(reference, BoundaryReference):
            raise ValueError("immutable physical boundary reference required")
        self.reference = reference
        self.anchor, self.normals = reference.points_xy, reference.normals
        self.line_points = self.anchor
        self.cfg = MappingProxyType(dict(config))
        self.normal_valid = np.ones(len(self.anchor), dtype=bool)
        self.normal_radii = np.zeros(len(self.anchor))
        self.normal_position_support = np.zeros(len(self.anchor), dtype=np.int64)
        self.thickness_p95_m = 0.
        self.reused_geometry_points = self.direction_supported_points = 0
        self.graph = (None if reference.directed_edges is None
                      else _FrozenBoundaryGraph(reference, self.cfg))
        if self.graph is not None:
            self.normal_valid = np.zeros(len(self.anchor), dtype=bool)
            self.normal_valid[self.graph.primitive_nodes[self.graph.direction_valid].reshape(-1)] = True
            self.normal_position_support = self.graph.degree.copy()
            if reference.certified_model_segments is not None:
                admitted = self.graph.direction_valid[self.graph.graph_primitive_count:]
                np.add.at(self.normal_position_support, reference.certified_model_owner[admitted], 1)
            return
        self.intervals = [[] for _ in self.anchor]
        for segment, owner in zip(reference.support_segments, reference.segment_owner):
            tangent = np.array([-self.normals[owner, 1], self.normals[owner, 0]])
            bounds = (segment-self.anchor[owner]) @ tangent
            self.intervals[owner].append((float(np.min(bounds)), float(np.max(bounds))))
        for index, spans in enumerate(self.intervals):
            # Union only overlapping intervals, no padding or gap filling.
            merged = []
            for lower, upper in sorted(spans):
                if merged and lower <= merged[-1][1]:
                    merged[-1] = (merged[-1][0], max(upper, merged[-1][1]))
                else:
                    merged.append((lower, upper))
            self.intervals[index] = tuple(merged)
            self.normal_position_support[index] = len(spans)
            self.normal_valid[index] = bool(merged)
        # The consensus mean and its latest observed support need not have the
        # same tangent centre. Index the FINITE geometry used by the residual,
        # not just that mean point. Otherwise a closer mean can steal a query
        # from a neighbouring, genuinely observed interval and then reject it.
        rows = [(owner, lower, upper) for owner, spans in enumerate(self.intervals)
                for lower, upper in spans]
        self._span_owner = np.asarray([row[0] for row in rows], dtype=np.int64)
        self._span_lower = np.asarray([row[1] for row in rows], dtype=np.float64)
        self._span_upper = np.asarray([row[2] for row in rows], dtype=np.float64)
        self._bounds_lower = np.full((len(self.anchor), 2), np.inf)
        self._bounds_upper = np.full((len(self.anchor), 2), -np.inf)
        if rows:
            tangent = np.column_stack((-self.normals[self._span_owner, 1],
                                        self.normals[self._span_owner, 0]))
            lower = self.anchor[self._span_owner] + tangent*self._span_lower[:, None]
            upper = self.anchor[self._span_owner] + tangent*self._span_upper[:, None]
            np.minimum.at(self._bounds_lower, self._span_owner, np.minimum(lower, upper))
            np.maximum.at(self._bounds_upper, self._span_owner, np.maximum(lower, upper))
        for array in (self._span_owner, self._span_lower, self._span_upper,
                      self._bounds_lower, self._bounds_upper):
            array.flags.writeable = False

    def nearest(self, points, normals):
        if self.graph is not None:
            return self.graph.query(points, normals)[0]
        output = np.full(len(points), -1, dtype=np.int64)
        keys = np.floor(points/self.cfg["hash_cell_m"]).astype(np.int64)
        unique, inverse, _ = _unique_cells(keys)
        radius = self.cfg["match_distance_m"]
        cosine = math.cos(self.cfg["normal_max_angle_rad"])
        for group, key in enumerate(unique):
            targets = np.flatnonzero(inverse == group)
            lower = np.min(points[targets], axis=0)-radius
            upper = np.max(points[targets], axis=0)+radius
            candidates = np.flatnonzero(self.normal_valid
                & np.all(self._bounds_upper >= lower, axis=1)
                & np.all(self._bounds_lower <= upper, axis=1))
            if len(candidates) > self.cfg["max_patch_points"]:
                # Fail closed rather than changing which competing line wins.
                continue
            if not len(candidates):
                continue
            member = np.zeros(len(self.anchor), dtype=bool)
            member[candidates] = True
            spans = np.flatnonzero(member[self._span_owner])
            best_distance = np.full(len(targets), np.inf)
            best_owner = np.full(len(targets), -1, dtype=np.int64)
            # Bounded scratch space even for a loaded reference with many
            # disconnected intervals. Stable owner/interval order breaks ties.
            for start in range(0, len(spans), 512):
                selected = spans[start:start+512]
                owners = self._span_owner[selected]
                direction = self.normals[owners]
                tangent = np.column_stack((-direction[:, 1], direction[:, 0]))
                difference = points[targets, None, :]-self.anchor[owners][None, :, :]
                along = np.sum(difference*tangent[None, :, :], axis=2)
                residual = np.sum(difference*direction[None, :, :], axis=2)
                clamped = np.clip(along, self._span_lower[selected], self._span_upper[selected])
                distances = residual**2 + (along-clamped)**2
                distances[normals[targets] @ direction.T < cosine] = np.inf
                choices = np.argmin(distances, axis=1)
                values = distances[np.arange(len(targets)), choices]
                better = values < best_distance
                best_distance[better] = values[better]
                best_owner[better] = owners[choices[better]]
            valid = best_distance <= radius**2
            output[targets[valid]] = best_owner[valid]
            # Do NOT prefer interior projections here. The nearest finite
            # object wins even when only its endpoint is reachable; covered()
            # subsequently rejects that query without trying a farther object.
        return output

    def covered(self, points, owners):
        if self.graph is not None:
            raise ValueError("graph coverage requires the complete signed query")
        tangent = np.column_stack((-self.normals[owners, 1], self.normals[owners, 0]))
        along = np.sum((points-self.anchor[owners])*tangent, axis=1)
        return np.asarray([any(a <= value <= b for a, b in self.intervals[owner])
                           for value, owner in zip(along, owners)], dtype=bool)


class BoundaryLaneLocalizer(LaneMapLocalizer):
    """Same motion/optimization/commit contracts, explicit boundary geometry."""
    def __init__(self, reference, config=None, anchor_support_xy=None,
                 anchor_support_stamp_ns=None, _reuse_index=None):
        if anchor_support_xy is not None or anchor_support_stamp_ns is not None:
            raise ValueError("boundary references cannot borrow filled paint direction")
        self.config = _configuration(config)
        self._index = FrozenBoundaryIndex(reference, self.config)
        self._anchor = reference.points_xy
        if len(self._anchor) > self.config["max_anchor_points"]:
            raise ValueError("boundary reference capacity")
        self.anchor_direction_source_stamp_ns = reference.source_frontier_ns
        self.reset()
        self._boundary_observation = None

    def match(self, *args, boundary_observation=None, **kwargs):
        self._boundary_observation = boundary_observation
        try:
            return super().match(*args, **kwargs)
        finally:
            self._boundary_observation = None

    def _prepare_scan_geometry(self, scan, support, metrics):
        observation = self._boundary_observation
        if not isinstance(observation, BoundaryObservation):
            return None, "missing_boundary_observation"
        if observation.source_stamp_ns != self.last_seen_stamp_ns:
            return None, "boundary_mask_source_mismatch"
        prepared, failure = super()._prepare_scan_geometry(scan, support, metrics)
        if failure is not None:
            return None, failure
        foreground, normals, weights, denominator = prepared
        mass, evidence = assign_foreground_mass(foreground, normals, weights, observation, self.config)
        points, directions, weights = _scan_representatives(observation, mass, self.config)
        metrics.update(evidence)
        metrics.update(line_reference=("confirmed_adjacency_mean_node_polyline_v2" if self._index.graph is not None
                                       else "confirmed_measured_signed_paint_boundary_v1"),
                       boundary_version=2 if self._index.graph is not None else 1,
                       correspondence_extent=("confirmed_polyline_chords_and_certified_internal_joints" if self._index.graph is not None
                                              else "union_of_observed_boundary_segments_no_extrapolation"),
                       overlap_denominator="complete_original_foreground_mass_before_boundary_assignment",
                       boundary_unique_scan_points=len(points))
        if self._index.reference.certified_model_segments is not None:
            metrics.update(boundary_version=3,
                line_reference="confirmed_graph_and_creation_chart_finite_models_v3",
                correspondence_extent="globally_nearest_confirmed_chords_or_certified_finite_domain",
                certified_model_representation=self._index.reference.certified_model_representation)
        if len(points) > self.config["max_scan_points"]:
            return None, "boundary_scan_capacity"
        return (points, directions, weights, denominator), None

    def _correspondences(self, local, estimate, scan_normals=None):
        rotation = _rotation(estimate[2])
        rotated = local @ rotation.T
        world = rotated + estimate[:2]
        if self._index.graph is not None:
            owners, positions, directions, covered = self._index.graph.query(world, scan_normals @ rotation.T)
            matched = owners >= 0
            selected = np.flatnonzero(matched & covered)
            self._last_correspondence_support_metrics = {
                "nearest_correspondences_before_support": int(np.sum(matched)),
                "finite_support_rejected_correspondences": int(np.sum(matched & ~covered)),
                "finite_support_covered_correspondences": len(selected)}
            normals = directions[selected]
            residual = np.sum((world[selected]-positions[selected])*normals, axis=1)
            return world, rotated, selected, normals, residual
        nearest = self._index.nearest(world, scan_normals @ rotation.T)
        selected = np.flatnonzero(nearest >= 0)
        owners = nearest[selected]
        covered = self._index.covered(world[selected], owners)
        self._last_correspondence_support_metrics = {
            "nearest_correspondences_before_support": len(selected),
            "finite_support_rejected_correspondences": int(np.sum(~covered)),
            "finite_support_covered_correspondences": int(np.sum(covered))}
        selected, owners = selected[covered], owners[covered]
        normals = self._index.normals[owners]
        residual = np.sum((world[selected]-self._anchor[owners])*normals, axis=1)
        return world, rotated, selected, normals, residual

    def matches_anchor_source(self, reference):
        if not isinstance(reference, BoundaryReference):
            return False
        original = self._index.reference
        return (reference.source_frontier_ns == original.source_frontier_ns and all(
            np.array_equal(getattr(reference, name), getattr(original, name)) for name in
            ("points_xy", "normals", "support_segments", "segment_owner", "votes",
             "reliable_votes", "record_ids", "directed_edges", "edge_frame_votes",
             "edge_reliable_votes", "edge_last_stamp_ns", "full_in_degree", "full_out_degree",
             "certified_model_segments", "certified_model_owner", "certified_model_frame_votes",
             "certified_model_reliable_votes", "certified_model_last_stamp_ns")))
