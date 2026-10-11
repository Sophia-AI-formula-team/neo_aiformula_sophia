"""Observed semantic paint boundaries, separate from the filled-mask map.

A half-pixel face is a quantized white/black transition, NOT recovered subpixel
paint truth. Image edges, projection failures and sensitivity cuts never create
faces. Direction is fitted along the directed pixel contour, not a spatial PCA
which can mix opposite sides of a thin stripe. No ROS, cv2, pose truth or history
is consumed by extraction. Callers own image/motion arrival-time admission.
"""

from dataclasses import dataclass
import math
from numbers import Integral, Real
from types import MappingProxyType

import numpy as np


REPRESENTATION = "paint_boundary_v1"
BOUNDARY_VERSION = 2
TOPOLOGY_REPRESENTATION = "confirmed_contour_node_chords_v1"
DEFAULT_CONFIG = {
    "normal_radius_m": .45, "normal_max_radius_m": 1.35,
    "normal_min_points": 6, "normal_max_variance_ratio": .15,
    "normal_max_thickness_m": .30, "max_patch_points": 512,
    "reliable_sensitivity": .1, "max_input_edges": 200000,
}
REASON_METRIC = 1
REASON_SENSITIVITY = 2
REASON_DIRECTION = 4
REASON_J4_UNOBSERVED = 8


def _integer(value, name, minimum=0, maximum=2 ** 63 - 1):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral)
            or not minimum <= int(value) <= maximum):
        raise ValueError("invalid " + name)
    return int(value)


def _positive(value, name, zero=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError("invalid " + name)
    result = float(value)
    if not math.isfinite(result) or (result < 0 if zero else result <= 0):
        raise ValueError("invalid " + name)
    return result


def _frozen(value, dtype, shape, name, finite=True):
    raw = np.asarray(value)
    if dtype == np.int64 and raw.dtype.kind not in "iu":
        # Explicit empty lists are useful for an empty reference.
        if raw.size:
            raise ValueError(name + " must contain integers")
    if dtype == np.bool_ and raw.dtype.kind != "b":
        if raw.size:
            raise ValueError(name + " must contain booleans")
    array = np.asarray(raw, dtype=dtype)
    if array.shape != shape or (finite and not np.all(np.isfinite(array))):
        raise ValueError("invalid " + name)
    return np.frombuffer(array.tobytes(), dtype=dtype).reshape(shape)


@dataclass(frozen=True)
class BoundaryObservation:
    points: np.ndarray
    normals: np.ndarray
    segments: np.ndarray
    sensitivity: np.ndarray
    metric_valid: np.ndarray
    stable: np.ndarray
    normal_valid: np.ndarray
    valid: np.ndarray
    reliable: np.ndarray
    source_stamp_ns: int
    pixel_segments: np.ndarray
    white_pixels: np.ndarray
    black_pixels: np.ndarray
    previous: np.ndarray
    next: np.ndarray
    contour_ids: np.ndarray
    reason_codes: np.ndarray
    normal_support_count: np.ndarray
    normal_radius: np.ndarray
    position_estimates: object = None
    position_valid: object = None
    position_sensitivity: object = None
    position_reliable: object = None
    position_normals: object = None

    def __post_init__(self):
        count = len(np.asarray(self.points))
        if count > 200000:
            raise ValueError("boundary observation capacity")
        object.__setattr__(self, "source_stamp_ns", _integer(self.source_stamp_ns, "source stamp", 1))
        floats = {"points": (count, 2), "normals": (count, 2),
                  "segments": (count, 2, 2), "sensitivity": (count,),
                  "pixel_segments": (count, 2, 2), "normal_radius": (count,)}
        for name, shape in floats.items():
            object.__setattr__(self, name, _frozen(getattr(self, name), np.float64, shape, name,
                                                name not in {"points", "segments", "sensitivity"}))
        for name in ("metric_valid", "stable", "normal_valid", "valid", "reliable"):
            object.__setattr__(self, name, _frozen(getattr(self, name), np.bool_, (count,), name))
        for name in ("white_pixels", "black_pixels", "previous", "next", "contour_ids",
                     "reason_codes", "normal_support_count"):
            shape = (count, 2) if name.endswith("pixels") else (count,)
            object.__setattr__(self, name, _frozen(getattr(self, name), np.int64, shape, name))
        if (np.any(self.previous < -1) or np.any(self.previous >= count)
                or np.any(self.next < -1) or np.any(self.next >= count)
                or np.any(self.contour_ids < 0) or np.any(self.reason_codes < 0)
                or np.any(self.white_pixels < 0) or np.any(self.black_pixels < 0)
                or np.any(self.normal_support_count < 0) or np.any(self.normal_radius < 0)
                or not np.array_equal(self.valid, self.stable & self.normal_valid)
                or np.any(self.stable & ~self.metric_valid) or np.any(self.reliable & ~self.valid)):
            raise ValueError("inconsistent boundary validity/topology")
        finite = np.all(np.isfinite(self.points), axis=1) & np.all(np.isfinite(self.segments), axis=(1, 2))
        if (np.any(self.metric_valid & ~finite) or np.any(self.stable & ~np.isfinite(self.sensitivity))
                or np.any(np.isfinite(self.sensitivity) & (self.sensitivity < 0))):
            raise ValueError("invalid metric boundary evidence")
        if np.any(np.abs(np.linalg.norm(self.normals[self.normal_valid], axis=1) - 1.) > 1.e-6):
            raise ValueError("valid boundary normals must be unit vectors")
        for side, reverse in ((self.next, self.previous), (self.previous, self.next)):
            slots = np.flatnonzero(side >= 0)
            if not np.array_equal(reverse[side[slots]], slots):
                raise ValueError("nonreciprocal contour links")
            if np.any(self.contour_ids[slots] != self.contour_ids[side[slots]]):
                raise ValueError("contour id mismatch")
        model_fields = (self.position_estimates, self.position_valid, self.position_sensitivity,
                        self.position_reliable, self.position_normals)
        if any(value is not None for value in model_fields):
            if not all(value is not None for value in model_fields):
                raise ValueError("boundary position model requires all arrays")
            for name, dtype, shape in (("position_estimates", np.float64, (count, 2)),
                    ("position_valid", np.bool_, (count,)), ("position_sensitivity", np.float64, (count,)),
                    ("position_reliable", np.bool_, (count,)), ("position_normals", np.float64, (count, 2))):
                object.__setattr__(self, name, _frozen(getattr(self, name), dtype, shape, name,
                                                    finite=dtype == np.bool_))
            if (np.any(self.position_valid & ~self.valid)
                    or np.any(self.position_valid & ~np.all(np.isfinite(self.position_estimates), axis=1))
                    or np.any(self.position_valid & ~np.isfinite(self.position_sensitivity))
                    or np.any(np.isfinite(self.position_sensitivity) & (self.position_sensitivity < 0))
                    or np.any(self.position_reliable & ~self.position_valid)
                    or np.any(~np.isfinite(self.position_normals[self.position_valid]))
                    or np.any(np.abs(np.linalg.norm(self.position_normals[self.position_valid], axis=1)-1.) > 1.e-6)):
                raise ValueError("invalid quantization-constrained position evidence")

    @property
    def model_points(self):
        return self.points if self.position_estimates is None else self.position_estimates

    @property
    def model_valid(self):
        return self.valid if self.position_valid is None else self.valid & self.position_valid

    @property
    def model_sensitivity(self):
        return self.sensitivity if self.position_sensitivity is None else np.maximum(
            self.sensitivity, self.position_sensitivity)

    @property
    def model_reliable(self):
        return self.reliable if self.position_reliable is None else self.reliable & self.position_reliable

    @property
    def model_normals(self):
        return self.normals if self.position_normals is None else self.position_normals

    @property
    def points_xy(self):
        return self.points

    def transformed(self, pose_xyyaw):
        pose = np.asarray(pose_xyyaw, dtype=np.float64)
        if pose.shape != (3,) or not np.all(np.isfinite(pose)):
            raise ValueError("invalid boundary transform")
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rotation = np.array([[c, -s], [s, c]])
        values = dict(self.__dict__)
        with np.errstate(over="raise", invalid="ignore"):
            values["points"] = self.points @ rotation.T + pose[:2]
            values["normals"] = self.normals @ rotation.T
            values["segments"] = self.segments @ rotation.T + pose[:2]
            if self.position_estimates is not None:
                values["position_estimates"] = self.position_estimates @ rotation.T + pose[:2]
                values["position_normals"] = self.position_normals @ rotation.T
        return BoundaryObservation(**values)


@dataclass(frozen=True)
class BoundaryReference:
    points_xy: np.ndarray
    normals: np.ndarray
    support_segments: np.ndarray
    segment_owner: np.ndarray
    votes: np.ndarray
    reliable_votes: np.ndarray
    record_ids: np.ndarray
    source_frontier_ns: object
    directed_edges: object = None
    edge_frame_votes: object = None
    edge_reliable_votes: object = None
    edge_last_stamp_ns: object = None
    full_in_degree: object = None
    full_out_degree: object = None

    def __post_init__(self):
        n, m = len(np.asarray(self.points_xy)), len(np.asarray(self.support_segments))
        if n > 100000 or m > 200000:
            raise ValueError("boundary reference capacity")
        for name, dtype, shape in (
                ("points_xy", np.float64, (n, 2)), ("normals", np.float64, (n, 2)),
                ("support_segments", np.float64, (m, 2, 2)),
                ("segment_owner", np.int64, (m,)), ("votes", np.int64, (n,)),
                ("reliable_votes", np.int64, (n,)), ("record_ids", np.int64, (n,))):
            object.__setattr__(self, name, _frozen(getattr(self, name), dtype, shape, name))
        if self.source_frontier_ns is not None:
            object.__setattr__(self, "source_frontier_ns", _integer(self.source_frontier_ns, "reference frontier", 1))
        if ((n or m) and self.source_frontier_ns is None
                or np.any(self.segment_owner < 0) or np.any(self.segment_owner >= n)
                or np.any(self.votes < 1) or np.any(self.reliable_votes < 0)
                or np.any(self.reliable_votes > self.votes) or np.any(self.record_ids < 0)
                or len(np.unique(self.record_ids)) != n
                or np.any(np.abs(np.linalg.norm(self.normals, axis=1) - 1.) > 1.e-6)):
            raise ValueError("inconsistent boundary reference")
        graph_names = ("directed_edges", "edge_frame_votes", "edge_reliable_votes", "edge_last_stamp_ns")
        supplied = [getattr(self, name) is not None for name in graph_names]
        if any(supplied) and not all(supplied):
            raise ValueError("boundary graph requires all four arrays")
        degree_names = ("full_in_degree", "full_out_degree")
        degree_supplied = [getattr(self, name) is not None for name in degree_names]
        if any(degree_supplied) and (not all(degree_supplied) or not all(supplied)):
            raise ValueError("boundary full degrees require an explicit graph and both arrays")
        if all(supplied):
            raw_edges = np.asarray(self.directed_edges)
            if raw_edges.ndim != 2 or raw_edges.shape[1] != 2:
                raise ValueError("invalid directed_edges")
            k = len(raw_edges)
            if k > 200000:
                raise ValueError("boundary graph capacity")
            for name in graph_names:
                shape = (k, 2) if name == "directed_edges" else (k,)
                object.__setattr__(self, name, _frozen(getattr(self, name), np.int64, shape, name))
            edges = self.directed_edges
            if (np.any(edges < 0) or np.any(edges >= n)
                    or np.any(edges[:, 0] == edges[:, 1])
                    or len(set(map(tuple, edges))) != k
                    or np.any(self.edge_frame_votes < 1) or np.any(self.edge_reliable_votes < 0)
                    or np.any(self.edge_reliable_votes > self.edge_frame_votes)
                    or np.any(self.edge_last_stamp_ns <= 0)
                    or (k and self.source_frontier_ns is None)
                    or (k and np.any(self.edge_last_stamp_ns > self.source_frontier_ns))):
                raise ValueError("inconsistent boundary graph")
            if (k and (np.any(self.edge_frame_votes[:, None] > self.votes[edges])
                       or np.any(self.edge_reliable_votes[:, None] > self.reliable_votes[edges]))):
                raise ValueError("boundary edge votes exceed endpoint evidence")
            local_degrees = (np.bincount(edges[:, 1], minlength=n),
                             np.bincount(edges[:, 0], minlength=n))
            for name, local in zip(degree_names, local_degrees):
                value = getattr(self, name) if all(degree_supplied) else local
                value = _frozen(value, np.int64, (n,), name)
                if np.any(value < local) or np.any(value > 200000):
                    raise ValueError("inconsistent boundary full degree")
                object.__setattr__(self, name, value)

    @property
    def representation(self):
        return REPRESENTATION

    @property
    def topology_representation(self):
        return None if self.directed_edges is None else TOPOLOGY_REPRESENTATION

    def subset(self, indices):
        raw = np.asarray(indices)
        if raw.ndim != 1 or (raw.size and raw.dtype.kind not in "iu"):
            raise ValueError("boundary subset indices must be integers")
        ids = np.asarray(raw, dtype=np.int64)
        if (np.any(ids < 0) or np.any(ids >= len(self.points_xy))
                or len(np.unique(ids)) != len(ids)):
            raise ValueError("invalid boundary subset indices")
        remap = np.full(len(self.points_xy), -1, dtype=np.int64)
        remap[ids] = np.arange(len(ids))
        owners = remap[self.segment_owner]
        keep = owners >= 0
        graph = {}
        if self.directed_edges is not None:
            edges = remap[self.directed_edges]
            retain = np.all(edges >= 0, axis=1)
            graph = dict(directed_edges=edges[retain], edge_frame_votes=self.edge_frame_votes[retain],
                         edge_reliable_votes=self.edge_reliable_votes[retain],
                         edge_last_stamp_ns=self.edge_last_stamp_ns[retain],
                         full_in_degree=self.full_in_degree[ids],
                         full_out_degree=self.full_out_degree[ids])
        return BoundaryReference(self.points_xy[ids], self.normals[ids], self.support_segments[keep],
                                 owners[keep], self.votes[ids], self.reliable_votes[ids],
                                 self.record_ids[ids], self.source_frontier_ns, **graph)


CONSENSUS_DEFAULT_CONFIG = {
    "grid_resolution_m": .1, "minimum_frame_votes": 5, "minimum_reliable_votes": 1,
    "max_sensitivity": .5, "reliable_sensitivity": .1,
    "max_candidate_cells": 200000, "max_confirmed_cells": 100000,
    "candidate_ttl_s": 30., "normal_max_angle_rad": math.pi / 6.,
    "max_support_segments": 200000,
}


@dataclass(frozen=True)
class _BoundaryRecord:
    record_id: int
    cell: tuple
    point: tuple
    normal: tuple
    votes: int
    reliable_votes: int
    last_stamp_ns: int
    segments: np.ndarray
    neighbours: np.ndarray


@dataclass(frozen=True)
class _BoundaryEdge:
    source_id: int
    target_id: int
    votes: int
    reliable_votes: int
    last_stamp_ns: int


@dataclass(frozen=True)
class PreparedBoundaryUpdate:
    """Guard-ready immutable delta, not a second publicly mutable map."""
    source_stamp_ns: int
    base_generation: int
    owner: object
    updates: tuple
    expired_ids: tuple
    next_record_id: int
    segment_count: int
    edge_updates: tuple
    expired_edge_keys: tuple
    stats: object


def _contiguous_cell_runs(obs, eligible, cells, cosine):
    """Partition actual directed contours, never collect a cell's re-entry.

    Every face in a run is signed-compatible with every earlier face in that
    run. This avoids fixed angle-sector discontinuities without combining two
    oppositely directed sides. Invalid faces break runs even inside one cell.
    """
    admitted = np.zeros(len(obs.points), dtype=bool)
    admitted[eligible] = True
    all_cells = np.zeros((len(obs.points), 2), dtype=np.int64)
    all_cells[eligible] = cells
    visited = np.zeros(len(obs.points), dtype=bool)
    output = []
    normals = obs.model_normals
    for seed in list(np.flatnonzero(obs.previous < 0)) + list(range(len(obs.points))):
        if visited[seed]:
            continue
        path, current = [], int(seed)
        while current >= 0 and not visited[current]:
            visited[current] = True
            path.append(current)
            current = int(obs.next[current])
        parts, run = [], []
        for face in path:
            same = (bool(run) and admitted[face]
                    and all_cells[face, 0] == all_cells[run[0], 0]
                    and all_cells[face, 1] == all_cells[run[0], 1]
                    and np.all(normals[run] @ normals[face] >= cosine))
            if run and not same:
                parts.append(run)
                run = []
            if admitted[face]:
                run.append(face)
        if run:
            parts.append(run)
        # A closed contour's arbitrary iteration start is not a physical cut.
        # Merge across it only if the original faces are genuinely adjacent.
        if (len(parts) > 1 and obs.next[parts[-1][-1]] == parts[0][0]
                and all_cells[parts[-1][0], 0] == all_cells[parts[0][0], 0]
                and all_cells[parts[-1][0], 1] == all_cells[parts[0][0], 1]
                and np.all(normals[parts[-1]] @ normals[parts[0]].T >= cosine)):
            parts[0] = parts[-1] + parts[0]
            parts.pop()
        for faces in parts:
            first = faces[0]
            output.append((tuple(map(int, all_cells[first])), int(obs.contour_ids[first]), faces))
    return sorted(output, key=lambda item: (item[0], item[1], item[2][0]))


def _group_frame_means(points, normals, member_groups):
    """Frame means in original group/member order, without normalizing.

    The two coordinate columns use the same ordered additions as the original
    per-group ``np.mean(points[faces], axis=0)``. Keep a positive-zero initial
    accumulator, including for singleton and signed-zero groups. Membership
    has already been established by the directed contour walk.
    """
    count = len(member_groups)
    if not count:
        return np.empty((0, 2)), np.empty((0, 2))
    sizes = np.fromiter((len(group) for group in member_groups), dtype=np.int64, count=count)
    members = np.concatenate(member_groups).astype(np.int64, copy=False)
    owners = np.repeat(np.arange(count), sizes)
    point_means, normal_means = np.zeros((count, 2)), np.zeros((count, 2))
    np.add.at(point_means, owners, points[members])
    np.add.at(normal_means, owners, normals[members])
    point_means /= sizes[:, None]
    normal_means /= sizes[:, None]
    return point_means, normal_means


class BoundaryConsensus:
    """One vote per directed record per frame; filled consensus is untouched.

    Spatial cells are buckets, not unique keys: opposite normals and distinct
    same-frame contours can occupy the same cell. Only unambiguous same-cell,
    signed-direction associations can advance an old record. A rejected prepare
    or an unused token cannot advance votes, geometry, expiry or source frontier.

    Records come from contiguous cell-runs, never disconnected re-entries.
    Directed record links require their OWN per-frame confirmation evidence.
    A production reference includes a finite polyline of confirmed node means:
    these chords are a measured-node interpolation model, NOT exact historical
    pixel faces. Latest actual faces remain separately available for audit.
    """
    def __init__(self, config=None):
        cfg = dict(CONSENSUS_DEFAULT_CONFIG)
        if config is not None:
            if set(config) - set(cfg):
                raise ValueError("unknown boundary consensus setting")
            cfg.update(config)
        integer_names = {"minimum_frame_votes", "minimum_reliable_votes", "max_candidate_cells",
                         "max_confirmed_cells", "max_support_segments"}
        for name in cfg:
            cfg[name] = (_integer(cfg[name], name, 1) if name in integer_names
                         else _positive(cfg[name], name, name == "candidate_ttl_s"))
        if (cfg["minimum_reliable_votes"] > cfg["minimum_frame_votes"]
                or cfg["reliable_sensitivity"] > cfg["max_sensitivity"]
                or cfg["max_candidate_cells"] > 200000 or cfg["max_confirmed_cells"] > 100000
                or cfg["max_support_segments"] > 200000 or cfg["normal_max_angle_rad"] >= math.pi / 2
                or not math.isfinite(cfg["candidate_ttl_s"] * 1.e9)):
            raise ValueError("inconsistent boundary consensus settings")
        self.config = MappingProxyType(cfg)
        self._records = {}
        self._edges = {}
        self._generation = 0
        self._next_record_id = 0
        self._segment_count = 0
        self._confirmed_count = 0
        self.source_frontier_ns = None

    @property
    def confirmed_count(self):
        return self._confirmed_count

    @property
    def last_stamp_ns(self):
        return self.source_frontier_ns

    def _confirmed(self, record):
        return (record.votes >= self.config["minimum_frame_votes"]
                and record.reliable_votes >= self.config["minimum_reliable_votes"])

    def prepare(self, observation_world):
        if not isinstance(observation_world, BoundaryObservation):
            raise ValueError("boundary consensus requires explicit observation")
        obs, cfg = observation_world, self.config
        stamp = obs.source_stamp_ns
        if self.source_frontier_ns is not None and stamp <= self.source_frontier_ns:
            raise ValueError("boundary source stamp not increasing")
        ttl = int(cfg["candidate_ttl_s"] * 1.e9)
        expired = tuple(record_id for record_id, record in self._records.items()
                        if ttl and not self._confirmed(record) and stamp - record.last_stamp_ns > ttl)
        expired_set = set(expired)
        buckets = {}
        for record_id, record in self._records.items():
            if record_id not in expired_set:
                buckets.setdefault(record.cell, []).append(record)
        model_points, model_j, model_reliable = obs.model_points, obs.model_sensitivity, obs.model_reliable
        model_normals = obs.model_normals
        eligible = np.flatnonzero(obs.model_valid & (model_j <= cfg["max_sensitivity"]))
        if np.any(np.abs(model_points[eligible]) > 1.e7):
            raise ValueError("boundary world coordinate bound")
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            cells_float = np.rint(model_points[eligible] / cfg["grid_resolution_m"])
        if np.any(np.abs(cells_float) > np.iinfo(np.int32).max):
            raise ValueError("boundary cell coordinate bound")
        cells = cells_float.astype(np.int64)
        cosine = math.cos(cfg["normal_max_angle_rad"])
        groups = _contiguous_cell_runs(obs, eligible, cells, cosine)
        frame_points, frame_normals = _group_frame_means(
            model_points, model_normals, [members for _, _, members in groups])
        prepared_groups, claims = [], {}
        for group_index, (cell, _, members) in enumerate(groups):
            faces = np.asarray(members, dtype=np.int64)
            point = frame_points[group_index]
            normal = frame_normals[group_index]
            normal /= np.linalg.norm(normal)
            if not np.all(np.isfinite(point)) or not np.all(np.isfinite(normal)):
                raise ValueError("nonfinite boundary frame mean")
            choices = [record for record in buckets.get(cell, ())
                       if float(np.dot(normal, record.normal)) >= cosine]
            chosen = None
            if choices:
                distances = np.asarray([np.sum((point - record.point) ** 2) for record in choices])
                best = np.flatnonzero(distances == np.min(distances))
                if len(best) == 1:
                    chosen = choices[int(best[0])]
                    claims.setdefault(chosen.record_id, []).append(len(prepared_groups))
            prepared_groups.append((cell, faces, point, normal, chosen))
        next_id, face_owner, updates = self._next_record_id, np.full(len(obs.points), -1, dtype=np.int64), []
        ambiguous_groups = 0
        for cell, faces, point, normal, old in prepared_groups:
            if old is not None and len(claims[old.record_id]) != 1:
                old = None
                ambiguous_groups += 1
            if old is None:
                record_id, votes, reliable_votes = next_id, 1, 0
                next_id += 1
            else:
                record_id, votes, reliable_votes = old.record_id, old.votes + 1, old.reliable_votes
                # Equal FRAME influence, independent of edge pixel count.
                point = np.asarray(old.point) + (point - old.point) / votes
                normal = np.asarray(old.normal) + (normal - old.normal) / votes
                normal /= np.linalg.norm(normal)
            reliable_votes += int(np.any(model_reliable[faces] & (model_j[faces] <= cfg["reliable_sensitivity"])))
            if (not np.all(np.isfinite(point)) or not np.all(np.isfinite(normal))
                    or not np.all(np.abs(point / cfg["grid_resolution_m"] - np.asarray(cell)) <= .5 + 1.e-7)):
                raise ValueError("boundary mean left original cell")
            face_owner[faces] = record_id
            updates.append((record_id, cell, tuple(point), tuple(normal), votes, reliable_votes, faces))
        # Owners are final before any neighbour lookup. Integer gathering once
        # for the frame preserves every original face slot and missing link.
        all_neighbours = np.full((len(obs.points), 2), -1, dtype=np.int64)
        for side, links in enumerate((obs.previous, obs.next)):
            known = links >= 0
            all_neighbours[known, side] = face_owner[links[known]]
        records = []
        for record_id, cell, point, normal, votes, reliable_votes, faces in updates:
            neighbours = all_neighbours[faces]
            # BoundaryObservation validates and freezes raw geometry once.
            # Eligible members are model-valid, hence also metric-valid. The
            # gathered arrays have the known dtype/shape; preserve separate
            # immutable byte backing per record without revalidating each one.
            segments = np.frombuffer(obs.segments[faces].tobytes(), dtype=np.float64).reshape(-1, 2, 2)
            links = np.frombuffer(neighbours.tobytes(), dtype=np.int64).reshape(-1, 2)
            records.append(_BoundaryRecord(record_id, cell, point, normal, votes, reliable_votes, stamp,
                                           segments, links))
        new_ids = sum(record.record_id not in self._records for record in records)
        final_count = len(self._records) - len(expired) + new_ids
        if final_count > cfg["max_candidate_cells"]:
            raise ValueError("boundary candidate capacity")
        old_confirmed = self._confirmed_count
        added_confirmed = sum(self._confirmed(record) and (
            record.record_id not in self._records or not self._confirmed(self._records[record.record_id])) for record in records)
        if old_confirmed + added_confirmed > cfg["max_confirmed_cells"]:
            raise ValueError("boundary confirmed capacity")
        segments = self._segment_count - sum(len(self._records[key].segments) for key in expired)
        segments += sum(len(record.segments) - (len(self._records[record.record_id].segments)
                        if record.record_id in self._records else 0) for record in records)
        if segments > cfg["max_support_segments"]:
            raise ValueError("boundary support segment capacity")

        # Only an ACTUAL directed adjacent-face transition observes a link.
        # No FOV endpoint, unassigned face, J cut or same-cell proximity does.
        edge_votes = {}
        face_reliable = model_reliable & obs.model_valid & (model_j <= cfg["reliable_sensitivity"])
        faces = eligible[obs.next[eligible] >= 0]
        following = obs.next[faces]
        admitted = (face_owner[following] >= 0) & (face_owner[faces] != face_owner[following])
        faces, following = faces[admitted], following[admitted]
        if len(faces):
            sources, targets = face_owner[faces], face_owner[following]
            order = np.lexsort((targets, sources))
            sources, targets = sources[order], targets[order]
            reliable = (face_reliable[faces] & face_reliable[following])[order]
            first = np.r_[True, (sources[1:] != sources[:-1]) | (targets[1:] != targets[:-1])]
            starts = np.flatnonzero(first)
            reliable = np.logical_or.reduceat(reliable, starts)
            edge_votes = {(int(sources[start]), int(targets[start])): bool(vote)
                          for start, vote in zip(starts, reliable)}
        expired_edges = tuple(key for key, edge in self._edges.items()
                              if edge.source_id in expired_set or edge.target_id in expired_set
                              or (ttl and not self._confirmed(edge) and stamp - edge.last_stamp_ns > ttl))
        expired_edge_set = set(expired_edges)
        edge_updates = []
        for key, reliable in sorted(edge_votes.items()):
            old = None if key in expired_edge_set else self._edges.get(key)
            edge_updates.append(_BoundaryEdge(key[0], key[1], 1 if old is None else old.votes + 1,
                                int(reliable) + (0 if old is None else old.reliable_votes), stamp))
        edge_count = len(self._edges) - len(expired_edges) + sum(
            key not in self._edges or key in expired_edge_set for key in edge_votes)
        if edge_count > cfg["max_support_segments"]:
            raise ValueError("boundary topology edge capacity")
        stats = MappingProxyType({"input_edges": len(obs.points), "eligible_edges": len(eligible),
                                  "invalid_edges": len(obs.points) - len(eligible), "updated_records": len(records),
                                  "ambiguous_groups": ambiguous_groups, "expired_candidates": len(expired),
                                  "candidate_records": final_count, "confirmed_records": old_confirmed + added_confirmed,
                                  "retained_segments": segments, "topology_candidate_edges": edge_count,
                                  "topology_updated_edges": len(edge_updates),
                                  "topology_expired_edges": len(expired_edges),
                                  "topology_representation": TOPOLOGY_REPRESENTATION})
        return PreparedBoundaryUpdate(stamp, self._generation, self, tuple(records), expired,
                                      next_id, segments, tuple(edge_updates), expired_edges, stats)

    def can_commit(self, prepared):
        return (isinstance(prepared, PreparedBoundaryUpdate) and prepared.owner is self
                and prepared.base_generation == self._generation
                and (self.source_frontier_ns is None or prepared.source_stamp_ns > self.source_frontier_ns))

    def commit(self, prepared):
        if not self.can_commit(prepared):
            return False
        for record_id in prepared.expired_ids:
            del self._records[record_id]
        for record in prepared.updates:
            self._records[record.record_id] = record
        for key in prepared.expired_edge_keys:
            del self._edges[key]
        for edge in prepared.edge_updates:
            self._edges[(edge.source_id, edge.target_id)] = edge
        self._next_record_id = prepared.next_record_id
        self._segment_count = prepared.segment_count
        self._confirmed_count = prepared.stats["confirmed_records"]
        self.source_frontier_ns = prepared.source_stamp_ns
        self._generation += 1
        return True

    def snapshot(self):
        records = [record for _, record in sorted(self._records.items()) if self._confirmed(record)]
        ids = np.asarray([record.record_id for record in records], dtype=np.int64)
        if records:
            all_segments = np.concatenate([record.segments for record in records], axis=0)
            all_neighbours = np.concatenate([record.neighbours for record in records], axis=0)
            all_owners = np.repeat(np.arange(len(records), dtype=np.int64),
                                   [len(record.segments) for record in records])
            # ONE membership computation for the entire reference, preserving
            # record/face order exactly. Repeating isin(all IDs) per record was
            # unnecessarily quadratic in map size. Both endpoints still need
            # confirmed owners; no FOV/J gap or unconfirmed neighbour is bridged.
            known = np.all(np.isin(all_neighbours, ids), axis=1)
            segments, owners = all_segments[known], all_owners[known]
        else:
            segments, owners = np.empty((0, 2, 2)), np.empty(0, dtype=np.int64)
        slots = {int(record_id): slot for slot, record_id in enumerate(ids)}
        links = [edge for _, edge in sorted(self._edges.items())
                 if self._confirmed(edge) and edge.source_id in slots and edge.target_id in slots]
        incoming, outgoing = {}, {}
        for edge in links:
            incoming[edge.target_id] = incoming.get(edge.target_id, 0) + 1
            outgoing[edge.source_id] = outgoing.get(edge.source_id, 0) + 1
        # Keep branch interiors as nearest candidates: deleting them could make
        # a farther unrelated edge win. Full confirmed degrees certify joints
        # separately and survive subsets, which must not turn a fork into a
        # seemingly unambiguous continuation by dropping its third branch.
        graph = dict(directed_edges=np.asarray([[slots[edge.source_id], slots[edge.target_id]] for edge in links],
                                              dtype=np.int64).reshape(-1, 2),
                     edge_frame_votes=np.asarray([edge.votes for edge in links], dtype=np.int64),
                     edge_reliable_votes=np.asarray([edge.reliable_votes for edge in links], dtype=np.int64),
                     edge_last_stamp_ns=np.asarray([edge.last_stamp_ns for edge in links], dtype=np.int64),
                     full_in_degree=np.asarray([incoming.get(int(record_id), 0) for record_id in ids], dtype=np.int64),
                     full_out_degree=np.asarray([outgoing.get(int(record_id), 0) for record_id in ids], dtype=np.int64))
        return BoundaryReference(np.asarray([record.point for record in records], dtype=np.float64).reshape(-1, 2),
                                 np.asarray([record.normal for record in records], dtype=np.float64).reshape(-1, 2),
                                 segments, owners,
                                 np.asarray([record.votes for record in records], dtype=np.int64),
                                 np.asarray([record.reliable_votes for record in records], dtype=np.int64), ids,
                                 self.source_frontier_ns, **graph)


def _configuration(config):
    cfg = dict(DEFAULT_CONFIG)
    if config is not None:
        if set(config) - set(cfg):
            raise ValueError("unknown boundary setting")
        cfg.update(config)
    for name in cfg:
        if name in {"normal_min_points", "max_patch_points", "max_input_edges"}:
            cfg[name] = _integer(cfg[name], name, 1)
        else:
            cfg[name] = _positive(cfg[name], name)
    if (cfg["normal_max_radius_m"] < cfg["normal_radius_m"]
            or not 2 <= cfg["normal_min_points"] <= cfg["max_patch_points"] <= 512
            or cfg["max_input_edges"] > 200000 or cfg["normal_max_variance_ratio"] >= 1):
        raise ValueError("inconsistent boundary settings")
    return cfg


def _pixel_faces(active):
    """Only internal white/black faces; orientation keeps white on the left."""
    rows, cols = np.nonzero(active[:, :-1] != active[:, 1:])
    left = active[rows, cols]
    vertical = np.stack((np.column_stack((2 * cols + 1, 2 * rows - 1)),
                         np.column_stack((2 * cols + 1, 2 * rows + 1))), axis=1)
    vertical[~left] = vertical[~left, ::-1]
    white_v = np.column_stack((cols + (~left), rows))
    black_v = np.column_stack((cols + left, rows))
    rows, cols = np.nonzero(active[:-1, :] != active[1:, :])
    above = active[rows, cols]
    horizontal = np.stack((np.column_stack((2 * cols - 1, 2 * rows + 1)),
                           np.column_stack((2 * cols + 1, 2 * rows + 1))), axis=1)
    horizontal[above] = horizontal[above, ::-1]
    white_h = np.column_stack((cols, rows + (~above)))
    black_h = np.column_stack((cols, rows + above))
    return (np.concatenate((vertical, horizontal)).astype(np.int64),
            np.concatenate((white_v, white_h)).astype(np.int64),
            np.concatenate((black_v, black_h)).astype(np.int64))


def _contours(vertices, white):
    n = len(vertices)
    previous, following = np.full(n, -1, dtype=np.int64), np.full(n, -1, dtype=np.int64)
    starts = {}
    for index, vertex in enumerate(vertices[:, 0]):
        starts.setdefault(tuple(vertex), []).append(index)
    for index, vertex in enumerate(vertices[:, 1]):
        choices = starts.get(tuple(vertex), ())
        if len(choices) > 1:
            # Checkerboard contact: follow the same WHITE pixel, preserving
            # four-connected white components rather than joining diagonals.
            choices = [item for item in choices if np.array_equal(white[item], white[index])]
        if len(choices) == 1:
            successor = choices[0]
            if previous[successor] != -1:
                raise ValueError("ambiguous directed contour")
            following[index], previous[successor] = successor, index
    ids, paths = np.full(n, -1, dtype=np.int64), []
    for seed in list(np.flatnonzero(previous < 0)) + list(range(n)):
        if ids[seed] >= 0:
            continue
        path, current = [], int(seed)
        while current >= 0 and ids[current] < 0:
            ids[current] = len(paths)
            path.append(current)
            current = int(following[current])
        paths.append((np.asarray(path, dtype=np.int64), current == seed))
    return previous, following, ids, paths


def _projector(lookup, normal_body):
    from .tilted_projection import TiltedGroundLookup
    normal, basis, _ = TiltedGroundLookup._normal_and_basis(normal_body)
    camera = np.asarray(lookup.camera_matrix, dtype=np.float64)
    transform = np.asarray(lookup.base_from_camera, dtype=np.float64)
    if (camera.shape != (3, 3) or transform.shape != (4, 4)
            or not np.all(np.isfinite(camera)) or not np.all(np.isfinite(transform))):
        raise ValueError("invalid boundary calibration")
    ray_transform = transform[:3, :3] @ np.linalg.inv(camera)
    origin = transform[:3, 3]
    offset = getattr(lookup, "plane_offset_m", 0.)
    if isinstance(offset, (bool, np.bool_)) or not isinstance(offset, Real) or not math.isfinite(float(offset)):
        raise ValueError("invalid boundary plane offset")
    height = float(normal @ origin) - float(offset)
    if height <= 0 or not math.isfinite(height):
        raise ValueError("camera must be above boundary observation plane")

    def project(uv):
        uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
        rays = np.column_stack((uv, np.ones(len(uv)))) @ ray_transform.T
        denominator = rays @ normal
        valid = np.isfinite(denominator) & (denominator < -1.e-9)
        xy = np.full((len(uv), 2), np.nan)
        with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
            scale = -height / denominator[valid]
            xy[valid] = (origin + scale[:, None] * rays[valid]) @ basis.T
        valid &= np.all(np.isfinite(xy), axis=1)
        return xy, valid
    def unproject(xy):
        base = np.asarray(xy, dtype=np.float64).reshape(-1, 2) @ basis + float(offset)*normal
        camera_xyz = (base-origin) @ transform[:3, :3]
        homogeneous = camera_xyz @ camera.T
        good = np.all(np.isfinite(homogeneous), axis=1) & (homogeneous[:, 2] > 0.)
        uv = np.full((len(base), 2), np.nan)
        np.divide(homogeneous[:, :2], homogeneous[:, 2, None], out=uv, where=good[:, None])
        return uv, good
    return project, unproject


def _directions(points, segments, white_xy, black_xy, pixel_outward, stable, paths, cfg):
    """Batched contour-only windows. Stops at missing/J-cut/opposite faces.

    Limiting arc distance (rather than Euclidean distance alone) also prevents
    a nearby U arm being pulled into the local patch through a remote bridge.
    Opposite IMAGE face polarities stop traversal even around a very short end
    cap. Perspective is not angle-preserving: orthogonal pixel stair faces can
    have obtuse metric normals, which must not split one observed paint side.
    """
    n = len(points)
    normals, valid = np.zeros((n, 2)), np.zeros(n, dtype=bool)
    curve_coefficients = np.full((n, 3), np.nan)
    curve_scale = np.full(n, np.nan)
    curve_bounds = np.full((n, 2), np.nan)
    support_count, used_radius = np.zeros(n, dtype=np.int64), np.zeros(n)
    tangent = segments[:, 1] - segments[:, 0]
    length = np.linalg.norm(tangent, axis=1)
    white_to_black = black_xy - white_xy
    crossing_length = np.linalg.norm(white_to_black, axis=1)
    ok = (stable & np.isfinite(length) & (length > 0)
          & np.isfinite(crossing_length) & (crossing_length > 0))
    crossing_direction = np.zeros_like(points)
    crossing_direction[ok] = white_to_black[ok] / crossing_length[ok, None]
    max_points = cfg["max_patch_points"]
    left_max, right_max = (max_points - 1) // 2, max_points // 2
    radii, radius = [], cfg["normal_radius_m"]
    while radius < cfg["normal_max_radius_m"]:
        radii.append(radius)
        radius = min(radius * 2., cfg["normal_max_radius_m"])
    radii.append(cfg["normal_max_radius_m"])
    for path, closed in paths:
        count = len(path)
        if count < cfg["normal_min_points"]:
            continue
        left = min(left_max, (count - 1) // 2) if closed else min(left_max, count - 1)
        right = min(right_max, count - 1 - left) if closed else min(right_max, count - 1)
        offsets = np.arange(-left, right + 1)
        # Arc distance follows the measured face, including its corner vertex.
        step = np.linalg.norm(segments[path, 1] - points[path], axis=1)
        step += np.linalg.norm(points[np.roll(path, -1)] - segments[path, 1], axis=1)
        step[~np.isfinite(step)] = np.inf
        arc = np.concatenate(([0.], np.cumsum(np.where(np.isfinite(step), step, 0.))))
        circumference = arc[-1]
        for start in range(0, count, 128):
            positions = np.arange(start, min(start + 128, count))
            # An invalid seed makes its complete allowed row false. Avoid its
            # temporary arrays, but never remove faces from any other window.
            positions = positions[ok[path[positions]]]
            if not len(positions):
                continue
            targets = path[positions]
            slots = positions[:, None] + offsets
            inside = np.ones(slots.shape, dtype=bool) if closed else ((slots >= 0) & (slots < count))
            wrapped = slots % count
            candidates = path[wrapped]
            arc_candidate = arc[wrapped]
            if closed:
                arc_candidate = arc_candidate + np.floor_divide(slots, count) * circumference
            distance = np.abs(arc_candidate - arc[positions, None])
            # Exact cardinal integer pixel directions: same=1, stair=0,
            # opposite=-1. Keep the seed-relative test and contiguous stop;
            # neither a later compatible face nor a J gap can be jumped.
            compatible = (pixel_outward[targets, None, 0] * pixel_outward[candidates, 0]
                          + pixel_outward[targets, None, 1] * pixel_outward[candidates, 1]) >= 0
            allowed = inside & ok[targets, None] & ok[candidates] & compatible
            # Every intervening face must be usable; never jump an invalid face.
            if left:
                allowed[:, :left] = np.logical_and.accumulate(allowed[:, :left][:, ::-1], axis=1)[:, ::-1]
            if right:
                allowed[:, left + 1:] = np.logical_and.accumulate(allowed[:, left + 1:], axis=1)
            differences = points[candidates] - points[targets, None, :]
            differences[~np.isfinite(differences)] = 0.
            for radius in radii:
                if np.all(valid[targets]):
                    break
                selected = allowed & (distance <= radius)
                counts = np.count_nonzero(selected, axis=1)
                enough = (~valid[targets]) & (counts >= cfg["normal_min_points"])
                if not np.any(enough):
                    continue
                delta, weights = differences[enough], selected[enough]
                means = np.sum(delta * weights[:, :, None], axis=1) / counts[enough, None]
                centered = delta - means[:, None, :]
                cov = _weighted_patch_products(centered, centered, weights) / counts[enough, None, None]
                eigenvalues, eigenvectors = np.linalg.eigh(cov)
                accepted = ((eigenvalues[:, 1] > 1.e-5)
                            & (eigenvalues[:, 0] <= cfg["normal_max_thickness_m"] ** 2)
                            & (eigenvalues[:, 0] <= cfg["normal_max_variance_ratio"] * eigenvalues[:, 1]))
                fitted = eigenvectors[:, :, 0]
                # A normal is a covector. Its sign against the actual measured
                # white->black displacement survives camera shear/perspective;
                # dotting two projected normals does not. Normalize the
                # displacement so the existing 1e-12 angular check is unitless.
                orientation = np.sum(fitted * crossing_direction[targets[enough]], axis=1)
                accepted &= np.abs(orientation) > 1.e-12
                if not np.any(accepted):
                    continue
                fitted[orientation < 0] *= -1
                indices = targets[enough][accepted]
                normals[indices] = fitted[accepted]
                coefficients, scales, bounds = _fit_quadratic_patches(
                    delta[accepted], weights[accepted], fitted[accepted])
                curve_coefficients[indices], curve_scale[indices] = coefficients, scales
                curve_bounds[indices] = bounds
                valid[indices] = True
                support_count[indices] = counts[enough][accepted]
                used_radius[indices] = radius
    return normals, valid, support_count, used_radius, curve_coefficients, curve_scale, curve_bounds


def _weighted_patch_products(left, right, weights):
    """Same three operands/K order, one small output component at a time.

    Avoid the generic multidimensional einsum loop without BLAS, algebraic
    reassociation, symmetry copying, zero-column removal or altered windows.
    The literal previous contraction remains an independent test oracle.
    """
    result = np.empty((len(left), left.shape[2], right.shape[2]), dtype=np.float64)
    for i in range(left.shape[2]):
        for j in range(right.shape[2]):
            result[:, i, j] = np.einsum('nk,nk,nk->n', left[:, :, i], right[:, :, j],
                                       weights, optimize=False)
    return result


def _fit_quadratic_patches(delta, weights, normals):
    """Local y=a+b*x+c*x² in the measured PCA frame, not a tangent line.

    The same existing contour window is used. Scaling x to unit magnitude
    avoids metre-dependent normal-equation rank decisions. No ridge, invented
    sample, changed support radius, or truth-derived curvature is introduced.
    """
    tangents = np.column_stack((normals[:, 1], -normals[:, 0]))
    # np.sum of the original two-element axis uses a positive-zero identity.
    # Retain its signed-zero result as well as the two multiply/add operations.
    x = (delta[:, :, 0]*tangents[:, None, 0] + delta[:, :, 1]*tangents[:, None, 1]) + 0.
    y = (delta[:, :, 0]*normals[:, None, 0] + delta[:, :, 1]*normals[:, None, 1]) + 0.
    scales = np.max(np.where(weights, np.abs(x), 0.), axis=1)
    scaled = np.zeros_like(x)
    np.divide(x, scales[:, None], out=scaled, where=scales[:, None] > 0.)
    design = np.stack((np.ones_like(x), scaled, scaled*scaled), axis=2)
    gram = _weighted_patch_products(design, design, weights)
    rhs = np.empty((len(design), 3))
    for i in range(3):
        rhs[:, i] = np.einsum('nk,nk,nk->n', design[:, :, i], y, weights, optimize=False)
    eigenvalues = np.linalg.eigvalsh(gram)
    good = (scales > 0.) & (eigenvalues[:, 0] > 64.*np.finfo(np.float64).eps*eigenvalues[:, -1])
    coefficients = np.full((len(delta), 3), np.nan)
    coefficients[good] = np.linalg.solve(gram[good], rhs[good, :, None])[:, :, 0]
    bounds = np.column_stack((np.min(np.where(weights, scaled, np.inf), axis=1),
                              np.max(np.where(weights, scaled, -np.inf), axis=1)))
    return coefficients, scales, bounds


def _quantized_curve_positions(white_xy, black_xy, origins, normals, coefficients, scales, bounds, valid):
    """Unique, outward, in-bracket curve intersection; never extrapolate a fit."""
    valid = (valid & np.isfinite(scales) & (scales > 0.)
             & np.all(np.isfinite(coefficients), axis=1) & np.all(np.isfinite(bounds), axis=1)
             & (bounds[:, 0] <= bounds[:, 1])
             & np.all(np.isfinite(white_xy), axis=1) & np.all(np.isfinite(black_xy), axis=1)
             & np.all(np.isfinite(origins), axis=1) & np.all(np.isfinite(normals), axis=1))
    tangents = np.column_stack((normals[:, 1], -normals[:, 0]))
    crossing, offset = black_xy-white_xy, white_xy-origins
    with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
        x0 = np.sum(offset*tangents, axis=1)/scales
        dx = np.sum(crossing*tangents, axis=1)/scales
        y0, dy = np.sum(offset*normals, axis=1), np.sum(crossing*normals, axis=1)
        a, b, c = coefficients.T
        qa, qb, qc = -c*dx*dx, dy-b*dx-2.*c*x0*dx, y0-a-b*x0-c*x0*x0
        roots = np.full((len(valid), 2), np.nan)
        linear = valid & (qa == 0.) & (qb != 0.)
        roots[linear, 0] = -qc[linear]/qb[linear]
        discriminant = qb*qb-4.*qa*qc
        quadratic = valid & (qa != 0.) & np.isfinite(discriminant) & (discriminant > 0.)
        q = -.5*(qb[quadratic]+np.copysign(np.sqrt(discriminant[quadratic]), qb[quadratic]))
        roots[quadratic, 0] = q/qa[quadratic]
        roots[quadratic, 1] = qc[quadratic]/q
        inside = np.isfinite(roots) & (roots >= 0.) & (roots <= 1.)
        unique = valid & (np.sum(inside, axis=1) == 1)
        fraction = roots[np.arange(len(valid)), np.argmax(inside, axis=1)]
        x = x0+fraction*dx
        positions = white_xy+fraction[:, None]*crossing
        gradient = normals-((b+2.*c*x)/scales)[:, None]*tangents
        length = np.linalg.norm(gradient, axis=1)
        derivative = np.sum(gradient*crossing, axis=1)
        uncertainty = 8.*np.finfo(np.float64).eps*np.sum(np.abs(gradient)*np.abs(crossing), axis=1)
        good = (unique & np.all(np.isfinite(positions), axis=1) & np.isfinite(length) & (length > 0.)
                & np.isfinite(derivative) & (derivative > uncertainty)
                & (x >= bounds[:, 0]) & (x <= bounds[:, 1]))
        fitted_normals = gradient/length[:, None]
        good &= np.sum(fitted_normals*normals, axis=1) >= math.cos(CONSENSUS_DEFAULT_CONFIG['normal_max_angle_rad'])
    positions[~good], fitted_normals[~good] = np.nan, 0.
    return positions, fitted_normals, good


def _quantized_positions(white_xy, black_xy, normals, line_points, valid):
    """Estimate a boundary INSIDE its observed white/black pixel-centre bracket.

    A TLS line is a local model, not subpixel truth. Never clamp an outside
    intersection or replace an invalid estimate with the convenient raw centre.
    """
    crossing = black_xy-white_xy
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        denominator = np.sum(normals*crossing, axis=1)
        numerator = np.sum(normals*(line_points-white_xy), axis=1)
        scale = np.sum(np.abs(normals)*np.abs(crossing), axis=1)
        good = (valid & np.isfinite(denominator) & np.isfinite(numerator)
                & (denominator > 8.*np.finfo(np.float64).eps*scale))
        fraction = np.full(len(valid), np.nan)
        np.divide(numerator, denominator, out=fraction, where=good)
        good &= (fraction >= 0.) & (fraction <= 1.)
        positions = white_xy + fraction[:, None]*crossing
        good &= np.all(np.isfinite(positions), axis=1)
    positions[~good] = np.nan
    return positions, good


def extract_boundary_observation(mask, lookup, normal_body, source_stamp_ns, *, threshold=127, config=None):
    """Extract all internal faces, retaining invalid evidence and its reasons.

    The original filled projection is not called or changed. J4 is evaluated
    at face centre AND endpoints; all four perturbations must be in the image
    and metric-valid. Thus lack of sensitivity evidence is explicit invalidity,
    never a fabricated low-J boundary. Pixel face coordinates use integer pixel
    centres and half-integer cell edges, consistent with the existing lookup.
    """
    cfg = _configuration(config)
    stamp = _integer(source_stamp_ns, "source stamp", 1)
    width, height = _integer(lookup.width, "width", 2), _integer(lookup.height, "height", 2)
    raw = np.asarray(mask)
    if raw.shape != (height, width) or raw.dtype.kind not in "buif" or not np.all(np.isfinite(raw)):
        raise ValueError("invalid full mask")
    if isinstance(threshold, (bool, np.bool_)) or not isinstance(threshold, Real) or not math.isfinite(float(threshold)):
        raise ValueError("invalid mask threshold")
    maximum_j = _positive(lookup.max_sensitivity_m_per_px, "max sensitivity")
    if cfg["reliable_sensitivity"] > maximum_j:
        raise ValueError("reliable sensitivity exceeds stable sensitivity")
    vertices, white, black = _pixel_faces(raw > threshold)
    n = len(vertices)
    if n > cfg["max_input_edges"]:
        raise ValueError("boundary observation capacity")
    previous, following, contour_ids, paths = _contours(vertices, white)
    pixel_segments = vertices.astype(np.float64) * .5
    centers = np.mean(pixel_segments, axis=1)
    project, unproject = _projector(lookup, normal_body)
    points, point_valid = project(centers)
    endpoints, endpoint_valid = project(pixel_segments)
    segments = endpoints.reshape(n, 2, 2)
    white_xy, white_valid = project(white)
    black_xy, black_valid = project(black)
    metric = point_valid & np.all(endpoint_valid.reshape(n, 2), axis=1) & white_valid & black_valid
    sensitivity, j_observed = np.full(n, np.inf), np.ones(n, dtype=bool)
    shifts = np.array([[-1., 0.], [1., 0.], [0., -1.], [0., 1.]])
    for start in range(0, n, 4096):
        stop = min(start + 4096, n)
        probes = np.concatenate((centers[start:stop, None, :], pixel_segments[start:stop]), axis=1)
        base, base_valid = project(probes)
        neighbours = probes[:, :, None, :] + shifts
        in_image = ((neighbours[..., 0] >= -.5) & (neighbours[..., 0] <= width - .5)
                    & (neighbours[..., 1] >= -.5) & (neighbours[..., 1] <= height - .5))
        xy, good = project(neighbours)
        good = good.reshape(stop - start, 3, 4) & in_image
        all_good = np.all(good, axis=(1, 2)) & np.all(base_valid.reshape(stop - start, 3), axis=1)
        with np.errstate(over="ignore", invalid="ignore"):
            delta = xy.reshape(stop - start, 3, 4, 2) - base.reshape(stop - start, 3, 1, 2)
            values = np.max(np.linalg.norm(delta, axis=3), axis=(1, 2))
        sensitivity[start:stop] = np.where(all_good, values, np.inf)
        j_observed[start:stop] = all_good
    stable = metric & j_observed & np.isfinite(sensitivity) & (sensitivity <= maximum_j)
    normals, normal_valid, support_count, radii, coefficients, scales, bounds = _directions(
        points, segments, white_xy, black_xy, black - white, stable, paths, cfg)
    valid = stable & normal_valid
    reliable = valid & (sensitivity <= cfg["reliable_sensitivity"])
    model_points, model_normals, model_valid = _quantized_curve_positions(
        white_xy, black_xy, points, normals, coefficients, scales, bounds, valid)
    # Projective metric fraction is NOT the image fraction. Reproject the
    # actual estimate before evaluating its own J4; raw-face J is insufficient.
    model_uv, uv_valid = unproject(model_points)
    model_base, base_valid = project(model_uv)
    probes = model_uv[:, None, :] + shifts
    in_image = ((probes[..., 0] >= -.5) & (probes[..., 0] <= width-.5)
                & (probes[..., 1] >= -.5) & (probes[..., 1] <= height-.5))
    probe_xy, probe_valid = project(probes)
    good_j = uv_valid & base_valid & np.all(probe_valid.reshape(n, 4) & in_image, axis=1)
    with np.errstate(over="ignore", invalid="ignore"):
        model_j = np.max(np.linalg.norm(probe_xy.reshape(n, 4, 2)-model_base[:, None, :], axis=2), axis=1)
    model_j[~good_j] = np.inf
    model_valid &= good_j & np.isfinite(model_j) & (model_j <= maximum_j)
    reasons = ((~metric).astype(np.int64) * REASON_METRIC
               | (~stable).astype(np.int64) * REASON_SENSITIVITY
               | (~normal_valid).astype(np.int64) * REASON_DIRECTION
               | (~j_observed).astype(np.int64) * REASON_J4_UNOBSERVED)
    return BoundaryObservation(points, normals, segments, sensitivity, metric, stable, normal_valid,
                               valid, reliable, stamp, pixel_segments, white, black, previous,
                               following, contour_ids, reasons, support_count, radii,
                               model_points, model_valid, model_j,
                               model_valid & (model_j <= cfg["reliable_sensitivity"]), model_normals)
