"""Exclusive, hash-checked local-map bundles independent of GNSS.

Mapping and route readiness depend on local geometry, not GNSS availability.
Repeat-start GNSS evidence belongs outside this bundle. The outer schema
deliberately cannot be loaded by the older VectorNav/ENU follower.
Route geometry retains schema 1 for the shared pure closed-route controller.
"""

import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import re
import uuid

import numpy as np

from lane_mapping_lya_reference.route import build_route
from lane_mapping_lya_reference.controller import ClosedRouteController
from lane_mapping_lya_reference.paint_geometry import BoundaryReference

from .mapping import MapSnapshot


SCHEMA_VERSION = 2
BOUNDARY_SCHEMA_VERSION = 3
BOUNDARY_REPRESENTATION = "paint_boundary_v1"
BOUNDARY_VERSION = 1
BOUNDARY_GRAPH_VERSION = 2
BOUNDARY_CERTIFIED_VERSION = 3
BOUNDARY_CONTINUATION_VERSION = 4
BOUNDARY_CURVE_VERSION = 5
CURVE_REPRESENTATION = "finite_creation_chart_curve_consensus_v1"
CERTIFIED_MODEL_REPRESENTATION = "confirmed_creation_chart_domains_v1"
CONTINUATION_REPRESENTATION = "all_continuations_owned_quad_support_v1"
BOUNDARY_ARRAYS = {"points_xy": ("<f8", (2,)), "normals": ("<f8", (2,)),
                   "support_segments": ("<f8", (2, 2)), "segment_owner": ("<i8", ()),
                   "votes": ("<i8", ()), "reliable_votes": ("<i8", ()),
                   "record_ids": ("<i8", ())}
BOUNDARY_EDGE_ARRAYS = {"directed_edges": ("<i8", (2,)),
                         "edge_frame_votes": ("<i8", ()),
                         "edge_reliable_votes": ("<i8", ()),
                         "edge_last_stamp_ns": ("<i8", ())}
BOUNDARY_GRAPH_ARRAYS = dict(BOUNDARY_EDGE_ARRAYS,
                            full_in_degree=("<i8", ()), full_out_degree=("<i8", ()))
BOUNDARY_CERTIFIED_ARRAYS = {
    "certified_model_segments": ("<f8", (2, 2)),
    "certified_model_owner": ("<i8", ()),
    "certified_model_frame_votes": ("<i8", ()),
    "certified_model_reliable_votes": ("<i8", ()),
    "certified_model_last_stamp_ns": ("<i8", ())}
BOUNDARY_CONTINUATION_ARRAYS = {
    "continuation_intervals": ("<f8", (2,)),
    "continuation_primitive": ("<i8", ()),
    "continuation_frame_votes": ("<i8", ()),
    "continuation_reliable_votes": ("<i8", ()),
    "continuation_last_stamp_ns": ("<i8", ())}
BOUNDARY_CURVE_ARRAYS = {
    "curve_segments": ("<f8", (2, 2)), "curve_normals": ("<f8", (2,)),
    "curve_owner": ("<i8", ()), "curve_frame_votes": ("<i8", ()),
    "curve_reliable_votes": ("<i8", ()), "curve_last_stamp_ns": ("<i8", ()),
    "curve_owner_status": ("<i8", ()), "curve_blocker_boxes": ("<f8", (2, 2)),
    "curve_blocker_owner": ("<i8", ())}
LOCALIZATION_KIND = "wheel_rawgyro_mask"
COORDINATE_CONVENTION = "start-relative local metres; x initial vehicle forward, y left; yaw CCW; not ENU"
MAX_PAYLOAD_BYTES = 16 * 1024 * 1024
MAX_GEOMETRY_POINTS = 100000
PAYLOAD_NAMES = ("route", "metadata", "consensus", "trace", "calibration")


def _json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")


def _json_copy(value):
    return json.loads(_json_bytes(value).decode("utf-8"))


def _digest(data):
    return hashlib.sha256(data).hexdigest()


def geometry_sha256(snapshot):
    """Hash exact frozen input arrays to verify snapshot preservation."""
    digest = hashlib.sha256()
    for array in (snapshot.trace, snapshot.consensus_xy, snapshot.votes):
        value = np.asarray(array, dtype="<f8")
        digest.update(str(value.shape).encode("ascii"))
        digest.update(value.tobytes())
    reference = snapshot.boundary_reference
    if reference is not None:
        # The legacy three-array digest above is deliberately unchanged.
        # Boundary identity includes normals, disconnected support and votes,
        # not just anchor positions that could describe another model.
        digest.update(_json_bytes(_boundary_payload(reference)))
    return digest.hexdigest()


def _boundary_payload(reference):
    if not isinstance(reference, BoundaryReference):
        raise ValueError("an immutable BoundaryReference is required")
    if reference.curve_geometry is not None:
        return _boundary_curve_payload(reference)
    graph = [getattr(reference, name, None) for name in BOUNDARY_GRAPH_ARRAYS]
    if any(value is None for value in graph) and not all(value is None for value in graph):
        raise ValueError("boundary graph members must all be present or all absent")
    version = BOUNDARY_VERSION if graph[0] is None else BOUNDARY_GRAPH_VERSION
    certified = [getattr(reference, name, None) for name in BOUNDARY_CERTIFIED_ARRAYS]
    if any(value is None for value in certified) and not all(value is None for value in certified):
        raise ValueError("certified model members must all be present or all absent")
    if certified[0] is not None:
        if version != BOUNDARY_GRAPH_VERSION:
            raise ValueError("certified model requires explicit boundary graph")
        if reference.certified_model_representation != CERTIFIED_MODEL_REPRESENTATION:
            raise ValueError("unsupported certified model representation")
        version = BOUNDARY_CERTIFIED_VERSION
    continuations = [getattr(reference, name, None) for name in BOUNDARY_CONTINUATION_ARRAYS]
    if any(value is None for value in continuations) and not all(value is None for value in continuations):
        raise ValueError("continuation members must all be present or all absent")
    if continuations[0] is not None:
        if version != BOUNDARY_CERTIFIED_VERSION:
            raise ValueError("continuation permissions require certified boundary geometry")
        if reference.continuation_representation != CONTINUATION_REPRESENTATION:
            raise ValueError("unsupported continuation representation")
        version = BOUNDARY_CONTINUATION_VERSION
    specs = dict(BOUNDARY_ARRAYS)
    if version >= BOUNDARY_GRAPH_VERSION:
        specs.update(BOUNDARY_GRAPH_ARRAYS)
    if version >= BOUNDARY_CERTIFIED_VERSION:
        specs.update(BOUNDARY_CERTIFIED_ARRAYS)
    if version == BOUNDARY_CONTINUATION_VERSION:
        specs.update(BOUNDARY_CONTINUATION_ARRAYS)
    arrays = {}
    for name, (dtype, suffix) in specs.items():
        value = np.asarray(getattr(reference, name), dtype=dtype)
        capacity = _boundary_capacity(name)
        if value.ndim != len(suffix) + 1 or value.shape[1:] != suffix or len(value) > capacity:
            raise ValueError("boundary array shape/capacity invalid: " + name)
        arrays[name] = {"dtype": dtype, "shape": list(value.shape),
                        "sha256": _digest(value.tobytes()), "data": value.tolist()}
    result = {"representation": BOUNDARY_REPRESENTATION, "version": version,
              "source_frontier_ns": int(reference.source_frontier_ns), "arrays": arrays}
    # Do not add new null markers to older versions: their identity is frozen.
    if version >= BOUNDARY_CERTIFIED_VERSION:
        result["certified_model_representation"] = CERTIFIED_MODEL_REPRESENTATION
    if version == BOUNDARY_CONTINUATION_VERSION:
        result["continuation_representation"] = CONTINUATION_REPRESENTATION
    return result


def _boundary_capacity(name):
    return (200000 if name in ("support_segments", "segment_owner")
            or name in BOUNDARY_EDGE_ARRAYS or name in BOUNDARY_CERTIFIED_ARRAYS
            or name in BOUNDARY_CONTINUATION_ARRAYS
            or name in BOUNDARY_CURVE_ARRAYS and name != "curve_owner_status"
            else MAX_GEOMETRY_POINTS)


def _boundary_curve_payload(reference):
    """V5 has explicit curve arrays; graph edges are topology, not chords.

    Every rejection-only box and status is hash checked too. A missing
    continuation set remains missing, distinct from an explicitly empty set.
    None of the legacy v1-v4 serialized bytes are changed by this branch.
    """
    curve = reference.curve_geometry
    specs = dict(BOUNDARY_ARRAYS, **BOUNDARY_GRAPH_ARRAYS, **BOUNDARY_CURVE_ARRAYS)
    continuation = reference.continuation_intervals is not None
    if continuation:
        specs.update(BOUNDARY_CONTINUATION_ARRAYS)
    arrays = {}
    for name, (dtype, suffix) in specs.items():
        source = getattr(curve, name[6:]) if name.startswith("curve_") else getattr(reference, name)
        value = np.asarray(source, dtype=dtype)
        if value.ndim != len(suffix)+1 or value.shape[1:] != suffix or len(value) > _boundary_capacity(name):
            raise ValueError("boundary array shape/capacity invalid: " + name)
        arrays[name] = dict(dtype=dtype, shape=list(value.shape),
                            sha256=_digest(value.tobytes()), data=value.tolist())
    return dict(representation=BOUNDARY_REPRESENTATION, version=BOUNDARY_CURVE_VERSION,
        source_frontier_ns=int(reference.source_frontier_ns), arrays=arrays,
        curve_representation=CURVE_REPRESENTATION, curve_complete=curve.complete,
        continuation_representation=CONTINUATION_REPRESENTATION if continuation else None)


def _load_boundary_curve(value, source_frontier_ns):
    from lane_mapping_lya_reference.curve_reference import CurveGeometry
    fields = {"representation", "version", "source_frontier_ns", "arrays",
              "curve_representation", "curve_complete", "continuation_representation"}
    if (set(value) != fields or value["representation"] != BOUNDARY_REPRESENTATION
            or type(value["version"]) is not int or value["version"] != BOUNDARY_CURVE_VERSION
            or type(value["source_frontier_ns"]) is not int or value["source_frontier_ns"] != source_frontier_ns
            or value["curve_representation"] != CURVE_REPRESENTATION
            or type(value["curve_complete"]) is not bool or not isinstance(value["arrays"], dict)
            or value["continuation_representation"] not in (None, CONTINUATION_REPRESENTATION)):
        raise ValueError("curve boundary representation/frontier mismatch")
    specs = dict(BOUNDARY_ARRAYS, **BOUNDARY_GRAPH_ARRAYS, **BOUNDARY_CURVE_ARRAYS)
    if value["continuation_representation"] is not None:
        specs.update(BOUNDARY_CONTINUATION_ARRAYS)
    if set(value["arrays"]) != set(specs):
        raise ValueError("curve boundary member set differs from explicit geometry version")
    arrays = {}
    for name, (dtype, suffix) in specs.items():
        record = value["arrays"][name]
        if not isinstance(record, dict) or set(record) != {"dtype", "shape", "sha256", "data"}:
            raise ValueError("invalid boundary member: " + name)
        shape = record["shape"]
        if (record["dtype"] != dtype or not isinstance(shape, list)
                or len(shape) != len(suffix)+1 or any(type(n) is not int for n in shape)
                or tuple(shape[1:]) != suffix or not 0 <= shape[0] <= _boundary_capacity(name)):
            raise ValueError("invalid boundary member layout: " + name)
        scalar_types = (int,) if dtype == "<i8" else (int, float)
        def valid(item, dimensions):
            if not dimensions:
                return type(item) in scalar_types
            return (isinstance(item, list) and len(item) == dimensions[0]
                    and all(valid(child, dimensions[1:]) for child in item))
        if not valid(record["data"], shape):
            raise ValueError("invalid boundary member scalar types: " + name)
        raw = np.asarray(record["data"])
        if raw.size == 0 and shape[0] == 0:
            raw = raw.reshape(shape)
        allowed = "iu" if dtype == "<i8" else "iuf"
        minimum = -1 if name == "curve_blocker_owner" else 0
        if (list(raw.shape) != shape or raw.size and raw.dtype.kind not in allowed
                or raw.size and not np.all(np.isfinite(raw))
                or dtype == "<i8" and raw.size and (int(np.min(raw)) < minimum or int(np.max(raw)) >= 2**63)):
            raise ValueError("invalid boundary member values: " + name)
        array = np.asarray(raw, dtype=dtype)
        if _digest(array.tobytes()) != record["sha256"]:
            raise ValueError("boundary member checksum mismatch: " + name)
        arrays[name] = array
    curve_arrays = {name[6:]: arrays.pop(name) for name in BOUNDARY_CURVE_ARRAYS}
    curve = CurveGeometry(**curve_arrays, complete=value["curve_complete"])
    return BoundaryReference(**arrays, source_frontier_ns=source_frontier_ns, curve_geometry=curve)


def _load_boundary(data, source_frontier_ns, expected_version):
    value = _decode_json(data)
    _require_local_only(value)
    if expected_version == BOUNDARY_CURVE_VERSION:
        return _load_boundary_curve(value, source_frontier_ns)
    fields = {"representation", "version", "source_frontier_ns", "arrays"}
    if expected_version >= BOUNDARY_CERTIFIED_VERSION:
        fields.add("certified_model_representation")
    if expected_version == BOUNDARY_CONTINUATION_VERSION:
        fields.add("continuation_representation")
    if (set(value) != fields
            or value["representation"] != BOUNDARY_REPRESENTATION
            or type(value["version"]) is not int or value["version"] != expected_version
            or type(value["source_frontier_ns"]) is not int
            or value["source_frontier_ns"] != source_frontier_ns
            or not isinstance(value["arrays"], dict)):
        raise ValueError("boundary representation/frontier mismatch")
    if (expected_version >= BOUNDARY_CERTIFIED_VERSION
            and value["certified_model_representation"] != CERTIFIED_MODEL_REPRESENTATION):
        raise ValueError("unsupported certified model representation")
    if (expected_version == BOUNDARY_CONTINUATION_VERSION
            and value["continuation_representation"] != CONTINUATION_REPRESENTATION):
        raise ValueError("unsupported continuation representation")
    specs = dict(BOUNDARY_ARRAYS)
    if expected_version in (BOUNDARY_GRAPH_VERSION, BOUNDARY_CERTIFIED_VERSION, BOUNDARY_CONTINUATION_VERSION):
        specs.update(BOUNDARY_GRAPH_ARRAYS)
    elif expected_version != BOUNDARY_VERSION:
        raise ValueError("unsupported boundary geometry version")
    if expected_version >= BOUNDARY_CERTIFIED_VERSION:
        specs.update(BOUNDARY_CERTIFIED_ARRAYS)
    if expected_version == BOUNDARY_CONTINUATION_VERSION:
        specs.update(BOUNDARY_CONTINUATION_ARRAYS)
    if set(value["arrays"]) != set(specs):
        raise ValueError("boundary member set differs from explicit geometry version")
    arrays = {}
    for name, (dtype, suffix) in specs.items():
        record = value["arrays"][name]
        if not isinstance(record, dict) or set(record) != {"dtype", "shape", "sha256", "data"}:
            raise ValueError("invalid boundary member: " + name)
        shape = record["shape"]
        capacity = _boundary_capacity(name)
        if (record["dtype"] != dtype or not isinstance(shape, list)
                or len(shape) != len(suffix) + 1 or any(type(n) is not int for n in shape)
                or tuple(shape[1:]) != suffix or not 0 <= shape[0] <= capacity):
            raise ValueError("invalid boundary member layout: " + name)
        # Validate JSON scalar types before NumPy promotes mixed bool/int rows.
        # A signed/rehashed bundle must not smuggle booleans as record IDs.
        scalar_types = (int,) if dtype == "<i8" else (int, float)
        def valid_values(item, dimensions):
            if not dimensions:
                return type(item) in scalar_types
            return (isinstance(item, list) and len(item) == dimensions[0]
                    and all(valid_values(child, dimensions[1:]) for child in item))
        if not valid_values(record["data"], shape):
            raise ValueError("invalid boundary member scalar types: " + name)
        raw = np.asarray(record["data"])
        if raw.size == 0 and shape[0] == 0:
            raw = raw.reshape(shape)
        allowed = "iu" if dtype == "<i8" else "iuf"
        if (list(raw.shape) != shape or (raw.size and raw.dtype.kind not in allowed)
                or (raw.size and not np.all(np.isfinite(raw)))
                or (dtype == "<i8" and raw.size and (int(np.min(raw)) < 0 or int(np.max(raw)) >= 2 ** 63))):
            raise ValueError("invalid boundary member values: " + name)
        array = np.asarray(raw, dtype=dtype)
        if _digest(array.tobytes()) != record["sha256"]:
            raise ValueError("boundary member checksum mismatch: " + name)
        arrays[name] = array
    return BoundaryReference(**arrays, source_frontier_ns=value["source_frontier_ns"])


def _boundary_schema(payload, schema):
    if schema == BOUNDARY_SCHEMA_VERSION:
        if (payload.get("feature_representation") != BOUNDARY_REPRESENTATION
                or type(payload.get("boundary_version")) is not int
                or payload["boundary_version"] not in (BOUNDARY_VERSION, BOUNDARY_GRAPH_VERSION,
                                                       BOUNDARY_CERTIFIED_VERSION, BOUNDARY_CONTINUATION_VERSION,
                                                       BOUNDARY_CURVE_VERSION)):
            raise ValueError("boundary schema requires explicit representation/version")
        if payload["boundary_version"] == BOUNDARY_CURVE_VERSION:
            if (payload.get("curve_representation") != CURVE_REPRESENTATION
                    or "certified_model_representation" in payload
                    or "continuation_representation" not in payload
                    or payload["continuation_representation"] not in (None, CONTINUATION_REPRESENTATION)):
                raise ValueError("boundary v5 requires explicit actual-curve representation")
            return
        if "curve_representation" in payload:
            raise ValueError("legacy boundary cannot claim curve geometry")
        if payload["boundary_version"] >= BOUNDARY_CERTIFIED_VERSION:
            if payload.get("certified_model_representation") != CERTIFIED_MODEL_REPRESENTATION:
                raise ValueError("boundary v3 requires explicit certified model representation")
        elif "certified_model_representation" in payload:
            raise ValueError("legacy boundary cannot claim certified model geometry")
        if payload["boundary_version"] == BOUNDARY_CONTINUATION_VERSION:
            if payload.get("continuation_representation") != CONTINUATION_REPRESENTATION:
                raise ValueError("boundary v4 requires explicit continuation representation")
        elif "continuation_representation" in payload:
            raise ValueError("legacy boundary cannot claim continuation permissions")
    elif any(key in payload for key in ("feature_representation", "boundary_version", "boundary_path", "boundary_sha256", "boundary_reference", "certified_model_representation", "continuation_representation", "curve_representation")):
        raise ValueError("legacy bundle cannot claim boundary geometry")


def _require_local_only(value):
    """Reject geographic coordinates/anchors even in optional evidence fields."""
    prohibited = {"latitude", "latitude_deg", "longitude", "longitude_deg", "lat", "lon",
                  "lla", "poslla", "origin_lla", "start_anchor", "end_anchor", "anchors",
                  "anchors_path", "anchors_sha256", "gnss_usage", "end_anchor_applied_to_geometry"}
    if isinstance(value, dict):
        for key, child in value.items():
            if str(key).lower() in prohibited:
                raise ValueError("GNSS coordinates/anchors do not belong in a map bundle")
            _require_local_only(child)
    elif isinstance(value, (list, tuple)):
        for child in value:
            _require_local_only(child)


def _calibration(value):
    result = _json_copy(value)
    if not isinstance(result, dict) or not result:
        raise ValueError("nonempty camera calibration evidence is required")
    _require_local_only(result)
    # The ROS wrapper owns sensor-specific calibration/frame validation. Keep
    # its complete original evidence rather than manufacturing missing fields.
    if len(_json_bytes(result)) > 1024 * 1024:
        raise ValueError("calibration exceeds bounded size")
    return result


def _csv_bytes(header, rows):
    output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(header)
    writer.writerows(rows)
    return output.getvalue().encode("utf-8")


def _check_snapshot(snapshot):
    if not isinstance(snapshot, MapSnapshot):
        raise ValueError("a frozen MapSnapshot is required")
    if (not 3 <= len(snapshot.trace) <= MAX_GEOMETRY_POINTS
            or not 3 <= len(snapshot.consensus_xy) <= MAX_GEOMETRY_POINTS):
        raise ValueError("snapshot geometry size is invalid")
    if (not np.all(np.isfinite(snapshot.trace)) or not np.all(np.isfinite(snapshot.consensus_xy))
            or not np.all(np.isfinite(snapshot.votes)) or np.any(snapshot.votes < 1)
            or np.any(snapshot.votes != np.floor(snapshot.votes))):
        raise ValueError("snapshot geometry/votes are invalid")
    if snapshot.boundary_reference is not None:
        if snapshot.boundary_reference.source_frontier_ns != snapshot.source_frontier_ns:
            raise ValueError("boundary reference differs from snapshot frontier")
        curve = snapshot.boundary_reference.curve_geometry
        if curve is not None and (not curve.complete or not len(curve.segments)):
            raise ValueError("ready bundle requires complete nonempty actual curve geometry")
        _boundary_payload(snapshot.boundary_reference)


def build_bundle(outputroot, snapshot, calibration, route_config=None, session_id=None):
    """Validate then exclusively save; return the absolute bundle.json Path.

    Input arrays are never modified or reanchored. A failing route check
    produces no ready manifest. No GNSS input is accepted; startup localization,
    publishing and arming belong to the caller.
    """
    _check_snapshot(snapshot)
    before = geometry_sha256(snapshot)
    calibration = _calibration(calibration)
    route = build_route(snapshot.trace.copy(), snapshot.consensus_xy.copy(), route_config)
    if geometry_sha256(snapshot) != before:
        raise ValueError("route generation changed frozen snapshot geometry")
    if session_id is None:
        session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:12]
    if not isinstance(session_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", session_id):
        raise ValueError("invalid session id")
    shared = {"localization_kind": LOCALIZATION_KIND, "frame_id": "lane_teach_local",
              "coordinate_convention": COORDINATE_CONVENTION,
              "origin_local_xyz": [0.0, 0.0, 0.0],
              "heading_source": "wheel_rawgyro_start_relative",
              "speed_policy": route["speed_policy"],
              "reference_speed_mps": route["reference_speed_mps"]}
    schema = SCHEMA_VERSION if snapshot.boundary_reference is None else BOUNDARY_SCHEMA_VERSION
    if schema == BOUNDARY_SCHEMA_VERSION:
        boundary_payload = _boundary_payload(snapshot.boundary_reference)
        shared.update(feature_representation=BOUNDARY_REPRESENTATION,
                      boundary_version=boundary_payload["version"])
        if boundary_payload["version"] == BOUNDARY_CURVE_VERSION:
            shared["curve_representation"] = CURVE_REPRESENTATION
            shared["continuation_representation"] = boundary_payload["continuation_representation"]
        elif boundary_payload["version"] >= BOUNDARY_CERTIFIED_VERSION:
            shared["certified_model_representation"] = CERTIFIED_MODEL_REPRESENTATION
        if boundary_payload["version"] == BOUNDARY_CONTINUATION_VERSION:
            shared["continuation_representation"] = CONTINUATION_REPRESENTATION
    route.update(shared)
    metadata = dict(shared, schema_version=schema, session_id=session_id,
                    finished=True, model_type="causal_odometry_mask_consensus",
                    source_frontier_ns=int(snapshot.source_frontier_ns),
                    snapshot_geometry_sha256=before, snapshot_geometry_after_build_sha256=before,
                    trace_points=len(snapshot.trace), consensus_points=len(snapshot.consensus_xy),
                    mapping_stats=dict(snapshot.stats),
                    gnss_used_for_mapping=False,
                    geometry_retroactively_corrected=False,
                    requires_operator_review=True, closed_loop_vehicle_validation=False)
    _require_local_only(route)
    _require_local_only(metadata)
    payloads = {
        "route": ("route.json", _json_bytes(route)),
        "metadata": ("metadata.json", _json_bytes(metadata)),
        "calibration": ("calibration.json", _json_bytes(calibration)),
        "consensus": ("consensus.csv", _csv_bytes(["x_m", "y_m", "frame_votes"],
                      ((float(point[0]), float(point[1]), int(vote))
                       for point, vote in zip(snapshot.consensus_xy, snapshot.votes)))),
        "trace": ("trace.csv", _csv_bytes(["x_m", "y_m", "yaw_rad"], snapshot.trace.tolist())),
    }
    if snapshot.boundary_reference is not None:
        payloads["boundary"] = ("boundary.json", _json_bytes(_boundary_payload(snapshot.boundary_reference)))
    if any(len(data) > MAX_PAYLOAD_BYTES for _, data in payloads.values()):
        raise ValueError("bundle payload exceeds bounded size")
    root = Path(outputroot).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    directory = root / (session_id + "_" + uuid.uuid4().hex[:12])
    directory.mkdir(exist_ok=False)
    manifest = dict(shared, schema_version=schema, ready=True, session_id=session_id,
                    bundle_id=uuid.uuid4().hex, session_directory=str(directory))
    for name, (filename, data) in payloads.items():
        with (directory / filename).open("xb") as stream:
            stream.write(data)
            stream.flush()
        manifest[name + "_path"] = filename
        manifest[name + "_sha256"] = _digest(data)
    manifest_path = directory / "bundle.json"
    with manifest_path.open("xb") as stream:
        stream.write(_json_bytes(manifest))
        stream.flush()
    return manifest_path


def _bounded_read(path):
    if not path.is_file() or path.stat().st_size > MAX_PAYLOAD_BYTES:
        raise ValueError("bundle payload missing or oversized")
    with path.open("rb") as stream:
        data = stream.read(MAX_PAYLOAD_BYTES + 1)
    if len(data) > MAX_PAYLOAD_BYTES:
        raise ValueError("bundle payload grew beyond size limit")
    return data


def _decode_json(data):
    def invalid_constant(value):
        raise ValueError("nonfinite JSON constant: " + value)
    result = json.loads(data.decode("utf-8"), parse_constant=invalid_constant)
    if not isinstance(result, dict):
        raise ValueError("bundle JSON must contain an object")
    return result


def _resolve_payload(directory, relative):
    if (not isinstance(relative, str) or not relative or "\\" in relative
            or ":" in relative or "\x00" in relative):
        raise ValueError("bundle path must be a safe relative file")
    path = Path(relative)
    if path.is_absolute() or path.anchor or ".." in path.parts:
        raise ValueError("bundle path escapes directory")
    target = (directory / path).resolve(strict=True)
    try:
        target.relative_to(directory)
    except ValueError:
        raise ValueError("bundle target escapes directory")
    return target


def _read_csv(data, header):
    reader = csv.reader(io.StringIO(data.decode("utf-8"), newline=""))
    if next(reader, None) != header:
        raise ValueError("bundle CSV header mismatch")
    rows = []
    for row in reader:
        if len(rows) >= MAX_GEOMETRY_POINTS or len(row) != len(header):
            raise ValueError("bundle CSV exceeds size or has wrong columns")
        values = [float(value) for value in row]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("bundle CSV contains nonfinite geometry")
        rows.append(values)
    if len(rows) < 3:
        raise ValueError("bundle CSV has insufficient geometry")
    return np.asarray(rows, dtype=float)


def load_bundle(manifest_path):
    """Return (manifest, route, metadata, consensus_xy, calibration).

    Legacy schema2 retains its five payloads and exact geometry hash. Boundary
    schema3 adds a hash-checked reference. metadata['boundary_reference'] is an
    in-memory immutable object (or None), never a serialized metadata field.
    """
    path = Path(manifest_path).expanduser().resolve(strict=True)
    manifest = _decode_json(_bounded_read(path))
    schema = manifest.get("schema_version")
    if (type(schema) is not int or schema not in (SCHEMA_VERSION, BOUNDARY_SCHEMA_VERSION) or manifest.get("ready") is not True
            or manifest.get("localization_kind") != LOCALIZATION_KIND):
        raise ValueError("unsupported or unready local-map bundle")
    _require_local_only(manifest)
    _boundary_schema(manifest, schema)
    payloads = {}
    resolved = set()
    for name in PAYLOAD_NAMES + (("boundary",) if schema == BOUNDARY_SCHEMA_VERSION else ()):
        target = _resolve_payload(path.parent, manifest.get(name + "_path"))
        if target in resolved or target == path:
            raise ValueError("bundle payload paths must be distinct")
        resolved.add(target)
        data = _bounded_read(target)
        if _digest(data) != manifest.get(name + "_sha256"):
            raise ValueError(name + " checksum mismatch")
        payloads[name] = data
    route = _decode_json(payloads["route"])
    metadata = _decode_json(payloads["metadata"])
    calibration = _calibration(_decode_json(payloads["calibration"]))
    _require_local_only(route)
    _require_local_only(metadata)
    _boundary_schema(route, schema)
    _boundary_schema(metadata, schema)
    if schema == BOUNDARY_SCHEMA_VERSION and any(
            payload["boundary_version"] != manifest["boundary_version"] for payload in (route, metadata)):
        raise ValueError("boundary geometry version differs across bundle payloads")
    if "boundary_reference" in metadata:
        raise ValueError("boundary reference is not a JSON metadata field")
    consensus = _read_csv(payloads["consensus"], ["x_m", "y_m", "frame_votes"])
    trace = _read_csv(payloads["trace"], ["x_m", "y_m", "yaw_rad"])
    if (metadata.get("schema_version") != schema or metadata.get("finished") is not True
            or metadata.get("geometry_retroactively_corrected") is not False
            or metadata.get("gnss_used_for_mapping") is not False
            or route.get("schema_version") != 1 or route.get("closed") is not True
            or route.get("diagnostics", {}).get("valid") is not True):
        raise ValueError("invalid route or completion metadata")
    expected = {"localization_kind": LOCALIZATION_KIND, "frame_id": "lane_teach_local",
                "coordinate_convention": COORDINATE_CONVENTION, "origin_local_xyz": [0., 0., 0.],
                "heading_source": "wheel_rawgyro_start_relative"}
    for key, value in expected.items():
        if any(payload.get(key) != value for payload in (manifest, route, metadata)):
            raise ValueError("bundle coordinate/policy mismatch: " + key)
    boundary_reference = (_load_boundary(payloads["boundary"], metadata.get("source_frontier_ns"),
                                        manifest["boundary_version"])
                 if schema == BOUNDARY_SCHEMA_VERSION else None)
    if schema == BOUNDARY_SCHEMA_VERSION and manifest["boundary_version"] == BOUNDARY_CURVE_VERSION:
        expected_continuation = (CONTINUATION_REPRESENTATION
            if boundary_reference.continuation_intervals is not None else None)
        if any(payload["continuation_representation"] != expected_continuation
               for payload in (manifest, route, metadata)):
            raise ValueError("curve continuation presence differs across bundle payloads")
    snapshot = MapSnapshot(trace, consensus[:, :2], consensus[:, 2],
                           metadata.get("source_frontier_ns", 0), metadata.get("mapping_stats", {}),
                           boundary_reference=boundary_reference)
    _check_snapshot(snapshot)
    digest = geometry_sha256(snapshot)
    if (metadata.get("snapshot_geometry_sha256") != digest
            or metadata.get("snapshot_geometry_after_build_sha256") != digest
            or metadata.get("trace_points") != len(trace)
            or metadata.get("consensus_points") != len(consensus)):
        raise ValueError("frozen geometry integrity mismatch")
    # Validate geometry against its recorded BUILD contract, not an unrelated
    # controller's legacy .8 m/s/profile defaults. The ROS node independently
    # constructs its controller with the current DEPLOYMENT limits before it
    # can accept this candidate for repeat; successful loading is not approval.
    policy = route.get("speed_policy", "profile")
    if policy == "fixed_reference":
        reference = route.get("reference_speed_mps")
        if (isinstance(reference, bool) or not isinstance(reference, (int, float))
                or not math.isfinite(reference) or reference <= 0):
            raise ValueError("fixed reference bundle requires a finite positive reference speed")
        for payload in (manifest, metadata):
            saved_reference = payload.get("reference_speed_mps")
            if (payload.get("speed_policy") != policy or isinstance(saved_reference, bool)
                    or not isinstance(saved_reference, (int, float))
                    or not math.isfinite(saved_reference) or saved_reference != reference):
                raise ValueError("bundle speed policy/reference metadata mismatch")
        config = route.get("config")
        if not isinstance(config, dict) or config.get("reference_speed_mps") != reference:
            raise ValueError("fixed reference bundle route build configuration mismatch")
        limits = {}
        for key in ("max_curvature_1pm", "max_lateral_accel_mps2"):
            value = config.get(key)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError("invalid saved route build limit: " + key)
            limits[key] = float(value)
        ClosedRouteController(route, reference_speed_mps=reference,
            maximum_speed=reference,
            maximum_yaw_rate=reference * limits["max_curvature_1pm"],
            maximum_lateral_acceleration=limits["max_lateral_accel_mps2"])
    else:
        # Older profile bundles predate these metadata fields. They remain
        # readable, but may not claim a contradictory fixed-reference policy.
        if any(payload.get("speed_policy", "profile") != "profile"
               or payload.get("reference_speed_mps", 0.0) != 0.0
               for payload in (manifest, route, metadata)):
            raise ValueError("bundle speed policy/reference metadata mismatch")
        ClosedRouteController(route)
    metadata["boundary_reference"] = boundary_reference
    return manifest, route, metadata, snapshot.consensus_xy, calibration
