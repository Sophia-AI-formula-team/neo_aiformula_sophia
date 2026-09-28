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

from .mapping import MapSnapshot


SCHEMA_VERSION = 2
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
    return digest.hexdigest()


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
    route.update(shared)
    metadata = dict(shared, schema_version=SCHEMA_VERSION, session_id=session_id,
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
    if any(len(data) > MAX_PAYLOAD_BYTES for _, data in payloads.values()):
        raise ValueError("bundle payload exceeds bounded size")
    root = Path(outputroot).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    directory = root / (session_id + "_" + uuid.uuid4().hex[:12])
    directory.mkdir(exist_ok=False)
    manifest = dict(shared, schema_version=SCHEMA_VERSION, ready=True, session_id=session_id,
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

    All five payloads are size-bounded and hash-checked before their content
    is trusted. GNSS evidence is never a map-bundle input or payload.
    """
    path = Path(manifest_path).expanduser().resolve(strict=True)
    manifest = _decode_json(_bounded_read(path))
    if (manifest.get("schema_version") != SCHEMA_VERSION or manifest.get("ready") is not True
            or manifest.get("localization_kind") != LOCALIZATION_KIND):
        raise ValueError("unsupported or unready local-map bundle")
    _require_local_only(manifest)
    payloads = {}
    resolved = set()
    for name in PAYLOAD_NAMES:
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
    consensus = _read_csv(payloads["consensus"], ["x_m", "y_m", "frame_votes"])
    trace = _read_csv(payloads["trace"], ["x_m", "y_m", "yaw_rad"])
    if (metadata.get("schema_version") != SCHEMA_VERSION or metadata.get("finished") is not True
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
    snapshot = MapSnapshot(trace, consensus[:, :2], consensus[:, 2],
                           metadata.get("source_frontier_ns", 0), metadata.get("mapping_stats", {}))
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
    return manifest, route, metadata, snapshot.consensus_xy, calibration
