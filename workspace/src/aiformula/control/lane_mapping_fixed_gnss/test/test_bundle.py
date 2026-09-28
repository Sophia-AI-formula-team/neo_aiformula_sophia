"""Local-map readiness is independent of GNSS and preserves geometry integrity."""

import hashlib
from contextlib import nullcontext
import inspect
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss.bundle import build_bundle, geometry_sha256, load_bundle
from lane_mapping_fixed_gnss.mapping import MapSnapshot
from lane_mapping_fixed_gnss.worker import MapWorker
from lane_mapping_lya_reference.controller import ClosedRouteController


@pytest.fixture
def snapshot():
    radius = 12.
    angle = np.linspace(-math.pi / 2., 3 * math.pi / 2., 501)
    trace = np.column_stack((radius * np.cos(angle), radius + radius * np.sin(angle), angle + math.pi / 2))
    lane_angle = np.linspace(-math.pi / 2., 3 * math.pi / 2., 2400, endpoint=False)
    lanes = np.vstack([np.column_stack((r * np.cos(lane_angle), radius + r * np.sin(lane_angle)))
                       for r in (radius - 1.2, radius + 1.2)])
    return MapSnapshot(trace, lanes, np.full(len(lanes), 5.), 10000000000, {"accepted": 501})


def create(tmp_path, snapshot):
    return build_bundle(tmp_path, snapshot, {"frame_id": "camera", "K": [1., 0., 0., 0., 1., 0., 0., 0., 1.]}, session_id="test")


def rewrite(path, obj):
    path.write_text(json.dumps(obj), encoding="utf-8")


def test_roundtrip_preserves_frozen_geometry_and_shared_controller(tmp_path, snapshot):
    before = geometry_sha256(snapshot)
    path = create(tmp_path, snapshot)
    manifest, route, metadata, points, calibration = load_bundle(path)
    assert manifest["schema_version"] == 2 and route["schema_version"] == 1
    assert manifest["localization_kind"] == "wheel_rawgyro_mask"
    assert "origin_lla" not in manifest and "vectornav" not in manifest["heading_source"]
    assert np.array_equal(points, snapshot.consensus_xy)
    assert metadata["snapshot_geometry_sha256"] == before == geometry_sha256(snapshot)
    assert metadata["snapshot_geometry_after_build_sha256"] == before
    assert metadata["gnss_used_for_mapping"] is False
    assert "anchors_path" not in manifest
    assert not (path.parent / "anchors.json").exists()
    assert {key[:-5] for key in manifest if key.endswith("_path")} == {
        "route", "metadata", "consensus", "trace", "calibration"}
    assert calibration["frame_id"] == "camera"
    assert ClosedRouteController(route).length > 70


def test_repeat_save_is_exclusive_never_overwrites(tmp_path, snapshot):
    first = create(tmp_path, snapshot)
    digest = hashlib.sha256(first.read_bytes()).hexdigest()
    second = create(tmp_path, snapshot)
    assert first != second
    assert hashlib.sha256(first.read_bytes()).hexdigest() == digest


def test_gnss_is_not_an_api_input_and_is_not_read(tmp_path, snapshot, monkeypatch):
    assert list(inspect.signature(build_bundle).parameters) == [
        "outputroot", "snapshot", "calibration", "route_config", "session_id"]
    baseline = create(tmp_path, snapshot)

    class PoisonGnss:
        def __getattribute__(self, name):
            if name.startswith("__"):
                return object.__getattribute__(self, name)
            raise AssertionError("mapping must not read external GNSS: " + name)

    # Unrelated missing/poisoned startup evidence cannot affect map generation.
    # This is a pure API isolation test, not a sensor replay or vehicle proof.
    import lane_mapping_fixed_gnss.bundle as module
    monkeypatch.setattr(module, "start_anchor", PoisonGnss(), raising=False)
    monkeypatch.setattr(module, "end_anchor", PoisonGnss(), raising=False)
    poisoned = create(tmp_path, snapshot)
    first = json.loads(baseline.read_text())
    second = json.loads(poisoned.read_text())
    for name in ("route", "metadata", "consensus", "trace", "calibration"):
        assert first[name + "_sha256"] == second[name + "_sha256"]


@pytest.mark.parametrize("key", ["start_anchor", "end_anchor"])
def test_gnss_keyword_is_rejected_without_writing(tmp_path, snapshot, key):
    with pytest.raises(TypeError):
        build_bundle(tmp_path, snapshot, {"frame_id": "camera"}, **{key: {"accepted": True}})
    assert not list(tmp_path.rglob("bundle.json"))


@pytest.mark.parametrize("key", ["origin_lla", "latitude_deg", "longitude_deg", "start_anchor"])
def test_gnss_coordinates_cannot_hide_in_calibration(tmp_path, snapshot, key):
    with pytest.raises(ValueError, match="GNSS"):
        build_bundle(tmp_path, snapshot, {"frame_id": "camera", "extra": {key: [35., 139., 0.]}})
    assert not list(tmp_path.rglob("bundle.json"))


@pytest.mark.parametrize("key", ["route", "metadata", "consensus", "trace", "calibration"])
def test_all_payloads_have_checksum_enforcement(tmp_path, snapshot, key):
    path = create(tmp_path, snapshot)
    manifest = json.loads(path.read_text())
    target = path.parent / manifest[key + "_path"]
    target.write_bytes(target.read_bytes() + b" ")
    with pytest.raises(ValueError, match="checksum"):
        load_bundle(path)


@pytest.mark.parametrize("relative", ["../route.json", "/route.json", "C:/route.json", "..\\route.json"])
def test_bundle_path_traversal_rejected(tmp_path, snapshot, relative):
    path = create(tmp_path, snapshot)
    manifest = json.loads(path.read_text())
    manifest["route_path"] = relative
    rewrite(path, manifest)
    with pytest.raises(ValueError, match="path"):
        load_bundle(path)


def test_updated_hash_does_not_allow_gnss_mapping_claim(tmp_path, snapshot):
    path = create(tmp_path, snapshot)
    manifest = json.loads(path.read_text())
    target = path.parent / manifest["metadata_path"]
    evidence = json.loads(target.read_text())
    evidence["gnss_used_for_mapping"] = True
    rewrite(target, evidence)
    manifest["metadata_sha256"] = hashlib.sha256(target.read_bytes()).hexdigest()
    rewrite(path, manifest)
    with pytest.raises(ValueError, match="completion metadata"):
        load_bundle(path)


@pytest.mark.parametrize("payload", ["route", "metadata", "calibration"])
def test_updated_hash_cannot_add_geographic_origin(tmp_path, snapshot, payload):
    path = create(tmp_path, snapshot)
    manifest = json.loads(path.read_text())
    target = path.parent / manifest[payload + "_path"]
    evidence = json.loads(target.read_text())
    evidence["origin_lla"] = [35., 139., 50.]
    rewrite(target, evidence)
    manifest[payload + "_sha256"] = hashlib.sha256(target.read_bytes()).hexdigest()
    rewrite(path, manifest)
    with pytest.raises(ValueError, match="GNSS"):
        load_bundle(path)


def test_updated_payload_hash_does_not_hide_geometry_change(tmp_path, snapshot):
    path = create(tmp_path, snapshot)
    manifest = json.loads(path.read_text())
    target = path.parent / manifest["consensus_path"]
    lines = target.read_text().splitlines()
    row = lines[1].split(",")
    row[0] = str(float(row[0]) + 1.)
    lines[1] = ",".join(row)
    target.write_text("\n".join(lines) + "\n")
    manifest["consensus_sha256"] = hashlib.sha256(target.read_bytes()).hexdigest()
    rewrite(path, manifest)
    with pytest.raises(ValueError, match="geometry integrity"):
        load_bundle(path)


def test_unknown_outer_schema_fails(tmp_path, snapshot):
    path = create(tmp_path, snapshot)
    manifest = json.loads(path.read_text())
    manifest["schema_version"] = 1
    rewrite(path, manifest)
    with pytest.raises(ValueError, match="unsupported"):
        load_bundle(path)


@pytest.mark.parametrize("reference", [0.2, 2.0, 2.5])
def test_fixed_reference_build_load_and_actual_worker_bundle_job(tmp_path, snapshot, reference):
    """Real geometry build/load/worker: no mocked builder, loader or matcher."""
    calibration = {"frame_id": "camera", "K": [1., 0., 0., 0., 1., 0., 0., 0., 1.]}
    config = {"reference_speed_mps": reference}
    before = geometry_sha256(snapshot)
    path = build_bundle(tmp_path / "direct", snapshot, calibration, route_config=config)
    manifest, route, metadata, anchor, loaded_calibration = load_bundle(path)
    for payload in (manifest, route, metadata):
        assert payload["speed_policy"] == "fixed_reference"
        assert payload["reference_speed_mps"] == reference
    assert {row["speed_mps"] for row in route["route_samples"]} == {reference}
    assert np.array_equal(anchor, snapshot.consensus_xy)
    assert loaded_calibration == calibration
    assert geometry_sha256(snapshot) == before

    worker = MapWorker(lambda unused_job: nullcontext(True))
    try:
        # This is an explicit analytic frozen-map fixture, not a claimed camera
        # lap. The real background bundle branch performs all subsequent work.
        worker.snapshot = snapshot
        worker.calibration = calibration
        worker.submit({"kind": "bundle", "root": str(tmp_path / "worker"), "route_config": config})
        deadline = time.monotonic() + 5.0
        result = None
        while result is None and time.monotonic() < deadline:
            assert worker.failure is None
            result = worker.take()
            if result is None:
                time.sleep(0.002)
        assert result is not None, "real bundle worker timed out"
        assert result["kind"] == "bundle", result
        assert Path(result["path"]).is_file()
        assert result["route"]["reference_speed_mps"] == reference
        assert result["manifest"]["speed_policy"] == "fixed_reference"
        assert worker.matcher is not None
        assert load_bundle(result["path"])[1]["route_samples"] == route["route_samples"]
        assert geometry_sha256(worker.snapshot) == before
    finally:
        worker.close()
    assert not worker.thread.is_alive()


def test_loaded_fixed_candidate_still_requires_current_deployment_limits(tmp_path, snapshot):
    # Build accepts this .52 m/s^2 circle under its recorded .60 limit. It must
    # load as a candidate, then fail the separate deployment .35 limit.
    path = build_bundle(tmp_path, snapshot, {"frame_id": "camera"},
                        route_config={"reference_speed_mps": 2.5})
    _, route, _, _, _ = load_bundle(path)
    with pytest.raises(ValueError, match="fixed speed infeasible"):
        ClosedRouteController(route, reference_speed_mps=2.5, maximum_speed=2.5)


@pytest.mark.parametrize("payload", ["manifest", "metadata", "route"])
def test_fixed_reference_policy_mismatch_rejected_even_with_updated_hash(tmp_path, snapshot, payload):
    path = build_bundle(tmp_path, snapshot, {"frame_id": "camera"},
                        route_config={"reference_speed_mps": 2.0})
    manifest = json.loads(path.read_text())
    if payload == "manifest":
        manifest["reference_speed_mps"] = 0.8
    else:
        target = path.parent / manifest[payload + "_path"]
        body = json.loads(target.read_text())
        body["reference_speed_mps"] = 0.8
        rewrite(target, body)
        manifest[payload + "_sha256"] = hashlib.sha256(target.read_bytes()).hexdigest()
    rewrite(path, manifest)
    with pytest.raises(ValueError, match="speed policy/reference"):
        load_bundle(path)


def test_fixed_bundle_geometry_still_checked_after_payload_hash_update(tmp_path, snapshot):
    path = build_bundle(tmp_path, snapshot, {"frame_id": "camera"},
                        route_config={"reference_speed_mps": 2.0})
    manifest = json.loads(path.read_text())
    target = path.parent / manifest["route_path"]
    route = json.loads(target.read_text())
    route["route_samples"][5]["curvature_1pm"] += 0.1
    rewrite(target, route)
    manifest["route_sha256"] = hashlib.sha256(target.read_bytes()).hexdigest()
    rewrite(path, manifest)
    with pytest.raises(ValueError, match="curvature is inconsistent"):
        load_bundle(path)
