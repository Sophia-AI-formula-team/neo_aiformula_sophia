"""Versioned finite-model certificates; persistence is not driving admission."""
from dataclasses import replace
import hashlib
import json

import numpy as np
import pytest

from lane_mapping_fixed_gnss import bundle
from lane_mapping_fixed_gnss.mapping import MapSnapshot
from lane_mapping_lya_reference.paint_geometry import BoundaryReference


CERT = "confirmed_creation_chart_domains_v1"
FIELDS = ("certified_model_segments", "certified_model_owner",
          "certified_model_frame_votes", "certified_model_reliable_votes",
          "certified_model_last_stamp_ns")
OLD_BASE = {"points_xy": "<f8", "normals": "<f8", "support_segments": "<f8",
            "segment_owner": "<i8", "votes": "<i8", "reliable_votes": "<i8", "record_ids": "<i8"}
OLD_GRAPH = {"directed_edges": "<i8", "edge_frame_votes": "<i8",
             "edge_reliable_votes": "<i8", "edge_last_stamp_ns": "<i8",
             "full_in_degree": "<i8", "full_out_degree": "<i8"}


def reference(version=3, empty=False):
    p = np.array([[0., 1.], [1., 1.], [2., 1.]])
    kwargs = {}
    if version >= 2:
        kwargs.update(directed_edges=np.array([[0, 1], [1, 2]]),
                      edge_frame_votes=np.array([5, 6]), edge_reliable_votes=np.array([1, 1]),
                      edge_last_stamp_ns=np.array([90, 100]),
                      full_in_degree=np.array([0, 1, 1]), full_out_degree=np.array([1, 2, 0]))
    if version == 3:
        kwargs.update(certified_model_segments=(np.empty((0, 2, 2)) if empty else
            np.array([[[-.2, 1.], [-.1, 1.]], [[.1, 1.], [.2, 1.]], [[1.9, 1.], [2.1, 1.]]])),
            certified_model_owner=np.array([] if empty else [0, 0, 2], dtype=np.int64),
            certified_model_frame_votes=np.array([] if empty else [5, 5, 6], dtype=np.int64),
            certified_model_reliable_votes=np.array([] if empty else [1, 1, 2], dtype=np.int64),
            certified_model_last_stamp_ns=np.array([] if empty else [90, 99, 100], dtype=np.int64))
    return BoundaryReference(p, np.tile([0., 1.], (3, 1)), np.stack((p-[.1, 0.], p+[.1, 0.]), axis=1),
                             np.arange(3), np.full(3, 8), np.full(3, 3), np.array([4, 8, 19]), 100, **kwargs)


def snapshot(ref):
    return MapSnapshot(np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]]),
                       np.array([[0., 1.1], [1., 1.1], [2., 1.1]]), np.full(3, 5.),
                       100, {"accepted": 10}, boundary_reference=ref)


@pytest.fixture
def create(tmp_path, monkeypatch):
    # Isolate persistence, keeping route feasibility in the existing route suite.
    monkeypatch.setattr(bundle, "build_route", lambda *args: dict(schema_version=1, closed=True,
        diagnostics={"valid": True}, speed_policy="profile", reference_speed_mps=0.))
    monkeypatch.setattr(bundle, "ClosedRouteController", lambda *args, **kwargs: object())
    return lambda ref: bundle.build_bundle(tmp_path, snapshot(ref), {"frame": "camera"}, session_id="certified")


def resign(path, name, mutate):
    manifest = json.loads(path.read_text())
    target = path.parent / manifest[name + "_path"]
    value = json.loads(target.read_text())
    mutate(value)
    data = json.dumps(value).encode()
    target.write_bytes(data)
    manifest[name + "_sha256"] = hashlib.sha256(data).hexdigest()
    path.write_text(json.dumps(manifest))


def rehash_array(record):
    record["sha256"] = hashlib.sha256(np.asarray(record["data"], dtype=record["dtype"]).tobytes()).hexdigest()


@pytest.mark.parametrize("empty", [False, True])
def test_v3_roundtrip_frozen_fields_and_gap_no_outer_schema_change(create, empty):
    original = reference(empty=empty)
    path = create(original)
    manifest, route, metadata, filled, _ = bundle.load_bundle(path)
    for payload in (manifest, route, metadata):
        assert payload["boundary_version"] == 3
        assert payload["certified_model_representation"] == CERT
    assert manifest["schema_version"] == metadata["schema_version"] == 3
    assert bundle.MAX_PAYLOAD_BYTES == 16 * 1024 * 1024
    loaded = metadata["boundary_reference"]
    assert loaded.boundary_version == 3 and loaded.certified_model_representation == CERT
    assert loaded.source_frontier_ns == 100
    for name in tuple(OLD_BASE) + tuple(OLD_GRAPH) + FIELDS:
        assert getattr(loaded, name).tobytes() == getattr(original, name).tobytes()
        with pytest.raises(ValueError):
            getattr(loaded, name).setflags(write=True)
    assert not np.array_equal(filled, loaded.points_xy)
    if not empty:
        assert loaded.certified_model_segments[0, 1, 0] < loaded.certified_model_segments[1, 0, 0]
    assert len(json.loads((path.parent / "boundary.json").read_text())["arrays"]) == 18


@pytest.mark.parametrize("version", [1, 2])
def test_old_payload_and_geometry_digest_match_literal_prior_layout(create, version):
    ref = reference(version)
    arrays = {}
    for name, dtype in dict(OLD_BASE, **(OLD_GRAPH if version == 2 else {})).items():
        a = np.asarray(getattr(ref, name), dtype=dtype)
        arrays[name] = dict(dtype=dtype, shape=list(a.shape), sha256=hashlib.sha256(a.tobytes()).hexdigest(), data=a.tolist())
    old_payload = dict(representation="paint_boundary_v1", version=version, source_frontier_ns=100, arrays=arrays)
    old_bytes = (json.dumps(old_payload, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)+"\n").encode()
    frozen = snapshot(ref)
    digest = hashlib.sha256()
    for a in (frozen.trace, frozen.consensus_xy, frozen.votes):
        a = np.asarray(a, dtype="<f8")
        digest.update(str(a.shape).encode("ascii")); digest.update(a.tobytes())
    digest.update(old_bytes)
    assert bundle.geometry_sha256(frozen) == digest.hexdigest()
    path = create(ref)
    assert (path.parent / "boundary.json").read_bytes() == old_bytes
    manifest, route, metadata, _, _ = bundle.load_bundle(path)
    for payload in (manifest, route, metadata):
        assert "certified_model_representation" not in payload
    assert all(getattr(metadata["boundary_reference"], name) is None for name in FIELDS)


@pytest.mark.parametrize("name", FIELDS)
def test_member_checksum_and_full_geometry_hash_cover_every_certificate(create, name):
    ref = reference()
    changed = getattr(ref, name).copy()
    changed.flat[0] += .01 if name == "certified_model_segments" else 1
    modified = replace(ref, **{name: changed})
    assert bundle.geometry_sha256(snapshot(ref)) != bundle.geometry_sha256(snapshot(modified))
    path = create(ref)
    def mutate(value):
        value["arrays"][name]["data"] = changed.tolist()
    resign(path, "boundary", mutate)
    with pytest.raises(ValueError, match="checksum"):
        bundle.load_bundle(path)


def test_rehashed_valid_domain_still_fails_frozen_snapshot_identity(create):
    path = create(reference())
    def mutate(value):
        record = value["arrays"]["certified_model_segments"]
        record["data"][0][0][0] -= .01
        rehash_array(record)
    resign(path, "boundary", mutate)
    with pytest.raises(ValueError, match="geometry integrity"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("name", FIELDS)
def test_missing_certificate_member_never_falls_back(create, name):
    path = create(reference())
    resign(path, "boundary", lambda value: value["arrays"].pop(name))
    with pytest.raises(ValueError, match="member set"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("name,values", [
    ("certified_model_owner", [3, 0, 2]),
    ("certified_model_frame_votes", [9, 5, 6]),
    ("certified_model_reliable_votes", [0, 1, 2]),
    ("certified_model_last_stamp_ns", [101, 99, 100]),
    ("certified_model_last_stamp_ns", [0, 99, 100]),
    ("certified_model_segments", [[[0., 1.], [0., 1.]], [[.1, 1.], [.2, 1.]], [[1.9, 1.], [2.1, 1.]]])])
def test_rehashed_invalid_certificates_rejected_by_reference(create, name, values):
    path = create(reference())
    def mutate(value):
        record = value["arrays"][name]
        record["data"] = values
        rehash_array(record)
    resign(path, "boundary", mutate)
    with pytest.raises(ValueError, match="certified"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("name", FIELDS[1:])
@pytest.mark.parametrize("bad", [True, 1.5, -1, 2**63])
def test_certified_integer_json_never_coerces(create, name, bad):
    path = create(reference())
    resign(path, "boundary", lambda value: value["arrays"][name]["data"].__setitem__(0, bad))
    with pytest.raises(ValueError):
        bundle.load_bundle(path)


@pytest.mark.parametrize("payload", ["boundary", "route", "metadata"])
@pytest.mark.parametrize("value", [None, "latest_face_extent", ""])
def test_v3_representation_required_in_each_payload(create, payload, value):
    path = create(reference())
    resign(path, payload, lambda obj: obj.update(certified_model_representation=value))
    with pytest.raises(ValueError, match="representation"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("version", [1, 2])
def test_legacy_boundary_cannot_claim_certificates(create, version):
    path = create(reference(version))
    resign(path, "metadata", lambda value: value.update(certified_model_representation=CERT))
    with pytest.raises(ValueError, match="legacy boundary"):
        bundle.load_bundle(path)


def test_subset_domain_ownership_and_full_degrees_survive_roundtrip(create):
    original = reference().subset([2, 0, 1])
    assert original.certified_model_owner.tolist() == [1, 1, 0]
    loaded = bundle.load_bundle(create(original))[2]["boundary_reference"]
    for name in FIELDS + ("full_in_degree", "full_out_degree"):
        assert getattr(original, name).tobytes() == getattr(loaded, name).tobytes()


def test_payload_capacity_remains_bounded_before_allocation(create):
    path = create(reference())
    resign(path, "boundary", lambda value: value["arrays"]["certified_model_segments"].update(shape=[200001, 2, 2]))
    with pytest.raises(ValueError, match="layout"):
        bundle.load_bundle(path)
