"""V4 permission persistence, not geometric quality or driving acceptance.

The old payload oracle is literal v1-v3 JSON layout, independent of the new
serializer's member lists. Route/controller behavior belongs to their existing
tests; only those two unrelated computations are stubbed here.
"""
from dataclasses import replace
import hashlib
import json

import numpy as np
import pytest

from lane_mapping_fixed_gnss import bundle
from lane_mapping_fixed_gnss.mapping import MapSnapshot
from lane_mapping_lya_reference.paint_geometry import BoundaryReference


CERT = "confirmed_creation_chart_domains_v1"
PERMISSION = "all_continuations_owned_quad_support_v1"
FIELDS = ("continuation_intervals", "continuation_primitive",
          "continuation_frame_votes", "continuation_reliable_votes",
          "continuation_last_stamp_ns")
BASE = {"points_xy": "<f8", "normals": "<f8", "support_segments": "<f8",
        "segment_owner": "<i8", "votes": "<i8", "reliable_votes": "<i8", "record_ids": "<i8"}
GRAPH = {"directed_edges": "<i8", "edge_frame_votes": "<i8",
         "edge_reliable_votes": "<i8", "edge_last_stamp_ns": "<i8",
         "full_in_degree": "<i8", "full_out_degree": "<i8"}
MODEL = {"certified_model_segments": "<f8", "certified_model_owner": "<i8",
         "certified_model_frame_votes": "<i8", "certified_model_reliable_votes": "<i8",
         "certified_model_last_stamp_ns": "<i8"}


def _reference(version=4, empty=False):
    points = np.array([[0., 1.], [1., 1.], [2., 1.]])
    extra = {}
    if version >= 2:
        extra.update(directed_edges=np.array([[0, 1], [1, 2]]),
                     edge_frame_votes=np.array([5, 6]), edge_reliable_votes=np.array([1, 1]),
                     edge_last_stamp_ns=np.array([90, 100]),
                     full_in_degree=np.array([0, 1, 1]), full_out_degree=np.array([1, 2, 0]))
    if version >= 3:
        extra.update(certified_model_segments=np.array([
            [[-.2, 1.], [-.1, 1.]], [[.1, 1.], [.2, 1.]], [[1.9, 1.], [2.1, 1.]]]),
            certified_model_owner=np.array([0, 0, 2]),
            certified_model_frame_votes=np.array([5, 5, 6]),
            certified_model_reliable_votes=np.array([1, 1, 2]),
            certified_model_last_stamp_ns=np.array([90, 99, 100]))
    if version == 4:
        extra.update(continuation_intervals=(np.empty((0, 2)) if empty else
                     np.array([[0., .25], [.5, .75], [.125, 1.]])),
                     continuation_primitive=np.array([] if empty else [0, 0, 2], dtype=np.int64),
                     continuation_frame_votes=np.array([] if empty else [5, 5, 6], dtype=np.int64),
                     continuation_reliable_votes=np.array([] if empty else [1, 1, 2], dtype=np.int64),
                     continuation_last_stamp_ns=np.array([] if empty else [90, 99, 100], dtype=np.int64))
    return BoundaryReference(points, np.tile([0., 1.], (3, 1)),
        np.stack((points-[.1, 0.], points+[.1, 0.]), axis=1), np.arange(3),
        np.full(3, 8), np.full(3, 3), np.array([4, 8, 19]), 100, **extra)


def _snapshot(ref):
    return MapSnapshot(np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]]),
                       np.array([[0., 1.1], [1., 1.1], [2., 1.1]]), np.full(3, 5.),
                       100, {"accepted": 10}, boundary_reference=ref)


def _json(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)+"\n").encode("utf8")


@pytest.fixture
def create(tmp_path, monkeypatch):
    monkeypatch.setattr(bundle, "build_route", lambda *args: dict(schema_version=1, closed=True,
        diagnostics={"valid": True}, speed_policy="profile", reference_speed_mps=0.))
    monkeypatch.setattr(bundle, "ClosedRouteController", lambda *args, **kwargs: object())
    return lambda ref: bundle.build_bundle(tmp_path, _snapshot(ref), {"frame": "camera"}, session_id="continuation")


def _resign(path, name, mutation):
    manifest = json.loads(path.read_text(encoding="utf8"))
    if name == "manifest":
        mutation(manifest)
    else:
        target = path.parent / manifest[name+"_path"]
        value = json.loads(target.read_text(encoding="utf8"))
        mutation(value)
        data = _json(value)
        target.write_bytes(data)
        manifest[name+"_sha256"] = hashlib.sha256(data).hexdigest()
    path.write_bytes(_json(manifest))


def _rehash(record):
    record["sha256"] = hashlib.sha256(np.asarray(record["data"], dtype=record["dtype"]).tobytes()).hexdigest()


@pytest.mark.parametrize("empty", [False, True])
def test_v4_roundtrip_is_explicit_frozen_and_does_not_rebuild_geometry(create, empty):
    ref = _reference(empty=empty)
    before = bundle.geometry_sha256(_snapshot(ref))
    path = create(ref)
    manifest, route, metadata, filled, calibration = bundle.load_bundle(path)
    for value in (manifest, route, metadata):
        assert value["boundary_version"] == 4
        assert value["certified_model_representation"] == CERT
        assert value["continuation_representation"] == PERMISSION
    assert manifest["schema_version"] == metadata["schema_version"] == 3
    assert bundle.MAX_PAYLOAD_BYTES == 16 * 1024 * 1024
    assert calibration == {"frame": "camera"}
    loaded = metadata["boundary_reference"]
    assert loaded.boundary_version == 4 and loaded.continuation_representation == PERMISSION
    assert loaded.source_frontier_ns == 100
    for name in tuple(BASE) + tuple(GRAPH) + tuple(MODEL) + FIELDS:
        original, actual = getattr(ref, name), getattr(loaded, name)
        assert actual.shape == original.shape and actual.dtype == original.dtype
        assert actual.tobytes() == original.tobytes()
        with pytest.raises(ValueError):
            actual.setflags(write=True)
    assert not np.array_equal(filled, loaded.points_xy)
    assert bundle.geometry_sha256(_snapshot(loaded)) == before
    payload = json.loads((path.parent / "boundary.json").read_text(encoding="utf8"))
    assert len(payload["arrays"]) == 23
    assert payload["continuation_representation"] == PERMISSION
    if not empty:
        # Unpermitted gaps and an entirely unpermitted primitive survive unchanged.
        assert loaded.continuation_intervals[0, 1] < loaded.continuation_intervals[1, 0]
        assert 1 not in loaded.continuation_primitive
        assert len(loaded.certified_model_segments) == 3


@pytest.mark.parametrize("version", [1, 2, 3])
def test_old_v1_v2_v3_bytes_and_geometry_digest_are_literal_prior_contract(create, version):
    ref = _reference(version)
    specs = dict(BASE)
    if version >= 2:
        specs.update(GRAPH)
    if version >= 3:
        specs.update(MODEL)
    arrays = {}
    for name, dtype in specs.items():
        value = np.asarray(getattr(ref, name), dtype=dtype)
        arrays[name] = {"dtype": dtype, "shape": list(value.shape),
                        "sha256": hashlib.sha256(value.tobytes()).hexdigest(), "data": value.tolist()}
    expected = {"representation": "paint_boundary_v1", "version": version,
                "source_frontier_ns": 100, "arrays": arrays}
    if version == 3:
        expected["certified_model_representation"] = CERT
    data = _json(expected)
    frozen = _snapshot(ref)
    digest = hashlib.sha256()
    for value in (frozen.trace, frozen.consensus_xy, frozen.votes):
        value = np.asarray(value, dtype="<f8")
        digest.update(str(value.shape).encode("ascii"))
        digest.update(value.tobytes())
    digest.update(data)
    assert bundle.geometry_sha256(frozen) == digest.hexdigest()
    path = create(ref)
    assert (path.parent / "boundary.json").read_bytes() == data
    manifest, route, metadata, _, _ = bundle.load_bundle(path)
    for payload in (manifest, route, metadata):
        assert "continuation_representation" not in payload
        assert payload["boundary_version"] == version
    loaded = metadata["boundary_reference"]
    assert loaded.boundary_version == version and loaded.continuation_representation is None
    assert all(getattr(loaded, name) is None for name in FIELDS)


@pytest.mark.parametrize("name", FIELDS)
def test_each_continuation_member_has_individual_and_full_snapshot_hash(create, name):
    ref = _reference()
    changed = getattr(ref, name).copy()
    if name == "continuation_intervals":
        changed[0, 1] = .2
    elif name == "continuation_primitive":
        changed[-1] = 1
    else:
        changed[0] += 1
    modified = replace(ref, **{name: changed})
    assert bundle.geometry_sha256(_snapshot(ref)) != bundle.geometry_sha256(_snapshot(modified))
    path = create(ref)
    _resign(path, "boundary", lambda value: value["arrays"][name].update(data=changed.tolist()))
    with pytest.raises(ValueError, match="checksum"):
        bundle.load_bundle(path)


def test_rehashed_valid_permission_cannot_change_frozen_snapshot(create):
    path = create(_reference())
    def mutation(value):
        record = value["arrays"]["continuation_intervals"]
        record["data"][0][1] = .2
        _rehash(record)
    _resign(path, "boundary", mutation)
    with pytest.raises(ValueError, match="geometry integrity"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("name", FIELDS)
def test_missing_or_mixed_member_set_is_never_legacy_fallback(create, name):
    path = create(_reference())
    _resign(path, "boundary", lambda value: value["arrays"].pop(name))
    with pytest.raises(ValueError, match="member set"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("payload", ["manifest", "route", "metadata", "boundary"])
@pytest.mark.parametrize("marker", [None, "", "latest_face_extent", "confirmed_creation_chart_domains_v1"])
def test_explicit_v4_marker_required_everywhere(create, payload, marker):
    path = create(_reference())
    _resign(path, payload, lambda value: value.update(continuation_representation=marker))
    with pytest.raises(ValueError, match="representation"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("payload", ["manifest", "route", "metadata", "boundary"])
def test_missing_v4_marker_is_not_null_or_inferred(create, payload):
    path = create(_reference())
    _resign(path, payload, lambda value: value.pop("continuation_representation"))
    with pytest.raises(ValueError, match="representation"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("version", [1, 2, 3])
@pytest.mark.parametrize("payload", ["manifest", "route", "metadata", "boundary"])
def test_older_versions_cannot_claim_continuation_permission(create, version, payload):
    path = create(_reference(version))
    _resign(path, payload, lambda value: value.update(continuation_representation=PERMISSION))
    with pytest.raises(ValueError):
        bundle.load_bundle(path)


@pytest.mark.parametrize("name,values", [
    ("continuation_intervals", [[-.01, .25], [.5, .75], [.125, 1.]]),
    ("continuation_intervals", [[0., 1.01], [.5, .75], [.125, 1.]]),
    ("continuation_intervals", [[.25, .25], [.5, .75], [.125, 1.]]),
    ("continuation_intervals", [[.5, .75], [0., .25], [.125, 1.]]),
    ("continuation_intervals", [[0., .5], [.5, .75], [.125, 1.]]),
    ("continuation_intervals", [[0., .6], [.5, .75], [.125, 1.]]),
    ("continuation_primitive", [0, 0, 3]),
    ("continuation_primitive", [1, 0, 2]),
    ("continuation_frame_votes", [0, 5, 6]),
    ("continuation_frame_votes", [9, 5, 6]),
    ("continuation_reliable_votes", [0, 1, 2]),
    ("continuation_reliable_votes", [4, 1, 2]),
    ("continuation_last_stamp_ns", [0, 99, 100]),
    ("continuation_last_stamp_ns", [101, 99, 100]),
])
def test_rehashed_invalid_permissions_reach_real_reference_validation(create, name, values):
    path = create(_reference())
    def mutation(value):
        record = value["arrays"][name]
        record["data"] = values
        _rehash(record)
    _resign(path, "boundary", mutation)
    with pytest.raises(ValueError, match="continuation"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("name", FIELDS[1:])
@pytest.mark.parametrize("bad", [True, 1.5, "1", -1, 2**63, 2**100])
def test_integer_json_types_and_range_not_silently_coerced(create, name, bad):
    path = create(_reference())
    _resign(path, "boundary", lambda value: value["arrays"][name]["data"].__setitem__(0, bad))
    with pytest.raises(ValueError):
        bundle.load_bundle(path)


@pytest.mark.parametrize("bad", [True, "0.25", None])
def test_interval_scalars_remain_numbers_not_booleans_or_strings(create, bad):
    path = create(_reference())
    _resign(path, "boundary", lambda value: value["arrays"]["continuation_intervals"]["data"][0].__setitem__(1, bad))
    with pytest.raises(ValueError, match="scalar types"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_interval_json_never_coerced(create, bad):
    path = create(_reference())
    manifest = json.loads(path.read_text(encoding="utf8"))
    target = path.parent / manifest["boundary_path"]
    value = json.loads(target.read_text(encoding="utf8"))
    value["arrays"]["continuation_intervals"]["data"][0][1] = bad
    data = json.dumps(value).encode("utf8")
    target.write_bytes(data)
    manifest["boundary_sha256"] = hashlib.sha256(data).hexdigest()
    path.write_bytes(_json(manifest))
    with pytest.raises(ValueError, match="nonfinite"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("name", FIELDS)
def test_capacity_layout_rejected_before_data_allocation(create, name):
    path = create(_reference())
    suffix = [2] if name == "continuation_intervals" else []
    _resign(path, "boundary", lambda value: value["arrays"][name].update(shape=[200001]+suffix))
    with pytest.raises(ValueError, match="layout"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("payload", ["route", "metadata"])
def test_mixed_version_payload_is_rejected(create, payload):
    path = create(_reference())
    def mutation(value):
        value["boundary_version"] = 3
        value.pop("continuation_representation")
    _resign(path, payload, mutation)
    with pytest.raises(ValueError, match="version differs"):
        bundle.load_bundle(path)


def test_frontier_mismatch_is_not_repaired_from_metadata(create):
    path = create(_reference())
    _resign(path, "boundary", lambda value: value.update(source_frontier_ns=99))
    with pytest.raises(ValueError, match="frontier"):
        bundle.load_bundle(path)


def test_subset_rebinds_primitive_ids_but_not_geometry_or_permission_spans(create):
    ref = _reference().subset([2, 1])
    assert ref.certified_model_owner.tolist() == [0]
    assert ref.continuation_primitive.tolist() == [0]
    assert ref.continuation_intervals.tolist() == [[.125, 1.]]
    loaded = bundle.load_bundle(create(ref))[2]["boundary_reference"]
    for name in FIELDS + ("certified_model_segments", "certified_model_owner", "full_in_degree", "full_out_degree"):
        assert getattr(ref, name).tobytes() == getattr(loaded, name).tobytes()
    assert loaded.full_out_degree.tolist() == [0, 2]


def test_empty_permissions_are_distinct_from_legacy_no_permissions(create):
    old, explicit_empty = _reference(3), _reference(4, empty=True)
    assert bundle.geometry_sha256(_snapshot(old)) != bundle.geometry_sha256(_snapshot(explicit_empty))
    loaded = bundle.load_bundle(create(explicit_empty))[2]["boundary_reference"]
    assert loaded.boundary_version == 4
    assert loaded.continuation_intervals.shape == (0, 2)
    assert loaded.continuation_representation == PERMISSION
