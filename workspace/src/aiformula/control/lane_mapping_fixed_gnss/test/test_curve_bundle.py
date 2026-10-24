"""Portable synthetic v5 save/load tests, prepared without execution.

Real bundle functions and fresh real matchers are used. The closed circle is an
analytic safe route fixture, not a mask/odometry lap or vehicle acceptance claim.
Legacy payload expectations use literal old field sets, not the new serializer.
"""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss import bundle as subject
from lane_mapping_fixed_gnss.mapping import MapSnapshot
from lane_mapping_lya_reference.boundary_localization import BoundaryLaneLocalizer
from lane_mapping_lya_reference.curve_reference import CurveGeometry
from lane_mapping_lya_reference.paint_geometry import BoundaryReference


FRONTIER = 10_000_000_000
STAMP = 8_000_000_000
CURVE_REPRESENTATION = "finite_creation_chart_curve_consensus_v1"
CONTINUATION_REPRESENTATION = "all_continuations_owned_quad_support_v1"
BASE = ("points_xy", "normals", "support_segments", "segment_owner", "votes",
        "reliable_votes", "record_ids")
GRAPH = ("directed_edges", "edge_frame_votes", "edge_reliable_votes",
         "edge_last_stamp_ns", "full_in_degree", "full_out_degree")
CERTIFIED = ("certified_model_segments", "certified_model_owner",
             "certified_model_frame_votes", "certified_model_reliable_votes",
             "certified_model_last_stamp_ns")
CONTINUATION = ("continuation_intervals", "continuation_primitive",
                "continuation_frame_votes", "continuation_reliable_votes",
                "continuation_last_stamp_ns")
CURVE = ("segments", "normals", "owner", "frame_votes", "reliable_votes",
         "last_stamp_ns", "owner_status", "blocker_boxes", "blocker_owner")
FLOAT_FIELDS = {"points_xy", "normals", "support_segments", "certified_model_segments",
                "continuation_intervals", "curve_segments", "curve_normals", "curve_blocker_boxes"}


def encode(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2,
                       allow_nan=False) + "\n").encode("utf-8")


def boundary(version=5, *, continuation="full", complete=True):
    extra = {}
    if version >= 2:
        extra.update(directed_edges=np.array([[0, 1]], dtype=np.int64),
            edge_frame_votes=np.array([5]), edge_reliable_votes=np.array([1]),
            edge_last_stamp_ns=np.array([STAMP]), full_in_degree=np.array([0, 1, 0]),
            full_out_degree=np.array([1, 0, 0]))
    if version in (3, 4):
        extra.update(certified_model_segments=np.array([[[-1., .125], [1., .125]]]),
            certified_model_owner=np.array([0]), certified_model_frame_votes=np.array([5]),
            certified_model_reliable_votes=np.array([1]), certified_model_last_stamp_ns=np.array([STAMP]))
    if version == 5:
        extra["curve_geometry"] = CurveGeometry(
            np.array([[[-1., .125], [1., .125]]]), np.array([[0., 1.]]),
            np.array([0]), np.array([5]), np.array([1]), np.array([STAMP]),
            np.array([2, 0, 1]), np.array([[[-1., 0.], [1., 0.]], [[2., 2.], [3., 2.]]]),
            np.array([1, 2]), complete)
    if version == 4 or version == 5 and continuation != "none":
        spans = [(0.25, 0.75)] if continuation == "full" else []
        count = len(spans)
        extra.update(continuation_intervals=np.array(spans, dtype=np.float64).reshape(-1, 2),
            continuation_primitive=np.zeros(count, dtype=np.int64),
            continuation_frame_votes=np.full(count, 5, dtype=np.int64),
            continuation_reliable_votes=np.ones(count, dtype=np.int64),
            continuation_last_stamp_ns=np.full(count, STAMP, dtype=np.int64))
        if continuation == "full":
            extra.update(full_in_degree=np.array([2, 1, 0]), full_out_degree=np.array([2, 0, 0]))
    return BoundaryReference(np.array([[-1., 0.], [1., 0.], [0., 2.]]),
        np.tile([0., 1.], (3, 1)), np.array([[[-1., 0.], [1., 0.]]]),
        np.array([0]), np.full(3, 10, dtype=np.int64), np.full(3, 5, dtype=np.int64),
        np.array([11, 19, 23]), FRONTIER, **extra)


def load_payload(payload, version=5):
    return subject._load_boundary(encode(payload), FRONTIER, version)


def assert_arrays_equal(original, loaded):
    assert loaded.boundary_version == original.boundary_version
    assert loaded.source_frontier_ns == original.source_frontier_ns
    for name in BASE + GRAPH + CERTIFIED + CONTINUATION:
        first, second = getattr(original, name), getattr(loaded, name)
        if first is None:
            assert second is None, name
        else:
            assert first.dtype == second.dtype and first.shape == second.shape, name
            assert first.tobytes() == second.tobytes(), name
            assert not second.flags.writeable, name
    if original.curve_geometry is None:
        assert loaded.curve_geometry is None
    else:
        assert loaded.curve_geometry is not original.curve_geometry
        assert loaded.curve_geometry.complete is original.curve_geometry.complete
        for name in CURVE:
            first, second = getattr(original.curve_geometry, name), getattr(loaded.curve_geometry, name)
            assert first.dtype == second.dtype and first.shape == second.shape, name
            assert first.tobytes() == second.tobytes(), name
            assert not second.flags.writeable, name


def query(reference):
    matcher = BoundaryLaneLocalizer(reference)
    points = np.array([[0., .1875], [0., .0625], [2., .1875], [-.875, .1875]])
    output = matcher._correspondences(points, np.zeros(3), np.tile([0., 1.], (len(points), 1)))
    return matcher, output


def snapshot(reference):
    radius = 12.
    angle = np.linspace(-math.pi / 2., 3. * math.pi / 2., 501)
    trace = np.column_stack((radius * np.cos(angle), radius + radius * np.sin(angle),
                             angle + math.pi / 2.))
    lane_angle = np.linspace(-math.pi / 2., 3. * math.pi / 2., 2400, endpoint=False)
    lanes = np.vstack([np.column_stack((r * np.cos(lane_angle), radius + r * np.sin(lane_angle)))
                       for r in (radius - 1.2, radius + 1.2)])
    return MapSnapshot(trace, lanes, np.full(len(lanes), 5.), FRONTIER,
                       {"synthetic_fixture_only": True}, boundary_reference=reference)


def build(tmp_path, reference):
    frozen = snapshot(reference)
    path = subject.build_bundle(tmp_path, frozen,
        {"frame_id": "camera", "K": [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
        route_config={"reference_speed_mps": .8}, session_id="synthetic_curve_test")
    return path, frozen


def literal_legacy_payload(reference, version):
    names = BASE + (GRAPH if version >= 2 else ()) + (CERTIFIED if version >= 3 else ())
    names += CONTINUATION if version == 4 else ()
    arrays = {}
    for name in names:
        dtype = "<f8" if name in FLOAT_FIELDS else "<i8"
        value = np.asarray(getattr(reference, name), dtype=dtype)
        arrays[name] = {"dtype": dtype, "shape": list(value.shape),
                        "sha256": hashlib.sha256(value.tobytes()).hexdigest(), "data": value.tolist()}
    payload = {"representation": "paint_boundary_v1", "version": version,
               "source_frontier_ns": FRONTIER, "arrays": arrays}
    if version >= 3:
        payload["certified_model_representation"] = "confirmed_creation_chart_domains_v1"
    if version == 4:
        payload["continuation_representation"] = CONTINUATION_REPRESENTATION
    return payload


def rehash_member(record):
    value = np.asarray(record["data"], dtype=record["dtype"]).reshape(record["shape"])
    record["sha256"] = hashlib.sha256(value.tobytes()).hexdigest()


def test_v5_payload_contains_every_curve_array_status_and_hash():
    reference = boundary()
    payload = subject._boundary_payload(reference)
    expected = set(BASE + GRAPH + CONTINUATION + tuple("curve_" + name for name in CURVE))
    assert set(payload["arrays"]) == expected
    assert payload["version"] == 5 and payload["curve_complete"] is True
    assert payload["curve_representation"] == CURVE_REPRESENTATION
    assert "certified_model_representation" not in payload
    for name, record in payload["arrays"].items():
        source = getattr(reference.curve_geometry, name[6:]) if name.startswith("curve_") else getattr(reference, name)
        canonical = np.asarray(source, dtype="<f8" if name in FLOAT_FIELDS else "<i8")
        assert record["sha256"] == hashlib.sha256(canonical.tobytes()).hexdigest(), name
        assert record["shape"] == list(canonical.shape), name
    assert payload["arrays"]["curve_owner_status"]["data"] == [2, 0, 1]
    assert payload["arrays"]["curve_blocker_owner"]["data"] == [1, 2]


def test_v5_payload_roundtrip_fresh_matcher_is_byte_exact_and_not_a_chord():
    original = boundary()
    loaded = load_payload(subject._boundary_payload(original))
    assert_arrays_equal(original, loaded)
    before, first = query(original)
    fresh, second = query(loaded)
    assert before is not fresh and before._index is not fresh._index
    for expected, actual in zip(first, second):
        assert expected.dtype == actual.dtype and expected.tobytes() == actual.tobytes()
    np.testing.assert_array_equal(first[2], [0])
    np.testing.assert_array_equal(first[4], [.0625])  # curve y.125, not mean-node chord y0
    unresolved = boundary(complete=False)
    unresolved_loaded = load_payload(subject._boundary_payload(unresolved))
    assert_arrays_equal(unresolved, unresolved_loaded)
    assert unresolved_loaded.curve_geometry.complete is False
    assert not len(query(unresolved_loaded)[1][2])
    for expected, actual in zip(query(unresolved)[1], query(unresolved_loaded)[1]):
        assert expected.tobytes() == actual.tobytes()


def test_continuation_none_empty_and_present_are_distinct_roundtrips():
    payloads = []
    for mode in ("none", "empty", "full"):
        original = boundary(continuation=mode)
        payload = subject._boundary_payload(original)
        loaded = load_payload(payload)
        assert_arrays_equal(original, loaded)
        payloads.append(encode(payload))
        if mode == "none":
            assert payload["continuation_representation"] is None
            assert not set(CONTINUATION) & set(payload["arrays"])
            assert loaded.continuation_intervals is None
        else:
            assert payload["continuation_representation"] == CONTINUATION_REPRESENTATION
            assert set(CONTINUATION) <= set(payload["arrays"])
            assert len(loaded.continuation_intervals) == (0 if mode == "empty" else 1)
        for first, second in zip(query(original)[1], query(loaded)[1]):
            assert first.tobytes() == second.tobytes()
    assert len(set(payloads)) == 3


def test_every_serialized_array_checksum_is_enforced():
    payload = subject._boundary_payload(boundary())
    for name in BASE + GRAPH + CONTINUATION + tuple("curve_" + field for field in CURVE):
        changed = deepcopy(payload)
        changed["arrays"][name]["sha256"] = "0" * 64
        with pytest.raises(ValueError, match="checksum"):
            load_payload(changed)


def test_missing_any_member_or_unexpected_member_is_rejected():
    payload = subject._boundary_payload(boundary())
    for name in BASE + GRAPH + CONTINUATION + tuple("curve_" + field for field in CURVE):
        changed = deepcopy(payload)
        del changed["arrays"][name]
        with pytest.raises(ValueError, match="member set"):
            load_payload(changed)
    changed = deepcopy(payload)
    changed["arrays"]["certified_model_segments"] = deepcopy(changed["arrays"]["curve_segments"])
    with pytest.raises(ValueError, match="member set"):
        load_payload(changed)


def test_rehashed_invalid_types_layout_and_semantics_still_reject():
    payload = subject._boundary_payload(boundary())
    for name, value in (("curve_owner", [3]), ("curve_frame_votes", [4]),
                        ("curve_last_stamp_ns", [FRONTIER + 1]),
                        ("curve_normals", [[1., 0.]]), ("curve_owner_status", [2, 3, 1])):
        changed = deepcopy(payload)
        changed["arrays"][name]["data"] = value
        rehash_member(changed["arrays"][name])
        with pytest.raises(ValueError):
            load_payload(changed)
    for field, value in (("dtype", ">f8"), ("shape", [True, 2, 2])):
        changed = deepcopy(payload)
        changed["arrays"]["curve_segments"][field] = value
        with pytest.raises(ValueError):
            load_payload(changed)
    changed = deepcopy(payload)
    changed["arrays"]["curve_owner_status"]["data"][0] = True
    rehash_member(changed["arrays"]["curve_owner_status"])
    with pytest.raises(ValueError, match="scalar types"):
        load_payload(changed)
    changed = deepcopy(payload)
    changed["arrays"]["curve_segments"]["data"][0][0][0] = float("nan")
    with pytest.raises(ValueError, match="nonfinite"):
        subject._load_boundary(json.dumps(changed).encode("utf-8"), FRONTIER, 5)


def test_rehashed_v5_permissions_topology_and_ready_status_require_five_plus_one():
    payload = subject._boundary_payload(boundary())
    changes = [("continuation_frame_votes", [votes]) for votes in range(1, 5)]
    changes += [("edge_frame_votes", [votes]) for votes in range(1, 5)]
    changes += [("edge_reliable_votes", [0])]
    for name, data in changes:
        changed = deepcopy(payload)
        changed["arrays"][name]["data"] = data
        rehash_member(changed["arrays"][name])
        with pytest.raises(ValueError):
            load_payload(changed)
    # Owner2 has no exported curve or edge: exercise status evidence alone,
    # without accidentally failing an unrelated endpoint/curve vote check.
    for votes, reliable in ((4, 1), (5, 0)):
        changed = deepcopy(payload)
        for name, value in (("curve_owner_status", 2), ("votes", votes),
                            ("reliable_votes", reliable)):
            changed["arrays"][name]["data"][2] = value
            rehash_member(changed["arrays"][name])
        with pytest.raises(ValueError):
            load_payload(changed)
    # Pending and UNKNOWN owners need not meet confirmation: retain them.
    for status in (0, 1):
        changed = deepcopy(payload)
        for name, value in (("curve_owner_status", status), ("votes", 1),
                            ("reliable_votes", 0)):
            changed["arrays"][name]["data"][2] = value
            rehash_member(changed["arrays"][name])
        restored = load_payload(changed)
        assert restored.curve_geometry.owner_status.tolist()[2] == status
        assert restored.votes.tolist()[2] == 1
    for version in (2, 3, 4):
        old = subject._boundary_payload(boundary(version))
        for name, data in (("edge_frame_votes", [4]), ("edge_reliable_votes", [0])):
            changed = deepcopy(old)
            changed["arrays"][name]["data"] = data
            rehash_member(changed["arrays"][name])
            assert load_payload(changed, version).boundary_version == version
        if version == 4:
            changed = deepcopy(old)
            changed["arrays"]["continuation_frame_votes"]["data"] = [4]
            rehash_member(changed["arrays"]["continuation_frame_votes"])
            assert load_payload(changed, version).boundary_version == 4


def test_version_markers_complete_and_outer_schema_cannot_be_omitted_or_faked():
    payload = subject._boundary_payload(boundary())
    for key in ("version", "curve_representation", "curve_complete", "continuation_representation"):
        changed = deepcopy(payload)
        del changed[key]
        with pytest.raises(ValueError):
            load_payload(changed)
    for key, value in (("version", True), ("version", 4), ("version", 6),
                       ("curve_representation", "confirmed_contour_node_chords_v1"),
                       ("curve_complete", 1), ("continuation_representation", "unsupported")):
        changed = deepcopy(payload)
        changed[key] = value
        with pytest.raises(ValueError):
            load_payload(changed)
    outer = {"feature_representation": "paint_boundary_v1", "boundary_version": 5,
             "curve_representation": CURVE_REPRESENTATION, "continuation_representation": None}
    for key in tuple(outer):
        changed = dict(outer)
        del changed[key]
        with pytest.raises(ValueError):
            subject._boundary_schema(changed, 3)
    with pytest.raises(ValueError):
        subject._boundary_schema(outer, 2)


def test_v5_cannot_load_old_certified_or_chord_payload_in_disguise():
    for version in (1, 2, 3, 4):
        old = literal_legacy_payload(boundary(version), version)
        with pytest.raises(ValueError):
            load_payload(old, 5)
    changed = subject._boundary_payload(boundary())
    changed["certified_model_representation"] = "confirmed_creation_chart_domains_v1"
    with pytest.raises(ValueError):
        load_payload(changed)
    outer = {"feature_representation": "paint_boundary_v1", "boundary_version": 5,
             "curve_representation": CURVE_REPRESENTATION, "continuation_representation": None,
             "certified_model_representation": "confirmed_creation_chart_domains_v1"}
    with pytest.raises(ValueError):
        subject._boundary_schema(outer, 3)
    for version in (1, 2, 3, 4):
        legacy_outer = {"feature_representation": "paint_boundary_v1", "boundary_version": version,
                        "curve_representation": CURVE_REPRESENTATION}
        if version >= 3:
            legacy_outer["certified_model_representation"] = "confirmed_creation_chart_domains_v1"
        if version == 4:
            legacy_outer["continuation_representation"] = CONTINUATION_REPRESENTATION
        with pytest.raises(ValueError):
            subject._boundary_schema(legacy_outer, 3)


def test_source_frontier_bool_stale_future_or_missing_is_rejected():
    payload = subject._boundary_payload(boundary())
    for value in (True, FRONTIER - 1, FRONTIER + 1, None):
        changed = deepcopy(payload)
        changed["source_frontier_ns"] = value
        with pytest.raises(ValueError):
            load_payload(changed)


def test_legacy_v1_to_v4_payload_bytes_match_literal_original_contract():
    for version in (1, 2, 3, 4):
        original = boundary(version)
        expected = literal_legacy_payload(original, version)
        actual = subject._boundary_payload(original)
        assert encode(actual) == encode(expected)
        assert "curve_representation" not in actual and "curve_complete" not in actual
        assert not any(name.startswith("curve_") for name in actual["arrays"])
        loaded = load_payload(actual, version)
        assert_arrays_equal(original, loaded)
        for first, second in zip(query(original)[1], query(loaded)[1]):
            assert first.tobytes() == second.tobytes()


def test_all_curve_arrays_and_complete_participate_in_snapshot_geometry_hash():
    reference = boundary()
    baseline = subject.geometry_sha256(snapshot(reference))
    changes = {
        "segments": np.array([[[-1., .25], [1., .25]]]), "normals": np.array([[0., -1.]]),
        "owner": np.array([1]), "frame_votes": np.array([6]), "reliable_votes": np.array([2]),
        "last_stamp_ns": np.array([STAMP - 1]), "owner_status": np.array([2, 1, 1]),
        "blocker_boxes": np.array([[[-1., .03125], [1., .03125]], [[2., 2.], [3., 2.]]]),
        "blocker_owner": np.array([0, 2]), "complete": False}
    assert set(changes) == set(CURVE) | {"complete"}
    for name, value in changes.items():
        changed = replace(reference, curve_geometry=replace(reference.curve_geometry, **{name: value}))
        assert subject.geometry_sha256(snapshot(changed)) != baseline, name
    assert subject.geometry_sha256(snapshot(reference)) == baseline


def test_full_synthetic_bundle_build_load_preserves_geometry_and_fresh_query(tmp_path):
    reference = boundary()
    before = subject.geometry_sha256(snapshot(reference))
    path, frozen = build(tmp_path, reference)
    manifest, route, metadata, points, calibration = subject.load_bundle(path)
    loaded = metadata["boundary_reference"]
    assert_arrays_equal(reference, loaded)
    assert metadata["snapshot_geometry_sha256"] == before
    assert metadata["snapshot_geometry_after_build_sha256"] == before
    restored = MapSnapshot(frozen.trace, points, frozen.votes, FRONTIER, frozen.stats, boundary_reference=loaded)
    assert subject.geometry_sha256(frozen) == before == subject.geometry_sha256(restored)
    for payload in (manifest, route, metadata):
        assert payload["boundary_version"] == 5
        assert payload["curve_representation"] == CURVE_REPRESENTATION
        assert "certified_model_representation" not in payload
    assert metadata["requires_operator_review"] is True
    assert metadata["closed_loop_vehicle_validation"] is False
    assert metadata["gnss_used_for_mapping"] is False and calibration["frame_id"] == "camera"
    first, outputs = query(reference)
    fresh, reloaded_outputs = query(loaded)
    assert first is not fresh
    for expected, actual in zip(outputs, reloaded_outputs):
        assert expected.tobytes() == actual.tobytes()


def test_full_bundle_outer_rehash_cannot_hide_boundary_geometry_change(tmp_path):
    path, _ = build(tmp_path, boundary())
    manifest = json.loads(path.read_bytes())
    target = path.parent / manifest["boundary_path"]
    original = target.read_bytes()
    target.write_bytes(original + b" ")
    with pytest.raises(ValueError, match="checksum"):
        subject.load_bundle(path)
    body = json.loads(original)
    for endpoint in body["arrays"]["curve_segments"]["data"][0]:
        endpoint[1] += .125
    rehash_member(body["arrays"]["curve_segments"])
    rewritten = encode(body)
    target.write_bytes(rewritten)
    manifest["boundary_sha256"] = hashlib.sha256(rewritten).hexdigest()
    path.write_bytes(encode(manifest))
    with pytest.raises(ValueError, match="geometry integrity"):
        subject.load_bundle(path)


def test_full_bundle_rejects_version_schema_and_continuation_marker_disagreement(tmp_path):
    path, _ = build(tmp_path, boundary())
    original_manifest = json.loads(path.read_bytes())
    changed = dict(original_manifest, schema_version=99)
    path.write_bytes(encode(changed))
    with pytest.raises(ValueError):
        subject.load_bundle(path)
    changed = dict(original_manifest, boundary_version=4)
    path.write_bytes(encode(changed))
    with pytest.raises(ValueError):
        subject.load_bundle(path)
    # A null marker in any outer file cannot contradict boundary.json's actual
    # permission set, even when that outer file's checksum has been updated.
    for name in ("manifest", "route", "metadata"):
        changed = dict(original_manifest)
        if name == "manifest":
            changed["continuation_representation"] = None
            path.write_bytes(encode(changed))
            with pytest.raises(ValueError):
                subject.load_bundle(path)
        else:
            target = path.parent / changed[name + "_path"]
            raw = target.read_bytes()
            body = json.loads(raw)
            body["continuation_representation"] = None
            rewritten = encode(body)
            target.write_bytes(rewritten)
            changed[name + "_sha256"] = hashlib.sha256(rewritten).hexdigest()
            path.write_bytes(encode(changed))
            try:
                with pytest.raises(ValueError):
                    subject.load_bundle(path)
            finally:
                target.write_bytes(raw)
    path.write_bytes(encode(original_manifest))


def test_incomplete_or_empty_v5_cannot_create_ready_bundle_directory(tmp_path):
    reference = boundary()
    incomplete = replace(reference, curve_geometry=replace(reference.curve_geometry, complete=False))
    empty_curve = replace(reference.curve_geometry,
        segments=np.empty((0, 2, 2)), normals=np.empty((0, 2)),
        owner=np.empty(0, dtype=np.int64), frame_votes=np.empty(0, dtype=np.int64),
        reliable_votes=np.empty(0, dtype=np.int64), last_stamp_ns=np.empty(0, dtype=np.int64))
    empty = replace(reference, curve_geometry=empty_curve,
                    **{name: None for name in CONTINUATION})
    for label, candidate in (("incomplete", incomplete), ("empty", empty)):
        # Direct serialization may retain the rejected diagnostic model. It
        # must not promote it into a ready map/controller bundle.
        assert_arrays_equal(candidate, load_payload(subject._boundary_payload(candidate)))
        outputroot = tmp_path / label
        with pytest.raises(ValueError):
            subject.build_bundle(outputroot, snapshot(candidate), {"frame_id": "camera"},
                                 route_config={"reference_speed_mps": .8}, session_id="rejected_curve_test")
        assert not outputroot.exists()


def test_standalone_unknown_footprints_roundtrip_without_fake_nodes(tmp_path):
    original = boundary()
    standalone = replace(original, curve_geometry=replace(original.curve_geometry,
        blocker_owner=np.array([-1, -1], dtype=np.int64)))
    loaded = load_payload(subject._boundary_payload(standalone))
    assert_arrays_equal(standalone, loaded)
    for name, invalid in (("curve_blocker_owner", [-2, -1]), ("curve_owner", [-1])):
        broken = deepcopy(subject._boundary_payload(standalone))
        broken["arrays"][name]["data"] = invalid
        rehash_member(broken["arrays"][name])
        with pytest.raises(ValueError):
            load_payload(broken)
    empty = loaded.subset(np.empty(0, dtype=np.int64))
    assert len(empty.record_ids) == 0
    assert empty.curve_geometry.blocker_owner.tolist() == [-1, -1]
    assert empty.curve_geometry.blocker_boxes.tobytes() == standalone.curve_geometry.blocker_boxes.tobytes()
    path, frozen = build(tmp_path, standalone)
    _, _, metadata, _, _ = subject.load_bundle(path)
    restored = metadata["boundary_reference"]
    assert_arrays_equal(standalone, restored)
    assert metadata["snapshot_geometry_sha256"] == subject.geometry_sha256(frozen)
    for before, after in zip(query(standalone)[1], query(restored)[1]):
        assert before.tobytes() == after.tobytes()
