"""Boundary identity persists separately from filled route/display geometry."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))
from lane_mapping_fixed_gnss import bundle
from lane_mapping_fixed_gnss.mapping import MapSnapshot
from lane_mapping_lya_reference.paint_geometry import BoundaryReference


def reference(frontier=100):
    points = np.array([[0., 1.], [1., 1.], [2., 1.]])
    # A genuine gap between segments must survive loading; no min/max envelope.
    segments = np.array([[[-.2, 1.], [-.1, 1.]], [[.1, 1.], [.2, 1.]],
                         [[.8, 1.], [1.2, 1.]], [[1.8, 1.], [2.2, 1.]]])
    return BoundaryReference(points, np.tile([0., 1.], (3, 1)), segments,
                             np.array([0, 0, 1, 2]), np.array([5, 6, 7]),
                             np.array([1, 2, 3]), np.array([4, 8, 19]), frontier)


def graph_reference(empty=False):
    return replace(reference(), directed_edges=np.empty((0,2),dtype=np.int64) if empty else np.array([[0,1],[1,2]]),
                   edge_frame_votes=np.array([],dtype=np.int64) if empty else np.array([3,4]),
                   edge_reliable_votes=np.array([],dtype=np.int64) if empty else np.array([0,1]),
                   edge_last_stamp_ns=np.array([],dtype=np.int64) if empty else np.array([90,100]))


def branch_subset_reference():
    points=np.array([[0.,1.],[1.,1.],[2.,1.],[1.,2.]])
    segments=np.stack((points-[.1,0.],points+[.1,0.]),axis=1)
    full=BoundaryReference(points,np.tile([0.,1.],(4,1)),segments,np.arange(4),
        np.full(4,5),np.full(4,2),np.arange(4),100,
        directed_edges=np.array([[0,1],[1,2],[1,3]]),edge_frame_votes=np.full(3,5),
        edge_reliable_votes=np.ones(3,dtype=np.int64),edge_last_stamp_ns=np.full(3,90))
    return full.subset([0,1,2])


def snapshot(boundary=True):
    trace = np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]])
    filled = np.array([[0., 1.1], [1., 1.1], [2., 1.1]])
    return MapSnapshot(trace, filled, np.full(3, 5.), 100, {"accepted": 10},
                       boundary_reference=reference() if boundary else None)


@pytest.fixture
def route_stubs(monkeypatch):
    # This suite isolates persistence. Actual route feasibility remains covered
    # by test_bundle/test_route; a straight stub is not claimed to be drivable.
    monkeypatch.setattr(bundle, "build_route", lambda *args: dict(
        schema_version=1, closed=True, diagnostics={"valid": True},
        speed_policy="profile", reference_speed_mps=0.))
    monkeypatch.setattr(bundle, "ClosedRouteController", lambda *args, **kwargs: object())


def create(tmp_path, route_stubs, value=None):
    return bundle.build_bundle(tmp_path, value or snapshot(), {"frame": "camera"}, session_id="boundary")


def resign(path, name, mutate):
    manifest = json.loads(path.read_text())
    target = path.parent / manifest[name + "_path"]
    value = json.loads(target.read_text())
    mutate(value)
    data = json.dumps(value).encode()
    target.write_bytes(data)
    manifest[name + "_sha256"] = hashlib.sha256(data).hexdigest()
    path.write_text(json.dumps(manifest))


def test_roundtrip_all_fields_immutable_and_filled_separate(tmp_path, route_stubs):
    original = snapshot()
    path = create(tmp_path, route_stubs, original)
    manifest, route, metadata, filled, calibration = bundle.load_bundle(path)
    assert manifest["schema_version"] == 3
    assert manifest["feature_representation"] == "paint_boundary_v1"
    loaded = metadata["boundary_reference"]
    assert isinstance(loaded, BoundaryReference)
    assert loaded.source_frontier_ns == original.source_frontier_ns
    for name in bundle.BOUNDARY_ARRAYS:
        assert getattr(loaded, name).tobytes() == getattr(original.boundary_reference, name).tobytes()
        with pytest.raises(ValueError):
            getattr(loaded, name).setflags(write=True)
    np.testing.assert_array_equal(filled, original.consensus_xy)
    assert not np.array_equal(filled, loaded.points_xy)
    assert len(loaded.support_segments) == 4
    assert loaded.support_segments[0, 1, 0] < loaded.support_segments[1, 0, 0]
    assert "boundary_reference" not in json.loads((path.parent / "metadata.json").read_text())
    assert len((manifest, route, metadata, filled, calibration)) == 5


def test_legacy_hash_and_schema_remain_exact(tmp_path, route_stubs):
    original = snapshot(False)
    digest = hashlib.sha256()
    for value in (original.trace, original.consensus_xy, original.votes):
        array = np.asarray(value, dtype="<f8")
        digest.update(str(array.shape).encode("ascii")); digest.update(array.tobytes())
    assert bundle.geometry_sha256(original) == digest.hexdigest()
    path = create(tmp_path, route_stubs, original)
    manifest, _, metadata, _, _ = bundle.load_bundle(path)
    assert manifest["schema_version"] == 2
    assert "boundary_path" not in manifest and not (path.parent / "boundary.json").exists()
    assert metadata["boundary_reference"] is None


@pytest.mark.parametrize("boundary", [False, True])
def test_fixed_speed_reference_does_not_replace_boundary_reference(tmp_path, route_stubs, monkeypatch, boundary):
    monkeypatch.setattr(bundle, "build_route", lambda *args: dict(
        schema_version=1, closed=True, diagnostics={"valid": True},
        speed_policy="fixed_reference", reference_speed_mps=2.,
        config={"reference_speed_mps": 2., "max_curvature_1pm": .5,
                "max_lateral_accel_mps2": 2.}))
    original = snapshot(boundary)
    path = create(tmp_path, route_stubs, original)
    _, _, metadata, _, _ = bundle.load_bundle(path)
    assert metadata["reference_speed_mps"] == 2.
    if boundary:
        assert isinstance(metadata["boundary_reference"], BoundaryReference)
        assert metadata["boundary_reference"].points_xy.tobytes() == original.boundary_reference.points_xy.tobytes()
    else:
        assert metadata["boundary_reference"] is None


@pytest.mark.parametrize("member", list(bundle.BOUNDARY_ARRAYS))
def test_each_member_has_independent_checksum(tmp_path, route_stubs, member):
    path = create(tmp_path, route_stubs)
    def change(value):
        data = value["arrays"][member]["data"]
        cell = data
        while isinstance(cell[0], list):
            cell = cell[0]
        cell[0] += 1
    resign(path, "boundary", change)
    with pytest.raises(ValueError, match="checksum"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("member", list(bundle.BOUNDARY_ARRAYS))
def test_geometry_digest_covers_each_reference_member(member):
    original = snapshot()
    value = getattr(original.boundary_reference, member).copy()
    if member == "normals":
        value *= -1.
    elif member == "segment_owner":
        value[0] = 1
    elif member in ("points_xy", "support_segments"):
        value.flat[0] += .01
    else:
        value[0] += 1
    changed = replace(original.boundary_reference, **{member: value})
    assert bundle.geometry_sha256(replace(original, boundary_reference=changed)) != bundle.geometry_sha256(original)


@pytest.mark.parametrize("payload", ["route", "metadata"])
def test_legacy_cannot_claim_boundary_in_any_metadata(tmp_path, route_stubs, payload):
    path = create(tmp_path, route_stubs, snapshot(False))
    resign(path, payload, lambda value: value.update(feature_representation="paint_boundary_v1"))
    with pytest.raises(ValueError, match="legacy"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("member,value", [("version", 2), ("version", True),
    ("source_frontier_ns", 101), ("representation", "filled_consensus")])
def test_wrong_version_representation_or_frontier_rejected(tmp_path, route_stubs, member, value):
    path = create(tmp_path, route_stubs)
    resign(path, "boundary", lambda obj: obj.update({member: value}))
    with pytest.raises(ValueError, match="representation/frontier"):
        bundle.load_bundle(path)


@pytest.mark.parametrize("value", [1.5, True, 2**63, -1, 10**1000])
def test_integer_members_never_truncate_or_wrap(tmp_path, route_stubs, value):
    path = create(tmp_path, route_stubs)
    resign(path, "boundary", lambda obj: obj["arrays"]["record_ids"]["data"].__setitem__(0, value))
    with pytest.raises(ValueError):
        bundle.load_bundle(path)


def test_unknown_field_and_gnss_rejected_even_with_outer_checksum(tmp_path, route_stubs):
    path = create(tmp_path, route_stubs)
    resign(path, "boundary", lambda obj: obj.update(latitude=35.))
    with pytest.raises(ValueError, match="GNSS"):
        bundle.load_bundle(path)


def test_duplicate_payload_path_is_rejected(tmp_path, route_stubs):
    path = create(tmp_path, route_stubs)
    manifest = json.loads(path.read_text())
    manifest["boundary_path"] = manifest["consensus_path"]
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="distinct"):
        bundle.load_bundle(path)


@pytest.mark.parametrize('empty', [False, True])
def test_graph_v2_roundtrip_all_fields_and_empty_is_not_legacy(tmp_path, route_stubs, empty):
    original = replace(snapshot(), boundary_reference=graph_reference(empty))
    path = create(tmp_path,route_stubs,original)
    manifest,route,metadata,_,_ = bundle.load_bundle(path)
    for payload in (manifest,route,metadata):
        assert payload['boundary_version'] == 2
    saved = json.loads((path.parent/'boundary.json').read_text())
    assert saved['version'] == 2
    assert set(saved['arrays']) == set(bundle.BOUNDARY_ARRAYS) | set(bundle.BOUNDARY_GRAPH_ARRAYS)
    loaded=metadata['boundary_reference']
    for name in bundle.BOUNDARY_GRAPH_ARRAYS:
        assert getattr(loaded,name).tobytes() == getattr(original.boundary_reference,name).tobytes()
        with pytest.raises(ValueError):
            getattr(loaded,name).setflags(write=True)
    assert loaded.directed_edges is not None


def test_v1_finite_bundle_keeps_original_layout_and_no_synthetic_graph(tmp_path,route_stubs):
    original=snapshot()
    old_payload=dict(representation='paint_boundary_v1',version=1,source_frontier_ns=100,arrays={})
    for name,(dtype,_) in bundle.BOUNDARY_ARRAYS.items():
        value=np.asarray(getattr(original.boundary_reference,name),dtype=dtype)
        old_payload['arrays'][name]=dict(dtype=dtype,shape=list(value.shape),
            sha256=hashlib.sha256(value.tobytes()).hexdigest(),data=value.tolist())
    digest=hashlib.sha256()
    for value in (original.trace,original.consensus_xy,original.votes):
        array=np.asarray(value,dtype='<f8')
        digest.update(str(array.shape).encode('ascii')); digest.update(array.tobytes())
    digest.update((json.dumps(old_payload,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)+'\n').encode())
    assert bundle.geometry_sha256(original)==digest.hexdigest()
    path=create(tmp_path,route_stubs,original)
    manifest,_,metadata,_,_=bundle.load_bundle(path)
    assert manifest['boundary_version']==1
    assert all(getattr(metadata['boundary_reference'],name) is None for name in bundle.BOUNDARY_GRAPH_ARRAYS)


@pytest.mark.parametrize('name',list(bundle.BOUNDARY_GRAPH_ARRAYS))
def test_graph_members_have_checksum_and_full_geometry_identity(tmp_path,route_stubs,name):
    original=replace(snapshot(),boundary_reference=graph_reference())
    path=create(tmp_path,route_stubs,original)
    def change(value):
        data=value['arrays'][name]['data']
        if name=='directed_edges': data[0]=[0,2]
        else: data[0]+=1
    resign(path,'boundary',change)
    with pytest.raises(ValueError,match='checksum'):
        bundle.load_bundle(path)
    value=getattr(original.boundary_reference,name).copy()
    if name=='directed_edges': value[0]=[0,2]
    else: value[0]+=1
    changes={name:value}
    if name=='directed_edges':
        changes.update(full_in_degree=None,full_out_degree=None)
    updated=replace(original.boundary_reference,**changes)
    assert bundle.geometry_sha256(replace(original,boundary_reference=updated)) != bundle.geometry_sha256(original)


@pytest.mark.parametrize('name',list(bundle.BOUNDARY_GRAPH_ARRAYS))
def test_graph_member_missing_is_not_v1_fallback(tmp_path,route_stubs,name):
    path=create(tmp_path,route_stubs,replace(snapshot(),boundary_reference=graph_reference()))
    resign(path,'boundary',lambda obj:obj['arrays'].pop(name))
    with pytest.raises(ValueError,match='member set'):
        bundle.load_bundle(path)


@pytest.mark.parametrize('payload',['route','metadata'])
def test_graph_version_cannot_disagree_across_payloads(tmp_path,route_stubs,payload):
    path=create(tmp_path,route_stubs,replace(snapshot(),boundary_reference=graph_reference()))
    resign(path,payload,lambda obj:obj.update(boundary_version=1))
    with pytest.raises(ValueError,match='version differs'):
        bundle.load_bundle(path)


@pytest.mark.parametrize('name,data',[
    ('directed_edges',[[0,0],[1,2]]),('directed_edges',[[0,1],[0,1]]),
    ('directed_edges',[[0,3],[1,2]]),('edge_frame_votes',[0,6]),
    ('edge_reliable_votes',[6,4]),('edge_last_stamp_ns',[0,100]),
    ('edge_last_stamp_ns',[101,100])])
def test_resigned_graph_still_checks_topology_votes_and_causal_frontier(name,data):
    payload=bundle._boundary_payload(graph_reference())
    value=np.asarray(data,dtype='<i8')
    payload['arrays'][name].update(data=data,sha256=hashlib.sha256(value.tobytes()).hexdigest())
    with pytest.raises(ValueError):
        bundle._load_boundary(bundle._json_bytes(payload),100,2)


def test_graph_capacity_rejects_layout_before_allocating_declared_rows():
    payload=bundle._boundary_payload(graph_reference())
    payload['arrays']['directed_edges']['shape']=[200001,2]
    with pytest.raises(ValueError,match='layout'):
        bundle._load_boundary(bundle._json_bytes(payload),100,2)


def test_subset_persistence_preserves_hidden_branch_full_degree(tmp_path,route_stubs):
    reference=branch_subset_reference()
    assert np.count_nonzero(reference.directed_edges[:,0]==1)==1
    assert reference.full_in_degree[1]==1 and reference.full_out_degree[1]==2
    original=replace(snapshot(),boundary_reference=reference)
    path=create(tmp_path,route_stubs,original)
    loaded=bundle.load_bundle(path)[2]['boundary_reference']
    assert len(json.loads((path.parent/'boundary.json').read_text())['arrays'])==13
    np.testing.assert_array_equal(loaded.full_in_degree,reference.full_in_degree)
    np.testing.assert_array_equal(loaded.full_out_degree,reference.full_out_degree)
    assert loaded.full_out_degree[1]>np.count_nonzero(loaded.directed_edges[:,0]==1)


def test_resigned_degree_cannot_relabel_known_cropped_branch_as_joint(tmp_path,route_stubs):
    path=create(tmp_path,route_stubs,replace(snapshot(),boundary_reference=branch_subset_reference()))
    def erase_branch(value):
        record=value['arrays']['full_out_degree']
        record['data'][1]=1  # Still >= local degree: only complete identity catches the lie.
        record['sha256']=hashlib.sha256(np.asarray(record['data'],dtype='<i8').tobytes()).hexdigest()
    resign(path,'boundary',erase_branch)
    with pytest.raises(ValueError,match='frozen geometry integrity'):
        bundle.load_bundle(path)


@pytest.mark.parametrize('name,data',[('full_in_degree',[0,0,1]),('full_out_degree',[0,1,0]),
                                    ('full_in_degree',[200001,1,1]),('full_out_degree',[1,200001,0])])
def test_degree_below_local_or_above_global_capacity_rejected_even_if_rehashed(name,data):
    payload=bundle._boundary_payload(graph_reference())
    payload['arrays'][name].update(data=data,sha256=hashlib.sha256(np.asarray(data,dtype='<i8').tobytes()).hexdigest())
    with pytest.raises(ValueError):
        bundle._load_boundary(bundle._json_bytes(payload),100,2)


@pytest.mark.parametrize('name',['full_in_degree','full_out_degree'])
def test_graph_degree_none_is_not_allowed_in_persisted_v2(name):
    payload=bundle._boundary_payload(graph_reference(True))
    payload['arrays'][name]['data']=None
    with pytest.raises(ValueError,match='scalar types'):
        bundle._load_boundary(bundle._json_bytes(payload),100,2)


def test_degree_array_node_capacity_is_not_edge_capacity():
    payload=bundle._boundary_payload(graph_reference())
    payload['arrays']['full_in_degree']['shape']=[100001]
    with pytest.raises(ValueError,match='layout'):
        bundle._load_boundary(bundle._json_bytes(payload),100,2)
