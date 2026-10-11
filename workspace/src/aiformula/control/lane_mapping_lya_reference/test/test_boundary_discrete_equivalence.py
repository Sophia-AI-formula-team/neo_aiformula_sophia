"""Discrete preparation equivalence against frozen, literal R12 expressions.

Oracle copied BEFORE optimization from paint source SHA
4218dd97941cd24babc65ed31c4db25b59314c7c0b06a62a3e16f7fc70fda350.
No runtime Git/history/diagnostic paths are needed. Geometry calculations,
capacity checks, and reduction order remain the old literal implementation.
R16 adds topology-witnessed adjacent-cell identity as an explicitly new domain.
These unchanged fixtures stay inside the legacy domain: every step must report
zero adjacent-cell associations. Only that new diagnostic key is supplied to
the old token comparator; the literal algorithm below remains unchanged.
R17 deliberately adds separately certified finite domains. The old oracle
continues to compare every original token field, statistic and 13 reference
arrays, not the new domain state/counter or five certificate arrays. Those
have independent consensus and certified-domain tests; copying production
domain results into this oracle would not establish equivalence.
"""
from dataclasses import fields, replace
import math
from types import MappingProxyType

import numpy as np
import pytest

from lane_mapping_lya_reference import paint_geometry as paint

BoundaryObservation = paint.BoundaryObservation
_BoundaryRecord = paint._BoundaryRecord
_BoundaryEdge = paint._BoundaryEdge
PreparedBoundaryUpdate = paint.PreparedBoundaryUpdate
_frozen = paint._frozen
TOPOLOGY_REPRESENTATION = paint.TOPOLOGY_REPRESENTATION


def _literal_runs(obs, eligible, cells, cosine):
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
                    and np.array_equal(all_cells[face], all_cells[run[0]])
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
                and np.array_equal(all_cells[parts[-1][0]], all_cells[parts[0][0]])
                and np.all(normals[parts[-1]] @ normals[parts[0]].T >= cosine)):
            parts[0] = parts[-1] + parts[0]
            parts.pop()
        for faces in parts:
            first = faces[0]
            output.append((tuple(map(int, all_cells[first])), int(obs.contour_ids[first]), faces))
    return sorted(output, key=lambda item: (item[0], item[1], item[2][0]))


def _literal_prepare(self, observation_world):
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
        groups = _literal_runs(obs, eligible, cells, cosine)
        prepared_groups, claims = [], {}
        for cell, _, members in groups:
            faces = np.asarray(members, dtype=np.int64)
            point = np.mean(model_points[faces], axis=0)
            normal = np.mean(model_normals[faces], axis=0)
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
        records = []
        for record_id, cell, point, normal, votes, reliable_votes, faces in updates:
            neighbours = np.full((len(faces), 2), -1, dtype=np.int64)
            for side, links in enumerate((obs.previous[faces], obs.next[faces])):
                known = links >= 0
                neighbours[known, side] = face_owner[links[known]]
            records.append(_BoundaryRecord(record_id, cell, point, normal, votes, reliable_votes, stamp,
                                           _frozen(obs.segments[faces], np.float64, (len(faces), 2, 2), "record segments"),
                                           _frozen(neighbours, np.int64, (len(faces), 2), "record neighbours")))
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
        for face in eligible:
            following = int(obs.next[face])
            if following < 0 or face_owner[following] < 0:
                continue
            source_id, target_id = int(face_owner[face]), int(face_owner[following])
            if source_id == target_id:
                continue
            key = (source_id, target_id)
            reliable = bool(face_reliable[face] and face_reliable[following])
            edge_votes[key] = edge_votes.get(key, False) or reliable
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

def observation(points, stamp, *, closed=False, split=None, valid=None,
                reliable=None, angles=None, model=False):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    n = len(points)
    angles = np.zeros(n) if angles is None else np.asarray(angles)
    normals = np.column_stack((np.cos(angles), np.sin(angles)))
    ids = np.arange(n, dtype=np.int64)
    previous, following = ids-1, ids+1
    if n:
        previous[0], following[-1] = (n-1, 0) if closed else (-1, -1)
    contour_ids = np.zeros(n, dtype=np.int64)
    for at in (() if split is None else split):
        following[at-1], previous[at] = -1, -1
        contour_ids[at:] += 1
    valid = np.ones(n, dtype=bool) if valid is None else np.asarray(valid, dtype=bool)
    reliable = valid.copy() if reliable is None else (np.asarray(reliable, dtype=bool) & valid)
    segments = np.stack((points+[0.,-.001], points+[0.,.001]), axis=1)
    kwargs = {}
    if model:
        kwargs = dict(position_estimates=points.copy(), position_valid=valid.copy(),
                      position_sensitivity=np.full(n,.02),
                      position_reliable=reliable.copy(), position_normals=normals.copy())
        points = points + [.001, -.001]
    return paint.BoundaryObservation(points,normals,segments,np.full(n,.02),
        np.ones(n,dtype=bool),valid,valid,valid,reliable,stamp,segments,
        np.zeros((n,2),dtype=np.int64),np.ones((n,2),dtype=np.int64),
        previous,following,contour_ids,np.zeros(n,dtype=np.int64),
        np.full(n,6,dtype=np.int64),np.full(n,.45),**kwargs)


def same_value(left, right):
    if isinstance(left,np.ndarray):
        assert left.dtype==right.dtype and left.shape==right.shape
        assert left.tobytes()==right.tobytes()
        assert left.flags.writeable==right.flags.writeable
    elif hasattr(left,'__dataclass_fields__'):
        for field in fields(left):
            if field.name!='owner':
                same_value(getattr(left,field.name),getattr(right,field.name))
    elif isinstance(left,(tuple,list)):
        assert len(left)==len(right)
        for a,b in zip(left,right):
            same_value(a,b)
    elif isinstance(left,(dict,MappingProxyType)):
        assert dict(left)==dict(right)
    elif isinstance(left,(float,np.floating)):
        assert np.asarray(left,dtype=np.float64).tobytes()==np.asarray(right,dtype=np.float64).tobytes()
    else:
        assert left==right


def same_legacy_snapshot(left, right):
    assert left.source_frontier_ns == right.source_frontier_ns
    for name in ('points_xy', 'normals', 'support_segments', 'segment_owner',
                 'votes', 'reliable_votes', 'record_ids', 'directed_edges',
                 'edge_frame_votes', 'edge_reliable_votes', 'edge_last_stamp_ns',
                 'full_in_degree', 'full_out_degree'):
        same_value(getattr(left, name), getattr(right, name))


def compare_step(fast, oracle, obs, commit=True):
    before_fast=(fast._generation,fast.last_stamp_ns,tuple(fast._records.items()),tuple(fast._edges.items()))
    before_old=(oracle._generation,oracle.last_stamp_ns,tuple(oracle._records.items()),tuple(oracle._edges.items()))
    result=fast.prepare(obs)
    expected=_literal_prepare(oracle,obs)
    assert result.stats['adjacent_cell_associations'] == 0
    expected=replace(expected,stats=MappingProxyType(dict(expected.stats,
                                                       adjacent_cell_associations=0)))
    assert isinstance(result.stats['certified_domain_intervals'], int)
    assert result.stats['certified_domain_intervals'] >= 0
    legacy_stats = dict(result.stats)
    del legacy_stats['certified_domain_intervals']
    legacy_result = replace(result, domain_updates=(), domain_interval_count=0,
                            stats=MappingProxyType(legacy_stats))
    same_value(legacy_result,expected)
    # Structural identities ensure preparing either token cannot mutate state.
    assert fast._generation==before_fast[0] and fast.last_stamp_ns==before_fast[1]
    assert oracle._generation==before_old[0] and oracle.last_stamp_ns==before_old[1]
    assert all(fast._records[k] is r for k,r in before_fast[2])
    assert all(oracle._records[k] is r for k,r in before_old[2])
    for rec in result.updates:
        for array in (rec.segments,rec.neighbours):
            assert not array.flags.writeable
            with pytest.raises(ValueError):
                array.setflags(write=True)
        # Each record remains independently frozen, not a shared frame buffer.
        for other in result.updates:
            if other is not rec:
                assert not np.shares_memory(rec.neighbours,other.neighbours)
                assert not np.shares_memory(rec.segments,other.segments)
    if commit:
        assert fast.commit(result) and oracle.commit(expected)
        same_legacy_snapshot(fast.snapshot(),oracle.snapshot())
        assert list(fast._records)==list(oracle._records)
        assert list(fast._edges)==list(oracle._edges)
        for key in fast._records:
            same_value(fast._records[key],oracle._records[key])
        for key in fast._edges:
            same_value(fast._edges[key],oracle._edges[key])
    return result,expected


@pytest.mark.parametrize('seed',range(12))
@pytest.mark.parametrize('ttl',[0.,.25])
def test_random_prepared_geometry_votes_topology_bytes(seed,ttl):
    rng=np.random.RandomState(seed)
    count=int(rng.randint(8,45))
    cells=rng.randint(-3,4,(count,2))
    base=cells*.1+rng.uniform(-.035,.035,(count,2))
    angles=rng.choice([0.,.15,.51,math.pi],count)
    configs={'candidate_ttl_s':ttl}
    fast,oracle=paint.BoundaryConsensus(configs),paint.BoundaryConsensus(configs)
    stamp=0
    for frame in range(8):
        stamp+=500000000 if frame==6 else 100000000
        jitter=rng.uniform(-.003,.003,(count,2))
        admitted=rng.rand(count)>.12
        reliable=rng.rand(count)>.25
        obs=observation(base+jitter,stamp,closed=seed%2==0,
            valid=admitted,reliable=reliable,angles=angles,model=bool(seed%3))
        compare_step(fast,oracle,obs)


@pytest.mark.parametrize('closed',[False,True])
@pytest.mark.parametrize('gap',[False,True])
@pytest.mark.parametrize('model',[False,True])
def test_aba_reentry_repeated_faces_closed_seam_and_gap(closed,gap,model):
    points=np.array([[.01,0.],[.02,0.],[.11,0.],[.12,0.],[.02,0.],[.01,0.]])
    valid=np.ones(6,dtype=bool)
    if gap:
        valid[2]=False
    fast,oracle=paint.BoundaryConsensus(),paint.BoundaryConsensus()
    for frame in range(6):
        obs=observation(points,100000000*(frame+1),closed=closed,valid=valid,model=model)
        cells=np.rint(obs.model_points[obs.model_valid]/.1).astype(np.int64)
        eligible=np.flatnonzero(obs.model_valid)
        assert paint._contiguous_cell_runs(obs,eligible,cells,math.cos(math.pi/6))==_literal_runs(
            obs,eligible,cells,math.cos(math.pi/6))
        compare_step(fast,oracle,obs)


@pytest.mark.parametrize('angles',[
    [0.,0.,0.,0.],
    [0.,math.pi/6,0.,math.pi/6],
    [0.,np.nextafter(math.pi/6,np.inf),0.,0.],
    [0.,math.pi,math.pi,0.],
])
def test_signed_pairwise_compatibility_and_same_cell_ties_unchanged(angles):
    fast,oracle=paint.BoundaryConsensus(),paint.BoundaryConsensus()
    points=[[.01,0.],[.02,0.],[.01,0.],[.02,0.]]
    for frame in range(6):
        compare_step(fast,oracle,observation(points,frame+1,split=[2],angles=angles))


def test_empty_no_links_and_zero_owner_are_preserved():
    fast,oracle=paint.BoundaryConsensus(),paint.BoundaryConsensus()
    compare_step(fast,oracle,observation([],1))
    token,_=compare_step(fast,oracle,observation([[.01,0.],[.11,0.]],2))
    assert token.edge_updates[0].source_id==0 and token.edge_updates[0].target_id==1
    compare_step(fast,oracle,observation([],3))


@pytest.mark.parametrize('reliable',[
    [False,False,False,False,False,False],
    [True,True,False,False,False,False],
    [False,True,True,False,False,False],
])
def test_duplicate_integer_pairs_have_one_frame_vote_with_any_reliable_witness(monkeypatch,reliable):
    # Isolate the integer aggregation stage with repeated pair witnesses.
    # Deliberately supplied groups are NOT claimed to be actual contour runs;
    # contour-run validity is independently covered by the preceding tests.
    groups=[((0,0),0,[0,2,4]),((1,0),0,[1,3,5])]
    literal=lambda *args:groups
    monkeypatch.setattr(paint,'_contiguous_cell_runs',literal)
    monkeypatch.setitem(_literal_prepare.__globals__,'_literal_runs',literal)
    obs=observation([[.01,0.],[.11,0.]]*3,1,closed=True,reliable=reliable)
    fast,oracle=paint.BoundaryConsensus(),paint.BoundaryConsensus()
    prepared,_=compare_step(fast,oracle,obs)
    edges={(edge.source_id,edge.target_id):edge for edge in prepared.edge_updates}
    assert set(edges)=={(0,1),(1,0)}
    assert all(edge.votes==1 for edge in edges.values())
    expected_forward=any(reliable[i] and reliable[i+1] for i in (0,2,4))
    expected_backward=any(reliable[i] and reliable[(i+1)%6] for i in (1,3,5))
    assert edges[(0,1)].reliable_votes==int(expected_forward)
    assert edges[(1,0)].reliable_votes==int(expected_backward)


def test_expired_unconfirmed_and_confirmed_retention_same_token():
    cfg={'candidate_ttl_s':.15}
    fast,oracle=paint.BoundaryConsensus(cfg),paint.BoundaryConsensus(cfg)
    for k in range(5):
        compare_step(fast,oracle,observation([[.01,0.],[.11,0.]],100000000*(k+1)))
    assert fast.confirmed_count==2
    compare_step(fast,oracle,observation([[1.01,0.],[1.11,0.]],600000000))
    token,_=compare_step(fast,oracle,observation([],1000000000))
    assert len(token.expired_ids)==2 and len(token.expired_edge_keys)==1
    assert fast.confirmed_count==2


def test_prepared_rejected_stale_and_wrong_owner_do_not_advance():
    fast,oracle=paint.BoundaryConsensus(),paint.BoundaryConsensus()
    rejected,_=compare_step(fast,oracle,observation([[.01,0.],[.11,0.]],1),False)
    assert fast.last_stamp_ns is None and fast.confirmed_count==0
    accepted,_=compare_step(fast,oracle,observation([[.01,0.],[.11,0.]],2))
    assert not fast.can_commit(rejected) and not fast.commit(rejected)
    assert not oracle.can_commit(accepted) and not oracle.commit(accepted)
    same_legacy_snapshot(fast.snapshot(),oracle.snapshot())


@pytest.mark.parametrize('settings',[
    {'max_candidate_cells':1},
    {'max_confirmed_cells':1},
    {'max_support_segments':1},
])
def test_capacity_rejection_is_same_without_mutation(settings):
    fast,oracle=paint.BoundaryConsensus(settings),paint.BoundaryConsensus(settings)
    for frame in range(1,7):
        obs=observation([[.01,0.],[.11,0.]],frame)
        errors=[]
        for obj,method in ((fast,fast.prepare),(oracle,lambda value:_literal_prepare(oracle,value))):
            generation=obj._generation
            try:
                token=method(obs)
            except ValueError as error:
                errors.append(str(error))
                assert obj._generation==generation
            else:
                errors.append(None)
                assert obj.commit(token)
        assert errors[0]==errors[1]
        same_legacy_snapshot(fast.snapshot(),oracle.snapshot())
        if errors[0] is not None:
            break
    else:
        pytest.fail('fixture never reached original capacity check')

