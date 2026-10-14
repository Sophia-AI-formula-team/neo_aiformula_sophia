"""Adapter for complete original BoundaryConsensus v5 pair inputs.

Caller must forward the original _contiguous_cell_runs once, without altering
its result, and pass exactly the prepared records from that same prepare call.
Only the explicit v5 development path uses it; TeachMapEngine/control do not.
It is not a new owner association or permission.
It gathers the SAME owner model-next and cross-owner incident pairs as the
original prepare code. Own position fragments stay in the old domain pipeline
and are not treated as nominal root samples here. No source/model execution
is performed by importing this file.
"""
import math
import numpy as np
from . import curve_evidence as evidence


def prepare_packets(obs, original_groups, prepared, config):
    """Authenticate final owners from the actual original prepare output.

    Group-member order and each prepared record's exact original raw face
    bytes must agree. This does not recover IDs by nearest/spatial matching.
    signed_side=+1 denotes the original white-to-black outward convention;
    it is NOT a guessed left/right lane ID. Signed owner normals and source
    contours remain separate. The caller owns the old global capacity gate.
    """
    groups = tuple(original_groups)
    records = tuple(prepared.updates)
    if (len(groups) != len(records) or prepared.source_stamp_ns != obs.source_stamp_ns
            or len({record.record_id for record in records}) != len(records)):
        raise ValueError('actual final-owner source inventory differs')
    count = len(obs.points)
    owners = np.full(count, -1, dtype=np.int64)
    members_by_owner = []
    for group, record in zip(groups, records):
        raw_members = group[2]
        # Original _contiguous_cell_runs returns list[int], not an ndarray.
        # Reject coercion/duplicates before any integer gather; preserve order.
        if (type(raw_members) is not list or not raw_members
                or any(type(value) is not int or not 0 <= value < count for value in raw_members)
                or len(set(raw_members)) != len(raw_members)):
            raise ValueError('expected original unique list[int] group members')
        members = np.asarray(raw_members, dtype=np.int64)
        if (members.ndim != 1 or not len(members) or np.any(members < 0)
                or np.any(members >= count) or np.any(owners[members] >= 0)
                or record.last_stamp_ns != obs.source_stamp_ns
                or record.segments.shape != obs.segments[members].shape
                or record.segments.dtype != obs.segments.dtype
                or record.segments.tobytes(order='C') != obs.segments[members].tobytes(order='C')):
            raise ValueError('original group/record face bytes or order differs')
        owners[members] = record.record_id
        members_by_owner.append(members)
    for record, members in zip(records, members_by_owner):
        expected_links = np.full((len(members), 2), -1, dtype=np.int64)
        for side, links in enumerate((obs.previous, obs.next)):
            neighbors = links[members]
            known = neighbors >= 0
            expected_links[known, side] = owners[neighbors[known]]
        if (record.neighbours.dtype != expected_links.dtype
                or record.neighbours.shape != expected_links.shape
                or record.neighbours.tobytes(order='C') != expected_links.tobytes(order='C')):
            raise ValueError('prepared final-owner neighbor bytes differ')
    points = obs.model_points
    normals = obs.model_normals
    valid = obs.model_valid & (obs.model_sensitivity <= config['max_sensitivity'])
    reliable = (obs.model_reliable & obs.model_valid
                & (obs.model_sensitivity <= config['reliable_sensitivity']))
    if np.any((owners >= 0) != valid):
        raise ValueError('actual eligible owner membership differs')
    # Use the original math.cos gate, not a numerical alternative.
    cosine = math.cos(config['normal_max_angle_rad'])
    first = np.flatnonzero(valid & (obs.next >= 0))
    second = obs.next[first]
    selected = (owners[second] >= 0) & (owners[first] != owners[second])
    first, second = first[selected], second[selected]
    incident = {}
    for a, b in zip(first, second):
        if float(np.dot(normals[a], normals[b])) < cosine:
            continue
        pair = (int(a), int(b))
        for owner_id in (int(owners[a]), int(owners[b])):
            incident.setdefault(owner_id, []).append(pair)
    packets = []
    if obs.position_footprints is None or obs.position_segment_valid is None:
        raise ValueError('complete actual final observation quads/flags are required')
    quad_valid = (obs.position_segment_valid
                  & (obs.position_segment_sensitivity <= config['max_sensitivity']))
    # One complete immutable table per actual source frame; invalid source faces
    # and independently valid own-footprint evidence are retained too.
    source = evidence.SourceFrame(
        stamp_ns=obs.source_stamp_ns,
        face_ids=np.arange(count, dtype=np.int64),
        previous_next=np.column_stack((obs.previous, obs.next)),
        contour_ids=obs.contour_ids, endpoint_owners=owners,
        model_roots=points, normals=normals, quads=obs.position_footprints,
        quad_valid=quad_valid, reliable=reliable, valid=valid)
    for record, members in zip(records, members_by_owner):
        following = obs.next[members]
        selected = following >= 0
        selected[selected] &= owners[following[selected]] == record.record_id
        pairs = np.column_stack((members[selected], following[selected]))
        roles = np.full(len(pairs), evidence.SAME_OWNER_MODEL_NEXT, dtype=np.uint8)
        cross = incident.get(record.record_id, ())
        if cross:
            pairs = np.concatenate((pairs, np.asarray(cross, dtype=np.int64)))
            roles = np.r_[roles, np.full(len(cross), evidence.CROSS_OWNER_INCIDENT, dtype=np.uint8)]
        before, after = int(obs.previous[members[0]]), int(obs.next[members[-1]])
        signature = (int(owners[before]) if before >= 0 else -1,
                     int(owners[after]) if after >= 0 else -1)
        packets.append(evidence.PairPacket(source=source,
            owner_id=record.record_id, signed_side=1, signature=signature,
            roles=roles, face_indices=pairs))
    return tuple(packets), owners
