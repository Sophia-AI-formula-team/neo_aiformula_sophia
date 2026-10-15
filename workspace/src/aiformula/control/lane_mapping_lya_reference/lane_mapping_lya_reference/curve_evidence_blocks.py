"""Lossless retained-face storage for the explicit v5 development mapper.

This module does not enable a runtime path, fit geometry or change a gate.
Complete original SourceFrame/PairPacket validation precedes projection. A
lease keeps all required own/incident rows and its source's complete unowned
capsule; unused other-owner rows need not pin a complete original array.
"""
from dataclasses import dataclass, field
from numbers import Integral

import numpy as np

from . import curve_evidence as original


REPRESENTATION = "complete_original_face_blocks_key_leases_v1"
_SEAL = object()
_LAYOUT = []
_ROW_BYTES = 0
for _name, _dtype, _shape in original._FIELDS:
    _dtype = np.dtype(_dtype)
    _count = int(np.prod(_shape, dtype=np.int64)) if _shape else 1
    _LAYOUT.append((_name, _dtype, _shape, _ROW_BYTES, _count))
    _ROW_BYTES += _dtype.itemsize * _count
_LAYOUT = tuple(_LAYOUT)


def _layout(name):
    for item in _LAYOUT:
        if name == item[0]:
            return item
    raise ValueError("unknown complete source field")


def _prepared(value):
    if value._seal is not _SEAL:
        raise ValueError("use prepared retained-evidence factories")


def _decode(blocks, name):
    _, dtype, shape, _, _ = _layout(name)
    # Row views remain ndarrays, including scalar booleans with noncanonical
    # true backing bytes. Extracting a np.bool_ scalar would normalize them.
    payload = b"".join(block.get(name).tobytes(order="C") for block in blocks)
    return np.frombuffer(payload, dtype=dtype).reshape((len(blocks),) + shape)


@dataclass(frozen=True, eq=False)
class FrameIdentity:
    """Source-event identity; no original table, buffer or block registry."""
    stamp_ns: int
    original_face_count: int
    source_kind: str
    _seal: object = field(default=None, repr=False)
    representation: str = field(default=REPRESENTATION, init=False)

    def __post_init__(self):
        _prepared(self)

    @property
    def legacy_support_records(self):
        return 1 + self.original_face_count

    @property
    def legacy_scalar_entries(self):
        return 3 + sum(self.original_face_count * count + 2 + len(shape)
                       for _, _, shape, _, count in _LAYOUT)


@dataclass(frozen=True, eq=False)
class FaceBlock:
    frame: FrameIdentity
    face_id: int
    payload: bytes
    _seal: object = field(default=None, repr=False)

    def __post_init__(self):
        _prepared(self)

    def get(self, name):
        """Exact field view backed only by this complete owned row."""
        _prepared(self)
        _, dtype, shape, offset, count = _layout(name)
        return np.frombuffer(self.payload, dtype=dtype, count=count,
                             offset=offset).reshape(shape)


@dataclass(frozen=True, eq=False)
class UnownedCapsule:
    """Every original valid unowned quad, in original face-ID order.

    The orphan reliable-quad classification is caller-owned. It must come
    from the independent position-footprint flags, not nominal reliable roots.
    """
    frame: FrameIdentity
    blocks: tuple
    _seal: object = field(default=None, repr=False)

    def __post_init__(self):
        _prepared(self)

    @property
    def stamp_ns(self):
        return self.frame.stamp_ns

    def gather_original(self, name):
        _prepared(self)
        return _decode(self.blocks, name)


@dataclass(frozen=True, eq=False)
class KeyFrameLease:
    frame: FrameIdentity
    owner_id: int
    signed_side: int
    signature: tuple
    own_blocks: tuple
    required_blocks: tuple
    unowned: UnownedCapsule
    _seal: object = field(default=None, repr=False)

    def __post_init__(self):
        _prepared(self)


@dataclass(frozen=True, eq=False)
class BlockPacket:
    lease: KeyFrameLease
    roles: np.ndarray
    face_indices: np.ndarray
    _has_reliable: bool
    _seal: object = field(default=None, repr=False)

    def __post_init__(self):
        _prepared(self)

    @property
    def owner_id(self):
        return self.lease.owner_id

    @property
    def signed_side(self):
        return self.lease.signed_side

    @property
    def signature(self):
        return self.lease.signature

    @property
    def stamp_ns(self):
        return self.lease.frame.stamp_ns

    @property
    def source_identity(self):
        return self.lease.frame

    @property
    def normals(self):
        # Required by unchanged curve_consensus._overlay. This is a transient
        # complete gather, not a persistent cache or a zero-copy claim.
        return self.gather_original("normals")

    def gather_original(self, name):
        """Readonly original pair-shaped bytes, including repeated endpoints."""
        _prepared(self)
        _, _, shape, _, _ = _layout(name)
        required = {block.face_id: block for block in self.lease.required_blocks}
        blocks = tuple(required[int(face)] for face in self.face_indices.reshape(-1))
        return _decode(blocks, name).reshape(self.face_indices.shape + shape)

    def occurrence_original(self, name, ordinal):
        """Both endpoints for one exact original occurrence, without reordering."""
        _prepared(self)
        if (isinstance(ordinal, (bool, np.bool_)) or not isinstance(ordinal, Integral)
                or not 0 <= int(ordinal) < len(self.roles)):
            raise ValueError("original occurrence ordinal outside packet")
        required = {block.face_id: block for block in self.lease.required_blocks}
        return _decode(tuple(required[int(face)]
                             for face in self.face_indices[int(ordinal)]), name)

    def occurrence_normals(self, ordinal):
        return self.occurrence_original("normals", ordinal)

    def own_original(self, name, *, valid_only=False, quad_valid_only=False):
        """Ordered own rows, with independent original root/quad filtering."""
        _prepared(self)
        if (type(valid_only) is not bool or type(quad_valid_only) is not bool
                or (valid_only and quad_valid_only)):
            raise ValueError("select one original own-row validity rule")
        flag = "valid" if valid_only else "quad_valid" if quad_valid_only else None
        blocks = self.lease.own_blocks
        if flag is not None:
            blocks = tuple(block for block in blocks if bool(block.get(flag)))
        return _decode(blocks, name)


def is_block_packet(value):
    return type(value) is BlockPacket and value._seal is _SEAL


def is_unowned_capsule(value):
    return type(value) is UnownedCapsule and value._seal is _SEAL


def _frozen(value):
    return np.frombuffer(bytes(bytearray(value.tobytes(order="C"))),
                         dtype=value.dtype).reshape(value.shape)


def project_frame(source, packets):
    """Project ALL packets once per original source event, before retention.

    Calls do not share a registry: repeating a call creates a distinct event
    identity and doubles its historical charge. The integration must reuse
    this one result, including its capsule when packets is empty. Different
    source objects are never merged merely because their stamps/bytes agree.
    """
    if type(source) is not original.SourceFrame or source._seal is not original._SOURCE_SEAL:
        raise ValueError("expected original complete validated source")
    packets = tuple(packets)
    keys = set()
    for packet in packets:
        if (type(packet) is not original.PairPacket
                or packet._seal is not original._PACKET_SEAL
                or packet.source is not source):
            raise ValueError("packet is not from this original source event")
        key = (packet.owner_id, packet.signed_side, packet.signature)
        if key in keys:
            raise ValueError("duplicate key frame lease")
        keys.add(key)
    frame = FrameIdentity(source.stamp_ns, len(source.face_ids), source.source_kind, _SEAL)
    blocks = {}

    def get_block(face):
        face = int(face)
        if face not in blocks:
            payload = b"".join(getattr(source, name)[face:face+1].tobytes(order="C")
                               for name, _, _, _, _ in _LAYOUT)
            if len(payload) != _ROW_BYTES:
                raise ValueError("complete row payload differs")
            blocks[face] = FaceBlock(frame, face, payload, _SEAL)
        return blocks[face]

    unowned = UnownedCapsule(frame, tuple(get_block(face) for face in np.flatnonzero(
        source.quad_valid & (source.endpoint_owners < 0))), _SEAL)
    projected = []
    for packet in packets:
        own_ids = np.flatnonzero((source.endpoint_owners == packet.owner_id)
                                & (source.valid | source.quad_valid))
        own_blocks = tuple(get_block(face) for face in own_ids)
        required_ids = sorted(set(map(int, own_ids))
                              | set(map(int, packet.face_indices.reshape(-1))))
        lease = KeyFrameLease(frame, packet.owner_id, packet.signed_side,
            packet.signature, own_blocks, tuple(get_block(face) for face in required_ids),
            unowned, _SEAL)
        projected.append(BlockPacket(lease, _frozen(packet.roles),
            _frozen(packet.face_indices), packet._has_reliable, _SEAL))
    return tuple(projected), unowned


def inventory(packets, capsules=()):
    """Distinct retained payload and unchanged historical charge, separately.

    ALL staged states/signatures and orphan capsules are caller-owned inputs.
    Source/key/frame/affine/piece/export charges outside source/packets must
    still be added by their original gates. Byte counts exclude Python object,
    allocator and transient-gather costs; they are not heap or runtime proof.
    """
    frames, blocks, packet_objects, unowned = {}, {}, {}, {}
    for packet in packets:
        if not is_block_packet(packet):
            raise ValueError("expected prepared retained block packet")
        packet_objects[id(packet)] = packet
        lease = packet.lease
        frames[id(lease.frame)] = lease.frame
        unowned[id(lease.unowned)] = lease.unowned
        for block in lease.required_blocks:
            blocks[id(block)] = block
    for capsule in tuple(capsules) + tuple(unowned.values()):
        if not is_unowned_capsule(capsule):
            raise ValueError("expected complete unowned capsule")
        frames[id(capsule.frame)] = capsule.frame
        for block in capsule.blocks:
            blocks[id(block)] = block
    frame_values, packet_values = tuple(frames.values()), tuple(packet_objects.values())
    faces = sum(frame.original_face_count for frame in frame_values)
    occurrences = sum(len(packet.roles) for packet in packet_values)
    face_bytes = sum(len(block.payload) for block in blocks.values())
    packet_bytes = sum(p.roles.nbytes + p.face_indices.nbytes for p in packet_values)
    return {
        "source_events": len(frames), "packets": len(packet_values),
        "pair_occurrences": occurrences, "legacy_source_faces": faces,
        "legacy_source_packet_weighted_charge": (
            sum(frame.legacy_support_records for frame in frame_values)
            + sum(10 + 2 * len(p.roles) for p in packet_values)),
        "legacy_source_packet_scalar_entries": (
            sum(frame.legacy_scalar_entries for frame in frame_values)
            + sum(15 + 3 * len(p.roles) for p in packet_values)),
        "legacy_source_packet_payload_bytes": faces * _ROW_BYTES + packet_bytes,
        "retained_complete_faces": len(blocks), "face_payload_bytes": face_bytes,
        "packet_payload_bytes": packet_bytes,
        "retained_array_payload_bytes": face_bytes + packet_bytes,
        "heap_or_runtime_acceptance": False,
    }
