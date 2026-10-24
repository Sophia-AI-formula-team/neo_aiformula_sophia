"""Shared immutable evidence for the explicit opt-in v5 development mapper.

TeachMapEngine/control do not enable v5; runtime acceptance is pending.

One table owns every original face once. Packets retain every directed pair,
role and occurrence in producer order, including cross-owner copies. No votes,
curve fit, permission, association or trimming is performed here. Semantic
support records and physical scalar/payload inventory have distinct units.
"""
from dataclasses import dataclass
from numbers import Integral

import numpy as np


REPRESENTATION = "shared_original_face_table_directed_occurrences_v1"
SOURCE_KIND = "actual_final_observation_directed_root_pairs"
MAX_RECORDS = 200000
SAME_OWNER_MODEL_NEXT = 1
CROSS_OWNER_INCIDENT = 2
_SOURCE_SEAL = object()
_PACKET_SEAL = object()
_FIELDS = (
    ("face_ids", np.int64, ()),
    ("previous_next", np.int64, (2,)),
    ("contour_ids", np.int64, ()),
    ("endpoint_owners", np.int64, ()),
    ("model_roots", np.float64, (2,)),
    ("normals", np.float64, (2,)),
    ("quads", np.float64, (4, 2)),
    ("quad_valid", np.bool_, ()),
    ("reliable", np.bool_, ()),
    ("valid", np.bool_, ()),
)


def _integer(value, name, minimum=0):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral):
        raise ValueError(name + " must be an integer")
    value = int(value)
    if not minimum <= value <= 2**63 - 1:
        raise ValueError(name + " outside source range")
    return value


def _freeze(value):
    # Avoid CPython's one-byte bytes interning for independently owned packets.
    backing = bytes(bytearray(value.tobytes(order="C")))
    return np.frombuffer(backing, dtype=value.dtype).reshape(value.shape)


def _native(value, dtype, shape, name):
    if (type(value) is not np.ndarray or value.dtype != np.dtype(dtype)
            or not value.dtype.isnative or value.shape != shape):
        raise ValueError("invalid original dtype/shape: " + name)


@dataclass(frozen=True)
class SourceFrame:
    """Complete source table, indexed by the original contiguous face IDs.

    Invalid placeholders and their exact bytes remain stored. A valid quad is
    independent of nominal-root validity: full source tables must not discard
    the old own-footprint evidence merely because a root was not admitted.
    Transport, pose and original producer authentication remain caller duties.
    """
    stamp_ns: int
    face_ids: np.ndarray
    previous_next: np.ndarray
    contour_ids: np.ndarray
    endpoint_owners: np.ndarray
    model_roots: np.ndarray
    normals: np.ndarray
    quads: np.ndarray
    quad_valid: np.ndarray
    reliable: np.ndarray
    valid: np.ndarray
    source_kind: str = SOURCE_KIND

    def __post_init__(self):
        stamp = _integer(self.stamp_ns, "stamp_ns", 1)
        if type(self.source_kind) is not str or self.source_kind != SOURCE_KIND:
            raise ValueError("unsupported original source kind")
        if type(self.face_ids) is not np.ndarray or self.face_ids.ndim != 1:
            raise ValueError("expected original one-dimensional face IDs")
        count = len(self.face_ids)
        if count > MAX_RECORDS:
            raise ValueError("source face record capacity")
        for name, dtype, tail in _FIELDS:
            _native(getattr(self, name), dtype, (count,) + tail, name)
        if (not np.array_equal(self.face_ids, np.arange(count, dtype=np.int64))
                or np.any(self.previous_next < -1)
                or np.any(self.previous_next >= count)
                or np.any(self.contour_ids < 0)
                or np.any(self.endpoint_owners < -1)):
            raise ValueError("original face/neighbor/contour/owner inventory differs")
        for side in (0, 1):
            known = self.previous_next[:, side] >= 0
            own = self.face_ids[known]
            other = self.previous_next[known, side]
            if (np.any(self.previous_next[other, 1 - side] != own)
                    or np.any(self.contour_ids[other] != self.contour_ids[own])):
                raise ValueError("original reciprocal contour links differ")
        if (np.any(self.reliable & ~self.valid)
                or np.any((self.endpoint_owners >= 0) != self.valid)):
            raise ValueError("original eligible face ownership/flags differ")
        for name, mask in (("model_roots", self.valid), ("normals", self.valid),
                           ("quads", self.quad_valid)):
            values = getattr(self, name)[mask]
            if not np.isfinite(values).all() or np.any(np.abs(values) > 1.e7):
                raise ValueError("invalid valid source geometry: " + name)
        # Same original unit tolerance; all valid faces checked once per frame.
        import math
        roundoff = 64. * np.finfo(np.float64).eps
        for normal in self.normals[self.valid]:
            if abs(math.hypot(float(normal[0]), float(normal[1])) - 1.) > roundoff:
                raise ValueError("valid source normal is not unit")
        for name, _, _ in _FIELDS:
            object.__setattr__(self, name, _freeze(getattr(self, name)))
        object.__setattr__(self, "stamp_ns", stamp)
        object.__setattr__(self, "_seal", _SOURCE_SEAL)

    @property
    def support_records(self):
        return 1 + len(self.face_ids)  # table header and every original face

    @property
    def scalar_entries(self):
        return 3 + sum(getattr(self, name).size + 1 + getattr(self, name).ndim
                       for name, _, _ in _FIELDS)

    @property
    def payload_bytes(self):
        return sum(getattr(self, name).nbytes for name, _, _ in _FIELDS)


@dataclass(frozen=True)
class PairPacket:
    """All ordered occurrences referencing one immutable original source.

    Repeated rows are deliberate records, not duplicates to remove. Semantic
    accounting charges each occurrence and its ordinal/role metadata record;
    source table storage is charged separately by the global reachable ledger.
    """
    source: SourceFrame
    owner_id: int
    signed_side: int
    signature: tuple
    roles: np.ndarray
    face_indices: np.ndarray

    def __post_init__(self):
        if type(self.source) is not SourceFrame or self.source._seal is not _SOURCE_SEAL:
            raise ValueError("expected immutable original source table")
        owner = _integer(self.owner_id, "owner_id")
        side = _integer(self.signed_side, "signed_side", -1)
        if side not in (-1, 1) or type(self.signature) is not tuple or len(self.signature) != 2:
            raise ValueError("invalid original side/signature")
        signature = tuple(_integer(value, "neighbor_owner", -1) for value in self.signature)
        if type(self.face_indices) is not np.ndarray or self.face_indices.ndim != 2:
            raise ValueError("expected original pair indices")
        count = len(self.face_indices)
        if count > MAX_RECORDS:
            raise ValueError("pair occurrence capacity")
        _native(self.face_indices, np.int64, (count, 2), "face_indices")
        _native(self.roles, np.uint8, (count,), "roles")
        if np.any(self.face_indices < 0) or np.any(self.face_indices >= len(self.source.face_ids)):
            raise ValueError("pair source index outside original table")
        first, second = self.face_indices.T
        table = self.source
        if (np.any(first == second) or np.any(table.previous_next[first, 1] != second)
                or np.any(table.previous_next[second, 0] != first)
                or np.any(table.contour_ids[first] != table.contour_ids[second])):
            raise ValueError("pair is not original reciprocal directed next")
        owners = table.endpoint_owners[self.face_indices]
        same, incident = self.roles == SAME_OWNER_MODEL_NEXT, self.roles == CROSS_OWNER_INCIDENT
        if (not np.all(same | incident) or np.any(owners < 0)
                or np.any(same & np.any(owners != owner, axis=1))
                or np.any(incident & (owners[:, 0] == owners[:, 1]))
                or np.any(incident & ~np.any(owners == owner, axis=1))):
            raise ValueError("source pair role/endpoint owner differs")
        object.__setattr__(self, "owner_id", owner)
        object.__setattr__(self, "signed_side", side)
        object.__setattr__(self, "signature", signature)
        object.__setattr__(self, "roles", _freeze(self.roles))
        object.__setattr__(self, "face_indices", _freeze(self.face_indices))
        object.__setattr__(self, "_has_reliable", bool(np.any(np.all(
            table.reliable[self.face_indices], axis=1))))
        object.__setattr__(self, "_seal", _PACKET_SEAL)

    @property
    def stamp_ns(self):
        return self.source.stamp_ns

    @property
    def support_records(self):
        return 10 + 2 * len(self.roles)  # full key/ref metadata and all ordinals

    @property
    def scalar_entries(self):
        return 10 + self.roles.size + 2 + self.face_indices.size + 3

    @property
    def payload_bytes(self):
        return self.roles.nbytes + self.face_indices.nbytes

    def gather_original(self, name):
        """Explicit transient reconstruction for byte-oracle/legacy adapters.

        The gather is not retained by this packet and is NOT a zero-copy claim.
        Compute/transient allocation costs belong inside the calling budget.
        """
        if name not in {field for field, _, _ in _FIELDS}:
            raise ValueError("unknown original source field")
        return getattr(self.source, name)[self.face_indices]

    @property
    def normals(self):
        # Legacy nominal-overlay compatibility only; this allocates a gather.
        return self.gather_original("normals")


def reachable_inventory(packets):
    """Count FULL reachable tables once, all distinct packet objects once.

    A single stale owner can pin an entire old source table: count that table
    completely, not just its current slices. No global registry retains stale
    tables. The caller must provide packets from ALL staged states/signatures,
    including UNKNOWN and non-updated owners, not a spatial/query subset.
    """
    sources, records = {}, {}
    for packet in packets:
        if type(packet) is not PairPacket or packet._seal is not _PACKET_SEAL:
            raise ValueError("expected prepared pair packet inventory")
        records[id(packet)] = packet
        sources[id(packet.source)] = packet.source
    values = tuple(sources.values()) + tuple(records.values())
    return {"source_tables": len(sources), "packets": len(records),
            "source_faces": sum(len(value.face_ids) for value in sources.values()),
            "pair_occurrences": sum(len(value.roles) for value in records.values()),
            "support_records": sum(value.support_records for value in values),
            "scalar_entries": sum(value.scalar_entries for value in values),
            "payload_bytes": sum(value.payload_bytes for value in values)}
