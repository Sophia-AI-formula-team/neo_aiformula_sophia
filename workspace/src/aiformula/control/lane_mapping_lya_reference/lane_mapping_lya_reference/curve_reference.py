"""Explicit immutable v5 curve geometry and rejection-only competitors.

Topology is not nominal geometry. Unknown support boxes only veto a nearest
match; they must never produce a residual or expose a farther convenient line.
An owner of -1 on a blocker denotes an independent unassociated source
footprint, never a nominal owner or permission-bearing graph node.
"""
from dataclasses import dataclass

import numpy as np


REPRESENTATION = "finite_curve_consensus_with_unknown_competitors_v5"
MAX_RECORDS = 200000
MAX_COORDINATE = 1.e7
UNIT_TOLERANCE = 64. * np.finfo(np.float64).eps


def _array(value, dtype, shape, name):
    raw = np.asarray(value)
    expected = np.dtype(dtype)
    if raw.shape != shape or raw.dtype.hasobject:
        raise ValueError("invalid curve " + name + " shape/type")
    if raw.size and raw.dtype.kind not in ("iu" if expected.kind == "i" else "fi"):
        raise ValueError("invalid curve " + name + " dtype")
    if expected.kind == "i" and raw.size:
        info = np.iinfo(expected)
        if np.any(raw < info.min) or np.any(raw > info.max):
            raise ValueError("curve " + name + " outside integer range")
    data = np.asarray(raw, dtype=expected)
    if expected.kind == "f" and (not np.isfinite(data).all()
                                  or np.any(np.abs(data) > MAX_COORDINATE)):
        raise ValueError("nonfinite/out-of-bound curve " + name)
    return np.frombuffer(data.tobytes(order="C"), dtype=expected).reshape(shape)


@dataclass(frozen=True)
class CurveGeometry:
    segments: object
    normals: object
    owner: object
    frame_votes: object
    reliable_votes: object
    last_stamp_ns: object
    owner_status: object
    blocker_boxes: object
    blocker_owner: object
    complete: bool

    def __post_init__(self):
        if type(self.complete) is not bool:
            raise ValueError("curve completeness must be explicit bool")
        status = np.asarray(self.owner_status)
        segments, boxes = np.asarray(self.segments), np.asarray(self.blocker_boxes)
        if (status.ndim != 1 or len(status) > 100000
                or segments.ndim != 3 or segments.shape[1:] != (2, 2)
                or boxes.ndim != 3 or boxes.shape[1:] != (2, 2)
                or len(segments) + len(boxes) > MAX_RECORDS):
            raise ValueError("invalid curve geometry dimensions/capacity")
        n, count, blockers = len(status), len(segments), len(boxes)
        fields = (("segments", np.float64, (count, 2, 2)),
                  ("normals", np.float64, (count, 2)),
                  ("owner", np.int64, (count,)),
                  ("frame_votes", np.int64, (count,)),
                  ("reliable_votes", np.int64, (count,)),
                  ("last_stamp_ns", np.int64, (count,)),
                  ("owner_status", np.int64, (n,)),
                  ("blocker_boxes", np.float64, (blockers, 2, 2)),
                  ("blocker_owner", np.int64, (blockers,)))
        for name, dtype, shape in fields:
            object.__setattr__(self, name, _array(getattr(self, name), dtype, shape, name))
        if (np.any(self.owner < 0) or np.any(self.owner >= n)
                or np.any(self.blocker_owner < -1) or np.any(self.blocker_owner >= n)
                or np.any(self.owner_status < 0) or np.any(self.owner_status > 2)
                or np.any(self.frame_votes < 1) or np.any(self.frame_votes > 2**31 - 1)
                or np.any(self.reliable_votes < 1)
                or np.any(self.reliable_votes > self.frame_votes)
                or np.any(self.last_stamp_ns <= 0)):
            raise ValueError("invalid curve owner/evidence")
        delta = self.segments[:, 1] - self.segments[:, 0]
        length_squared = np.sum(delta * delta, axis=1)
        if (np.any(length_squared <= 0.)
                or np.any(np.abs(np.linalg.norm(self.normals, axis=1) - 1.) > UNIT_TOLERANCE)
                or np.any(self.blocker_boxes[:, 0] > self.blocker_boxes[:, 1])):
            raise ValueError("invalid curve segment/normal/blocker box")
        # A unit vector alone is not a line normal. Keep its supplied sign,
        # but reject a residual direction inconsistent with this finite line.
        # This is binary64 representation consistency, not an angle relaxation.
        tangent = delta / np.sqrt(length_squared[:, None])
        if np.any(np.abs(np.sum(tangent * self.normals, axis=1)) > UNIT_TOLERANCE):
            raise ValueError("curve normal must be perpendicular to its segment")

    def subset(self, indices):
        raw = np.asarray(indices)
        if (raw.ndim != 1 or (raw.size and raw.dtype.kind not in "iu")
                or np.any(raw < 0) or np.any(raw >= len(self.owner_status))):
            raise ValueError("invalid curve subset indices")
        ids = np.asarray(raw, dtype=np.int64)
        if len(np.unique(ids)) != len(ids):
            raise ValueError("duplicate curve subset indices")
        remap = np.full(len(self.owner_status), -1, dtype=np.int64)
        remap[ids] = np.arange(len(ids))
        owners = remap[self.owner]
        associated = self.blocker_owner >= 0
        blockers = np.full(len(self.blocker_owner), -1, dtype=np.int64)
        blockers[associated] = remap[self.blocker_owner[associated]]
        # Standalone UNKNOWN footprints have no node to crop. Keep ALL of
        # them even for an empty owner subset; distance alone may exclude one.
        keep, keep_blocker = owners >= 0, ~associated | (blockers >= 0)
        return CurveGeometry(self.segments[keep], self.normals[keep], owners[keep],
            self.frame_votes[keep], self.reliable_votes[keep], self.last_stamp_ns[keep],
            self.owner_status[ids], self.blocker_boxes[keep_blocker], blockers[keep_blocker],
            self.complete)
