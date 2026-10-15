"""Close only evidence-certified numerical seams in a finite root curve.

This is an explicit development geometry model, not an epsilon/gate change.
Five original source paths and the original five-vote pixel domain must cover
the modified primitive. No owner/signature is merged, no raw evidence removed.
The next primitive adopts the preceding primitive's already finite endpoint;
its direction is recomputed and checked against ALL contributing source normals.
Exact shared-vertex component unions are converted inward only at the OUTER
endpoints, avoiding artificial UNKNOWN slivers at every floating-point vertex.
"""
from fractions import Fraction as F
import math

import numpy as np

from . import curve_consensus as arithmetic
from . import curve_consensus_compact as compact
from .curve_rounding import step
from .finite_support import DomainState


def _projection(point, state):
    return sum(F(t)*(F(float(x))-F(o)) for t, x, o in zip(state.tangent, point, state.origin))


def _inner(value, lower):
    converted = float(value)
    if not math.isfinite(converted):
        raise ArithmeticError('unrepresentable closed curve support')
    with np.errstate(all='ignore'):
        if (F(converted) < value) if lower else (F(converted) > value):
            converted = step(converted, math.inf if lower else -math.inf)
    return converted


def _frame_data(state, frame):
    roots = frame.packet.gather_original('model_roots')
    ids = frame.packet.gather_original('face_ids')
    normals = frame.packet.gather_original('normals')
    rows = []
    for piece in frame.pieces:
        index = piece.occurrence
        lo, hi = piece.lower_endpoint, piece.upper_endpoint
        a, b = roots[index, lo], roots[index, hi]
        left, right = _projection(a, state), _projection(b, state)
        if not left < right:
            raise ArithmeticError('nonfunctional original closed source chart')
        rows.append((left, right, (int(ids[index, lo]), a.tobytes()),
                     (int(ids[index, hi]), b.tobytes()), bool(piece.reliable), normals[index]))
    return tuple(rows)


def _covers(rows, lower, upper, *, reliable=False):
    start = end = previous = None
    for a, b, first, last, trusted, _ in rows:
        if reliable and not trusted:
            start = end = previous = None
            continue
        if previous == first and end == a:
            end = b
        else:
            start, end = a, b
        previous = last
        if start <= lower < upper <= end:
            return True
    return False


def close_supported_seams(state, domain, local, cosine):
    """Return (finite exports, closed-seam count), without publishing anything.

    Local exports must come from this key's unchanged _finite_piece operation.
    A missing/ambiguous prerequisite leaves that seam unchanged, not accepted.
    A source frame may cross a vertex only by original face ID AND exact root
    bytes, never coordinate coincidence between distinct branches/contours.
    """
    if (type(state) is not compact.CurveState or state._seal is not compact._SEAL
            or type(domain) is not DomainState
            or (state.origin, state.tangent) != (domain.origin, domain.tangent)
            or state.status != 'confirmed' or not 0. < cosine <= 1.):
        raise ValueError('expected one confirmed original compact key/domain')
    exports = sorted(tuple(local), key=lambda item: item[-1])
    if not exports:
        return (), 0
    data = {frame.packet.stamp_ns: _frame_data(state, frame) for frame in state.frames}
    if len(data) != len(state.frames):
        raise ValueError('duplicate original frame witness')

    def parent(item):
        lower, upper = item[-1]
        matches = [(index, piece) for index, piece in enumerate(state.pieces)
                   if piece.lower <= lower < upper <= piece.upper]
        return matches[0] if len(matches) == 1 else None

    original_parents = [parent(item) for item in exports]
    count = 0
    for index in range(1, len(exports)):
        left, right = exports[index-1], exports[index]
        a, b = original_parents[index-1], original_parents[index]
        if a is None or b is None or b[0] != a[0]+1:
            continue
        # No omitted consensus event and no domain hole may be bridged.
        first, last = left[0][1], right[0][1]
        lower, upper = _projection(first, state), _projection(last, state)
        if not lower < upper or not any(
                interval.frame_votes >= 5 and interval.reliable_votes >= 1
                and F(interval.lower) <= lower < upper <= F(interval.upper)
                for interval in domain.intervals):
            continue
        piece = b[1]
        contributors = piece.contributors
        if (len(contributors) < 5 or any(stamp not in data or not _covers(data[stamp], lower, upper)
                                         for stamp in contributors)
                or not any(stamp in data and _covers(data[stamp], lower, upper, reliable=True)
                           for stamp in piece.reliable_contributors)):
            continue
        # The original contributor order/frames stay intact. A different source
        # path can never be inferred from nearby roots or a convenient mean.
        delta = tuple(y-x for x, y in zip(first, last))
        try:
            normal = arithmetic._normal(delta)
        except ArithmeticError:
            continue
        if any(arithmetic._dot(normal, n) < cosine
               for stamp in contributors for _, _, _, _, _, pair in data[stamp] for n in pair):
            continue
        exports[index] = ((first, last), normal, right[2], right[3], right[4], right[5])
        count += 1

    # Only literal shared finite endpoints establish a connected nominal model.
    # Do not merge merely overlapping chart projections from disconnected paths.
    components, members = [], []
    for index, item in enumerate(exports):
        a, b = (_projection(point, state) for point in item[0])
        if not a < b:
            raise ArithmeticError('closed curve lost strict chart direction')
        if (members and exports[index-1][0][1] == item[0][0]
                and components[-1][1] == a):
            components[-1] = (components[-1][0], b)
            members[-1].append(index)
        else:
            components.append((a, b)); members.append([index])
    for (lower, upper), indices in zip(components, members):
        # The endpoints themselves remain nominal geometry. A conservatively
        # inward component parameter range is sufficient to exclude no source.
        covered = (_inner(lower, True), _inner(upper, False))
        if not covered[0] < covered[1]:
            continue
        for index in indices:
            item = exports[index]
            exports[index] = item[:-1]+(covered,)
    return tuple(exports), count
