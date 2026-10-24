"""Explicit development storage without a retained consensus-overlay table.

Every original retained frame, affine occurrence and packet remains reachable
by the exact same object.  The derived ``ConsensusPiece`` tuple and historical
overlay status/reason are omitted.
This representation is not enabled by default and is not accepted by compact
inventory/export APIs.  Callers still own the unchanged complete publication
inventory and capacity checks; this module changes no gate or coefficient.

``stage`` advances only the raw evidence and checks its capacity lower bound;
it does not establish readiness.  ``materialize`` invokes the current original
overlay once and applies the original complete per-key capacity check before
returning.  The complete global publication gate remains caller-owned.  A
zero-frame raw state naturally materializes as the original overlay's
``unconfirmed`` result, not the compact factory's historical ``empty`` status.
"""
from dataclasses import dataclass, field

from . import curve_consensus as arithmetic
from . import curve_consensus_compact as compact


_SEAL = object()


@dataclass(frozen=True, eq=False)
class RetainedCurveState:
    """Raw 5+1 evidence/chart only; no ``pieces``, ``status`` or ``reason``."""

    owner_id: int
    signed_side: int
    signature: tuple
    origin: tuple
    tangent: tuple
    normal_cosine: float
    max_support_records: int
    source_frontier_ns: int
    frames: tuple
    _seal: object = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        if type(self) is not RetainedCurveState or self._seal is not _SEAL:
            raise ValueError("use prepared retained curve factories")


def _require_retained(state):
    if type(state) is not RetainedCurveState or state._seal is not _SEAL:
        raise ValueError("expected prepared retained curve state")


def from_materialized(state):
    """Drop derived overlay/status; do not copy, trim or reinterpret frames."""
    if type(state) is not compact.CurveState or state._seal is not compact._SEAL:
        raise ValueError("expected prepared compact curve state")
    return RetainedCurveState(
        state.owner_id, state.signed_side, state.signature,
        state.origin, state.tangent, state.normal_cosine,
        state.max_support_records, state.source_frontier_ns, state.frames, _SEAL)


def stage(previous, current, *, available_before_ns):
    """Advance raw frames without overlay; this result is not publishable yet.

    The original shared frame helper keeps packet/type/key/time checks, 5+1
    selection and chart derivation.  Only the raw-header capacity lower bound
    is available here.  ``materialize`` must check the complete per-key charge
    including its transient overlay before a caller can publish a token.
    """
    _require_retained(previous)
    frames = compact._prepare_frames(
        previous, current, available_before_ns=available_before_ns)
    result = RetainedCurveState(
        previous.owner_id, previous.signed_side, previous.signature,
        previous.origin, previous.tangent, previous.normal_cosine,
        previous.max_support_records, current.stamp_ns, frames, _SEAL)
    key = (result.owner_id, result.signed_side, result.signature)
    if compact._inventory_parts({key: result}, ())["support_records"] > result.max_support_records:
        raise ValueError("complete compact support capacity exceeded")
    return result


def materialize(state):
    """Compute one exact overlay and check the original complete per-key gate.

    Returned geometry is transient.  Its caller must run the original complete
    map validation before publication, including when the arithmetic environment
    now produces a different status or different pieces.
    """
    _require_retained(state)
    pieces, status, reason = arithmetic._overlay(state, state.frames)
    result = compact.CurveState(
        state.owner_id, state.signed_side, state.signature,
        state.origin, state.tangent, state.normal_cosine,
        state.max_support_records, state.source_frontier_ns, state.frames,
        pieces, status, reason, compact._SEAL)
    if compact.inventory((result,))["support_records"] > state.max_support_records:
        raise ValueError("complete compact support capacity exceeded")
    return result


def advance(previous, current, *, available_before_ns):
    """Apply the original update once, returning a materialized compact state.

    The shared helper reads the old chart/key/frontier/frames, never old overlay
    pieces.  It retains the original packet/type/time checks, 5+1 selection,
    chart derivation, overlay and capacity rejection.  No confirmed-but-empty
    compact header is manufactured, and no second overlay is computed here.
    """
    _require_retained(previous)
    return compact._advance_prepared(
        previous, current, available_before_ns=available_before_ns)


from .curve_endpoint_native import _register_state_type as _register_endpoint_state
_register_endpoint_state(RetainedCurveState, _SEAL)
