"""Native current-frame union dispatcher and state integration tests.

Production integration is expected to expose private _NATIVE_CURRENT_UNION
and keep _current_union as the unchanged public-state caller's dispatcher.
The pure oracle is embedded in the sibling test file, never imported from
production or a diagnostic path. Actual-native tests require API9, not API8.
No mapping engine, estimator, timing benchmark or physical acceptance here.
"""
import math
import struct
from unittest.mock import patch

import numpy as np
import pytest

from lane_mapping_lya_reference import finite_support as finite
from test_native_current_union import (native9, oracle_current_union,
    oracle_projection, pieces_bytes, unit_normals, old_admission)


@pytest.fixture
def dispatch_ready():
    assert hasattr(finite, "_NATIVE_CURRENT_UNION"), (
        "native9 dispatcher has not been integrated; this draft cannot prove it")
    return finite


def plain_segments():
    return np.array([[[0., 0.], [1., 0.]], [[1., 0.], [2., 0.]],
                     [[2., 0.], [3., 0.]]], dtype=np.float64)


def scalar_backend(*args):
    """Independent stand-in for packing/call tests, NOT a native success."""
    payload, mask, reliable, count, ox, oy, tx, ty = args
    assert type(payload) is bytes and type(mask) is bytes and type(reliable) is bytes
    assert type(count) is int and len(payload) == count*32
    assert len(mask) == len(reliable) == count
    assert all(type(value) is float for value in (ox, oy, tx, ty))
    assert set(mask) <= {0, 1} and set(reliable) <= {0, 1}
    segments = np.frombuffer(payload, dtype=np.float64).reshape(count, 2, 2)
    normals = unit_normals(count, (tx, ty))
    normals[np.frombuffer(mask, dtype=np.uint8) == 0] *= -1.
    result = oracle_current_union(segments, normals, np.frombuffer(reliable, dtype=np.uint8) != 0,
                                  (ox, oy), (tx, ty), .5)
    return tuple(result)


def call_union(module, segments=None, normals=None, reliable=None, cosine=.5):
    segments = plain_segments() if segments is None else segments
    normals = unit_normals(len(segments)) if normals is None else normals
    reliable = np.ones(len(segments), bool) if reliable is None else reliable
    return module._current_union(segments, normals, reliable, (0., 0.), (1., 0.), cosine)


def test_eligible_dispatch_calls_once_and_returns_list(dispatch_ready, monkeypatch):
    calls = []
    def capture(*args):
        calls.append(args)
        return scalar_backend(*args)
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", capture)
    result = call_union(finite)
    assert type(result) is list and len(calls) == 1
    assert calls[0][:4] == (plain_segments().tobytes(), b"\1\1\1", b"\1\1\1", 3)
    expected = oracle_current_union(plain_segments(), unit_normals(3), np.ones(3, bool),
                                    (0., 0.), (1., 0.), .5)
    assert pieces_bytes(result) == pieces_bytes(expected)


def test_two_numpy_dots_keep_or_short_circuit_and_order(dispatch_ready, monkeypatch):
    segments = plain_segments()
    normals = np.array([[[0., -1.], [1., 0.]],
                        [[0., 1.], [0., -1.]],
                        [[0., 1.], [0., 1.]]])
    expected = oracle_current_union(segments, normals, np.ones(3, bool),
                                    (0., 0.), (1., 0.), .5)
    dots, calls = [], []
    original_dot = np.dot
    def dot(first, second):
        dots.append((first.tobytes(), second.tobytes()))
        return original_dot(first, second)
    def capture(*args):
        calls.append(args)
        # No oracle call inside the spy: it would add unrelated np.dot calls.
        return tuple(expected)
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", capture)
    with patch.object(finite.np, "dot", dot):
        result = call_union(finite, segments, normals)
    intended = [normals[0, 0], normals[1, 0], normals[1, 1], normals[2, 0], normals[2, 1]]
    assert [row[0] for row in dots] == [value.tobytes() for value in intended]
    assert calls[0][1] == b"\0\0\1" and len(calls) == 1
    assert pieces_bytes(result) == pieces_bytes(expected)


@pytest.mark.parametrize("first,second,expected", [
    (float.fromhex("0x1.fffffffffffffp-2"), 1., 0),
    (.5, 1., 1), (float.fromhex("0x1.0000000000001p-1"), 1., 1),
    (1., float.fromhex("0x1.fffffffffffffp-2"), 0)])
def test_gate_boundary_admitted_bytes(dispatch_ready, monkeypatch, first, second, expected):
    calls = []
    def capture(*args):
        calls.append(args)
        return ()
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", capture)
    # Private dispatch input isolates literal dot/gate arithmetic; public normal
    # validation is independently tested below and is not bypassed in runtime.
    normals = np.array([[[0., first], [0., second]]])
    finite._current_union(plain_segments()[:1], normals, np.ones(1, bool),
                          (0., 0.), (1., 0.), .5)
    assert calls[0][1] == bytes([expected])


def test_noncanonical_bool_backing_is_scalar_truth(dispatch_ready, monkeypatch):
    reliable = np.array([0, 2, 255], dtype=np.uint8).view(np.bool_)
    assert reliable.tobytes() == b"\0\2\xff"
    calls = []
    def capture(*args):
        calls.append(args)
        return scalar_backend(*args)
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", capture)
    result = call_union(finite, reliable=reliable)
    assert calls[0][2] == b"\0\1\1"
    expected = oracle_current_union(plain_segments(), unit_normals(3), reliable,
                                    (0., 0.), (1., 0.), .5)
    assert pieces_bytes(result) == pieces_bytes(expected)


class ArraySubclass(np.ndarray):
    pass


@pytest.mark.parametrize("representation", ["segment-strides", "normal-strides",
    "reliable-strides", "segment-subclass", "normal-subclass", "reliable-subclass",
    "big-endian", "float32"])
def test_unsupported_arrays_keep_full_old_path(dispatch_ready, monkeypatch, representation):
    segments, normals, reliable = plain_segments(), unit_normals(3), np.ones(3, bool)
    if representation == "segment-strides":
        segments = np.repeat(segments, 2, axis=0)[::2]
    elif representation == "normal-strides":
        normals = np.repeat(normals, 2, axis=0)[::2]
    elif representation == "reliable-strides":
        reliable = np.ones(6, bool)[::2]
    elif representation == "segment-subclass":
        segments = segments.view(ArraySubclass)
    elif representation == "normal-subclass":
        normals = normals.view(ArraySubclass)
    elif representation == "reliable-subclass":
        reliable = reliable.view(ArraySubclass)
    elif representation == "big-endian":
        segments, normals = segments.astype(">f8"), normals.astype(">f8")
    elif representation == "float32":
        segments, normals = segments.astype(np.float32), normals.astype(np.float32)
    def forbidden(*args):
        pytest.fail("unsupported input reached native9")
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", forbidden)
    expected = oracle_current_union(segments, normals, reliable, (0., 0.), (1., 0.), .5)
    assert pieces_bytes(call_union(finite, segments, normals, reliable)) == pieces_bytes(expected)


@pytest.mark.parametrize("under", ["raise", "warn", "call", "print", "log"])
def test_non_ignore_underflow_policy_keeps_old_path(dispatch_ready, monkeypatch, under):
    def forbidden(*args):
        pytest.fail("strict/warning underflow policy reached native9")
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", forbidden)
    with np.errstate(under=under):
        expected = oracle_current_union(plain_segments(), unit_normals(3), np.ones(3, bool),
                                        (0., 0.), (1., 0.), .5)
        assert pieces_bytes(call_union(finite)) == pieces_bytes(expected)


def test_native_exception_propagates_without_retry(dispatch_ready, monkeypatch):
    calls = []
    class Sentinel(RuntimeError):
        pass
    def fail(*args):
        calls.append(args)
        raise Sentinel("native dirty error")
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", fail)
    with pytest.raises(Sentinel, match="native dirty error"):
        call_union(finite)
    assert len(calls) == 1


def advance(module, previous=None, stamp=1, **changes):
    kwargs = dict(origin=(0., 0.), current_frame_normal=(0., 1.),
        segments=plain_segments(), normals=unit_normals(3), reliable=np.ones(3, bool),
        stamp_ns=stamp, minimum_frame_votes=5, minimum_reliable_votes=1,
        candidate_ttl_ns=4, normal_cosine=.5, max_intervals=100)
    kwargs.update(changes)
    return module.advance_domain(previous, **kwargs)


def state_bytes(state):
    if state is None:
        return None
    return (tuple(struct.pack("=d", v) for v in state.origin),
            tuple(struct.pack("=d", v) for v in state.tangent),
            state.source_stamp_ns,
            tuple((struct.pack("=d", p.lower), struct.pack("=d", p.upper),
                   p.frame_votes, p.reliable_votes, p.last_stamp_ns) for p in state.intervals))


def export_bytes(module, state):
    result = module.certified_segments(state, (.5, 0.), (0., 1.), .5, 5, 1)
    return tuple((value.dtype.str, value.shape, value.tobytes()) for value in result)


def outcome(function):
    try:
        return ("value", state_bytes(function()))
    except Exception as error:
        return ("error", type(error), str(error))


@pytest.mark.parametrize("changes", [
    {"stamp_ns": True}, {"stamp_ns": 0}, {"stamp_ns": 2**63},
    {"origin": (math.nan, 0.)}, {"current_frame_normal": (0., 2.)},
    {"minimum_frame_votes": 0}, {"minimum_reliable_votes": 6},
    {"candidate_ttl_ns": -1}, {"normal_cosine": 0.}, {"max_intervals": 0},
    {"segments": np.array([[[math.nan, 0.], [1., 0.]]]), "normals": unit_normals(1),
     "reliable": np.ones(1, bool)},
    {"reliable": np.array([1, 1, 1], dtype=np.uint8)},
    # Multiple bad values preserve public scalar-before-array-before-state.
    {"origin": (math.inf, 0.), "segments": None},
])
def test_public_validation_precedes_native_and_keeps_exact_exception(dispatch_ready, monkeypatch, changes):
    calls = []
    def forbidden(*args):
        calls.append(args)
        pytest.fail("invalid public input reached native9")
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", None)
    before = outcome(lambda: advance(finite, **changes))
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", forbidden)
    after = outcome(lambda: advance(finite, **changes))
    assert before[0] == "error" and after == before and calls == []


def test_previous_source_and_capacity_validation_before_native(dispatch_ready, monkeypatch):
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", None)
    old = advance(finite, stamp=3)
    saved = state_bytes(old)
    def forbidden(*args):
        pytest.fail("invalid previous/frontier reached native9")
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", forbidden)
    for stamp in (1, 3):
        with pytest.raises(ValueError, match="strictly increase"):
            advance(finite, old, stamp)
    with pytest.raises(ValueError, match="previous must be"):
        advance(finite, object(), 4)
    assert state_bytes(old) == saved


def test_native_error_does_not_mutate_previous_state(dispatch_ready, monkeypatch):
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", None)
    old = advance(finite)
    saved = state_bytes(old)
    def fail(*args):
        raise ValueError("native sentinel after validation")
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", fail)
    with pytest.raises(ValueError, match="native sentinel"):
        advance(finite, old, 2)
    assert state_bytes(old) == saved


def test_actual_native9_dispatch_not_only_import(dispatch_ready, native9, monkeypatch):
    calls = []
    def wrapped(*args):
        calls.append(args)
        return native9.finite_current_union(*args)
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", wrapped)
    result = call_union(finite)
    assert len(calls) == 1
    expected = oracle_current_union(plain_segments(), unit_normals(3), np.ones(3, bool),
                                    (0., 0.), (1., 0.), .5)
    assert pieces_bytes(result) == pieces_bytes(expected)


def test_whole_advance_export_saturation_ttl_and_duplicates(dispatch_ready, native9, monkeypatch):
    old_state = new_state = None
    base = plain_segments()
    events = []
    for stamp in range(1, 7):
        points = np.repeat(base, 2, axis=0) if stamp == 3 else base
        events.append((stamp, points, np.ones(len(points), bool)))
    # Extra temporary unconfirmed domain; then a gap/empty source frontier.
    events.extend([(7, np.array([[[10., 0.], [11., 0.]]]), np.zeros(1, bool)),
                   (8, np.empty((0, 2, 2)), np.empty(0, bool)),
                   (20, np.empty((0, 2, 2)), np.empty(0, bool)),
                   (21, base, np.ones(3, bool))])
    calls = []
    def wrapped(*args):
        calls.append(args)
        return native9.finite_current_union(*args)
    original_union = finite._current_union
    original_projection = finite._validated_projection_bounds
    for stamp, points, trusted in events:
        changes = dict(segments=points, normals=unit_normals(len(points)), reliable=trusted)
        old_before, new_before = state_bytes(old_state), state_bytes(new_state)
        with monkeypatch.context() as context:
            context.setattr(finite, "_current_union", oracle_current_union)
            context.setattr(finite, "_validated_projection_bounds", oracle_projection)
            old_next = advance(finite, old_state, stamp, **changes)
            old_export = export_bytes(finite, old_next)
        assert finite._current_union is original_union
        assert finite._validated_projection_bounds is original_projection
        monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", wrapped)
        new_next = advance(finite, new_state, stamp, **changes)
        assert state_bytes(old_next) == state_bytes(new_next)
        assert export_bytes(finite, new_next) == old_export
        assert state_bytes(old_state) == old_before and state_bytes(new_state) == new_before
        old_state, new_state = old_next, new_next
    assert len(calls) == len(events)
    assert len(new_state.intervals) == 1
    assert (new_state.intervals[0].frame_votes, new_state.intervals[0].reliable_votes) == (5, 1)
    for array in finite.certified_segments(new_state, (.5, 0.), (0., 1.), .5, 5, 1):
        assert not array.flags.writeable
        with pytest.raises(ValueError):
            array.flags.writeable = True


def test_actual_native_capacity_failure_is_transactional(dispatch_ready, native9, monkeypatch):
    monkeypatch.setattr(finite, "_NATIVE_CURRENT_UNION", native9.finite_current_union)
    old = advance(finite)
    saved = state_bytes(old)
    separated = np.array([[[10., 0.], [11., 0.]], [[20., 0.], [21., 0.]]])
    with pytest.raises(ValueError, match="interval capacity"):
        advance(finite, old, 2, segments=separated, normals=unit_normals(2),
                reliable=np.ones(2, bool), max_intervals=1)
    assert state_bytes(old) == saved
