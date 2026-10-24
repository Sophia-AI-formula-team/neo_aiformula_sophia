"""R26 batching equivalence, not a runtime or vehicle acceptance test.

The preparation oracle is the literal R23 body retained in the R25 test. Its
global retain_frames binding is replaced with the independent pre-cache body
below, not the new implementation. Public ContinuationFrame remains the old
array-construction oracle. No Git, external recordings, or unintegrated fitting
prototype is imported. All persisted owner buffers are compared as raw bytes.
"""
from collections.abc import Mapping
from dataclasses import asdict, fields, is_dataclass, replace
import math
import struct
from types import FunctionType, MappingProxyType, SimpleNamespace

import numpy as np
import pytest

from lane_mapping_lya_reference import continuation_support as support
from lane_mapping_lya_reference import paint_geometry as paint
from test_continuation_cache_equivalence import (
    _LiteralConsensus as _R23LiteralConsensus,
    _observation, _updates, _owners, _frame,
)


DT = 100_000_000
COSINE = math.cos(math.pi / 6.)


def _literal_retain(previous, current, minimum_frames, minimum_reliable):
    ordered = tuple(previous) + (current,)
    if (minimum_frames < 1 or not 1 <= minimum_reliable <= minimum_frames
            or any(second.stamp_ns <= first.stamp_ns
                   for first, second in zip(ordered, ordered[1:]))):
        raise ValueError('invalid continuation witness order/count')
    frames = ordered
    keep = {frame.stamp_ns for frame in frames[-minimum_frames:]}
    reliable = [frame for frame in frames if np.any(frame.reliable)]
    keep.update(frame.stamp_ns for frame in reliable[-minimum_reliable:])
    return tuple(frame for frame in frames if frame.stamp_ns in keep)


class _LiteralConsensus(_R23LiteralConsensus):
    # Copy the literal function's globals, not its numerical body. This avoids
    # sharing the new cache implementation or mutating another test module.
    _prepare_continuations = FunctionType(
        _R23LiteralConsensus._prepare_continuations.__code__,
        dict(_R23LiteralConsensus._prepare_continuations.__globals__,
             retain_frames=_literal_retain),
        '_prepare_continuations')


def _exact(value):
    if isinstance(value, np.ndarray):
        return ('array', value.dtype.str, value.shape, value.tobytes())
    if is_dataclass(value):
        return tuple((item.name, _exact(getattr(value, item.name)))
                     for item in fields(value) if item.name != 'owner')
    if isinstance(value, Mapping):
        return tuple((_exact(key), _exact(item)) for key, item in value.items())
    if isinstance(value, (tuple, list)):
        return tuple(_exact(item) for item in value)
    if isinstance(value, (float, np.floating)):
        return ('float64', struct.pack('>d', float(value)))
    return (type(value).__name__, value)


def _state(consensus):
    return _exact(vars(consensus))


def _backing(array):
    while isinstance(array, np.ndarray):
        assert array.base is not None
        array = array.base
    assert type(array) is bytes
    return array


def _packed(lengths=(2, 0, 3)):
    lengths = tuple(lengths)
    count = sum(lengths)
    x = np.arange(count, dtype=np.float64) * .125
    points = np.column_stack((x, np.full(count, -.25)))
    quads = np.stack((points+[-.01, -.02], points+[.01, -.02],
                      points+[.01, .02], points+[-.01, .02]), axis=1)
    normals = np.tile([-0., 1.], (count, 1))
    # Bool backing need not be canonical 0/1. Public truth and raw copying are
    # independently required; do not turn byte 2 into False or silently recode.
    reliable = np.resize(np.array([0, 2, 255], dtype=np.uint8), count).view(np.bool_)
    offsets = np.r_[0, np.cumsum(lengths, dtype=np.int64)]
    return quads, normals, reliable, offsets


def _batch(lengths=(2, 0, 3), stamp=DT):
    arrays = _packed(lengths)
    token = support._validate_continuation_batch(stamp, *arrays)
    assert token is not None
    return arrays, token


@pytest.mark.parametrize('lengths', [(), (0,), (1, 0, 3), (2, 3), (0, 2, 0)])
def test_batch_public_owner_bytes_and_separate_immutable_backings(lengths):
    arrays, token = _batch(lengths)
    quads, normals, reliable, offsets = arrays
    before = _exact(arrays)
    expected = [support.ContinuationFrame(DT, quads[a:b], normals[a:b], reliable[a:b])
                for a, b in zip(offsets, offsets[1:])]
    actual = [support._continuation_frame_from_batch(token, index)
              for index in range(len(lengths))]
    assert _exact(actual) == _exact(expected)
    assert _exact(arrays) == before
    owner_backings = {name: [] for name in ('quads', 'normals', 'reliable')}
    for owner, frame in enumerate(actual):
        assert frame.storage_units == 1+lengths[owner]
        for name in owner_backings:
            array = getattr(frame, name)
            assert array.flags.c_contiguous and not array.flags.writeable
            with pytest.raises(ValueError):
                array.setflags(write=True)
            root = _backing(array)
            assert len(root) == array.nbytes
            assert not np.shares_memory(array, getattr(token, name))
            if array.size:
                assert root is not _backing(getattr(token, name))
                assert all(root is not prior for prior in owner_backings[name])
                owner_backings[name].append(root)
    saved = _exact((token, actual))
    quads[...] = 99.
    normals[...] = 0.
    reliable[...] = False
    offsets[...] = 0
    assert _exact((token, actual)) == saved
    for name in ('quads', 'normals', 'reliable', 'offsets'):
        with pytest.raises(ValueError):
            getattr(token, name).setflags(write=True)


class _ArraySubclass(np.ndarray):
    pass


@pytest.mark.parametrize('case', [
    'stamp_zero', 'stamp_bool', 'stamp_numpy', 'list', 'subclass',
    'float32', 'big_endian', 'strided', 'integer_reliable', 'float_offsets',
    'quad_shape', 'normal_shape', 'reliable_shape', 'empty_offsets',
    'offset_start', 'offset_stop', 'offset_negative', 'offset_decrease',
    'nan_quad', 'inf_normal', 'huge_normal', 'zero_normal', 'nonunit_normal',
])
def test_ineligible_preflight_returns_none_without_conversion_or_fp_exception(case):
    stamp = DT
    quads, normals, reliable, offsets = _packed((2, 3))
    if case == 'stamp_zero':
        stamp = 0
    elif case == 'stamp_bool':
        stamp = True
    elif case == 'stamp_numpy':
        stamp = np.int64(DT)
    elif case == 'list':
        quads = quads.tolist()
    elif case == 'subclass':
        quads = quads.view(_ArraySubclass)
    elif case == 'float32':
        normals = normals.astype(np.float32)
    elif case == 'big_endian':
        normals = normals.astype('>f8' if np.little_endian else '<f8')
    elif case == 'strided':
        quads = np.repeat(quads, 2, axis=0)[::2]
    elif case == 'integer_reliable':
        reliable = reliable.astype(np.int8)
    elif case == 'float_offsets':
        offsets = offsets.astype(np.float64)
    elif case == 'quad_shape':
        quads = quads[:, :3].copy()
    elif case == 'normal_shape':
        normals = np.zeros((len(normals), 3))
    elif case == 'reliable_shape':
        reliable = reliable[:, None]
    elif case == 'empty_offsets':
        offsets = np.empty(0, dtype=np.int64)
    elif case == 'offset_start':
        offsets[0] = 1
    elif case == 'offset_stop':
        offsets[-1] = 4
    elif case == 'offset_negative':
        offsets[1] = -1
    elif case == 'offset_decrease':
        offsets = np.array([0, 4, 3, 5], dtype=np.int64)
    elif case == 'nan_quad':
        quads[-1, 0, 0] = np.nan
    elif case == 'inf_normal':
        normals[-1, 0] = np.inf
    elif case == 'huge_normal':
        normals[-1] = [1.e308, 1.e308]
    elif case == 'zero_normal':
        normals[-1] = 0.
    elif case == 'nonunit_normal':
        normals[-1, 1] = 1.+2.e-6
    before = _exact((quads, normals, reliable, offsets))
    with np.errstate(over='raise', invalid='raise', divide='raise', under='ignore'):
        assert support._validate_continuation_batch(
            stamp, quads, normals, reliable, offsets) is None
    assert _exact((quads, normals, reliable, offsets)) == before


def test_no_conversion_hook_is_called_by_optional_preflight():
    class Trap:
        def __array__(self, *args, **kwargs):
            raise AssertionError('preflight must not convert external inputs')
    arrays = _packed()
    assert support._validate_continuation_batch(DT, Trap(), *arrays[1:]) is None


@pytest.mark.parametrize('mode', ['warn', 'raise', 'call', 'log', 'print'])
def test_strict_underflow_returns_none_before_norm(mode, monkeypatch):
    arrays = _packed()
    def forbidden(*args, **kwargs):
        raise AssertionError('strict error policy must retain original route')
    monkeypatch.setattr(np.linalg, 'norm', forbidden)
    with np.errstate(under=mode):
        assert support._validate_continuation_batch(DT, *arrays) is None


@pytest.mark.parametrize('kind', ['quad_capacity', 'owner_capacity'])
def test_batch_capacity_failures_do_not_construct_public_state(kind):
    if kind == 'quad_capacity':
        count = 200001
        arrays = (np.zeros((count, 4, 2)), np.zeros((count, 2)),
                  np.zeros(count, dtype=bool), np.array([0, count], dtype=np.int64))
    else:
        arrays = _packed((0,))[:3] + (np.zeros(200002, dtype=np.int64),)
    assert support._validate_continuation_batch(DT, *arrays) is None


@pytest.mark.parametrize('index', [-1, 3, True, np.int64(0), .0, None])
def test_factory_rejects_invalid_group_index(index):
    _, token = _batch()
    with pytest.raises(ValueError, match='private continuation frame batch/group'):
        support._continuation_frame_from_batch(token, index)


def test_factory_rejects_foreign_or_wrong_seal_tokens():
    _, token = _batch()
    # The marker is an internal ownership convention, not a security boundary
    # against code deliberately copying a legitimate token's private seal.
    for forged in (None, object(), SimpleNamespace(**vars(token)),
                   replace(token, seal=object())):
        with pytest.raises(ValueError, match='private continuation frame batch/group'):
            support._continuation_frame_from_batch(forged, 0)


def test_one_norm_reduction_per_batch_and_none_per_owner(monkeypatch):
    arrays = _packed((1, 0, 2, 1))
    norm = np.linalg.norm
    seen = []
    def record(value, *args, **kwargs):
        seen.append((value.shape, args, kwargs.copy()))
        return norm(value, *args, **kwargs)
    monkeypatch.setattr(np.linalg, 'norm', record)
    token = support._validate_continuation_batch(DT, *arrays)
    assert token is not None
    for group in range(4):
        support._continuation_frame_from_batch(token, group)
    assert seen == [((4, 2), (), {'axis': 1})]


def test_public_dataclass_schema_and_old_objects_keep_original_reduction(monkeypatch):
    arrays, token = _batch((2,))
    public = support.ContinuationFrame(DT, *arrays[:3])
    private = support._continuation_frame_from_batch(token, 0)
    assert [item.name for item in fields(public)] == ['stamp_ns', 'quads', 'normals', 'reliable']
    assert list(asdict(private)) == list(asdict(public))
    assert '_reliable_present' not in vars(public)
    assert _exact(private) == _exact(public)
    original = np.any
    calls = []
    def spy(value, *args, **kwargs):
        calls.append(value)
        return original(value, *args, **kwargs)
    monkeypatch.setattr(np, 'any', spy)
    assert support._has_reliable(private) is True
    assert not calls
    assert bool(support._has_reliable(public)) is True
    assert len(calls) == 1 and calls[0] is public.reliable


@pytest.mark.parametrize('minimum,reliable_minimum', [(1, 1), (3, 2), (5, 1)])
def test_retention_matches_independent_original_with_mixed_public_private_frames(
        minimum, reliable_minimum):
    history = ()
    for index in range(1, 15):
        arrays = list(_packed((0 if index % 4 == 0 else 2,)))
        arrays[2][...] = index in (1, 6, 7, 14)
        token = support._validate_continuation_batch(index*DT, *arrays)
        assert token is not None
        current = (support._continuation_frame_from_batch(token, 0) if index % 2
                   else support.ContinuationFrame(index*DT, *arrays[:3]))
        expected = _literal_retain(history, current, minimum, reliable_minimum)
        actual = support.retain_frames(history, current, minimum, reliable_minimum)
        assert [id(item) for item in actual] == [id(item) for item in expected]
        assert _exact(actual) == _exact(expected)
        assert len(actual) <= minimum+reliable_minimum
        history = actual


@pytest.mark.parametrize('stamp,minimum,reliable', [(DT, 5, 1), (DT-1, 5, 1),
                                                 (2*DT, 0, 1), (2*DT, 2, 3)])
def test_retention_public_exception_contract_unchanged(stamp, minimum, reliable):
    prior = _frame(DT)
    _, token = _batch((0,), stamp)
    current = support._continuation_frame_from_batch(token, 0)
    errors = []
    for routine in (_literal_retain, support.retain_frames):
        with pytest.raises(ValueError) as caught:
            routine((prior,), current, minimum, reliable)
        errors.append((type(caught.value), str(caught.value)))
    assert errors[0] == errors[1]


def _pair(history=None, config=None):
    pair = (_LiteralConsensus(config), paint.BoundaryConsensus(config))
    for instance in pair:
        instance._continuations = dict(history or {})
        instance._continuation_storage_count = sum(
            1+sum(1+len(frame.quads) for frame in frames)
            for hypotheses in instance._continuations.values()
            for frames in hypotheses.values())
    return pair


def _direct(pair, obs, updates, owners, expired=()):
    before = tuple(_state(item) for item in pair)
    inputs = _exact((obs, updates, owners))
    outputs = [item._prepare_continuations(obs, updates, owners, expired, COSINE)
               for item in pair]
    assert _exact(outputs[0]) == _exact(outputs[1])
    assert tuple(_state(item) for item in pair) == before
    assert _exact((obs, updates, owners)) == inputs
    return outputs[1]


@pytest.mark.parametrize('case', [
    'plain', 'open', 'raw_j', 'model_j', 'fragment_j', 'invalid_model',
    'unreliable_owner', 'unreliable_neighbor', 'opposite_neighbor',
    'unowned_neighbor', 'signed_zero', 'rigid', 'missing_footprints',
    'normal_below', 'normal_equal', 'normal_above',
])
def test_packed_selection_signature_order_and_all_owner_bytes_match_literal(case, monkeypatch):
    obs = _observation(11*DT, closed=case != 'open')
    owners = np.array([8, 4, 4, 19, 8], dtype=np.int64)
    updates = _updates([(19, [3]), (8, [4, 0]), (4, [1, 2])])
    if case in ('raw_j', 'model_j', 'fragment_j'):
        field = {'raw_j': 'sensitivity', 'model_j': 'position_sensitivity',
                 'fragment_j': 'position_segment_sensitivity'}[case]
        obs = replace(obs, **{field: np.array([.03, .1, np.nextafter(.1, np.inf),
                                               .5, np.nextafter(.5, np.inf)])})
    elif case == 'invalid_model':
        valid = np.array([True, True, False, True, True])
        obs = replace(obs, position_valid=valid, position_reliable=valid,
                      position_segment_valid=valid, position_segment_reliable=valid)
    elif case in ('unreliable_owner', 'unreliable_neighbor'):
        reliable = np.ones(5, dtype=bool)
        reliable[3 if case == 'unreliable_owner' else 2] = False
        obs = replace(obs, position_reliable=reliable,
                      position_segment_reliable=reliable)
    elif case == 'opposite_neighbor':
        normals = obs.position_normals.copy()
        normals[2] = [0., -1.]
        obs = replace(obs, position_normals=normals)
    elif case == 'unowned_neighbor':
        owners[2] = -1
    elif case == 'signed_zero':
        normals = obs.position_normals.copy()
        normals[:, 0] = -0.
        quads = obs.position_footprints.copy()
        quads[1, 0, 1] = -0.
        obs = replace(obs, position_normals=normals, position_footprints=quads)
    elif case == 'rigid':
        obs = obs.transformed([1., -2., .31])
    elif case == 'missing_footprints':
        obs = replace(obs, position_footprints=None)
    elif case.startswith('normal_'):
        value = {'normal_below': np.nextafter(COSINE, -np.inf),
                 'normal_equal': COSINE,
                 'normal_above': np.nextafter(COSINE, np.inf)}[case]
        normals = obs.position_normals.copy()
        normals[2] = [math.sqrt(1.-value*value), value]
        obs = replace(obs, position_normals=normals)
    observed = []
    original = paint._validate_continuation_batch
    def spy(*args):
        token = original(*args)
        observed.append(token)
        return token
    monkeypatch.setattr(paint, '_validate_continuation_batch', spy)
    history = {77: MappingProxyType({(1, 2): (_frame(DT),)})}
    result, count = _direct(_pair(history), obs, updates, owners)
    assert [identity for identity, _ in result] == [19, 8, 4]
    if case == 'missing_footprints':
        assert not observed
    else:
        assert len(observed) == 1 and observed[0] is not None
        batch = observed[0]
        expected_quads, expected_normals, expected_reliable, lengths = [], [], [], []
        for _, hypotheses in result:
            frame = next(items[-1] for items in hypotheses.values()
                         if items[-1].stamp_ns == 11*DT)
            expected_quads.append(frame.quads)
            expected_normals.append(frame.normals)
            expected_reliable.append(frame.reliable)
            lengths.append(len(frame.quads))
        assert _exact(batch.quads) == _exact(np.concatenate(expected_quads))
        assert _exact(batch.normals) == _exact(np.concatenate(expected_normals))
        assert _exact(batch.reliable) == _exact(np.concatenate(expected_reliable))
        assert _exact(batch.offsets) == _exact(np.r_[0, np.cumsum(lengths, dtype=np.int64)])
    assert count == 3+sum(1+sum(frame.storage_units for frame in frames)
                         for _, hypotheses in result for frames in hypotheses.values())


@pytest.mark.parametrize('closed', [False, True])
def test_actual_prepare_batch_has_one_norm_call_for_many_owners(closed, monkeypatch):
    obs = _observation(2*DT, closed=closed)
    pair = _pair()
    updates = _updates([(11, [0]), (12, [1]), (13, [2]), (14, [3]), (15, [4])])
    expected = pair[0]._prepare_continuations(obs, updates, _owners(5), (), COSINE)
    norm = np.linalg.norm
    calls = []
    def spy(value, *args, **kwargs):
        calls.append((value.shape, args, kwargs))
        return norm(value, *args, **kwargs)
    monkeypatch.setattr(np.linalg, 'norm', spy)
    result = pair[1]._prepare_continuations(obs, updates, _owners(5), (), COSINE)
    assert _exact(result) == _exact(expected)
    assert calls == [(((5 if closed else 3), 2), (), {'axis': 1})]


@pytest.mark.parametrize('bad_neighbor', [None, 0, 3])
def test_endpoint_dot_inputs_order_and_short_circuit_remain_literal(bad_neighbor, monkeypatch):
    obs = _observation(2*DT, closed=True)
    if bad_neighbor is not None:
        normals = obs.position_normals.copy()
        normals[bad_neighbor] = [0., -1.]
        obs = replace(obs, position_normals=normals)
    pair = _pair()
    updates = _updates([(12, [1, 2]), (13, [3, 4])])
    original = np.dot
    trace = []
    def spy(first, second, *args, **kwargs):
        answer = original(first, second, *args, **kwargs)
        trace.append((_exact(first), _exact(second), _exact(answer)))
        return answer
    monkeypatch.setattr(np, 'dot', spy)
    expected = pair[0]._prepare_continuations(obs, updates, _owners(5), (), COSINE)
    old_trace = tuple(trace)
    trace.clear()
    result = pair[1]._prepare_continuations(obs, updates, _owners(5), (), COSINE)
    assert tuple(trace) == old_trace
    assert _exact(result) == _exact(expected)


@pytest.mark.parametrize('mode', ['warn', 'raise'])
def test_actual_prepare_strict_underflow_keeps_old_owner_constructor(mode, monkeypatch):
    obs = _observation(2*DT)
    pair = _pair()
    updates = _updates([(12, [1]), (13, [2]), (14, [3])])
    def forbidden(*args, **kwargs):
        raise AssertionError('strict mode must not invoke batch preflight')
    monkeypatch.setattr(paint, '_validate_continuation_batch', forbidden)
    with np.errstate(under=mode):
        _direct(pair, obs, updates, _owners(5))


def test_subclass_observation_preserves_original_fallback(monkeypatch):
    class DerivedObservation(paint.BoundaryObservation):
        pass
    base = _observation(2*DT)
    obs = DerivedObservation(**{field.name: getattr(base, field.name) for field in fields(base)})
    def forbidden(*args, **kwargs):
        raise AssertionError('subclass must not enter private validated route')
    monkeypatch.setattr(paint, '_validate_continuation_batch', forbidden)
    _direct(_pair(), obs, _updates([(12, [1, 2, 3])]), _owners(5))


@pytest.mark.parametrize('late_geometry_first', [False, True])
def test_ineligible_late_owner_preserves_original_first_exception(late_geometry_first):
    obs = _observation(DT)
    damaged = obs.position_footprints.copy()
    damaged[3, 0, 0] = np.nan
    # Deliberate invariant-corruption injection tests the documented fallback,
    # not a claim that public BoundaryObservation accepts this geometry.
    object.__setattr__(obs, 'position_footprints',
                       np.frombuffer(damaged.tobytes(), dtype=np.float64).reshape(damaged.shape))
    history = {12: MappingProxyType({(100, 102): (_frame(DT),)})}
    pair = _pair(history)
    groups = [(12, [1]), (13, [3])]
    if late_geometry_first:
        groups.reverse()
    before = tuple(_state(item) for item in pair)
    failures = []
    for item in pair:
        with pytest.raises(ValueError) as caught:
            item._prepare_continuations(obs, _updates(groups), _owners(5), (), COSINE)
        failures.append((type(caught.value), str(caught.value)))
    expected = ('invalid continuation geometry' if late_geometry_first
                else 'invalid continuation witness order/count')
    assert failures == [(ValueError, expected)]*2
    assert tuple(_state(item) for item in pair) == before


@pytest.mark.parametrize('closed', [False, True])
def test_empty_missing_and_legacy_frames_keep_all_metadata(closed):
    assert _direct(_pair(), _observation(xs=()), (), _owners(0)) == ((), 0)
    assert _direct(_pair(), _observation(model=False),
                   _updates([(1, [1])]), _owners(5)) == ((), 0)
    history = {12: MappingProxyType({(101, 103): (_frame(DT),)})}
    obs = _observation(2*DT, closed=closed, footprints=False)
    result, count = _direct(_pair(history), obs, _updates([(12, [2])]), _owners(5))
    frames = result[0][1][(101, 103)]
    assert len(frames[-1].quads) == 0 and frames[-1].storage_units == 1
    assert count == 1+sum(frame.storage_units for frame in frames)


def _prepare_pair(pair, obs):
    before = tuple(_state(item) for item in pair)
    tokens, errors = [], []
    for item in pair:
        try:
            tokens.append(item.prepare(obs))
            errors.append(None)
        except ValueError as error:
            tokens.append(None)
            errors.append((type(error), str(error)))
    assert errors[0] == errors[1]
    assert tuple(_state(item) for item in pair) == before
    assert _exact(tokens[0]) == _exact(tokens[1])
    return tokens, errors[0]


def _commit_pair(pair, tokens):
    assert all(item.commit(token) for item, token in zip(pair, tokens))
    assert _state(pair[0]) == _state(pair[1])
    assert _exact(pair[0].snapshot()) == _exact(pair[1].snapshot())
    assert _exact(pair[0].snapshot_near([0., .01], .3, 100)) == _exact(
        pair[1].snapshot_near([0., .01], .3, 100))
    for item in pair:
        recount = sum(1+sum(1+len(frame.quads) for frame in frames)
                      for hypotheses in item._continuations.values()
                      for frames in hypotheses.values())
        assert item._continuation_storage_count == recount


@pytest.mark.parametrize('force_fallback', [False, True])
def test_public_multiframe_prepare_commit_and_retention_exact(force_fallback, monkeypatch):
    if force_fallback:
        monkeypatch.setattr(paint, '_validate_continuation_batch', lambda *args: None)
    pair = _pair()
    for index in range(1, 13):
        xs = (-.2, 0., .4) if index <= 5 else (-.2, 0., .2, .4)
        obs = _observation(index*DT, xs=xs, trusted=index in (1, 6, 12))
        tokens, error = _prepare_pair(pair, obs)
        assert error is None
        _commit_pair(pair, tokens)
    assert pair[1]._generation == 12
    assert pair[1].source_frontier_ns == 12*DT
    assert all(len(frames) <= 6 for histories in pair[1]._continuations.values()
               for frames in histories.values())


@pytest.mark.parametrize('confirmed', [False, True])
def test_public_ttl_frontier_and_empty_updates_exact(confirmed):
    pair = _pair(config={'candidate_ttl_s': .15})
    for index in range(1, 6 if confirmed else 2):
        tokens, error = _prepare_pair(pair, _observation(index*DT))
        assert error is None
        _commit_pair(pair, tokens)
    original = set(pair[1]._records)
    tokens, error = _prepare_pair(pair, _observation(10*DT, xs=()))
    assert error is None
    assert set(tokens[1].expired_ids) == (set() if confirmed else original)
    _commit_pair(pair, tokens)
    assert bool(pair[1]._continuations) is confirmed


@pytest.mark.parametrize('capacity', [1, 14, 59, 60, 200000])
def test_public_capacity_rejection_is_same_and_never_partial(capacity):
    pair = _pair(config={'max_support_segments': capacity})
    for index in range(1, 9):
        tokens, error = _prepare_pair(pair, _observation(index*DT))
        if error is not None:
            assert 'capacity' in error[1]
            return
        _commit_pair(pair, tokens)
    assert pair[1]._generation == 8


def test_prepared_alternative_foreign_and_stale_tokens_do_not_mutate():
    pair = _pair()
    for index in range(1, 6):
        tokens, error = _prepare_pair(pair, _observation(index*DT))
        assert error is None
        _commit_pair(pair, tokens)
    stale, error = _prepare_pair(pair, _observation(6*DT, footprints=False))
    assert error is None
    chosen, error = _prepare_pair(pair, _observation(6*DT))
    assert error is None
    before = tuple(_state(item) for item in pair)
    assert not pair[1].commit(chosen[0])
    assert tuple(_state(item) for item in pair) == before
    _commit_pair(pair, chosen)
    after = tuple(_state(item) for item in pair)
    assert not pair[0].commit(stale[0]) and not pair[1].commit(stale[1])
    assert tuple(_state(item) for item in pair) == after
    tokens, error = _prepare_pair(pair, _observation(6*DT))
    assert tokens == [None, None] and 'not increasing' in error[1]
