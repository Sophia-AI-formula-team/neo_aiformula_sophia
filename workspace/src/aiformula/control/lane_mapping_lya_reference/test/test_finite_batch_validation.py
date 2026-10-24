"""Private frame validation vs unchanged public per-owner advance contracts.

No map/estimator, recording, relaxed timing or native installation is used.
The original public path is the arithmetic/error-order oracle. These tests
also exercise malformed batches falling back without choosing a later error.
"""
from dataclasses import FrozenInstanceError, replace
import math
from pathlib import Path
import struct
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from lane_mapping_lya_reference import finite_support as finite


def arrays(intervals, angle=0., origin=(0., 0.), reliable=True):
    tangent = np.array([math.cos(angle), math.sin(angle)])
    normal = np.array([-tangent[1], tangent[0]])
    intervals = np.asarray(intervals, dtype=np.float64).reshape(-1, 2)
    points = np.asarray(origin) + intervals[:, :, None]*tangent
    normals = np.broadcast_to(normal, points.shape).copy()
    flags = np.broadcast_to(np.asarray(reliable, dtype=bool), (len(points),)).copy()
    return points, normals, flags


def options(stamp=1, angle=0., origin=(0., 0.), **changes):
    result = dict(origin=origin, current_frame_normal=(-math.sin(angle), math.cos(angle)),
                  stamp_ns=stamp, minimum_frame_votes=5, minimum_reliable_votes=1,
                  candidate_ttl_ns=0, normal_cosine=math.cos(math.pi/6.), max_intervals=100)
    result.update(changes)
    return result


def pack(groups):
    offsets = np.r_[0, np.cumsum([len(group[0]) for group in groups], dtype=np.int64)]
    values = tuple(np.concatenate([group[column] for group in groups], axis=0)
                   for column in range(3))
    batch = finite._validate_frame_batch(*values, offsets)
    assert batch is not None
    return values, offsets, batch


def public(old, group, **kwargs):
    return finite.advance_domain(old, segments=group[0], normals=group[1],
                                 reliable=group[2], **kwargs)


def private(old, batch, index, **kwargs):
    return finite._advance_domain_validated(old, batch=batch, group_index=index, **kwargs)


def state_bytes(value):
    if value is None:
        return b'none'
    result = struct.pack('=ddddq', *(value.origin+value.tangent), value.source_stamp_ns)
    for item in value.intervals:
        result += struct.pack('=ddqqq', item.lower, item.upper, item.frame_votes,
                              item.reliable_votes, item.last_stamp_ns)
    return result


def outcome(call):
    try:
        return ('state', state_bytes(call()))
    except (ValueError, TypeError, OverflowError, FloatingPointError) as error:
        return ('error', type(error), error.args)


def export_outcome(state, angle=0., origin=(0., 0.)):
    try:
        result = finite.certified_segments(state, origin,
            (-math.sin(angle), math.cos(angle)), math.cos(math.pi/6.), 5, 1)
        return tuple((value.shape, value.dtype.str, value.tobytes()) for value in result)
    except ValueError as error:
        return ('error', error.args)


@pytest.mark.parametrize('angle', [0., .3, -1.1, math.pi/2., math.pi])
def test_multiple_owners_six_causal_frames_match_public_state_and_export_bits(angle):
    groups = [arrays([(0., 1.), (.5, 2.), (0., 1.)], angle, reliable=[False, True, True]),
              arrays([], angle),
              arrays([(-0., .2), (.2, 1.), (2., 3.)], angle, (12.3, -4.2),
                     reliable=[True, False, True])]
    values, offsets, batch = pack(groups)
    initial = tuple(value.tobytes() for value in values)+(offsets.tobytes(),)
    old_public, old_private = [None]*3, [None]*3
    for stamp in range(1, 7):
        for index, group in enumerate(groups):
            origin = (12.3, -4.2) if index == 2 else (0., 0.)
            kwargs = options(stamp, angle, origin)
            old_public[index] = public(old_public[index], group, **kwargs)
            old_private[index] = private(old_private[index], batch, index, **kwargs)
            assert state_bytes(old_private[index]) == state_bytes(old_public[index])
            assert export_outcome(old_private[index], angle, origin) == export_outcome(
                old_public[index], angle, origin)
    assert initial == tuple(value.tobytes() for value in values)+(offsets.tobytes(),)


def test_shared_endpoint_reliability_and_duplicate_segments_not_reinterpreted():
    points = np.array([[[0., 0.], [1., 0.]], [[1., 0.], [1., 1.]],
                       [[1., 1.], [0., 1.]], [[0., 0.], [1., 0.]]])
    normals = np.tile([0., 1.], (4, 2, 1))
    group = (points, normals, np.array([True, False, True, True]))
    _, _, batch = pack([group])
    expected = public(None, group, **options())
    actual = private(None, batch, 0, **options())
    assert state_bytes(actual) == state_bytes(expected)
    assert max(item.frame_votes for item in actual.intervals) == 1


@pytest.mark.parametrize('stamp', [2, 3, 4])
def test_ttl_equality_and_expiry_keep_identical_pending_and_confirmed_state(stamp):
    old = public(None, arrays([(0., 1.)], reliable=False), **options())
    new = arrays([(2., 3.)])
    _, _, batch = pack([new])
    kwargs = options(stamp, candidate_ttl_ns=2)
    assert outcome(lambda: private(old, batch, 0, **kwargs)) == outcome(
        lambda: public(old, new, **kwargs))


def test_batch_does_not_alias_callers_and_buffers_cannot_be_made_writeable():
    group = arrays([(0., 1.)])
    values, offsets, batch = pack([group])
    before = state_bytes(private(None, batch, 0, **options()))
    for value in (batch.segments, batch.normals, batch.reliable, batch.offsets):
        assert not value.flags.writeable
        with pytest.raises(ValueError):
            value.setflags(write=True)
    for value in values:
        value[...] = 0
    offsets[...] = 0
    assert state_bytes(private(None, batch, 0, **options())) == before
    with pytest.raises(FrozenInstanceError):
        batch.offsets = offsets


def test_private_skips_array_validator_but_public_still_calls_it(monkeypatch):
    group = arrays([(0., 1.)])
    _, _, batch = pack([group])
    expected = public(None, group, **options())
    calls = []
    original = finite._arrays

    def record(*args):
        calls.append(1)
        return original(*args)

    monkeypatch.setattr(finite, '_arrays', record)
    assert state_bytes(private(None, batch, 0, **options())) == state_bytes(expected)
    assert calls == []
    public(None, group, **options())
    assert calls == [1]


@pytest.mark.parametrize('change', [
    {'origin': (np.nan, 0.)}, {'current_frame_normal': (0., .5)}, {'stamp_ns': True},
    {'minimum_frame_votes': 0}, {'minimum_reliable_votes': 6}, {'candidate_ttl_ns': -1},
    {'normal_cosine': 0.}, {'max_intervals': 0},
])
def test_all_scalar_errors_keep_type_message_and_precede_invalid_previous(change):
    group = arrays([(0., 1.)])
    _, _, batch = pack([group])
    kwargs = options(**change)
    assert outcome(lambda: private(object(), batch, 0, **kwargs)) == outcome(
        lambda: public(object(), group, **kwargs))


@pytest.mark.parametrize('case', ['type', 'duplicate', 'older', 'capacity', 'saturation'])
def test_previous_checks_and_failed_advances_remain_identical_and_atomic(case):
    group = arrays([(0., 1.), (2., 3.)])
    _, _, batch = pack([group])
    old = public(None, group, **options())
    kwargs = options(2)
    if case == 'type':
        old = object()
    elif case == 'duplicate':
        kwargs['stamp_ns'] = 1
    elif case == 'older':
        old = public(old, group, **options(3))
    elif case == 'capacity':
        kwargs['max_intervals'] = 1
    else:
        old = public(old, group, **options(2))
        kwargs.update(stamp_ns=3, minimum_frame_votes=1)
    before = state_bytes(old) if type(old) is finite.DomainState else None
    expected = outcome(lambda: public(old, group, **kwargs))
    actual = outcome(lambda: private(old, batch, 0, **kwargs))
    assert expected[0] == 'error' and actual == expected
    if before is not None:
        assert state_bytes(old) == before


def test_invalid_later_owner_falls_back_without_masking_earlier_scalar_error():
    first, second = arrays([(0., 1.)]), arrays([(2., 3.)])
    second[1][0, 0] = (0., .5)
    values = tuple(np.concatenate((first[i], second[i])) for i in range(3))
    assert finite._validate_frame_batch(*values, np.array([0, 1, 2], dtype=np.int64)) is None
    # This is the required caller fallback, not a batch-wide exception.
    with pytest.raises(ValueError, match='origin must be finite'):
        for index, group in enumerate((first, second)):
            public(None, group, **options(origin=(np.nan, 0.) if index == 0 else (0., 0.)))


def test_public_arrays_still_precede_previous_state_error():
    group = arrays([(0., 1.)])
    group[1][0, 0] = (0., .5)
    with pytest.raises(ValueError, match='endpoint normals must already be finite unit vectors'):
        public(object(), group, **options())


@pytest.mark.parametrize('kind', ['float32', 'integer', 'big_endian', 'strided', 'subclass',
                                  'list', 'wrong_shape', 'reliable_integer', 'offset_float',
                                  'offset_descending', 'offset_negative', 'offset_end',
                                  'offset_overflow', 'nonfinite_point', 'point_bound',
                                  'nonfinite_normal', 'normal_bound', 'normal_length'])
def test_ineligible_or_invalid_batch_returns_none_without_conversion_or_exception(kind):
    points, normals, reliable = arrays([(0., 1.), (2., 3.)])
    offsets = np.array([0, 1, 2], dtype=np.int64)
    if kind == 'float32': points = points.astype(np.float32)
    elif kind == 'integer': points = points.astype(np.int64)
    elif kind == 'big_endian': points = points.astype('>f8')
    elif kind == 'strided': points = np.repeat(points, 2, axis=0)[::2]
    elif kind == 'subclass':
        class Forbidden(np.ndarray):
            def __array__(self, *args, **kwargs):
                raise AssertionError('must not convert a subclass')
        points = points.view(Forbidden)
    elif kind == 'list': points = points.tolist()
    elif kind == 'wrong_shape': points = points.reshape(4, 2)
    elif kind == 'reliable_integer': reliable = reliable.astype(np.int64)
    elif kind == 'offset_float': offsets = offsets.astype(np.float64)
    elif kind == 'offset_descending': offsets = np.array([0, 2, 1, 2], dtype=np.int64)
    elif kind == 'offset_negative': offsets[1] = -1
    elif kind == 'offset_end': offsets[-1] = 1
    elif kind == 'offset_overflow': offsets[1] = np.iinfo(np.int64).max
    elif kind == 'nonfinite_point': points[0, 0, 0] = np.nan
    elif kind == 'point_bound': points[0, 0, 0] = np.nextafter(finite.MAX_COORDINATE, np.inf)
    elif kind == 'nonfinite_normal': normals[0, 0, 0] = np.inf
    elif kind == 'normal_bound': normals[0, 0, 1] = 1.+2.*finite.UNIT_TOLERANCE
    elif kind == 'normal_length': normals[0, 0] = (.5, .5)
    with np.errstate(over='raise', invalid='raise', divide='raise', under='ignore'):
        assert finite._validate_frame_batch(points, normals, reliable, offsets) is None


@pytest.mark.parametrize('mode', ['warn', 'raise', 'call', 'print', 'log'])
def test_nonignore_underflow_retains_original_path_without_evaluating_norm(monkeypatch, mode):
    group = arrays([(0., 1.)])

    def forbidden(*args, **kwargs):
        raise AssertionError('no norm work in an ineligible error mode')

    monkeypatch.setattr(finite.np.linalg, 'norm', forbidden)
    with np.errstate(under=mode):
        assert finite._validate_frame_batch(*group, np.array([0, 1], dtype=np.int64)) is None


def test_strict_norm_tolerance_not_observation_one_part_per_million():
    group = arrays([(0., 1.)])
    group[1][0, 0, 1] = 1.+1.e-8
    assert finite._validate_frame_batch(*group, np.array([0, 1], dtype=np.int64)) is None
    with pytest.raises(ValueError, match='endpoint normals'):
        public(None, group, **options())


def test_group_and_frame_capacity_are_distinct_without_truncation(monkeypatch):
    monkeypatch.setattr(finite, 'MAX_ITEMS', 3)
    group = arrays([(0., 1.)]*6)
    batch = finite._validate_frame_batch(*group, np.array([0, 3, 6], dtype=np.int64))
    assert batch is not None and len(batch.segments) == 6
    assert finite._validate_frame_batch(*group, np.array([0, 4, 6], dtype=np.int64)) is None
    larger = arrays([(0., 1.)]*7)
    assert finite._validate_frame_batch(*larger, np.array([0, 3, 5, 7], dtype=np.int64)) is None
    assert finite._validate_frame_batch(*group, np.array([0, 1, 2, 3, 6], dtype=np.int64)) is None


def test_no_groups_and_multiple_empty_groups_and_invalid_private_tokens():
    group = arrays([])
    none = finite._validate_frame_batch(*group, np.array([0], dtype=np.int64))
    assert none is not None
    with pytest.raises(ValueError, match='group_index'):
        private(None, none, 0, **options())
    batch = finite._validate_frame_batch(*group, np.array([0, 0, 0], dtype=np.int64))
    for index in range(2):
        assert state_bytes(private(None, batch, index, **options())) == state_bytes(
            public(None, group, **options()))
    for invalid in (None, object(), replace(batch, seal=None)):
        with pytest.raises(ValueError, match='private validated frame batch'):
            private(None, invalid, 0, **options())
    for index in (-1, 2, True, .5):
        with pytest.raises(ValueError, match='group_index'):
            private(None, batch, index, **options())


def test_signed_zero_subnormal_and_fixed_random_norm_validation_match_public():
    rng = np.random.RandomState(120713)
    angles = rng.uniform(-math.pi, math.pi, 257)
    points = rng.uniform(-10., 10., (len(angles), 2, 2))
    normals = np.repeat(np.column_stack((-np.sin(angles), np.cos(angles)))[:, None, :], 2, axis=1)
    normals[0] = [[-0., 1.], [np.nextafter(0., 1.), 1.]]
    points[0] = [[-0., +0.], [1., -0.]]
    reliable = rng.randint(0, 2, len(points)).astype(bool)
    expected = finite._arrays(points, normals, reliable)
    batch = finite._validate_frame_batch(points, normals, reliable,
        np.array([0, 1, 32, 257], dtype=np.int64))
    assert batch is not None
    for actual, original in zip((batch.segments, batch.normals, batch.reliable), expected):
        assert actual.shape == original.shape and actual.dtype == original.dtype
        assert actual.tobytes() == original.tobytes()


def consensus_fingerprint(consensus):
    from dataclasses import fields
    reference = consensus.snapshot()
    arrays = tuple((item.name, value.shape, value.dtype.str, value.tobytes())
        for item in fields(reference)
        for value in (getattr(reference, item.name),) if isinstance(value, np.ndarray))
    assert len(arrays) == 18
    records = tuple((key, item.cell, struct.pack('=dddd', *(item.point+item.normal)),
        item.votes, item.reliable_votes, item.last_stamp_ns,
        item.segments.tobytes(), item.neighbours.tobytes())
        for key, item in consensus._records.items())
    return (arrays, records, tuple(consensus._edges.items()),
            tuple((key, state_bytes(value)) for key, value in consensus._domains.items()),
            tuple((key, tuple(value)) for key, value in consensus._cell_records.items()),
            tuple(consensus._domain_expiry.items()), consensus._generation,
            consensus.source_frontier_ns, consensus._next_record_id,
            consensus._domain_interval_count, consensus._segment_count,
            consensus._confirmed_count)


@pytest.mark.parametrize('capacity,invalid_normal', [(100, False), (4, False), (100, True)])
def test_real_consensus_batch_vs_forced_public_path_all_arrays_transactions_and_errors(
        monkeypatch, capacity, invalid_normal):
    from lane_mapping_lya_reference import paint_geometry as paint
    # Reuse only this existing analytical observation builder, not its tests.
    from test_boundary_domain_consensus import observation

    fast = paint.BoundaryConsensus({'max_support_segments': capacity, 'candidate_ttl_s': .15})
    slow = paint.BoundaryConsensus({'max_support_segments': capacity, 'candidate_ttl_s': .15})
    validator = paint._validate_frame_batch
    real_calls, tokens, failures = [], [], []

    def counted(*args):
        result = validator(*args)
        real_calls.append(1)
        tokens.append(result)
        return result

    for frame in range(1, 9):
        points = np.array([[-.125, .01], [-.09375, .01], [-.0625, .01],
                           [.03125, .01], [.0625, .01], [.09375, .01], [.125, .01]])
        points[:, 0] += .001*(frame % 2)
        normals = np.tile([0., 1.], (len(points), 1))
        if invalid_normal:
            normals[-1, 1] += 1.e-8  # Observation accepts; finite array contract rejects.
        obs = observation(frame*100_000_000, points, reliable=(frame % 3 == 0), normals=normals)
        before = consensus_fingerprint(fast)
        assert before == consensus_fingerprint(slow)
        prepared, errors = [], []
        for consensus, validate in ((fast, counted), (slow, lambda *args: None)):
            monkeypatch.setattr(paint, '_validate_frame_batch', validate)
            try:
                prepared.append(consensus.prepare(obs))
                errors.append(None)
            except ValueError as error:
                prepared.append(None)
                errors.append((type(error), error.args))
        assert errors[0] == errors[1]
        assert consensus_fingerprint(fast) == before == consensus_fingerprint(slow)
        if errors[0] is not None:
            failures.append(errors[0])
            continue
        assert dict(prepared[0].stats) == dict(prepared[1].stats)
        assert fast.commit(prepared[0]) and slow.commit(prepared[1])
        assert consensus_fingerprint(fast) == consensus_fingerprint(slow)
        after = consensus_fingerprint(fast)
        assert not fast.commit(prepared[0]) and not slow.commit(prepared[1])
        assert consensus_fingerprint(fast) == after == consensus_fingerprint(slow)
        for consensus in (fast, slow):
            with pytest.raises(ValueError, match='boundary source stamp not increasing'):
                consensus.prepare(obs)
    if capacity == 100 and not invalid_normal:
        assert real_calls and all(token is not None for token in tokens)
        assert not failures and fast.source_frontier_ns == 800_000_000
    else:
        assert failures and fast.source_frontier_ns is None
    if invalid_normal:
        assert real_calls and all(token is None for token in tokens)
