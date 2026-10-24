"""R25 exact continuation-cache regression; no runtime/geometry pass claim.

_LiteralConsensus embeds the two unchanged methods from paint_geometry.py at
764d860e64c5fe726fa03346f2142571fe1b2a99. No Git, diagnostic paths or external
fixtures are required in CI. The unchanged ContinuationFrame/retain_frames
contracts are shared; this oracle isolates only the modified preparation body.
Public transaction tests compare all token/state/reference values except the
intentional different token-owner identities. Float arrays use exact bytes.
"""
from collections.abc import Mapping
from dataclasses import fields, is_dataclass, replace
import math
from types import MappingProxyType

import numpy as np
import pytest

from lane_mapping_lya_reference.paint_geometry import BoundaryConsensus, BoundaryObservation
from lane_mapping_lya_reference.continuation_support import ContinuationFrame, retain_frames

DT = 100_000_000
COSINE = math.cos(math.pi / 6.)


class _LiteralConsensus(BoundaryConsensus):
    @staticmethod
    def _continuation_size(hypotheses):
        return sum(1 + sum(frame.storage_units for frame in frames) for frames in hypotheses.values())

    def _prepare_continuations(self, obs, updates, face_owner, expired, cosine):
        """Immutable per-frame identity alternatives, including missing data.

        One final record owns one actual contiguous run in this frame. Its two
        immediate observed neighbours define the hypothesis; there is no graph
        traversal, synthetic subdivision, gap bridging, or edge-vote copying.
        Only the owner's own qualified pixel quads supply geometric evidence.
        """
        # Legacy/manual observations contain no pixel-area evidence. They
        # retain the old branch rejection and need no empty sidecar inventory.
        if obs.position_footprints is None and not self._continuations:
            return (), 0
        count = self._continuation_storage_count - sum(
            self._continuation_size(self._continuations.get(key, {})) for key in expired)
        result, cfg = [], self.config
        for record_id, _, _, _, _, _, faces in updates:
            previous, following = int(obs.previous[faces[0]]), int(obs.next[faces[-1]])
            signature = (int(face_owner[previous]) if previous >= 0 else -1,
                         int(face_owner[following]) if following >= 0 else -1)
            old = self._continuations.get(record_id, {})
            hypotheses = dict(old)
            selected = np.empty(0, dtype=np.int64)
            if obs.position_footprints is not None:
                admitted = (obs.model_valid[faces] & obs.position_segment_valid[faces]
                    & (obs.model_sensitivity[faces] <= cfg['max_sensitivity'])
                    & (obs.position_segment_sensitivity[faces] <= cfg['max_sensitivity']))
                selected = faces[admitted]
            quads = (np.empty((0, 4, 2)) if not len(selected)
                     else obs.position_footprints[selected])
            trusted = np.zeros(len(selected), dtype=bool)
            if len(selected):
                trusted = (obs.model_reliable[selected] & obs.position_segment_reliable[selected]
                    & (obs.model_sensitivity[selected] <= cfg['reliable_sensitivity'])
                    & (obs.position_segment_sensitivity[selected] <= cfg['reliable_sensitivity']))
            # Neighbour validity/normal agreement is part of the observed run
            # identity, not just a direction at the public mean. A broken end
            # remains inventoried but contributes no permission this frame.
            endpoints = (int(faces[0]), int(faces[-1]))
            if min(signature) < 0 or any(float(np.dot(obs.model_normals[own], obs.model_normals[other])) < cosine
                    for own, other in zip(endpoints, (previous, following)) if other >= 0):
                selected, quads, trusted = np.empty(0, dtype=np.int64), np.empty((0, 4, 2)), np.empty(0, dtype=bool)
            elif not np.all(obs.model_reliable[[*endpoints, previous, following]] &
                            (obs.model_sensitivity[[*endpoints, previous, following]] <= cfg['reliable_sensitivity'])):
                trusted[:] = False
            frame = ContinuationFrame(obs.source_stamp_ns, quads, obs.model_normals[selected], trusted)
            hypotheses[signature] = retain_frames(old.get(signature, ()), frame,
                cfg['minimum_frame_votes'], cfg['minimum_reliable_votes'])
            count += self._continuation_size(hypotheses)-self._continuation_size(old)
            result.append((record_id, MappingProxyType(hypotheses)))
        return tuple(result), count
def _freeze(value):
    if isinstance(value, np.ndarray):
        return (value.dtype.str, value.shape, value.tobytes())
    if is_dataclass(value):
        return tuple((item.name, _freeze(getattr(value, item.name)))
                     for item in fields(value) if item.name != 'owner')
    if isinstance(value, Mapping):
        # Preserve insertion order: association/hypothesis ordering is evidence.
        return tuple((_freeze(key), _freeze(item)) for key, item in value.items())
    if isinstance(value, (tuple, list)):
        return tuple(_freeze(item) for item in value)
    return value


def _state(consensus):
    return _freeze(vars(consensus))


def _observation(stamp=DT, *, xs=(-.2, 0., .2, .4, .6), closed=False,
                 footprints=True, model=True, trusted=True):
    """Explicit analytical faces; no camera/estimator or truth input."""
    xs = np.asarray(xs, dtype=np.float64)
    count = len(xs)
    points = np.column_stack((xs, np.full(count, .01)))
    normal = np.tile([0., 1.], (count, 1))
    segments = np.stack((points-[.03, 0.], points+[.03, 0.]), axis=1)
    previous = np.arange(-1, count-1, dtype=np.int64)
    following = np.arange(1, count+1, dtype=np.int64)
    if count:
        following[-1] = -1
        if closed:
            previous[0], following[-1] = count-1, 0
    yes = np.ones(count, dtype=bool)
    pixels = np.column_stack((np.arange(count, dtype=np.int64),
                              np.zeros(count, dtype=np.int64)))
    values = dict(
        points=points, normals=normal, segments=segments,
        sensitivity=np.full(count, .02), metric_valid=yes, stable=yes,
        normal_valid=yes, valid=yes, reliable=yes,
        source_stamp_ns=stamp, pixel_segments=segments, white_pixels=pixels,
        black_pixels=pixels+[0, 1], previous=previous, next=following,
        contour_ids=np.zeros(count, dtype=np.int64),
        reason_codes=np.zeros(count, dtype=np.int64),
        normal_support_count=np.full(count, 6, dtype=np.int64),
        normal_radius=np.full(count, .45))
    if model:
        trusted_flags = np.full(count, trusted, dtype=bool)
        values.update(
            position_estimates=points.copy(), position_valid=yes,
            position_sensitivity=np.full(count, .03),
            position_reliable=trusted_flags, position_normals=normal.copy(),
            position_segments=segments.copy(), position_segment_valid=yes,
            position_segment_sensitivity=np.full(count, .04),
            position_segment_reliable=trusted_flags)
        if footprints:
            values['position_footprints'] = np.stack((
                points+[-.03, -.004], points+[.03, -.004],
                points+[.03, .004], points+[-.03, .004]), axis=1)
    return BoundaryObservation(**values)


def _updates(groups):
    # Only record id and ordered faces are consumed by the selected method.
    return tuple((identity, None, None, None, None, None,
                  np.asarray(members, dtype=np.int64)) for identity, members in groups)


def _owners(count):
    return np.arange(100, 100+count, dtype=np.int64)


def _frame(stamp, count=1, reliable=True, offset=0.):
    points = np.column_stack((np.arange(count)*.02+offset, np.zeros(count)))
    quads = np.stack((points+[-.01, -.01], points+[.01, -.01],
                      points+[.01, .01], points+[-.01, .01]), axis=1)
    return ContinuationFrame(stamp, quads, np.tile([0., 1.], (count, 1)),
                             np.full(count, reliable, dtype=bool))


def _pair(history=None, config=None):
    first, second = _LiteralConsensus(config), BoundaryConsensus(config)
    history = {} if history is None else history
    for consensus in (first, second):
        consensus._continuations = dict(history)
        consensus._continuation_storage_count = sum(
            1+sum(1+len(frame.quads) for frame in frames)
            for hypotheses in history.values() for frames in hypotheses.values())
    return first, second


def _direct(pair, obs, updates, owners, expired=()):
    before = tuple(_state(item) for item in pair)
    inputs = _freeze((obs, updates, owners))
    old = pair[0]._prepare_continuations(obs, updates, owners, expired, COSINE)
    new = pair[1]._prepare_continuations(obs, updates, owners, expired, COSINE)
    assert _freeze(new) == _freeze(old)
    assert tuple(_state(item) for item in pair) == before
    assert _freeze((obs, updates, owners)) == inputs
    for _, hypotheses in new[0]:
        assert isinstance(hypotheses, MappingProxyType)
        for frames in hypotheses.values():
            for frame in frames:
                for name in ('quads', 'normals', 'reliable'):
                    with pytest.raises(ValueError):
                        getattr(frame, name).setflags(write=True)
    return new


@pytest.mark.parametrize('case', [
    'plain', 'raw_j', 'model_j', 'fragment_j', 'invalid', 'untrusted',
    'untrusted_neighbor', 'opposite_neighbor', 'unowned_neighbor',
    'open_endpoint', 'signed_zero', 'rotated', 'no_footprints'])
def test_original_order_admission_signatures_and_observation_arrays_exact(case):
    obs = _observation(11*DT)
    owners = _owners(len(obs.points))
    members = [1, 2, 3]
    if case in ('raw_j', 'model_j', 'fragment_j'):
        name = {'raw_j': 'sensitivity', 'model_j': 'position_sensitivity',
                'fragment_j': 'position_segment_sensitivity'}[case]
        values = np.array([.03, .1, np.nextafter(.1, np.inf),
                           .5, np.nextafter(.5, np.inf)])
        obs = replace(obs, **{name: values})
    elif case == 'invalid':
        valid = np.array([True, True, False, True, True])
        obs = replace(obs, position_valid=valid, position_reliable=valid,
                      position_segment_valid=valid, position_segment_reliable=valid)
    elif case in ('untrusted', 'untrusted_neighbor'):
        trusted = np.ones(5, dtype=bool)
        trusted[[2] if case == 'untrusted' else [0]] = False
        obs = replace(obs, position_reliable=trusted,
                      position_segment_reliable=trusted)
    elif case == 'opposite_neighbor':
        normals = obs.position_normals.copy()
        normals[0] = [0., -1.]
        obs = replace(obs, position_normals=normals)
    elif case == 'unowned_neighbor':
        owners[0] = -1
    elif case == 'open_endpoint':
        members = [0, 1]
    elif case == 'signed_zero':
        normals = obs.position_normals.copy()
        normals[:, 0] = -0.
        quads = obs.position_footprints.copy()
        quads[2, 0, 1] = -0.
        obs = replace(obs, position_normals=normals, position_footprints=quads)
    elif case == 'rotated':
        obs = obs.transformed([1., -2., .31])
    elif case == 'no_footprints':
        obs = replace(obs, position_footprints=None)
    history = {7: MappingProxyType({(8, 9): (_frame(DT),)})}
    result, count = _direct(_pair(history), obs, _updates([(17, members)]), owners)
    assert len(result) == 1 and result[0][0] == 17
    assert count == 3+sum(1+sum(frame.storage_units for frame in frames)
                         for frames in result[0][1].values())


@pytest.mark.parametrize('model,footprints', [(False, False), (True, False), (True, True)])
def test_empty_arrays_and_no_updates_keep_old_result(model, footprints):
    obs = _observation(xs=(), model=model, footprints=footprints)
    assert _direct(_pair(), obs, (), _owners(0)) == ((), 0)


def test_legacy_missing_pixels_does_not_create_history():
    obs = _observation(model=False)
    result = _direct(_pair(), obs, _updates([(1, [1, 2])]), _owners(5))
    assert result == ((), 0)


@pytest.mark.parametrize('closed', [False, True])
def test_multiple_updates_and_closed_run_preserve_face_and_signature_order(closed):
    obs = _observation(2*DT, closed=closed)
    updates = _updates([(41, [3, 4]), (8, [0, 1]), (27, [2])])
    result, count = _direct(_pair(), obs, updates, _owners(5))
    assert [identity for identity, _ in result] == [41, 8, 27]
    expected = ((102, 100), (104, 102), (101, 103)) if closed else (
        (102, -1), (-1, 102), (101, 103))
    assert tuple(next(iter(histories)) for _, histories in result) == expected
    assert count == sum(1+sum(frame.storage_units for frame in frames)
                        for _, hypotheses in result for frames in hypotheses.values())


@pytest.mark.parametrize('seed', range(8))
def test_many_unmodified_hypotheses_replacements_and_expiry_exact(seed):
    rng = np.random.RandomState(seed)
    obs = _observation(20*DT, closed=bool(seed % 2))
    histories = {}
    for identity in (11, 12, 13, 99):
        hypotheses = {}
        for key in ((100, 102), (102, 104), (-1, 102), (123, 321)):
            hypotheses[key] = tuple(_frame((index+1)*DT, int(rng.randint(0, 4)),
                                           bool(index % 2), offset=identity*.001)
                                    for index in range(int(rng.randint(1, 7))))
        histories[identity] = MappingProxyType(hypotheses)
    pair = _pair(histories)
    result, count = _direct(pair, obs, _updates([(12, [1]), (11, [3]), (13, [0])]),
                            _owners(5), expired=(99,))
    expected = dict(histories)
    expected.pop(99)
    expected.update(result)
    # Independent full recount verifies the changed-signature delta identity.
    assert count == sum(1+sum(1+len(frame.quads) for frame in frames)
                        for hypotheses in expected.values() for frames in hypotheses.values())
    for identity, hypotheses in result:
        for signature, frames in histories[identity].items():
            if signature != next(key for key, items in hypotheses.items()
                                 if items[-1].stamp_ns == 20*DT):
                assert hypotheses[signature] is frames


def test_retained_reliable_old_frame_and_latest_five_are_not_reweighted():
    key = (101, 103)
    frames = (_frame(DT, 2, True),) + tuple(
        _frame(index*DT, 1, False) for index in range(6, 11))
    pair = _pair({12: MappingProxyType({key: frames})})
    result, _ = _direct(pair, _observation(11*DT, trusted=False),
                        _updates([(12, [2])]), _owners(5))
    retained = result[0][1][key]
    assert [frame.stamp_ns//DT for frame in retained] == [1, 7, 8, 9, 10, 11]
    assert retained[0] is frames[0]
    assert sum(bool(np.any(frame.reliable)) for frame in retained) == 1


@pytest.mark.parametrize('stamp', [DT, DT-1])
def test_nonincreasing_witness_error_and_state_are_exact(stamp):
    pair = _pair({12: MappingProxyType({(101, 103): (_frame(DT),)})})
    obs, updates, owners = _observation(stamp), _updates([(12, [2])]), _owners(5)
    before = tuple(_state(item) for item in pair)
    errors = []
    for consensus in pair:
        with pytest.raises(ValueError) as caught:
            consensus._prepare_continuations(obs, updates, owners, (), COSINE)
        errors.append((type(caught.value), str(caught.value)))
    assert errors[0] == errors[1]
    assert tuple(_state(item) for item in pair) == before


def test_model_properties_read_once_per_frame_and_never_cached_across_calls(monkeypatch):
    pair = _pair()
    originals = {name: getattr(BoundaryObservation, name)
                 for name in ('model_valid', 'model_sensitivity', 'model_reliable', 'model_normals')}
    calls = dict.fromkeys(originals, 0)
    def make(name):
        def read(obs):
            calls[name] += 1
            return originals[name].fget(obs)
        return property(read)
    for name in originals:
        monkeypatch.setattr(BoundaryObservation, name, make(name))
    updates = _updates([(1, [1]), (2, [2]), (3, [3])])
    first = _observation(DT)
    one = pair[1]._prepare_continuations(first, updates, _owners(5), (), COSINE)
    assert list(calls.values()) == [1]*4
    changed = replace(first, source_stamp_ns=2*DT, position_sensitivity=np.full(5, .6))
    two = pair[1]._prepare_continuations(changed, updates, _owners(5), (), COSINE)
    assert list(calls.values()) == [2]*4
    assert any(len(frames[-1].quads) for _, hypotheses in one[0] for frames in hypotheses.values())
    assert all(not len(frames[-1].quads) for _, hypotheses in two[0] for frames in hypotheses.values())


def _prepare_pair(pair, obs):
    before = tuple(_state(item) for item in pair)
    inputs = _freeze(obs)
    tokens = []
    errors = []
    for consensus in pair:
        try:
            tokens.append(consensus.prepare(obs))
            errors.append(None)
        except ValueError as exc:
            tokens.append(None)
            errors.append((type(exc), str(exc)))
    assert errors[0] == errors[1]
    assert tuple(_state(item) for item in pair) == before
    assert _freeze(obs) == inputs
    if tokens[0] is not None:
        assert _freeze(tokens[0]) == _freeze(tokens[1])
    return tokens, errors[0]


def _commit_pair(pair, tokens):
    assert all(consensus.commit(token) for consensus, token in zip(pair, tokens))
    assert _state(pair[0]) == _state(pair[1])
    assert _freeze(pair[0].snapshot()) == _freeze(pair[1].snapshot())
    assert _freeze(pair[0].snapshot_near([0., .01], .3, 100)) == _freeze(
        pair[1].snapshot_near([0., .01], .3, 100))
    for consensus in pair:
        measured = sum(1+sum(1+len(frame.quads) for frame in frames)
                       for hypotheses in consensus._continuations.values()
                       for frames in hypotheses.values())
        assert consensus._continuation_storage_count == measured


@pytest.mark.parametrize('closed', [False, True])
def test_public_prepare_commit_five_plus_one_multiple_runs_then_replacement(closed):
    pair = _pair()
    for index in range(1, 13):
        xs = (-.2, 0., .4) if index <= 5 else (-.2, 0., .2, .4)
        obs = _observation(index*DT, xs=xs, closed=closed, trusted=index in (1, 6, 12))
        tokens, error = _prepare_pair(pair, obs)
        assert error is None
        _commit_pair(pair, tokens)
    assert pair[1]._generation == 12
    assert pair[1].source_frontier_ns == 12*DT
    assert any(len(hypotheses) > 1 for hypotheses in pair[1]._continuations.values())
    before = tuple(_state(item) for item in pair)
    assert not pair[0].commit(tokens[0]) and not pair[1].commit(tokens[1])
    assert tuple(_state(item) for item in pair) == before


def test_public_rejected_alternative_and_foreign_token_preserve_all_state():
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
    repeated, error = _prepare_pair(pair, _observation(6*DT))
    assert repeated == [None, None] and error[0] is ValueError


@pytest.mark.parametrize('confirmed', [False, True])
def test_public_ttl_expiry_debits_only_unconfirmed_continuations(confirmed):
    pair = _pair(config={'candidate_ttl_s': .15})
    for index in range(1, 6 if confirmed else 2):
        tokens, error = _prepare_pair(pair, _observation(index*DT))
        assert error is None
        _commit_pair(pair, tokens)
    ids = set(pair[1]._records)
    tokens, error = _prepare_pair(pair, _observation(10*DT, xs=()))
    assert error is None
    assert set(tokens[1].expired_ids) == (set() if confirmed else ids)
    _commit_pair(pair, tokens)
    assert bool(pair[1]._continuations) is confirmed


@pytest.mark.parametrize('capacity', [1, 7, 14, 59, 60, 200000])
def test_public_capacity_boundaries_same_error_or_token_no_partial_mutation(capacity):
    pair = _pair(config={'max_support_segments': capacity})
    for index in range(1, 11):
        xs = (-.2, 0., .4) if index <= 5 else (-.2, 0., .2, .4)
        tokens, error = _prepare_pair(pair, _observation(index*DT, xs=xs))
        if error is not None:
            assert 'capacity' in error[1]
            return
        _commit_pair(pair, tokens)
    assert pair[1]._generation == 10

