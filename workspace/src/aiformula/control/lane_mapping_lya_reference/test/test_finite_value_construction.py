"""Literal pre-change constructor invariants, including ordered failures."""
import math
import struct
from unittest.mock import patch

import numpy as np
import pytest

from lane_mapping_lya_reference import finite_support as fs


def original_interval(lower, upper, votes, reliable, stamp):
    lower = fs._number(lower, 'interval.lower')
    upper = fs._number(upper, 'interval.upper')
    if not lower < upper or max(abs(lower), abs(upper)) > 4. * fs.MAX_COORDINATE:
        raise ValueError('interval requires bounded positive extent')
    votes = fs._integer(votes, 'interval.frame_votes', maximum=fs.MAX_VOTES)
    reliable = fs._integer(reliable, 'interval.reliable_votes', minimum=0, maximum=votes)
    stamp = fs._integer(stamp, 'interval.last_stamp_ns')
    return lower, upper, votes, reliable, stamp


def original_domain(origin, tangent, intervals, stamp):
    origin = fs._vector(origin, 'origin')
    tangent = fs._vector(tangent, 'tangent', unit=True)
    stamp = fs._integer(stamp, 'source_stamp_ns')
    try:
        intervals = tuple(intervals)
    except TypeError as error:
        raise ValueError('intervals must be a finite sequence') from error
    if len(intervals) > fs.MAX_ITEMS or any(type(item) is not fs.Interval for item in intervals):
        raise ValueError('invalid interval sequence or capacity')
    for index, piece in enumerate(intervals):
        if piece.last_stamp_ns > stamp:
            raise ValueError('interval observation is newer than state frontier')
        if index and intervals[index-1].upper > piece.lower:
            raise ValueError('interval interiors must be sorted and disjoint')
    return origin, tangent, intervals, stamp


def normalize(value):
    if type(value) is float:
        return struct.pack('d', value)
    if type(value) in (tuple, list):
        return tuple(normalize(item) for item in value)
    if type(value) is fs.Interval:
        return normalize((value.lower, value.upper, value.frame_votes,
                          value.reliable_votes, value.last_stamp_ns))
    if type(value) is fs.DomainState:
        return normalize((value.origin, value.tangent, value.intervals, value.source_stamp_ns))
    return value


def outcome(function, args):
    try:
        return 'value', normalize(function(*args))
    except Exception as error:
        return type(error), str(error)


@pytest.mark.parametrize('field,values', [
    (0, [-0., -1., 0, np.float64(0), True, '0', math.nan, math.inf, -4.e7-1]),
    (1, [0., 1., 1, np.float64(1), False, '1', math.nan, math.inf, 4.e7+1]),
    (2, [0, 1, 5, fs.MAX_VOTES, fs.MAX_VOTES+1, True, np.int64(5), 1.]),
    (3, [-1, 0, 1, 5, 6, True, np.int64(1), 1.]),
    (4, [0, 1, fs.MAX_STAMP, fs.MAX_STAMP+1, True, np.int64(1), 1.]),
])
def test_interval_values_and_first_errors(field, values):
    for value in values:
        args = [0., 1., 5, 1, 1]
        args[field] = value
        assert outcome(fs.Interval, args) == outcome(original_interval, args)


def test_domain_preserves_all_public_checks_and_signed_zero():
    normal = ((-0., 0.), (1., -0.), (fs.Interval(-0., 1., 5, 1, 1),), 1)
    cases = [normal, ((0, 0), (1, 0), [], np.int64(1)),
             ((math.nan, 0.), (2., 0.), (), 0),
             ((0., 0.), (2., 0.), (), 0),
             ((0., 0.), (1., 0.), (object(),), 1),
             ((0., 0.), (1., 0.), None, 1),
             ((0., 0.), (1., 0.), (fs.Interval(0., 1., 1, 0, 2),), 1),
             ((0., 0.), (1., 0.), (fs.Interval(0., 2., 1, 0, 1),
                                    fs.Interval(1., 3., 1, 0, 1)), 1)]
    for args in cases:
        assert outcome(fs.DomainState, args) == outcome(original_domain, args)


def test_rebound_validators_keep_ordered_calls():
    for actual, literal, args in [(fs.Interval, original_interval, (0., 1., 5, 1, 1)),
                                  (fs.DomainState, original_domain, ((0., 0.), (1., 0.), (), 1))]:
        for name in ('_number', '_integer', '_vector'):
            original = getattr(fs, name)
            with patch.object(fs, name, wraps=original) as recorder:
                expected = outcome(literal, args)
                calls = recorder.call_args_list[:]
                recorder.reset_mock()
                assert outcome(actual, args) == expected
                assert recorder.call_args_list == calls


def test_forged_interval_is_not_silently_trusted_by_domain():
    piece = fs.Interval(0., 1., 1, 0, 1)
    object.__setattr__(piece, 'last_stamp_ns', 3)
    args = ((0., 0.), (1., 0.), (piece,), 1)
    assert outcome(fs.DomainState, args) == outcome(original_domain, args)
    assert outcome(fs.DomainState, args)[0] is ValueError


def test_interval_subclass_descriptor_keeps_both_original_assignments():
    assigned = []
    class RecordedInterval(fs.Interval):
        @property
        def lower(self):
            return self._lower
        @lower.setter
        def lower(self, value):
            assigned.append(value)
            object.__setattr__(self, '_lower', value)
    value = RecordedInterval(-0., 1., 5, 1, 1)
    assert len(assigned) == 2
    assert all(struct.pack('d', item) == struct.pack('d', -0.) for item in assigned)
    assert value.lower == -0.
