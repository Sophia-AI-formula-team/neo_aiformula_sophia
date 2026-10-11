"""Byte-equivalence of allocation-only footprint packing against literal R20.

The oracle is the original packing block in paint_geometry.py at commit
51890caaac3be996b6ffef53eee7c64cd8043859, with only its surrounding arguments
and return made explicit. It does not call the candidate packing helper.
Tests use synthetic arrays, never a private diagnostic path or recorded mask.
This is a packing contract, not geometry admission or a performance benchmark.
"""
from types import SimpleNamespace
import unittest

import numpy as np

from lane_mapping_lya_reference import paint_geometry as paint


def literal_r20_packing(obs, updates, offsets, packed_segments, packed_normals,
                        packed_reliable, face_reliable, cfg):
    model_normals = obs.model_normals
    if obs.position_segments is not None:
        segment_groups, normal_groups, reliability_groups = [], [], []
        for index, (_, _, _, _, _, _, members) in enumerate(updates):
            start, stop = offsets[index:index+2]
            own = members[obs.position_segment_valid[members]
                & (obs.position_segment_sensitivity[members] <= cfg["max_sensitivity"])]
            own_normals = np.repeat(model_normals[own, None, :], 2, axis=1)
            own_reliable = (face_reliable[own] & obs.position_segment_reliable[own]
                & (obs.position_segment_sensitivity[own] <= cfg["reliable_sensitivity"]))
            segment_groups.append(np.concatenate((packed_segments[start:stop], obs.position_segments[own])))
            normal_groups.append(np.concatenate((packed_normals[start:stop], own_normals)))
            reliability_groups.append(np.r_[packed_reliable[start:stop], own_reliable])
        if segment_groups:
            offsets = np.r_[0, np.cumsum([len(group) for group in segment_groups], dtype=np.int64)]
            packed_segments, packed_normals = np.concatenate(segment_groups), np.concatenate(normal_groups)
            packed_reliable = np.concatenate(reliability_groups)
    return offsets, packed_segments, packed_normals, packed_reliable


def identity(value):
    if isinstance(value, np.ndarray):
        return value.dtype.str, value.shape, value.strides, value.flags.writeable, value.tobytes()
    if isinstance(value, SimpleNamespace):
        return identity(vars(value))
    if isinstance(value, dict):
        return tuple((key, identity(child)) for key, child in sorted(value.items()))
    if isinstance(value, (tuple, list)):
        return tuple(identity(child) for child in value)
    return value


def make_case(groups=(), pair_counts=None, *, nfaces=None, seed=2100):
    groups = [np.asarray(group, dtype=np.int64) for group in groups]
    if nfaces is None:
        nfaces = max((int(np.max(group))+1 for group in groups if len(group)), default=0)
    pair_counts = [0]*len(groups) if pair_counts is None else list(pair_counts)
    if len(pair_counts) != len(groups):
        raise ValueError('test fixture counts do not match groups')
    offsets = np.r_[0, np.cumsum(pair_counts, dtype=np.int64)]
    rng = np.random.default_rng(seed)
    obs = SimpleNamespace(model_normals=rng.normal(size=(nfaces, 2)),
        position_segments=rng.normal(size=(nfaces, 2, 2)),
        position_segment_valid=np.ones(nfaces, dtype=bool),
        position_segment_sensitivity=np.full(nfaces, .05, dtype=np.float64),
        position_segment_reliable=np.ones(nfaces, dtype=bool))
    updates = [(100+index, (index, -index), (0., 0.), (0., 1.), 5, 1, group)
               for index, group in enumerate(groups)]
    count = int(offsets[-1])
    return dict(obs=obs, updates=updates, offsets=offsets,
        packed_segments=rng.normal(size=(count, 2, 2)),
        packed_normals=rng.normal(size=(count, 2, 2)),
        packed_reliable=rng.random(count) < .5,
        face_reliable=np.ones(nfaces, dtype=bool),
        cfg=dict(max_sensitivity=.5, reliable_sensitivity=.1))


def strided(array):
    storage = np.empty((len(array)*2,)+array.shape[1:], dtype=array.dtype)
    storage[::2] = array
    return storage[::2]


class FootprintPackingTests(unittest.TestCase):
    def assert_same(self, values):
        before = identity(values)
        expected = literal_r20_packing(**values)
        self.assertEqual(identity(values), before, 'literal oracle mutated its inputs')
        actual = paint._pack_footprint_support(**values)
        self.assertEqual(identity(values), before, 'candidate mutated input bytes/shape/strides/flags')
        self.assertEqual(len(actual), 4)
        for index, (got, want) in enumerate(zip(actual, expected)):
            with self.subTest(output=index):
                self.assertEqual(got.dtype.str, want.dtype.str)
                self.assertEqual(got.shape, want.shape)
                self.assertEqual(got.tobytes(), want.tobytes())
        self.assertEqual(actual[0].dtype, np.dtype(np.int64))
        self.assertEqual(actual[1].dtype, np.dtype(np.float64))
        self.assertEqual(actual[2].dtype, np.dtype(np.float64))
        self.assertEqual(actual[3].dtype, np.dtype(bool))
        return actual

    def test_no_groups_no_faces(self):
        output = self.assert_same(make_case())
        self.assertEqual(output[0].tolist(), [0])
        self.assertEqual(output[1].shape, (0, 2, 2))

    def test_no_groups_unused_faces_not_packed(self):
        output = self.assert_same(make_case(nfaces=12))
        self.assertEqual(len(output[1]), 0)

    def test_single_empty_group(self):
        output = self.assert_same(make_case([[]]))
        self.assertEqual(output[0].tolist(), [0, 0])

    def test_several_empty_groups(self):
        output = self.assert_same(make_case([[], [], []]))
        self.assertEqual(output[0].tolist(), [0, 0, 0, 0])

    def test_only_own_fragments(self):
        values = make_case([[2, 0], [1, 3]])
        output = self.assert_same(values)
        self.assertEqual(output[0].tolist(), [0, 2, 4])
        self.assertEqual(output[1].tobytes(), values['obs'].position_segments[[2, 0, 1, 3]].tobytes())

    def test_only_incident_pairs(self):
        values = make_case([[0, 1], [], [2]], [2, 1, 3])
        values['obs'].position_segment_valid[:] = False
        output = self.assert_same(values)
        self.assertEqual(output[0].tolist(), [0, 2, 3, 6])
        self.assertEqual(output[1].tobytes(), values['packed_segments'].tobytes())

    def test_empty_members_can_have_existing_pairs(self):
        output = self.assert_same(make_case([[], [], []], [0, 2, 1]))
        self.assertEqual(output[0].tolist(), [0, 0, 2, 3])

    def test_pairs_before_own_and_group_order(self):
        values = make_case([[2, 0], [], [1, 3]], [1, 2, 1])
        output = self.assert_same(values)
        pair, own = values['packed_segments'], values['obs'].position_segments
        expected = np.concatenate((pair[:1], own[[2, 0]], pair[1:3], pair[3:], own[[1, 3]]))
        self.assertEqual(output[0].tolist(), [0, 3, 5, 8])
        self.assertEqual(output[1].tobytes(), expected.tobytes())

    def test_group_identity_fields_are_not_selection_or_order(self):
        values = make_case([[3, 1], [0, 2]], [2, 1])
        values['updates'] = [(91, (99, -99), (9., 8.), (-1., 0.), 321, 0, values['updates'][0][-1]),
                             (2, (-88, 77), (-9., -8.), (1., 0.), 6, 5, values['updates'][1][-1])]
        self.assert_same(values)

    def test_maximum_j_is_inclusive_and_neighbours_differ(self):
        values = make_case([list(range(5))], [1])
        values['obs'].position_segment_sensitivity[:] = [np.nextafter(.5, -np.inf), .5,
                                                        np.nextafter(.5, np.inf), np.inf, np.nan]
        output = self.assert_same(values)
        self.assertEqual(output[0].tolist(), [0, 3])
        self.assertEqual(output[1][1:].tobytes(), values['obs'].position_segments[:2].tobytes())

    def test_reliable_j_is_inclusive_without_changing_admission(self):
        values = make_case([[0, 1, 2]])
        values['obs'].position_segment_sensitivity[:] = [np.nextafter(.1, -np.inf), .1,
                                                        np.nextafter(.1, np.inf)]
        output = self.assert_same(values)
        self.assertEqual(output[3].tolist(), [True, True, False])

    def test_three_reliability_inputs_all_required(self):
        values = make_case([list(range(8))])
        values['face_reliable'][:] = [False, False, False, False, True, True, True, True]
        values['obs'].position_segment_reliable[:] = [False, False, True, True]*2
        values['obs'].position_segment_sensitivity[:] = [.2, .05]*4
        output = self.assert_same(values)
        self.assertEqual(output[3].tolist(), [False]*7+[True])

    def test_invalid_rows_nan_coordinates_and_normals_are_not_touched(self):
        values = make_case([[0, 1, 2], [3, 4]], [1, 0])
        invalid = np.array([0, 2, 4])
        values['obs'].position_segment_valid[invalid] = False
        values['obs'].position_segments[invalid] = np.nan
        values['obs'].model_normals[invalid] = np.nan
        values['obs'].position_segment_sensitivity[invalid] = np.nan
        output = self.assert_same(values)
        self.assertTrue(np.all(np.isfinite(output[1])))
        self.assertTrue(np.all(np.isfinite(output[2])))

    def test_above_j_rows_nan_geometry_is_not_copied(self):
        values = make_case([[0, 1, 2]])
        values['obs'].position_segment_sensitivity[[0, 2]] = np.inf
        values['obs'].position_segments[[0, 2]] = np.nan
        values['obs'].model_normals[[0, 2]] = np.nan
        output = self.assert_same(values)
        self.assertEqual(len(output[1]), 1)
        self.assertTrue(np.all(np.isfinite(output[1])))

    def test_all_rows_invalid_leaves_pairs_unchanged(self):
        values = make_case([[0, 1], [2]], [1, 2])
        values['obs'].position_segment_valid[:] = False
        values['obs'].position_segments[:] = np.nan
        values['obs'].model_normals[:] = np.nan
        output = self.assert_same(values)
        self.assertEqual(output[1].tobytes(), values['packed_segments'].tobytes())

    def test_endpoint_normals_repeat_exact_order_not_pair_normals(self):
        values = make_case([[2, 0], [1]], [2, 0])
        values['obs'].model_normals[:] = [[1., 2.], [3., 4.], [5., 6.]]
        output = self.assert_same(values)
        expected = np.array([[[5., 6.], [5., 6.]], [[1., 2.], [1., 2.]], [[3., 4.], [3., 4.]]])
        self.assertEqual(output[2][2:].tobytes(), expected.tobytes())

    def test_signed_zero_geometry_and_normals_bits(self):
        values = make_case([[1, 0]], [2])
        pattern = np.array([0., -0., -0., 0.])
        values['obs'].position_segments[:] = pattern.reshape(1, 2, 2)
        values['obs'].model_normals[:] = [[-0., 0.], [0., -0.]]
        values['packed_segments'][:] = pattern[::-1].reshape(1, 2, 2)
        values['packed_normals'][:] = pattern.reshape(1, 2, 2)
        output = self.assert_same(values)
        self.assertEqual(int(np.count_nonzero(np.signbit(output[1]))), 8)
        self.assertEqual(int(np.count_nonzero(np.signbit(output[2]))), 8)

    def test_subnormal_and_large_finite_values_are_only_copied(self):
        values = make_case([[0, 1]], [1])
        tiny = float.fromhex('0x0.0000000000001p-1022')
        pattern = np.array([tiny, -tiny,
                            np.finfo(float).max, -np.finfo(float).max])
        values['obs'].position_segments[:] = pattern.reshape(1, 2, 2)
        values['packed_segments'][:] = pattern[::-1].reshape(1, 2, 2)
        self.assert_same(values)

    def test_noncontiguous_all_array_inputs(self):
        values = make_case([[3, 1], [], [0, 2]], [2, 0, 1])
        for key, value in vars(values['obs']).items():
            setattr(values['obs'], key, strided(value))
        for key in ('offsets', 'packed_segments', 'packed_normals', 'packed_reliable', 'face_reliable'):
            values[key] = strided(values[key])
        values['updates'] = [tuple(update[:-1])+(strided(update[-1]),) for update in values['updates']]
        self.assert_same(values)

    def test_negative_stride_members_and_geometry(self):
        values = make_case([[0, 1, 2], [3, 4]], [2, 1])
        for key, value in vars(values['obs']).items():
            setattr(values['obs'], key, value[::-1])
        values['face_reliable'] = values['face_reliable'][::-1]
        values['updates'] = [tuple(update[:-1])+(update[-1][::-1],) for update in values['updates']]
        for key in ('packed_segments', 'packed_normals', 'packed_reliable'):
            values[key] = values[key][::-1]
        self.assert_same(values)

    def test_fortran_geometry(self):
        values = make_case([[0, 2], [1, 3]], [1, 3])
        for key in ('model_normals', 'position_segments'):
            setattr(values['obs'], key, np.asfortranarray(getattr(values['obs'], key)))
        for key in ('packed_segments', 'packed_normals'):
            values[key] = np.asfortranarray(values[key])
        self.assert_same(values)

    def test_immutable_byte_backed_inputs(self):
        values = make_case([[3, 0], [], [1, 2]], [2, 1, 0])
        def freeze(array):
            return np.frombuffer(array.tobytes(), dtype=array.dtype).reshape(array.shape)
        for key, value in vars(values['obs']).items():
            setattr(values['obs'], key, freeze(value))
        for key in ('offsets', 'packed_segments', 'packed_normals', 'packed_reliable', 'face_reliable'):
            values[key] = freeze(values[key])
        values['updates'] = [tuple(update[:-1])+(freeze(update[-1]),) for update in values['updates']]
        self.assert_same(values)

    def test_strict_numpy_error_mode_for_finite_copy_inputs(self):
        values = make_case([[0, 2], [1]], [0, 2])
        with np.errstate(all='raise'):
            self.assert_same(values)

    def test_deterministic_randomized_mixed_groups(self):
        rng = np.random.default_rng(0x2120518)
        for case in range(64):
            groups_count = int(rng.integers(0, 18))
            nfaces = int(rng.integers(0, 70))
            members = rng.permutation(nfaces)
            cuts = sorted(rng.integers(0, nfaces+1, size=max(0, groups_count-1)).tolist())
            groups = np.split(members, cuts) if groups_count else []
            pairs = rng.integers(0, 9, size=groups_count)
            values = make_case(groups, pairs, nfaces=nfaces, seed=case)
            values['obs'].position_segment_valid[:] = rng.random(nfaces) < .7
            values['obs'].position_segment_reliable[:] = rng.random(nfaces) < .5
            values['face_reliable'][:] = rng.random(nfaces) < .6
            values['obs'].position_segment_sensitivity[:] = rng.choice(
                [0., .1, np.nextafter(.1, np.inf), .5, np.nextafter(.5, np.inf), np.nan], nfaces)
            with self.subTest(case=case):
                self.assert_same(values)

    def test_many_groups_with_repeated_empty_offsets(self):
        groups = [[index] if index % 3 else [] for index in range(2048)]
        pairs = [int(index % 5 == 0) for index in range(len(groups))]
        values = make_case(groups, pairs, nfaces=2048)
        output = self.assert_same(values)
        self.assertEqual(len(output[0]), 2049)
        self.assertEqual(int(output[0][-1]), sum(map(len, groups))+sum(pairs))

    def test_original_two_hundred_thousand_fragment_capacity_scale(self):
        # Two groups exercise a real 200000-segment packed buffer without a
        # quadratic test oracle or calling consensus/solver/capacity policy.
        values = make_case([np.arange(50000), np.arange(50000, 100000)], [50000, 50000])
        output = self.assert_same(values)
        self.assertEqual(output[0].tolist(), [0, 100000, 200000])
        self.assertEqual(output[1].shape, (200000, 2, 2))


if __name__ == '__main__':
    unittest.main()
