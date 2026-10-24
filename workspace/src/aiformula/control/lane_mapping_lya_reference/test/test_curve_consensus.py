"""Independent artificial directed-pair schema tests; candidate NOT RUN.

No real mask, ROS, native module, pose/GNSS truth or Unity is used. Every pair
has full original dual-endpoint fields; exact integer-ratio arithmetic checks
exported finite chart support without reusing the subject enclosure.
"""
from dataclasses import FrozenInstanceError
import math
import unittest

import numpy as np

from lane_mapping_lya_reference import curve_consensus as subject


FIELDS = ('roles', 'face_ids', 'previous_next', 'contour_ids', 'endpoint_owners',
          'model_roots', 'normals', 'quads', 'quad_valid', 'reliable', 'valid')
COSINE = math.cos(math.pi/6.)  # Existing source default30deg, never relaxed.


def _packet_args(stamp, intervals=((0., 2.),), *, intercept=0., slope=0.,
                 function=None, derivative=None, reliable=False, owner=11,
                 side=1, signature=(7, 19), origin=(0., 0.), tangent=(1., 0.),
                 connect=True, roles=None, endpoint_owners=None):
    function = function or (lambda u: intercept+slope*u)
    derivative = derivative or (lambda u: slope)
    points, pairs, contours, neighbor = {}, [], {}, {}
    previous_stop, previous_face, contour = None, None, -1
    for lower, upper in intervals:
        if connect and previous_face is not None and lower == previous_stop:
            first = previous_face
        else:
            contour += 1
            first = len(points)
            points[first] = lower
            contours[first] = contour
            neighbor[first] = [-1, -1]
        second = len(points)
        points[second] = upper
        contours[second] = contour
        neighbor[second] = [first, -1]
        neighbor[first][1] = second
        pairs.append((first, second))
        previous_stop, previous_face = upper, second
    count = len(pairs)
    face_ids = np.array(pairs, dtype=np.int64).reshape(-1, 2)
    endpoint_owners = (np.full((count, 2), owner, dtype=np.int64) if endpoint_owners is None
                       else np.array(endpoint_owners, dtype=np.int64).reshape(count, 2))
    role_array = (np.full(count, 1, dtype=np.uint8) if roles is None
                  else np.array(roles, dtype=np.uint8).reshape(count))
    t, n = tuple(tangent), (-tangent[1], tangent[0])
    denominator = t[0]*t[0]+t[1]*t[1]
    roots, normals, quads = {}, {}, {}
    for face, u in points.items():
        v = function(u)
        roots[face] = tuple(origin[i]+(t[i]*u+n[i]*v)/denominator for i in (0, 1))
        dv = derivative(u)
        direction = tuple((t[i]+n[i]*dv)/denominator for i in (0, 1))
        length = math.hypot(*direction)
        normals[face] = (-direction[1]/length, direction[0]/length)
        # One original per-face audit quad; repeated source faces reuse EXACT
        # bytes. This is not a nominal sample or2D connection permission.
        quads[face] = [tuple(roots[face][i]+a*t[i]+b*n[i] for i in (0, 1))
                      for a, b in ((-.0625, -.0625), (.0625, -.0625),
                                   (.0625, .0625), (-.0625, .0625))]
    return dict(owner_id=owner, signed_side=side, signature=signature, stamp_ns=stamp,
        roles=role_array, face_ids=face_ids,
        previous_next=np.array([[neighbor[face] for face in pair] for pair in pairs],
                               dtype=np.int64).reshape(count, 2, 2),
        contour_ids=np.array([[contours[face] for face in pair] for pair in pairs],
                             dtype=np.int64).reshape(count, 2),
        endpoint_owners=endpoint_owners,
        model_roots=np.array([[roots[face] for face in pair] for pair in pairs],
                             dtype=np.float64).reshape(count, 2, 2),
        normals=np.array([[normals[face] for face in pair] for pair in pairs],
                         dtype=np.float64).reshape(count, 2, 2),
        quads=np.array([[quads[face] for face in pair] for pair in pairs],
                       dtype=np.float64).reshape(count, 2, 4, 2),
        quad_valid=np.ones((count, 2), dtype=np.bool_),
        reliable=np.broadcast_to(np.asarray(reliable, dtype=np.bool_), (count, 2)).copy(),
        valid=np.ones((count, 2), dtype=np.bool_))


def _packet(stamp, intervals=((0., 2.),), **kwargs):
    return subject.FrameCurve(**_packet_args(stamp, intervals, **kwargs))


def _state(*, owner=11, side=1, signature=(7, 19), origin=(0., 0.),
           tangent=(1., 0.), capacity=200000):
    return subject.create_curve_state(owner, side, signature, origin, tangent,
        normal_cosine=COSINE, max_storage_units=capacity)


def _advance(state, packet):
    return subject.advance_curve(state, packet, available_before_ns=packet.stamp_ns)


def _five(*, capacity=200000, origin=(0., 0.), tangent=(1., 0.),
          intervals=((0., 2.),), intercept=0., slope=0.):
    state = _state(capacity=capacity, origin=origin, tangent=tangent)
    for stamp in range(1, 6):
        state = _advance(state, _packet(stamp, intervals, reliable=(stamp == 1),
            origin=origin, tangent=tangent, intercept=intercept, slope=slope))
    return state


def _signature(value):
    return value.dtype.str, value.shape, value.tobytes(), value.flags.writeable


def _packet_signature(packet):
    return (packet.owner_id, packet.signed_side, packet.signature, packet.stamp_ns,
            packet.source_kind, tuple(_signature(getattr(packet, name)) for name in FIELDS))


def _state_signature(state):
    return (state.owner_id, state.signed_side, state.signature, state.origin,
            state.tangent, state.normal_cosine, state.max_storage_units,
            state.source_frontier_ns, state.status, state.reason,
            tuple((_packet_signature(frame.packet), frame.status, frame.pieces)
                  for frame in state.frames), state.pieces, state.storage_units)


def _ratio_add(first, second):
    return first[0]*second[1]+second[0]*first[1], first[1]*second[1]


def _ratio_mul(first, second):
    return first[0]*second[0], first[1]*second[1]


def _exact_chart_u(point, origin, tangent):
    result = (0, 1)
    for x, o, t in zip(point, origin, tangent):
        ox = float(o).as_integer_ratio()
        delta = _ratio_add(float(x).as_integer_ratio(), (-ox[0], ox[1]))
        result = _ratio_add(result, _ratio_mul(delta, float(t).as_integer_ratio()))
    return result


def _ratio_le(first, second):
    return first[0]*second[1] <= second[0]*first[1]


def _value(state, coordinate):
    matches = [piece for piece in state.pieces if piece.lower < coordinate < piece.upper]
    if len(matches) != 1:
        raise AssertionError('literal query needs one finite piece, got '+str(len(matches)))
    piece = matches[0]
    return piece.intercept+piece.slope*coordinate, piece


class CurveConsensusPairTests(unittest.TestCase):
    def assertInsideOriginalDomains(self, state):
        for piece in state.pieces:
            lo, hi = float(piece.lower).as_integer_ratio(), float(piece.upper).as_integer_ratio()
            first = _exact_chart_u(piece.segment[0], state.origin, state.tangent)
            second = _exact_chart_u(piece.segment[1], state.origin, state.tangent)
            for endpoint in (first, second):
                self.assertTrue(_ratio_le(lo, endpoint))
                self.assertTrue(_ratio_le(endpoint, hi))
            self.assertTrue(first[0]*second[1] < second[0]*first[1])
            self.assertEqual(piece.frame_votes, 5)
            self.assertEqual(piece.reliable_votes, 1)
            self.assertEqual(len(piece.contributors), len(set(piece.contributors)))
            self.assertLessEqual(len(piece.contributors), 6)
            self.assertLessEqual(piece.source_stamp_ns, state.source_frontier_ns)

    def test_literal_directed_straight_pairs_confirm_five_plus_one(self):
        state = _five(intercept=.25)
        value, piece = _value(state, 1.)
        self.assertEqual(state.status, 'confirmed')
        self.assertEqual(value, .25)
        self.assertEqual((piece.intercept, piece.slope), (.25, 0.))
        self.assertEqual(piece.world_derivative, (1., 0.))
        self.assertEqual(piece.normal, (0., 1.))
        self.assertEqual(piece.contributors, (1, 2, 3, 4, 5))
        self.assertEqual(piece.reliable_contributors, (1,))
        self.assertInsideOriginalDomains(state)

    def test_same_owner_and_both_incident_roles_keep_all_dual_sources(self):
        args = _packet_args(1, ((-1., 0.), (0., 1.), (1., 2.)),
            roles=(2, 1, 2), endpoint_owners=((7, 11), (11, 11), (11, 19)))
        packet = subject.FrameCurve(**args)
        self.assertEqual(packet.roles.tolist(), [2, 1, 2])
        self.assertEqual(packet.face_ids.tolist(), [[0, 1], [1, 2], [2, 3]])
        self.assertEqual(packet.endpoint_owners.tolist(), [[7, 11], [11, 11], [11, 19]])
        for name in FIELDS:
            self.assertEqual(_signature(getattr(packet, name))[:3], _signature(args[name])[:3])
        state = _advance(_state(), packet)
        self.assertEqual(state.frames[0].status, 'known')
        self.assertEqual(len(state.frames[0].pieces), 3)

    def test_incident_copy_remains_separate_owner_and_counts_full_bytes(self):
        center_args = _packet_args(1, ((0., 2.),), roles=(2,), endpoint_owners=((11, 19),))
        neighbor_args = dict(center_args, owner_id=19, signature=(11, -1))
        center, neighbor = subject.FrameCurve(**center_args), subject.FrameCurve(**neighbor_args)
        self.assertEqual(center.payload_bytes+neighbor.payload_bytes, 558)
        for name in FIELDS:
            self.assertFalse(np.shares_memory(getattr(center, name), getattr(neighbor, name)))
            self.assertEqual(getattr(center, name).tobytes(), getattr(neighbor, name).tobytes())
        with self.assertRaises(ValueError):
            _advance(_state(), neighbor)
        self.assertEqual(_advance(_state(owner=19, signature=(11, -1)), neighbor).source_frontier_ns, 1)

    def test_roles_original_next_previous_contour_and_owner_are_strict(self):
        modifications = (
            ('roles', (0,), 0), ('roles', (0,), 3),
            ('previous_next', (0, 0, 1), -1), ('previous_next', (0, 0, 1), 77),
            ('previous_next', (0, 1, 0), -1), ('previous_next', (0, 1, 0), 77),
            ('contour_ids', (0, 1), 2), ('endpoint_owners', (0, 1), 19),
            ('face_ids', (0, 1), 0), ('face_ids', (0, 0), -1),
        )
        for name, index, value in modifications:
            with self.subTest(name=name, index=index, value=value):
                args = _packet_args(6)
                args[name][index] = value
                before = {key: _signature(args[key]) for key in FIELDS}
                with self.assertRaises(ValueError):
                    subject.FrameCurve(**args)
                self.assertEqual({key: _signature(args[key]) for key in FIELDS}, before)
        for owners in ((11, 11), (7, 19), (-1, 11)):
            args = _packet_args(6, roles=(2,), endpoint_owners=(owners,))
            with self.assertRaises(ValueError):
                subject.FrameCurve(**args)

    def test_repeated_source_face_fields_must_be_identical_not_refitted(self):
        for name in ('previous_next', 'contour_ids', 'endpoint_owners', 'model_roots',
                     'normals', 'quads', 'quad_valid', 'reliable', 'valid'):
            with self.subTest(name=name):
                args = _packet_args(6, ((0., 1.), (1., 2.)), reliable=True)
                if name == 'previous_next':
                    args[name][1, 0, 0] = 88
                elif name == 'contour_ids':
                    args[name][1, :] = 2
                elif name == 'endpoint_owners':
                    args[name][1, 0] = 12
                elif name == 'normals':
                    args[name][1, 0] = [-.125/math.hypot(.125, 1.), 1./math.hypot(.125, 1.)]
                elif name in ('model_roots', 'quads'):
                    args[name][1, 0].flat[0] += .125
                elif name == 'valid':
                    args[name][1, 0] = False
                    args['reliable'][1, 0] = False
                    args['quad_valid'][1, 0] = False
                else:
                    args[name][1, 0] = False
                with self.assertRaises(ValueError):
                    subject.FrameCurve(**args)

    def test_no_confirmation_with_four_frames_or_only_one_reliable_endpoint(self):
        state = _state()
        for stamp in range(1, 5):
            state = _advance(state, _packet(stamp, reliable=(stamp == 1)))
        self.assertEqual(state.pieces, ())
        state = _state()
        for stamp in range(1, 7):
            state = _advance(state, _packet(stamp, reliable=(True, False)))
        self.assertEqual(state.status, 'unconfirmed')
        self.assertEqual(state.pieces, ())
        self.assertFalse(any(frame.packet._has_reliable for frame in state.frames))

    def test_pair_density_is_not_frame_votes_or_extra_mean_weight(self):
        dense = tuple((i/8., (i+1)/8.) for i in range(16))
        state = _advance(_state(), _packet(1, dense, reliable=True))
        self.assertEqual(state.pieces, ())
        self.assertEqual(len(state.frames[0].packet.face_ids), 16)
        for stamp in range(2, 6):
            state = _advance(state, _packet(stamp, intercept=1.))
        self.assertEqual(_value(state, .3125)[0], 4./5.)
        self.assertEqual(_value(state, .3125)[1].contributors, (1, 2, 3, 4, 5))
        self.assertInsideOriginalDomains(state)

    def test_curved_visibility_uses_common_u_not_frame_centroids(self):
        def bend(u):
            return .125*u if u <= 1. else .25*u-.125
        def gradient(u):
            return .125 if u < 1. else (.25 if u > 1. else .1875)
        windows = (((0., 1.), (1., 2.)), ((.25, 1.), (1., 1.75)),
                   ((.5, .75), (.75, 1.), (1., 1.25), (1.25, 1.5)),
                   ((0., 1.), (1., 1.5)), ((.25, 1.), (1., 2.)))
        state, centroids = _state(), []
        for stamp, intervals in enumerate(windows, 1):
            packet = _packet(stamp, intervals, function=bend, derivative=gradient,
                             reliable=(stamp == 1))
            faces = {}
            for ids, roots in zip(packet.face_ids, packet.model_roots):
                faces.update(zip(map(int, ids), map(tuple, roots)))
            centroids.append(tuple(np.mean(list(faces.values()), axis=0)))
            state = _advance(state, packet)
        self.assertEqual(state.status, 'confirmed')
        self.assertEqual(_value(state, .625)[0], .078125)
        self.assertEqual(_value(state, 1.375)[0], .21875)
        for u, slope, intercept in ((.625, .125, 0.), (1.375, .25, -.125)):
            piece = _value(state, u)[1]
            self.assertEqual((piece.slope, piece.intercept), (slope, intercept))
            self.assertEqual(len(piece.contributors), 5)
        mean_x = math.fsum(x for x, _ in centroids)/5
        mean_y = math.fsum(y for _, y in centroids)/5
        self.assertGreater(abs(mean_y-bend(mean_x)), .01)
        self.assertInsideOriginalDomains(state)

    def test_missing_next_gap_is_not_filled_by_a_root_or_quad(self):
        state = _five(intervals=((0., .5), (1., 2.)))
        self.assertEqual(len(state.pieces), 2)
        self.assertFalse(any(piece.lower < .75 < piece.upper for piece in state.pieces))
        self.assertEqual(state.frames[-1].packet.previous_next[:, 0, 0].tolist(), [-1, -1])
        self.assertEqual(state.frames[-1].packet.previous_next[:, 1, 1].tolist(), [-1, -1])
        self.assertInsideOriginalDomains(state)

    def test_true_shared_face_endpoint_keeps_events_without_snapping_export(self):
        state = _five(intervals=((0., 1.), (1., 2.)))
        self.assertEqual(len(state.pieces), 2)
        for frame in state.frames:
            self.assertEqual(frame.packet.face_ids[0, 1], frame.packet.face_ids[1, 0])
            self.assertEqual(frame.packet.model_roots[0, 1].tobytes(),
                             frame.packet.model_roots[1, 0].tobytes())
        self.assertLess(state.pieces[0].upper, state.pieces[1].lower)
        self.assertNotEqual(state.pieces[0].segment[1], state.pieces[1].segment[0])
        self.assertInsideOriginalDomains(state)

    def test_coincident_world_roots_with_different_faces_are_not_invented_adjacency(self):
        packet = _packet(6, ((0., 1.), (1., 2.)), connect=False)
        self.assertNotEqual(packet.face_ids[0, 1], packet.face_ids[1, 0])
        state = _advance(_five(), packet)
        self.assertEqual(state.status, 'unknown')
        self.assertEqual(state.reason, 'overlap_or_nonunique_chart')
        self.assertEqual(state.pieces, ())

    def test_branch_overlap_and_duplicate_pairs_remain_whole_owner_unknown(self):
        packets = [_packet(6, ((0., 2.), (1., 3.)), connect=False)]
        args = _packet_args(6)
        for name in FIELDS:
            args[name] = np.repeat(args[name], 2, axis=0)
        packets.append(subject.FrameCurve(**args))
        for packet in packets:
            with self.subTest(packet_ids=packet.face_ids.tolist()):
                before = _packet_signature(packet)
                state = _advance(_five(), packet)
                self.assertEqual(state.status, 'unknown')
                self.assertEqual(state.reason, 'overlap_or_nonunique_chart')
                self.assertEqual(state.pieces, ())
                self.assertEqual(len(state.frames[-1].packet.face_ids), 2)
                self.assertEqual(_packet_signature(packet), before)

    def test_reverse_original_next_chain_preserves_source_direction_and_chart(self):
        state = _five(intervals=((2., 1.), (1., 0.)), intercept=.25, slope=.125)
        self.assertEqual(state.status, 'confirmed')
        self.assertEqual(state.origin, (0., 0.))
        self.assertEqual(state.tangent, (1., 0.))
        self.assertEqual(_value(state, .5)[0], .3125)
        self.assertEqual(_value(state, 1.5)[0], .4375)
        for frame in state.frames:
            self.assertEqual(frame.packet.face_ids.tolist(), [[0, 1], [1, 2]])
            self.assertEqual(frame.packet.previous_next[:, 0, 1].tolist(), [1, 2])
            self.assertEqual(frame.packet.model_roots[:, :, 0].tolist(), [[2., 1.], [1., 0.]])
            self.assertEqual(tuple(piece.occurrence for piece in frame.pieces), (1, 0))
            self.assertEqual(tuple((piece.lower_endpoint, piece.upper_endpoint)
                                  for piece in frame.pieces), ((1, 0), (1, 0)))
            first, second = frame.pieces
            self.assertEqual(frame.packet.face_ids[first.occurrence, first.upper_endpoint],
                             frame.packet.face_ids[second.occurrence, second.lower_endpoint])
            self.assertEqual(frame.packet.model_roots[first.occurrence, first.upper_endpoint].tobytes(),
                             frame.packet.model_roots[second.occurrence, second.lower_endpoint].tobytes())
            self.assertTrue(np.all(frame.packet.normals[:, :, 1] > 0.))
        self.assertInsideOriginalDomains(state)

    def test_reverse_bent_chain_and_mixed_frame_directions_use_same_common_u(self):
        def bend(u):
            return .125*u if u <= 1. else .25*u-.125
        def gradient(u):
            return .125 if u < 1. else (.25 if u > 1. else .1875)
        state = _state()
        for stamp in range(1, 6):
            intervals = ((2., 1.), (1., 0.)) if stamp % 2 else ((0., 1.), (1., 2.))
            packet = _packet(stamp, intervals, function=bend, derivative=gradient,
                             reliable=(stamp == 1))
            before = _packet_signature(packet)
            state = _advance(state, packet)
            self.assertEqual(_packet_signature(packet), before)
        self.assertEqual(state.status, 'confirmed')
        self.assertEqual(_value(state, .625)[0], .078125)
        self.assertEqual(_value(state, 1.375)[0], .21875)
        self.assertEqual(_value(state, .625)[1].contributors, (1, 2, 3, 4, 5))
        self.assertInsideOriginalDomains(state)

    def test_mixed_direction_fold_is_whole_owner_unknown_not_absolute_direction(self):
        for intervals in (((0., 2.), (2., 1.)), ((2., 0.), (0., 1.))):
            with self.subTest(intervals=intervals):
                packet = _packet(6, intervals)
                before = _packet_signature(packet)
                state = _advance(_five(), packet)
                self.assertEqual(state.status, 'unknown')
                self.assertEqual(state.reason, 'overlap_or_nonunique_chart')
                self.assertEqual(state.pieces, ())
                self.assertIs(state.frames[-1].packet, packet)
                self.assertEqual(_packet_signature(packet), before)

    def test_zero_and_nonunique_chart_preserve_original_pair(self):
        for intervals in (((1., 1.),),):
            with self.subTest(intervals=intervals):
                packet = _packet(6, intervals)
                state = _advance(_five(), packet)
                self.assertEqual(state.status, 'unknown')
                self.assertEqual(state.pieces, ())
                self.assertIs(state.frames[-1].packet, packet)
        args = _packet_args(6)
        args['model_roots'][0] = [[1., 0.], [1., .125]]
        state = _advance(_five(), subject.FrameCurve(**args))
        self.assertEqual(state.status, 'unknown')
        self.assertEqual(state.pieces, ())

    def test_both_endpoint_signed_normals_are_checked_without_average_or_flip(self):
        for endpoint in (0, 1):
            args = _packet_args(6)
            args['normals'][0, endpoint] *= -1.
            state = _advance(_five(), subject.FrameCurve(**args))
            self.assertEqual(state.reason, 'signed_direction')
            self.assertEqual(state.pieces, ())
        args = _packet_args(6, slope=.5)
        args['normals'][0, 0] = [0., 1.]
        args['normals'][0, 1] = [.25, math.sqrt(15.)/4.]
        state = _advance(_five(), subject.FrameCurve(**args))
        self.assertEqual(state.reason, 'pair_direction')
        self.assertEqual(state.pieces, ())

    def test_invalid_source_endpoint_is_retained_unknown_not_filtered(self):
        args = _packet_args(6)
        args['valid'][0, 1] = False
        args['quad_valid'][0, 1] = False
        args['model_roots'][0, 1] = [np.nan, np.nan]
        args['normals'][0, 1] = [np.nan, np.nan]
        args['quads'][0, 1] = np.nan
        packet = subject.FrameCurve(**args)
        before = _packet_signature(packet)
        state = _advance(_five(), packet)
        self.assertEqual(state.reason, 'invalid_source_support')
        self.assertEqual(state.pieces, ())
        self.assertIs(state.frames[-1].packet, packet)
        self.assertEqual(_packet_signature(packet), before)

    def test_missing_quad_evidence_is_retained_but_never_nominal_sample_or_permission(self):
        state = _state()
        for stamp in range(1, 6):
            args = _packet_args(stamp, reliable=(stamp == 1))
            args['quad_valid'][:] = False
            args['quads'][:] = np.nan
            state = _advance(state, subject.FrameCurve(**args))
        self.assertEqual(state.status, 'confirmed')
        self.assertEqual(_value(state, 1.)[0], 0.)
        self.assertTrue(all(np.isnan(frame.packet.quads).all() for frame in state.frames))
        self.assertFalse(hasattr(state.pieces[0], 'permission'))
        self.assertInsideOriginalDomains(state)

    def test_latest_five_plus_old_reliable_pair_is_bounded_and_pointwise(self):
        state = _advance(_state(), _packet(1, ((0., 1.),), reliable=True))
        for stamp in range(2, 7):
            state = _advance(state, _packet(stamp, intercept=1.))
        self.assertEqual(tuple(frame.packet.stamp_ns for frame in state.frames), (1, 2, 3, 4, 5, 6))
        self.assertEqual(_value(state, .5)[0], 5./6.)
        self.assertFalse(any(piece.lower < 1.5 < piece.upper for piece in state.pieces))
        self.assertEqual(_value(state, .5)[1].reliable_contributors, (1,))
        state = _advance(state, _packet(7, intercept=1., reliable=True))
        self.assertEqual(tuple(frame.packet.stamp_ns for frame in state.frames), (3, 4, 5, 6, 7))
        self.assertEqual(_value(state, 1.5)[0], 1.)
        self.assertInsideOriginalDomains(state)

    def test_empty_pair_frames_consume_metadata_not_votes_or_new_support(self):
        state = _advance(_five(), _packet(6, ()))
        self.assertEqual(state.source_frontier_ns, 6)
        self.assertEqual(state.frames[-1].packet.payload_bytes, 0)
        self.assertGreater(state.frames[-1].storage_units, 0)
        self.assertEqual(_value(state, 1.)[1].source_stamp_ns, 5)
        state = _advance(state, _packet(7, ()))
        self.assertEqual(state.status, 'unconfirmed')
        self.assertEqual(state.pieces, ())
        self.assertEqual(len(state.frames), 6)

    def test_owner_side_and_continuation_keys_never_merge(self):
        previous = _five()
        before = _state_signature(previous)
        for key in ({'owner': 12}, {'side': -1}, {'signature': (8, 19)}, {'signature': (-1, 19)}):
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    _advance(previous, _packet(6, **key))
                self.assertEqual(_state_signature(previous), before)

    def test_duplicate_noncausal_and_future_source_rejection_is_atomic(self):
        previous = _five()
        before = _state_signature(previous)
        for stamp, available in ((5, 5), (4, 6), (6, 5)):
            packet = _packet(stamp)
            packet_before = _packet_signature(packet)
            with self.assertRaises(ValueError):
                subject.advance_curve(previous, packet, available_before_ns=available)
            self.assertEqual(_state_signature(previous), before)
            self.assertEqual(_packet_signature(packet), packet_before)

    def test_complete_pair_payload_metadata_and_derived_capacity_boundary(self):
        packet = _packet(1)
        self.assertEqual(packet.storage_units, 87)  # 9header+37shape/array+41scalar.
        self.assertEqual(packet.payload_bytes, 279)
        state = _five()
        self.assertEqual(state.storage_units, 20+5*(4+87+9)+(21+5+1))
        self.assertEqual(state.payload_bytes, 5*279)
        exact = _five(capacity=state.storage_units)
        self.assertEqual(exact.storage_units, exact.max_storage_units)
        previous = _state(capacity=state.storage_units-1)
        for stamp in range(1, 5):
            previous = _advance(previous, _packet(stamp, reliable=(stamp == 1)))
        before = _state_signature(previous)
        with self.assertRaises(ValueError):
            _advance(previous, _packet(5))
        self.assertEqual(_state_signature(previous), before)

    def test_raw_budget_rejection_never_trims_old_or_current_pairs(self):
        previous = _advance(_state(capacity=150), _packet(1, reliable=True))
        before = _state_signature(previous)
        current = _packet(2)
        packet_before = _packet_signature(current)
        with self.assertRaises(ValueError):
            _advance(previous, current)
        self.assertEqual(_state_signature(previous), before)
        self.assertEqual(_packet_signature(current), packet_before)

    def test_fields_own_independent_readonly_bytes_not_full_observation_slice(self):
        args = _packet_args(1)
        packed = np.ones((1000, 2, 2), dtype=np.float64)
        packed[17] = args['model_roots'][0]
        args['model_roots'] = packed[17:18]
        packet = subject.FrameCurve(**args)
        before = _packet_signature(packet)
        for name in FIELDS:
            value = getattr(packet, name)
            self.assertFalse(value.flags.writeable)
            self.assertFalse(np.shares_memory(value, args[name]))
            base = value
            while isinstance(base, np.ndarray):
                base = base.base
            self.assertIsInstance(base, bytes)
            self.assertEqual(len(base), value.nbytes)
            with self.assertRaises(ValueError):
                value.setflags(write=True)
        packed[:] = 99.
        self.assertEqual(_packet_signature(packet), before)
        with self.assertRaises(FrozenInstanceError):
            packet.stamp_ns = 7

    def test_raw_pair_order_and_signed_zero_are_preserved(self):
        args = _packet_args(1, ((0., 1.), (1., 2.)))
        for name in FIELDS:
            args[name] = args[name][::-1]
        for name in ('model_roots', 'normals', 'quads'):
            args[name][args[name] == 0.] = -0.
        packet = subject.FrameCurve(**args)
        for name in FIELDS:
            self.assertEqual(_signature(getattr(packet, name))[:3], _signature(args[name])[:3])
        state = _advance(_state(), packet)
        self.assertEqual(state.frames[0].status, 'known')
        self.assertEqual(tuple(piece.occurrence for piece in state.frames[0].pieces), (1, 0))
        self.assertEqual(packet.face_ids.tolist(), [[1, 2], [0, 1]])

    def test_creation_chart_never_follows_visibility_or_mean_root(self):
        state = _five(origin=(10., -8.), intercept=.25)
        self.assertEqual(state.origin, (10., -8.))
        self.assertEqual(state.tangent, (1., 0.))
        self.assertEqual(_value(state, 1.)[0], .25)
        self.assertTrue(all(point[1] == -7.75 for point in state.pieces[0].segment))
        self.assertInsideOriginalDomains(state)

    def test_nonaxis_inverse_and_nonexact_unit_chart_have_finite_certified_endpoints(self):
        state = _five(origin=(10., -8.), tangent=(.6, .8), intercept=.25)
        self.assertEqual(state.status, 'confirmed')
        self.assertEqual(state.tangent, (.6, .8))
        self.assertInsideOriginalDomains(state)
        t = float(np.nextafter(1., np.inf))
        state = _five(tangent=(t, 0.))
        self.assertNotEqual(t*t, 1.)
        self.assertEqual(state.pieces[0].world_derivative, (t/(t*t), 0.))
        self.assertEqual(state.pieces[0].normal, (0., 1.))
        self.assertInsideOriginalDomains(state)

    def test_strict_fp_settings_are_restored_for_known_unknown_and_placeholders(self):
        with np.errstate(all='raise'):
            state = _five(origin=(10., -8.))
            self.assertEqual(state.status, 'confirmed')
            state = _advance(state, _packet(6, ((0., 2.), (2., 1.)), origin=(10., -8.)))
            self.assertEqual(state.status, 'unknown')
            args = _packet_args(7)
            args['quad_valid'][:] = False
            args['quads'][:] = np.nan
            _advance(state, subject.FrameCurve(**args))
            self.assertEqual(np.geterr(), dict(divide='raise', over='raise', under='raise', invalid='raise'))

    def test_valid_endpoint_nonfinite_geometry_and_invalid_native_dtypes_reject(self):
        for name in ('model_roots', 'normals', 'quads'):
            for bad in (np.nan, np.inf, -np.inf):
                args = _packet_args(6)
                args[name].flat[0] = bad
                before = {key: _signature(args[key]) for key in FIELDS}
                with self.assertRaises(ValueError):
                    subject.FrameCurve(**args)
                self.assertEqual({key: _signature(args[key]) for key in FIELDS}, before)
        for name, dtype in (('roles', np.bool_), ('model_roots', np.float32),
                            ('face_ids', np.float64), ('normals', np.complex128),
                            ('quad_valid', np.int64), ('reliable', np.float64)):
            args = _packet_args(6)
            args[name] = args[name].astype(dtype)
            with self.assertRaises(ValueError):
                subject.FrameCurve(**args)
        args = _packet_args(6)
        args['model_roots'] = args['model_roots'].astype('>f8' if np.little_endian else '<f8')
        with self.assertRaises(ValueError):
            subject.FrameCurve(**args)

    def test_malformed_dual_shapes_flags_ids_and_own_fragment_source_reject(self):
        for name, shape in (('roles', (1, 1)), ('face_ids', (1,)),
                            ('previous_next', (1, 2)), ('contour_ids', (1,)),
                            ('endpoint_owners', (1,)), ('model_roots', (1, 2)),
                            ('normals', (1, 2)), ('quads', (1, 4, 2)),
                            ('quad_valid', (1,)), ('valid', (1,)), ('reliable', (1,))):
            args = _packet_args(6)
            args[name] = np.zeros(shape, dtype=args[name].dtype)
            with self.assertRaises(ValueError):
                subject.FrameCurve(**args)
        for name in ('previous_next', 'contour_ids', 'endpoint_owners'):
            args = _packet_args(6)
            args[name].flat[0] = -2
            with self.assertRaises(ValueError):
                subject.FrameCurve(**args)
        args = _packet_args(6, reliable=True)
        args['valid'][0, 1] = False
        with self.assertRaises(ValueError):
            subject.FrameCurve(**args)
        args = _packet_args(6)
        args['source_kind'] = 'own_position_fragments'
        with self.assertRaises(ValueError):
            subject.FrameCurve(**args)

    def test_boolean_and_invalid_scalar_configuration_rejects_atomically(self):
        for changes in ({'owner': True}, {'side': True}, {'side': 0},
                        {'signature': (True, 19)}, {'signature': (7,)},
                        {'origin': (True, 0.)}, {'origin': (np.nan, 0.)},
                        {'tangent': (2., 0.)}, {'capacity': True},
                        {'capacity': 19}, {'capacity': 200001}):
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    _state(**changes)
        for name in ('owner_id', 'signed_side', 'stamp_ns'):
            args = _packet_args(6)
            args[name] = True
            with self.assertRaises(ValueError):
                subject.FrameCurve(**args)
        for available in (True, np.bool_(True), 0, -1, 1.5):
            with self.assertRaises(ValueError):
                subject.advance_curve(_state(), _packet(1), available_before_ns=available)
        for cosine in (True, 0., -1., 1.1, np.inf):
            with self.assertRaises(ValueError):
                subject.create_curve_state(11, 1, (7, 19), (0., 0.), (1., 0.),
                    normal_cosine=cosine, max_storage_units=200000)

    def test_rejected_types_cannot_mutate_or_forge_a_prepared_curve_state(self):
        state = _five()
        before = _state_signature(state)
        with self.assertRaises(ValueError):
            subject.advance_curve(state, object(), available_before_ns=6)
        with self.assertRaises(ValueError):
            subject.advance_curve(object(), _packet(6), available_before_ns=6)
        self.assertEqual(_state_signature(state), before)
        with self.assertRaises(FrozenInstanceError):
            state.origin = (3., 4.)


if __name__ == '__main__':
    unittest.main()
