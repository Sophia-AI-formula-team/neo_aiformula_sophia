"""Independent small deterministic tests; no core, estimator, recordings or Unity."""
from dataclasses import FrozenInstanceError
from fractions import Fraction as F
import ast
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np

from lane_mapping_lya_reference import finite_support
from lane_mapping_lya_reference.finite_support import (
    DomainState, Interval, advance_domain, certified_segments, projection_bounds,
)

COSINE = math.cos(math.pi/6.)


def observation(intervals, *, normal=(0., 1.), y=0.):
    segments=np.asarray([[[lo,y],[hi,y]] for lo,hi in intervals],dtype=np.float64).reshape(-1,2,2)
    normals=np.tile(np.asarray(normal,dtype=np.float64),(len(segments),2,1))
    return segments,normals


def advance(state, intervals=((0.,1.),), *, stamp=None, reliable=True, **options):
    segments,normals=observation(intervals)
    reliability=(np.full(len(segments),reliable,dtype=bool)
                 if isinstance(reliable,(bool,np.bool_)) else reliable)
    values=dict(origin=(0.,0.),current_frame_normal=(0.,1.),segments=segments,normals=normals,
        reliable=reliability,stamp_ns=1 if state is None else state.source_stamp_ns+1,
        minimum_frame_votes=5,minimum_reliable_votes=1,candidate_ttl_ns=0,
        normal_cosine=COSINE,max_intervals=100)
    if stamp is not None:values['stamp_ns']=stamp
    values.update(options)
    return advance_domain(state,**values)


def votes_at(state, position):
    items=[piece for piece in state.intervals if piece.lower<=position<=piece.upper]
    # Never add overlapping endpoint certificates together.
    return (max((p.frame_votes for p in items),default=0),
            max((p.reliable_votes for p in items),default=0))


def confirmed(rounds=5):
    state=None
    for _ in range(rounds):state=advance(state)
    return state


def exact_projection(point,origin,tangent):
    return sum((F(float(p))-F(float(o)))*F(float(t)) for p,o,t in zip(point,origin,tangent))


class CausalVotingTests(unittest.TestCase):
    def test_one_frame_many_overlapping_segments_is_one_vote(self):
        state=advance(None,((0.,2.),(.2,1.7),(.4,1.5))*7)
        self.assertEqual(votes_at(state,1.),(1,1))

    def test_reliable_subunion_does_not_certify_unreliable_tail(self):
        segments,normals=observation(((0.,3.),(1.,2.)))
        state=advance(None,segments=segments,normals=normals,reliable=np.array([False,True]))
        self.assertEqual(votes_at(state,.5),(1,0));self.assertEqual(votes_at(state,1.5),(1,1))
        self.assertEqual(votes_at(state,2.5),(1,0))

    def test_five_distinct_frames_and_one_reliable_frame_required(self):
        state=None
        for _ in range(4):state=advance(state,reliable=False)
        self.assertEqual(len(certified_segments(state,(0.,0.),(0.,1.),COSINE,5,1)[0]),0)
        state=advance(state,reliable=True)
        self.assertEqual(votes_at(state,.5),(5,1))
        self.assertEqual(len(certified_segments(state,(0.,0.),(0.,1.),COSINE,5,1)[0]),1)

    def test_five_unreliable_frames_do_not_confirm(self):
        state=None
        for _ in range(7):state=advance(state,reliable=False)
        self.assertEqual(votes_at(state,.5),(5,0))
        self.assertEqual(len(certified_segments(state,(0.,0.),(0.,1.),COSINE,5,1)[0]),0)

    def test_votes_saturate_but_last_actual_stamp_updates(self):
        state=confirmed(9)
        self.assertEqual(votes_at(state,.5),(5,1))
        self.assertEqual(state.intervals[0].last_stamp_ns,9)

    def test_reliable_counter_can_be_more_than_one(self):
        state=None
        for _ in range(7):state=advance(state,minimum_reliable_votes=3)
        self.assertEqual(votes_at(state,.5),(5,3))

    def test_gap_is_not_filled_by_minmax_hull(self):
        state=None
        for _ in range(5):state=advance(state,((0.,1.),(2.,3.)))
        self.assertEqual(votes_at(state,1.5),(0,0))
        self.assertEqual(len(certified_segments(state,(0.,0.),(0.,1.),COSINE,5,1)[0]),2)

    def test_density_does_not_retroactively_increase_votes(self):
        state=advance(None,((0.,3.),),reliable=False)
        state=advance(state,tuple((i/100.,(i+1)/100.) for i in range(300)),max_intervals=2000)
        self.assertEqual(votes_at(state,1.234),(2,1))

    def test_partial_observation_preserves_old_stamp(self):
        state=advance(None,((0.,3.),),stamp=10)
        state=advance(state,((1.,2.),),stamp=20)
        self.assertEqual(next(p for p in state.intervals if p.lower<.5<p.upper).last_stamp_ns,10)
        self.assertEqual(next(p for p in state.intervals if p.lower<1.5<p.upper).last_stamp_ns,20)

    def test_merge_requires_same_counts_and_last_stamp(self):
        state=DomainState((0.,0.),(1.,0.),(Interval(0.,1.,1,1,1),Interval(1.,2.,1,1,2)),2)
        next_state=advance(state,(),stamp=3)
        self.assertEqual(next_state.intervals,state.intervals)

    def test_merge_adjacent_equal_metadata(self):
        state=DomainState((0.,0.),(1.,0.),(Interval(0.,1.,1,1,1),Interval(1.,2.,1,1,1)),1)
        next_state=advance(state,(),stamp=2)
        self.assertEqual(next_state.intervals,(Interval(0.,2.,1,1,1),))

    def test_overlay_matches_independent_per_frame_membership(self):
        observations=[(((0.,2.),(1.,3.)),False),(((.5,2.5),),True),(((1.,4.),),False),
                      (((0.,4.),),False),(((0.,4.),),False),(((2.,5.),),True)]
        state=None
        for number,(segments,reliable) in enumerate(observations,1):
            state=advance(state,segments,reliable=reliable)
            for x in (.25,.75,1.25,1.75,2.25,2.75,3.25,3.75,4.25):
                count=sum(any(a<x<b for a,b in s) for s,_ in observations[:number])
                trusted=sum(r and any(a<x<b for a,b in s) for s,r in observations[:number])
                self.assertEqual(votes_at(state,x),(min(5,count),min(1,trusted)))


class ExpiryAndStateTests(unittest.TestCase):
    def test_ttl_expiry_only_after_strict_deadline(self):
        state=advance(None,stamp=10)
        self.assertEqual(votes_at(advance(state,(),stamp=20,candidate_ttl_ns=10),.5),(1,1))
        self.assertEqual(advance(state,(),stamp=21,candidate_ttl_ns=10).intervals,())

    def test_confirmed_never_expires(self):
        state=confirmed()
        self.assertEqual(advance(state,(),stamp=1000,candidate_ttl_ns=1).intervals,state.intervals)

    def test_five_votes_without_reliability_can_expire(self):
        state=None
        for _ in range(5):state=advance(state,reliable=False)
        self.assertEqual(advance(state,(),stamp=100,candidate_ttl_ns=2).intervals,())

    def test_expired_support_does_not_retroactively_vote(self):
        state=advance(None,stamp=1)
        state=advance(state,stamp=100,candidate_ttl_ns=2)
        self.assertEqual(votes_at(state,.5),(1,1))

    def test_zero_ttl_disables_expiry(self):
        state=advance(None)
        self.assertEqual(advance(state,(),stamp=1000,candidate_ttl_ns=0).intervals,state.intervals)

    def test_duplicate_and_reversed_stamps_fail_without_mutation(self):
        state=advance(None,stamp=5);before=repr(state)
        for stamp in (4,5):
            with self.assertRaises(ValueError):advance(state,stamp=stamp)
        self.assertEqual(repr(state),before)

    def test_source_frontier_advances_on_empty_frame_without_changing_last_seen(self):
        state=advance(None,stamp=1);next_state=advance(state,(),stamp=2)
        self.assertEqual(next_state.source_stamp_ns,2);self.assertEqual(next_state.intervals,state.intervals)

    def test_creation_chart_does_not_follow_current_mean(self):
        state=advance(None)
        next_state=advance(state,origin=(4.,6.),current_frame_normal=(.1,math.sqrt(.99)))
        self.assertEqual(next_state.origin,state.origin);self.assertEqual(next_state.tangent,state.tangent)

    def test_dataclass_copies_mutable_origin_and_interval_container(self):
        origin=[0.,0.];pieces=[Interval(0.,1.,1,1,1)]
        state=DomainState(origin,[1.,0.],pieces,1);origin[0]=99;pieces.clear()
        self.assertEqual(state.origin,(0.,0.));self.assertEqual(len(state.intervals),1)
        with self.assertRaises(FrozenInstanceError):state.source_stamp_ns=2
        with self.assertRaises(FrozenInstanceError):state.intervals[0].upper=7

    def test_input_mutation_after_return_cannot_change_state(self):
        segments,normals=observation(((0.,1.),));reliable=np.ones(1,dtype=bool)
        state=advance(None,segments=segments,normals=normals,reliable=reliable);before=repr(state)
        segments[:]=8;normals[:]=0;reliable[:]=False
        self.assertEqual(repr(state),before)

    def test_capacity_overflow_is_atomic_not_truncated(self):
        state=advance(None,((0.,1.),));before=repr(state)
        with self.assertRaisesRegex(ValueError,'capacity'):
            advance(state,((2.,3.),),max_intervals=1)
        self.assertEqual(repr(state),before)

    def test_rejects_overlapping_unsorted_and_future_piece_state(self):
        for intervals,stamp in [((Interval(1.,2.,1,0,1),Interval(0.,1.,1,0,1)),1),
            ((Interval(0.,2.,1,0,1),Interval(1.,3.,1,0,1)),1),((Interval(0.,1.,1,0,2),),1)]:
            with self.assertRaises(ValueError):DomainState((0.,0.),(1.,0.),intervals,stamp)


class ProjectionAndExportTests(unittest.TestCase):
    def test_rational_oracle_enclosed_for_cancellation_and_subnormal(self):
        inputs=[((1.e7,1.e7),(1.e7-1.e-7,1.e7-2.e-7),(.6,-.8)),
                ((float(np.nextafter(0.,1.)),-float(np.nextafter(0.,1.))),(0.,0.),(.6,.8)),
                ((-0.,0.),(0.,-0.),(0.,-1.)),((1.,1.),(-1.,-1.),(.6,-.8))]
        rng=np.random.default_rng(581)
        for _ in range(50):
            p=rng.uniform(-1.e7,1.e7,2);origin=rng.uniform(-1.e7,1.e7,2);angle=rng.uniform(-math.pi,math.pi)
            inputs.append((p,origin,(math.cos(angle),math.sin(angle))))
        for p,o,t in inputs:
            lo,hi=projection_bounds(p,o,t);exact=exact_projection(p,o,t)
            self.assertLessEqual(F(lo),exact);self.assertGreaterEqual(F(hi),exact)

    def test_published_projection_interval_is_inward_by_rational_oracle(self):
        angle=.713;normal=(math.sin(angle),math.cos(angle));t=(normal[1],-normal[0]);origin=(123.,-57.)
        segment=np.array([[[123.1,-56.5],[125.3,-59.4]]])
        state=advance(None,origin=origin,current_frame_normal=normal,segments=segment,
            normals=np.tile(normal,(1,2,1)))
        a,b=sorted(exact_projection(p,origin,t) for p in segment[0])
        for piece in state.intervals:self.assertGreaterEqual(F(piece.lower),a);self.assertLessEqual(F(piece.upper),b)

    def test_reversed_segment_has_identical_support(self):
        a=advance(None);segments,normals=observation(((0.,1.),))
        b=advance(None,segments=segments[:,::-1],normals=normals[:,::-1])
        self.assertEqual(a,b)

    def test_zero_extent_does_not_create_support(self):
        self.assertEqual(advance(None,((1.,1.),)).intervals,())

    def test_uncertain_one_ulp_extent_is_not_extended(self):
        state=advance(None,((1.,float(np.nextafter(1.,math.inf))),))
        self.assertEqual(state.intervals,())

    def test_both_endpoint_normals_must_match_chart(self):
        state=advance(None);segments,normals=observation(((0.,1.),));normals[0,1]=(1.,0.)
        later=advance(state,segments=segments,normals=normals)
        self.assertEqual(later.intervals,state.intervals)

    def test_opposite_signed_normal_not_abs_compatible(self):
        state=advance(None);segments,normals=observation(((0.,1.),),normal=(0.,-1.))
        self.assertEqual(advance(state,segments=segments,normals=normals).intervals,state.intervals)

    def test_export_uses_old_chart_not_moving_mean_relative_interval(self):
        state=confirmed();segments,votes,reliable,stamps=certified_segments(state,(4.,2.),(0.,1.),COSINE,5,1)
        self.assertEqual(len(segments),1)
        self.assertTrue(np.all(segments[:,:,0]>=0.));self.assertTrue(np.all(segments[:,:,0]<=1.))
        np.testing.assert_array_equal(segments[:,:,1],[[2.,2.]])
        np.testing.assert_array_equal(votes,[5]);np.testing.assert_array_equal(reliable,[1]);np.testing.assert_array_equal(stamps,[5])

    def test_export_rotated_current_line_keeps_creation_coordinates(self):
        state=confirmed();normal=(math.sin(.2),math.cos(.2))
        segments,*_=certified_segments(state,(3.,2.),normal,COSINE,5,1)
        for endpoint in segments[0]:
            s=exact_projection(endpoint,state.origin,state.tangent)
            self.assertGreaterEqual(s,F(state.intervals[0].lower));self.assertLessEqual(s,F(state.intervals[0].upper))
            self.assertAlmostEqual(float(np.dot(endpoint-(3.,2.),normal)),0.,places=13)

    def test_export_buffers_are_bytes_backed_immutable(self):
        result=certified_segments(confirmed(),(0.,0.),(0.,1.),COSINE,5,1)
        for a in result:
            self.assertFalse(a.flags.writeable)
            with self.assertRaises(ValueError):a.setflags(write=True)

    def test_export_reversed_endpoints_fail_closed(self):
        # Inject only the numerical endpoint solver result to ensure even
        # distinct endpoints cannot certify a backwards interval.
        with patch('lane_mapping_lya_reference.finite_support._export_endpoint',side_effect=[(.8,0.),(.2,0.)]):
            with self.assertRaisesRegex(ValueError,'reversed'):
                certified_segments(confirmed(),(0.,0.),(0.,1.),COSINE,5,1)

    def test_unrepresentable_short_interval_with_far_mean_fails_closed(self):
        state=DomainState((0.,0.),(1.,0.),(Interval(0.,1.e-20,5,1,5),),5)
        with self.assertRaises(ValueError):
            certified_segments(state,(1.e7,0.),(0.,1.),COSINE,5,1)

    def test_strict_numpy_underflow_still_encloses_subnormal_projection(self):
        tiny=float(np.nextafter(0.,1.))
        with np.errstate(all='raise'):
            lo,hi=projection_bounds((tiny,-tiny),(0.,0.),(.6,.8))
        exact=exact_projection((tiny,-tiny),(0.,0.),(.6,.8))
        self.assertLessEqual(F(lo),exact);self.assertGreaterEqual(F(hi),exact)

    def test_export_angle_rejection_is_explicit_empty_geometry(self):
        self.assertEqual(certified_segments(confirmed(),(0.,0.),(1.,0.),COSINE,5,1)[0].shape,(0,2,2))

    def test_rigid_transform_geometry_equivalence(self):
        angle=.37;rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        translation=np.array([10.,-7.]);segments,normals=observation(((0.,1.),))
        transformed=segments@rotation.T+translation;directions=normals@rotation.T;state=None
        for _ in range(5):state=advance(state,origin=translation,current_frame_normal=rotation@(0.,1.),
            segments=transformed,normals=directions)
        exported=certified_segments(state,translation,rotation@(0.,1.),COSINE,5,1)[0]
        np.testing.assert_allclose(exported,transformed,rtol=0,atol=5.e-14)


class ValidationTests(unittest.TestCase):
    def test_rejects_bad_numbers_and_huge_int_origin(self):
        for value in (float('nan'),float('inf'),True,10**1000):
            with self.assertRaises(ValueError):advance(None,origin=(value,0.))

    def test_rejects_bad_stamp_and_counts(self):
        for option,value in [('stamp_ns',True),('stamp_ns',0),('stamp_ns',1.5),('stamp_ns',2**63),
            ('minimum_frame_votes',0),('minimum_reliable_votes',0),('minimum_reliable_votes',6),
            ('max_intervals',0),('candidate_ttl_ns',-1)]:
            with self.assertRaises(ValueError):advance(None,**{option:value})

    def test_rejects_array_shape_and_bad_dtype(self):
        cases=[dict(segments=np.zeros((2,2))),dict(normals=np.zeros((1,2))),
            dict(reliable=np.array([1])),dict(segments=np.array([[['x','y'],['z','w']]])),
            dict(segments=np.zeros((1,2,2),complex)),dict(normals=np.full((1,2,2),float('nan')))]
        for values in cases:
            with self.assertRaises(ValueError):advance(None,**values)

    def test_rejects_nonunit_normals_even_for_empty_or_no_geometry(self):
        with self.assertRaises(ValueError):advance(None,(),current_frame_normal=(0.,2.))
        with self.assertRaises(ValueError):advance(None,((0.,0.),),normals=np.zeros((1,2,2)))

    def test_rejects_invalid_export_normal_and_point(self):
        state=confirmed()
        for point,normal in [((0.,float('nan')),(0.,1.)),((0.,0.),(0.,0.))]:
            with self.assertRaises(ValueError):certified_segments(state,point,normal,COSINE,5,1)

    def test_rejects_threshold_downshift_versus_saturated_state(self):
        with self.assertRaises(ValueError):advance(confirmed(),minimum_frame_votes=3)

    def test_signed_zero_and_empty_outputs_are_valid(self):
        state=advance(None,(),origin=(-0.,0.));out=certified_segments(state,(0.,0.),(0.,1.),COSINE,5,1)
        self.assertEqual(out[0].shape,(0,2,2));self.assertEqual(out[1].dtype,np.dtype('int64'))

    def test_foxy_python38_syntax_and_api_surface(self):
        paths=(Path(finite_support.__file__), Path(__file__))
        for path in paths:
            tree=ast.parse(path.read_text(encoding='utf8'),feature_version=(3,8))
            for node in ast.walk(tree):
                if isinstance(node,ast.Call):
                    self.assertFalse(any(keyword.arg=='slots' for keyword in node.keywords))
                if isinstance(node,ast.Attribute) and isinstance(node.value,ast.Name):
                    self.assertNotEqual((node.value.id,node.attr),('math','nextafter'))
                    self.assertNotEqual((node.value.id,node.attr),('math','ulp'))


class ConfirmedExportRegressionTests(unittest.TestCase):
    def test_adjacent_confirmed_export_uses_conservative_counts_latest_frontier(self):
        state=DomainState((0.,0.),(1.,0.),
            (Interval(0.,1.,7,2,10),Interval(1.,2.,5,1,20)),20)
        before=repr(state)
        segments,votes,reliable,stamps=certified_segments(state,(.2,0.),(0.,1.),COSINE,5,1)
        self.assertEqual(len(segments),1)
        np.testing.assert_array_equal(votes,[5]);np.testing.assert_array_equal(reliable,[1])
        np.testing.assert_array_equal(stamps,[20])
        self.assertEqual(repr(state),before)
        self.assertEqual([piece.last_stamp_ns for piece in state.intervals],[10,20])
        self.assertTrue(segments[0,0,0]>=0. and segments[0,1,0]<=2.)

    def test_confirmed_export_cannot_merge_across_real_gap(self):
        state=DomainState((0.,0.),(1.,0.),
            (Interval(0.,1.,5,1,10),Interval(1.25,2.,5,1,20)),20)
        segments,*_=certified_segments(state,(.2,0.),(0.,1.),COSINE,5,1)
        self.assertEqual(len(segments),2)
        self.assertLessEqual(segments[0,1,0],1.)
        self.assertGreaterEqual(segments[1,0,0],1.25)

    def test_unconfirmed_middle_piece_is_not_a_confirmed_union_bridge(self):
        state=DomainState((0.,0.),(1.,0.),(Interval(0.,1.,5,1,10),
            Interval(1.,2.,4,1,20),Interval(2.,3.,5,1,20)),20)
        segments,*_=certified_segments(state,(0.,0.),(0.,1.),COSINE,5,1)
        self.assertEqual(len(segments),2)
        self.assertFalse(any(min(s[:,0])<=1.5<=max(s[:,0]) for s in segments))

    def test_adjacent_confirmed_sliver_is_not_exported_as_separate_endpoint_pair(self):
        lower=.009999999999999997;upper=.010000000000000004
        state=DomainState((0.,.01),(1.,0.),
            (Interval(-.03,lower,5,1,20),Interval(lower,upper,5,1,10),
             Interval(upper,.03,5,1,20)),20)
        segments,votes,reliable,stamps=certified_segments(state,(.001666666666666667,.01),
                                                        (0.,1.),COSINE,5,1)
        self.assertEqual(len(segments),1)
        self.assertTrue(segments[0,0,0]<0.<segments[0,1,0])
        np.testing.assert_array_equal(stamps,[20])

    def test_world_resolution_progress_preserves_tiny_real_chart_gap(self):
        first=Interval(-.031250000000000014,-2.7755575615628926e-17,5,1,5)
        second=Interval(-2.7755575615628907e-17,.031249999999999965,5,1,5)
        state=DomainState((.20000000000000004,.01),(1.,0.),(first,second),5)
        segments,*_=certified_segments(state,state.origin,(0.,1.),COSINE,5,1)
        self.assertEqual(len(segments),2)
        for segment,piece in zip(segments,state.intervals):
            self.assertGreater(segment[1,0],segment[0,0])
            for endpoint in segment:
                lo,hi=projection_bounds(endpoint,state.origin,state.tangent)
                self.assertGreaterEqual(lo,piece.lower);self.assertLessEqual(hi,piece.upper)
        self.assertLess(segments[0,1,0],segments[1,0,0])

    def test_isolated_tiny_certified_piece_still_fails_without_padding(self):
        state=DomainState((0.,.01),(1.,0.),
            (Interval(.009999999999999997,.010000000000000004,5,1,5),),5)
        with self.assertRaises(ValueError):
            certified_segments(state,(.001666666666666667,.01),(0.,1.),COSINE,5,1)


if __name__=='__main__':unittest.main()
