"""Independent finite-ray, signed-side and complete-denominator regressions."""
from dataclasses import replace
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lane_mapping_lya_reference.boundary_localization import (
    assign_foreground_mass, _scan_representatives, FrozenBoundaryIndex,
    BoundaryLaneLocalizer)
from lane_mapping_lya_reference.mask_localization import _configuration
from lane_mapping_lya_reference.paint_geometry import BoundaryObservation, BoundaryReference


def observation(segments, normals, *, reliable=None, valid=None, contour_ids=None):
    segments = np.asarray(segments, dtype=np.float64).reshape(-1,2,2)
    n = len(segments)
    normals = np.asarray(normals,dtype=np.float64).reshape(n,2)
    valid = np.ones(n,dtype=bool) if valid is None else np.asarray(valid,dtype=bool)
    reliable = valid.copy() if reliable is None else np.asarray(reliable,dtype=bool)
    return BoundaryObservation(points=segments.mean(axis=1),normals=normals,segments=segments,
        sensitivity=np.full(n,.01),metric_valid=np.ones(n,dtype=bool),stable=np.ones(n,dtype=bool),
        normal_valid=valid,valid=valid,reliable=reliable,source_stamp_ns=100,
        pixel_segments=segments.copy(),white_pixels=np.zeros((n,2),dtype=np.int64),
        black_pixels=np.ones((n,2),dtype=np.int64),previous=np.full(n,-1,dtype=np.int64),
        next=np.full(n,-1,dtype=np.int64),contour_ids=np.arange(n,dtype=np.int64) if contour_ids is None
        else np.asarray(contour_ids,dtype=np.int64),reason_codes=np.zeros(n,dtype=np.int64),
        normal_support_count=np.full(n,6,dtype=np.int64),normal_radius=np.full(n,.45))


def strip(*, upper=.2, half_length=1.):
    return observation([[[-half_length,-.2],[half_length,-.2]],
                        [[-half_length,upper],[half_length,upper]]],[[0,-1],[0,1]])


def reference(points, normals, segments, owners, reliable_votes=None):
    n=len(points)
    return BoundaryReference(np.asarray(points,dtype=np.float64).reshape(n,2),
        np.asarray(normals,dtype=np.float64).reshape(n,2),
        np.asarray(segments,dtype=np.float64).reshape(-1,2,2),np.asarray(owners,dtype=np.int64),
        np.full(n,5,dtype=np.int64),np.ones(n,dtype=np.int64) if reliable_votes is None else
        np.asarray(reliable_votes,dtype=np.int64),np.arange(n,dtype=np.int64),99 if n else None)


def assign(obs, points=((0.,0.),), normals=((0.,1.),), weights=(1.,)):
    return assign_foreground_mass(np.asarray(points,dtype=np.float64),
        np.asarray(normals,dtype=np.float64),np.asarray(weights,dtype=np.float64),obs,_configuration(None))


def assert_mass_conserved(mass, metrics, denominator):
    assert np.sum(mass)+metrics['boundary_unassigned_mass'] == pytest.approx(denominator,abs=1e-14)
    assert metrics['boundary_original_foreground_mass'] == denominator
    assert (metrics['boundary_missing_mass']+metrics['boundary_uncertified_mass']+
            metrics['boundary_ambiguous_mass']) == pytest.approx(metrics['boundary_unassigned_mass'],abs=1e-14)


def test_two_actual_sides_share_original_mass_exactly():
    mass,metrics=assign(strip(),weights=(3.,))
    np.testing.assert_array_equal(mass,[1.5,1.5])
    assert_mass_conserved(mass,metrics,3.)


def test_missing_side_does_not_donate_to_observed_side_or_reduce_D():
    obs=observation([[[-1,-.2],[1,-.2]]],[[0,-1]])
    mass,metrics=assign(obs)
    np.testing.assert_array_equal(mass,[.5])
    assert metrics['boundary_missing_mass']==.5
    assert_mass_conserved(mass,metrics,1.)


@pytest.mark.parametrize('kind',['unreliable','invalid_direction','opposite_signed_direction'])
def test_nearer_uncertified_transition_cannot_be_skipped(kind):
    normals=np.array([[0.,-1.],[0.,1.],[0.,1.]])
    reliable=np.ones(3,dtype=bool); valid=np.ones(3,dtype=bool)
    if kind=='unreliable': reliable[1]=False
    if kind=='invalid_direction': valid[1]=False; reliable[1]=False; normals[1]=0
    if kind=='opposite_signed_direction': normals[1]=[0.,-1.]
    obs=observation([[[-1,-.2],[1,-.2]],[[-1,.1],[1,.1]],[[-1,.3],[1,.3]]],
                    normals,reliable=reliable,valid=valid)
    mass,metrics=assign(obs)
    np.testing.assert_array_equal(mass,[.5,0,0])
    assert metrics['boundary_uncertified_mass']==.5
    assert_mass_conserved(mass,metrics,1.)


def test_reversing_foreground_PCA_normal_only_swaps_plus_minus_not_assignment():
    a,ma=assign(strip())
    b,mb=assign(strip(),normals=((0.,-1.),))
    np.testing.assert_array_equal(a,b)
    assert ma==mb


def test_ray_does_not_hit_infinite_extension_of_short_measured_face():
    mass,metrics=assign(strip(half_length=.1),points=((.2,0.),))
    np.testing.assert_array_equal(mass,[0.,0.])
    assert metrics['boundary_missing_mass']==1.
    assert_mass_conserved(mass,metrics,1.)


def test_shared_vertex_different_contours_is_ambiguous_not_arbitrary():
    obs=observation([[[-1,-.2],[1,-.2]],[[-1,.2],[0,.2]],[[0,.2],[1,.2]]],
                    [[0,-1],[0,1],[0,1]],contour_ids=[0,1,2])
    mass,metrics=assign(obs)
    np.testing.assert_array_equal(mass,[.5,0,0])
    assert metrics['boundary_ambiguous_mass']==.5
    assert_mass_conserved(mass,metrics,1.)


def test_same_contour_compatible_vertex_receives_only_one_half_mass():
    obs=observation([[[-1,-.2],[1,-.2]],[[-1,.2],[0,.2]],[[0,.2],[1,.2]]],
                    [[0,-1],[0,1],[0,1]],contour_ids=[0,1,1])
    mass,metrics=assign(obs)
    assert mass[0]==.5 and mass[1:].sum()==.5
    assert_mass_conserved(mass,metrics,1.)


def test_nextafter_beyond_segment_endpoint_is_not_physical_padding():
    exact,_=assign(strip(half_length=.25),points=((.25,0.),))
    # Two ULPs survive addition of the other endpoint in the cross product;
    # one ULP can round back to exactly1 in IEEE arithmetic, not an epsilon.
    outside=np.nextafter(np.nextafter(.25,np.inf),np.inf)
    beyond,metrics=assign(strip(half_length=.25),points=((outside,0.),))
    assert exact.sum()==1.
    assert beyond.sum()==0.
    assert_mass_conserved(beyond,metrics,1.)


def test_representatives_preserve_assigned_mass_and_actual_input_points():
    obs=observation([[[0,.2],[.04,.2]],[[.04,.2],[.08,.2]],
                     [[0,-.2],[.04,-.2]]],[[0,1],[0,1],[0,-1]],contour_ids=[0,0,1])
    mass=np.array([.2,.7,.5])
    points,normals,weights=_scan_representatives(obs,mass,_configuration(None))
    assert weights.sum()==pytest.approx(mass.sum())
    assert all(any(np.array_equal(point,p) for p in obs.points) for point in points)
    assert len(points)==2


def test_same_contour_corner_mass_cannot_move_to_incompatible_representative():
    # First normal is central. Both later normals match it, but are58 degrees
    # apart. The representative at cell centre is the +29-degree normal.
    theta=np.radians([0.,-29.,29.])
    normals=np.column_stack((np.cos(theta),np.sin(theta)))
    centers=np.array([[.01,.01],[.02,.02],[.05,.05]])
    segments=np.stack((centers+[-.001,0],centers+[.001,0]),axis=1)
    obs=observation(segments,normals,contour_ids=[0,0,0])
    points,directions,weights=_scan_representatives(obs,np.ones(3),_configuration(None))
    cosine=np.cos(np.pi/6)
    # No output may carry3 units unless all3 original signed directions fit it.
    assert all(weight<3 or np.all(normals@direction>=cosine) for direction,weight in zip(directions,weights))


def test_reference_finite_union_never_fills_unobserved_gap():
    ref=reference([[0,0]],[[0,1]],[[[-1,0],[-.2,0]],[[.2,0],[1,0]]],[0,0])
    index=FrozenBoundaryIndex(ref,_configuration(None))
    np.testing.assert_array_equal(index.covered(np.array([[-.5,0],[0,0],[.5,0]]),np.zeros(3,dtype=np.int64)),
                                  [True,False,True])


def test_target_side_compatibility_is_signed_not_absolute_dot():
    ref=reference([[0,0]],[[0,1]],[[[-1,0],[1,0]]],[0])
    index=FrozenBoundaryIndex(ref,_configuration(None))
    np.testing.assert_array_equal(index.nearest(np.array([[0,.1],[0,.1]]),np.array([[0,1],[0,-1]])),[0,-1])


def test_unsupported_mean_is_not_a_nearer_finite_geometric_object():
    ref=reference([[0,0],[.1,.1]],[[0,1],[0,1]],
                  [[[.3,0],[.5,0]],[[-.5,.1],[.5,.1]]],[0,1])
    matcher=BoundaryLaneLocalizer(ref)
    _,_,selected,_,_=matcher._correspondences(np.array([[0,.01]]),np.zeros(3),np.array([[0,1]]))
    # The first mean has no geometry near the query. Its actual segment is
    # farther than the second finite segment, so it cannot block that segment.
    # A genuinely nearer ENDPOINT still blocks farther interior matches; that
    # separate safety contract is tested in test_boundary_association.py.
    np.testing.assert_array_equal(selected, [0])


def test_exact_reference_identity_includes_reliable_votes_not_only_geometry():
    ref=reference([[0,0]],[[0,1]],[[[-1,0],[1,0]]],[0],reliable_votes=[2])
    matcher=BoundaryLaneLocalizer(ref)
    assert matcher.matches_anchor_source(ref)
    assert not matcher.matches_anchor_source(replace(ref,reliable_votes=np.array([1],dtype=np.int64)))


def test_frozen_boundary_index_owns_immutable_configuration_snapshot():
    ref=reference([[2.,0.]],[[0,1]],[[[1.5,0.],[2.5,0.]]],[0])
    cfg=_configuration(None)
    index=FrozenBoundaryIndex(ref,cfg)
    query=np.array([[2.,.1]])
    normals=np.array([[0.,1.]])
    np.testing.assert_array_equal(index.nearest(query,normals),[0])
    original=dict(index.cfg)
    # These alter query hash coordinates, neighbourhood and eligibility if an
    # alias leaks into the index after its cells have already been constructed.
    cfg.update(hash_cell_m=1.,match_distance_m=.01,normal_max_angle_rad=.001,max_patch_points=1)
    assert dict(index.cfg)==original
    np.testing.assert_array_equal(index.nearest(query,normals),[0])
    with pytest.raises(TypeError):
        index.cfg['hash_cell_m']=2.
