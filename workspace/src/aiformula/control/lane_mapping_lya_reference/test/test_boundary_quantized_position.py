"""Independent quantized boundary-position, camera and topology oracles.

No ground truth enters a matcher. These small geometry tests do not establish
real-time budgets or full-lap quality. Closed pixel brackets are observations,
not claims of recovered subpixel paint truth.
"""
from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np
import pytest

from lane_mapping_lya_reference.paint_geometry import (
    BoundaryConsensus, _fit_quadratic_patches, _quantized_curve_positions,
    _quantized_positions, extract_boundary_observation,
)


def camera(width=160, height=160, scale=.01):
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1.
    return SimpleNamespace(width=width, height=height,
        camera_matrix=np.array([[1./scale, 0., width/2.], [0., 1./scale, height/2.], [0., 0., 1.]]),
        base_from_camera=transform, plane_offset_m=0., max_sensitivity_m_per_px=.5)


def deployed_camera():
    return SimpleNamespace(width=640, height=360,
        camera_matrix=np.array([[254.3916130065918,0.,330.0130343437195],
                                [0.,254.39162492752075,181.14964331500232], [0.,0.,1.]]),
        base_from_camera=np.array([[0.,0.,1.,0.],[-1.,0.,0.,0.],
                                   [0.,-1.,0.,.6549999713897705],[0.,0.,0.,1.]]),
        plane_offset_m=.007, max_sensitivity_m_per_px=.5)


def plane_basis(up):
    normal = np.asarray(up, dtype=float)
    normal = normal/np.linalg.norm(normal)
    forward = np.array([1.,0.,0.]) - normal[0]*normal
    forward /= np.linalg.norm(forward)
    left = np.cross(normal, forward)
    return normal, np.array([forward, left])


def scalar_project(uv, calibration, up):
    normal, basis = plane_basis(up)
    rotation, origin = calibration.base_from_camera[:3,:3], calibration.base_from_camera[:3,3]
    ray = rotation @ np.linalg.solve(calibration.camera_matrix, np.array([uv[0],uv[1],1.]))
    denominator = float(np.dot(normal,ray))
    if not math.isfinite(denominator) or denominator >= -1.e-9:
        return None
    distance = (calibration.plane_offset_m-float(np.dot(normal,origin)))/denominator
    result = basis @ (origin+distance*ray)
    return result if np.all(np.isfinite(result)) else None


def scalar_unproject(point, calibration, up):
    normal, basis = plane_basis(up)
    physical = point @ basis + calibration.plane_offset_m*normal
    relative = calibration.base_from_camera[:3,:3].T @ (physical-calibration.base_from_camera[:3,3])
    homogeneous = calibration.camera_matrix @ relative
    return homogeneous[:2]/homogeneous[2]


def scalar_j4(point, calibration, up):
    uv = scalar_unproject(point,calibration,up)
    base = scalar_project(uv,calibration,up)
    if base is None:
        return math.inf
    distances = []
    for shift in ((-1.,0.),(1.,0.),(0.,-1.),(0.,1.)):
        probe = uv+shift
        if not (-.5 <= probe[0] <= calibration.width-.5 and -.5 <= probe[1] <= calibration.height-.5):
            return math.inf
        projected = scalar_project(probe,calibration,up)
        if projected is None:
            return math.inf
        distances.append(float(np.linalg.norm(projected-base)))
    return max(distances)


def diagonal_mask(calibration, phase=0.):
    yy,xx = np.indices((calibration.height,calibration.width))
    signed = xx+.43*yy-(calibration.width*.72+phase)
    return ((signed >= 0.) & (signed <= 5.)).astype(np.uint8)*255


def observe(mask, calibration=None, up=(0.,0.,1.), stamp=1):
    calibration = calibration or camera(mask.shape[1],mask.shape[0])
    return extract_boundary_observation(mask,calibration,up,stamp)


@pytest.mark.parametrize('fraction', [0., .125, .5, .9, 1.])
@pytest.mark.parametrize('angle', [0., .43, -1.7])
def test_known_line_crossing_returns_exact_bracket_position(fraction,angle):
    white = np.array([[2.,-3.]])
    normal = np.array([[math.cos(angle),math.sin(angle)]])
    tangent = np.array([[-normal[0,1],normal[0,0]]])
    crossing = 2.*normal+.4*tangent
    black = white+crossing
    expected = white+fraction*crossing
    # Move the supplied line point along the tangent; the intersection itself
    # must not be replaced by that arbitrary PCA-centroid coordinate.
    # For a CLOSED endpoint oracle use the endpoint itself. A float trig
    # tangent translation can move the stored plane ~1e-17 outside, which the
    # deliberately strict no-clamping contract correctly rejects. Interior
    # tests still exercise independence from the arbitrary line point.
    line_point = expected.copy() if fraction in (0.,1.) else expected+.3*tangent
    actual,valid = _quantized_positions(white,black,normal,line_point,np.array([True]))
    assert valid.tolist() == [True]
    np.testing.assert_allclose(actual,expected,rtol=0.,atol=2.e-15)
    assert abs(float(np.dot(actual[0]-line_point[0],normal[0]))) < 2.e-15


@pytest.mark.parametrize('fraction', [-.01, 1.01, -5., 8.])
def test_outside_line_intersection_is_rejected_not_clamped(fraction):
    result,valid = _quantized_positions(np.array([[0.,0.]]),np.array([[2.,0.]]),
        np.array([[1.,0.]]),np.array([[2.*fraction,3.]]),np.array([True]))
    assert not valid[0] and np.isnan(result).all()


@pytest.mark.parametrize('position',[np.nextafter(0.,-np.inf),np.nextafter(1.,np.inf)])
def test_one_float_outside_exact_bracket_is_not_silently_admitted(position):
    result,valid = _quantized_positions(np.array([[0.,0.]]),np.array([[1.,0.]]),
        np.array([[1.,0.]]),np.array([[position,0.]]),np.array([True]))
    assert not valid[0] and np.isnan(result).all()


@pytest.mark.parametrize('kind', ['tangent', 'cancelling_near_tangent', 'wrong_outward',
                                'nan_white', 'nan_line', 'nan_normal', 'infinity', 'invalid_seed'])
def test_uncertified_numeric_or_orientation_cases_never_fall_back(kind):
    white,black = np.array([[0.,0.]]),np.array([[1.,0.]])
    normal,line,admitted = np.array([[1.,0.]]),np.array([[.5,0.]]),np.array([True])
    if kind == 'tangent': normal[:] = [0.,1.]
    if kind == 'cancelling_near_tangent':
        black[:] = [1.,-1.+np.finfo(float).eps]
        normal[:] = np.array([1.,1.])/math.sqrt(2.)
        line = .5*black
    if kind == 'wrong_outward': normal *= -1
    if kind == 'nan_white': white[0,0] = np.nan
    if kind == 'nan_line': line[0,1] = np.nan
    if kind == 'nan_normal': normal[0,0] = np.nan
    if kind == 'infinity': black[0,0] = np.inf
    if kind == 'invalid_seed': admitted[0] = False
    with np.errstate(invalid='ignore'):
        result,valid = _quantized_positions(white,black,normal,line,admitted)
    assert not valid[0] and np.isnan(result).all()


def test_empty_position_helper_and_empty_real_extraction_stay_explicit():
    result,valid = _quantized_positions(np.empty((0,2)),np.empty((0,2)),np.empty((0,2)),
                                      np.empty((0,2)),np.empty(0,dtype=bool))
    assert result.shape == (0,2) and valid.shape == (0,)
    observed = observe(np.zeros((30,40),dtype=np.uint8))
    assert observed.position_estimates is not None and observed.position_estimates.shape == (0,2)
    assert observed.position_valid.shape == observed.position_sensitivity.shape == observed.position_reliable.shape == (0,)


def test_model_and_raw_observation_are_separate_immutable_and_transform_together():
    calibration = camera()
    observed = observe(diagonal_mask(calibration),calibration)
    selected = np.flatnonzero(observed.model_valid)
    assert len(selected) > 100
    assert np.max(np.linalg.norm(observed.model_points[selected]-observed.points[selected],axis=1)) > 1.e-4
    for name in ('points','segments','sensitivity','position_estimates','position_valid',
                 'position_sensitivity','position_reliable','position_normals'):
        with pytest.raises(ValueError):getattr(observed,name).setflags(write=True)
    # Independent ray/plane check still recovers RAW half-pixel centres.
    for i in selected[::max(1,len(selected)//25)]:
        expected=scalar_project(observed.pixel_segments[i].mean(axis=0),calibration,(0.,0.,1.))
        np.testing.assert_allclose(observed.points[i],expected,rtol=0.,atol=1.e-15)
    before = {name:getattr(observed,name).tobytes() for name in observed.__dict__ if isinstance(getattr(observed,name),np.ndarray)}
    pose=np.array([3.,-2.,.37]);c,s=math.cos(pose[2]),math.sin(pose[2]);rotation=np.array([[c,-s],[s,c]])
    moved=observed.transformed(pose)
    np.testing.assert_allclose(moved.position_estimates,observed.position_estimates@rotation.T+pose[:2],rtol=0.,atol=0.,equal_nan=True)
    np.testing.assert_allclose(moved.position_normals,observed.position_normals@rotation.T,rtol=0.,atol=0.,equal_nan=True)
    np.testing.assert_allclose(moved.points,observed.points@rotation.T+pose[:2],rtol=0.,atol=0.,equal_nan=True)
    for name in ('position_valid','position_sensitivity','position_reliable','pixel_segments','white_pixels','black_pixels'):
        assert getattr(moved,name).tobytes()==getattr(observed,name).tobytes()
    assert all(getattr(observed,name).tobytes()==value for name,value in before.items())


def test_only_explicit_legacy_none_uses_raw_positions_not_invalid_models():
    calibration=camera();observed=observe(diagonal_mask(calibration),calibration)
    legacy=replace(observed,position_estimates=None,position_valid=None,position_sensitivity=None,position_reliable=None,
                   position_normals=None, position_segments=None, position_segment_valid=None,
                   position_segment_sensitivity=None, position_segment_reliable=None,
                   position_footprints=None)
    # An explicit legacy fixture clears every model sidecar. Keeping the new
    # footprint without its parent model remains malformed, never a fallback.
    with pytest.raises(ValueError, match='position footprints require finite position fragments'):
        replace(legacy, position_footprints=observed.position_footprints)
    assert legacy.model_points is legacy.points and legacy.model_valid is legacy.valid
    assert legacy.model_sensitivity is legacy.sensitivity and legacy.model_reliable is legacy.reliable
    assert legacy.model_normals is legacy.normals
    invalid=replace(observed,position_estimates=np.full_like(observed.points,np.nan),
        position_valid=np.zeros(len(observed.points),dtype=bool),position_sensitivity=np.full(len(observed.points),np.inf),
        position_reliable=np.zeros(len(observed.points),dtype=bool),
        position_segment_valid=np.zeros(len(observed.points),dtype=bool),
        position_segment_reliable=np.zeros(len(observed.points),dtype=bool))
    assert np.isnan(invalid.model_points).all() and not invalid.model_valid.any()
    assert np.isinf(invalid.model_sensitivity).all() and not invalid.model_reliable.any()
    assert invalid.points.tobytes()==observed.points.tobytes()
    consensus=BoundaryConsensus();proposal=consensus.prepare(invalid)
    assert proposal.stats['eligible_edges']==0 and not proposal.updates


@pytest.mark.parametrize('bad',['partial','normal_missing','valid_nan','valid_infinite_j','negative_j',
                              'reliable_invalid','valid_nan_normal','valid_nonunit_normal'])
def test_model_evidence_constructor_rejects_inconsistent_flags(bad):
    calibration=camera();observation=observe(diagonal_mask(calibration),calibration)
    changes={};i=int(np.flatnonzero(observation.position_valid)[0])
    if bad=='partial':changes['position_valid']=None
    if bad=='normal_missing':changes['position_normals']=None
    if bad=='valid_nan':
        values=observation.position_estimates.copy();values[i]=np.nan;changes['position_estimates']=values
    if bad in ('valid_infinite_j','negative_j'):
        values=observation.position_sensitivity.copy();values[i]=np.inf if bad=='valid_infinite_j' else -.1;changes['position_sensitivity']=values
    if bad=='reliable_invalid':
        valid=observation.position_valid.copy();valid[i]=False
        reliable=observation.position_reliable.copy();reliable[i]=True
        changes.update(position_valid=valid,position_reliable=reliable)
    if bad in ('valid_nan_normal','valid_nonunit_normal'):
        values=observation.position_normals.copy();values[i]=[np.nan,0.] if bad=='valid_nan_normal' else [2.,0.]
        changes['position_normals']=values
    with pytest.raises(ValueError):replace(observation,**changes)


@pytest.mark.parametrize('up',[(0.,0.,1.),(.012,-.007,math.sqrt(1.-.012**2-.007**2))])
def test_fractional_uv_j4_matches_independent_homography_with_offset_and_tilt(up):
    calibration=deployed_camera()
    # A full image-local diagonal paint stripe supplies many real side faces
    # under a strongly non-affine forward-camera homography. No ROI is made.
    mask=diagonal_mask(calibration,phase=.27)
    observed=observe(mask,calibration,up)
    selected=np.flatnonzero(observed.position_valid)
    assert len(selected)>30
    projective_fraction_difference=[]
    for i in selected[::max(1,len(selected)//70)]:
        point=observed.position_estimates[i]
        expected_j=scalar_j4(point,calibration,up)
        np.testing.assert_allclose(observed.position_sensitivity[i],expected_j,rtol=5.e-11,atol=1.e-12)
        white=scalar_project(observed.white_pixels[i],calibration,up)
        black=scalar_project(observed.black_pixels[i],calibration,up)
        crossing=black-white
        fraction=float(np.dot(point-white,crossing)/np.dot(crossing,crossing))
        assert -.000000000001<=fraction<=1.000000000001  # independent projection roundoff only
        np.testing.assert_allclose(point,white+fraction*crossing,rtol=1.e-12,atol=1.e-12)
        actual_uv=scalar_unproject(point,calibration,up)
        wrong_uv=observed.white_pixels[i]+fraction*(observed.black_pixels[i]-observed.white_pixels[i])
        projective_fraction_difference.append(float(np.linalg.norm(actual_uv-wrong_uv)))
    assert max(projective_fraction_difference)>1.e-5
    np.testing.assert_array_equal(observed.position_reliable,
                                  observed.position_valid&(observed.position_sensitivity<=.1))
    np.testing.assert_array_equal(observed.model_valid,observed.valid&observed.position_valid)
    np.testing.assert_array_equal(observed.model_reliable,observed.reliable&observed.position_reliable)
    np.testing.assert_array_equal(observed.model_sensitivity,np.maximum(observed.sensitivity,observed.position_sensitivity))
    assert np.all(observed.position_sensitivity[observed.position_valid]<=.5)


@pytest.mark.parametrize('phase',[0.,.2,.45,.8])
def test_straight_different_pixel_phases_keep_models_inside_real_crossings(phase):
    calibration=camera();mask=diagonal_mask(calibration,phase)
    observation=observe(mask,calibration)
    uv=observation.pixel_segments.mean(axis=1)
    central=(uv[:,0]>40)&(uv[:,0]<120)&(uv[:,1]>40)&(uv[:,1]<120)&observation.position_valid
    assert central.sum()>40
    # The true synthetic line's image equations are independently known. This
    # is a fixed absolute raster-scale quality check, not a tuned gate.
    for index in np.flatnonzero(central):
        point=observation.position_estimates[index]
        model_uv=scalar_unproject(point,calibration,(0.,0.,1.))
        signed=model_uv[0]+.43*model_uv[1]-(calibration.width*.72+phase)
        distance=min(abs(signed),abs(signed-5.))/math.sqrt(1.+.43**2)
        assert distance<=.5
        w=scalar_project(observation.white_pixels[index],calibration,(0.,0.,1.))
        b=scalar_project(observation.black_pixels[index],calibration,(0.,0.,1.))
        assert np.linalg.norm(point-w)+np.linalg.norm(point-b)<=np.linalg.norm(b-w)+1.e-12


@pytest.mark.parametrize('shape',['u','gap'])
def test_real_u_or_gap_does_not_gain_unobserved_links_from_near_model_positions(shape):
    calibration=camera(200,200)
    mask=np.zeros((200,200),dtype=np.uint8)
    if shape=='u':
        mask[15:185,70:76]=255;mask[15:185,82:88]=255;mask[179:185,70:88]=255
    else:
        mask[10:88,98:102]=255;mask[112:190,98:102]=255
    observation=observe(mask,calibration)
    assert observation.position_valid.sum()>80
    consensus=BoundaryConsensus()
    first=consensus.prepare(observation)
    face_by_segment={segment.tobytes():i for i,segment in enumerate(observation.segments)}
    owners={}
    for record in first.updates:
        for segment in record.segments:owners[face_by_segment[segment.tobytes()]]=record.record_id
    expected=set()
    for face,owner in owners.items():
        following=int(observation.next[face])
        if following in owners and owners[following]!=owner:expected.add((owner,owners[following]))
    assert consensus.commit(first)
    for stamp in range(2,6):assert consensus.commit(consensus.prepare(replace(observation,source_stamp_ns=stamp)))
    reference=consensus.snapshot()
    actual={tuple(reference.record_ids[edge]) for edge in reference.directed_edges}
    assert actual==expected
    assert np.all(reference.votes==5) and np.all(reference.edge_frame_votes==5)
    if shape=='gap':
        for a,b in reference.directed_edges:
            assert reference.points_xy[a,1]*reference.points_xy[b,1]>0
    else:
        # Four separately observed middle-arm sides remain four positions,
        # despite both thin arms/edges being closer than a world-grid cell.
        middle=np.abs(reference.points_xy[:,1])<.2
        expected_x=np.array([-.305,-.245,-.185,-.125])
        assert middle.sum()>8
        nearest=np.argmin(abs(reference.points_xy[middle,0,None]-expected_x),axis=1)
        assert set(nearest)=={0,1,2,3}
        assert np.max(np.min(abs(reference.points_xy[middle,0,None]-expected_x),axis=1))<1.e-12


@pytest.mark.parametrize('angle',[0.,.7,-1.3])
@pytest.mark.parametrize('curvature',[0.,.12,-.25])
def test_quadratic_fit_known_polynomial_rotated_frame_and_masked_outliers(angle,curvature):
    # Analytic y=.03-.08*x+curvature*x² in the independently chosen frame.
    # The two arbitrary outliers are EXCLUDED evidence, not fit-dependent pruning.
    x=np.array([-2.,-1.5,-.9,-.4,0.,.3,.8,1.4,1.8,2.,-20.,30.])
    y=.03-.08*x+curvature*x*x
    y[-2:]=[400.,-600.]
    normal=np.array([math.sin(angle),math.cos(angle)])
    tangent=np.array([math.cos(angle),-math.sin(angle)])
    delta=x[:,None]*tangent+y[:,None]*normal
    selected=np.array([True]*10+[False]*2)
    coefficients,scales,bounds=_fit_quadratic_patches(delta[None,:,:],selected[None,:],normal[None,:])
    np.testing.assert_allclose(scales,[2.],rtol=0.,atol=1.e-15)
    np.testing.assert_allclose(bounds,[[-1.,1.]],rtol=0.,atol=1.e-15)
    expected=np.array([.03,-.08*2.,curvature*4.])
    np.testing.assert_allclose(coefficients[0],expected,rtol=2.e-13,atol=2.e-14)
    # Separate rectangular least-squares oracle, not the production normal equations.
    design=np.column_stack((np.ones(10),x[:10]/2.,(x[:10]/2.)**2))
    literal=np.linalg.lstsq(design,y[:10],rcond=None)[0]
    np.testing.assert_allclose(coefficients[0],literal,rtol=2.e-13,atol=2.e-14)


@pytest.mark.parametrize('extent',[1.e-3,.2,8.])
def test_quadratic_fit_scaling_preserves_physical_polynomial(extent):
    x=np.linspace(-extent,extent,13)
    # Define normalized coordinates to avoid changing condition with metres.
    y=.02+.01*x/extent+.025*(x/extent)**2
    delta=np.stack((x,y),axis=1)[None,:,:]
    coefficients,scales,bounds=_fit_quadratic_patches(delta,np.ones((1,13),dtype=bool),np.array([[0.,1.]]))
    np.testing.assert_allclose(coefficients,[[.02,.01,.025]],rtol=2.e-13,atol=1.e-15)
    assert scales[0]==extent
    np.testing.assert_array_equal(bounds,[[-1.,1.]])


@pytest.mark.parametrize('x,mask',[
    ([0.]*6,[True]*6),
    ([-1.,1.]*3,[True]*6),
    ([-1.,0.,1.,-1.,0.,1.],[False]*6),
    ([-1.,0.,1.,-1.,0.,1.],[True,False,False,True,False,False]),
])
def test_rank_deficient_quadratic_support_does_not_invent_coefficients(x,mask):
    delta=np.column_stack((x,np.ones(len(x))*.03))[None,:,:]
    coefficients,scales,bounds=_fit_quadratic_patches(delta,np.asarray([mask]),np.array([[0.,1.]]))
    assert np.isnan(coefficients).all()
    points,directions,valid=_quantized_curve_positions(np.array([[0.,-.1]]),np.array([[0.,.1]]),
        np.zeros((1,2)),np.array([[0.,1.]]),coefficients,scales,bounds,np.array([True]))
    assert not valid[0] and np.isnan(points).all() and np.array_equal(directions,[[0.,0.]])


def curve_call(white,black,coefficients=(0.,0.,1.),scale=1.,bounds=(-1.,1.),
               origin=(0.,0.),normal=(0.,1.),valid=True):
    return _quantized_curve_positions(np.asarray([white],dtype=float),np.asarray([black],dtype=float),
        np.asarray([origin],dtype=float),np.asarray([normal],dtype=float),np.asarray([coefficients],dtype=float),
        np.asarray([scale],dtype=float),np.asarray([bounds],dtype=float),np.array([valid]))


@pytest.mark.parametrize('angle',[0.,.4,-1.2])
@pytest.mark.parametrize('kind',['line','parabola','oblique_parabola'])
def test_curve_intersection_known_position_and_differential_normal(angle,kind):
    if kind=='line':
        white,black=np.array([.25,-.1]),np.array([.25,.5])
        coefficients=(.1,.2,0.);expected=np.array([.25,.15]);gradient=np.array([-.2,1.])
    elif kind=='parabola':
        white,black=np.array([.2,-.2]),np.array([.2,.2])
        coefficients=(0.,0.,1.);expected=np.array([.2,.04]);gradient=np.array([-.4,1.])
    else:
        # y=x², crossing x=.25t, y=-.05+.15t has t=.4 and2.
        # Exactly ONE root lies inside the observed bracket.
        white,black=np.array([0.,-.05]),np.array([.25,.1])
        coefficients=(0.,0.,1.);expected=np.array([.1,.01]);gradient=np.array([-.2,1.])
    c,s=math.cos(angle),math.sin(angle);rotation=np.array([[c,-s],[s,c]])
    origin=np.array([3.,-2.])
    actual,direction,valid=curve_call(white@rotation.T+origin,black@rotation.T+origin,coefficients,
        origin=origin,normal=np.array([0.,1.])@rotation.T)
    assert valid.tolist()==[True]
    np.testing.assert_allclose(actual[0],expected@rotation.T+origin,rtol=0.,atol=2.e-15)
    gradient/=np.linalg.norm(gradient)
    np.testing.assert_allclose(direction[0],gradient@rotation.T,rtol=0.,atol=3.e-15)
    assert np.dot(direction[0],(black-white)@rotation.T)>0.


@pytest.mark.parametrize('kind',['no_root','two_roots','double_root','outside_bracket','wrong_outward',
                                'outside_observed_patch','too_steep','invalid_seed','nan_coefficients',
                                'infinite_scale','negative_scale','zero_scale','nan_bounds',
                                'infinite_bounds','reversed_bounds','nan_white'])
def test_curve_ambiguous_unobserved_nonfinite_or_wrong_direction_is_rejected(kind):
    white,black=np.array([.2,-.2]),np.array([.2,.2])
    kwargs={}
    if kind=='no_root':white,black=np.array([-1.,-.1]),np.array([1.,-.1])
    if kind=='two_roots':white,black=np.array([-1.,.25]),np.array([1.,.25])
    if kind=='double_root':white,black=np.array([-1.,0.]),np.array([1.,0.])
    if kind=='outside_bracket':white,black=np.array([.2,-.3]),np.array([.2,-.2])
    if kind=='wrong_outward':white,black=black,white
    if kind=='outside_observed_patch':kwargs['bounds']=(-.1,.1)
    if kind=='too_steep':white,black=np.array([.6,.2]),np.array([.6,.5])
    if kind=='invalid_seed':kwargs['valid']=False
    if kind=='nan_coefficients':kwargs['coefficients']=(0.,np.nan,1.)
    if kind=='infinite_scale':kwargs['scale']=np.inf
    if kind=='negative_scale':kwargs['scale']=-1.
    if kind=='zero_scale':kwargs['scale']=0.
    if kind=='nan_bounds':kwargs['bounds']=(np.nan,1.)
    if kind=='infinite_bounds':kwargs['bounds']=(-np.inf,np.inf)
    if kind=='reversed_bounds':kwargs['bounds']=(1.,-1.)
    if kind=='nan_white':white[0]=np.nan
    actual,direction,valid=curve_call(white,black,**kwargs)
    assert not valid[0] and np.isnan(actual).all()
    np.testing.assert_array_equal(direction,[[0.,0.]])


@pytest.mark.parametrize('x',[-.25,.25])
def test_curve_no_extrapolation_closed_observed_bounds(x):
    # Dyadic coefficients make this a literal exact closed-endpoint oracle;
    # the initial .2 fixture had only an irrelevant 2.8e-17 reconstruction
    # rounding difference, retained in the first XML report.
    actual,direction,valid=curve_call([x,-.25],[x,.25],bounds=(-.25,.25))
    assert valid[0]
    np.testing.assert_array_equal(actual,[[x,x*x]])
    just_outside=np.nextafter(x,-np.inf if x<0 else np.inf)
    rejected,_,valid=curve_call([just_outside,-.25],[just_outside,.25],bounds=(-.25,.25))
    assert not valid[0] and np.isnan(rejected).all()
