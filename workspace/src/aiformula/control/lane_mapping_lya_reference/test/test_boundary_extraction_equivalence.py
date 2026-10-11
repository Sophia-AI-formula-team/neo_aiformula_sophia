"""Every-byte extraction oracle for exact structural acceleration.

Literal old functions below were copied from pre-optimization paint source
SHA256 4218dd97941cd24babc65ed31c4db25b59314c7c0b06a62a3e16f7fc70fda350.
There is no runtime dependency on diagnostics, Git blobs or a local path.
These bounded geometry tests are NOT runtime/full-lap acceptance evidence.
"""
from dataclasses import fields
import math
from types import SimpleNamespace

import numpy as np
import pytest

from lane_mapping_lya_reference import paint_geometry as paint


def _old_fit_quadratic_patches(delta, weights, normals):
    """Local y=a+b*x+c*x^2 in the measured PCA frame, not a tangent line.

    The same existing contour window is used. Scaling x to unit magnitude
    avoids metre-dependent normal-equation rank decisions. No ridge, invented
    sample, changed support radius, or truth-derived curvature is introduced.
    """
    tangents = np.column_stack((normals[:, 1], -normals[:, 0]))
    x = np.sum(delta*tangents[:, None, :], axis=2)
    y = np.sum(delta*normals[:, None, :], axis=2)
    scales = np.max(np.where(weights, np.abs(x), 0.), axis=1)
    scaled = np.zeros_like(x)
    np.divide(x, scales[:, None], out=scaled, where=scales[:, None] > 0.)
    design = np.stack((np.ones_like(x), scaled, scaled*scaled), axis=2)
    gram = np.einsum('nki,nkj,nk->nij', design, design, weights)
    rhs = np.einsum('nki,nk,nk->ni', design, y, weights)
    eigenvalues = np.linalg.eigvalsh(gram)
    good = (scales > 0.) & (eigenvalues[:, 0] > 64.*np.finfo(np.float64).eps*eigenvalues[:, -1])
    coefficients = np.full((len(delta), 3), np.nan)
    coefficients[good] = np.linalg.solve(gram[good], rhs[good, :, None])[:, :, 0]
    bounds = np.column_stack((np.min(np.where(weights, scaled, np.inf), axis=1),
                              np.max(np.where(weights, scaled, -np.inf), axis=1)))
    return coefficients, scales, bounds


def _old_directions(points, segments, white_xy, black_xy, pixel_outward, stable, paths, cfg):
    """Batched contour-only windows. Stops at missing/J-cut/opposite faces.

    Limiting arc distance (rather than Euclidean distance alone) also prevents
    a nearby U arm being pulled into the local patch through a remote bridge.
    Opposite IMAGE face polarities stop traversal even around a very short end
    cap. Perspective is not angle-preserving: orthogonal pixel stair faces can
    have obtuse metric normals, which must not split one observed paint side.
    """
    n = len(points)
    normals, valid = np.zeros((n, 2)), np.zeros(n, dtype=bool)
    curve_coefficients = np.full((n, 3), np.nan)
    curve_scale = np.full(n, np.nan)
    curve_bounds = np.full((n, 2), np.nan)
    support_count, used_radius = np.zeros(n, dtype=np.int64), np.zeros(n)
    tangent = segments[:, 1] - segments[:, 0]
    length = np.linalg.norm(tangent, axis=1)
    white_to_black = black_xy - white_xy
    crossing_length = np.linalg.norm(white_to_black, axis=1)
    ok = (stable & np.isfinite(length) & (length > 0)
          & np.isfinite(crossing_length) & (crossing_length > 0))
    crossing_direction = np.zeros_like(points)
    crossing_direction[ok] = white_to_black[ok] / crossing_length[ok, None]
    max_points = cfg["max_patch_points"]
    left_max, right_max = (max_points - 1) // 2, max_points // 2
    radii, radius = [], cfg["normal_radius_m"]
    while radius < cfg["normal_max_radius_m"]:
        radii.append(radius)
        radius = min(radius * 2., cfg["normal_max_radius_m"])
    radii.append(cfg["normal_max_radius_m"])
    for path, closed in paths:
        count = len(path)
        if count < cfg["normal_min_points"]:
            continue
        left = min(left_max, (count - 1) // 2) if closed else min(left_max, count - 1)
        right = min(right_max, count - 1 - left) if closed else min(right_max, count - 1)
        offsets = np.arange(-left, right + 1)
        # Arc distance follows the measured face, including its corner vertex.
        step = np.linalg.norm(segments[path, 1] - points[path], axis=1)
        step += np.linalg.norm(points[np.roll(path, -1)] - segments[path, 1], axis=1)
        step[~np.isfinite(step)] = np.inf
        arc = np.concatenate(([0.], np.cumsum(np.where(np.isfinite(step), step, 0.))))
        circumference = arc[-1]
        for start in range(0, count, 128):
            positions = np.arange(start, min(start + 128, count))
            targets = path[positions]
            slots = positions[:, None] + offsets
            inside = np.ones(slots.shape, dtype=bool) if closed else ((slots >= 0) & (slots < count))
            wrapped = slots % count
            candidates = path[wrapped]
            arc_candidate = arc[wrapped]
            if closed:
                arc_candidate = arc_candidate + np.floor_divide(slots, count) * circumference
            distance = np.abs(arc_candidate - arc[positions, None])
            # Exact cardinal integer pixel directions: same=1, stair=0,
            # opposite=-1. Keep the seed-relative test and contiguous stop;
            # neither a later compatible face nor a J gap can be jumped.
            compatible = np.sum(pixel_outward[targets, None, :] * pixel_outward[candidates], axis=2) >= 0
            allowed = inside & ok[targets, None] & ok[candidates] & compatible
            # Every intervening face must be usable; never jump an invalid face.
            if left:
                allowed[:, :left] = np.logical_and.accumulate(allowed[:, :left][:, ::-1], axis=1)[:, ::-1]
            if right:
                allowed[:, left + 1:] = np.logical_and.accumulate(allowed[:, left + 1:], axis=1)
            differences = points[candidates] - points[targets, None, :]
            differences[~np.isfinite(differences)] = 0.
            for radius in radii:
                selected = allowed & (distance <= radius)
                counts = np.count_nonzero(selected, axis=1)
                enough = (~valid[targets]) & (counts >= cfg["normal_min_points"])
                if not np.any(enough):
                    continue
                delta, weights = differences[enough], selected[enough]
                means = np.sum(delta * weights[:, :, None], axis=1) / counts[enough, None]
                centered = delta - means[:, None, :]
                cov = np.einsum("nki,nkj,nk->nij", centered, centered, weights) / counts[enough, None, None]
                eigenvalues, eigenvectors = np.linalg.eigh(cov)
                accepted = ((eigenvalues[:, 1] > 1.e-5)
                            & (eigenvalues[:, 0] <= cfg["normal_max_thickness_m"] ** 2)
                            & (eigenvalues[:, 0] <= cfg["normal_max_variance_ratio"] * eigenvalues[:, 1]))
                fitted = eigenvectors[:, :, 0]
                # A normal is a covector. Its sign against the actual measured
                # white->black displacement survives camera shear/perspective;
                # dotting two projected normals does not. Normalize the
                # displacement so the existing 1e-12 angular check is unitless.
                orientation = np.sum(fitted * crossing_direction[targets[enough]], axis=1)
                accepted &= np.abs(orientation) > 1.e-12
                fitted[orientation < 0] *= -1
                indices = targets[enough][accepted]
                normals[indices] = fitted[accepted]
                coefficients, scales, bounds = _old_fit_quadratic_patches(
                    delta[accepted], weights[accepted], fitted[accepted])
                curve_coefficients[indices], curve_scale[indices] = coefficients, scales
                curve_bounds[indices] = bounds
                valid[indices] = True
                support_count[indices] = counts[enough][accepted]
                used_radius[indices] = radius
    return normals, valid, support_count, used_radius, curve_coefficients, curve_scale, curve_bounds

def assert_array_bytes(actual, expected, name='array'):
    assert actual.shape==expected.shape, name
    assert actual.dtype==expected.dtype, name
    assert actual.tobytes()==expected.tobytes(), name


def assert_all_fields(actual, expected):
    assert type(actual) is type(expected)
    for field in fields(actual):
        value, prior=getattr(actual,field.name),getattr(expected,field.name)
        if isinstance(value,np.ndarray):
            assert_array_bytes(value,prior,field.name)
            assert value.flags.writeable==prior.flags.writeable, field.name
        elif isinstance(value,(float,np.floating)):
            assert np.float64(value).tobytes()==np.float64(prior).tobytes(), field.name
        else:
            assert value==prior,field.name


def camera(width=96,height=96,perspective=False,max_j=.5):
    if perspective:
        # Same deployed optical directions and scaled pixel calibration.
        k=np.array([[254.3916130065918,0.,330.0130343437195],
                    [0.,254.39162492752075,181.14964331500232],[0.,0.,1.]])
        k[0,:]*=width/640.
        k[1,:]*=height/360.
        transform=np.array([[0.,0.,1.,0.],[-1.,0.,0.,0.],
                            [0.,-1.,0.,.6549999713897705],[0.,0.,0.,1.]])
        offset=.007
    else:
        k=np.array([[75.,0.,width/2.],[0.,75.,height/2.],[0.,0.,1.]])
        transform=np.diag([1.,-1.,-1.,1.])
        transform[2,3]=1.
        offset=0.
    return SimpleNamespace(width=width,height=height,camera_matrix=k,
        base_from_camera=transform,plane_offset_m=offset,max_sensitivity_m_per_px=max_j)


def make_mask(kind,width,height):
    yy,xx=np.indices((height,width))
    x,y=xx/width,yy/height
    if kind=='empty':
        active=np.zeros((height,width),dtype=bool)
    elif kind=='full':
        active=np.ones((height,width),dtype=bool)
    elif kind=='straight':
        active=(x>.40)&(x<.48)&(y>.07)&(y<.94)
    elif kind=='slanted_fov':
        active=np.abs(x+.4*y-.62)<.038
    elif kind=='curve':
        active=np.abs(x-(.3+.33*(y-.45)**2))<.035
    elif kind=='closed':
        radius=np.hypot(x-.5,y-.5)
        active=(radius>.23)&(radius<.29)
    elif kind=='checker':
        active=(xx+yy)%2==0
    elif kind=='crossing':
        active=(np.abs(x-.5)<.035)|(np.abs(y-.65)<.035)
    elif kind=='u_gap':
        active=((x>.25)&(x<.30)|(x>.60)&(x<.65))&(y>.10)&(y<.85)
        active|=(x>.25)&(x<.65)&(y>.80)&(y<.85)
        active[(y>.35)&(y<.39)&(x<.4)]=False
    elif kind=='random_blocks':
        rng=np.random.RandomState(61011)
        active=np.zeros((height,width),dtype=bool)
        for _ in range(12):
            sx,sy=rng.randint(0,width-6),rng.randint(0,height-6)
            active[sy:min(height,sy+int(rng.randint(2,15))),sx:min(width,sx+int(rng.randint(2,15)))]=True
    else:
        raise AssertionError(kind)
    return active.astype(np.uint8)*255


@pytest.mark.parametrize('perspective',[False,True])
@pytest.mark.parametrize('kind',['empty','full','straight','slanted_fov','curve',
                                 'closed','checker','crossing','u_gap','random_blocks'])
def test_complete_observation_every_field_matches_old_whole_extraction(monkeypatch,perspective,kind):
    width,height=(160,96) if perspective else (96,96)
    calibration=camera(width,height,perspective)
    up=np.array([.0083,-.0012,1.]) if perspective else np.array([0.,0.,1.])
    up/=np.linalg.norm(up)
    mask=make_mask(kind,width,height)
    mask_before=mask.tobytes()
    current=paint.extract_boundary_observation(mask,calibration,up,123456789)
    with monkeypatch.context() as scope:
        scope.setattr(paint,'_directions',_old_directions)
        scope.setattr(paint,'_fit_quadratic_patches',_old_fit_quadratic_patches)
        expected=paint.extract_boundary_observation(mask,calibration,up,123456789)
    assert mask.tobytes()==mask_before
    assert_all_fields(current,expected)


@pytest.mark.parametrize('max_j',[.5,.1,.025])
@pytest.mark.parametrize('cap',[128,512])
def test_perspective_tilt_j_gaps_and_original_fov_are_not_bridged(monkeypatch,max_j,cap):
    calibration=camera(320,180,True,max_j)
    mask=make_mask('slanted_fov',320,180)
    mask[120:126,140:150]=0
    up=np.array([.018,-.006,1.])
    up/=np.linalg.norm(up)
    cfg={'max_patch_points':cap,'reliable_sensitivity':min(.1,max_j)}
    current=paint.extract_boundary_observation(mask,calibration,up,17,config=cfg)
    with monkeypatch.context() as scope:
        scope.setattr(paint,'_directions',_old_directions)
        scope.setattr(paint,'_fit_quadratic_patches',_old_fit_quadratic_patches)
        expected=paint.extract_boundary_observation(mask,calibration,up,17,config=cfg)
    assert_all_fields(current,expected)
    assert not np.any(np.all(current.pixel_segments[:,:,0]==-.5,axis=1))
    assert np.any(~current.stable)


@pytest.mark.parametrize('patch_size',[128,512])
@pytest.mark.parametrize('seed',range(4))
@pytest.mark.parametrize('layout',['contiguous','strided'])
def test_random_three_operand_contractions_every_output_bit(patch_size,seed,layout):
    rng=np.random.RandomState(seed)
    rows=3 if seed<3 else 129
    delta=rng.normal(size=(rows,patch_size,2))*.04
    delta[:,:,0]+=np.linspace(-.45,.45,patch_size)[None,:]
    weights=rng.rand(rows,patch_size)>.25
    weights[0,:]=seed!=0
    if seed==1:
        delta[0,:,0]=0. # rank-deficient patch must remain invalid
    angle=rng.uniform(-math.pi,math.pi,rows)
    normals=np.column_stack((np.cos(angle),np.sin(angle)))
    if layout=='strided':
        backing=np.empty((rows,patch_size*2,4))
        backing[:,::2,::2]=delta
        delta=backing[:,::2,::2]
        mask_backing=np.zeros((rows,patch_size*2),dtype=bool)
        mask_backing[:,::2]=weights
        weights=mask_backing[:,::2]
        normal_backing=np.empty((rows,4))
        normal_backing[:,::2]=normals
        normals=normal_backing[:,::2]
    old_bytes=tuple(v.tobytes() for v in (delta,weights,normals))
    actual=paint._fit_quadratic_patches(delta,weights,normals)
    expected=_old_fit_quadratic_patches(delta,weights,normals)
    for name,a,b in zip(('coefficients','scales','bounds'),actual,expected):
        assert_array_bytes(a,b,name)
    assert old_bytes==tuple(v.tobytes() for v in (delta,weights,normals))


@pytest.mark.parametrize('rows',[0,1,128,129])
def test_signed_zero_rank_deficient_and_empty_rows(rows):
    count=128
    delta=np.zeros((rows,count,2))
    delta[:,:,0]=np.linspace(-1.,1.,count)[None,:]
    delta[:,::2,1]=-0.
    normals=np.tile([0.,1.],(rows,1))
    weights=np.ones((rows,count),dtype=bool)
    actual=paint._fit_quadratic_patches(delta,weights,normals)
    expected=_old_fit_quadratic_patches(delta,weights,normals)
    for a,b in zip(actual,expected):
        assert_array_bytes(a,b)


@pytest.mark.parametrize('patch_size',[128,512])
@pytest.mark.parametrize('rows',[1,129])
def test_both_negative_zero_products_preserve_sum_positive_zero(patch_size,rows):
    # np.sum's positive-zero identity is not equivalent to (-0.) + (-0.).
    # Exercise both the tangent x contraction and normal y contraction;
    # invalid quadratic rank must still leave exact scales/bounds/sign bits.
    delta=np.full((rows,patch_size,2),-0.,dtype=np.float64)
    component=math.sqrt(.5)
    weights=np.ones((rows,patch_size),dtype=bool)
    for direction in ((-component,component),(component,component)):
        normals=np.tile(direction,(rows,1))
        vector=(np.column_stack((normals[:,1],-normals[:,0]))
                if direction[0]<0 else normals)
        products=delta*vector[:,None,:]
        assert np.all(np.signbit(products))
        original_sum=np.sum(products,axis=2)
        direct_add=products[:,:,0]+products[:,:,1]
        assert not np.any(np.signbit(original_sum))
        assert np.all(np.signbit(direct_add))
        actual=paint._fit_quadratic_patches(delta,weights,normals)
        expected=_old_fit_quadratic_patches(delta,weights,normals)
        for name,a,b in zip(('coefficients','scales','bounds'),actual,expected):
            assert_array_bytes(a,b,name)


def direction_inputs(closed,invalid):
    count=541
    if closed:
        angle=np.linspace(0.,2.*math.pi,count,endpoint=False)
        points=np.column_stack((np.cos(angle),np.sin(angle)))*.75
        outward=np.column_stack((np.cos(angle),np.sin(angle)))
        tangent=np.column_stack((-np.sin(angle),np.cos(angle)))
    else:
        x=np.linspace(-2.,2.,count)
        points=np.column_stack((x,.02*x*x))
        outward=np.column_stack((-.04*x,np.ones(count)))
        outward/=np.linalg.norm(outward,axis=1)[:,None]
        tangent=np.column_stack((outward[:,1],-outward[:,0]))
    segments=np.stack((points-.002*tangent,points+.002*tangent),axis=1)
    white,black=points-.004*outward,points+.004*outward
    pixel_outward=np.zeros((count,2),dtype=np.int64)
    greatest=np.argmax(np.abs(outward),axis=1)
    pixel_outward[np.arange(count),greatest]=np.sign(outward[np.arange(count),greatest]).astype(np.int64)
    stable=np.ones(count,dtype=bool)
    if invalid!='none':
        ids=np.array([0,1,126,127,128,129,255,256,382,383,384,539,540])
        stable[ids]=False
        if invalid=='nonfinite':
            points[ids[::2]]=np.nan
            segments[ids[1::2]]=np.inf
            white[ids[::2]]=np.nan
        if invalid=='all':
            stable[:]=False
    return points,segments,white,black,pixel_outward,stable,[(np.arange(count,dtype=np.int64),closed)]


@pytest.mark.parametrize('closed',[False,True])
@pytest.mark.parametrize('invalid',['none','gaps','nonfinite','all'])
@pytest.mark.parametrize('cap',[128,512])
def test_original_batch_targets_and_unusable_rows_keep_all_direction_outputs(closed,invalid,cap):
    args=direction_inputs(closed,invalid)
    cfg=dict(paint.DEFAULT_CONFIG,max_patch_points=cap)
    with np.errstate(invalid='ignore',over='ignore'):
        actual=paint._directions(*args,cfg)
        expected=_old_directions(*args,cfg)
    for name,a,b in zip(('normals','valid','support_count','radius','coefficients','scale','bounds'),actual,expected):
        assert_array_bytes(a,b,name)


def test_old_direction_oracle_does_not_call_optimized_fit(monkeypatch):
    def forbidden(*args,**kwargs):
        raise AssertionError('old oracle called production numerical helper')
    args=direction_inputs(False,'none')
    monkeypatch.setattr(paint,'_fit_quadratic_patches',forbidden)
    result=_old_directions(*args,dict(paint.DEFAULT_CONFIG))
    assert result[1].any()

