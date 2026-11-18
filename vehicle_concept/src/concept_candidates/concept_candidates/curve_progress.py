"""Runtime source extracted from the accepted V6.26 freeze: experiments/v616_decision/curve_progress.py. """

from dataclasses import dataclass

import numpy as np

from concept_road.compare61513 import Guided

@dataclass(frozen=True)
class ReferenceObservation(Guided):
    reference:object=None

def build_reference(evidence):
    material=evidence.get('material_evidence',{})
    edges=material.get('physical_edges',[])
    left=[e for e in edges if e['side']==-1];right=[e for e in edges if e['side']==1]
    pair=None;source=None
    if left and right:
        pair=[left[0],right[0]];source='observed_material_outer_edges'
    else:
        lane=evidence.get('current_lane');tracks=evidence.get('tracks',[])
        if lane and lane['score']>.45:
            pair=[tracks[lane['left']],tracks[lane['right']]];source='supported_current_lane_markings'
    if pair is None:return None
    lo=max(r['zmin'] for r in pair);hi=min(r['zmax'] for r in pair)
    if hi<=lo:return None
    coef=np.mean([r['coef'] for r in pair],axis=0)
    z=np.linspace(lo,hi,int(np.ceil((hi-lo)/.08))+1)
    x=np.polynomial.polynomial.polyval(z,coef)
    sigma=max(float(r.get('residual',r.get('error',.045))) for r in pair)
    return dict(points=np.c_[z,x],source=source,coef=coef,sigma=sigma,support=[lo,hi],
        rule='Observed span only; no projection-clamping credit at either endpoint; not a tracking target')

try:
    from . import _progress
except ImportError:  # Unbuilt source checkout.
    _progress = None


def _segments(reference):
    segments = reference.get('_segments')
    if segments is None:
        points = np.asarray(reference['points'], dtype=float)
        edges = np.diff(points, axis=0)
        lengths = np.linalg.norm(edges, axis=1)
        segments = points, edges, lengths, np.r_[0., np.cumsum(lengths)]
        reference['_segments'] = segments
    return segments


def project(reference, z, x):
    if _progress is None:
        return reference_project(reference, z, x)
    last = reference.get('_last_projection')
    if last is not None and np.array_equal(z, last[0]) and np.array_equal(x, last[1]):
        return last[2]
    result = _progress.project(*_segments(reference), z, x, reference['coef'], reference['sigma'])
    reference['_last_projection'] = (z.copy(), x.copy(), result)
    return result


def reference_project(reference,z,x):
    points=np.asarray(reference['points']);edge=np.diff(points,axis=0);length=np.linalg.norm(edge,axis=1)
    query=np.c_[z,x];relative=query[:,None,:]-points[:-1][None,:,:]
    fraction=np.sum(relative*edge[None,:,:],axis=2)/np.maximum(length**2,1e-12)
    bounded=np.clip(fraction,0.,1.)
    nearest=points[:-1][None,:,:]+bounded[:,:,None]*edge[None,:,:]
    distance=np.sum((query[:,None,:]-nearest)**2,axis=2);index=np.argmin(distance,axis=1)
    i=np.arange(len(query));part=bounded[i,index]
    arc=np.r_[0.,np.cumsum(length)]
    value=arc[index]+part*length[index]
    supported=~(((index==0)&(fraction[i,index]<=0))|((index==len(length)-1)&(fraction[i,index]>=1)))
    prior=np.c_[np.full(len(query),np.inf),distance[:,:-1]]
    following=np.c_[distance[:,1:],np.full(len(query),np.inf)]
    local_min=(distance<=prior)&(distance<=following)
    all_arc=arc[:-1][None,:]+bounded*length[None,:]
    separated=abs(all_arc-value[:,None])>.16
    indistinguishable=np.sqrt(distance)<=np.sqrt(distance[i,index])[:,None]+reference['sigma']
    supported &= ~np.any(local_min&separated&indistinguishable,axis=1)
    # A nearest point can become ill-conditioned near the centre of curvature.
    # Its existing boundary-fit error must not span the Frenet singularity.
    zref=nearest[i,index,0];a,b,c=reference['coef']
    slope=b+2*c*zref;curvature=2*c/(1+slope*slope)**1.5
    normal=np.c_[-slope,np.ones(len(slope))]/np.sqrt(1+slope*slope)[:,None]
    offset=np.sum((query-nearest[i,index])*normal,axis=1)
    jacobian=1-curvature*offset
    supported &= jacobian>abs(curvature)*reference['sigma']
    return value,supported

def increment(obs,old_z,old_x,z,x,default):
    reference=getattr(obs,'reference',None)
    if reference is None:return default,np.zeros_like(default,bool)
    before,a=project(reference,old_z,old_x);after,b=project(reference,z,x)
    return supported_increment(reference,before,after,a,b,old_x,x,default)


def supported_increment(reference,before,after,a,b,old_x,x,default):
    valid=a&b
    points, _, _, arc = _segments(reference)
    u0=np.interp(before,arc,points[:,0]);u1=np.interp(after,arc,points[:,0])
    constant,linear,quadratic=reference['coef']
    stationary=-linear/(2*quadratic) if quadratic!=0 else 0.
    u=np.clip(stationary,np.minimum(u0,u1),np.maximum(u0,u1))
    f=constant+linear*u+quadratic*u*u
    slope=linear+2*quadratic*u
    # G(u)=0.5*((u-zq)^2+(f(u)-xq)^2). Strict convexity over the
    # intervening reference rules out switching closest-point branches.
    # Its second derivative is quadratic in u and affine in xq, so this
    # checks its exact interval minimum and both car-segment endpoints.
    lower=np.minimum(1+slope*slope+2*quadratic*(f-old_x),
                     1+slope*slope+2*quadratic*(f-x))
    continuous=lower>abs(2*quadratic)*reference['sigma']
    valid &= continuous
    return np.where(valid,after-before,default),valid
