"""Runtime source extracted from the accepted V6.26 freeze: experiments/v615_recovery/geometry61514.py. """

import numpy as np

from scipy.special import ndtr

from concept_feasibility.geometry import road_clearance

from concept_road.v3_model import BODY_HALF_WIDTH, BODY_FRONT, BODY_REAR

try:
    from .path_risk import path_clearance as compiled_path_clearance
except ImportError:
    compiled_path_clearance = None

RISK_LIMIT=float(ndtr(-2.))

def rectangle_upper(z,x,sz,sx,rho,r):
    """Analytic upper bound with covariance; no integration grid parameter.

    P(A and B) <= P(A) sup_{a in A} P(B|a), and the symmetric expression.
    A Gaussian interval probability is maximized when its conditional mean is
    nearest the interval midpoint. That mean is linear in the conditioning
    coordinate, so its supremum on the entire interval is available exactly.
    Nested collision rectangles preserve monotonicity for the margin search.
    """
    a,b=-BODY_REAR-r,BODY_FRONT+r
    w=BODY_HALF_WIDTH+r
    pa=np.maximum(0.,ndtr((b-z)/sz)-ndtr((a-z)/sz))
    pb=np.maximum(0.,ndtr((w-x)/sx)-ndtr((-w-x)/sx))
    residual=np.maximum(0.,1-rho*rho)
    slope_x=rho*sx/sz
    m0,m1=x+slope_x*(a-z),x+slope_x*(b-z)
    closest_x=np.clip(np.zeros_like(m0),np.minimum(m0,m1),np.maximum(m0,m1))
    sigma_x=sx*np.sqrt(residual)
    sup_x=np.maximum(0.,ndtr((w-closest_x)/np.maximum(sigma_x,1e-12))-ndtr((-w-closest_x)/np.maximum(sigma_x,1e-12)))
    slope_z=rho*sz/sx
    n0,n1=z+slope_z*(-w-x),z+slope_z*(w-x)
    closest_z=np.clip((a+b)*.5,np.minimum(n0,n1),np.maximum(n0,n1))
    sigma_z=sz*np.sqrt(residual)
    sup_z=np.maximum(0.,ndtr((b-closest_z)/np.maximum(sigma_z,1e-12))-ndtr((a-closest_z)/np.maximum(sigma_z,1e-12)))
    # At singular or nearly singular covariance retain the old marginal upper
    # bound. A point mass on an interval endpoint must not become a half mass.
    conditional=np.minimum(pa*sup_x,pb*sup_z)
    bound=np.where(residual<=1e-12,np.minimum(pa,pb),np.minimum(np.minimum(pa,pb),conditional))
    return np.where((b>=a)&(w>=0),bound,0.)

def _components(objects,z,x,heading,time,future_travel=None,future_turn=None):
    c,s=np.cos(heading),np.sin(heading)
    parts=[]
    for row in objects:
        if len(row)<69:raise ValueError('Two-model obstacle schema required')
        probability=row[25]*np.exp(-row[68]*time)
        for weight,state,p,intensity,static in [
            (probability,row[26:30],row[30:46].reshape(4,4),row[67],True),
            (1-probability,row[46:50],row[50:66].reshape(4,4),row[66],False)]:
            f=np.c_[np.eye(2),np.eye(2)*time]
            process = np.eye(2)*intensity**2 if static or len(row)<73 else row[69:73].reshape(2,2)
            covariance=f@p@f.T+process*(time if static else time**3/3)
            dz,dx=state[0]+state[2]*time-z,state[1]+state[3]*time-x
            if static and time>0:
                if len(row)<77 or future_travel is None or future_turn is None:
                    raise ValueError('Static forecast requires the same cumulative ego-motion budget as the tracker')
                future_radius=row[75]+future_travel  # triangle-inequality bound along candidate
                budget=.01*(row[73]+future_travel)+.02*(row[74]+future_turn)*future_radius
                extra=np.maximum(0.,budget**2-row[76]**2)
                covariance=np.broadcast_to(covariance,(len(z),2,2)).copy()
                covariance[:,0,0]+=extra;covariance[:,1,1]+=extra
            along,side=dz*c+dx*s,dx*c-dz*s
            vz=c*c*covariance[...,0,0]+2*c*s*covariance[...,0,1]+s*s*covariance[...,1,1]
            vx=s*s*covariance[...,0,0]-2*c*s*covariance[...,0,1]+c*c*covariance[...,1,1]
            cross=c*s*(covariance[...,1,1]-covariance[...,0,0])+(c*c-s*s)*covariance[...,0,1]
            sz,sx=np.sqrt(np.maximum(vz,1e-12)),np.sqrt(np.maximum(vx,1e-12))
            rho=np.clip(cross/(sz*sx),-1.,1.)
            parts.append((weight,along,side,sz,sx,row[4]+.07,rho))
    return parts

def _risk(parts,inflation):
    risk=np.zeros_like(np.asarray(inflation,float))
    for weight,z,x,sz,sx,radius,rho in parts:
        risk+=weight*rectangle_upper(z,x,sz,sx,rho,radius+inflation)
    return np.minimum(risk,1.)

def obstacle_clearance(objects,z,x,heading,future_time,uncertainty_scale=1.,covariances=None,future_travel=None,future_turn=None):
    if not len(objects):return np.full(len(z),100.),np.full(len(z),100.)
    if any(len(o)<69 for o in objects):
        raise ValueError('Two-model obstacle schema required')
    if uncertainty_scale!=1.:raise ValueError('Risk model uses explicit probabilities, not an extra multiplier')
    c,s=np.cos(heading),np.sin(heading);geometric=np.full(len(z),100.)
    for row in objects:
        dz,dx=row[0]+row[2]*future_time-z,row[1]+row[3]*future_time-x
        along,side=dz*c+dx*s,dx*c-dz*s
        gap=np.hypot(along-np.clip(along,-BODY_REAR,BODY_FRONT),side-np.clip(side,-BODY_HALF_WIDTH,BODY_HALF_WIDTH))-row[4]
        geometric=np.minimum(geometric,gap)
    parts=_components(objects,z,x,heading,future_time,future_travel,future_turn)
    # Meter-valued diagnostic: extra enlargement of collision rectangle before
    # the bound reaches the risk limit. A negative value rejects the candidate.
    lower,upper=np.full(len(z),-8.),np.full(len(z),12.)
    for _ in range(20):
        middle=(lower+upper)*.5
        exceeds=_risk(parts,middle)>RISK_LIMIT
        upper=np.where(exceeds,middle,upper);lower=np.where(exceeds,lower,middle)
    return geometric,lower

def path_obstacle_clearance(objects,states,active=None):
    """Same discrete minimum as separate 20-iteration searches at every time.

    Each candidate has one shared inflation threshold. Requiring that threshold
    at ALL checked times is equivalent to taking their minimum. Object/mode
    summation order and finite bisection resolution remain exactly unchanged.
    """
    count=len(states[0][0]);times=len(states)
    if not len(objects):return np.full(count,100.),np.full(count,100.)
    if compiled_path_clearance is not None:
        return compiled_path_clearance(objects, states, active, RISK_LIMIT)
    active=np.ones((times,count),bool) if active is None else np.asarray(active,bool)
    at_times=[_components(objects,z,x,h,t,travel,turn) for z,x,h,t,travel,turn in states]
    parts=[]
    for index in range(len(at_times[0])):
        modes=[item[index] for item in at_times]
        parts.append((np.array([m[0] for m in modes])[:,None],
                      *[np.stack([m[k] for m in modes]) for k in range(1,5)],modes[0][5],
                      np.stack([m[6] for m in modes])))
    geometric=np.full(count,100.)
    for (z,x,h,t,travel,turn),enabled in zip(states,active):
        c,s=np.cos(h),np.sin(h)
        for row in objects:
            dz,dx=row[0]+row[2]*t-z,row[1]+row[3]*t-x
            along,side=dz*c+dx*s,dx*c-dz*s
            gap=np.hypot(along-np.clip(along,-BODY_REAR,BODY_FRONT),side-np.clip(side,-BODY_HALF_WIDTH,BODY_HALF_WIDTH))-row[4]
            geometric=np.minimum(geometric,np.where(enabled,gap,100.))
    lower,upper=np.full(count,-8.),np.full(count,12.)
    for _ in range(20):
        middle=(lower+upper)*.5
        risk=np.zeros((times,count))
        for weight,z,x,sz,sx,radius,rho in parts:
            risk+=weight*rectangle_upper(z,x,sz,sx,rho,radius+middle)
        exceeds=np.any(active & (np.minimum(risk,1.)>RISK_LIMIT),axis=0)
        upper=np.where(exceeds,middle,upper);lower=np.where(exceeds,lower,middle)
    return geometric,lower
