"""Pack whole path histories in one array; each candidate keeps its own twenty bisections."""
import numpy as np
from scipy.special import ndtri
from concept_road.v3_model import BODY_HALF_WIDTH, BODY_FRONT, BODY_REAR
from . import _risk


def path_clearance(objects, states, active, limit):
    arrays = [np.stack([row[i] for row in states]) for i in (0, 1, 2, 4, 5)]
    t = np.array([row[3] for row in states])
    return array_clearance(objects, *arrays, t, active, limit)


def array_clearance(objects, z, x, heading, travel, turn, t, active, limit):
    nt, count = z.shape
    if not len(objects):
        return np.full(count, 100.), np.full(count, 100.)
    enabled = np.ones((nt, count), dtype=bool) if active is None else np.asarray(active, dtype=bool)
    if enabled.shape != (nt,count):
        raise ValueError('invalid active trajectory mask')
    c, s = np.cos(heading), np.sin(heading)
    tf = np.zeros((nt,2,4))
    tf[:,0,0] = tf[:,1,1] = 1.
    tf[:,0,2] = tf[:,1,3] = t
    geometric = np.full(count,100.)
    terms = np.empty((count,nt,2*len(objects),7))
    for i,row in enumerate(objects):
        if len(row)<69:
            raise ValueError('two-mode obstacle state required')
        dz=row[0]+row[2]*t[:,None]-z
        dx=row[1]+row[3]*t[:,None]-x
        along,side=dz*c+dx*s,dx*c-dz*s
        gap=np.hypot(along-np.clip(along,-BODY_REAR,BODY_FRONT),side-np.clip(side,-BODY_HALF_WIDTH,BODY_HALF_WIDTH))-row[4]
        geometric=np.minimum(geometric,np.min(np.where(enabled,gap,100.),axis=0))
        probability=row[25]*np.exp(-row[68]*t)
        for mode,weight,state,p,intensity in [(0,probability,row[26:30],row[30:46],row[67]),
                                               (1,1-probability,row[46:50],row[50:66],row[66])]:
            process=np.eye(2)*intensity**2 if mode==0 or len(row)<73 else row[69:73].reshape(2,2)
            times=t if mode==0 else t**3/3
            cov=tf@p.reshape(4,4)@np.swapaxes(tf,1,2)+process*times[:,None,None]
            zz=cov[:,0,0,None];xx=cov[:,1,1,None];xz=cov[:,0,1,None]
            if mode==0 and np.any(t>0):
                if len(row)<77:raise ValueError('static forecast requires measured motion budget')
                budget=.01*(row[73]+travel)+.02*(row[74]+turn)*(row[75]+travel)
                extra=np.where(t[:,None]>0,np.maximum(0.,budget**2-row[76]**2),0.)
                zz=zz+extra;xx=xx+extra
            dz=state[0]+state[2]*t[:,None]-z;dx=state[1]+state[3]*t[:,None]-x
            sz=np.sqrt(np.maximum(c*c*zz+2*c*s*xz+s*s*xx,1e-12))
            sx=np.sqrt(np.maximum(s*s*zz-2*c*s*xz+c*c*xx,1e-12))
            rho=np.clip((c*s*(xx-zz)+(c*c-s*s)*xz)/(sz*sx),-1.,1.)
            term=terms[:,:,2*i+mode]
            term[:,:,0]=weight
            term[:,:,1]=(dz*c+dx*s).T;term[:,:,2]=(dx*c-dz*s).T
            term[:,:,3]=sz.T;term[:,:,4]=sx.T
            term[:,:,5]=row[4]+.07;term[:,:,6]=rho.T
    # Additional 1 sigma makes the marginal precheck strictly conservative
    # even at the last representable bit of the CDF comparison.
    tail=-float(ndtri(limit/len(objects)))+1.
    return geometric,_risk.bisect(terms,np.ascontiguousarray(enabled,dtype=np.uint8),limit,tail,
                                   BODY_FRONT,BODY_REAR,BODY_HALF_WIDTH,4)
