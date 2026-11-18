"""Pack whole path histories in one array; each candidate keeps its own twenty bisections."""
import numpy as np
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
    # Covariance of each observed motion model at each future time is shared
    # by all 75 trajectories. Candidate-dependent rotations and motion error
    # budgets are evaluated in native worker-local memory.
    tf=np.zeros((nt,2,4));tf[:,0,0]=tf[:,1,1]=1.;tf[:,0,2]=tf[:,1,3]=t
    centers=np.empty((nt,len(objects),3))
    models=np.empty((nt,2*len(objects),12))
    for i,row in enumerate(objects):
        if len(row)<69:raise ValueError('two-mode obstacle state required')
        centers[:,i,0]=row[0]+row[2]*t;centers[:,i,1]=row[1]+row[3]*t;centers[:,i,2]=row[4]
        stationary=row[25]*np.exp(-row[68]*t)
        for mode,weight,state,p,intensity in [(0,stationary,row[26:30],row[30:46],row[67]),
                                                (1,1-stationary,row[46:50],row[50:66],row[66])]:
            process=np.eye(2)*intensity**2 if mode==0 or len(row)<73 else row[69:73].reshape(2,2)
            times=t if mode==0 else t**3/3
            cov=tf@p.reshape(4,4)@np.swapaxes(tf,1,2)+process*times[:,None,None]
            out=models[:,2*i+mode];out[:,0]=weight;out[:,1]=state[0]+state[2]*t;out[:,2]=state[1]+state[3]*t
            out[:,3]=cov[:,0,0];out[:,4]=cov[:,0,1];out[:,5]=cov[:,1,1];out[:,6]=row[4]+.07
            out[:,7]=0.
            if mode==0 and np.any(t>0):
                if len(row)<77:raise ValueError('static forecast requires measured motion budget')
                out[:,7]=t>0
                out[:,8]=row[73];out[:,9]=row[74];out[:,10]=row[75];out[:,11]=row[76]**2
    return _risk.path(models,centers,z,x,heading,travel,turn,np.ascontiguousarray(enabled,dtype=np.uint8),limit,
                      BODY_FRONT,BODY_REAR,BODY_HALF_WIDTH,4)
