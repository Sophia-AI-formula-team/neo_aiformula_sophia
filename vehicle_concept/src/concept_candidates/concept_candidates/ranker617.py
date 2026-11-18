"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/ranker617.py. """

import numpy as np

DEPTHS=(1.,3.,5.,8.,10.)

CANDIDATE_FEATURES=24

ROAD_CONTEXT_FEATURES=35

def observation_features(obs,audit):
    table=np.asarray(audit['table'],float)
    objects=np.asarray(obs.objects,float).reshape(-1,77)
    road=np.asarray(obs.road,float)
    if table.size==0:return np.empty((0,CANDIDATE_FEATURES),np.float32),objects.astype(np.float32),np.zeros(ROAD_CONTEXT_FEATURES,np.float32),np.zeros(0,bool)
    if table.shape!=(75,12) or road.ndim!=2 or road.shape[1]<5 or len(road)<3:
        raise ValueError('Unexpected live candidate or road format')
    paths=np.asarray(audit['paths'],float)
    if paths.ndim!=3 or paths.shape[0]!=75 or paths.shape[2]!=2:raise ValueError('Expected 75 predicted paths')
    index=np.linspace(0,paths.shape[1]-1,5).round().astype(int)
    samples=paths[:,index,:]/np.array([12.,6.]);nominal=np.asarray(audit['nominal_feasible'],bool)
    if len(nominal)!=75:raise ValueError('Missing continuous-path classification')
    schedules=audit['schedule'];retimed=np.array([x.startswith('drive_') and x.endswith('s_then_stop_and_wait') for x in schedules],float)
    drive_seconds=np.array([float(x.split('_')[1][:-1]) if flag else 0. for x,flag in zip(schedules,retimed)])
    candidate=np.c_[table[:,0]/4.,table[:,1]/1.55,table[:,2]/6.,table[:,3]/5.,table[:,4]/5.,table[:,5]/5.,
        table[:,8]/4.,np.asarray(audit['road_lower'])/5.,np.asarray(audit['obstacle_lower'])/5.,nominal,retimed,drive_seconds/5.,
        samples.reshape(75,10),np.full(75,audit['horizon']/5.),np.full(75,audit['profile_checked_seconds']/8.)]
    context=[]
    for z in DEPTHS:
        present=road[0,0]<=z<=road[-1,0]
        if present:context.extend([np.interp(z,road[:,0],road[:,1])/6.,np.interp(z,road[:,0],road[:,2])/6.,
            np.interp(z,road[:,0],obs.heading)/np.pi,np.interp(z,road[:,0],road[:,3])/2.,np.interp(z,road[:,0],road[:,4])/2.,1.])
        else:context.extend([0.]*6)
    previous=audit['previous_command'];context.extend([obs.speed/4.,obs.yaw,obs.dt/.2,previous[0]/4.,previous[1]/1.55])
    factors=np.ones(77);factors[:6]=[12.,6.,3.,3.,.5,3.]
    state_scale=np.array([12.,6.,3.,3.]);factors[6:22]=np.outer(state_scale,state_scale).ravel()
    factors[22:25]=[5.,10.,1.];factors[26:30]=state_scale;factors[30:46]=np.outer(state_scale,state_scale).ravel()
    factors[46:50]=state_scale;factors[50:66]=np.outer(state_scale,state_scale).ravel()
    object_features=objects/factors
    if candidate.shape!=(75,CANDIDATE_FEATURES) or len(context)!=ROAD_CONTEXT_FEATURES:raise AssertionError('Feature schema changed')
    if not np.isfinite(candidate).all() or not np.isfinite(object_features).all() or not np.isfinite(context).all():raise ValueError('Non-finite sensor-derived features')
    return np.clip(candidate,-10,10).astype(np.float32),np.clip(object_features,-10,10).astype(np.float32),np.clip(context,-10,10).astype(np.float32),table[:,6].astype(bool)

def choose(values,allowed):
    allowed=np.asarray(allowed,bool);values=np.asarray(values)
    if values.shape!=allowed.shape or values.ndim!=1:raise ValueError('Value and mask shape mismatch')
    choices=np.flatnonzero(allowed)
    if not len(choices):return None
    if not np.isfinite(values[choices]).all():raise ValueError('Non-finite value for a feasible candidate')
    return int(choices[np.argmax(values[choices])])
