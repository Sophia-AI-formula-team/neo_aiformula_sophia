"""Run the same 50 ms integration and checks on whole control sequences."""
import numpy as np

from concept_feasibility import _motion
from concept_feasibility.geometry import road_clearance
from concept_feasibility.geometry61514 import RISK_LIMIT
from concept_feasibility.path_risk import array_clearance
from concept_feasibility.response61515 import MODEL, inverse_yaw
from .curve_progress import project, supported_increment

DT = .05
MOTOR = (np.asarray(MODEL['speed']), np.asarray(MODEL['yaw']), MODEL['stop']['delay'],
         MODEL['stop']['deceleration'])


def initial_state(obs, commands, stop_age, *, forward=False):
    state = np.zeros((len(commands),8))
    state[:,3] = max(0., obs.speed) if forward else obs.speed
    state[:,4] = obs.yaw
    state[:,7] = np.where(commands==0., stop_age, 0.)
    return state


def driven_path(obs, commands, slope, aa, bb, look, steps, previous, age, parameters):
    n=len(commands)
    state=np.empty((steps*4+1,n,8))
    controls=np.empty((steps*4,n,2))
    state[0]=initial_state(obs,commands,age,forward=True)
    limit=parameters.get('yaw_limit',1.55)
    last=np.full(n,previous[1])
    for k in range(steps):
        t=4*k
        z,x,heading,speed,yaw=state[t,:,:5].T
        pursuit=z+np.clip(1.+.7*speed,1.2,3.)
        q=np.minimum(pursuit,look)
        reference=aa*q**3+bb*q**2+np.maximum(pursuit-look,0)*slope
        dz,dx=pursuit-z,reference-x
        lateral=dx*np.cos(heading)-dz*np.sin(heading)
        desired=parameters['steer_gain']*np.maximum(speed,.5)*2*lateral/(dz*dz+dx*dx+.01)-parameters['yaw_damping']*yaw
        dt=obs.dt if k==0 else .2
        lo=np.maximum(-limit,last-parameters['slew']*dt);hi=np.minimum(limit,last+parameters['slew']*dt)
        last=inverse_yaw(desired,commands,lower=lo,upper=hi,current_yaw=yaw,dt=dt)
        controls[t:t+4,:,0]=commands;controls[t:t+4,:,1]=last
        state[t+1:t+5]=_motion.integrate(state[t],controls[t:t+4],DT,*MOTOR)[1:]
    # The frozen simulator's nominal horizon uses this particular evaluation order.
    ticks=np.arange(steps*4)
    times=np.r_[0.,(ticks//4)*.2+(ticks%4+1)*DT]
    g,margin,known,obs_margin=clearance(obs,state,times,parameters['uncertainty'])
    progress,used,heading_cost=progress_over(obs,state)
    return state, controls, g,margin,known,obs_margin,progress,used,heading_cost


def progress_over(obs,state):
    rows, count = state.shape[:2]
    z,x,h,v,yaw=(state[:,:,i] for i in range(5))
    direction=np.interp((z[:-1]+z[1:])*.5,obs.road[:,0],obs.heading)
    fallback=(z[1:]-z[:-1])*np.cos(direction)+(x[1:]-x[:-1])*np.sin(direction)
    reference=getattr(obs,'reference',None)
    if reference is None:
        delta=fallback
        used=np.zeros_like(delta,bool)
    else:
        arc,seen=project(reference,z.ravel(),x.ravel())
        arc,seen=arc.reshape(rows,count),seen.reshape(rows,count)
        delta,used=supported_increment(reference,arc[:-1],arc[1:],seen[:-1],seen[1:],x[:-1],x[1:],fallback)
    mid=h[:-1]+.25*(yaw[:-1]+yaw[1:])*DT
    error=(v[:-1]+v[1:])*.5*(mid-direction)**2*DT
    # The frozen loop adds time steps in order for each candidate.
    return np.add.accumulate(delta,axis=0)[-1],np.sum(used,axis=0),np.add.accumulate(error,axis=0)[-1]


def clearance(obs,state,times,uncertainty,enabled=None):
    shape=state.shape[:2]
    z,x,heading,travel,turn=(state[:,:,i] for i in (0,1,2,5,6))
    gap,lower,known=road_clearance(obs.road,z.ravel(),x.ravel(),heading.ravel(),uncertainty)
    gap=gap.reshape(shape)
    lower=lower.reshape(shape)-uncertainty*(.04*travel+.08*turn*1.16)
    known=known.reshape(shape)
    obj_gap,obj_lower=array_clearance(obs.objects,z,x,heading,travel,turn,times,enabled,RISK_LIMIT)
    if enabled is not None:
        lower=np.where(enabled,lower,100.)
        known=~enabled | known
    return np.minimum(gap.min(axis=0),obj_gap),lower.min(axis=0),known.all(axis=0),obj_lower


def braking(obs,commands,first,stop_age,uncertainty,coast_seconds,details,control_sequence,coast_sequence):
    n=len(commands)
    coast=int(round(coast_seconds/DT)) if coast_sequence is None else coast_sequence.shape[1]
    ticks=np.arange(104)
    controls=np.zeros((104,n,2))
    if control_sequence is not None:
        controls[:]=control_sequence[:,np.minimum(ticks,control_sequence.shape[1]-1)].transpose(1,0,2)
        last=np.max(np.where(abs(control_sequence[:,:,0])>1e-8,
                             np.arange(control_sequence.shape[1]),-1),axis=1)
    else:
        last=np.full(n,coast-1)
        cruise= min(coast,104)
        if coast_sequence is None:
            controls[:cruise,:,0]=commands
            controls[:cruise,:,1]=first
            endpoint=first
        else:
            controls[:cruise]=coast_sequence[:,:cruise].transpose(1,0,2)
            endpoint=coast_sequence[:,-1,1]
        controls[cruise:,:,1]=endpoint*.65**(((ticks[cruise:]-coast)//4+1)[:,None])
    states=_motion.integrate(initial_state(obs,commands,stop_age),controls,DT,*MOTOR)
    stopped=np.logical_or.accumulate((ticks[:,None]>last)&(abs(controls[:,:,0])<1e-8)&(states[1:,:,3]<.04),axis=0)
    all_stopped=np.flatnonzero(stopped.all(axis=1))
    nt=int(all_stopped[0])+1 if len(all_stopped) else len(controls)
    states,controls,stopped=states[:nt+1],controls[:nt],stopped[:nt]
    # The step at which a candidate first stops is checked, as is the initial pose.
    enabled=np.vstack((np.ones((2,n),bool),~stopped[:-1]))
    _,road,known,obstacle=clearance(obs,states,np.arange(nt+1)*DT,uncertainty,enabled)
    valid=known&(road>=0.)&(obstacle>0.)&stopped[-1]
    if details:
        return valid,np.minimum(road,obstacle),states[1:,:,:2].transpose(1,0,2),controls.transpose(1,0,2)
    return valid,np.minimum(road,obstacle)


def profile(obs,sequence,horizon,stop_age,uncertainty):
    n,original=sequence.shape[:2]
    nominal=int(round(horizon/DT)); nt=max(nominal,original)
    controls=np.zeros((nt,n,2))
    controls[:original]=sequence.transpose(1,0,2)
    ticks=np.arange(nt-original)
    controls[original:,:,1]=sequence[:,-1,1]*.65**((ticks//4+1)[:,None])
    state=_motion.integrate(initial_state(obs,controls[0,:,0],stop_age),controls,DT,*MOTOR)
    geometric,road,known,obstacle=clearance(obs,state,np.arange(nt+1)*DT,uncertainty)
    progress,used,cost=progress_over(obs,state[:nominal+1])
    finite=np.isfinite(state[1:,:,3:5]).all(axis=(0,2))
    valid=(road>=0)&(obstacle>0)&known&finite&(state[-1,:,3]<.04)&(abs(controls[-1,:,0])<1e-8)
    path_steps=np.unique(np.r_[np.arange(4,nominal+1,4),nominal])
    z,x,heading,speed=state[nominal,:,:4].T
    return dict(valid=valid,road_lower=road,obstacle_lower=obstacle,evidence=known,dynamics=finite,
                geometric=geometric,road_progress=progress,heading_integral=cost,reference_steps=used,
                z=z.copy(),x=x.copy(),heading=heading.copy(),speed=speed.copy(),
                paths=state[path_steps,:,:2].transpose(1,0,2),controls=controls.transpose(1,0,2),checked_seconds=nt*DT)
