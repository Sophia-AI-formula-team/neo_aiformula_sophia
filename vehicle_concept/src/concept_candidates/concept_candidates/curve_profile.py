"""Runtime source extracted from the accepted V6.26 freeze: experiments/v616_decision/curve_profile.py. """

import numpy as np

try:
    from . import trajectory_batch as batch
except ImportError:
    batch = None

from concept_feasibility.response61515 import advance

from concept_candidates.curve_progress import increment

from concept_feasibility.geometry61514 import road_clearance, obstacle_clearance, path_obstacle_clearance

def evaluate_profile(obs,sequence,horizon,stop_age,uncertainty=1.):
    if batch is not None:
        return batch.profile(obs,np.asarray(sequence),horizon,stop_age,uncertainty)
    sequence=np.asarray(sequence);n=sequence.shape[0];dt=.05
    nominal_steps=int(round(horizon/dt));steps=max(nominal_steps,sequence.shape[1])
    z,x,heading,travel,turn,progress=[np.zeros(n) for _ in range(6)]
    speed,yaw=np.full(n,obs.speed),np.full(n,obs.yaw)
    zero_age=np.where(sequence[:,0,0]==0,stop_age,0.)
    geometric,road_lower,known=road_clearance(obs.road,z,x,heading,uncertainty)
    obj_gap,obstacle_lower=obstacle_clearance(obs.objects,z,x,heading,0.,uncertainty)
    geometric=np.minimum(geometric,obj_gap)
    states=[(z.copy(),x.copy(),heading.copy(),0.,travel.copy(),turn.copy())]
    paths=[];controls=[];finite=np.ones(n,bool);terminal=None;heading_integral=np.zeros(n);reference_steps=np.zeros(n,int)
    for i in range(steps):
        if i<sequence.shape[1]:drive,steer=sequence[:,i,:].T
        else:
            drive=np.zeros(n);steer=sequence[:,-1,1]*.65**((i-sequence.shape[1])//4+1)
        controls.append(np.c_[drive,steer])
        old_z,old_x,old_speed,old_yaw=z.copy(),x.copy(),speed.copy(),yaw.copy()
        speed,yaw,zero_age=advance(speed,yaw,drive,steer,dt,zero_age)
        mid=heading+.25*(old_yaw+yaw)*dt
        x+=(old_speed+speed)*.5*np.sin(mid)*dt;z+=(old_speed+speed)*.5*np.cos(mid)*dt
        heading+=(old_yaw+yaw)*.5*dt
        travel+=(abs(old_speed)+abs(speed))*.5*dt;turn+=(abs(old_yaw)+abs(yaw))*.5*dt
        finite &= np.isfinite(speed)&np.isfinite(yaw)
        if i<nominal_steps:
            direction=np.interp((old_z+z)*.5,obs.road[:,0],obs.heading)
            delta=(z-old_z)*np.cos(direction)+(x-old_x)*np.sin(direction)
            delta,used=increment(obs,old_z,old_x,z,x,delta)
            progress+=delta;reference_steps+=used
            heading_integral+=(old_speed+speed)*.5*(mid-direction)**2*dt
            if (i+1)%4==0 or i+1==nominal_steps:paths.append(np.c_[z.copy(),x.copy()])
        if i+1==nominal_steps:terminal=dict(z=z.copy(),x=x.copy(),heading=heading.copy(),speed=speed.copy())
        rg,rl,k=road_clearance(obs.road,z,x,heading,uncertainty)
        rl-=uncertainty*(.04*travel+.08*turn*1.16)
        geometric=np.minimum(geometric,rg);road_lower=np.minimum(road_lower,rl);known &= k
        states.append((z.copy(),x.copy(),heading.copy(),(i+1)*dt,travel.copy(),turn.copy()))
    object_gap,obstacle_lower=path_obstacle_clearance(obs.objects,states)
    geometric=np.minimum(geometric,object_gap)
    # An explicit stopping profile must actually reach the same stopped
    # threshold as the existing backup, not end while still driving.
    valid=(road_lower>=0)&(obstacle_lower>0)&known&finite&(speed<.04)&(abs(controls[-1][:,0])<1e-8)
    return dict(valid=valid,road_lower=road_lower,obstacle_lower=obstacle_lower,evidence=known,
        dynamics=finite,geometric=geometric,road_progress=progress,heading_integral=heading_integral,reference_steps=reference_steps,**terminal,
        paths=np.asarray(paths).transpose(1,0,2),controls=np.asarray(controls).transpose(1,0,2),
        checked_seconds=steps*dt)
