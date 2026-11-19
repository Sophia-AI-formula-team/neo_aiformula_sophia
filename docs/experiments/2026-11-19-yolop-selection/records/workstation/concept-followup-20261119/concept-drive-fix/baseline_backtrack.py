"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/backtrack_forward617.py. """

import numpy as np

from concept_road.compare61513 import Guided

from concept_candidates.curve_progress import ReferenceObservation

from concept_feasibility.geometry61514 import road_clearance, path_obstacle_clearance

from concept_feasibility.reverse_model617 import step as advance_reverse

from concept_feasibility.response61515 import MODEL

def checked_retreat(obs,reverse_seconds,stop_age,horizon,previous_yaw=0.,reverse_steer=0.):
    z=x=angle=travel=turn=0.;v=obs.speed;w=obs.yaw;age=stop_age
    controls=[];paths=[];states=[tuple(np.array([q]) if k!=3 else q for k,q in enumerate([0.,0.,0.,0.,0.,0.]))]
    initial_gap,initial_lower,initial_known=road_clearance(obs.road,np.array([0.]),np.array([0.]),np.array([0.]),1.)
    lower=float(initial_lower[0]);geometric=float(initial_gap[0]);known=bool(initial_known[0]);max_speed=abs(v)
    steps=int(np.ceil(max(horizon,reverse_seconds+.5)/.05))
    for i in range(steps):
        q=-.2 if (i+.5)*.05<reverse_seconds else 0.
        release_from=reverse_steer if reverse_seconds>0 else previous_yaw
        release_step=max(0,i-int(round(reverse_seconds/.05)))
        steer=reverse_steer if q<0 else np.sign(release_from)*max(0.,abs(release_from)-1.8*(obs.dt+(release_step//4)*.2))
        controls.append([q,float(steer)]);ov,ow=v,w
        v,w,age=[float(a) for a in advance_reverse(v,w,q,steer,.05,age)]
        mid=angle+.25*(ow+w)*.05
        z+=(ov+v)*.5*np.cos(mid)*.05;x+=(ov+v)*.5*np.sin(mid)*.05
        angle+=(ow+w)*.5*.05;travel+=(abs(ov)+abs(v))*.5*.05;turn+=(abs(ow)+abs(w))*.5*.05
        rg,rl,k=road_clearance(obs.road,np.array([z]),np.array([x]),np.array([angle]),1.)
        lower=min(lower,float(rl[0]-.04*travel-.08*turn*1.16));geometric=min(geometric,float(rg[0]));known &= bool(k[0])
        max_speed=max(max_speed,abs(v));states.append(tuple(np.array([a]) if j!=3 else a for j,a in enumerate([z,x,angle,(i+1)*.05,travel,turn])))
        if (i+1)%4==0:paths.append([z,x])
    gap,obstacle=path_obstacle_clearance(obs.objects,states)
    # Inferred startup road may remain populated after historical rear cells
    # disappear. It can never authorize another powered reverse interval.
    observed_rear=(reverse_seconds==0 or getattr(obs,'rear_observed_limit',None) is not None)
    valid=lower>=0 and obstacle[0]>0 and known and observed_rear and abs(v)<.04 and max_speed<=.3
    return dict(valid=bool(valid),controls=np.asarray(controls),path=paths,road_lower=lower,
        obstacle_lower=float(obstacle[0]),geometric=min(geometric,float(gap[0])),known=known,
        maximum_speed=max_speed,travel_m=travel,checked_seconds=steps*.05,
        end_z=z,end_x=x,end_heading=angle)

def opening_target(obs):
    """Current observed and remembered openings rank checked reverse actions."""
    ahead=[o for o in obs.objects if o[0]>.5 and o[0]<=obs.road[-1,0]]
    if not ahead:return None
    nearest=min(ahead,key=lambda item:item[0]);depth=float(nearest[0])
    left=float(np.interp(depth,obs.road[:,0],obs.road[:,1]))+.38+float(np.interp(depth,obs.road[:,0],obs.road[:,3]))
    right=float(np.interp(depth,obs.road[:,0],obs.road[:,2]))-.38-float(np.interp(depth,obs.road[:,0],obs.road[:,4]))
    openings=[(left,right)] if right>left else []
    for obj in ahead:
        if abs(obj[0]-depth)>obj[4]+nearest[4]:continue
        lo,hi=obj[1]-obj[4]-.38-obj[5],obj[1]+obj[4]+.38+obj[5]
        parts=[]
        for a,b in openings:
            if lo>a:parts.append((a,min(b,lo)))
            if hi<b:parts.append((max(a,hi),b))
        openings=[(a,b) for a,b in parts if b>a]
    if not openings:return None
    a,b=max(openings,key=lambda pair:pair[1]-pair[0])
    return depth,(a+b)*.5

class BacktrackSteer:
    def __init__(self,base):
        self.base=base;self.last={};self.command=np.zeros(2);self.pending=None
        self.budget=None;self.used=0.;self.previous_speed=0.;self.zero_age=0.
        self.brake_until=0.;self.time=0.;self.phase='forward';self.reserved=None
        self.pose=np.zeros(3);self.previous_yaw=0.;self.entry=None
        self.event_travel=self.event_turn=0.

    def reserved_release(self):
        if self.reserved is None:return False
        record=self.reserved;offset=int(round((self.time-record['issued_at'])/.05))
        if record.get('release_at') is None:
            if offset not in record['release_controls']:return False
            record['release_at']=offset
        controls=record['release_controls'][record['release_at']]
        if offset>=len(controls):return False
        self.command=controls[offset].copy();self.phase='release'
        self.last=dict(reason='executing_checked_reverse_release',candidates=0,feasible=0,path=[],
            contingency_execution=True,recovery='Previously checked early release; original deadline retained',
            audit=dict(selected=None,table=[],paths=[],recovery_release=True),
            contingency=dict(issued_at=record['issued_at'],selected=0,mode='checked_early_reverse_release',
                brake_at=record['issued_at']+record['release_at']*.05,
                stop_at=record['issued_at']+len(controls)*.05,consumed_steps=offset,remaining_steps=len(controls)-offset))
        return True

    def step(self,obs):
        self.time+=obs.dt;self.pending=None
        distance=(self.previous_speed+obs.speed)*.5*obs.dt
        rotation=(self.previous_yaw+obs.yaw)*.5*obs.dt
        middle=self.pose[2]+rotation*.5
        self.pose[:2]+=distance*np.array([np.cos(middle),np.sin(middle)]);self.pose[2]+=rotation
        if self.entry is not None:
            self.event_travel+=(abs(self.previous_speed)+abs(obs.speed))*.5*obs.dt
            self.event_turn+=(abs(self.previous_yaw)+abs(obs.yaw))*.5*obs.dt
        self.previous_yaw=obs.yaw
        self.used+=(max(0.,-self.previous_speed)+max(0.,-obs.speed))*.5*obs.dt if self.budget is not None else 0.
        self.previous_speed=obs.speed
        self.zero_age=self.zero_age+obs.dt if self.command[0]==0 else 0.
        if len(obs.road)<3 or (self.phase!='forward' and getattr(obs,'rear_observed_limit',None) is None):
            if self.phase=='forward':
                self.base.command=self.command.copy()
                self.command=self.base.step(obs)
                self.last=self.base.last
                return self.command.copy()
            # Loss of rear evidence prohibits further reverse.  Once the
            # measured vehicle has stopped and the checked braking interval is
            # over, independently check a forward plan using current evidence.
            # A verified forward command does not depend on seeing behind.
            if (len(obs.road)>=3 and self.phase=='release' and abs(obs.speed)<.04
                    and self.time>=self.brake_until):
                forward=(ReferenceObservation(obs.road,obs.objects,max(0.,obs.speed),obs.yaw,obs.dt,obs.heading,
                         getattr(obs,'guidance',None),obs.reference) if hasattr(obs,'reference') else
                         Guided(obs.road,obs.objects,max(0.,obs.speed),obs.yaw,obs.dt,obs.heading,
                                getattr(obs,'guidance',None)))
                self.base.command=self.command.copy()
                candidate=self.base.step(forward)
                a=self.base.last.get('audit',{});selected=a.get('selected');table=np.asarray(a.get('table',[]))
                usable=(selected is not None and len(table)>selected and table[selected,6]>0
                        and candidate[0]>0 and not self.base.last.get('contingency_execution')
                        and np.allclose(candidate[:2],table[selected,:2],atol=1e-5,rtol=0))
                if usable:
                    self.phase='forward';self.reserved=None;self.command=candidate.copy();self.last=self.base.last
                    self.last['audit']['resume_after_rear_loss']='Stopped; new forward plan independently checked on current road'
                    return self.command.copy()
            if self.reserved_release():return self.command.copy()
            self.command=np.array([0.,0.])
            self.last=dict(reason='recovery_unknown_road',candidates=0,feasible=0,path=[],
                recovery='No observed rear corridor; release',audit=dict(selected=None,table=[],paths=[]))
            return self.command.copy()
        original=obs
        # Forward proposals are evaluated for the stopped state during retreat.
        # They are never issued until the calibrated release interval completes.
        forward=ReferenceObservation(obs.road,obs.objects,max(0.,obs.speed),obs.yaw,obs.dt,obs.heading,getattr(obs,'guidance',None),obs.reference) if hasattr(obs,'reference') else Guided(obs.road,obs.objects,max(0.,obs.speed),obs.yaw,obs.dt,obs.heading,getattr(obs,'guidance',None))
        self.base.command=self.command.copy()
        normal=self.base.step(forward);forward_count=self.base.last.get('feasible_forward',0)
        self.last=self.base.last
        table=np.asarray(self.last['audit']['table'])
        nominal=np.asarray(self.last['audit'].get('nominal_feasible',[]),bool)
        continuous=(table[:,0]>0)&nominal if len(table) and len(nominal)==len(table) else np.zeros(len(table),bool)
        eligible=((table[:,0]>0)&(table[:,6]>0)) if len(table) else np.zeros(0,bool)
        if self.phase!='forward' and eligible.any():
            # A checked short forward-and-stop profile can be a sensible way
            # out of a moving-obstacle encounter.  Prefer continuing paths when
            # available, but do not keep reversing solely because every legal
            # forward alternative includes a checked stop.
            choices=continuous if continuous.any() else eligible
            pick=int(np.argmax(np.where(choices,table[:,7],-np.inf)))
            pending=self.base.pending
            if pending['retimed'][pick]:controls=pending['profile_controls'][pick]
            elif pending['brake_ok'][pick]:controls=pending['brake_controls'][pick]
            else:controls=pending['escape_controls'][pick]
            normal=controls[0].copy();self.base.command=normal.copy()
            self.last['audit']['selected']=pick
            self.last['audit']['recovery_exit']='Use a checked forward option after measured stop; prefer continuous when available'
            self.last['path']=self.last['audit']['paths'][pick];self.last['target']=self.last['path'][-1]
            self.last['margin']=float(table[pick,5])
        if self.phase=='forward' and (forward_count or abs(obs.speed)>=.04 or self.base.last.get('contingency_execution')):
            if forward_count:
                self.reserved=None
                if self.entry is not None:
                    progress=float((self.pose[:2]-self.entry[:2])@np.array([np.cos(self.entry[2]),np.sin(self.entry[2])]))
                    error=.06+.04*self.event_travel+.08*self.event_turn*1.16
                    if progress>error:self.budget=None;self.used=0.;self.entry=None
            self.command=normal;return self.command.copy()
        if self.budget is None and abs(obs.speed)<.04:
            # Fixed at entry. Newly transported history cannot refresh this
            # retreat distance indefinitely. The rear bumper is 0.88m behind.
            rear=getattr(obs,'rear_observed_limit',None)
            self.budget=max(0.,-float(rear)-.88) if rear is not None else 0.
            self.used=0.
            self.entry=self.pose.copy();self.entry[2]+=float(np.interp(0.,obs.road[:,0],obs.heading))
            self.event_travel=self.event_turn=0.
        horizon=float(self.last['audit'].get('horizon',5.))
        want_forward=bool(eligible.any()) if self.phase!='forward' else forward_count>0
        reversing=(not want_forward and self.budget is not None and self.used<self.budget and obs.speed<.04)
        if reversing:
            stop=MODEL['stop'];stop_distance=.3*stop['delay']+.3**2/(2*stop['deceleration'])
            remaining=max(0.,self.budget-self.used-stop_distance)
            duration=min(1.,np.floor(remaining/.3/.2)*.2)
            slew=getattr(self.base,'p',{}).get('slew',1.8);limit=min(.18,slew*obs.dt/2.)
            candidates=[]
            for steer in [-limit,0.,limit]:
                if abs(steer-self.command[1])>slew*obs.dt+1e-8:continue
                candidate=checked_retreat(original,duration,self.zero_age,horizon,self.command[1],steer)
                valid=duration>=.2 and candidate['valid'] and self.used+candidate['travel_m']<=self.budget
                releases={}
                # Every possible next control-period release is a different motion.
                for cutoff in range(0,int(round(duration/.2))+1):
                    if not valid:break
                    branch=checked_retreat(original,cutoff*.2,self.zero_age,horizon,self.command[1],steer)
                    if not branch['valid']:valid=False;break
                    releases[cutoff*4]=branch['controls']
                if valid:
                    candidate['release_controls']=releases;candidate['reverse_steer']=steer;candidates.append(candidate)
            reversing=bool(candidates)
            if reversing:
                opening=opening_target(original)
                def cost(candidate):
                    if opening is None:return abs(candidate['reverse_steer'])
                    depth,target=opening
                    projected=candidate['end_x']+(depth-candidate['end_z'])*np.tan(candidate['end_heading'])
                    return abs(projected-target)
                check=min(candidates,key=cost)
        if not reversing:
            if self.phase!='forward' and abs(obs.speed)<.04 and self.time>=self.brake_until and want_forward:
                self.phase='forward';self.reserved=None;self.command=normal;return normal.copy()
            check=checked_retreat(original,0.,self.zero_age,horizon,self.command[1])
            check['release_controls']={i:check['controls'] for i in range(0,len(check['controls']),4)}
        if not check['valid']:
            self.command=np.array([0.,0.]);self.last=dict(reason='recovery_no_checked_motion',candidates=0,feasible=0,
                path=[],recovery='Unknown rear or occupied swept path; holding release',
                audit=dict(selected=None,table=[],paths=[],recovery_failed=dict(road=check['road_lower'],obstacle=check['obstacle_lower'],known=check['known'])))
            return self.command.copy()
        if reversing:
            self.phase='retreat'
            stop=MODEL['stop'];self.brake_until=self.time+stop['delay']+.3/stop['deceleration']
        else:self.phase='release'
        self.command=check['controls'][0].copy();self.pending=check
        # One explicitly checked recovery action; do not manufacture 75 unique
        # reverse motions from the one calibrated command.
        row=[*self.command,check['path'][-1][1],check['geometric'],0.,min(check['road_lower'],check['obstacle_lower']),1.,0.,0.,0.,0.,0.]
        self.last=dict(reason='checked_slow_reverse' if reversing else 'checked_reverse_release',candidates=1,feasible=1,
            feasible_forward=forward_count,path=check['path'],target=check['path'][-1],target_meaning='predicted endpoint',
            recovery=dict(phase=self.phase,remaining_historical_distance_m=max(0.,self.budget-self.used) if self.budget is not None else 0.,
                budget_fixed_at_entry=True,maximum_body_speed_mps=.3,forward_proposals_assume_stationary=obs.speed<0),
            audit=dict(selected=0,table=[row],paths=[check['path']],previous_command=self.base.last['audit'].get('previous_command',[0,0]),
                columns=['speed','yaw','target_x','geometric_clearance','sigma','robust_clearance','feasible','score','progress_score','heading_penalty','smooth_penalty','margin_penalty'],
                road_lower=[check['road_lower']],obstacle_lower=[check['obstacle_lower']],
                schedule=['reverse_1s_then_stop_and_wait' if reversing else 'release_and_wait'],
                obstacle_risk_limit=.022750131948179195,recovery_action=True,
                profile_checked_seconds=check['checked_seconds'],forward_candidate_count=75,
                rule='Recovery only with no forward choice; three bounded steering choices checked through full stop and wait; visible opening ranks valid actions'))
        return self.command.copy()

    def commit(self,index):
        if self.pending is None:
            if (self.last.get('audit',{}).get('recovery_release') or
                    self.last.get('reason') in ('recovery_unknown_road','recovery_no_checked_motion')):
                return
            self.base.commit(index);self.last=self.base.last;return
        if index!=0:raise ValueError('Only checked recovery action0 exists')
        controls=self.pending['controls'];self.reserved=dict(issued_at=self.time,controls=controls.copy(),
            release_controls=self.pending['release_controls'],release_at=None)
        self.last['contingency']=dict(issued_at=self.time,selected=0,mode='slow_reverse_then_stop',
            brake_at=self.time+float(np.sum(controls[:,0]<0))*.05,
            stop_at=self.time+len(controls)*.05,controls_005s=controls.tolist(),
            lower_m=min(self.pending['road_lower'],self.pending['obstacle_lower']))
