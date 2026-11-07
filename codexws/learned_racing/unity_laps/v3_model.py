"""V3: sensor-only, explicit trajectory prediction; procedural PRETRAINING plant.

The referee owns a generated full course. Planner.step receives only a local
Observation, never a Scenario, global pose, lap fraction, or future obstacles.
Parameters are interpretable CEM search variables, not an image-to-action net.
This surrogate is not Unity/PhysX and is not physical-vehicle validation.
"""
from dataclasses import dataclass
import math
import json
from pathlib import Path
import numpy as np

FIT_PATH=Path(__file__).resolve().parent/"v3_evidence/plant_fit.json"
PLANT_FIT=json.loads(FIT_PATH.read_bytes()) if FIT_PATH.exists() else None

def yaw_response(speed):
    if PLANT_FIT is None:raise RuntimeError("Run v3_calibrate.py before using the model")
    bands=PLANT_FIT["yaw_gain_by_speed"]
    return np.interp(speed,bands["speed"],bands["gain"])

BODY_HALF_WIDTH = .38
BODY_FRONT, BODY_REAR = .28, .88
DT = .20
KINDS = ("A", "B", "C", "sequence", "moving", "mixed", "bend", "clear", "missing")
DEFAULT_PARAMETERS = dict(pace=4.45, lookahead=1.1, steer_gain=2.0,
                         yaw_damping=.65, slew=1.8, clearance=.18,
                         uncertainty=1.8, smoothness=.12, response=.35)
PARAMETER_BOUNDS = dict(pace=(3.6,6.), lookahead=(.65,1.65), steer_gain=(1.,3.5),
                        yaw_damping=(.3,1.3), slew=(.7,3.8), clearance=(.04,.5),
                        uncertainty=(.8,3.), smoothness=(.02,.6), response=(.15,.75))

@dataclass(frozen=True)
class Observation:
    # Rows: forward distance, visible left/right bounds, lateral uncertainty.
    road: np.ndarray
    # Rows: forward, right, estimated forward/right velocity, radius, uncertainty.
    objects: np.ndarray
    speed: float
    yaw: float
    dt: float = DT

def candidate_targets(obs,p):
    """Sample visible free openings as well as broad pavement positions.

    Narrow cone gaps must be explicit candidates; a coarse lane grid could
    otherwise exclude the physically possible passage before learning begins.
    """
    road=obs.road
    look=float(np.clip(1.5+p["lookahead"]*max(obs.speed,1.),road[0,0],road[-1,0]-.2))
    ahead=[o[0] for o in obs.objects if o[0]>1.2]
    if ahead:look=min(look,max(2.,min(ahead)))
    left,right=(float(np.interp(look,road[:,0],road[:,i])) for i in (1,2))
    inset=BODY_HALF_WIDTH+p["clearance"]
    lo,hi=left+inset,right-inset
    base=np.linspace(lo,hi,9).tolist()+[float(np.clip(0,lo,hi))]
    openings=[(lo,hi)]
    for z,x,vz,vx,r,noise in obs.objects:
        if abs(z-look)>1.6:continue
        at=x+vx*max(0,z)/max(1.,obs.speed)
        radius=r+inset+p["uncertainty"]*noise
        next_open=[]
        for a,b in openings:
            if at-radius>a:next_open.append((a,min(b,at-radius)))
            if at+radius<b:next_open.append((max(a,at+radius),b))
        openings=[q for q in next_open if q[1]>q[0]]
    extras=[(a+b)/2 for a,b in sorted(openings,key=lambda q:q[1]-q[0],reverse=True)[:5]]
    extras += [base[-1]]*(5-len(extras))
    return look,np.sort(base+extras)

class Planner:
    """Predict several steer/speed actions with explicit footprint constraints.

    No target lane or return-to-reference cost. Visible road bounds only; the
    horizon stops at the last observed section, with braking for missing road.
    Predicted risk is an uncertainty margin, NOT a calibrated probability.
    """
    def __init__(self, parameters=None):
        self.p = {**DEFAULT_PARAMETERS, **(parameters or {})}
        self.command = np.zeros(2)
        self.last = {}

    def step(self, obs):
        if not isinstance(obs, Observation):
            raise TypeError("Planner accepts only the sensor Observation contract")
        road = np.asarray(obs.road, float)
        if len(road) < 3:
            self.command = np.array([max(0., self.command[0]-3.*obs.dt),
                                     self.command[1]*.65])
            self.last = dict(reason="unobserved_road", candidates=0)
            return self.command.copy()
        if not np.all(np.isfinite(road)) or np.any(np.diff(road[:,0]) <= 0):
            raise ValueError("Road observations must be finite and ordered")
        p=self.p
        look,targets=candidate_targets(obs,p)
        desired=p["steer_gain"]*max(obs.speed,1.)*2*targets/(look*look+targets*targets)
        desired-=p["yaw_damping"]*obs.yaw
        desired/=max(.05,float(yaw_response(min(obs.speed,3.5))))
        desired=np.clip(desired,-1.55,1.55)
        # Continuous controls, discretised for a bounded online search; candidate
        # limits and learned coefficients are saved in every model checkpoint.
        speeds=np.clip(np.array([.25,.55,.8,1.])*p["pace"],.35,6.)
        uu, ww=np.meshgrid(speeds,desired)
        uu=uu.ravel();ww=ww.ravel()
        ww=np.clip(ww,self.command[1]-p["slew"]*obs.dt,self.command[1]+p["slew"]*obs.dt)
        n=len(uu); x=np.zeros(n);z=np.zeros(n);h=np.zeros(n)
        v=np.full(n,max(0.,obs.speed),dtype=float);w=np.full(n,obs.yaw,dtype=float)
        clearance=np.full(n,np.inf); uncertainty=np.zeros(n)
        # Arc prediction under response lag. Stopping/finite visibility is part
        # of feasibility; unseen road is never silently extended into the horizon.
        horizon=float(np.clip((look+2.)/max(obs.speed,1.2),1.2,2.6));steps=12;dt=horizon/steps
        predicted=[]
        for j in range(steps):
            v += np.clip((np.minimum(uu,PLANT_FIT["body_speed_cap"])-v)/max(.15,p["response"]),-3.,2.)*dt
            w += np.clip((ww*yaw_response(v)-w)/max(.2,PLANT_FIT["yaw"]["tau"]),-4.,4.)*dt
            h+=w*dt;x+=v*np.sin(h)*dt;z+=v*np.cos(h)*dt
            # Body normal projected onto this camera frame (conservative width).
            half=BODY_HALF_WIDTH*np.abs(np.cos(h))+BODY_REAR*np.abs(np.sin(h))
            lo=np.interp(z,road[:,0],road[:,1]);hi=np.interp(z,road[:,0],road[:,2])
            sigma=np.interp(z,road[:,0],road[:,3])
            road_gap=np.minimum(x-half-lo,hi-x-half)
            road_gap=np.minimum(road_gap,road[-1,0]-z-BODY_FRONT)
            clearance=np.minimum(clearance,road_gap)
            uncertainty=np.maximum(uncertainty,sigma)
            for oz,ox,vz,vx,r,noise in obs.objects:
                dz=oz+vz*(j+1)*dt-z;dx=ox+vx*(j+1)*dt-x
                # Oriented rectangular vehicle vs circular cone, including rear.
                side=dx*np.cos(h)-dz*np.sin(h);along=dz*np.cos(h)+dx*np.sin(h)
                gap=np.hypot(np.maximum(abs(side)-BODY_HALF_WIDTH,0),
                             np.maximum(np.maximum(along-BODY_FRONT,-BODY_REAR-along),0))-r
                clearance=np.minimum(clearance,gap)
                uncertainty=np.maximum(uncertainty,noise+.04*(j+1)*dt)
            predicted.append(np.stack([z.copy(),x.copy()],axis=1))
        robust=clearance-p["uncertainty"]*uncertainty
        feasible=robust>=p["clearance"]
        # Short visible horizon: preference is distance/time, not a particular
        # lane. Jerk/steering cost suppresses needless straight oscillations.
        # Terminal heading follows visible pavement direction, without asking
        # the car to return to its centre or its original lane.
        slope=np.gradient((road[:,1]+road[:,2])*.5,road[:,0])
        road_heading=np.arctan(np.interp(z,road[:,0],slope))
        score=z/horizon-.8*v*(h-road_heading)**2-p["smoothness"]*((ww-self.command[1])**2+.15*ww**2)
        score-=.15*np.maximum(0,.5-robust)**2
        if np.any(feasible):score=np.where(feasible,score,-1e6)
        else:score=robust*100+score-.3*uu # retain failure, choose least harmful action
        pick=int(np.argmax(score));self.command=np.array([uu[pick],ww[pick]])
        self.last=dict(reason="feasible" if feasible[pick] else "no_feasible_trajectory",
                       candidates=n,feasible=int(feasible.sum()),margin=float(robust[pick]),
                       target=[float(look),float(targets[pick//len(speeds)])],
                       path=np.asarray(predicted)[:,pick,:].round(4).tolist())
        return self.command.copy()

@dataclass
class Scenario:
    center: np.ndarray
    width: float
    obstacles: list
    plant: dict
    seed: int
    kind: str
    start: int
    visibility: float
    dropout: float

def make_scenario(seed, kind=None):
    """Independent closed loops; map is environment/referee state only."""
    rng=np.random.default_rng(seed);kind=kind or KINDS[int(rng.integers(len(KINDS)))]
    if kind not in KINDS:raise ValueError(kind)
    a=np.linspace(0,2*np.pi,360,endpoint=False)
    radius=rng.uniform(13,22)*(1+rng.uniform(.03,.12)*np.sin(3*a+rng.uniform(0,6.28)))
    center=np.c_[radius*np.cos(a),radius*np.sin(a)]
    start=int(rng.integers(len(center)));width=float(rng.uniform(5.4,7.8))
    obstacles=[]
    if kind!="clear":
        stations=[(start+int(k*360))%360 for k in (.17,.43,.71)]
        if kind in ("A","B","C"):
            gap=rng.uniform(1.0,1.65) if kind!="B" else rng.uniform(.32,.69)
            if kind=="A": # wide pair leaves middle feasible and outside narrower
                width=float(rng.uniform(3.3,3.7))
            shift=rng.uniform(-.4,.4)
            for st in stations:
                for side in (-1,1):obstacles.append((st,shift+side*(gap/2+.34),0.,0.,0.))
        else:
            count=6 if kind in ("sequence","mixed") else 3
            for j in range(count):
                st=(start+int((.12+j*.12)*360))%360
                moving=kind=="moving" or (kind=="mixed" and j%2==1)
                obstacles.append((st,float(rng.uniform(-width*.28,width*.28)),
                                  float(rng.uniform(.5,1.3)) if moving else 0.,
                                  float(rng.uniform(.35,.85)),float(rng.uniform(0,6.28))))
    # This is a bounded perturbation of the Unity observed response, not a new
    # claim that the real car has these exact parameters.
    if PLANT_FIT is None:raise RuntimeError("Missing Unity response fit")
    plant=dict(tau=float(PLANT_FIT["speed"]["tau"]*rng.uniform(.65,1.6)),yaw_tau=float(PLANT_FIT["yaw"]["tau"]*rng.uniform(.65,1.4)),
               accel=float(rng.uniform(1.25,2.4)),brake=float(rng.uniform(2,4.2)),
               gain=float(rng.uniform(.87,1.03)),bias=float(rng.uniform(-.045,.045)),
               slip=float(rng.uniform(-.08,.10)),cap=4.5)
    return Scenario(center,width,obstacles,plant,int(seed),kind,start,
                    float(rng.uniform(7.5,12)),.17 if kind=="missing" else .025)

class LocalTracker:
    """Causal ego odometry from wheel/gyro; frame-local obstacle track history."""
    def __init__(self):self.items=[]
    def update(self,detections,speed,yaw,dt):
        angle=yaw*dt;c,s=math.cos(angle),math.sin(angle)
        old=[]
        for z,x,vz,vx,age in self.items:
            z-=speed*dt
            old.append([c*z+s*x,-s*z+c*x,c*vz+s*vx,-s*vz+c*vx,age+dt])
        result=[];taken=set()
        for z,x in detections:
            eligible=[(math.hypot(z-o[0],x-o[1]),i) for i,o in enumerate(old) if i not in taken]
            dist,i=min(eligible,default=(99,-1))
            vz=vx=0.
            if dist<.9:
                o=old[i];taken.add(i)
                vz=.55*o[2]+.45*(z-o[0])/dt;vx=.55*o[3]+.45*(x-o[1])/dt
            result.append([z,x,float(np.clip(vz,-2,2)),float(np.clip(vx,-2,2)),0.])
        result += [o for i,o in enumerate(old) if i not in taken and o[4]<.6 and o[0]>-.9]
        self.items=result
        return np.asarray([[z,x,vz,vx,.34,.06+age*.15] for z,x,vz,vx,age in result]).reshape(-1,6)

def rollout(scenario, parameters, seed, record=False):
    controller=Planner(parameters);simulation=rollout_steps(scenario,seed,record)
    try:
        obs=next(simulation)
        while True:
            command=controller.step(obs)
            obs=simulation.send((command,controller.last))
    except StopIteration as done:return done.value

def rollout_steps(scenario, seed, record=False):
    """Coroutine for batched GPU planning; yields only the sensor contract."""
    rng=np.random.default_rng(seed);track=scenario.center;n=len(track)
    direction=np.roll(track,-1,axis=0)-np.roll(track,1,axis=0)
    direction/=np.linalg.norm(direction,axis=1)[:,None]
    right=np.c_[direction[:,1],-direction[:,0]]
    i=scenario.start;pos=track[i]+right[i]*rng.uniform(-.35,.35)
    heading=math.atan2(direction[i,0],direction[i,1])+rng.uniform(-.055,.055)
    tracker=LocalTracker();v=w=progress=travel=steering=0.
    previous=i;trace=[];cause="timeout";plant=scenario.plant;min_gap=99.;command=np.zeros(2)
    lap_length=float(np.linalg.norm(np.roll(track,-1,axis=0)-track,axis=1).sum())
    for step in range(700):
        t=step*DT;f=np.array([math.sin(heading),math.cos(heading)]);r=np.array([f[1],-f[0]])
        indices=(previous+np.arange(-12,13))%n
        i=int(indices[np.argmin(np.sum((track[indices]-pos)**2,axis=1))])
        di=(i-previous+n//2)%n-n//2;progress+=di;previous=i
        off=float(np.dot(pos-track[i],right[i]))
        # Full geometry exists only in the environment and scorer.
        all_objects=[]
        for station,offset,amp,hz,phase in scenario.obstacles:
            at=track[station]+right[station]*(offset+amp*math.sin(t*hz+phase))
            delta=at-pos;oz=float(delta@f);ox=float(delta@r);all_objects.append([oz,ox])
            gap=math.hypot(max(abs(ox)-BODY_HALF_WIDTH,0),max(oz-BODY_FRONT,-BODY_REAR-oz,0))-.34
            min_gap=min(min_gap,gap)
            if gap<=0:cause="contact";break
        if cause=="contact":break
        # Same conservative body-edge criterion as previous Unity experiments.
        side_angle=math.acos(float(np.clip(np.dot(f,direction[i]),-1,1)))
        extent=BODY_HALF_WIDTH*abs(math.cos(side_angle))+BODY_REAR*abs(math.sin(side_angle))
        if abs(off)+extent>scenario.width/2:cause="departure";break
        if progress>=n:cause="lap";break
        # Symbolic camera emulator: visible drivable sections, no class/course ID.
        indices=(i+np.arange(-8,54))%n
        delta=track[indices]-pos;z=delta@f;x=delta@r
        visible=(z>=1.)&(z<=scenario.visibility)&(abs(x)<1.18*z)
        z=z[visible];x=x[visible]
        order=np.argsort(z);z=z[order];x=x[order]
        if len(z):
            unique=np.r_[True,np.diff(z)>.05];z=z[unique];x=x[unique]
        left=np.maximum(x-scenario.width/2,-1.18*z)
        bound=np.minimum(x+scenario.width/2,1.18*z)
        noise=rng.normal(0,.025,len(z));road=np.c_[z,left+noise,bound+noise,np.full(len(z),.035)]
        if rng.random()<scenario.dropout:road=road[::3]
        # Wheel error and raw gyro noise; neither global position nor orientation
        # crosses the Observation boundary. No GNSS/INS compensation is used.
        wheel=max(0,v*(1+plant["slip"])+rng.normal(0,.015));gyro=w+rng.normal(0,.008)
        measured=[]
        for oz,ox in all_objects:
            if .2<oz<scenario.visibility and abs(ox)<1.18*oz and rng.random()>.025:
                measured.append([oz+rng.normal(0,.025),ox+rng.normal(0,.02)])
        objects=tracker.update(measured,wheel,gyro,DT)
        new,decision=yield Observation(road,objects,wheel,gyro)
        steering+=abs(new[1]-command[1]);command=new
        # Explicit lag model, with small internal steps for swept collision evidence.
        for _ in range(4):
            ds=DT/4
            v+=np.clip((min(command[0]*plant["gain"],plant["cap"])-v)/plant["tau"],-plant["brake"],plant["accel"])*ds
            w+=np.clip((command[1]*float(yaw_response(v))+plant["bias"]*v/(1+v)-w)/plant["yaw_tau"],-4,4)*ds
            heading+=w*ds;pos+=v*np.array([math.sin(heading),math.cos(heading)])*ds;travel+=v*ds
        if record:trace.append(dict(t=round(t,3),pos=pos.round(4).tolist(),heading=heading,speed=v,command=new.tolist(),road=road.tolist(),objects=objects.tolist(),decision=decision))
    seconds=(step+1)*DT
    return dict(completed=cause=="lap",contact=cause=="contact",departure=cause=="departure",status=cause,
                time=seconds,progress=float(max(0,progress)/n),steering_tv=float(steering),travel=float(travel),
                lap_length=lap_length,min_gap=min_gap,kind=scenario.kind,seed=scenario.seed,
                mean_speed=float(travel/seconds),trace=trace if record else None)
