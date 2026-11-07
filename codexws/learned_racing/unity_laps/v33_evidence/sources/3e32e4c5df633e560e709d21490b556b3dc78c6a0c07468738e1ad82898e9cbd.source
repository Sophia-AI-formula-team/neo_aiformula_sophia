"""V3.3 explicit planner: physical-edge heading instead of FOV midpoint heading.
V3 coefficients remain frozen unless passed explicitly; no global state.
"""
import numpy as np
from v3_model import (Observation,DEFAULT_PARAMETERS,PLANT_FIT,yaw_response,
                      BODY_HALF_WIDTH,BODY_FRONT,BODY_REAR)

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

class Planner33:
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
        desired=np.clip(desired,-p.get("yaw_limit",1.55),p.get("yaw_limit",1.55))
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
        road_heading=np.interp(z,road[:,0],obs.heading) if getattr(obs,"heading",None) is not None else np.arctan(np.interp(z,road[:,0],slope))
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

