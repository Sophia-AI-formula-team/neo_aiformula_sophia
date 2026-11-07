"""V3.2 GPU-batched white-box candidate prediction, same CPU sensor/plant core.

Each active environment supplies only Observation. CUDA evaluates 60 actions
per environment together; CPU advances the calibrated procedural plant. CEM
updates are inexpensive CPU operations. This is one GPU, not multiple GPUs.
"""
import time
import numpy as np
import torch
from v3_model import DEFAULT_PARAMETERS,PLANT_FIT,make_scenario,rollout_steps,candidate_targets

class BatchPlanner:
    def __init__(self,parameters,device="cuda",dtype=torch.float32,use_graph=False):
        if device=="cuda" and not torch.cuda.is_available():raise RuntimeError("CUDA unavailable; no silent CPU fallback")
        self.device=device;self.dtype=dtype
        self.names=list(DEFAULT_PARAMETERS)
        self.parameters=parameters
        self.p=torch.tensor([[p[n] for n in self.names] for p in parameters],device=device,dtype=dtype)
        self.command=torch.zeros((len(parameters),2),device=device,dtype=dtype)
        self.band_speed=torch.tensor(PLANT_FIT["yaw_gain_by_speed"]["speed"],device=device,dtype=dtype)
        self.band_gain=torch.tensor(PLANT_FIT["yaw_gain_by_speed"]["gain"],device=device,dtype=dtype)
        self.calls=0;self.candidate_evaluations=0
        self.use_graph=use_graph;self.graph=None
        self.ratios=torch.tensor([.25,.55,.8,1.],device=device,dtype=dtype)

    def tensor(self,a):return torch.as_tensor(np.asarray(a),device=self.device,dtype=self.dtype)
    def gain(self,v):
        idx=torch.searchsorted(self.band_speed,v.contiguous()).clamp(1,3)
        a=self.band_speed[idx-1];b=self.band_speed[idx];w=((v-a)/(b-a)).clamp(0,1)
        return self.band_gain[idx-1]*(1-w)+self.band_gain[idx]*w

    @torch.inference_mode()
    def step(self,observations,ids,updated=None):
        ids_t=torch.as_tensor(ids,device=self.device,dtype=torch.long)
        prior=self.command[ids_t];p=self.p[ids_t];B=len(ids)
        length=np.array([len(o.road) for o in observations]);valid=length>=3
        max_len=64 if self.use_graph else max(3,int(length.max()))
        if int(length.max())>max_len:raise ValueError("Road capacity exceeded")
        if self.use_graph and hasattr(self,"host_road"):
            road=self.host_road
        else:
            road=np.zeros((B,max_len,5),np.float32)
            if self.use_graph:self.host_road=road
        for i in (range(B) if updated is None else updated):
            o=observations[i];road[i]=0
            if len(o.road)<3:
                road[i,:,0]=np.arange(max_len)+1;road[i,:,1]=-1;road[i,:,2]=1
            else:
                n=len(o.road);road[i,:n,:4]=o.road
                road[i,:n,4]=np.gradient((o.road[:,1]+o.road[:,2])*.5,o.road[:,0])
                road[i,n:]=road[i,n-1];road[i,n:,0]+=np.arange(1,max_len-n+1)*.01
        r=self.tensor(road);ends=self.tensor([o.road[-1,0] if len(o.road)>=3 else 2 for o in observations])
        speed=self.tensor([o.speed for o in observations]);yaw=self.tensor([o.yaw for o in observations]);period=self.tensor([o.dt for o in observations])
        nobj=max([len(o.objects) for o in observations]+[0])
        slots=32 if self.use_graph else max(1,nobj)
        if nobj>slots:raise ValueError("Object capacity exceeded")
        objects=np.zeros((B,slots,6),np.float32);objects[:,:,0]=1e5
        for i,o in enumerate(observations):objects[i,:len(o.objects)]=o.objects
        cones=self.tensor(objects);valid_t=torch.as_tensor(valid,device=self.device)
        samples=[candidate_targets(o,self.parameters[i]) if len(o.road)>=3 else (2.,np.zeros(15)) for o,i in zip(observations,ids)]
        look=self.tensor([s[0] for s in samples]);targets=self.tensor([s[1] for s in samples])
        inputs=[r,ends,speed,yaw,period,prior,p,cones,valid_t,look,targets]
        if self.use_graph:
            if self.graph is None:
                self.buffers=[t.clone() for t in inputs]
                stream=torch.cuda.Stream();stream.wait_stream(torch.cuda.current_stream())
                with torch.cuda.stream(stream):
                    for _ in range(2):self.predict(*self.buffers)
                torch.cuda.current_stream().wait_stream(stream)
                self.graph=torch.cuda.CUDAGraph()
                with torch.cuda.graph(self.graph):self.output=self.predict(*self.buffers)
            else:
                for buffer,value in zip(self.buffers,inputs):buffer.copy_(value)
            self.graph.replay();command=self.output
        else:command=self.predict(*inputs)
        self.command[ids_t]=command
        self.calls+=1;self.candidate_evaluations+=B*60
        return command.cpu().numpy()

    def predict(self,r,ends,speed,yaw,period,prior,p,cones,valid_t,look,targets):
        B,max_len=r.shape[:2]
        def interp(z,column):
            z=torch.maximum(r[:,0,0,None],torch.minimum(z,ends[:,None]))
            idx=torch.searchsorted(r[:,:,0].contiguous(),z.contiguous()).clamp(1,max_len-1)
            lo=torch.gather(r[:,:,0],1,idx-1);hi=torch.gather(r[:,:,0],1,idx)
            weight=(z-lo)/(hi-lo).clamp_min(1e-6)
            return torch.gather(r[:,:,column],1,idx-1)*(1-weight)+torch.gather(r[:,:,column],1,idx)*weight
        pace,lookahead,steer,damping,slew,pad,risk,smooth,response=p.T
        desired=steer[:,None]*speed.clamp_min(1)[:,None]*2*targets/(look[:,None]**2+targets**2)-damping[:,None]*yaw[:,None]
        desired/=self.gain(speed.clamp_max(3.5)).clamp_min(.05)[:,None]
        desired=desired.clamp(-1.55,1.55)
        uu=(pace[:,None]*self.ratios).clamp(.35,6)[:,None,:].expand(-1,15,-1).reshape(B,60)
        ww=desired[:,:,None].expand(-1,-1,4).reshape(B,60)
        ww=torch.minimum(torch.maximum(ww,(prior[:,1]-slew*period)[:,None]),(prior[:,1]+slew*period)[:,None])
        x=torch.zeros_like(uu);z=torch.zeros_like(uu);h=torch.zeros_like(uu)
        v=speed.clamp_min(0)[:,None].expand(-1,60).clone();w=yaw[:,None].expand(-1,60).clone()
        clearance=torch.full_like(uu,10000.);uncertainty=torch.zeros_like(uu)
        horizon=((look+2)/speed.clamp_min(1.2)).clamp(1.2,2.6);dt=horizon[:,None]/12
        for j in range(12):
            v+=((uu.clamp_max(PLANT_FIT["body_speed_cap"])-v)/response.clamp_min(.15)[:,None]).clamp(-3,2)*dt
            w+=((ww*self.gain(v)-w)/max(.2,PLANT_FIT["yaw"]["tau"])).clamp(-4,4)*dt
            h+=w*dt;x+=v*torch.sin(h)*dt;z+=v*torch.cos(h)*dt
            half=.38*torch.cos(h).abs()+.88*torch.sin(h).abs()
            road_gap=torch.minimum(x-half-interp(z,1),interp(z,2)-x-half)
            road_gap=torch.minimum(road_gap,ends[:,None]-z-.28)
            clearance=torch.minimum(clearance,road_gap);uncertainty=torch.maximum(uncertainty,interp(z,3))
            if cones.shape[1]:
                oz,ox,vz,vx,radius,noise=cones.unbind(-1)
                dz=oz[:,None,:]+vz[:,None,:]*(j+1)*dt[:,:,None]-z[:,:,None]
                dx=ox[:,None,:]+vx[:,None,:]*(j+1)*dt[:,:,None]-x[:,:,None]
                side=dx*torch.cos(h)[:,:,None]-dz*torch.sin(h)[:,:,None]
                along=dz*torch.cos(h)[:,:,None]+dx*torch.sin(h)[:,:,None]
                gap=torch.hypot((side.abs()-.38).clamp_min(0),torch.maximum(along-.28,-.88-along).clamp_min(0))-radius[:,None,:]
                clearance=torch.minimum(clearance,gap.min(-1).values)
                noise=torch.where(oz<1e4,noise+.04*(j+1)*dt[:,0,None],torch.zeros_like(noise))
                uncertainty=torch.maximum(uncertainty,noise.max(-1).values[:,None])
        robust=clearance-risk[:,None]*uncertainty;feasible=robust>=pad[:,None]
        road_heading=torch.atan(interp(z,4))
        score=z/horizon[:,None]-.8*v*(h-road_heading)**2-smooth[:,None]*((ww-prior[:,1,None])**2+.15*ww**2)
        score-=.15*(.5-robust).clamp_min(0)**2
        score=torch.where(feasible.any(1)[:,None],torch.where(feasible,score,torch.full_like(score,-1e6)),robust*100+score-.3*uu)
        pick=score.argmax(1);index=torch.arange(B,device=self.device)
        command=torch.stack([uu[index,pick],ww[index,pick]],1)
        missing=torch.stack([(prior[:,0]-3*period).clamp_min(0),prior[:,1]*.65],1)
        return torch.where(valid_t[:,None],command,missing)

def run_jobs(jobs,device="cuda"):
    """Yield indexed actual rollout results; path candidates do not count as trials."""
    jobs=list(jobs);planner=BatchPlanner([j[3] for j in jobs],device=device,use_graph=True)
    simulations=[rollout_steps(make_scenario(j[1],j[2]),j[1]+100000000) for j in jobs]
    ids=list(range(len(jobs)));active=set(ids);observations=[next(s) for s in simulations]
    while active:
        commands=planner.step(observations,ids,updated=active)
        for i in list(active):
            try:
                observations[i]=simulations[i].send((commands[i],{}))
            except StopIteration as done:
                active.remove(i)
                index,seed,kind,params=jobs[i];result=done.value;result.pop("trace",None)
                yield dict(index=index,scenario_seed=seed,parameters=params,**result)
