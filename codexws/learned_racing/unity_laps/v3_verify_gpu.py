"""CUDA/NumPy action parity on evolving, sensor-only inputs; no provider calls."""
import json
from pathlib import Path
import time
import numpy as np
import torch
from v3_model import DEFAULT_PARAMETERS,KINDS,Planner,make_scenario,rollout_steps
from v3_gpu import BatchPlanner

def main():
    torch.set_num_threads(2);count=32;ids=list(range(count))
    plants=[rollout_steps(make_scenario(880100+i,KINDS[i%len(KINDS)]),i) for i in ids]
    obs=[next(s) for s in plants];cpu=[Planner() for i in ids]
    gpu=BatchPlanner([DEFAULT_PARAMETERS]*count,use_graph=True)
    errors=[];comparisons=0
    for step in range(30):
        a=np.array([p.step(o) for p,o in zip(cpu,obs)]);b=gpu.step(obs,ids)
        error=float(abs(a-b).max());errors.append(error);comparisons+=len(a)
        if error>2e-4:raise AssertionError((step,error))
        for i in ids:
            try:obs[i]=plants[i].send((a[i],cpu[i].last))
            except StopIteration:
                plants[i]=rollout_steps(make_scenario(880100+i+step*40,KINDS[i%len(KINDS)]),i)
                obs[i]=next(plants[i]);cpu[i]=Planner();gpu.command[i]=0
    # Warm GPU graph vs one CPU planner per observation, same batch and state.
    t=time.perf_counter()
    for _ in range(20):gpu.step(obs,ids)
    torch.cuda.synchronize();cuda_seconds=time.perf_counter()-t
    t=time.perf_counter()
    for _ in range(20):
        for p,o in zip(cpu,obs):p.step(o)
    cpu_seconds=time.perf_counter()-t
    result=dict(device=torch.cuda.get_device_name(0),comparisons=comparisons,max_command_difference=max(errors),
                planner_only=dict(batch=count,repeats=20,cuda_seconds=cuda_seconds,single_process_cpu_seconds=cpu_seconds),
                caveat="Planner-only comparison; end-to-end CPU physics and multiprocessing timings are separate")
    dest=Path(__file__).resolve().parent/"v3_evidence/gpu_parity.json";dest.write_text(json.dumps(result,indent=2),encoding="utf8")
    print(json.dumps(result))
if __name__=="__main__":main()
