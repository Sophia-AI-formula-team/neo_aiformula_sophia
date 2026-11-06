"""Select a pace and lateral-response blend using actual A4 RGB closed-loop training races."""
import json
import socket
import struct
import subprocess
import time
import numpy as np
from run import BASE,HERE,Lab,NAMES,SIM,write
from train import summarize

PRACTICE=(41021,5948)
QUALIFIER=33551
BLIND=(6329,78441,96527)

def search(lab,meta):
    root=lab.root;fast=np.array(json.loads((root/"joint_policy.json").read_bytes())["weights"]);slow=np.asarray(BASE)
    model="a4/single_frame/"+meta["sha256"];record=[];pool=[];rng=np.random.default_rng(16071)
    def trial(w,seed,name,gen,idx):
        r=lab.run(name,w,seed,True,vision=meta["port"],model=model)
        record.append(summarize(r,"vision",gen,idx));write(root/"camera_training.json",record);return r
    def pair(alpha,g,c):
        # The original trained policy defines the upper endpoint. Independently blend
        # pace, lateral line gains and obstacle-response gains towards a cautious prior.
        a,b,h=alpha
        blend=np.array([a,b,b,b,h,h,h,b,b]);w=slow+blend*(fast-slow)
        runs=[trial(w,s,f"rgb_train_g{g}_{c}_s{i}",g,c) for i,s in enumerate(PRACTICE)]
        return (score(runs),alpha,runs)
    def score(runs):
        fr=[min(1.,r["progress"]/r["length"]) for r in runs]
        return (sum(r["status"]=="lap" for r in runs),min(fr),sum(fr),-sum(r["seconds"] for r in runs))
    # The endpoints are controls. The other candidates are stratified across pace.
    for i,a in enumerate((0.,.23,.48,.73,1.)):
        alpha=np.repeat(a,3) if i in (0,4) else np.array([a,rng.uniform(.15,.85),rng.uniform(.4,1.)])
        pool.append(pair(alpha,1,i))
    elites=sorted(pool,key=lambda r:r[0],reverse=True)[:2]
    mean=np.mean([v[1] for v in elites],axis=0);sigma=np.maximum(np.std([v[1] for v in elites],axis=0),[.12,.18,.18])
    for i,a in enumerate(np.clip(rng.normal(mean,sigma,size=(3,3)),0,1)):pool.append(pair(a,2,i))
    checked=[]
    for i,(_,alpha,pair_runs) in enumerate(sorted(pool,key=lambda v:v[0],reverse=True)[:3]):
        check=trial(pair_runs[0]["request"]["weights"],QUALIFIER,f"rgb_qual_{i}",3,i)
        runs=pair_runs+[check];checked.append((score(runs),alpha,runs))
    _,alpha,runs=max(checked,key=lambda v:v[0])
    write(root/"camera_policy.json",dict(weights=runs[0]["request"]["weights"],fields=NAMES,source=runs[0]["id"],cross=runs[1]["id"],qualifier=runs[2]["id"],
            blend=alpha.tolist(),trainingSeeds=[*PRACTICE,QUALIFIER],holdoutSeeds=BLIND,model=model,solver=runs[0]["request"]["solver"]))
    for seed in BLIND:
        # The companion camera runner films all three frozen-policy holdout laps.
        r=lab.run(f"rgb_test_{seed}_initial",BASE,seed,True,vision=meta["port"],model=model)
        lab.event("camera_holdout",trial=r["id"],policy="initial")

def main():
    root=HERE/"evidence";lab=Lab(SIM,root);work=lab.work/"camera"/"training_a4";work.mkdir(parents=True,exist_ok=True)
    settings=json.loads((SIM/"Simulator/UserSettings/AIFormulaRoadDetector.json").read_bytes());ready=work/"ready.json";ready.unlink(missing_ok=True)
    with (work/"server.log").open("w",encoding="utf8") as log:
        proc=subprocess.Popen([settings["python"],"-B",str(HERE/"vision.py"),str(SIM/"Simulator/Assets/StreamingAssets/RoadDetector/a4"),str(ready),"--no-samples"],
                              stdout=log,stderr=log,creationflags=getattr(subprocess,"CREATE_NO_WINDOW",0))
        try:
            start=time.monotonic()
            while not ready.is_file():
                if proc.poll() is not None or time.monotonic()-start>120:raise RuntimeError("GPU worker startup failed: "+str(work))
                time.sleep(.25)
            meta=json.loads(ready.read_bytes());lab.event("camera_training_protocol",practice=PRACTICE,qualifier=QUALIFIER,blind=BLIND,model=meta["sha256"])
            search(lab,meta)
        finally:
            if proc.poll() is None and ready.exists():
                with socket.create_connection(("127.0.0.1",json.loads(ready.read_bytes())["port"]),timeout=2) as s:s.sendall(struct.pack("<I",0))
            try:proc.wait(timeout=6)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait()
if __name__=="__main__":main()
