"""Build a portable file:// report from completed physical trials, videos and source logs."""
import hashlib
import json
from pathlib import Path
import numpy as np
from render import HERE,OUT,RAW,track,write
from traffic_training import HOLDOUT
from camera_training import BLIND

def read(p):return json.loads(p.read_bytes())
def get(run):return read(RAW/(run+".json"))
def info(run):
    d=get(run);q=d["request"];rows=np.array(d["rows"])
    zones=[]
    for a,b in ((0,0),(1,2),(3,3)):
        if len(d["props"])>b:
            t0,t1=d["zoneStart"][a],d["zoneEnd"][b]
            zones.append(dict(time=round(t1-t0,2) if t0>=0 and t1>=0 else None,
                              started=t0>=0,clearance=round(min(d["propMin"][a:b+1]),3) if t0>=0 else None))
    return dict(id=run,status=d["status"],s=round(d["seconds"],2),p=round(d["progress"],2),g=d["gate"],seed=q["seed"],
                minRoad=round(d["minRoad"],3),minGap=round(d["minGap"],3) if d["props"] else None,zones=zones,
                start=round(q["start"],3),yaw=round(q["yaw"],2),model=q["model"],det=d["coneseens"],frames=d["seenFrames"],
                avg=round(d["progress"]/d["seconds"],2),peak=round(float(rows[:,4].max()),2),weights=q["weights"])

def training_data():
    trials=[]
    for log in ("training.json","joint_training.json","camera_training.json"):
        if not (HERE/"evidence"/log).exists():continue
        for r in read(HERE/"evidence"/log):
            if r["stage"] in ("test","joint_test"):continue
            trial=get(r["trial"]);rows=np.array(trial["rows"]);ids=np.unique(np.linspace(0,len(rows)-1,min(195,len(rows)),dtype=int))
            d=info(r["trial"]);d.update(stage=r["stage"],gen=r["generation"],candidate=r["candidate"],
                trace=rows[ids][:,[0,1,2,4,5,7,8,11]].round(3).tolist(),props=trial["props"])
            trials.append(d)
    return trials

def main():
    OUT.mkdir(exist_ok=True);holdout=[];visuals=[]
    for seed in HOLDOUT:
        row=dict(seed=seed)
        for tag,id_ in (("idealBefore",f"joint_test_{seed}_traffic_initial"),("idealAfter",f"joint_test_{seed}_traffic_learned"),
                        ("cameraBefore",f"base_a4_{seed}"),("cameraAfter",f"final_a4_{seed}"),
                        ("clearBefore",f"joint_test_{seed}_clear_initial"),("clearAfter",f"joint_test_{seed}_clear_learned")):
            row[tag]=info(id_)
        holdout.append(row)
    blind=[]
    if all((RAW/f"rgb_test_{s}_learned.json").exists() for s in BLIND):
        for seed in BLIND:blind.append(dict(seed=seed,cameraBefore=info(f"rgb_test_{seed}_initial"),cameraAfter=info(f"rgb_test_{seed}_learned")))
    for name in ("rgb-highlights","traffic-highlights","motion-ablation","rgb-blind-failure","rgb-full","traffic-full","rgb-clean","rgb-pretraining","camera-final-clear","pretraining-start","pretraining-fast","pretraining-winner","camera-failure","first-failure","camera-clear"):
        p=OUT/(name+".json")
        if p.exists():visuals.append(read(p))
    stages=training_data()
    clear=[info("base_a4_clear"),info("final_a4_clear")]
    if (RAW/"rgb_clear_6329_initial.json").exists():clear=[info("rgb_clear_6329_initial"),info("rgb_clear_6329_learned")]
    summary=dict(holdout=holdout,blind=blind,clearCamera=clear,a3=info("camera_a3_initial"),
                 trials=len(stages),cameraMode="A4 single-frame 5 Hz, actual Unity RGB",policy=read(HERE/"evidence/joint_policy.json"))
    if (HERE/"evidence/camera_policy.json").is_file():summary["cameraPolicy"]=read(HERE/"evidence/camera_policy.json")
    if (HERE/"evidence/ablation.json").is_file():
        ids=read(HERE/"evidence/ablation.json");summary["ablation"]={k:info(v) for k,v in ids["trials"].items()}
    write(HERE/"evidence/summary.json",summary)
    mesh=track()[:,::4,:].round(2).tolist()
    data=dict(**summary,training=stages,geometry=mesh,videos=visuals)
    template=(HERE/"replay_template.html").read_text(encoding="utf8")
    (OUT/"index.html").write_text(template.replace("__DATA_JSON__",json.dumps(data,ensure_ascii=False,separators=(",",":")).replace("<","\\u003c")),encoding="utf8")
    print("Report:",len(stages),"practice episodes",len(visuals),"videos",flush=True)

if __name__=="__main__":main()
