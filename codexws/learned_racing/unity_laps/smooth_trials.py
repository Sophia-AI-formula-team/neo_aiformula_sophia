"""Deterministic scenario generation and resumable real Unity/A4 action rollouts."""
import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path

import numpy as np
from learn_encounters import FROZEN, HERE, LENGTH, Lab, SIM, detector, write
from scenario_shots import geometry

ROOT=HERE/"smooth_revision"
V2=json.loads((HERE/"learned_risk_revision/model.json").read_bytes())
NAMES=["keep_v2","hold_line_brisk","minimum_left_brisk","minimum_right_brisk","buffer_left_moderate","buffer_right_moderate","hold_line_yield"]
FEATURES=["cross_m","motion_mps","range_m","projected_cross_m","history_s","fit_noise_m","count","min_cross","max_cross","spread_range","last_cross","last_motion","curvature_per_m","near_slope","far_slope","actual_speed","yaw_rate","current_offset"]

def bank():
    prev=json.loads((HERE/"learned_risk_revision/training.json").read_bytes())
    gaps=[r["minCone"] for group in prev["observedOutcomes"] for r in group if r["success"]]
    tight,buffer=np.clip(np.quantile(gaps,[.10,.40]),.005,.5)
    historic=[]
    for p in (HERE/"risk_revision/runs").glob("frozen_*_traffic.json"):
        r=json.loads(p.read_bytes())
        historic.extend(d["speed"] for d in r["decisions"] if d["speed"]>0.3)
    slow,moderate,brisk=np.quantile(historic,[.15,.45,.90])
    return [[0.,0.,0.],[0.,0.,brisk],[-1.,tight,brisk],[1.,tight,brisk],[-1.,buffer,moderate],[1.,buffer,moderate],[0.,0.,slow]]

def layouts(n,seed,prefix):
    rng=np.random.default_rng(seed);_,stations,_,curvature,length=geometry();out={}
    for i in range(n):
        mode=i%8; curved=(i//2)%2==1
        if i>=64:curved=not curved
        dist=float(rng.uniform(34.,46.) if i>=64 else rng.uniform(16.,23.))
        for attempt in range(10000):
            spawn=int(rng.integers(10000000,99999999))
            start=np.random.default_rng(spawn).random()*length
            idx=int(np.argmin(abs(stations-(start+dist)%length)))
            if (abs(curvature[idx])>.025)==curved:break
        orbital=lambda: [rng.uniform(.6,1.90),rng.uniform(.32,.79),rng.uniform(-np.pi,np.pi)]
        pos=float(rng.uniform(-.72,.72))
        if mode==0:props=[[dist,pos,0.,0.,0.]]
        elif mode==1:props=[[dist,rng.choice([-1,1])*rng.uniform(.8,1.7),0.,0.,0.]]
        elif mode==2:props=[[dist,pos,*orbital()]]
        elif mode==3:props=[[dist,rng.choice([-1,1])*rng.uniform(.75,1.65),*orbital()]]
        elif mode==4:props=[[dist,pos,0.,0.,0.],[dist+rng.uniform(2.5,5.),-pos+.25,0.,0.,0.]]
        elif mode==5:props=[[dist,pos,0.,0.,0.],[dist+rng.uniform(2.2,5.),-pos,*orbital()]]
        elif mode==6:props=[[dist,pos,*orbital()],[dist+rng.uniform(3.,5.5),-pos,*orbital()]]
        else:props=[[dist,pos,0.,0.,0.],[dist+rng.uniform(3.,5.),-pos,*orbital()],[dist+rng.uniform(7.2,10.8),pos-.3,0.,0.,0.]]
        case=dict(id=f"{prefix}{i+1:03}",seed=spawn,approach=0,limitM=max(o[0]for o in props)+(19. if i>=64 else 9.),props=props,
                  family=mode,inBend=bool(curved))
        out[case["id"]]=case
    return out

def run_one(lab,id_,case,cfg,port,release,film):
    if release!=FROZEN["model"]:raise ValueError("A4 release changed")
    return lab.run(id_,FROZEN["weights"],case["seed"],vision=port,media=film,limit=case["limitM"]/LENGTH,capture=.05,
                   model=release,obstacle_specs=[[p[0]/LENGTH,*p[1:]]for p in case["props"]],
                   timing=dict(mode="both",approach=case["approach"]),encounter=V2,smooth=cfg)

def training(args):
    root=args.output;lab=Lab(SIM,root);scenes=layouts(80,617934,"tr");actions=bank()
    write(root/"training_layouts.json",scenes);write(root/"action_bank.json",dict(actions=actions,names=NAMES,features=FEATURES))
    with detector(lab,"outer-edge","training") as (port,release):
        for number,case in enumerate(scenes.values(),1):
            if number<args.start or number>args.end:continue
            for a in range(len(actions)):
                cfg=dict(mode="probe",modelId="paired_practice",forced=a,actions=actions)
                run_one(lab,case["id"]+"_a"+str(a),case,cfg,port,release,False)

def development_cases():
    scenes=list(layouts(14,7481291,"d").values())
    # Long clear straights and whole laps provide a separate natural-pace
    # check. Geometry is confined to referee layout selection and scoring.
    _,stations,_,curvature,length=geometry()
    rng=np.random.default_rng(450921)
    for i in range(3):
        while True:
            spawn=int(rng.integers(10000000,99999999))
            start=np.random.default_rng(spawn).random()*length
            inds=np.searchsorted(stations,(start+np.arange(12.,38.))%length).clip(0,len(stations)-1)
            if np.max(abs(curvature[inds]))<=.015:break
        scenes.append(dict(id=f"dc{i+1:02}",seed=spawn,approach=0,limitM=42.,props=[]))
    for i in range(2):
        case=layouts(8,4891500+i,"x")[f"x00{6+i}"]; local=case["props"]
        props=[[v[0]+25.,*v[1:]] for v in local]+[[133.,.28,0.,0.,0.],[209.,-.35,1.20,.6,i+.45]]
        scenes.append(dict(id=f"dl{i+1:02}",seed=case["seed"],approach=0,limitM=LENGTH,props=props))
    return dict(schema=1,courseLengthForFractionsM=LENGTH,cases=scenes)

def evaluate(args):
    lab=Lab(SIM,args.output)
    if args.matrix:
        matrix=json.loads(args.matrix.read_bytes())
    else:
        matrix=development_cases();write(args.output/"development_layouts.json",matrix)
    assert abs(matrix["courseLengthForFractionsM"]-LENGTH)<.05
    model=json.loads(args.model.read_bytes()) if args.model else None
    with ExitStack() as stack:
        shared=None
        for i,case in enumerate(matrix["cases"]):
            if args.ids and case["id"] not in args.ids:continue
            label=case["id"];order=["base","learned"] if i%2==0 else ["learned","base"]
            if args.arm:order=[args.arm]
            for arm in order:
                if arm=="learned" and not model:raise ValueError("Frozen smooth model required")
                cfg=model if arm=="learned" else None
                if arm=="fixed":
                    if args.force is None or not 0<=args.force<len(NAMES):raise ValueError("Expected a training-selected fixed action")
                    cfg=dict(mode="probe",modelId="fixed_training_choice",forced=args.force,actions=bank())
                name=(args.tag+"_" if args.tag else "")+label+"_"+arm
                if args.film:
                    with detector(lab,"outer-edge",name,film=True)as (port,release):
                        run_one(lab,name,case,cfg,port,release,True)
                else:
                    if shared is None:shared=stack.enter_context(detector(lab,"outer-edge","evaluation"))
                    run_one(lab,name,case,cfg,*shared,False)

def parity(args):
    ref=json.loads((HERE/"learned_risk_revision/runs/diag_full01_learned.json").read_bytes())
    req=ref["request"];lab=Lab(SIM,args.output)
    with detector(lab,"outer-edge","parity") as (port,release):
        r=lab.run("default_v2_parity",FROZEN["weights"],req["seed"],vision=port,limit=1.,
                  capture=.05,model=release,obstacle_specs=req["obstacles"],timing=req["timing"],encounter=V2)
    same=r["rows"]==ref["rows"] and [{k:v for k,v in d.items() if k!="smooth"} for d in r["decisions"]]==ref["decisions"]
    write(args.output/"parity.json",dict(reference=ref["id"],previousSolver=req["solver"],solver=r["request"]["solver"],
          entirePhysicsAndVisionAndCommandsExact=same));assert same

def film_training(args):
    if not args.ids:raise ValueError("Choose a recorded training run ID")
    lab=Lab(SIM,args.output);scenes=json.loads((args.output/"training_layouts.json").read_bytes())
    for sample in args.ids:
        ref=json.loads((lab.raw/(sample+".json")).read_bytes());case=scenes[sample.split("_")[0]]
        name="film_"+sample
        with detector(lab,"outer-edge",name,film=True) as (port,release):
            r=run_one(lab,name,case,ref["request"]["smooth"],port,release,True)
        matched=r["rows"]==ref["rows"] and r["decisions"]==ref["decisions"]
        write(args.output/"film_replays"/(name+".json"),dict(trainingId=sample,replayId=name,
              matchesPhysicsAndPerceptionAndCommands=matched,rows=len(r["rows"]),decisions=len(r["decisions"])))
        assert matched

def main():
    p=argparse.ArgumentParser();p.add_argument("cmd",choices=["train","eval","parity","film_training"])
    p.add_argument("--start",type=int,default=1);p.add_argument("--end",type=int,default=80)
    p.add_argument("--output",type=Path,default=ROOT);p.add_argument("--matrix",type=Path)
    p.add_argument("--model",type=Path);p.add_argument("--film",action="store_true");p.add_argument("--tag",default="")
    p.add_argument("--ids",nargs="+")
    p.add_argument("--arm",choices=["base","learned","fixed"]);p.add_argument("--force",type=int)
    args=p.parse_args()
    if not os.environ.get("AIFORMULA_LAP_COPY"):raise RuntimeError("Private Unity only")
    if args.cmd=="train":return training(args)
    if args.cmd=="parity":return parity(args)
    if args.cmd=="film_training":return film_training(args)
    return evaluate(args)
if __name__=="__main__":main()
