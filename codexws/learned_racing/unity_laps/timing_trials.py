"""Causal moving-cone timing tests; referee placements are never supplied to the policy."""
import argparse
import json
import os
from pathlib import Path

from risk_trials import HERE, Lab, SIM, detector, write
from scenario_shots import geometry

ROOT=HERE/"timing_revision"
FROZEN=json.loads((HERE/"risk_revision/frozen.json").read_bytes())

def layout(phase,case):
    # Distance from seeded spawn, not an observation available to the planner.
    length=geometry()[-1]
    spec=[24/length, -1.65, 2.05, .42, phase]
    if case=="static":
        spec=[24/length, -.35, 0., 0., 0.]
    if case=="blocked":
        spec=[24/length,-.35,.55,.42,phase]
    return [spec],37.0/length

def main():
    p=argparse.ArgumentParser()
    p.add_argument("ids",nargs="+",help="unique label:phase:case:modes:approach, e.g practice1:-4.4:cross:both:2.4")
    p.add_argument("--film",action="store_true")
    p.add_argument("--output",type=Path,default=ROOT,help="Use a fresh evidence directory to replay cached trials.")
    args=p.parse_args()
    if not os.environ.get("AIFORMULA_LAP_COPY"):
        raise RuntimeError("Private Unity project required")
    lab=Lab(SIM,args.output)
    def drive(entry,port,model):
        if entry=="default_reference":
            original=json.loads((HERE/"scenario_shots/runs/mixed_01.json").read_bytes())
            params=original["request"]
            replay=lab.run(entry,params["weights"],params["seed"],vision=port,media=args.film,
                           limit=params["limit"],capture=params["captureInterval"],model=model,
                           obstacle_specs=params["obstacles"])
            same_physics=original["rows"]==replay["rows"]
            same_commands=all(a=={k:v for k,v in b.items() if k!="timing"} for a,b in
                              zip(original["decisions"],replay["decisions"])) and len(original["decisions"])==len(replay["decisions"])
            write(lab.root/"default_regression.json",dict(reference=original["id"],referenceSolver=params["solver"],
                  replay=entry,simulatorRowsExact=same_physics,decisionTraceExact=same_commands))
            if not same_commands or not same_physics:raise AssertionError("Default policy differed from its frozen reference")
            return
        name,phase,case,mode,approach=entry.split(":")
        if case not in ("cross","static","blocked"):raise ValueError(case)
        if model!=FROZEN["model"]:raise RuntimeError("Camera model release mismatch")
        specs,limit=layout(float(phase),case)
        r=lab.run(name,FROZEN["weights"],25,vision=port,media=args.film,limit=limit,capture=.05,
                  model=model,obstacle_specs=specs,timing=dict(mode=mode,approach=float(approach)))
        acts=[(round(d["t"],1),round(d["speed"],2),round(d["timing"]["marginFast"],2),round(d["timing"]["marginPace"],2))
              for d in r["decisions"] if d["timing"]["offered"]]
        print("offers",acts,flush=True)
    if args.film:
        for entry in args.ids:
            with detector(lab,"outer-edge",entry.split(":")[0],film=True) as (port,model):
                drive(entry,port,model)
    else:
        with detector(lab,"outer-edge","practice") as (port,model):
            for entry in args.ids:drive(entry,port,model)
if __name__=="__main__":main()
