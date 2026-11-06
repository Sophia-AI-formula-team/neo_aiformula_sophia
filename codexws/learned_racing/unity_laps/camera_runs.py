"""Replay a saved policy with real camera frames and pinned, single-frame GPU weights."""
import argparse
import json
import socket
import struct
import subprocess
import time
from pathlib import Path

from run import BASE,HERE,Lab,SIM,write


def main():
    p=argparse.ArgumentParser();p.add_argument("--release",choices=["a3-v14","a4"],required=True)
    p.add_argument("--name",required=True);p.add_argument("--policy",type=Path);p.add_argument("--seed",type=int,default=7116)
    p.add_argument("--baseline-of",type=Path,help="Use initial weights with this frozen policy's exact planner revision")
    p.add_argument("--ignore-objects",action="store_true",help="Ablate only the cone observations supplied to the controller")
    p.add_argument("--zero-forecast",action="store_true",help="Ablate only the lateral motion extrapolation gain")
    p.add_argument("--limit",type=float,default=1.);p.add_argument("--obstacles",action="store_true")
    p.add_argument("--capture",type=float,default=.20);args=p.parse_args()
    out=HERE/"evidence";reference=out/"runs/learn_base.json"
    record=args.policy or args.baseline_of
    if record:
        frozen=json.loads(record.read_bytes());reference=record.parent/"runs"/(frozen["source"]+".json")
    solver=json.loads(reference.read_bytes())["request"]["solver"] if reference.is_file() else None
    lab=Lab(SIM,out,solver)
    settings=json.loads((SIM/"Simulator/UserSettings/AIFormulaRoadDetector.json").read_bytes())
    weight_root=SIM/"Simulator/Assets/StreamingAssets/RoadDetector"/args.release
    work=lab.work/"camera"/args.name;work.mkdir(parents=True,exist_ok=True)
    ready=work/"ready.json";ready.unlink(missing_ok=True)
    policy=json.loads(args.policy.read_bytes())["weights"] if args.policy else BASE
    if args.zero_forecast:
        policy=list(policy);policy[6]=0
    with (work/"server.log").open("w",encoding="utf8") as log:
        proc=subprocess.Popen([settings["python"],"-B",str(HERE/"vision.py"),str(weight_root),str(ready),*(["--ignore-objects"] if args.ignore_objects else [])],
                              stdout=log,stderr=log,creationflags=getattr(subprocess,"CREATE_NO_WINDOW",0))
        try:
            began=time.monotonic()
            while not ready.exists():
                if proc.poll() is not None or time.monotonic()-began>120:raise RuntimeError((work/"server.log").read_text(encoding="utf8"))
                time.sleep(.25)
            meta=json.loads(ready.read_bytes());lab.event("camera_ready",trial=args.name,**meta)
            run=lab.run(args.name,policy,args.seed,args.obstacles,meta["port"],True,args.limit,args.capture,
                        model=f'{args.release}/single_frame/{meta["sha256"]}'+("/objects_muted" if args.ignore_objects else ""))
            write(out/"vision"/(args.name+".json"),dict(trial=run["id"],frames=run["seenFrames"],empty=run["empty"],cones=run["coneseens"],**meta))
        finally:
            if proc.poll() is None and ready.exists():
                with socket.create_connection(("127.0.0.1",json.loads(ready.read_bytes())["port"]),timeout=2) as sock:sock.sendall(struct.pack("<I",0))
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait()
if __name__=="__main__":main()
