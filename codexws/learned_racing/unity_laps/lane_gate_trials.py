"""One locked A4 comparison of the optional odometry gate against frozen ck64."""
from contextlib import ExitStack
import argparse
import hashlib
import json
import os
from pathlib import Path

import numpy as np
from smooth_trials import FROZEN,HERE,LENGTH,Lab,SIM,V2,detector,geometry,layouts,write

ROOT=HERE/"lane_gate_revision"
THRESHOLDS=json.loads((HERE/"smooth_revision/lane_diagnosis.json").read_bytes())["thresholdsM"][:3]
CK64=json.loads((HERE/"smooth_revision/checkpoints/ck64/model.json").read_bytes())


def matrix():
    cases=list(layouts(8,51168097,"og").values())
    _,stations,_,curvature,_=geometry();rng=np.random.default_rng(6517329)
    for i in range(4):
        while True:
            seed=int(rng.integers(10000000,99999999))
            start=np.random.default_rng(seed).random()*LENGTH
            idx=np.searchsorted(stations,(start+np.arange(12.,35.))%LENGTH).clip(0,len(stations)-1)
            if max(abs(curvature[idx]))<=.015:break
        cases.append(dict(id=f"os{i+1:02}",seed=seed,approach=0,limitM=44,props=[]))
    cases.append(dict(id="ol01",seed=63849481,approach=0,limitM=LENGTH,props=[]))
    for_lap=layouts(8,3170841,"z")["z008"]
    cases.append(dict(id="ol02",seed=for_lap["seed"],approach=0,limitM=LENGTH,
                      props=[[p[0]+12,*p[1:]]for p in for_lap["props"]]+
                            [[113.,-.20,1.29,.48,1.10],[162.,.18,0.,0.,0.],[214.,.27,1.12,.56,-.70]]))
    return cases


def main():
    p=argparse.ArgumentParser();p.add_argument("--prepare",action="store_true");p.add_argument("--film",action="store_true")
    p.add_argument("--known-b01",type=Path,help="Separately replay a known original blind failure after primary evaluation")
    args=p.parse_args();cases=matrix();root=ROOT
    if args.known_b01:
        data=args.known_b01.read_bytes()
        assert hashlib.sha256(data).hexdigest()=="7fbc1a929be8622f0995ab98a478c336ebcb8b4964c3e36341953c52147d36e9"
        cases=[c for c in json.loads(data)["cases"]if c["id"]=="b01"]
        assert len(cases)==1
        root=ROOT/"known_b01"
    root.mkdir(exist_ok=True)
    matrix_path=root/"layouts.json"
    if matrix_path.exists():assert json.loads(matrix_path.read_bytes())==cases
    else:write(matrix_path,cases)
    if args.prepare:return
    if not os.environ.get("AIFORMULA_LAP_COPY"):raise RuntimeError("Use an isolated private Unity project")
    lab=Lab(SIM,root)
    with ExitStack()as stack:
        shared=None if args.film else stack.enter_context(detector(lab,"outer-edge","evaluation"))
        for i,case in enumerate(cases):
            for filtered in [False,True]if i%2==0 else [True,False]:
                id_=case["id"]+("_gate"if filtered else "_raw")
                with ExitStack() as scene:
                    port,release=shared or scene.enter_context(detector(lab,"outer-edge",id_,film=True))
                    assert release==FROZEN["model"]
                    lab.run(id_,FROZEN["weights"],case["seed"],vision=port,media=args.film,limit=case["limitM"]/LENGTH,
                            capture=.05,model=release,obstacle_specs=[[p[0]/LENGTH,*p[1:]]for p in case["props"]],
                            timing=dict(mode="both",approach=0),encounter=V2,smooth=CK64,
                            lane_gate=THRESHOLDS if filtered else None)
if __name__=="__main__":main()
