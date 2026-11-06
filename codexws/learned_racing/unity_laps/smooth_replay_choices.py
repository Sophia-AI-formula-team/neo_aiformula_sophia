"""Pick illustrative, labelled training replays by a reproducible rule."""
import argparse
import json
import numpy as np
from smooth_trials import ROOT,write
def select(root,size):
    ck=json.loads((root/"checkpoints"/f"ck{size:02}"/"training.json").read_bytes())
    cases=ck["outcomes"];eligible=[];rescued=[]
    for group in cases:
        base=group["outcomes"][0]
        for a,q in enumerate(group["outcomes"][1:],1):
            if not q["completed"]:continue
            if not base["completed"]:
                rescued.append((q["minConeM"],group["id"],a));continue
            if q["minConeM"]>.50 or q["minConeM"]<0:continue
            h=q["headingResidualTVperM"]/max(1e-8,base["headingResidualTVperM"])
            w=q["yawCommandTVperM"]/max(1e-8,base["yawCommandTVperM"])
            if h<1 and w<1 and q["zoneTimeS"]<=base["zoneTimeS"]:eligible.append((.5*(h+w),group["id"],a))
    selected=[]
    for reason,entries in [("median jointly smoother and no slower pass within 0.5 m",eligible),
                           ("median measured clearance among alternatives that rescued reference contact",rescued)]:
        if not entries:continue
        _,name,a=sorted(entries)[len(entries)//2];selected.append(dict(why=reason,population=len(entries),runs=[name+"_a0",name+"_a"+str(a)]))
    write(root/"training_replay_choices.json",selected)
    return selected
if __name__=="__main__":
    from pathlib import Path
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path,default=ROOT);parser.add_argument("--size",type=int,default=80)
    a=parser.parse_args();print(select(a.output,a.size))
