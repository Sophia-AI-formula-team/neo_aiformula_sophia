"""Explain the disclosed, previously failed B01 straight on both frozen clocks."""
import argparse
import hashlib
import json
import numpy as np
from lane_gate_trials import ROOT,write


def explain(run):
    rows=np.array(run["rows"]);ds=run["decisions"];t0,t1=np.interp([12.,34.],rows[:,7],rows[:,0])
    reasons={};limited=0.;bins=[];lowest=99
    for i,d in enumerate(ds):
        end=ds[i+1]["t"] if i+1<len(ds) else rows[-1,0]
        duration=max(0.,min(end,t1)-max(d["t"],t0))
        if not duration:continue
        reasons[d["reason"]]=reasons.get(d["reason"],0.)+duration
        if d["speedCmd"]<4:limited+=duration
        lowest=min(lowest,d["speedCmd"])
        if d.get("laneGate",{}).get("mask"):bins.append(dict(timeS=d["t"],mask=d["laneGate"]["mask"],curvature=d["curvature"]))
    return dict(gateMps=22/(t1-t0),gateEnterExitS=[t0,t1],reasonS=reasons,timeTargetBelow4S=limited,
                minimumSpeedTargetMps=lowest,gateFramesReplaced=bins)


def main():
    parser=argparse.ArgumentParser();parser.add_argument("referenceKnownCk64");a=parser.parse_args()
    root=ROOT/"known_b01";old_bytes=open(a.referenceKnownCk64,"rb").read();old=json.loads(old_bytes)
    raw_bytes=(root/"runs/b01_raw.json").read_bytes();raw=json.loads(raw_bytes)
    gate=json.loads((root/"runs/b01_gate.json").read_bytes())
    assert old["id"]=="b01_learned" and old["request"]["smooth"]==raw["request"]["smooth"]
    result=dict(referenceOriginalSHA256=hashlib.sha256(old_bytes).hexdigest(),newRawSHA256=hashlib.sha256(raw_bytes).hexdigest(),
                originalRawIdenticalPhysics=old["rows"]==raw["rows"],originalRawIdenticalDecisions=old["decisions"]==raw["decisions"],
                original=explain(old),newRaw=explain(raw),gated=explain(gate))
    write(root/"known_case_explanation.json",result)
if __name__=="__main__":main()
