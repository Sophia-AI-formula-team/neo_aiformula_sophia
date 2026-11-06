"""Score the separately locked closed-loop comparison, including all contacts."""
import json
import argparse
from pathlib import Path

import numpy as np
from lane_gate_trials import ROOT,THRESHOLDS,matrix,write
from learned_audit import images
from smooth_lane_diagnosis import clean,load_trial
from smooth_metrics import metrics


def read(root,case,suffix):
    path=root/"runs"/(case["id"]+"_"+suffix+".json")
    r=json.loads(path.read_bytes());log=np.array(r["rows"]);m=metrics(r)
    if case["id"].startswith("os") or case.get("straightCruiseGatesM")==[[12.,34.]]:
        enter,leave=np.interp([12.,34.],log[:,7],log[:,0])
        m["straightGateMps"]=22/(leave-enter)if log[-1,7]>=34 else None
    m["images"]=images(root,r)
    return path,r,m

def main():
    parser=argparse.ArgumentParser();parser.add_argument("--known-b01",action="store_true");args=parser.parse_args()
    root=ROOT/"known_b01" if args.known_b01 else ROOT
    cases=json.loads((root/"layouts.json").read_bytes()) if args.known_b01 else matrix()
    out={}
    for case in cases:
        _,raw,a=read(root,case,"raw");path,gate,b=read(root,case,"gate")
        same=dict(gate["request"]);same.pop("laneGateThresholdsM");assert same==raw["request"]
        replay=load_trial(path);filtered,mask=clean(replay,np.array(THRESHOLDS+[np.inf]))
        ds=gate["decisions"];logged=np.array([d["laneGate"]["filtered"] for d in ds],float)
        expected=mask@(2**np.arange(4))
        assert np.array_equal(expected,np.array([d["laneGate"]["mask"]for d in ds]))
        assert np.allclose(filtered,logged,atol=4e-6,equal_nan=True)
        first=next((d["t"]for d in ds if d["laneGate"]["mask"]),None)
        t=9999 if first is None else first
        before=lambda r:[row for row in r["rows"]if row[0]<=t+1e-6]
        assert before(raw)==before(gate),(case["id"],"physics changed before intervention")
        out[case["id"]]=dict(raw=a,gate=b,affectedFrames=int(np.any(mask,axis=1).sum()),totalFrames=len(ds),
                             firstFilteredT=first,identicalPhysicalPrefix=True,filtersVerifiedAgainstPython=True)
    write(root/"evaluation.json",dict(scenarios=len(out),actualArms=len(out)*2,knownDiagnostic=args.known_b01,model="ck64, both arms",
                                      thresholdOrigin="64 training layouts, locked source 46e64f5",pairs=out))
if __name__=="__main__":main()
