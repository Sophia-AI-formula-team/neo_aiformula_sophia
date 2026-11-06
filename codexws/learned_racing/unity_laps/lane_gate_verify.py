"""Validate the actual C# gate against the Python offline replay on training data."""
import json
import os
from pathlib import Path
import subprocess

import numpy as np
from smooth_lane_diagnosis import clean,load_trial
from lane_gate_trials import ROOT,THRESHOLDS,write

def main():
    p=Path(os.environ["AIFORMULA_LAP_COPY"])/"LabQueue"/"owner.json"
    engine=Path(json.loads(p.read_bytes())["engine"]).parent
    mono=engine/"Data/MonoBleedingEdge/bin/mono.exe";compiler=engine/"Data/MonoBleedingEdge/lib/mono/4.5/mcs.exe"
    work=ROOT/".work";work.mkdir(exist_ok=True)
    exe=work/"lane_gate_test.exe"
    subprocess.run([mono,compiler,"-out:"+str(exe),ROOT/"test_harness/Replay.cs",ROOT.parent/"OdometryLaneGate.cs"],check=True)
    limits=np.array(THRESHOLDS+[np.inf]);folder=Path(os.environ["LANE_GATE_TRAINING_RUNS"])
    masks=[];lines=[];inputs=[" ".join(map(str,THRESHOLDS))];paths=[p for p in sorted(folder.glob("tr???_a?.json"))if int(p.stem[2:5])>64]
    assert len(paths)==112
    for path in paths:
        inputs.append("reset");ds=json.loads(path.read_bytes())["decisions"]
        for d in ds:inputs.append(" ".join("NaN"if x is None else str(x)for x in [d["t"],d["speed"],d["yaw"],*d["observed"]]))
        c,h=clean(load_trial(path),limits);masks.extend((h@(2**np.arange(4))).tolist());lines.extend(c)
    p=subprocess.run([mono,exe],input=("\n".join(inputs)+"\n").encode("ascii"),capture_output=True,check=True)
    cs=np.array([[float(v)for v in l.split()]for l in p.stdout.decode("utf-8-sig").splitlines()])
    delta=np.nanmax(abs(cs[:,1:]-np.array(lines)))
    mismatch=int((cs[:,0]!=np.array(masks)).sum());missing=int((np.isnan(cs[:,1:])!=np.isnan(lines)).sum())
    assert mismatch==0 and missing==0 and delta<2e-5,(mismatch,missing,delta)
    write(ROOT/"implementation_check.json",dict(layouts=16,runs=len(paths),frames=len(masks),maskMismatch=mismatch,
                                                maxBeamDifferenceM=float(delta),missingMismatch=missing))
if __name__=="__main__":main()
