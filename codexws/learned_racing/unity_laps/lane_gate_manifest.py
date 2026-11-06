"""Check start/finish uniqueness and record every run and verification file hash."""
from collections import Counter
from pathlib import Path
import hashlib
import json
from lane_gate_trials import ROOT,write

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    checks={}
    for label,folder,count in [("unseen",ROOT,28),("known_b01",ROOT/"known_b01",2)]:
        files=sorted((folder/"runs").glob("*.json"));assert len(files)==count
        ev=[json.loads(l)for l in (folder/"events.jsonl").read_text().splitlines()]
        names=Counter({f.stem:1 for f in files})
        for kind in ["start","finish"]:assert Counter(e["trial"]for e in ev if e["kind"]==kind)==names
        report=json.loads((folder/"evaluation.json").read_bytes());assert report["actualArms"]==count
        checks[label]=dict(actualUnityRuns=count,uniqueStarts=count,uniqueFinishes=count,
                           rawSHA256={f.name:sha(f)for f in files},layoutsSHA256=sha(folder/"layouts.json"),
                           eventsSHA256=sha(folder/"events.jsonl"),evaluationAndImageManifestSHA256=sha(folder/"evaluation.json"))
    checksum=[ROOT/"implementation_check.json",ROOT/"known_b01/known_case_explanation.json",ROOT/"known_b01/speed_and_lane.png",
              ROOT.parent/"smooth_revision/lane_diagnosis.json",ROOT.parent/"smooth_revision/checkpoints/ck64/model.json",
              ROOT.parent/"learned_risk_revision/model.json"]
    write(ROOT/"manifest.json",dict(totalNewClosedLoopRuns=30,unseenCountsSeparatelyFromKnown=True,partitions=checks,
                                    artifacts={p.relative_to(ROOT.parent).as_posix():sha(p) for p in checksum}))
if __name__=="__main__":main()
