"""Check measured lap gates, failure traces, blind splits and the timing of policy freezing."""
import hashlib
import json
import os
from pathlib import Path
import av
import numpy as np
from run import HERE,SIM,sha,write
from camera_training import PRACTICE,QUALIFIER,BLIND

def audit():
    evidence=HERE/"evidence";raw=evidence/"runs"
    runs={p.stem:json.loads(p.read_bytes()) for p in sorted(raw.glob("*.json"))}
    events=[json.loads(l) for l in (evidence/"events.jsonl").read_text(encoding="utf8").splitlines()]
    finished={e["trial"] for e in events if e["kind"]=="finish"}
    assert set(runs)<=finished
    for name,r in runs.items():
        rows=np.array(r["rows"])
        assert r["id"]==name and rows.shape[1]==15 and np.isfinite(rows).all(),name
        assert np.all(np.diff(rows[:,0])>0) and r["physicsStep"]==.01 and r["profile"]=="JanuaryLapIdentified" and r["coupled"],name
        if r["status"]=="lap":
            assert r["gate"]==32 and r["progress"]>=r["length"] and r["minRoad"]>=0 and r["minGap"]>0,name
        if r["status"]=="boundary":assert r["minRoad"]<0,name
        if r["status"]=="obstacle":assert r["minGap"]<=0,name
        for i,(a,b) in enumerate(zip(r["zoneStart"],r["zoneEnd"])):
            assert b<0 or (a>=0 and a<b<=r["seconds"]),name
    policy=json.loads((evidence/"camera_policy.json").read_bytes())
    train=json.loads((evidence/"camera_training.json").read_bytes())
    assert {r["seed"] for r in train}==set([*PRACTICE,QUALIFIER])
    assert not (set(BLIND)&{r["seed"] for r in train})
    starts=[i for i,e in enumerate(events) if e["kind"]=="start" and e["trial"].startswith("rgb_test_")]
    assert max(i for i,e in enumerate(events) if e["kind"]=="finish" and e["trial"].startswith("rgb_qual_"))<min(starts)
    paired=[]
    for seed in BLIND:
        a=runs[f"rgb_test_{seed}_initial"];b=runs[f"rgb_test_{seed}_learned"]
        for key in ("start","offset","yaw","obstacles","solver","model","vision"):
            assert a["request"][key]==b["request"][key],(seed,key)
        assert b["request"]["weights"]==policy["weights"] and len(b["props"])==4 and b["props"][3][4]>0 and b["props"][3][6]>0
        paired.append([seed,a["status"],b["status"]])
    private=Path(os.environ["AIFORMULA_LAP_COPY"]);live=SIM/"Simulator"
    source_paths=sorted(p.relative_to(private) for p in (private/"Assets/AIFormula/Runtime/Physics").rglob("*.cs"))
    source_paths+=list(map(Path,["Assets/Scenes/MotorControllerValidation.unity","Assets/TrainingEnvironment/Surface/track_surface.json",
                             "Assets/TrainingEnvironment/Surface/TrackMetricCalibration.json","Assets/TrainingEnvironment/Prefabs/TrafficCone_75cm.prefab",
                             "Assets/AIFormula/Runtime/Perception/AIFormulaRgbCameraTopic.cs"]))
    sources=[dict(file=p.as_posix(),privateHash=sha(private/p),liveHash=sha(live/p)) for p in source_paths]
    clips=[]
    for f in sorted((HERE/"replay").glob("*.mp4")):
        meta=json.loads(f.with_suffix(".json").read_bytes());r=runs[meta["id"]]
        with av.open(str(f)) as v:
            s=v.streams.video[0];n=sum(1 for _ in v.decode(s))
            assert (s.width,s.height)==(1280,720) and n==len(meta["simTimes"]) and max(meta["simTimes"])<=r["seconds"]+.01,f
            clips.append(dict(file=f.name,frames=n,bytes=f.stat().st_size,sha256=sha(f)))
    result=dict(trials=len(runs),practice=len(train)+len([r for f in ("training.json","joint_training.json")
                                  for r in json.loads((evidence/f).read_bytes()) if r["stage"] not in ("test","joint_test")]),
                allFinishEvents=True,allGateAndContactChecks=True,pairedBlind=paired,
                sameSimulatorSources=all(s["privateHash"]==s["liveHash"] for s in sources),sources=sources,videos=clips)
    write(evidence/"run_hashes.json",{p.stem:sha(p) for p in sorted(raw.glob("*.json"))})
    write(evidence/"validation.json",result);print({k:v for k,v in result.items() if k not in ("sources","videos")},"verified videos",len(clips))
if __name__=="__main__":audit()
