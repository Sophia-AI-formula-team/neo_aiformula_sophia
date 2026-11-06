"""Independent paired holdout audit for smooth local driving, cruising and whole laps."""
import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np

from learned_holdout_score import first_control_change
from smooth_window_metrics import body_speed, extract, intervals_report, obstacle_windows

HERE=Path(__file__).resolve().parent
PROTOCOL=HERE/"smooth_holdout"


def load(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def release_audit(run_dir, model_path):
    root = run_dir.parent
    freeze = load(root / "frozen.json")
    original = load(HERE / "risk_revision/frozen.json")
    selected = load(model_path)
    if sha(model_path) != freeze["modelFileSHA256"] or selected["modelId"] != freeze["selectedModel"]:
        raise ValueError("Holdout policy differs from the sealed model")
    canonical = {key: value for key, value in selected.items() if key != "modelId"}
    canonical_sha = hashlib.sha256(json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    if canonical_sha != selected["modelId"]:
        raise ValueError("Frozen model ID differs from its actual exported coefficients")
    development = root / "development.json"
    if freeze["devSHA256"] != sha(development):
        raise ValueError("The development decision file changed after freezing")
    for name, expected in freeze["developmentTrialSHA256"].items():
        if sha(run_dir / name) != expected:
            raise ValueError("Development evidence changed: " + name)
    return dict(frozenSha256=sha(root / "frozen.json"), canonicalModelSha256=canonical_sha,
                checkpoint=freeze["checkpoint"], sourceCommit=freeze["sourceCommit"], freezeUTC=freeze["utc"],
                developmentRunsChecked=len(freeze["developmentTrialSHA256"]), solver=freeze["sourceCodeSolver"],
                A4=original["model"], ordinaryControllerWeights=original["weights"])


def bin_margin(x,road=False):
    if x < 0 or (x == 0 and not road):return "contact_or_out"
    for cap,label in ((.10,"[0,.10)"),(.25,"[.10,.25)"),(.50,"[.25,.50)"),(1.,"[.50,1.00)")):
        if x < cap:return label
    return ">=1.00"


def criteria(run,case):
    goal=run["length"]*(case["limitM"] / 280.87656260722264)
    normal=("lap" if case["scope"]=="lap" else "segment")
    complete=(run["status"]==normal and (case["scope"]!="lap" or run["gate"]==32)
              and run["progress"]+.002>=goal and all(t>=0 for t in run["zoneEnd"]))
    succeed=complete and run["minRoad"]>=0 and all(g>0 for g in run["propMin"])
    maxima=max(row[7] for row in run["rows"])
    cones=[dict(index=i,stationM=p[7],approached=maxima>=p[7]-12.,exited=run["zoneEnd"][i]>=0,
                minM=run["propMin"][i] if maxima>=p[7]-12. else None) for i,p in enumerate(run["props"])]
    return dict(normalCompletion=complete,physicalSuccess=succeed,terminal=run["status"],elapsedS=run["seconds"],
                gatesPassed=run["gate"],progressM=run["progress"],targetM=goal,unfinishedM=max(0.,goal-run["progress"]),
                minRoadM=run["minRoad"],minConeM=min(run["propMin"],default=None),cones=cones)


def compare_request(a,b,c,length,base_model):
    ar,br=a["request"],b["request"]
    # Optional learner payload is the only treatment allowed; check everything environmental and motor-related equally.
    for k in (ar.keys()|br.keys())-{"smooth"}:
        if ar.get(k)!=br.get(k):raise ValueError(f"{c['id']}: unmatched {k}")
    for k in ("length","props","profile","coupled","physicsStep","unity"):
        if a[k]!=b[k]:raise ValueError(f"{c['id']}: unmatched simulation field {k}")
    if ar.get("encounter")!=base_model or ar.get("smooth") is not None:raise ValueError(f"{c['id']}: reference differs from frozen V2")
    expected=[[p[0]/length,*p[1:]] for p in c["props"]]
    rng=np.random.default_rng(c["seed"])
    if (ar["seed"]!=c["seed"] or not ar["vision"] or ar["timing"]!={"mode":"both","approach":c["approach"]}
        or not np.allclose([ar["start"],ar["offset"],ar["yaw"]],[rng.random(),rng.uniform(-.30,.30),rng.uniform(-3.2,3.2)],rtol=0,atol=1e-8)
        or abs(ar["limit"]-c["limitM"]/length)>1e-8 or len(ar["obstacles"])!=len(expected)
        or not np.allclose(ar["obstacles"],expected,rtol=0,atol=1e-8)):
        raise ValueError(f"{c['id']}: different course, approach or seeded request")


def disjoint_free(end,obstacles):
    cuts=[0.,end]
    for left,right in obstacles:cuts += [max(0.,min(end,left)),max(0.,min(end,right))]
    points=sorted(set(cuts))
    return [[a,b] for a,b in zip(points,points[1:]) if not any(lo<=(a+b)/2<=hi for lo,hi in obstacles)]


def observe(a,b,c,bend):
    result=dict(id=c["id"],scope=c["scope"],family=c["family"],road=c["road"],capMps=c["approach"],
                firstCommandChange=first_control_change(a,b),base=criteria(a,c),learned=criteria(b,c))
    rows=[np.array(x["rows"],float) for x in (a,b)]
    limit=min(np.max(t[:,7]) for t in rows)
    zones=obstacle_windows(a)
    sections=dict(obstacle=zones,straight=c["straightCruiseGatesM"],freeBend=bend,
                  free=disjoint_free(c["limitM"],zones))
    for arm,trace in zip(("base","learned"),rows):
        out=result[arm]
        out["shared"]={name:intervals_report(trace,windows,limit)["totals"] for name,windows in sections.items()}
        out["straightGates"]=intervals_report(trace,sections["straight"])
        out["obstacleGates"]=intervals_report(trace,zones)
    result["sharedUntilM"]=float(limit)
    result["exactPhysics"]=a["rows"]==b["rows"]
    return result


def replay_selector(run,model):
    if run["request"].get("smooth")!=model or model.get("mode")=="probe":
        raise ValueError("Challenger model differs from frozen artifact or forces actions")
    choices=[]
    max_error=0.
    for d in run["decisions"]:
        ev=d.get("smooth") or {}
        if ev.get("state")!="selected":
            continue
        features=np.array(ev["features"],dtype=np.float32)
        scores=np.zeros(len(model["actions"]),dtype=np.float32)
        for tree in model["forest"]:
            node=tree[0]
            while node[0]>=0:
                row=2 if features[int(node[0])]<=np.float32(node[1]) else 3
                node=tree[int(node[row])]
            scores+=np.array(node[1:],dtype=np.float32)
        scores/=len(model["forest"])
        err=float(np.max(np.abs(scores-ev["actionScores"])))
        max_error=max(err,max_error)
        if err>.0003 or int(scores.argmax())!=ev["chosen"] or ev["modelId"]!=model["modelId"]:
            raise ValueError("Learned Unity decision differs from independent forest replay")
        choices.append(dict(t=d["t"],action=ev["chosen"],targetMps=d["speedCmd"],measuredMps=d["speed"],reason=d["reason"]))
    return dict(choices=choices,maxScoreError=max_error)


def pair_state(p):
    return tuple(p[a]["physicalSuccess"] for a in ("base","learned"))


def deltas(pairs,where,metric):
    result=[]
    for p in pairs:
        x,y=(p[a]["shared"][where].get(metric) for a in ("base","learned"))
        tiny=.001 if metric=="yawTravelRadPer10m" else 1e-10
        if x is not None and y is not None and x>tiny:result.append((y-x)/x)
    return result


def median_or_none(a):
    return float(np.median(a)) if len(a) else None


def speed(pairs,key,arm):
    data=[p[arm]["shared"][key] for p in pairs]
    seconds=sum(v["seconds"] for v in data)
    return sum(v["progressM"] for v in data)/seconds if seconds else None


def percentile_ci(values):
    vals=np.array(values,dtype=float)
    vals=vals[np.isfinite(vals)]
    return np.percentile(vals,[2.5,97.5]).tolist() if len(vals) else None


def cohort(pairs):
    mutually=[p for p in pairs if pair_state(p)==(True,True)]
    completed_obstacles=[p for p in mutually if p["base"]["shared"]["obstacle"]["progressM"]>0]
    natural=[p for p in pairs if not p["capMps"]]
    yaw=deltas(completed_obstacles,"obstacle","yawTravelRadPer10m")
    zone_time=deltas(completed_obstacles,"obstacle","seconds")
    free=[speed(natural,"straight",a) for a in ("base","learned")]
    rng=np.random.default_rng(2840511); total=len(pairs)
    # Sample whole fixed cases, including crashes, then form comparable subsets per bootstrap replicate.
    resample=rng.integers(0,total,size=(10000,total))
    diff=np.array([int(pair_state(p)[1])-int(pair_state(p)[0]) for p in pairs])
    def med_ci(metric):
        mapping=[]
        for p in pairs:
            ds=deltas([p],"obstacle",metric) if pair_state(p)==(True,True) else []
            mapping.append(ds[0] if ds else np.nan)
        a=np.array(mapping)[resample]; valid=np.any(np.isfinite(a),axis=1)
        return percentile_ci(np.nanmedian(a[valid],axis=1)) if valid.any() else None
    cruise=[]
    for a in ("base","learned"):
        x=np.array([[p[a]["shared"]["straight"][k] if not p["capMps"] else 0. for k in ("progressM","seconds")] for p in pairs])
        sums=x[resample].sum(axis=1)
        cruise.append(np.divide(sums[:,0],sums[:,1],out=np.full(10000,np.nan),where=sums[:,1]>0))
    def direction(data):
        return dict(lower=sum(v<-.000001 for v in data),tie=sum(abs(v)<=.000001 for v in data),higher=sum(v>.000001 for v in data))
    diagnostic={}
    for key in ("yawTravelRadPer10m","yawRateVariationRadSPer10m","excessYawRadPer10m","yawReversalsPer10m",
                "commandYawReversalsPer10m","meanAbsWheelDifferentialRpm","wheelDifferentialVariationRpmPer10m",
                "excessPathMPer10m","lateralTravelMPer10m"):
        values=[[p[arm]["shared"]["obstacle"][key] for p in completed_obstacles] for arm in ("base","learned")]
        differences=[v-u for u,v in zip(*values)]
        diagnostic[key]=dict(medians=[median_or_none(v) for v in values],medianPairedDifference=median_or_none(differences),
                             signs=direction(differences))
    return dict(pairs=total,success=[sum(p[a]["physicalSuccess"] for p in pairs) for a in ("base","learned")],
                failureToSuccess=[p["id"] for p in pairs if pair_state(p)==(False,True)],
                successToFailure=[p["id"] for p in pairs if pair_state(p)==(True,False)],
                bothFailed=[p["id"] for p in pairs if pair_state(p)==(False,False)],
                mutuallySuccessfulObstacles=[p["id"] for p in completed_obstacles],
                medianMatchedYawPer10mChange=median_or_none(yaw),yawChangeSigns=direction(yaw),medianMatchedZoneTimeChange=median_or_none(zone_time),
                zoneTimeSigns=direction(zone_time), pooledSharedStraightMps=free,
                pooledSharedStraightChange=free[1]/free[0]-1 if all(free) else None,
                mutuallySuccessfulSteering=diagnostic,
                bootstrap95=dict(completionRateDifference=percentile_ci(diff[resample].mean(axis=1)),medianYawFraction=med_ci("yawTravelRadPer10m"),
                                 medianZoneTimeFraction=med_ci("seconds"),pooledSharedStraightMpsDifference=percentile_ci(cruise[1]-cruise[0])))


def cruise_group(p):
    if p["family"]=="clear":return "clearLocal"
    if p["family"]=="full-clear":return "clearLaps"
    if p["family"]=="full-traffic":return "trafficLaps"
    return "otherNaturalLocal" if not p["capMps"] else None


def cruise_summary(pairs,signals):
    groups={}
    for name,which in (("clearLocal",lambda p:p["family"]=="clear"),("clearLaps",lambda p:p["family"]=="full-clear"),
                       ("trafficLaps",lambda p:p["family"]=="full-traffic"),("otherNaturalLocal",lambda p:p["scope"]=="segment" and p["family"]!="clear" and not p["capMps"])):
        rows=[p for p in pairs if which(p)]
        reports={}
        for arm in ("base","learned"):
            vals=[p[arm]["straightGates"]["totals"] for p in rows]
            plan=sum(s["plannedM"] for s in vals);seen=sum(s["progressM"] for s in vals);duration=sum(s["seconds"] for s in vals)
            reports[arm]=dict(cases=[p["id"] for p in rows],plannedM=plan,observedM=seen,speedOnObservedMps=seen/duration if duration else None,
                              fractionPlannedAtLeast4=sum(s.get("atLeast4M",0) for s in vals)/plan if plan else None,
                              fullSuccessAt4=[p["id"] for p,s in zip(rows,vals) if p[arm]["physicalSuccess"] and (s["coverage"] or 0)>=.99999 and (s["progressSpeedMps"] or 0)>=4.])
            reports[arm].update(body_speed(signals.get((name,arm),[])))
        groups[name]=reports
    return groups


def motion_by_road(pairs):
    """Keep the turn geometry, capped approaches and complete-only time comparisons visible."""
    groupings = {
        "straightTrafficLocal": [p for p in pairs if p["scope"] == "segment" and p["road"] == "straight" and p["family"] != "clear"],
        "bendTrafficLocal": [p for p in pairs if p["scope"] == "segment" and p["road"] == "bend"],
        "externallyCappedLocal": [p for p in pairs if p["capMps"]],
        "naturalTrafficLocal": [p for p in pairs if p["scope"] == "segment" and p["family"] != "clear" and not p["capMps"]],
        "trafficLaps": [p for p in pairs if p["family"] == "full-traffic"],
        "clearLaps": [p for p in pairs if p["family"] == "full-clear"],
    }
    reports = {}
    for name, members in groupings.items():
        together = [p for p in members if all(pair_state(p))]
        report = cohort(members)
        report["ids"] = [p["id"] for p in members]
        report["matchedSuccessSpeedMps"] = {
            section: [speed(together, section, arm) for arm in ("base", "learned")]
            for section in ("obstacle", "free", "freeBend", "straight")
        }
        report["jointSuccessElapsedSeconds"] = [
            {"id": p["id"], "base": p["base"]["elapsedS"], "learned": p["learned"]["elapsedS"]} for p in together
        ]
        reports[name] = report
    return reports


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("runs",type=Path);ap.add_argument("--out",type=Path,required=True)
    ap.add_argument("--base",type=Path,default=HERE/"learned_risk_revision/model.json")
    ap.add_argument("--model",type=Path,required=True)
    args=ap.parse_args();plan=load(PROTOCOL/"layouts.json");bends=load(PROTOCOL/"bend_controls.json")
    base_model=load(args.base);model=load(args.model);pairs=[];versions=set();signals={}
    sealed=release_audit(args.runs,args.model)
    for c in plan["cases"]:
        paths=[args.runs/f"{c['id']}_{arm}.json" for arm in ("base","learned")]
        a,b=map(load,paths); compare_request(a,b,c,plan["courseLengthForFractionsM"],base_model)
        if (a["request"]["model"]!=sealed["A4"] or a["request"]["solver"]!=sealed["solver"]
            or a["request"]["weights"]!=sealed["ordinaryControllerWeights"]):
            raise ValueError("Unexpected perception, solver or ordinary driving settings in " + c["id"])
        versions.add((a["request"]["solver"],a["request"]["model"]))
        if len(versions)!=1:raise ValueError("Solver or A4 changed between held-out scenarios")
        row=observe(a,b,c,bends[c["id"]]);row["runSha256"]=[sha(f) for f in paths]
        row["selector"]=replay_selector(b,model);pairs.append(row)
        group=cruise_group(row)
        if group:
            for arm,run in (("base",a),("learned",b)):
                parts=signals.setdefault((group,arm),[])
                trace=np.asarray(run["rows"])
                for lo,hi in c["straightCruiseGatesM"]:
                    part=extract(trace,lo,hi)
                    if part is not None:parts.append(part)
    seg=[p for p in pairs if p["scope"]=="segment"];laps=[p for p in pairs if p["scope"]=="lap"]
    parts={k:cohort(v) for k,v in (("segment",seg),("lap",laps))}
    cruise=cruise_summary(pairs,signals)
    local=parts["segment"];pooled=[speed([p for p in pairs if not p["capMps"]],"straight",a) for a in ("base","learned")]
    clears={a:len(cruise["clearLocal"][a]["fullSuccessAt4"])==4 and len(cruise["clearLaps"][a]["fullSuccessAt4"])==2 for a in ("base","learned")}
    smooth=(not any(r["successToFailure"] for r in parts.values()) and all(r["success"][1]>=r["success"][0] for r in parts.values())
            and local["medianMatchedYawPer10mChange"] is not None and local["medianMatchedYawPer10mChange"]<=-.05
            and local["medianMatchedZoneTimeChange"] is not None and local["medianMatchedZoneTimeChange"]<=.02
            and all(pooled) and pooled[1]>=.98*pooled[0])
    margin={}
    for arm in ("base","learned"):
        vals=[c["minM"] for p in pairs for c in p[arm]["cones"] if c["approached"]]
        margin[arm]=dict(approachedConeBins=dict(Counter(bin_margin(x) for x in vals)),unvisitedCones=sum(not c["approached"] for p in pairs for c in p[arm]["cones"]),
                         roadBins=dict(Counter(bin_margin(p[arm]["minRoadM"],True) for p in pairs)))
    report=dict(protocolCommit="7a91a5f",registeredLayoutsSha256=sha(PROTOCOL/"layouts.json"),
                frozenFileHashes=dict(base=sha(args.base),learned=sha(args.model)),releaseAudit=sealed,versions=sorted(versions),cohorts=parts,
                naturalCruise=cruise,conditions=motion_by_road(pairs),demonstrates4MpsOnClear=clears,
                meetsCombinedSmoothness=bool(smooth),margins=margin,pairs=pairs)
    args.out.write_text(json.dumps(report,allow_nan=False,ensure_ascii=False,indent=2)+"\n",encoding="utf8")

if __name__=="__main__":
    main()
