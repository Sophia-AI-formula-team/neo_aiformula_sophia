"""Judge whole-vehicle passage through real left/right cone pairs at fixed stations."""
import argparse
import json
from pathlib import Path

import numpy as np
from crossing_audit import section
from smooth_holdout_score import HERE, criteria, load, sha
from smooth_window_metrics import extract, measured

PLAN=HERE/"gap_holdout"


def gate(run,spec):
    ia,ib=spec["propIndices"]
    left,right=(np.asarray(run["props"][i],float) for i in (ia,ib))
    if not np.allclose(left[2:4],right[2:4],atol=1e-6) or abs(left[7]-right[7])>.005:
        raise ValueError("Left and right props do not share a physical cross section")
    n=left[2:4]; n=n/np.linalg.norm(n)
    spacing=float((right[:2]-left[:2])@n)
    if spacing<=0:raise ValueError("Cone indices reversed")
    times=section(run,left)
    enter=times.get("frontEnterS");axle=times.get("axleCrossS");leave=times.get("rearClearS")
    rows=np.asarray(run["rows"],float)
    reached=max(rows[:,7])>=left[7]-8.
    out=dict(index=spec["index"],propIndices=[ia,ib],role=spec["role"],stationM=left[7],
             leftHomeXZ=left[:2].tolist(),rightHomeXZ=right[:2].tolist(),gateRightXZ=n.tolist(),
             approached=bool(reached),frontEnterS=enter,axleCrossS=axle,rearClearS=leave,
             entryBodyMps=None if enter is None else float(np.interp(enter,rows[:,0],rows[:,4])),
             exitBodyMps=None if leave is None else float(np.interp(leave,rows[:,0],rows[:,4])),
             minOfficialPropM=[run["propMin"][i] if reached else None for i in (ia,ib)],
             route="not_reached" if enter is None else "unfinished")
    if enter is None:return out
    end=leave if leave is not None else rows[-1,0]
    if end<enter:return out
    ts=np.linspace(enter,end,max(2,1+int(np.ceil((end-enter)/.005))))
    car=np.c_[np.interp(ts,rows[:,0],rows[:,1]),np.interp(ts,rows[:,0],rows[:,2])]
    yaw=np.interp(ts,rows[:,0],np.unwrap(rows[:,3]))
    f=np.c_[np.sin(yaw),np.cos(yaw)];r=np.c_[f[:,1],-f[:,0]]
    x=np.array([((car-left[:2])+z*f+w*r)@n for z in (-.88,.28) for w in (-.38,.38)])
    lo=x.min(axis=0);hi=x.max(axis=0)
    a=left[4]*np.sin(left[5]+left[6]*ts)
    b=spacing+right[4]*np.sin(right[5]+right[6]*ts)
    space_left=lo-a-.34;space_right=b-.34-hi
    out.update(minLeftProjectedM=float(space_left.min()),minRightProjectedM=float(space_right.min()),
               gateOccupancyS=float(end-enter),gateMinFreeOpeningM=float((b-a-.68).min()),
               entranceLaneOffsetM=float(np.interp(enter,rows[:,0],rows[:,8])),
               axleLaneOffsetM=None if axle is None else float(np.interp(axle,rows[:,0],rows[:,8])),
               exitLaneOffsetM=None if leave is None else float(np.interp(leave,rows[:,0],rows[:,8])))
    if axle is not None and enter<=axle<=end:
        center=((car-left[:2])@n)
        car_at=float(np.interp(axle,ts,center))
        inner=float(np.interp(axle,ts,a));outer=float(np.interp(axle,ts,b))
        identity="inside_pair" if inner<car_at<outer else ("inside_inner_lane_cone" if car_at<=inner else "outside_outer_lane_cone")
        out["axleInstant"]=dict(centerBetweenCones=identity,centerFromInnerConeM=car_at-inner,
            outerConeFromCenterM=outer-car_at,footprintFromInnerConeEdgeM=float(np.interp(axle,ts,space_left)),
            outerConeEdgeFromFootprintM=float(np.interp(axle,ts,space_right)),note="Instantaneous, separate from the whole-passage label")
    if leave is not None:
        # Descriptive geometric check after the primary route has been defined. Slice the physical rectangle
        # at the immobile gate plane; unlike the strict primary test, its distant corners do not extend the slice.
        tangent=np.array([-n[1],n[0]])
        corners=np.array([(car-left[:2])+z*f+w*r for z,w in ((-.88,-.38),(-.88,.38),(.28,.38),(.28,-.38))])
        xs=corners@n;zs=corners@tangent;left_margins=[];right_margins=[]
        for j,k in ((0,1),(1,2),(2,3),(3,0)):
            cross=(zs[j]*zs[k]<=0.) & (np.abs(zs[j]-zs[k])>1e-12)
            fraction=zs[j,cross]/(zs[j,cross]-zs[k,cross])
            cut=xs[j,cross]+fraction*(xs[k,cross]-xs[j,cross])
            if len(cut):
                left_margins.extend(cut-a[cross]-.34);right_margins.extend(b[cross]-.34-cut)
        if left_margins:
            vl,vr=float(min(left_margins)),float(min(right_margins))
            out["planeSliceDiagnostic"]=dict(minInnerM=vl,minOuterM=vr,centralAtPlane=min(vl,vr)>.01,
                note="Supplementary body/gate-plane intersection only; not used in primary scores.")
    if leave is not None:
        if min(space_left.min(),space_right.min())>.01:
            out["route"]="middle"
        elif (a-.34-hi).min()>.01:
            out["route"]="outside_inner_lane_cone"
        elif (lo-b-.34).min()>.01:
            out["route"]="outside_outer_lane_cone"
        else:
            out["route"]="straddle_or_uncertain"
    return out


def assess(run,case):
    status=criteria(run,case)
    gates=[gate(run,g) for g in case["gates"]]
    rows=np.array(run["rows"],float)
    first,last=gates[0]["stationM"]-8.,gates[-1]["stationM"]+7.
    part=extract(rows,first,last)
    fixed=None
    if part is not None:
        m=measured(part)
        fixed=dict(progressM=m["progressM"],seconds=m["seconds"],speedMps=m["progressM"]/m["seconds"],
                   absLateralTravelM=m["lateralTravelM"],signedLateralDisplacementM=float(part[-1,8]-part[0,8]),
                   yawTravelRad=m["yawTravelRad"],complete=bool(m["progressM"]>=last-first-.004))
    status.update(gates=gates,middleGates=sum(g["route"]=="middle" for g in gates),window=fixed,
                  windowFromToM=[first,last],allMiddleSuccess=bool(status["physicalSuccess"] and all(g["route"]=="middle" for g in gates)))
    return status


def paired_summary(pairs,reference):
    def summarize(group):
        td=[p["learned"]["window"]["seconds"]-p[reference]["window"]["seconds"] for p in group]
        if not group:return dict(n=0,ids=[],medianTimeDifferenceS=None)
        rng=np.random.default_rng(1244802);boot=rng.choice(td,size=(10000,len(td)),replace=True)
        return dict(n=len(group),ids=[p["id"] for p in group],medianTimeDifferenceS=float(np.median(td)),
                    pairedDifferencesS=td,lower=sum(v<-.0001 for v in td),tie=sum(abs(v)<=.0001 for v in td),
                    higher=sum(v>.0001 for v in td),bootstrapMedian95S=np.percentile(np.median(boot,axis=1),[2.5,97.5]).tolist())
    def physical(p):return p[reference]["physicalSuccess"] and p["learned"]["physicalSuccess"]
    safe=[p for p in pairs if physical(p) and all(p[a]["window"] and p[a]["window"]["complete"] for a in (reference,"learned"))]
    central=[p for p in safe if p[reference]["allMiddleSuccess"] and p["learned"]["allMiddleSuccess"]]
    out=dict(order=[reference,"learned"],allCases=len(pairs),plannedGates=sum(len(p["learned"]["gates"]) for p in pairs))
    for name in ("physicalSuccess","allMiddleSuccess","middleGates"):
        out[name]=[sum(p[a][name] for p in pairs) for a in (reference,"learned")]
        change=np.array([int(p["learned"][name])-int(p[reference][name]) for p in pairs])
        n=len(pairs);indices=np.random.default_rng(1245001).integers(0,n,(10000,n))
        out[name+"MeanPairedDifference95"]=np.percentile(change[indices].mean(axis=1),[2.5,97.5]).tolist()
        out[name+"Regressions"]=[p["id"] for p in pairs if p[reference][name]>p["learned"][name]]
        out[name+"Rescues"]=[p["id"] for p in pairs if p[reference][name]<p["learned"][name]]
    out["matchedPhysicalWindow"]=summarize(safe);out["matchedMiddleWindow"]=summarize(central)
    out["demonstratesNewMiddle"]=bool(out["allMiddleSuccessRescues"] and not out["physicalSuccessRegressions"])
    out["demonstratesGeneralPace"]=bool(out["demonstratesNewMiddle"] and not out["allMiddleSuccessRegressions"] and safe
                                       and out["matchedPhysicalWindow"]["medianTimeDifferenceS"]<-.0001)
    return out


def check_run(run,case,ref):
    req=run["request"];length=280.87656260722264
    spawn=np.random.default_rng(case["seed"])
    props=[[p[0]/length,*p[1:]] for p in case["props"]]
    if (req["seed"]!=case["seed"] or not np.allclose([req["start"],req["offset"],req["yaw"]],[spawn.random(),spawn.uniform(-.30,.30),spawn.uniform(-3.2,3.2)],rtol=0,atol=1e-8)
        or req["timing"]!={"mode":"both","approach":0} or abs(req["limit"]-case["limitM"]/length)>1e-8
        or not req["vision"] or not np.allclose(req["obstacles"],props,rtol=0,atol=1e-8)):
        raise ValueError("Judged case does not match preregistration: "+case["id"])
    for key in ("length","props","coupled","profile","physicsStep","unity"):
        if run[key]!=ref[key]:raise ValueError("Unmatched simulation field "+key)
    for key in (req.keys()|ref["request"].keys())-{"smooth"}:
        if req.get(key)!=ref["request"].get(key):raise ValueError("Unmatched driving condition "+key)


def main():
    ap=argparse.ArgumentParser();ap.add_argument("runs",type=Path);ap.add_argument("--model",type=Path,required=True,
        help="Frozen previous smooth policy shared by all arms")
    ap.add_argument("--out",type=Path,required=True);ap.add_argument("--pace",action="store_true")
    args=ap.parse_args();plan=load(PLAN/"layouts.json");model=load(args.model);pairs=[]
    arms=["base","learned"]+(["pace"] if args.pace else []);templates={}
    root=args.runs.parent;freeze=load(root/"frozen.json");frozen_paths={"base":args.model,"learned":root/"challenger.json","pace":root/"pace_only.json"}
    frozen_keys={"base":"reference","learned":"challenger","pace":"pace"}
    frozen={a:load(frozen_paths[a]) for a in arms}
    for a in arms:
        if sha(frozen_paths[a])!=freeze[frozen_keys[a]]:raise ValueError("Pre-test freeze changed: "+a)
    if sha(root/"practice_scores.json")!=freeze["report"]:raise ValueError("Practice selection changed")
    practice=load(root/"practice.json")["cases"];test_seeds={c["seed"] for c in plan["cases"]}
    if test_seeds & {c["seed"] for c in practice}:raise ValueError("Evaluation and calibration seeds overlap")
    if args.pace:
        expected=dict(frozen["learned"]["gap"],openings=False)
        if not frozen["learned"]["gap"]["openings"] or frozen["pace"]["gap"]!=expected:
            raise ValueError("Pace-only treatment differs beyond the opening planner")
    solver=load(root/"baseline.json")["solverHash"];original=load(HERE/"risk_revision/frozen.json")
    for case in plan["cases"]:
        paths={a:args.runs/f"{case['id']}_{a}.json" for a in arms};logs={a:load(p) for a,p in paths.items()}
        pair=dict(id=case["id"],road=case["road"],family=case["family"],runHashes={a:sha(p) for a,p in paths.items()})
        for a,run in logs.items():
            check_run(run,case,logs["base"])
            cfg=run["request"]["smooth"]
            if (cfg!=frozen[a] or run["request"]["solver"]!=solver or run["request"]["model"]!=original["model"]
                or run["request"]["weights"]!=original["weights"]):
                raise ValueError("Run does not match the frozen release: "+case["id"]+"_"+a)
            if a=="base" and cfg!=model:raise ValueError("Different baseline policy")
            if a!="base" and {k:v for k,v in cfg.items() if k!="gap"}!=model:
                raise ValueError("The gap-enabled arm changed prior model coefficients")
            settings=cfg.get("gap")
            if a!="base" and settings is None:raise ValueError("Gap experiment was not configured")
            if a in templates and settings!=templates[a]:raise ValueError("Gap config differs by scenario")
            templates[a]=settings;pair[a]=assess(run,case)
        pairs.append(pair)
    result=dict(layoutsSHA256=sha(PLAN/"layouts.json"),baseModelSHA256=sha(args.model),gapConfigurations=templates,
                freeze=freeze,solver=solver,disjointPracticeSeeds=len(practice),evaluationSeeds=len(test_seeds),
                cohort=paired_summary(pairs,"base"),paceControl=paired_summary(pairs,"pace") if args.pace else None,pairs=pairs)
    args.out.write_text(json.dumps(result,ensure_ascii=False,allow_nan=False,indent=2)+"\n",encoding="utf8")


if __name__=="__main__":
    main()
