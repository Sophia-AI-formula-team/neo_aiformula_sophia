"""Descriptive same-action control. It never enters the 32-case primary pass/fail rule."""
import argparse
import json
from pathlib import Path
import numpy as np
from smooth_holdout_score import HERE, PROTOCOL, load, sha, observe, cohort, compare_request, replay_selector

COSTS=("headingResidualTVperM","yawCommandTVperM","lateralTravelPerM","extraPathPerM")


def constant_action(training):
    """Choose solely from all supplied completed training probes and the frozen training smoothness scales."""
    rows=[c["outcomes"] for c in training["outcomes"]]
    scales=np.array([training["scales"][k] for k in COSTS])
    ranking=[]
    for a in range(len(rows[0])):
        completed=[r[a] for r in rows if r[a]["completed"]]
        smooth=[]
        for r in completed:
            raw=np.array([r[k] for k in COSTS])
            smooth.append(np.dot([.35,.30,.20,.15],1./(1.+raw/scales)))
        if completed:
            score=float(np.mean(smooth));time=float(np.mean([r["zoneTimeS"] for r in completed]))
        else:
            score=time=None
        ranking.append(dict(action=a,completed=len(completed),smoothBenefit=score,meanTimeS=time))
    best=max((r for r in ranking if r["completed"]),key=lambda r:(r["completed"],r["smoothBenefit"],-r["meanTimeS"],-r["action"]))
    return best["action"],ranking


def report(runs,model,training,selection,baseline):
    action,ranking=constant_action(training)
    if action!=selection:
        raise ValueError("Frozen constant action and independent training-only rank differ")
    matrix=load(PROTOCOL/"layouts.json");bends=load(PROTOCOL/"bend_controls.json");pairs=[]
    for case in matrix["cases"]:
        if case["scope"]!="segment" or not case["props"]:continue
        label=case["id"];ref=load(runs/(label+"_base.json"));learner=load(runs/(label+"_learned.json"))
        fixed_path=runs/(label+("_fixed.json" if action else "_base.json"))
        fixed=load(fixed_path)
        compare_request(ref,fixed,case,matrix["courseLengthForFractionsM"],baseline)
        compare_request(ref,learner,case,matrix["courseLengthForFractionsM"],baseline)
        if action:
            cfg=fixed["request"]["smooth"]
            if cfg["mode"]!="probe" or cfg["forced"]!=action or cfg["actions"]!=model["actions"]:
                raise ValueError("The matched control did not force the preregistered action")
            decisions=[d["smooth"]["chosen"] for d in fixed["decisions"] if (d.get("smooth") or {}).get("state")=="selected"]
            if any(v!=action for v in decisions):raise ValueError("Observed action differs from the fixed action")
        p=observe(fixed,learner,case,bends[label]);p["fixedFileSha256"]=sha(fixed_path)
        p["learnerChoices"]=replay_selector(learner,model)["choices"]
        pairs.append(p)
    if len(pairs)!=20:raise ValueError("All twenty traffic segments are required")
    summary=cohort(pairs)
    for p in pairs:p["constantAction"]=p.pop("base")
    return dict(protocolCommit="3349da4",descriptive=True,summaryOrder=["constantAction","learned"],
                trainingRanking=ranking,frozenAction=action,summary=summary,pairs=pairs)


if __name__=="__main__":
    ap=argparse.ArgumentParser();ap.add_argument("runs",type=Path)
    ap.add_argument("--training",type=Path,required=True,help="Checkpoint training metrics for all collected training groups")
    ap.add_argument("--model",type=Path,required=True);ap.add_argument("--action",type=int,required=True)
    ap.add_argument("--out",type=Path,required=True);ap.add_argument("--base",type=Path,default=HERE/"learned_risk_revision/model.json")
    args=ap.parse_args()
    result=report(args.runs,load(args.model),load(args.training),args.action,load(args.base))
    args.out.write_text(json.dumps(result,ensure_ascii=False,allow_nan=False,indent=2)+"\n",encoding="utf8")
