"""Reconstruct ordinary returns, risk debit and the deployed action from camera-state logs."""
import argparse
import json
from pathlib import Path

import numpy as np
from learned_audit import audit, images
from learn_encounters import ROOT as V1, NAMES, predictive_scores, write
from risk_encounters import ROOT

def main():
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path,default=ROOT);args=p.parse_args();root=args.output
    model=json.loads((root/"model.json").read_bytes());result={}
    for path in sorted((root/"runs").glob("*_learned.json")):
        stem=path.stem.removesuffix("_learned");run=json.loads(path.read_bytes())
        if not stem.startswith("diag_"):
            evidence=audit(root,model,stem)
        else:
            ref=stem.removeprefix("diag_")+"_base";baseline=json.loads((V1/"runs"/(ref+".json")).read_bytes())
            params=run["request"].copy();assert params.pop("encounter")==model and params==baseline["request"]
            evidence=dict(diagnosticReference=ref,physicsTraceEqual=baseline["rows"]==run["rows"],status=run["status"],
                          gate=run["gate"],progressM=run["progress"],minGapM=run["minGap"],minRoadM=run["minRoad"],streams=images(root,run))
        contributions=[]
        for d in run["decisions"]:
            e=d["encounter"]
            if e["state"]!="selected":continue
            def values(key):
                return predictive_scores(np.asarray(model["contexts"]),np.asarray(model[key]),np.asarray(model["scales"]),
                                         model["width"],np.asarray(e["features"]))
            ordinary=values("rawReturns");debit=model["riskWeight"]*values("riskUpper");adjusted=ordinary-debit
            assert np.allclose(adjusted,e["actionScores"],atol=3e-5) and np.argmax(adjusted)==e["chosen"]
            contributions.append(dict(t=d["t"],ordinary=ordinary.tolist(),riskDebit=debit.tolist(),adjusted=adjusted.tolist(),
                                      ordinaryChoice=NAMES[int(np.argmax(ordinary))],deployedChoice=NAMES[e["chosen"]]))
        evidence["riskContributions"]=contributions;result[stem]=evidence
    write(root/"risk_action_audit.json",result)
if __name__=="__main__":main()
