"""Replay one development comparison selected before viewing held-out results."""
from pathlib import Path
import json
from types import SimpleNamespace
from smooth_trials import ROOT,evaluate,replay_evidence,write
def main():
    report=json.loads((ROOT/"development.json").read_bytes());frozen=json.loads((ROOT/"frozen.json").read_bytes())["checkpoint"]
    current=report[frozen];baseline=report["dev"]
    pool=[]
    for k in current["matchedObstacleSegments"]:
        if not any(current["choices"][k]):continue
        y=current["metrics"][k];b=baseline["metrics"][k]
        h=y["headingResidualTVperM"]/b["headingResidualTVperM"];w=y["yawCommandTVperM"]/b["yawCommandTVperM"]
        if max(h,w)<1 and y["zoneTimeS"]<=b["zoneTimeS"]:pool.append(((h+w)/2,k))
    case=sorted(pool)[len(pool)//2][1]
    write(ROOT/"development_replay_selection.json",dict(rule="median of genuine, jointly smoother, no-slower passes",pool=pool,chosen=case))
    for tag,source,arm in [("filmdev",frozen,None),("filmck32","ck32","learned")]:
        args=SimpleNamespace(output=ROOT,matrix=None,model=ROOT/"checkpoints"/source/"model.json",tag=tag,arm=arm,force=None,ids=[case],film=True)
        evaluate(args)
        for role in [arm] if arm else ["base","learned"]:
            original=("dev"if role=="base"else source)+"_"+case+"_"+role
            label=tag+"_"+case+"_"+role
            ref=json.loads((ROOT/"runs"/(original+".json")).read_bytes())
            run=json.loads((ROOT/"runs"/(label+".json")).read_bytes())
            receipt=replay_evidence(ref,run);receipt["referenceDevelopmentId"]=receipt.pop("trainingId")
            write(ROOT/"film_replays"/(label+".json"),receipt)
if __name__=="__main__":main()
