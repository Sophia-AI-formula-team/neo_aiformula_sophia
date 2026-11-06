"""Make a separately labelled gate film after the three-arm blind score exists."""
import argparse
from pathlib import Path
import json
import subprocess
import sys

import av

from assemble_obstacle_films import inspect, join
from three_view import digest

HERE=Path(__file__).parent


def read(p):
    return json.loads(p.read_bytes())


def write(p,data):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n",encoding="utf8")


def select(pairs,single):
    candidates=[p for p in pairs if (len(p["learned"]["gates"])==1)==single]
    if not candidates:return None
    rescued=next((p for p in candidates if p["learned"]["allMiddleSuccess"] and not p["pace"]["allMiddleSuccess"]),None)
    if rescued:return rescued,"同一巡航设定下，首个参考臂未全程中穿而加入寻缝后完成所有中穿的登记场景。"
    safe_middle=next((p for p in candidates if p["learned"]["allMiddleSuccess"]),None)
    if safe_middle:return safe_middle,"登记顺序中首个加入寻缝后全车从每排中间通过并安全完段的场景。"
    return candidates[0],"这个类别没有全程中穿并安全完段的正例，展示首个登记场景的实际动作和结果。"


def window(p,root):
    arms=("base","pace","learned");ids=[p["id"]+"_"+k for k in arms]
    runs=[read(root/"runs"/(id_+".json")) for id_ in ids]
    starts=[];ends=[]
    for k,run in zip(arms,runs):
        gates=p[k]["gates"]
        entry=gates[0]["frontEnterS"]
        if entry is None:
            # A failure before the first crossing is still shown.
            entry=run["seconds"]-2
        end=gates[-1]["rearClearS"]
        if end is None or run["status"] not in ("lap","segment"):end=run["seconds"]
        starts.append(entry);ends.append(end)
    start=min(max(0,min(starts)-3.0),min(r["seconds"] for r in runs)-.2)
    return start,min(max(r["seconds"] for r in runs),max(ends)+1.25)


def main(args):
    score=read(args.results);pairs=score["pairs"]
    cfg=score["gapConfigurations"]
    if not cfg["learned"]["openings"] or cfg["pace"]["openings"] or cfg["pace"]["paceScale"]!=cfg["learned"]["paceScale"]:
        raise ValueError("The selected control must share the frozen straight pace while disabling local opening selection")
    specs=[]
    for single,label,name in ((True,"双桶单排","01-single-gate"),(False,"连续双排门","02-staggered-gates")):
        pick=select(pairs,single)
        if pick:specs.append((*pick,label,name))
    covered={p["id"] for p,*_ in specs}
    failure=next((p for p in pairs if p["pace"]["physicalSuccess"] and not p["learned"]["physicalSuccess"] and p["id"] not in covered),None)
    if failure is None:
        failure=next((p for p in pairs if not p["learned"]["physicalSuccess"] and p["id"] not in covered),None)
    if failure:specs.append((failure,"未展示场景中首个参考臂完成而寻缝失败的场景，若无则保留其他实际失败。","寻缝未完成","03-failure"))
    paths=[];titles=[];counts=[];proof=[];fingerprints=[];sig=None
    ev=args.package/"evidence";vid=args.package/"videos";ev.mkdir(parents=True,exist_ok=True);vid.mkdir(exist_ok=True)
    for p,why,label,name in specs:
        case=p["id"];start,end=window(p,args.root);title=f"二阶段独立考核 {case.upper()} · {label}";path=vid/(name+".mp4")
        cmd=[sys.executable,"-B",str(HERE/"smooth_gate_triple.py"),"--root",str(args.root),"--case",case,
             "--start",str(start),"--end",str(end),"--title",title,"--results",str(args.results),"--out",str(path)]
        subprocess.run(cmd,check=True)
        receipt=read(path.with_suffix(".json"));path.with_suffix(".json").replace(ev/(name+".json"))
        rate,size,code,marks=inspect(path,ev/(name+".jpg"),min(3.,(end-start)*.5))
        if sig and sig!=(rate,size,code):raise ValueError("Incompatible separate gate videos")
        sig=rate,size,code;counts.append(len(marks));fingerprints+=marks;paths.append(path);titles.append(title)
        proof.append(dict(id=case,file=path.name,sha256=digest(path),fps=float(rate),frames=len(marks),reason=why,
            trainedMiddle=[g["route"] for g in p["learned"]["gates"]],paceMiddle=[g["route"] for g in p["pace"]["gates"]],
            trialResults={a:p[a]["terminal"] for a in ("base","pace","learned")},selectionAudit=receipt["judge"]["judgeSHA256"]))
    output=vid/"00-gate-scenes.mp4";join(paths,sig[0],counts,titles,output)
    _,_,_,joined=inspect(output)
    if joined!=fingerprints:raise ValueError("Changed source frames in joined gate reel")
    with av.open(str(output)) as a:
        if [c["metadata"]["title"] for c in a.chapters()]!=titles:raise ValueError("Wrong gate chapter metadata")
    write(ev/"collection.json",dict(scoreSHA256=digest(args.results),combined=output.name,sha256=digest(output),frames=len(joined),chapters=proof))
    print(json.dumps(proof,ensure_ascii=False))


if __name__=="__main__":
    a=argparse.ArgumentParser(description=__doc__)
    for key in ("root","results","package"):a.add_argument("--"+key,type=Path,required=True)
    main(a.parse_args())
