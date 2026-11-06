"""Select and package auditable steering clips from the registered score in case order."""
import argparse
from html import escape
from pathlib import Path
import json
import shutil
import subprocess
import sys
from functools import lru_cache
import av
import numpy as np

from assemble_obstacle_films import inspect,join
from three_view import digest
from verify_three_view import verify

HERE=Path(__file__).parent


@lru_cache(maxsize=1)
def road_geometry():
    from scenario_shots import geometry
    center,_,_,curvature,_=geometry()
    return center,curvature


def local_scene(run,anchor):
    if not run["props"] or anchor is None:return {"props":[],"categories":["clear"]}
    group=[(i,p) for i,p in enumerate(run["props"]) if abs(p[7]-run["props"][anchor][7])<=8]
    center,curvature=road_geometry()
    facts=[]
    for i,p in group:
        idx=int(np.argmin(np.linalg.norm(center-np.asarray(p[:2]),axis=1)))
        facts.append({"index":i,"stationM":p[7],"amplitudeM":p[4],"omegaRadS":p[6],
                      "moving":bool(p[4] and p[6]),"curvaturePerM":float(curvature[idx])})
    moving=sum(x["moving"] for x in facts)
    tags=["static" if moving==0 else "moving" if moving==len(facts) else "mixed"]
    if any(abs(x["curvaturePerM"])>.025 for x in facts):tags.append("bend")
    return {"props":facts,"categories":tags}


def load(p):return json.loads(p.read_bytes())


def save(p,obj):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+"\n",encoding="utf8")


def yaw(p):
    vals=[p[a]["shared"]["obstacle"].get("yawTravelRadPer10m") for a in ("base","learned")]
    if any(x is None for x in vals) or vals[0]<.01:return None
    return vals[1]/vals[0]-1


def short_scene(p,runs):
    arms=[load(runs/(p["id"]+"_"+a+".json")) for a in ("base","learned")]
    failures=[r for r in arms if r["status"] in ("obstacle","boundary")]
    if not arms[0]["props"]:
        return dict(start=2.,end=min(12.,max(r["seconds"] for r in arms)),prop=None)
    failed=min(failures,key=lambda r:r["seconds"]) if failures else None
    prop=(min(range(len(failed["props"])),key=lambda i:failed["propMin"][i]) if failed
          else min(range(len(arms[1]["props"])),key=lambda i:arms[1]["propMin"][i]))
    near=[i for i,v in enumerate(arms[0]["props"]) if abs(v[7]-arms[0]["props"][prop][7])<=8]
    enters=[r["zoneStart"][i] for r in arms for i in near if r["zoneStart"][i]>=0]
    ends=[r["zoneEnd"][i] for r in arms for i in near if r["zoneEnd"][i]>=0]
    if failed:
        enters.append(failed["seconds"]-3.5);ends.append(failed["seconds"]+.6)
    first=max(0,min(enters or [2])-2)
    first=min(first,min(r["seconds"] for r in arms)-.3)
    last=min(max(r["seconds"] for r in arms),max(ends or [first+8])+1)
    return dict(start=float(f"{first:.2f}"),end=last,prop=prop)


def selection(args):
    rep=load(args.results);pairs=rep["pairs"]
    local=[p for p in pairs if p["scope"]=="segment" and p["base"]["cones"]]
    traffic=[p for p in pairs if p["base"]["cones"]]
    def ok(p,a):return p[a]["physicalSuccess"]
    reversal=[p for p in local if not ok(p,"base") and ok(p,"learned")]
    common=[p for p in local if ok(p,"base") and ok(p,"learned") and yaw(p) is not None]
    timely=[p for p in common if p["learned"]["shared"]["obstacle"]["seconds"]<=1.02*p["base"]["shared"]["obstacle"]["seconds"] and yaw(p)<-.0001]
    neutral=next((p for p in local if not p["exactPhysics"]),local[0])
    if reversal:best,mode,why=reversal[0],"improvement","首个基线失败、长训完成的局部场景"
    elif timely:best,mode,why=min(timely,key=yaw),"improvement","同段用时增幅不超2%，在双方完成的局部场景中横摆角行程降幅最大"
    else:best,mode,why=neutral,"comparison","首个出现实际轨迹差异的局部场景；若完全同轨则按登记顺序"
    regress=[p for p in traffic if ok(p,"base") and not ok(p,"learned")]
    if regress:
        worst=min(regress,key=lambda p:p["learned"]["progressM"]/p["learned"]["targetM"])
        worst_why="基线完成而长训失败的含障碍场景中，长训完成路程比例最低"
    else:
        comparable=[p for p in common if yaw(p)>.0001]
        worst=max(comparable,key=yaw) if comparable else None
        worst_why="双方均完成的局部场景中，横摆角行程相对增长最大"
    out=[]
    def add(p,name,title,reason):
        if p:
            window=short_scene(p,args.runs)
            scene=local_scene(load(args.runs/(p["id"]+"_learned.json")),window["prop"])
            out.append(dict(name=name,title=title,case=p["id"],kind="pair",selection=reason,
                       **window,localScene=scene,base=p["base"]["terminal"],trained=p["learned"]["terminal"],
                       fullCaseMatchedYawChange=yaw(p)))
    add(best,"02-matched-"+mode,"同场对照 "+best["id"].upper()+" · "+("改善例" if mode=="improvement" else "代表例"),why)
    if worst and worst["id"]!=best["id"]:
        add(worst,"03-matched-regression","同场对照 "+worst["id"].upper()+" · 回退例",worst_why)
    failed=next((p for p in traffic if not ok(p,"learned") and p["learned"]["terminal"] in ("obstacle","boundary")
                 and p["id"] not in (best["id"],worst["id"] if worst else "")),None)
    if failed:add(failed,"05-contact","失败回放 "+failed["id"].upper(),"登记顺序中尚未展示的首次长训实物碰撞或触边")
    labels={"static":"静态桶","moving":"移动桶","mixed":"静动混合","bend":"弯中遇障"}
    for number,(category,caption) in enumerate(labels.items(),7):
        if any(category in q["localScene"]["categories"] for q in out):continue
        eligible=[]
        for q in local:
            window=short_scene(q,args.runs)
            facts=local_scene(load(args.runs/(q["id"]+"_learned.json")),window["prop"])
            if category in facts["categories"]:eligible.append(q)
        if not eligible:raise ValueError("No filmed local layout of required physical type: "+category)
        choice=next((q for q in eligible if not q["exactPhysics"] and ok(q,"learned")),eligible[0])
        add(choice,f"{number:02}-"+category,caption+"  "+choice["id"].upper(),
            "补全实体场地类别；优先按登记顺序选择轨迹不同且长训能完成的场景")
    coverage={k:next(row["name"] for row in out if k in row["localScene"]["categories"]) for k in labels}
    laps=[p for p in pairs if p["scope"]=="lap"]
    full=next((p for p in laps if ok(p,"learned")),laps[0])
    out.append(dict(name="04-first-complete-lap" if ok(full,"learned") else "04-full-attempt",
               title="完整赛道 "+full["id"].upper()+" · 从起步到"+("终点" if ok(full,"learned") else "终止"),
               case=full["id"],kind="full",selection="登记顺序中首个完成的整圈；若全部失败则保留首趟完整尝试"))
    if not full["base"]["cones"]:
        lap_with_traffic=next((p for p in laps if p["base"]["cones"] and ok(p,"learned")),None)
        if lap_with_traffic:out.append(dict(name="06-first-traffic-lap",case=lap_with_traffic["id"],kind="full",
            title="含障碍整圈 "+lap_with_traffic["id"].upper()+" · 从起步到终点",selection="首个完成的含障碍整圈"))
    record=dict(scoreHash=digest(args.results),frozenHash=rep["frozenFileHashes"]["learned"],
                sceneCoverage=coverage,cases=sorted(out,key=lambda x:x["name"]))
    save(args.out,record);print(json.dumps(record,ensure_ascii=False,indent=2))


def film(args):
    spec=next(x for x in load(args.plan)["cases"] if x["name"]==args.name)
    name=spec["name"];case=spec["case"];raw=args.root/".work";base,learned=case+"_base",case+"_learned"
    cfg=load(args.root/"runs"/(learned+".json"))["request"].get("smooth") or {}
    if cfg.get("modelId")!=args.model_id or cfg.get("mode")=="probe":raise ValueError("Not the frozen model")
    temp=args.package/".work"/name/(name+".mp4");temp.parent.mkdir(parents=True,exist_ok=True)
    clips=args.package/"videos";ev=args.package/"evidence";(ev/"posters").mkdir(parents=True,exist_ok=True);clips.mkdir(exist_ok=True)
    if spec["kind"]=="pair":
        cmd=[sys.executable,"-B",str(HERE/"smooth_comparison.py"),"--root",str(args.root),"--base",base,"--trained",learned,
             "--start",str(spec["start"]),"--end",str(spec["end"]),"--title",spec["title"],"--model-id",args.model_id,"--out",str(temp)]
        if spec["prop"] is not None:cmd.extend(("--prop",str(spec["prop"])))
        subprocess.run(cmd,check=True)
    else:
        data=args.root/"runs"/(learned+".json");r=load(data);frames=raw/"frames"/learned;a4=raw/"camera"/learned/"a4_samples"
        subprocess.run([sys.executable,"-B",str(HERE/"three_view.py"),"--trajectory",str(data),"--frames",str(frames),
            "--a4",str(a4),"--start","0","--end",str(r["seconds"]),"--whole-lap","--referee-inset","--smooth-panel",
            "--title",spec["title"],"--out",str(temp)],check=True)
        save(ev/(name+".verification.json"),verify(temp.with_suffix(".json"),frames,a4))
        if any(load(temp.with_suffix(".json"))["missing_camera_output_frames"].values()):raise ValueError("Missing original frames")
    with temp.with_suffix(".json").open(encoding="utf8") as f:receipt=json.load(f)
    fps,size,_,marks=inspect(temp)
    poster_s=(len(marks)/fps)*(.45 if spec["kind"]=="pair" else .33)
    inspect(temp,ev/"posters"/(name+".jpg"),float(poster_s))
    receipt.update(dict(clipSHA256=digest(temp),case=case,sourceModelId=args.model_id,selection=spec["selection"],
                        durationS=float(len(marks)/fps),frameSize=size))
    save(ev/(name+".json"),receipt);temp.replace(clips/temp.name)


def assemble(args):
    plan=load(args.plan);ev=args.package/"evidence";videos=args.package/"videos";out=videos/"00-all-evidence.mp4"
    names=["01-training-evidence"]+[c["name"] for c in plan["cases"]]
    titles=["训练、检查点与独立考核"]+[c["title"] for c in plan["cases"]]
    paths=[videos/(n+".mp4") for n in names];counts=[];all_marks=[];signature=None
    for p in paths:
        fps,size,codec,marks=inspect(p)
        if signature and signature!=(fps,size,codec):raise ValueError("Incompatible encoded assets")
        signature=(fps,size,codec);counts.append(len(marks));all_marks+=marks
    join(paths,signature[0],counts,titles,out)
    fps,sz,_,marks=inspect(out)
    if all_marks!=marks:raise ValueError("Combined footage changed during muxing")
    running=0;chapters=[]
    notes=["同一布局试七种动作，随后比较共同开发集和独立考核。"]+[c["selection"] for c in plan["cases"]]
    posters=ev/"posters"
    posters.mkdir(exist_ok=True,parents=True)
    inspect(paths[0],posters/(names[0]+".jpg"),3.0)
    for name,path,title,n,note in zip(names,paths,titles,counts,notes):
        chapters.append(dict(name=name,title=title,file=path.name,sha256=digest(path),startS=float(running/fps),
                             durationS=float(n/fps),selection=note))
        running+=n
    with av.open(str(out)) as video:
        chapters_in_file=video.chapters()
        if len(chapters_in_file)!=len(chapters):raise ValueError("Wrong number of chapters in MP4")
        for cue,c in zip(chapters_in_file,chapters):
            if abs(float(cue["start"]*cue["time_base"])-c["startS"])>=1/fps or cue["metadata"]["title"]!=c["title"]:
                raise ValueError("Encoded chapter does not match the player")
    save(ev/"collection.json",dict(file=out.name,sha256=digest(out),frames=running,durationS=float(running/fps),chapters=chapters,
                                   allFrameFingerprintsVerified=running))
    cards="".join(f'<article><button type="button" aria-current="false"><img class="thumb" src="evidence/posters/{c["name"]}.jpg" alt=""><span class="caption"><b>{escape(c["title"])}</b><small>{c["durationS"]:.1f} 秒</small></span></button><p>{escape(c["selection"])}</p><a href="videos/{c["file"]}">独立 MP4</a></article>' for c in chapters)
    data=json.dumps(chapters,ensure_ascii=False).replace("<","\\u003c")
    html=(HERE/"smooth_player.html").read_text(encoding="utf-8-sig").replace("{{CARDS}}",cards).replace("{{CHAPTERS}}",data)
    (args.package/"index.html").write_text(html,encoding="utf8");print(out,digest(out))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest="stage",required=True)
    a=sub.add_parser("plan")
    for k in ("runs","results","out"):a.add_argument("--"+k,type=Path,required=True)
    a=sub.add_parser("film")
    for k in ("root","plan","package"):a.add_argument("--"+k,type=Path,required=True)
    for k in ("name","model-id"):a.add_argument("--"+k,required=True)
    a=sub.add_parser("assemble")
    for k in ("plan","package"):a.add_argument("--"+k,type=Path,required=True)
    args=p.parse_args();{"plan":selection,"film":film,"assemble":assemble}[args.stage](args)
