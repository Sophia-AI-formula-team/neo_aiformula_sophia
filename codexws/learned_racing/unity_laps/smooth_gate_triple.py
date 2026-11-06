"""One-clock three-arm film for the frozen controller, faster cruise, and visible-opening planner."""
import argparse
import json
import math
from functools import lru_cache
from pathlib import Path

import av
from PIL import Image,ImageDraw,ImageChops,ImageStat

from gap_film import GateView,ROUTE,number
from smooth_comparison import BASE,FIT,GRID,INK,MUTED,PAPER,SIZE,WHITE,LocalPaths,selected_text
from three_view import FPS,digest,label
from timing_comparison import Recording,TERMINAL

ARMS=("base","pace","learned")
TITLES=("冻结长训策略","只提高巡航速度","同巡航速度＋近障寻缝")
COLORS=("#65568c",BASE,FIT)
X=(38,704,1370)
CAM=(640,360)
A4=(256,144)


@lru_cache(maxsize=36)
def photo(path,size):
    with Image.open(path) as source:
        if abs(source.width/source.height-16/9)>.01:raise ValueError(f"Unexpected optical aspect: {path}")
        return source.convert("RGB").resize(size,Image.Resampling.BICUBIC)


def records(root,case):
    result=[]
    for arm in ARMS:
        id_=case+"_"+arm
        result.append(Recording(root/"runs"/(id_+".json"),root/".work/frames"/id_,root/".work/camera"/id_/"a4_samples"))
    first=result[0].run
    for r in result[1:]:
        for k in ("props","length","physicsStep","profile","coupled","unity"):
            if first[k]!=r.run[k]:raise ValueError("Not a matched trial: "+k)
        for k,v in first["request"].items():
            if k!="smooth" and r.run["request"].get(k)!=v:raise ValueError("Different trial request: "+k)
    return result


def stat(d,x,y,g,sim,stop):
    start=g["frontEnterS"];end=g["rearClearS"]
    entered=start is not None and sim>=start
    finished=end is not None and sim>=end
    status=ROUTE[g["route"]] if finished or stop else "车身正在过门" if entered else "正在接近"
    label(d,(x,y),status,23,INK,True)
    va=number(g["entryBodyMps"] if entered else None);vb=number(g["exitBodyMps"] if finished else None)
    label(d,(x,y+36),f"实速进 / 出    {va} / {vb} m/s",18,MUTED)
    after=finished or (entered and stop)
    opening=number(g.get("gateMinFreeOpeningM") if after else None)
    le=number(g.get("minLeftProjectedM") if after else None);ri=number(g.get("minRightProjectedM") if after else None)
    label(d,(x,y+69),f"最窄开口 {opening} m",18,MUTED)
    label(d,(x,y+100),f"两侧投影净距   {le} / {ri} m",17,MUTED)


def render(args):
    recs=records(args.root,args.case);judge=GateView(args.results,recs)
    if len(judge.arms[0])>2:raise ValueError("This dashboard reserves separate rows for two gate stations")
    scene=LocalPaths(recs,args.start,args.end,judge.indices[0],judge)
    count=math.ceil((args.end-args.start)*FPS-.000001)+1
    terminal=any(abs(args.end-r.run["seconds"])<.00002 and r.run["status"] in ("obstacle","boundary") for r in recs)
    total=count+(round(.7*FPS) if terminal else 0)
    checks={0,total//2,total-1}
    for gs,r in zip(judge.arms,recs):
        stamps=[v for gate in gs for v in (gate["frontEnterS"],gate["rearClearS"]) if v is not None]
        stamps+= [r.run["seconds"]]+[q["t"] for q in r.decisions if (q.get("gap") or {}).get("state")=="visible_opening"][:1]
        for t in stamps:
            if args.start<=t<=args.end:checks.add(min(total-1,math.ceil((t-args.start)*FPS-1e-5)))
    sheet=Image.new("RGB",SIZE,PAPER);d=ImageDraw.Draw(sheet)
    d.rectangle((0,0,2048,90),fill=INK);label(d,(38,48),args.title,39,WHITE,True)
    for x,title,color in zip(X,TITLES,COLORS):
        label(d,(x,124),title,30,color,True)
        d.rounded_rectangle((x,540,x+640,746),12,fill=WHITE,outline=GRID)
    d.rounded_rectangle((22,770,697,1302),13,fill=WHITE,outline=GRID)
    label(d,(41,809),"场地俯视 · 裁判",28,INK,True)
    d.rounded_rectangle((715,770,2026,1302),13,fill=WHITE,outline=GRID)
    label(d,(734,809),"每排双桶之间，全车有没有通过？",28,INK,True)
    for j,(name,col) in enumerate(zip(TITLES,COLORS)):
        x=736+428*j;label(d,(x,872),name,22,col,True)
        for n,g in enumerate(judge.arms[j]):
            y=925+164*n
            label(d,(x,y),f"第 {n+1} 排",19,col,True)
    d.rectangle((0,1320,2048,1496),fill=INK)
    label(d,(38,1355),"整段裁判结果",23,"#dce9e5",True)
    for x,r in zip(X,recs):
        obj=r.run
        label(d,(x,1410),TERMINAL.get(obj["status"],obj["status"])+f"   {obj['seconds']:.2f} s",27,WHITE,True)
        label(d,(x,1460),f"最小实物净距 {obj['minGap']:.2f} m    路界 {obj['minRoad']:.2f} m",20,"#d3e6e1")
    args.out.parent.mkdir(parents=True,exist_ok=True)
    samples_qa={}
    with av.open(str(args.out),"w",options={"movflags":"+faststart"}) as out:
        stream=out.add_stream("libx264",rate=FPS,options={"preset":"veryfast","crf":"19","threads":"4"})
        stream.width,stream.height=SIZE;stream.pix_fmt="yuv420p"
        for i in range(total):
            t=min(args.end,args.start+i/FPS);ss=[r.sample(t) for r in recs]
            im=sheet.copy();d=ImageDraw.Draw(im)
            label(d,(2008,48),f"同一仿真时钟   {t:.2f} s   /   "+("终止停帧" if i>=count else "1×"),24,WHITE,True,anchor="rm")
            refs=[]
            for j,(x,r,s,col) in enumerate(zip(X,recs,ss,COLORS)):
                obs=r.overlays/f"{s['decision_id']:05d}.jpg"
                if not s["file"].is_file() or not obs.is_file():raise FileNotFoundError("Absent raw chase or A4 sample")
                im.paste(photo(s["file"],CAM),(x,158));im.paste(photo(obs,A4),(x+8,551))
                label(d,(x+10,718),f"A4 真前视  ·  {s['decision']['t']:.2f} s",17,MUTED)
                if s["stopped"]:
                    d.rectangle((x,477,x+639,517),fill="#60362c")
                    label(d,(x+11,497),TERMINAL.get(r.run["status"],r.run["status"])+f"   /   {s['t']:.2f} s 停帧",19,WHITE,True)
                q=s["decision"];gap=q.get("gap") or {}
                label(d,(x+280,575),f"实速 {s['row'][4]:.2f} m/s",27,col,True)
                label(d,(x+280,617),f"目标 {q['speedCmd']:.2f} m/s   横摆 {s['row'][14]:+.2f} rad/s",16,MUTED)
                label(d,(x+280,657),selected_text(q),18,INK,True)
                if gap.get("state")=="visible_opening":
                    label(d,(x+280,704),f"计划 {len(gap['crossings'])} 排 · 预计 {gap['travelTime']:.2f} s",18,MUTED)
                else:
                    label(d,(x+280,704),f"左−右目标轮速 {q['leftCan']-q['rightCan']:+d} rpm",18,MUTED)
                for n,g in enumerate(judge.arms[j]):
                    stat(d,736+428*j,954+164*n,g,s["t"],s["t"]>=r.run["seconds"]-.00002)
                refs.append(dict(chase=str(s["file"]),a4=str(obs),timeChase=s["frame_t"],timeA4=q["t"],held=s["stopped"],
                    measuredBodyMps=s["row"][4],requestedMps=q["speedCmd"],planner=gap))
            im.paste(scene.draw(ss,t),(44,875))
            label(d,(43,1237),"紫 · 冻结基线     橙 · 只提高巡航     青 · 加入寻缝",19,MUTED)
            label(d,(44,1275),"横线连接同排两桶。轨迹为最近 2.5 s 的实际位置。",18,MUTED)
            label(d,(735,1274),"进门为车头抵达，出门为车尾清空。侧向净距为车身占据门区间内的最小投影值。",18,MUTED)
            if i in checks:samples_qa[i]=dict(simTime=t,refs=refs)
            for packet in stream.encode(av.VideoFrame.from_image(im)):out.mux(packet)
        for packet in stream.encode():out.mux(packet)
    with av.open(str(args.out)) as inp:
        if inp.streams.video[0].frames!=total:raise ValueError("Film clock differs from encoded frames")
        for i,frame in enumerate(inp.decode(video=0)):
            if i not in samples_qa:continue
            q=samples_qa[i];im=frame.to_image()
            for x,s in zip(X,q['refs']):
                errs=[]
                for k,xy,size,h in (("chase",(x,158),CAM,316 if s["held"] else CAM[1]),("a4",(x+8,551),A4,A4[1])):
                    src=Path(s[k]);delta=ImageChops.difference(im.crop((xy[0],xy[1],xy[0]+size[0],xy[1]+h)),photo(src,size).crop((0,0,size[0],h)))
                    err=sum(ImageStat.Stat(delta).mean)/3
                    if err>8:raise ValueError("Original camera pixel mismatch")
                    errs.append(round(err,3));s[k+"SHA256"]=digest(src);s[k]=src.name
                s["decodedMeanRGBErrors"]=errs
    data=dict(case=args.case,title=args.title,sha256=digest(args.out),frames=total,fps=FPS,sameClock=[args.start,args.end],
        originalRunSHA256=[digest(r.path) for r in recs],compared=list(ARMS),judge=judge.provenance,localRoad=scene.road_check,sampledFrames=samples_qa)
    args.out.with_suffix(".json").write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n",encoding="utf8")
    print(args.case,digest(args.out))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("root","results","out"):p.add_argument("--"+k,type=Path,required=True)
    for k in ("start","end"):p.add_argument("--"+k,type=float,required=True)
    for k in ("case","title"):p.add_argument("--"+k,required=True)
    render(p.parse_args())
