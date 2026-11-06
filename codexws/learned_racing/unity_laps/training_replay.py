"""Animate the measured action probes. Replays use the referee's poses, never synthetic camera images."""
import argparse
import bisect
import json
import math
from pathlib import Path

import av
import numpy as np
from PIL import Image, ImageDraw

from referee_inset import position
from three_view import label


SIZE = (1440, 810)
FPS = 20
PAPER, INK, SECONDARY = "#f5f5ee", "#172f33", "#496569"
PASS, FAIL, ACCENT = "#196d5d", "#ac402c", "#0e6376"
MOVE = "#bd3273"
ACTION_LABELS = {
    "baseline": "原控制器", "same_line_fast": "同线快行", "inner_slow": "向内慢行",
    "inner_fast": "向内快行", "outward_slow": "向外慢行", "same_line_slow": "同线慢行",
}
STATUS = {"obstacle": "碰桶", "boundary": "触边", "segment": "完段", "lap": "完圈",
          "stalled": "停滞", "lost_lane": "丢线", "invalid_progress": "进度异常"}


def read(path):
    return json.loads(path.read_bytes())


def sheet(title, sub):
    im = Image.new("RGB", SIZE, PAPER)
    d = ImageDraw.Draw(im)
    label(d, (46, 33), "AI FORMULA    /    UNITY   +   YOLOP A4", 19, ACCENT, True, True)
    label(d, (46, 88), title, 44, INK, True)
    label(d, (46, 147), sub, 25, SECONDARY)
    return im


def status(run):
    return STATUS.get(run["status"], run["status"])


def matrix(runs, cases, names, visible):
    im = sheet("同一起点，六种开法各跑一次", "每列是一种训练布局。格内是从起步到完段的秒数，或试车的终止原因。")
    d = ImageDraw.Draw(im)
    x0, y0, dx, dy = 204, 224, 99, 75
    for i, case in enumerate(cases):
        label(d, (x0 + dx*i + 46, 200), case.upper(), 23, SECONDARY, True, True, anchor="mm")
    for arm, name in enumerate(names):
        y = y0 + dy*arm
        label(d, (48, y+33), ACTION_LABELS[name], 25, INK, True)
        for i, case in enumerate(cases):
            r = runs[case, arm]
            shown = r["id"] in visible
            finished = r["status"] in ("segment", "lap")
            x = x0+dx*i
            d.rounded_rectangle((x,y,x+91,y+65), radius=8,
                                fill=("#d3e9df" if finished else "#f4d6ca") if shown else "#e9ece6")
            if shown:
                label(d,(x+46,y+32), f"{r['seconds']:.1f}" if finished else status(r),
                      23, PASS if finished else FAIL, True, finished, anchor="mm")
    finished = sum(r["status"] in ("segment","lap") for r in runs.values() if r["id"] in visible)
    label(d, (49, 714), f"已试 {len(visible)} / {len(runs)} 趟     完段 {finished} 趟", 29, INK, True)
    label(d, (49, 767), "同列可以比较时间。不同列的路段长度、障碍和巡航条件各不相同。", 23, SECONDARY)
    return im


def axes(prop):
    right = np.array(prop[2:4])
    return np.array([[-right[1],right[0]], right])


def view_pair(first, second, source_start):
    runs = [first,second]
    for key in ("props","profile","coupled","physicsStep"):
        if first[key] != second[key]:
            raise ValueError(f"Unmatched exercise: {key}")
    select = [next(d for d in r["decisions"] if d["encounter"]["state"]=="selected") for r in runs]
    if select[0]["encounter"]["features"] != select[1]["encounter"]["features"] or select[0]["t"] != select[1]["t"]:
        raise ValueError("Compared actions did not start at the same camera observation")
    if source_start is None:
        source_start = max(0, select[0]["t"]-1.5)
    ref = np.array(first["props"][0][:2])
    base = axes(first["props"][0])
    records = [np.array(r["rows"]) for r in runs]
    corners = []
    for r, rows in zip(runs,records):
        points = list(rows[rows[:,0]>=source_start,1:3])
        points.extend(position(p, t) for p in r["props"] for t in rows[::3,0])
        corners.extend((np.array(points)-ref) @ base.T)
    lo,hi=np.min(corners,axis=0),np.max(corners,axis=0)
    mid=(hi+lo)/2
    scale=min(603/max(hi[0]-lo[0]+3,1),215/max(hi[1]-lo[1]+2.5,1))
    def panel(d, x, run, rows, t):
        def xy(point):
            f,s=(base@(point-ref)-mid)*scale
            return float(x+330+f),float(370+s)
        now=min(t,run["seconds"])
        seen=rows[rows[:,0]<=now+1e-5]
        row=seen[-1]
        d.rounded_rectangle((x,235,x+660,500),10,fill="#e6ede7")
        path=[xy(a[1:3]) for a in seen if a[0]>=source_start]
        if len(path)>1:
            d.line(path,fill=ACCENT,width=4)
        for p in run["props"]:
            cf=xy(np.array(position(p,now)))
            rad=max(5,.34*scale)
            d.ellipse((cf[0]-rad,cf[1]-rad,cf[0]+rad,cf[1]+rad),fill=MOVE)
        pos=row[1:3]; yaw=row[3]
        forward=np.array([math.sin(yaw),math.cos(yaw)])
        right=np.array([forward[1],-forward[0]])
        points=[xy(pos+f*forward+s*right) for f,s in ((.28,-.38),(.28,.38),(-.88,.38),(-.88,-.38))]
        d.polygon(points,fill=ACCENT)
        d.line(points[:2],fill="#f2bd45",width=3)
        decisions=run["decisions"]
        cmd=decisions[bisect.bisect_right([q["t"] for q in decisions],now)-1]
        label(d,(x+16,539),f"实速  {row[4]:.2f} m/s",30,INK,True)
        label(d,(x+320,539),f"目标  {cmd['speedCmd']:.2f} m/s",27,SECONDARY)
        ended=t>=run["seconds"]
        gap = run["minGap"] if ended else row[10]
        label(d,(x+16,600),f"本趟最小桶距  {gap:.2f} m" if ended else f"当前桶距  {gap:.2f} m",24,SECONDARY)
        if ended:
            label(d,(x+16,666),f"{status(run)}   ·   {run['seconds']:.2f} s  停帧",31,
                  PASS if run["status"]=="segment" else FAIL,True)
        else:
            label(d,(x+16,666),f"模拟时间   {now:.2f} s",28,ACCENT,True)
    def frame(t, names):
        im=sheet("同样的速度目标，换一条线会怎样？", "同一训练布局、同一帧 A4 观测。两台车同步重放，蓝线来自 Unity 的实测车身轨迹。")
        d=ImageDraw.Draw(im)
        for i,(r,rows,name) in enumerate(zip(runs,records,names)):
            x=48+i*685
            label(d,(x+5,204),name,29,INK,True)
            panel(d,x,r,rows,t)
        label(d,(49,762),"俯视为赛后裁判视图。粉色为障碍，车头以黄色标出。",23,SECONDARY)
        return im
    return frame, source_start, max(r["seconds"] for r in runs)


def render(root, output, pair):
    book=read(root/"actions.json")
    events=[json.loads(x) for x in (root/"events.jsonl").read_text(encoding="utf-8-sig").splitlines() if x]
    probes={}
    source={}
    for path in sorted((root/"runs").glob("t??_a*.json")):
        r=read(path)
        case,arm=r["id"].split("_a")
        probes[case,int(arm)]=r
        source[r["id"]]=r
    cases=sorted({case for case,arm in probes})
    names=book["actionNames"]
    expected=[probes[c,i]["id"] for c in cases for i in range(len(names))]
    chronological=list(dict.fromkeys(e["trial"] for e in events if e.get("trial") in expected and e.get("kind")=="finish"))
    if set(expected)!=set(chronological):
        raise ValueError("Missing trial completion events")
    # Refuse to draw a finished exercise grid until the frozen model exists.
    fit=read(root/"training.json")
    if set(fit["layouts"]) != set(cases):
        raise ValueError("Training grid incomplete")
    first,second=map(source.__getitem__,pair)
    chosen=[next(q["encounter"]["chosen"] for q in r["decisions"] if q["encounter"]["state"]=="selected") for r in (first,second)]
    if book["actions"][chosen[0]][1] != book["actions"][chosen[1]][1] or chosen[0]==chosen[1]:
        raise ValueError("Pair must compare distinct lines at the same target speed")
    replay,start,end=view_pair(first,second,None)
    titles=[f"{ACTION_LABELS[names[i]]}   {book['actions'][i][1]:.2f} m/s" for i in chosen]
    last=sheet("把各次尝试的结果交给模型", "模型输入来自车上观测。训练评分使用场地裁判记录的完段、余量和用时。")
    d=ImageDraw.Draw(last)
    selected=fit["selected"]
    used=sum(s<1e8 for s in selected["scales"])
    choices=selected["choices"]
    for y,num,caption in [(265,f"{len(probes)} 趟","同布局改变动作，在 Unity 中重新驾驶并记录结果"),
                          (403,f"{used} 维","冻结模型用于区分场景的观测维度"),
                          (541,f"{len(set(choices))} 种","训练中每次留出一个布局时，模型实际选择过的动作")]:
        label(d,(55,y),num,52,ACCENT,True)
        label(d,(430,y),caption,30,INK)
        d.line((52,y+61,1387,y+61),fill="#cbd7cf",width=2)
    label(d,(53,730),"此片为训练记录。新布局的独立考核收录在同目录的行驶录像中。",24,SECONDARY)
    output.parent.mkdir(parents=True,exist_ok=True)
    with av.open(str(output),"w",options={"movflags":"+faststart"}) as container:
        stream=container.add_stream("libx264",rate=FPS)
        stream.width,stream.height=SIZE
        stream.pix_fmt="yuv420p"
        stream.options={"preset":"veryfast","crf":"19"}
        frame_number = 0
        def emit(im, repeats=1):
            nonlocal frame_number
            frame=av.VideoFrame.from_image(im)
            for _ in range(repeats):
                frame.pts=frame_number
                frame_number+=1
                for packet in stream.encode(frame):
                    container.mux(packet)
        for i in range(len(expected)+1):
            emit(matrix(probes,cases,names,set(chronological[:i])),2 if i==len(expected) else 3)
        emit(matrix(probes,cases,names,set(expected)),FPS*3)
        for n in range(math.ceil((end-start+1.4)*FPS)):
            emit(replay(start+n/FPS,titles))
        emit(last,FPS*6)
        for packet in stream.encode():
            container.mux(packet)


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("root",type=Path)
    p.add_argument("output",type=Path)
    p.add_argument("--pair",nargs=2,required=True)
    a=p.parse_args()
    render(a.root,a.output,a.pair)
