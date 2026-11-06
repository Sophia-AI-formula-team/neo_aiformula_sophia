"""Build the training film exclusively from complete Unity action probes, checkpoints and sealed evaluations."""
import argparse
import json
import math
from fractions import Fraction
from pathlib import Path

import av
import numpy as np
from PIL import Image,ImageDraw

from three_view import FPS,digest,label
from smooth_comparison import BASE,FIT,GRID,INK,MUTED,PAPER,SIZE,WHITE

BAD="#a44627";WARN="#c29753";GAP="#dce4df"
NAMES=["沿用 V2","沿参考线 · 快","左侧 · 小余量快行","右侧 · 小余量快行","左侧 · 留量稳行","右侧 · 留量稳行","沿参考线 · 慢行"]
ARMS=("base","learned")


def read(path):return json.loads(path.read_bytes())


def complete(r):
    return r["status"] in ("lap","segment") and all(x>=0 for x in r["zoneEnd"])


def area_tv(run):
    rows=np.array(run["rows"]);s=rows[:,7]
    mask=np.zeros(len(s),bool)
    for p in run["props"]:mask|=(s>=p[7]-12)&(s<=p[7]+8)
    active=mask[:-1]&mask[1:]
    distance=float(np.maximum(0,np.diff(s)[active]).sum())
    return float(np.abs(np.diff(rows[:,14])[active]).sum()),distance


def gather(root,blind_path):
    results=read(blind_path)
    sources={"blindScore":digest(blind_path)}
    train=[];models=[];stage=[]
    dirs=sorted((root/"checkpoints").glob("ck*"),key=lambda p:int(p.name[2:]))
    if not dirs:raise ValueError("No fitted training checkpoints")
    basefiles=sorted((root/"runs").glob("dev_d*_base.json"))
    if not basefiles:raise ValueError("No common development references")
    base={p.stem.removeprefix("dev_").removesuffix("_base"):read(p) for p in basefiles}
    stage.append(("基线 V2",0,base))
    for p in dirs:
        if not (p/"training.json").is_file():continue
        summary=read(p/"training.json");size=summary["cases"];cfg=read(p/"model.json")
        if summary["modelId"]!=cfg["modelId"]:raise ValueError(f"Wrong checkpoint {p}")
        inputs={}
        for id_ in base:
            f=root/"runs"/f"{p.name}_{id_}_learned.json"
            r=read(f);inputs[id_]=r
            if (r["request"].get("smooth") or {}).get("modelId")!=cfg["modelId"]:raise ValueError(f"Mismatched model for {f}")
            if r["props"]!=base[id_]["props"] or r["request"]["seed"]!=base[id_]["request"]["seed"]:raise ValueError(f"Unmatched development trial {f}")
        stage.append((f"{size} 布局",size,inputs));models.append(cfg["modelId"])
        sources[p.name]=dict(model=digest(p/"model.json"),training=digest(p/"training.json"))
        train=summary["outcomes"]
    if not train or not models:raise ValueError("Missing checkpoint outcomes")
    snapshots=[]
    local=[k for k,r in base.items() if r["request"]["limit"]<.999]
    lap=[k for k in base if k not in local]
    same=[k for k in local if base[k]["props"] and all(complete(row[2][k]) for row in stage)]
    for title,size,records in stage:
        variation=sum(area_tv(records[k])[0] for k in same);distance=sum(area_tv(records[k])[1] for k in same)
        snapshots.append({"label":title,"trainingLayouts":size,
            "localSuccess":sum(complete(records[k]) for k in local),"lapSuccess":sum(complete(records[k]) for k in lap),
            "localTotal":len(local),"lapTotal":len(lap),"commonQualifiedObstacleCases":same,
            "pooledYawRateTVPer10m":10*variation/distance if distance else None})
    source_digest={}
    for id_,r in base.items():source_digest["dev_"+id_+"_base"]=digest(root/"runs"/("dev_"+id_+"_base.json"))
    for p in dirs:
        for id_ in base:
            f=root/"runs"/f"{p.name}_{id_}_learned.json"
            if f.exists():source_digest[f.stem]=digest(f)
    sources["developmentRuns"]=source_digest
    if len(train[-1]["outcomes"])!=len(NAMES):raise ValueError("Different action set")
    # Use the exact frozen model from the scored independent trials, including if an earlier checkpoint won.
    sample=results["pairs"][0]["id"]
    check=read(root/"runs"/(sample+"_learned.json"))
    model=(check["request"].get("smooth") or {}).get("modelId")
    if model not in models:raise ValueError("Scored controller is absent from the training checkpoint list")
    freeze_size=stage[1+models.index(model)][1]
    return train,snapshots,results,sources,freeze_size


def header(title,desc,chapter):
    im=Image.new("RGB",SIZE,PAPER);d=ImageDraw.Draw(im)
    d.rectangle((0,0,SIZE[0],110),fill=INK)
    label(d,(54,52),title,43,WHITE,True)
    label(d,(1986,52),chapter,26,"#b8d8d2",True,anchor="rm")
    label(d,(54,158),desc,28,MUTED)
    return im


def card(d,rect):
    d.rounded_rectangle(rect,radius=20,fill=WHITE,outline=GRID,width=1)


def side_note(d,x,y,headline,note):
    label(d,(x,y),headline,30,INK,True);label(d,(x,y+51),note,23,MUTED)


def sample_page(train,ratio):
    im=header("同一个场景，分别试七种控制动作","每格是一次独立的 Unity 闭环运行，格子按训练布局编号依次揭示。","训练采样")
    d=ImageDraw.Draw(im);card(d,(43,219,2003,1221))
    total=len(train);seen=max(1,min(total,math.ceil(total*ratio)))
    left,top=407,344;cw=1525/total;rh=95
    label(d,(72,272),f"已揭示 {seen} / {total} 个布局",32,INK,True)
    colors=(FIT,WARN,BAD);names=("完成 · 桶净距 ≥ 0.50 m","完成 · 桶净距 < 0.50 m","发生碰撞、越界或未完成")
    for i,(c,n) in enumerate(zip(colors,names)):
        x=772+i*387;d.ellipse((x,262,x+16,278),fill=c);label(d,(x+24,271),n,19,MUTED)
    counter=[0,0,0]
    for a,name in enumerate(NAMES):
        y=top+a*rh
        label(d,(76,y+33),name,28,INK,a==0)
        for i,c in enumerate(train):
            v=c["outcomes"][a];x=left+cw*i
            n=2 if not v["completed"] else 0 if v["minConeM"] is None or v["minConeM"]>=.5 else 1
            if i<seen:counter[n]+=1
            d.rectangle((x,y,x+cw-2,y+64),fill=colors[n] if i<seen else GAP)
    for col in range(0,total,8):
        x=left+cw*col;label(d,(x,1030),str(col+1),18,MUTED,mono=True)
    label(d,(74,1127),f"这 {seen*len(NAMES)} 次对照中     完成 {counter[0]+counter[1]}      未完成 {counter[2]}",31,INK,True)
    side_note(d,62,1292,"评分最高的动作取自训练数据。","碰桶或越界计失败。成功时比较行进时间、车身与道路朝向的变化、控制变化；锥桶净距的奖励在 0.50 m 封顶。")
    return im


def dev_page(snapshots,freeze,ratio):
    im=header("训练到不同规模后，在同一组开发场景复跑","每一个模型都跑相同的局部路段和完整赛道。基线 V2 使用同一起点与障碍布置。","冻结前 · 开发验证")
    d=ImageDraw.Draw(im);card(d,(44,221,2003,1123))
    values=snapshots;shown=min(len(values),max(1,math.ceil(ratio*len(values))))
    xs=np.linspace(370,1830,len(values))
    label(d,(69,273),"物理完成数",31,INK,True)
    for i,v in enumerate(values):
        x=int(xs[i]);label(d,(x,342),v["label"],27,INK,True,anchor="mm")
        if i>=shown:continue
        label(d,(x,428),f"{v['localSuccess']}/{v['localTotal']}",54,FIT,True,True,"mm")
        label(d,(x,492),"局部完段",22,MUTED,anchor="mm")
        label(d,(x,581),f"{v['lapSuccess']}/{v['lapTotal']}",46,BASE,True,True,"mm")
        label(d,(x,636),"完整圈",22,MUTED,anchor="mm")
    d.line((73,688,1970,688),fill=GRID,width=2)
    label(d,(69,746),"障碍区每 10 m 的实测横摆速率总变差",31,INK,True)
    n=len(values[0]["commonQualifiedObstacleCases"])
    label(d,(70,793),f"只比较各检查点都完成的同一组局部障碍场景（{n} 个）；纵轴单位 (rad/s)/10 m。",22,MUTED)
    tv=[v["pooledYawRateTVPer10m"] for v in values]
    if all(a is not None for a in tv):
        ymax=max(tv)*1.28+.01;ymin=max(0,min(tv)*.75)
        d.line((207,1035,1910,1035),fill=GRID,width=2)
        pts=[(float(x),1035-(v-ymin)/(ymax-ymin)*155) for x,v in zip(xs,tv)]
        if shown>1:d.line(pts[:shown],fill=FIT,width=6)
        for i,(x,y) in enumerate(pts[:shown]):
            d.ellipse((x-8,y-8,x+8,y+8),fill=FIT)
            label(d,(x,y-22),f"{tv[i]:.2f}",25,INK,True,True,"mm")
    side_note(d,62,1220,f"冻结模型来自 {freeze} 个训练布局的检查点。","下面的独立考核在模型冻结后运行，单独报告局部和整圈结果；开发集用于选择模型。")
    return im


def percent(x):
    return "—" if x is None else f"{100*x:.0f}%"


def change(x):
    return "—" if x is None else f"{100*x:+.1f}%"


def result_page(report):
    im=header("冻结后的独立考核","同场景配对，碰撞或越界保留在完成率的分母中。速度与横摆只在双方都通过的共同路段比较。","独立考核 · 局部与整圈")
    d=ImageDraw.Draw(im)
    for j,(key,title) in enumerate((("segment","局部路段"),("lap","整圈尝试"))):
        x=44+j*995;card(d,(x,224,x+954,1181));r=report["cohorts"][key]
        label(d,(x+37,289),title,37,INK,True)
        label(d,(x+38,378),"基线 V2",24,BASE,True);label(d,(x+500,378),"长训策略",24,FIT,True)
        for k,color in enumerate((BASE,FIT)):
            label(d,(x+38+462*k,465),f"{r['success'][k]}/{r['pairs']}",85,color,True,True)
        label(d,(x+39,572),"物理完成数",23,MUTED)
        d.line((x+38,631,x+912,631),fill=GRID,width=2)
        for n,(field,desc) in enumerate((("medianMatchedYawPer10mChange","实测横摆角行程 / 10 m"),
                                        ("medianMatchedZoneTimeChange","同一障碍区通过时间"))):
            label(d,(x+38,709+n*91),desc,24,INK,True)
            label(d,(x+900,709+n*91),change(r[field]),32,INK,True,True,"rm")
        label(d,(x+38,922),f"双方均完成的含障碍场景   {len(r['mutuallySuccessfulObstacles'])}",23,MUTED)
        label(d,(x+38,990),f"原本完成、长训后失败  {len(r['successToFailure'])}",23,BAD if r["successToFailure"] else MUTED)
        label(d,(x+38,1051),f"原本失败、长训后完成  {len(r['failureToSuccess'])}",23,FIT if r["failureToSuccess"] else MUTED)
    side_note(d,60,1266,"横摆角行程来自车身实际运动。","相对变化为各个可比场景的中位数。道路本身的转弯也贡献横摆，应结合同场景轨迹、实际余量和完圈情况判断。")
    return im


def cruise_page(root,report,ratio):
    im=header("无障碍直道能持续达到 4 m/s 吗？","四个预登记直道，从沿程 12 m 到 34 m。实线来自车体物理测速，横线标出 4.0 m/s。","独立考核 · 实测巡航")
    d=ImageDraw.Draw(im);cases=[p for p in report["pairs"] if p["family"]=="clear"]
    if len(cases)!=4:raise ValueError("Expected all four prespecified clear-road cases")
    for j,case in enumerate(cases):
        x=48+(j%2)*995;y=220+(j//2)*317
        card(d,(x,y,x+955,y+289));label(d,(x+25,y+38),case["id"].upper(),24,INK,True,True)
        curves=[]
        for k,arm in enumerate(ARMS):
            rec=read(root/"runs"/(case["id"]+"_"+arm+".json"))
            gates=case[arm]["straightGates"]["gates"]
            if len(gates)!=1 or (gates[0]["fromM"],gates[0]["toM"])!=(12,34):raise ValueError("Clear-road gate contract differs")
            rows=np.asarray(rec["rows"]);pts=[(s,v) for s,v in rows[:,[7,4]] if 12<=s<=34]
            for gate in (12.,34.):
                if rows[0,7]<=gate<=np.max(rows[:,7]):
                    idx=np.flatnonzero(rows[:,7]>=gate)[0]
                    if idx>0:pts.append((gate,float(np.interp(gate,rows[idx-1:idx+1,7],rows[idx-1:idx+1,4]))))
            curves.append(sorted(pts))
        cmin,cmax=2.,5.
        px=lambda m:x+80+(m-12)/22*838
        py=lambda v:y+240-(v-cmin)/(cmax-cmin)*150
        d.line((px(12),py(4),px(34),py(4)),fill="#758887",width=2)
        label(d,(px(12)-6,py(4)),"4.0",16,MUTED,mono=True,anchor="rm")
        for k,pts in enumerate(curves):
            visible=[(px(s),py(v)) for s,v in pts if s<=12+22*ratio]
            if len(visible)>1:d.line(visible,fill=(BASE,FIT)[k],width=6 if k==0 else 3)
        for st in (12,23,34):label(d,(px(st),y+254),f"{st} m",16,MUTED,mono=True,anchor="mm")
    group=report["naturalCruise"]; y=914
    label(d,(66,y),"固定直道门汇总",29,INK,True)
    for j,(key,title) in enumerate((("clearLocal","四段无障碍直道"),("clearLaps","两次空场整圈内的直道门"))):
        x=66+j*997;label(d,(x,980),title,27,INK,True)
        for k,arm in enumerate(ARMS):
            g=group[key][arm];label(d,(x,1045+k*77),("基线 V2" if k==0 else "长训策略"),22,(BASE,FIT)[k],True)
            speed=g["measuredBodyMeanMps"];v="—" if speed is None else f"{speed:.2f}"
            label(d,(x+171,1045+k*77),f"{v} m/s  ·  ≥4 覆盖 {percent(g['fractionPlannedAtLeast4'])}",26,INK,True)
    side_note(d,62,1271,"上表均速来自实际车体测速，按物理时间加权。","≥4 覆盖率的分母包含所有预定直道里程，未到达的路段不给达标里程。")
    return im


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--root",type=Path,required=True)
    p.add_argument("--blind-results",type=Path,required=True);p.add_argument("--out",type=Path,required=True)
    args=p.parse_args();train,stages,report,sources,freeze=gather(args.root,args.blind_results)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    # Independent data panels, with no animation pretending to be the real-world clock.
    sections=[("真实动作采样",7.0),("共同开发集与冻结",8.0),("独立局部与整圈",8.0),("实测直道与4m/s覆盖",9.0)]
    card_results=result_page(report)
    ends=np.cumsum([round(s*FPS) for _,s in sections]);start=0
    with av.open(str(args.out),"w",options={"movflags":"+faststart"}) as out:
        chapters=[]
        for n,(title,seconds) in enumerate(sections):
            chapters.append(dict(id=n,start=start,end=int(ends[n]),time_base=Fraction(1,FPS),metadata={"title":title}));start=int(ends[n])
        out.set_chapters(chapters)
        video=out.add_stream("libx264",rate=FPS,options={"preset":"veryfast","crf":"19","threads":"4"});video.width,video.height=SIZE;video.pix_fmt="yuv420p"
        # Reuse repeated panels rather than reading log files on every cinema frame.
        cache={}
        for i in range(int(ends[-1])):
            s=int(np.searchsorted(ends,i,side="right"));first=0 if s==0 else ends[s-1]
            fraction=min(1.,(i-first+1)/(max(1,ends[s]-first)*.68))
            key=(s,math.ceil(fraction*(len(train) if s==0 else len(stages) if s==1 else 18 if s==3 else 1)))
            if key not in cache:
                if s==0:frame=sample_page(train,key[1]/len(train))
                elif s==1:frame=dev_page(stages,freeze,key[1]/len(stages))
                elif s==2:frame=card_results
                else:frame=cruise_page(args.root,report,key[1]/18)
                cache.clear();cache[key]=frame
            for packet in video.encode(av.VideoFrame.from_image(cache[key])):out.mux(packet)
        for packet in video.encode():out.mux(packet)
    with av.open(str(args.out)) as movie:
        stream=movie.streams.video[0];num=sum(1 for _ in movie.decode(video=0))
        if num!=int(ends[-1]) or stream.width!=SIZE[0] or int(stream.average_rate)!=FPS:raise ValueError("Training movie timeline validation failed")
    receipt={"sha256":digest(args.out),"sourceHashes":sources,"frames":num,"fps":FPS,"sections":sections,
        "checkpoints":stages,"frozenLayoutCount":freeze,"trainingLayoutCount":len(train),"probeCount":len(train)*len(NAMES),
        "independentCohorts":report["cohorts"],"naturalCruise":report["naturalCruise"]}
    args.out.with_suffix(".json").write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+"\n",encoding="utf8")
    print(args.out,receipt["sha256"])


if __name__=="__main__":main()
