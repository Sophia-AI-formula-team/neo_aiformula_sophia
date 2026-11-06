"""Make an original-speed, same-clock comparison of two recorded policies on one encounter."""
import argparse
import bisect
import json
import math
from pathlib import Path

import av
from PIL import Image, ImageChops, ImageDraw, ImageStat

from referee_inset import position
from three_view import FPS, INK, MUTED, PAPER, SIZE, WHITE, at, camera, digest, label

LEFT, RIGHT = "#9b4908", "#006e6f"
MOVING = "#b72865"
XPOS = (42,1046)
CHASE_Y = 157
TERMINAL = {"obstacle":"碰桶终止", "boundary":"触边终止", "lap":"完成一圈", "segment":"完成录制路段",
            "stalled":"进度停滞", "lost_lane":"道路观测丢失", "invalid_progress":"进度无效"}
REASONS = {"road":"按弯度调速", "yield":"减速避让", "recover":"恢复车速", "hold_pass":"保持避让侧",
           "yield_corridor":"寻找可用空间", "insufficient_lane":"道路观测不足", "go_first":"选择加速先行"}


def short_seconds(value):
    return "—" if value is None else f"{value:.2f} s"


class Recording:
    def __init__(self, path, frame_dir):
        self.path, self.frame_dir = path, frame_dir
        self.run=json.loads(path.read_text(encoding="utf-8-sig"))
        self.frames=self.run.get("frames",[])
        self.decisions=self.run.get("decisions",[])
        self.rows=self.run.get("rows",[])
        if not self.frames or not self.decisions or not self.rows:
            raise ValueError(f"{path} lacks original camera timeline, commands, or recorded poses")
        self.ft=[p[0] for p in self.frames]
        self.dt=[p["t"] for p in self.decisions]
        self.rt=[p[0] for p in self.rows]
        if not any(frame_dir.glob("*c.jpg")):
            raise ValueError(f"No actual chase footage in {frame_dir}")
        for times in (self.ft,self.dt,self.rt):
            if any(a>=b for a,b in zip(times,times[1:])):
                raise ValueError(f"Unordered source timestamps in {path}")

    def sample(self,t):
        when=min(t,self.run["seconds"])
        fi,di,ri=(at(v,when) for v in (self.ft,self.dt,self.rt))
        if min(fi,di,ri)<0:
            raise ValueError("Requested interval precedes footage/decisions/poses")
        number=int(self.frames[fi][1])
        path=self.frame_dir/f"{number:04d}c.jpg"
        return {"t":when,"frame_t":self.ft[fi], "file":path,"frame_id":number,
                "row":self.rows[ri],"decision":self.decisions[di],"stopped":t>=self.run["seconds"]-2e-5}


def check_pair(left,right,prop):
    a,b=left.run,right.run
    for key in ("seed","start","offset","yaw","obstacles","model","vision"):
        if a["request"].get(key)!=b["request"].get(key):
            raise ValueError(f"Unmatched experiment ({key}); refusing a same-scene comparison")
    if a["props"]!=b["props"] or a.get("profile")!=b.get("profile") or a.get("coupled")!=b.get("coupled"):
        raise ValueError("Physical scene or plant mismatch")
    if len(a["props"])<=prop or prop<0 or abs(a["props"][prop][4])<1e-6:
        raise ValueError("The selected prop must be a recorded moving obstacle")
    if any(abs(x-y)>.001 for x,y in zip(a["rows"][0][1:4],b["rows"][0][1:4])):
        raise ValueError("Recorded spawn poses differ")
    if left.path.resolve()==right.path.resolve() or a["decisions"]==b["decisions"]:
        raise ValueError("Identical driving record; it cannot support a policy comparison")


class Referee:
    def __init__(self, prop, records, start, end):
        self.prop=prop
        rx,rz=prop[2:4]
        n=math.hypot(rx,rz)
        self.right=rx/n,rz/n
        self.forward=-self.right[1],self.right[0]
        lateral=[-abs(prop[4]),abs(prop[4])]
        for rec in records:
            for row in rec.rows:
                if not start<=row[0]<=end:continue
                f,r=self.axes(row[1],row[2])
                if -15<=f<=7:lateral.append(r)
        self.center_lat=(max(lateral)+min(lateral))/2
        self.scale=min(40,246/(max(lateral)-min(lateral)+1.8))

    def axes(self,x,z):
        dx,dz=x-self.prop[0],z-self.prop[1]
        return (dx*self.forward[0]+dz*self.forward[1],dx*self.right[0]+dz*self.right[1])

    def point(self,f,r):
        return round(638+f*self.scale),round(165+(r-self.center_lat)*self.scale)

    def project(self,x,z):
        return self.point(*self.axes(x,z))

    def draw(self,records,samples,t):
        surface=Image.new("RGB",(936,310),"#e9f1ef");d=ImageDraw.Draw(surface)
        for f in range(-24,15,2):
            x,_=self.point(f,self.center_lat);d.line((x,0,x,310),fill="#cbdad6",width=1)
        for r in range(-14,15,2):
            _,y=self.point(0,r);d.line((0,y,936,y),fill="#cbdad6",width=1)
        gx,_=self.point(0,0)
        d.line((gx,0,gx,310),fill="#7b9091",width=2)
        label(d,(gx+7,15),"桶所在断面",18,MUTED)
        for n,(rec,s,color) in enumerate(zip(records,samples,(LEFT,RIGHT))):
            start=bisect.bisect_left(rec.rt,s["t"]-1.5)
            finish=at(rec.rt,s["t"])
            pts=[self.project(row[1],row[2]) for row in rec.rows[start:finish+1]]
            if len(pts)>1:d.line(pts,fill=color,width=4 if n==0 else 2)
            row=s["row"]; x,z,yaw=row[1:4];fw=(math.sin(yaw),math.cos(yaw));rt=(fw[1],-fw[0])
            def body(a,b):
                return self.project(x+a*fw[0]+b*rt[0],z+a*fw[1]+b*rt[1])
            hull=[body(a,b) for a,b in ((.28,0),(.1,.38),(-.88,.38),(-.88,-.38),(.1,-.38))]
            if n==0:
                d.polygon(hull,fill="#f3c79c",outline=color,width=4)
            else:
                d.polygon(hull,fill=None,outline=color,width=3)
                d.line((body(-.6,0),body(.18,0)),fill=color,width=3)
        rail=[self.point(0,sgn*abs(self.prop[4])) for sgn in (-1,1)]
        d.line(rail,fill="#c798aa",width=3)
        for x,y in rail:d.line((x-7,y,x+7,y),fill="#c798aa",width=2)
        path=[self.project(*position(self.prop,max(0,t-1.5+j*.1))) for j in range(16)]
        d.line(path,fill=MOVING,width=5)
        x,y=path[-1];d.ellipse((x-9,y-9,x+9,y+9), fill=MOVING,outline=WHITE,width=2)
        label(d,(12,16),"行进 →",19,MUTED,True)
        step=2*self.scale
        d.line((15,289,15+step,289),fill=INK,width=3)
        label(d,(19+step,289),"2 m",16,MUTED,mono=True)
        return surface


def draw_metric_band(d,x,s,name,color):
    row,dec=s["row"],s["decision"]
    label(d,(x,733),name,25,color,True)
    label(d,(x+185,733),"实测",22,MUTED)
    label(d,(x+242,733),f"{row[4]:.2f}",38,INK,True,True)
    label(d,(x+335,736),"m/s",21,MUTED,mono=True)
    label(d,(x+421,733),"目标",22,MUTED)
    label(d,(x+479,733),f"{dec['speedCmd']:.2f}",38,color,True,True)
    label(d,(x+573,736),"m/s",21,MUTED,mono=True)
    reason=REASONS.get(dec["reason"],dec["reason"])
    label(d,(x+956,734),reason,22,color,True,anchor="rm")


def stage(section,t):
    enter=section.get("frontEnterS")
    clear=section.get("rearClearS")
    if enter is None:return "当前记录没有进入断面"
    if t<enter:return f"车头距入区 {enter-t:.2f} s"
    if clear is None or t<clear:return "车身正在通过断面"
    return f"车尾已离开 {t-clear:.2f} s"


def verdict(section,run_end):
    if section.get("rearClearS") is None:return "记录结束时，车尾尚未证实清空断面"
    lead=section.get("rearLeadS")
    if lead is None:return "随后的 3 秒内未证实桶进入同一通行带"
    if lead<=0:return "车尾清空时，同一通行带已被占用"
    if section.get("nextConeEntersCrossedStripS",0)>run_end:return f"按场景运动推算，车尾早 {lead:.2f} s 清空"
    return f"车尾比桶进入同一通行带早 {lead:.2f} s"


def draw_audit(d,records,samples,audit):
    label(d,(1056,820),"断面与通行时间",31,INK,True)
    label(d,(1980,825),"裁判复核  /  仿真时钟",20,MUTED,anchor="rm")
    for n,(which,color) in enumerate((("off",LEFT),("on",RIGHT))):
        y=890+117*n
        data=audit.get(which,{})
        sec=data.get("section",{})
        caption="基线" if n==0 else "候选"
        if not sec:
            label(d,(1058,y),caption+" · 缺少断面审计",25,color,True)
            continue
        label(d,(1058,y),caption+" · "+stage(sec,samples[n]["t"]),26,color,True)
        label(d,(1058,y+46),"车头进入  "+short_seconds(sec.get("frontEnterS"))+
              "     车尾清空  "+short_seconds(sec.get("rearClearS")),22,INK)
        gap=data.get("minConeClearanceM")
        gap_text="" if gap is None else f"  ·  最小净距 {gap:.2f} m"
        label(d,(1058,y+79),verdict(sec,records[n].run["seconds"])+gap_text,20,MUTED)
    d.line((1058,1109,1976,1109),fill="#cbdad6",width=2)
    label(d,(1058,1144),"规划器当时的判断",23,INK,True)


def prediction(dec,first_go):
    """Read the candidate's recorded per-cycle prediction when available."""
    e=dec.get("timing") or {}
    if e.get("rearFast",0)>0 and e.get("rearPace",0)>0:
        return (f"预计车尾清空  ·  提速 {e['rearFast']:.2f} s   /   维持 {e['rearPace']:.2f} s"
                f"   ·   提速预计净余量 {e['marginFast']:.2f} m")
    if e.get("state")=="committed":
        source=(first_go or {}).get("timing") or {}
        last=(f"  ·  {first_go['t']:.2f} s 时预计提速后 {source['rearFast']:.2f} s 清空"
              if source.get("rearFast",0)>0 else "")
        return f"继续先行，剩余 {e['remaining']:.2f} m"+last
    states={"no_track":"尚无可用目标", "multiple_tracks":"多目标，不选先行",
            "motion_unconfirmed":"运动估计尚未确认","no_approaching_motion":"未确认目标向通行带靠近",
            "outside_envelope":"当前条件不进入先行评估", "abort":"撤销先行计划","finished":"先行计划结束"}
    if e.get("state") in states:
        return states[e["state"]]+"  ·  本周期没有新的清空时间预测"
    return "当前记录未提供预计清空时间"


def render(args):
    records=(Recording(args.baseline,args.baseline_frames),Recording(args.candidate,args.candidate_frames))
    check_pair(*records,args.prop)
    audit=json.loads(args.audit.read_text(encoding="utf-8-sig")) if args.audit else {}
    if audit and any(audit.get(which,{}).get("id")!=r.run.get("id") for which,r in zip(("off","on"),records)):
        raise ValueError("Audit IDs must match the actual baseline and candidate recordings")
    # End holds are explicit: do not manufacture new motion or commands after either run stopped.
    if not (0<=args.start<args.end<=max(r.run["seconds"] for r in records)+.00002):
        raise ValueError("Comparison must overlap the actual common-start timeline")
    if args.start>=min(r.run["seconds"] for r in records):
        raise ValueError("Comparison must start while both recorded cars are active")
    for r in records:
        first=max(0,at(r.ft,args.start))
        last=at(r.ft,min(args.end,r.run["seconds"]))
        if not any((r.frame_dir/f"{int(f[1]):04d}c.jpg").is_file() for f in r.frames[first:last+1]):
            raise ValueError(f"No real chase frames for {r.path.name} in the requested window")
    judge=Referee(records[0].run["props"][args.prop],records,args.start,args.end)
    first_go=next((d for d in records[1].decisions if d["reason"]=="go_first"),None)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    nframes=math.ceil((args.end-args.start)*FPS-1e-7)
    # Include the exact terminal camera tick for an encounter truncated by a collision/boundary.
    include_end=any(abs(r.run["seconds"]-args.end)<2e-5 and r.run["status"] in ("obstacle","boundary") for r in records)
    if include_end:nframes+=1
    template=Image.new("RGB",SIZE,PAPER);d=ImageDraw.Draw(template)
    d.rectangle((0,0,2048,94),fill=INK)
    label(d,(44,46),args.title,41,WHITE,True)
    for x,text,color in zip(XPOS,("冻结基线","时间规划候选"),(LEFT,RIGHT)):
        label(d,(x,125),text,31,color,True)
    d.rounded_rectangle((31,782,1002,1228),radius=16,fill=WHITE)
    d.rounded_rectangle((1038,782,2009,1228),radius=16,fill=WHITE)
    label(d,(55,819),"场地俯视 · 裁判",31,INK,True)
    label(d,(978,824),"两趟共用固定场地坐标",20,MUTED,anchor="rm")
    for x,txt,col in ((57,"基线车",LEFT),(210,"候选车",RIGHT),(362,"移动桶及 1.5 秒尾迹",MOVING)):
        label(d,(x,1198),txt,21,col,True)
    label(d,(43,1254),"两个追车机位均取自对应试验。下方裁判俯视和通行时间供回放核对，右侧预测只取当时的车端记录。",22,MUTED)
    checks=[];missing=[0,0];max_age=[0.,0.]
    with av.open(str(args.out),"w",options={"movflags":"+faststart"}) as out:
        video=out.add_stream("libx264",rate=FPS,options={"preset":"veryfast","crf":"18","threads":"4"})
        video.width,video.height=SIZE;video.pix_fmt="yuv420p"
        for n in range(nframes):
            t=min(args.end,args.start+n/FPS)
            samples=[r.sample(t) for r in records]
            canvas=template.copy();d=ImageDraw.Draw(canvas)
            label(d,(2004,44),f"同一仿真时钟    {t:.2f} s   /   1×",27,WHITE,True,anchor="rm")
            for j,(x,s,r) in enumerate(zip(XPOS,samples,records)):
                src=camera(s["file"])
                if src is None:
                    d.rectangle((x,CHASE_Y,x+959,CHASE_Y+539),fill="#d6e1de")
                    label(d,(x+480,CHASE_Y+270),"该采样没有真实追车影像",27,MUTED,anchor="mm")
                    missing[j]+=1
                else:
                    canvas.paste(src,(x,CHASE_Y))
                    max_age[j]=max(max_age[j],s["t"]-s["frame_t"])
                if s["stopped"]:
                    d.rectangle((x,CHASE_Y+476,x+959,CHASE_Y+539),fill="#572e27")
                    status=TERMINAL.get(r.run["status"],r.run["status"])
                    label(d,(x+15,CHASE_Y+507),f'{status}  ·  {s["t"]:.2f} s 停帧',27,WHITE,True)
                draw_metric_band(d,x,s,"车速与动作",(LEFT,RIGHT)[j])
            canvas.paste(judge.draw(records,samples,t),(53,855))
            draw_audit(d,records,samples,audit)
            decision=samples[1]["decision"]
            known_go=first_go if first_go and first_go["t"]<=decision["t"] else None
            if known_go:
                label(d,(2001,127),f"首次选择先行  {known_go['t']:.2f} s",22,RIGHT,anchor="rm")
            label(d,(1058,1187),prediction(decision,known_go),22,MUTED)
            if n in {0,nframes//2,nframes-1}:
                checks.append({"videoFrame":n,"simS":t,"sources":[{"sourceFrame":s["frame_id"],"sourceS":s["frame_t"],
                              "decisionS":s["decision"]["t"],"poseS":s["row"][0],"heldAfterEnd":s["stopped"],
                              "sha256":digest(s["file"]) if s["file"].is_file() else None} for s in samples]})
            for packet in video.encode(av.VideoFrame.from_image(canvas)):out.mux(packet)
        for packet in video.encode():out.mux(packet)
    indexed={k["videoFrame"]:k for k in checks}
    with av.open(str(args.out)) as playback:
        stream=playback.streams.video[0]
        if stream.frames!=nframes or stream.average_rate!=FPS or stream.codec_context.name!="h264":
            raise ValueError("Incorrect output timing or codec")
        for index,frame in enumerate(playback.decode(video=0)):
            if index not in indexed:continue
            sample=indexed[index];img=frame.to_image()
            for x,r,s in zip(XPOS,records,sample["sources"]):
                if not s["sha256"]:continue
                reference=camera(r.frame_dir/f"{s['sourceFrame']:04d}c.jpg")
                height=470 if s["heldAfterEnd"] else 540
                delta=ImageChops.difference(img.crop((x,CHASE_Y,x+960,CHASE_Y+height)),reference.crop((0,0,960,height)))
                s["decodedMeanRGBError"]=round(sum(ImageStat.Stat(delta).mean)/3,3)
                if s["decodedMeanRGBError"]>8:raise ValueError("Decoded video differs from the referenced original camera pixels")
    evidence={"title":args.title,"fps":FPS,"width":SIZE[0],"height":SIZE[1],"playback":"1x common simulation clock",
              "startS":args.start,"endS":args.end,"terminalEndIncluded":include_end,"frames":nframes,
              "prop":args.prop,"runs":[r.run.get("id") for r in records],"trajectorySHA256":[digest(r.path) for r in records],
              "auditSHA256":digest(args.audit) if args.audit else None,"missingCameraFrames":missing,"maxCameraAgeS":max_age,
              "groundTruth":"Fixed prop coordinates from UnityRace.PropPosition; car from latest recorded referee pose. The two runs share an identical scene clock.",
              "checkFrames":checks}
    args.out.with_suffix(".json").write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+"\n",encoding="utf8")
    return evidence


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("baseline","baseline-frames","candidate","candidate-frames","out"):
        p.add_argument("--"+name,type=Path,required=True)
    p.add_argument("--audit",type=Path)
    p.add_argument("--prop",type=int,required=True,help="Zero-based moving prop index")
    p.add_argument("--title",default="横移锥桶 · 同场双车对照")
    for name in ("start","end"):p.add_argument("--"+name,type=float,required=True)
    print(json.dumps(render(p.parse_args()),ensure_ascii=False))


if __name__=="__main__":
    main()
