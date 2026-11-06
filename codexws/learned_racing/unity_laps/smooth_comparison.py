"""Show two real Unity runs on their common clock with recorded paths, yaw and physical clearance."""
import argparse
import json
import math
from pathlib import Path

import av
from PIL import Image, ImageChops, ImageDraw, ImageStat

from referee_inset import position
from three_view import FPS, at, camera, digest, label
from timing_comparison import Recording, optical, TERMINAL

SIZE=(2048,1496)
X=(40,1048)
CAMY=146
PAPER="#f0f4f0"; INK="#162e35"; MUTED="#4f6870"; WHITE="#ffffff"; GRID="#d0dcd6"
BASE="#a65313"; FIT="#087b7c"; MOVING="#bc356c"
ASIZE=(288,162); AX,AY=660,10


def read(p):
    return json.loads(p.read_bytes())


def pair(inputs):
    records=[Recording(inputs.root/"runs"/(name+".json"),inputs.root/".work"/"frames"/name,
                inputs.root/".work"/"camera"/name/"a4_samples") for name in (inputs.base,inputs.trained)]
    a,b=(r.run for r in records)
    for k in ("props","length","physicsStep","unity","profile","coupled"):
        if a[k]!=b[k]:raise ValueError(f"Physical scene mismatch: {k}")
    for k in ("model","vision","limit","start","seed","offset","yaw","obstacles"):
        if a["request"].get(k)!=b["request"].get(k):raise ValueError(f"Pair is not matched: {k}")
    if inputs.base==inputs.trained:raise ValueError("Use two independently recorded runs")
    fitted=b["request"].get("smooth") or {}
    if inputs.model_id and (fitted.get("modelId")!=inputs.model_id or fitted.get("mode")=="probe"):
        raise ValueError("This is not the requested frozen learned controller")
    if not (0<=inputs.start<min(r.run["seconds"] for r in records) and inputs.start<inputs.end<=max(r.run["seconds"] for r in records)+2e-5):
        raise ValueError("Invalid shared simulation window")
    return records


def pose_hull(row,project):
    px,pz,angle=row[1:4];sn,cs=math.sin(angle),math.cos(angle)
    return [project(px+a*sn+b*cs,pz+a*cs-b*sn) for a,b in ((.28,0),(.11,.38),(-.88,.38),(-.88,-.38),(.11,-.38))]


class LocalPaths:
    def __init__(self,records,start,end,prop_index):
        a=records[0]
        mid=a.rows[at(a.rt,(start+min(a.run["seconds"],end))*.5)]
        props=a.run["props"]
        anchor=prop_index
        if props and anchor is None: anchor=min(range(len(props)),key=lambda i:abs(props[i][7]-mid[7]))
        if anchor is not None and not 0<=anchor<len(props):raise ValueError("Unknown local prop")
        self.props=[] if anchor is None else [p for p in props if abs(p[7]-props[anchor][7])<=8.]
        if self.props:
            prop=props[anchor];self.center=prop[:2];self.right=prop[2:4]
        else:
            self.center=mid[1:3];self.right=(math.cos(mid[3]),-math.sin(mid[3]))
        self.forward=(-self.right[1],self.right[0])
        lateral=[-2.3,2.3]
        for p in self.props:
            for k in (-1,1):lateral.append(self.local(p[0]+k*p[2]*p[4],p[1]+k*p[3]*p[4])[1])
        for r in records:
            for row in r.rows:
                f,l=self.local(row[1],row[2])
                if start<=row[0]<=end and -10<=f<=7:lateral.append(l)
        self.mid_lat=(max(lateral)+min(lateral))*.5
        self.scale=min(36,250/(max(lateral)-min(lateral)+1))
        self.records=records

    def local(self,x,z):
        dx=x-self.center[0];dz=z-self.center[1]
        return (dx*self.forward[0]+dz*self.forward[1],dx*self.right[0]+dz*self.right[1])

    def project(self,x,z):
        f,l=self.local(x,z)
        return (round(383+f*self.scale),round(164+(l-self.mid_lat)*self.scale))

    def draw(self,samples,t):
        f=Image.new("RGB",(630,330),"#eaf1eb");d=ImageDraw.Draw(f)
        for j in range(-20,15,2):
            x=383+j*self.scale;d.line((x,0,x,330),fill=GRID)
            y=164+(j-self.mid_lat)*self.scale;d.line((0,y,630,y),fill=GRID)
        for p in self.props:
            x,y=self.project(*position(p,t))
            if p[4] and p[6]:
                pts=[self.project(*position(p,max(0,t-1.5+j*.1))) for j in range(16)]
                d.line(pts,fill=MOVING,width=5);d.ellipse((x-8,y-8,x+8,y+8),fill=MOVING,outline=WHITE,width=2)
            else:d.polygon([(x,y-8),(x+8,y+6),(x-8,y+6)],fill="#d69927",outline=INK,width=1)
        for i,(r,s,color) in enumerate(zip(self.records,samples,(BASE,FIT))):
            start=max(0,at(r.rt,s["t"]-2.5));end=at(r.rt,s["t"]);hist=r.rows[start:end+1]
            pts=[self.project(*q[1:3]) for q in hist]
            if len(pts)>1:d.line(pts,fill=color,width=6 if i==0 else 3)
            d.polygon(pose_hull(s["row"],self.project),outline=color,fill="#f6d1af" if i==0 else None,width=4)
        label(d,(10,18),"行进 →",19,MUTED,True)
        d.line((10,310,10+2*self.scale,310),fill=INK,width=2)
        label(d,(20+2*self.scale,310),"2 m",16,MUTED,mono=True)
        return f


def chart_yaw(d,r,s,x,y,w,h,color):
    span=4.;t=s["t"];vmax=1.8
    xx=lambda u:x+w*(1+(u-t)/span)
    yy=lambda u:y+h*(.5-.5*max(-vmax,min(vmax,u))/vmax)
    d.rectangle((x,y,x+w,y+h),fill="#f0f4f1")
    d.line((x,yy(0),x+w,yy(0)),fill="#829a95",width=2)
    for seq,times,func,solid in ((r.decisions,r.dt,lambda q:q["yawCmd"],False),(r.rows,r.rt,lambda q:q[14],True)):
        start=max(0,at(times,t-span));end=at(times,t)
        pts=[(max(x,xx(times[i])),yy(func(seq[i]))) for i in range(start,end+1)]
        for i,(a,b) in enumerate(zip(pts,pts[1:])):
            if solid:d.line((a,b),fill=color,width=4)
            else:
                # A lighter segment followed by an open gap distinguishes the requested yaw rate.
                c=(a[0]+(b[0]-a[0])*.64,a[1]+(b[1]-a[1])*.64)
                d.line((a,c),fill="#657886",width=3)
    label(d,(x+5,y+6),"+1.8",12,MUTED,mono=True,anchor="lt")
    label(d,(x+5,y+h-6),"−1.8",12,MUTED,mono=True,anchor="lb")


def margin(d,x,y,name,value):
    w=290
    label(d,(x,y),name,18,MUTED)
    if value is None:
        label(d,(x+w,y),"无锥桶",20,MUTED,anchor="rm");return
    label(d,(x+w,y),f"{value:.2f} m",21,INK,True,True,"rm")
    bx,by=x,y+21
    d.rectangle((bx,by,bx+w,by+11),fill="#dde5df")
    d.rectangle((bx,by,bx+w*.5,by+11),fill="#e7e1d3")
    d.line((bx+w*.5,by-3,bx+w*.5,by+15),fill=INK,width=2)
    dot=bx+w*max(0,min(1,value))
    d.ellipse((dot-5,by-1,dot+5,by+10),fill=FIT if value>=.5 else BASE)


def selected_text(dec):
    reason=dec.get("reason","")
    tags={"road":"沿道路选线","yield":"避让","recover":"恢复速度","go_first":"选择先行",
          "learned_encounter":"执行学习动作","insufficient_lane":"道路观测不足","hold_pass":"保持通过侧"}
    if dec.get("smooth"):
        e=dec["smooth"]
        if e.get("chosen",-1)>=0 and e.get("state") not in ("observing","idle"):
            return "动作 0 · 沿用 V2" if e["chosen"]==0 else f"动作 {e['chosen']}  ·  目标偏移 {e.get('targetOffset',e.get('chosenOffset',0)):+.2f} m"
    return tags.get(reason,reason)


def render(args):
    recs=pair(args);scene=LocalPaths(recs,args.start,args.end,args.prop)
    count=math.ceil((args.end-args.start)*FPS-1e-7)
    terminal=any(abs(args.end-r.run["seconds"])<.00002 and r.run["status"] in ("obstacle","boundary") for r in recs)
    hold=round(.8*FPS) if terminal else 0
    base=Image.new("RGB",SIZE,PAPER);d=ImageDraw.Draw(base)
    d.rectangle((0,0,SIZE[0],90),fill=INK);label(d,(40,45),args.title,39,WHITE,True)
    for x,title,color in zip(X,(args.base_label,args.trained_label),(BASE,FIT)):label(d,(x,119),title,29,color,True)
    for x1,x2,head in ((27,693,"场地俯视 · 裁判"),(710,1369,"车身横摆 · 最近 4 秒"),(1385,2019,"实测余量 · 米")):
        d.rounded_rectangle((x1,824,x2,1281),radius=15,fill=WHITE);label(d,(x1+18,858),head,26,INK,True)
    label(d,(43,1306),"同一仿真时钟  ·  路径与余量来自裁判；目标偏航和轮速差来自车辆发出的控制指令。",21,MUTED)
    d.rectangle((0,1342,SIZE[0],SIZE[1]),fill=INK)
    label(d,(42,1375),"赛后裁判记录",25,WHITE,True)
    for j,(r,x,title) in enumerate(zip(recs,X,(args.base_label,args.trained_label))):
        rule=TERMINAL.get(r.run["status"],r.run["status"])
        gap=f"桶 {r.run['minGap']:.2f} m   /  " if r.run["props"] else "场内无桶   /  "
        label(d,(x,1439),f"{title}  ·  {rule}     最小净距   "+gap+f"边界 {r.run['minRoad']:.2f} m",26,
              "#ffcd9e" if j==0 else "#b3ebe1",True)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    checks={0,count//2,count-1,count+hold-1};samples_checked=[];missing=[[0,0],[0,0]]
    with av.open(str(args.out),"w",options={"movflags":"+faststart"}) as out:
        v=out.add_stream("libx264",rate=FPS,options={"preset":"veryfast","crf":"19","threads":"4"});v.width,v.height=SIZE;v.pix_fmt="yuv420p"
        for index in range(count+hold):
            t=args.end if index>=count else args.start+index/FPS
            samples=[r.sample(t) for r in recs];im=base.copy();d=ImageDraw.Draw(im)
            time_text=f"同一仿真时钟   {t:.2f} s   /   "+("终止停帧" if index>=count else "1×")
            label(d,(2003,45),time_text,25,WHITE,True,anchor="rm")
            for j,(x,r,s,col) in enumerate(zip(X,recs,samples,(BASE,FIT))):
                a4=r.overlays/f"{s['decision_id']:05d}.jpg";real=camera(s["file"]);obs=optical(a4)
                if real is None:missing[j][0]+=1
                if obs is None:missing[j][1]+=1
                if real is not None:im.paste(real,(x,CAMY))
                ox,oy=x+AX,CAMY+AY;d.rectangle((ox-2,oy-2,ox+ASIZE[0]+1,oy+ASIZE[1]+26),fill=INK,outline=WHITE)
                if obs is not None:im.paste(obs,(ox,oy))
                label(d,(ox+5,oy+ASIZE[1]+14),f"A4 前视  ·  {s['decision']['t']:.2f} s",15,WHITE)
                if s["stopped"]:
                    d.rectangle((x,CAMY+480,x+959,CAMY+539),fill="#61342b")
                    label(d,(x+16,CAMY+510),TERMINAL.get(r.run["status"],r.run["status"])+f"   /   {s['t']:.2f} s 停帧",26,WHITE,True)
                conf=s["decision"];row=s["row"]
                label(d,(x,722),f"实速 {row[4]:.2f} m/s",31,col,True)
                label(d,(x+318,723),f"目标 {conf['speedCmd']:.2f} m/s",24,MUTED)
                label(d,(x+957,724),selected_text(conf),22,INK,True,anchor="rm")
                label(d,(x,782),f"实测横摆 {row[14]:+.2f} rad/s",24,INK,True)
                label(d,(x+415,782),f"偏航指令 {conf['yawCmd']:+.2f} rad/s",22,MUTED)
                label(d,(x+957,782),f"左−右目标轮速 {conf['leftCan']-conf['rightCan']:+d} rpm",19,MUTED,anchor="rm")
                label(d,(735,913+j*157),args.base_label if j==0 else args.trained_label,18,col,True)
                chart_yaw(d,r,s,741,934+j*157,592,96,col)
                label(d,(1410,920+j*131),args.base_label if j==0 else args.trained_label,22,col,True)
                margin(d,1412,961+j*131,"锥桶",row[10] if r.run["props"] else None)
                margin(d,1715,961+j*131,"道路边界",row[9])
            im.paste(scene.draw(samples,t),(45,890))
            label(d,(45,1249),f"橙 · {args.base_label}    青 · {args.trained_label}    最近 2.5 s 轨迹",19,MUTED)
            label(d,(736,1249),"彩色实线：车身实测     灰虚线：偏航指令（rad/s）",18,MUTED)
            label(d,(1411,1199),"黑线 = 0.50 m",21,INK,True)
            label(d,(1411,1244),"超过 0.50 m 的部分不增加余量奖励。",19,MUTED)
            if index in checks:
                samples_checked.append({"i":index,"simTime":t,"sources":[{"chase":s["frame_id"],"a4":s["decision_id"],"chaseS":s["frame_t"],"a4S":s["decision"]["t"],"stopped":s["stopped"]} for s in samples]})
            for q in v.encode(av.VideoFrame.from_image(im)):out.mux(q)
        for q in v.encode():out.mux(q)
    if any(n for m in missing for n in m):raise ValueError(f"Missing genuine camera observations: {missing}")
    indexed={s["i"]:s for s in samples_checked}
    with av.open(str(args.out)) as inp:
        stream=inp.streams.video[0]
        if stream.frames!=count+hold or int(stream.average_rate)!=FPS:raise ValueError("Unexpected encoded clock")
        for i,frame in enumerate(inp.decode(video=0)):
            if i not in indexed:continue
            img=frame.to_image()
            for x,r,s in zip(X,recs,indexed[i]["sources"]):
                path=r.frame_dir/f"{s['chase']:04d}c.jpg";a4=r.overlays/f"{s['a4']:05d}.jpg"
                height=473 if s["stopped"] else 540
                delta=ImageChops.difference(img.crop((x,CAMY,x+960,CAMY+height)),camera(path).crop((0,0,960,height)))
                mask=Image.new("L",delta.size,255);ImageDraw.Draw(mask).rectangle((AX-2,AY-2,AX+ASIZE[0]+1,AY+ASIZE[1]+26),fill=0)
                ox,oy=x+AX,CAMY+AY
                small=ImageChops.difference(img.crop((ox,oy,ox+ASIZE[0],oy+ASIZE[1])),optical(a4))
                s["decodedMeanRGBError"]=[round(sum(ImageStat.Stat(delta,mask).mean)/3,3),round(sum(ImageStat.Stat(small).mean)/3,3)]
                s["sourceSHA256"]=[digest(path),digest(a4)]
                if max(s["decodedMeanRGBError"])>8:raise ValueError("Original camera comparison failed")
    report={"title":args.title,"ids":[args.base,args.trained],"rangeS":[args.start,args.end],"holdsS":hold/FPS,
            "frames":count+hold,"size":SIZE,"fps":FPS,"mp4SHA256":digest(args.out),"runSHA256":[digest(r.path) for r in recs],
            "terminalStates":[r.run["status"] for r in recs],"physicalProps":recs[0].run["props"],"sampledFrames":samples_checked}
    args.out.with_suffix(".json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf8")
    print(json.dumps({k:report[k] for k in ("ids","frames","mp4SHA256")},ensure_ascii=False))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--root",type=Path,required=True)
    for name in ("base","trained","title"):p.add_argument("--"+name,required=True)
    p.add_argument("--base-label",default="冻结基线");p.add_argument("--trained-label",default="长训策略")
    p.add_argument("--prop",type=int);p.add_argument("--out",required=True,type=Path)
    p.add_argument("--model-id",help="Require the exact frozen smooth-controller model ID")
    for name in ("start","end"):p.add_argument("--"+name,type=float,required=True)
    render(p.parse_args())
