"""Compose original Unity frame pairs, synchronized policy telemetry and measured track geometry."""
import argparse
import json
from functools import lru_cache
from fractions import Fraction
from pathlib import Path
import av
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scipy.ndimage import gaussian_filter1d
from run import HERE,SIM,outer_verge,resample,write

RAW=HERE/"evidence/runs";WORK=HERE/"evidence/.work";OUT=HERE/"replay"
BG="#111c22";INK="#eef4ed";MUTE="#a4b7b7";LIME="#c5ee71";BLUE="#72cbde";ORANGE="#ffae67";PANEL="#1b2a31"
FPS=18

@lru_cache()
def font(size,bold=False,mono=False):
    name="consolab.ttf" if mono and bold else "consola.ttf" if mono else "msyhbd.ttc" if bold else "msyh.ttc"
    return ImageFont.truetype("C:/Windows/Fonts/"+name,size)

def label(draw,x,y,value,size=20,color=INK,bold=False,mono=False):
    draw.text((x,y),str(value),font=font(size,bold,mono),fill=color,anchor="lt")

def track():
    surface=SIM/"Simulator/Assets/TrainingEnvironment/Surface"
    vertices=outer_verge(json.loads((surface/"track_surface.json").read_bytes())["meshes"])
    cal=json.loads((surface/"TrackMetricCalibration.json").read_bytes());a=np.deg2rad(cal["unityYawDegrees"])
    pos=gaussian_filter1d(resample(vertices,.35),1,axis=0,mode="wrap")
    outer=pos@np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])*cal["scale"]+[cal["offsetX"],cal["offsetZ"]]
    delta=np.roll(outer,-1,axis=0)-np.roll(outer,1,axis=0);delta/=np.linalg.norm(delta,axis=1)[:,None]
    right=np.column_stack([delta[:,1],-delta[:,0]]);lane=cal["outerLaneCenterWidthM"]
    return np.array([outer-right*i for i in (0.,lane,2*lane)])

def coords(xy,bounds,region):
    x,y,w,h=region;lo,hi=bounds;scale=min(w/(hi[0]-lo[0]),h/(hi[1]-lo[1]))
    center=(lo+hi)/2;return (xy-center)*[scale,-scale]+[x+w/2,y+h/2]

def tuples(arr):return [tuple(map(float,row)) for row in arr]

def at_zone(run,s):
    groups=["单桶","连续双桶","连续双桶","横移路障"]
    if run["props"]:
        ds=[o[-1]-s for o in run["props"]];i=int(np.argmin(np.abs(ds)))
        if -7<ds[i]<17:return groups[i],ds[i],i
    return "赛道行驶",None,None

def chapters(run):
    rows=np.array(run["rows"]);list_=[]
    for i,caption in ((0,"单桶"),(1,"连续双桶"),(3,"横移路障")):
        if len(run["props"])<=i:continue
        station=run["props"][i][-1];end=run["props"][2][-1] if i==1 else station
        if rows[-1,7]<station-13:continue
        start=float(np.interp(station-13,rows[:,7],rows[:,0]));stop=float(np.interp(end+8,rows[:,7],rows[:,0]))
        list_.append((caption,start,stop))
    return list_

def obstacle_plan(geom,run,r,prior,index):
    """Referee-only close view fixed at the cone, so its lateral motion is visible independently of the car."""
    plane=Image.new("RGB",(221,134),PANEL);g=ImageDraw.Draw(plane)
    o=run["props"][index];right=np.array(o[2:4]);forward=np.array([-right[1],right[0]])
    origin=np.asarray(o[:2])-forward*1.8
    def q(points):
        diff=np.asarray(points)-origin
        return np.stack([111+8.2*(diff@right),64-8.2*(diff@forward)],axis=-1)
    g.polygon(tuples(q(geom[0])),fill="#44514d");g.polygon(tuples(q(geom[2])),fill=PANEL)
    for j in (0,1,2):g.line(tuples(q(geom[j]))+[tuple(q(geom[j,0]))],fill="#a2b3aa" if j!=1 else "#778881",width=1)
    g.line(tuples(q(prior[:,1:3])),fill=BLUE,width=2)
    def marker(prop,t):
        return q(np.array(prop[:2])+np.array(prop[2:4])*prop[4]*np.sin(prop[5]+t*prop[6]))
    for prop in run["props"]:
        x,y=marker(prop,r[0])
        if prop[4]:
            g.line([tuple(marker(prop,t)) for t in np.linspace(max(0,r[0]-2.0),r[0],24)],fill="#fbc9a0",width=3)
        g.polygon([(x,y-4),(x-4,y+3.5),(x+4,y+3.5)],fill=ORANGE)
    car=np.array(r[1:3]);front=np.array([np.sin(r[3]),np.cos(r[3])]);side=np.array([front[1],-front[0]])
    pts=[car+i*front+j*side for i,j in ((.28,-.38),(.28,.38),(-.88,.38),(-.88,-.38))]
    g.polygon(tuples(q(pts)),fill=LIME);label(g,4,4,"裁判俯视 · 局部放大",11,INK)
    return plane

def compose(run_id,name,title,highlight=False):
    run=json.loads((RAW/(run_id+".json")).read_bytes());frames=run["frames"];rows=np.array(run["rows"])
    if not frames:raise ValueError("No actual images "+run_id)
    OUT.mkdir(exist_ok=True);media=OUT/(name+".mp4");poster=OUT/(name+".jpg");clips=chapters(run) if highlight else []
    indices=[int(i) for t,i in frames if not clips or any(a<=t<=b for _,a,b in clips)]
    if not indices:raise ValueError("No frames inside requested zones")
    vs=run["request"]["vision"];release=run["request"]["model"].split("/")[0]
    overlay=WORK/"camera"/run_id/(release+"_samples");stride=1
    if vs:
        with (WORK/"camera"/run_id/"ready.json").open() as f:stride=json.load(f).get("sampleStride",5)
    frame_dir=WORK/"frames"/run_id;geom=track();bounds=(geom.reshape(-1,2).min(0),geom.reshape(-1,2).max(0))
    def xy(arr):return coords(np.asarray(arr),bounds,(817,376,213,117))
    base=Image.new("RGB",(1280,720),BG);b=ImageDraw.Draw(base)
    label(b,26,17,"AI FORMULA",18,LIME,True,True);label(b,150,16,"/   UNITY DRIVING LAB",17,MUTE,False,True)
    label(b,23,43,title,32,INK,True)
    label(b,26,102,"追车机位",17,INK,True);label(b,815,102,("车载相机  /  "+release.upper()+" 识别") if vs else "车载相机  /  原始 RGB",17,INK,True)
    b.rounded_rectangle((807,357,1254,519),radius=10,fill=PANEL)
    for j in (0,2,1):
        pts=tuples(xy(geom[j]));b.line(pts+[pts[0]],fill="#859491" if j!=1 else "#4c5d60",width=2 if j!=1 else 1)
    label(b,1043,378,"最近路障净距",14,MUTE);label(b,1043,445,"车身距路边",14,MUTE)
    b.line((25,572,1254,572),fill="#34464b",width=1)
    label(b,26,588,"车速随圈内距离",18,INK,True);label(b,164,591,"m/s",15,MUTE,False,True);label(b,230,591,"实测",15,BLUE);label(b,297,591,"指令",15,LIME)
    chart=(70,622,1130,61);cx,cy,cw,ch=chart;maxv=max(3,int(np.ceil(max(rows[:,4].max(),rows[:,5].max()))))
    for v in (0,maxv):
        yy=cy+ch-v/maxv*ch;b.line((cx,yy,cx+cw,yy),fill="#334349",width=1);label(b,27,yy-6,str(v),14,MUTE,False,True)
    for m in (0,70,140,210,280):label(b,cx+m/run["length"]*cw-7,690,str(m),13,MUTE,False,True)
    label(b,1218,685,"m",16,MUTE)
    def graph(i):
        return np.column_stack([cx+rows[:,7]/run["length"]*cw,cy+ch-rows[:,i]/maxv*ch])
    chart_v,chart_cmd=graph(4),graph(5)
    b.line(tuples(chart_v[::3]),fill="#435c65",width=2)
    if len(chart_cmd)>1:b.line(tuples(chart_cmd[::3]),fill="#4c5945",width=1)
    path=xy(rows[:,1:3]);f0=xy(rows[0,1:3]);b.ellipse((*tuple(f0-3),*tuple(f0+3)),outline=INK,width=2)
    times=[];poster_at=int(len(indices)*.36)
    rate=Fraction(1,1)/Fraction(str(run["request"]["captureInterval"])) if highlight else Fraction(FPS)
    fps=float(rate)
    with av.open(str(media),"w",options={"movflags":"+faststart"}) as container:
        stream=container.add_stream("libx264",rate=rate);stream.width,stream.height=1280,720
        stream.pix_fmt="yuv420p";stream.options={"crf":"21","preset":"veryfast"}
        for pos,index in enumerate(indices):
            t=frames[index][0];times.append(t);i=max(0,min(len(rows)-1,int(np.searchsorted(rows[:,0],t+.001,side="right")-1)));r=rows[i]
            img=base.copy();draw=ImageDraw.Draw(img)
            with Image.open(frame_dir/f"{index:04d}c.jpg") as shot:img.paste(shot.resize((768,432),Image.Resampling.BILINEAR),(24,128))
            fp=frame_dir/f"{index:04d}f.jpg"
            if vs:
                sample=int((t+.001)/.20)//stride*stride;candidate=overlay/f"{sample:05d}.jpg"
                if candidate.exists():fp=candidate
            with Image.open(fp) as shot:img.paste(shot.resize((439,247),Image.Resampling.BILINEAR),(815,128))
            # White circle marks the initial position. Moving markers use the actual referee trajectory.
            if i>1:draw.line(tuples(path[:i+1:2]),fill=BLUE,width=2)
            for j,o in enumerate(run["props"]):
                obj=xy(np.asarray(o[:2])+np.asarray(o[2:4])*o[4]*np.sin(o[5]+t*o[6]));x,y=obj
                draw.polygon([(x,y-4),(x-3.5,y+3.5),(x+3.5,y+3.5)],fill=ORANGE)
            x,y=path[i];draw.ellipse((x-4,y-4,x+4,y+4),fill=LIME)
            label(draw,1042,404,f"{r[10]:.2f} m" if r[10]<15 else "—",25,ORANGE,True,True)
            label(draw,1042,469,f"{r[9]:.2f} m",23,INK,True,True)
            zone,dist,which=at_zone(run,r[7])
            if which is not None:
                img.paste(obstacle_plan(geom,run,r,rows[max(0,i-50):i+1],which),(814,380))
            state=zone+(("  ·  前方 %.1f m"%dist) if dist is not None and dist>0 else "")
            label(draw,814,534,state,18,ORANGE if dist is not None else MUTE,True)
            label(draw,770,20,"仿真时间",15,MUTE);label(draw,768,48,f"{t:05.1f} s",26,INK,True,True)
            label(draw,930,20,"圈内距离",15,MUTE);label(draw,928,48,f"{r[7]:05.1f} m",26,INK,True,True)
            label(draw,1110,20,"实测车速",15,MUTE);label(draw,1108,48,f"{r[4]:.2f} m/s",26,LIME,True,True)
            if i>=2:
                draw.line(tuples(chart_cmd[:i+1:2]),fill=LIME,width=2);draw.line(tuples(chart_v[:i+1:2]),fill=BLUE,width=2)
            gx,gy=chart_v[i];draw.line((gx,cy,gx,cy+ch),fill=MUTE);draw.ellipse((gx-3,gy-3,gx+3,gy+3),fill=BLUE)
            if pos==poster_at:img.save(poster,quality=89)
            frame=av.VideoFrame.from_ndarray(np.asarray(img),format="rgb24")
            for packet in stream.encode(frame):container.mux(packet)
        for packet in stream.encode():container.mux(packet)
    meta=dict(id=run_id,title=title,file=media.name,poster=poster.name,duration=len(indices)/fps,simTimes=[round(x,2) for x in times],
              status=run["status"],seconds=run["seconds"],progress=run["progress"],seed=run["request"]["seed"],release=release,
              chapters=[dict(title=l,video=round(int(np.argmin(np.abs(np.array(times)-a)))/fps,3),sim=round(a,2)) for l,a,_ in chapters(run)],fps=fps)
    write(OUT/(name+".json"),meta);print(media,len(indices),flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("run");p.add_argument("name");p.add_argument("title");p.add_argument("--highlights",action="store_true")
    a=p.parse_args();compose(a.run,a.name,a.title,a.highlights)
