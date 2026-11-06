"""Side-by-side causal comparison of actual Unity passes at the moving cone."""
import json
import av
import numpy as np
from PIL import Image,ImageDraw
from render import HERE,OUT,RAW,WORK,BG,INK,MUTE,LIME,BLUE,ORANGE,font,label,obstacle_plan,track,write

def main():
    cases=json.loads((HERE/"evidence/ablation.json").read_bytes())["trials"]
    runs=[json.loads((RAW/(cases[k]+".json")).read_bytes()) for k in ("normal","zeroForecast")]
    geom=track();traces=[np.array(r["rows"]) for r in runs];source_frames=[np.array(r["frames"]) for r in runs]
    starts=[float(np.interp(r["props"][3][7]-11,trace[:,7],trace[:,0])) for r,trace in zip(runs,traces)]
    ends=[min(r["seconds"],float(np.interp(r["props"][3][7]+7,trace[:,7],trace[:,0]))) for r,trace in zip(runs,traces)]
    hz=12;duration=max(b-a for a,b in zip(starts,ends))+1.1;times=[];mp4=OUT/"motion-ablation.mp4"
    base=Image.new("RGB",(1280,720),BG);g=ImageDraw.Draw(base)
    label(g,25,18,"AI FORMULA   /   CONTROL EXPERIMENT",17,LIME,True,True);label(g,24,47,"同一个横移路障，两次驾驶选择",30,INK,True)
    label(g,25,100,"保留路障运动外推",22,LIME,True);label(g,652,100,"将外推系数设为零",22,ORANGE,True)
    label(g,25,683,"按车头到达路障前 11 m 对齐。两次物理独立推进，右侧在首次接触时定格。",16,MUTE)
    with av.open(str(mp4),"w",options={"movflags":"+faststart"}) as box:
        stream=box.add_stream("libx264",rate=hz);stream.width=1280;stream.height=720;stream.pix_fmt="yuv420p";stream.options={"crf":"20","preset":"veryfast"}
        for k in range(int(np.ceil(duration*hz))):
            scene=base.copy();draw=ImageDraw.Draw(scene)
            for j,(r,rows,fr,begin,end) in enumerate(zip(runs,traces,source_frames,starts,ends)):
                t=min(begin+k/hz,end);x=25+j*627;i=max(0,np.searchsorted(rows[:,0],t,side="right")-1)
                ix=max(0,np.searchsorted(fr[:,0],t,side="right")-1);frame=int(fr[ix,1]);rw=rows[i]
                with Image.open(WORK/"frames"/r["id"]/f"{frame:04d}c.jpg") as im:scene.paste(im.resize((602,339),Image.Resampling.BILINEAR),(x,142))
                if t==end and r["status"]=="obstacle":label(draw,x+15,155,"接触 · 定格",20,ORANGE,True)
                sample=min((int(t/.2)),r["seenFrames"]-1);overlay=WORK/"camera"/r["id"]/"a4_samples"/f"{sample:05d}.jpg"
                with Image.open(overlay) as im:scene.paste(im.resize((270,152),Image.Resampling.BILINEAR),(x,501))
                plan=obstacle_plan(geom,r,rw,rows[max(0,i-50):i+1],3)
                scene.paste(plan.resize((170,103),Image.Resampling.BILINEAR),(x+277,537))
                label(draw,x+278,505,"实际 A4 / 裁判俯视",14,MUTE)
                label(draw,x+451,506,"仿真时间",13,MUTE);label(draw,x+451,531,f"{t:.2f} s",20,INK,True,True)
                label(draw,x+451,568,"路障净距",13,MUTE);label(draw,x+451,594,f"{rw[10]:.2f} m",22,LIME if j==0 else ORANGE,True,True)
                if j==0:times.append(round(t,2))
            if k==int((ends[1]-starts[1]) * hz):scene.save(OUT/"motion-ablation.jpg",quality=88)
            for packet in stream.encode(av.VideoFrame.from_ndarray(np.asarray(scene),format="rgb24")):box.mux(packet)
        for packet in stream.encode():box.mux(packet)
    r=runs[0];write(OUT/"motion-ablation.json",dict(id=r["id"],title="运动外推对照 · 通过与碰桶",file=mp4.name,poster="motion-ablation.jpg",duration=len(times)/hz,
           simTimes=times,status=r["status"],seconds=r["seconds"],progress=r["progress"],seed=r["request"]["seed"],release="a4",chapters=[],fps=hz,comparison=cases))
if __name__=="__main__":main()
