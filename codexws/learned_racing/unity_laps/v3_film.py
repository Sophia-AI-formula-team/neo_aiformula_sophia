"""Three panels of actual Unity camera/control evidence, including failures."""
import json
from pathlib import Path
import av
import numpy as np
from PIL import Image,ImageDraw
from render import font
from run import HERE

def make(name):
    root=HERE/"v3_unity_evidence";run=json.loads((root/"runs"/(name+".json")).read_bytes())
    frames=root/".work/frames"/name;perception=root/".work"/name/"frames"
    dst=root/(name+".mp4");rows=np.array(run["rows"]);times=np.array([d["t"] for d in run["decisions"]])
    captures=np.array(run["frames"]);last=None
    with av.open(str(dst),"w") as output:
        stream=output.add_stream("libx264",rate=10);stream.width=1440;stream.height=640;stream.pix_fmt="yuv420p";stream.options={"crf":"22"}
        for t in np.arange(0,run["seconds"]+1,.1):
            k=max(0,int(np.searchsorted(captures[:,0],t,side="right")-1));frame=int(captures[k,1])
            j=max(0,int(np.searchsorted(times,t,side="right")-1));d=run["decisions"][j];obs=d["decision"]
            row=rows[max(0,int(np.searchsorted(rows[:,0],t,side="right")-1))]
            canvas=Image.new("RGB",(1440,640),"#111827");draw=ImageDraw.Draw(canvas)
            draw.text((20,15),name+" · Unity A4 实际闭环",font=font(25),fill="white")
            for x,label in ((15,"跟车摄像机"),(495,"A4 / RGB 当帧观测"),(975,"车体坐标：观测范围与规划")):
                draw.text((x,57),label,font=font(18),fill="#b6c2d2")
            for x,path in ((0,frames/f"{frame:04d}c.jpg"),(480,perception/f"{j:05d}.jpg")):
                if path.exists():canvas.paste(Image.open(path).convert("RGB").resize((480,270)),(x,90))
            def q(z,x):return (1200+x*32,390-z*26)
            road=obs["road"]
            if len(road)>1:draw.polygon([q(r[0],r[1]) for r in road]+[q(r[0],r[2]) for r in reversed(road)],fill="#334155")
            for z,x,vz,vx,r,noise in obs["objects"]:
                px,py=q(z,x);draw.ellipse((px-10,py-10,px+10,py+10),fill="#fbbf24")
                draw.line([q(z,x),q(z+vz,x+vx)],fill="#fb7185",width=2)
            if obs.get("path"):draw.line([q(*p) for p in obs["path"]],fill="#2dd4bf",width=3)
            draw.rectangle((1188,382,1212,416),fill="#60a5fa")
            text=[f"时间 {min(t,run['seconds']):.1f} s   车身实测速 {row[4]:.2f} m/s   目标轮速 {d['speedCmd']:.2f} m/s",
                  f"轮速代理估计 {obs['speedEstimate']:.2f} m/s   转向指令 {d['yawCmd']:.2f}   决策 {obs['reason']}",
                  f"结果 {run['status']}   实际进度 {run['progress']:.1f} / {run['length']:.1f} m   完整保留失败终点",
                  "右图仅来自当前图像和轮速/gyro；没有全局位置、完整赛道或外圈车道补点。",
                  "当前道路提取：A4 白线 + RGB 路面颜色；轮速是仿真电机模型代理，仍非独立编码器。"]
            for n,line in enumerate(text):draw.text((20,430+n*36),line,font=font(20 if n<3 else 17),fill="#e2e8f0")
            video=av.VideoFrame.from_image(canvas)
            for packet in stream.encode(video):output.mux(packet)
            last=canvas
        for packet in stream.encode():output.mux(packet)
    if last:last.save(root/(name+".png"))
    return str(dst)

if __name__=="__main__":
    import sys
    for name in sys.argv[1:]:print(make(name))
