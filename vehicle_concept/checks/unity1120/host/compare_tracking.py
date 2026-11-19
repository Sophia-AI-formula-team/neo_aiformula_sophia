"""Match actual ROS decisions to Unity images, plot used tracking points and score both drives."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
sys.path.insert(0,str(HERE.parents[2]/"src/concept_yolop"))
from score_turning import summarize
from concept_yolop.geometry import FX,FY,CX,CY,HEIGHT,load_projection

NAMES={"pure_pursuit":"Pure Pursuit","lya":"Lyapunov"}
COLORS={"pure_pursuit":"#007991","lya":"#ba413d"}
POINT_COLORS=["#ffdc52","#fd9071","#a9f5fc"]


def read(path):
    return json.loads(path.read_bytes())


def lines(path):
    return [json.loads(line) for line in path.read_bytes().splitlines() if line]


def same_file(left,right):
    return left.read_bytes()==right.read_bytes()


def active(d):
    return bool(d.get("active")) and d.get("owner")=="candidate"


def projected(zx, inverse):
    """Map metric ground points into the saved raw rectified-left JPEG."""
    zx=np.asarray(zx,dtype=float).reshape(-1,2)
    if not len(zx):return np.empty((0,2))
    z,x=zx.T
    canonical=np.column_stack((FX*x+CX*z,np.full_like(z,FY*HEIGHT)+CY*z,z))
    source=canonical@inverse.T
    uv=np.full((len(z),2),np.nan)
    visible=source[:,2]>.01
    uv[visible]=source[visible,:2]/source[visible,2:3]
    return uv


def read_run(folder, ros_roots, mode):
    host=lines(folder/"exchange.jsonl")
    matches=[]
    for root in ros_roots:
        for file in root.glob("*/frames.jsonl"):
            with file.open("rb") as stream:first=json.loads(stream.readline())
            if first["reply"]==host[0]["command"]:
                matched=lines(file)
                if len(matched)==len(host) and all(x["reply"]==y["command"] for x,y in zip(matched,host)):
                    matches.append((file,matched))
    if len(matches)!=1:raise ValueError("Cannot uniquely pair ROS session with "+str(folder))
    session,rows=matches[0]
    cfg=read(session.with_name("session_config.json"))
    if cfg["tracking_controller"]!=mode:raise ValueError("Unexpected launched tracking_controller")
    for seq,(x,y) in enumerate(zip(rows,host)):
        if (x["input"]!=y["frame"] or y["command"]["seq"]!=seq or
            y["command"]["tracking_controller"]!=mode):
            raise ValueError("Image, command or mode mismatch")
        d=x["decision"]
        if d["status"]=="ok" and d["tracking_controller"]["mode"]!=mode:
            raise ValueError("Driver executed a different controller")
    return dict(mode=mode,folder=folder,host=host,ros=rows,session=session)


def timing(run):
    fields={}
    for h,r in zip(run["host"],run["ros"]):
        d=r["decision"]
        cameras=d.get("camera_timing_ms",{})
        measures={
            "unity_capture_to_CAN_ms":(h["can_set_utc_us"]-h["capture_start_utc_us"])/1000,
            "unity_encoded_to_reply_ms":(h["reply_received_utc_us"]-h["capture_encoded_utc_us"])/1000,
            "bridge_processing_ms":r["reply"]["wall_ms"],
            "ros_driver_processing_ms":d.get("processing_ms"),
        }
        for key in ("decode_and_projection","inference","restrict_predictions"):
            measures[key+"_ms"]=cameras.get(key)
        for key,value in measures.items():
            if value is not None:
                fields.setdefault(key,[]).append(float(value))
    result={}
    for key,values in fields.items():
        steady=values[1:]
        result[key]=dict(first=values[0],steady_count=len(steady),
                         p50=float(np.percentile(steady,50)) if steady else None,
                         p95=float(np.percentile(steady,95)) if steady else None)
    return result


def annotate(ax,x,y,label,color,photo):
    if not np.isfinite(x+y):return
    if photo and not (0<=x<640 and 0<=y<360):return
    ax.scatter([x],[y],s=75,color=color,edgecolors="#172a31",linewidths=1.4,zorder=5)
    ax.annotate(label,(x,y),xytext=(5,5),textcoords="offset points",weight="bold",color="#15272f",fontsize=9,
                bbox=dict(boxstyle="round,pad=.12",facecolor=color,edgecolor="none",alpha=.92),zorder=7)


def tracking_figure(runs, seq, calibration, out, identical, font):
    plt.rcParams.update({"font.family":FontProperties(fname=str(font)).get_name(),"font.size":11,"axes.unicode_minus":False})
    inverse=np.linalg.inv(calibration.warp)
    fig=plt.figure(figsize=(14.3,9.5),facecolor="white")
    grid=fig.add_gridspec(2,3,width_ratios=[2.05,1.15,1.14],left=.034,right=.98,top=.865,bottom=.080,hspace=.31,wspace=.23)
    fig.suptitle("同一路段，两种控制器用哪些点转向？",x=.033,y=.97,ha="left",fontsize=23,fontweight="bold")
    fig.text(.034,.912,("两车此时收到的相机图像完全相同。" if identical else "下图取两场相同模拟时刻，位置已经分开。")+
             "  青线为本帧选中的候选轨迹，圆点来自实际执行的控制器记录。",fontsize=12)
    for row,run in enumerate(runs):
        host=run["host"][seq];d=run["ros"][seq]["decision"];ctrl=d.get("tracking_controller") or {}
        mode=run["mode"]; folder=run["folder"]; shot=host["camera_file"]
        img=cv2.imread(str(folder/shot))
        if img is None:raise FileNotFoundError(folder/shot)
        label=NAMES[mode]; dest=out/mode;dest.mkdir()
        shutil.copy2(folder/shot,dest/"raw.jpg")
        if not cv2.imwrite(str(dest/"canonical.png"),calibration.rectify(img)):raise IOError(dest)
        stamp=d["stamp"]
        overlay=run["session"].parents[1]/"overlays"/f"{stamp['sec']}-{stamp['nanosec']:09d}.png"
        if overlay.exists():
            shutil.copy2(overlay,dest/"perception-overlay.png")
        (dest/"decision.json").write_text(json.dumps(d,ensure_ascii=False),encoding="utf8")
        (dest/"command.json").write_text(json.dumps(host,ensure_ascii=False),encoding="utf8")
        ax=fig.add_subplot(grid[row,0]);top=fig.add_subplot(grid[row,1]);notes=fig.add_subplot(grid[row,2]);notes.axis("off")
        ax.imshow(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))
        ax.set_xlim(-.5,639.5);ax.set_ylim(359.5,-.5);ax.axis("off");ax.set_title(f"{label}　｜　原始左相机　{host['frame']['simulation_time_s']:.1f} s")
        top.set_title("从车上向前看（m）");top.axvline(0,color="#a5b8c1",ls=":",lw=1);top.grid(alpha=.2)
        top.plot(0,0,"^",color="#1e3440",ms=9);top.set_xlabel("向右为正（m）");top.set_ylabel("向前距离（m）")
        path=np.asarray(d["selected_path"],dtype=float)
        if len(path):
            uv=projected(path[:,:2],inverse);ax.plot(uv[:,0],uv[:,1],color="#47e5df",lw=2.4)
            top.plot(path[:,1],path[:,0],color="#159d9b",lw=2.4)
        points=np.asarray(ctrl["points_zx_m"] if active(ctrl) else [],dtype=float).reshape(-1,2)
        raw_uv=projected(points,inverse)
        if len(points)==3:
            top.plot(points[:2,1],points[:2,0],c="#8f6030",ls="--",lw=2)
            ax.plot(raw_uv[:2,0],raw_uv[:2,1],c="#ffdc52",ls="--",lw=1.6)
        tags=["P"] if mode=="pure_pursuit" else ["A","B","C"]
        desc=[]
        for i,(point,uv) in enumerate(zip(points,raw_uv)):
            z,x=point;color=POINT_COLORS[i];tag=tags[i]
            annotate(ax,*uv,tag,color,True);annotate(top,x,z,tag,color,False)
            desc.append(f"{tag}　{z:.2f} m 前，{abs(x):.2f} m "+("右" if x>=0 else "左"))
        xmax=max([.5]+[abs(p[1]) for p in points]+([float(np.max(abs(path[:,1])))] if len(path) else []))
        zmax=max([2.]+[float(p[0]) for p in points]+([float(path[:,0].max())] if len(path) else []))
        top.set_xlim(-1.3*xmax,1.3*xmax);top.set_ylim(-.2,zmax*1.12)
        v=host["command"]["linear_mps"];w=host["command"]["angular_z_radps"];gyro=host["frame"]["gyro_ros_radps"]
        owner=ctrl.get("owner","unavailable")
        notes.text(0,.95,f"{label}　／　轨迹 {d['selected_index']}",weight="bold",fontsize=15,color=COLORS[mode],va="top")
        words=("追向一个前方点 P。" if mode=="pure_pursuit" else "A 是目标，A 到 B 给参考方向。\n再比较 AB 与 BC 的方向变化。")
        notes.text(0,.80,words if active(ctrl) else "本帧由 "+owner+" 分支决定。",fontsize=11,va="top")
        notes.text(0,.59,"\n".join(desc),fontsize=10,va="top")
        requested=ctrl.get("requested_yaw_right_radps")
        desired="无" if requested is None else f"{-float(requested):+.3f} rad/s"
        lines_out=[f"前进指令　{v:.2f} m/s",f"控制律要求　{desired}",f"实际发送转向　{w:+.3f} rad/s",f"车身实测转动　{gyro:+.3f} rad/s",
                   f"左右 CAN　{host['set_left_can']} / {host['set_right_can']} rpm"]
        notes.text(0,.28,"\n".join(lines_out),va="top",fontsize=10,linespacing=1.38)
    fig.text(.034,.020,"左转角速度为正。车前近处可能落在相机画面之外，这些真实控制点仍在车身坐标图上标出。实际转动取拍照时的 gyro。",fontsize=10)
    fig.savefig(out/"tracking-points.png",dpi=145,facecolor="white");plt.close(fig)


def run_figure(runs, out, font):
    plt.rcParams.update({"font.family":FontProperties(fname=str(font)).get_name(),"axes.unicode_minus":False})
    fig,axes=plt.subplots(2,2,figsize=(12.3,7.3))
    for run in runs:
        mode=run["mode"];host=run["host"];r=read(run["folder"]/"result.json");physical=np.asarray(r["rows"])
        sample=np.array([x["frame"]["simulation_time_s"] for x in host]);end=r["seconds"]
        cmd=np.array([x["command"]["angular_z_radps"] for x in host])
        kwargs=dict(label=NAMES[mode],color=COLORS[mode],ls="-" if mode=="pure_pursuit" else "--",lw=2.15)
        axes[0,0].plot(physical[:,0],physical[:,7],**kwargs);axes[0,1].plot(physical[:,0],physical[:,4],**kwargs)
        axes[1,0].step(np.r_[sample,end],np.r_[cmd,cmd[-1]],where="post",**kwargs)
        axes[1,1].plot(physical[:,0],-physical[:,14],**kwargs)
    for ax,title in zip(axes.ravel(),["沿赛道进度（m）","车身实际前进速度（m/s）","转向指令（rad/s，左为正）","车身实测角速度（rad/s，左为正）"]):
        ax.set_title(title);ax.set_xlabel("模拟时间（s）");ax.grid(alpha=.23);ax.spines[["top","right"]].set_visible(False)
    axes[0,0].legend(frameon=False)
    fig.suptitle("两辆虚拟车从同一起点出发，实际怎样走？",fontsize=21)
    fig.tight_layout(rect=(.01,.015,.99,.94));fig.savefig(out/"closed-loop.png",dpi=145);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pure-pursuit",type=Path,required=True);parser.add_argument("--lya",type=Path,required=True)
    parser.add_argument("--ros",type=Path,action="append",required=True);parser.add_argument("--rings",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True);parser.add_argument("--font",type=Path,required=True)
    parser.add_argument("--exit-progress",type=float,help="Optional known scoring-only exit distance from this start (m)")
    a=parser.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    runs=[read_run(path,a.ros,mode) for path,mode in [(a.pure_pursuit,"pure_pursuit"),(a.lya,"lya")]]
    calibration=load_projection(HERE.parent/"foxy/honda.json")
    rings=read(a.rings)
    evidence=[]
    for run in runs:
        score=summarize(run["folder"],rings)
        counts=Counter((s["decision"].get("tracking_controller") or {}).get("owner","unavailable") for s in run["ros"])
        physical=np.asarray(read(run["folder"]/"result.json")["rows"])
        exit_time=None
        crossings=[] if a.exit_progress is None else np.flatnonzero(physical[:,7]>=a.exit_progress)
        if len(crossings):
            i=int(crossings[0])
            exit_time=float(np.interp(a.exit_progress,physical[max(0,i-1):i+1,7],physical[max(0,i-1):i+1,0]))
        evidence.append(dict(mode=run["mode"],session=run["session"].parent.name,score=score,source_frame_counts=dict(counts),
                             tracking_active_frames=sum(active(d["decision"].get("tracking_controller") or {}) for d in run["ros"]),
                             exit_progress_m=a.exit_progress,exit_sim_s=exit_time,timing=timing(run)))
        with (a.out/(run["mode"]+"-frames.csv")).open("w",encoding="utf8",newline="") as file:
            writer=csv.writer(file)
            writer.writerow(["seq","sim_s","status","owner","selected","feasible","actual_points_zx_m","target_zx_m",
                             "requested_w_right","checked_w_right","emitted_w_ros","gyro_ros","command_v","body_v",
                             "can_right","can_left","raw_jpeg_sha256"])
            for r,s in zip(run["ros"],run["host"]):
                d=r["decision"];c=d.get("tracking_controller") or {};h=s["frame"];cmd=s["command"]
                writer.writerow([h["seq"],h["simulation_time_s"],d["status"],c.get("owner"),d["selected_index"],
                                 d["feasible_count"],json.dumps(c.get("points_zx_m")),json.dumps(c.get("target_zx_m")),
                                 c.get("requested_yaw_right_radps"),c.get("checked_yaw_right_radps"),cmd["angular_z_radps"],
                                 h["gyro_ros_radps"],cmd["linear_mps"],s["body_forward_mps"],s["set_right_can"],s["set_left_can"],
                                 hashlib.sha256((run["folder"]/s["camera_file"]).read_bytes()).hexdigest()])
    if evidence[0]["score"]["first_jpeg_sha256"]!=evidence[1]["score"]["first_jpeg_sha256"]:
        raise ValueError("Initial images differ; cannot label starts as matched")
    choices=[]
    for seq,(x,y) in enumerate(zip(runs[0]["ros"],runs[1]["ros"])):
        c=[r["decision"].get("tracking_controller") or {} for r in (x,y)]
        if not all(active(s) for s in c):continue
        if [len(s["points_zx_m"]) for s in c]!=[1,3]:raise ValueError("Unexpected used tracking points")
        hosts=[run["host"][seq] for run in runs]
        img=[run["folder"]/h["camera_file"] for run,h in zip(runs,hosts)]
        identical=same_file(*img)
        delta=abs(x["reply"]["angular_z_radps"]-y["reply"]["angular_z_radps"])
        # Favor a shared observation, then frames with the largest applied difference.
        choices.append((identical,delta,seq))
    if not choices:raise ValueError("No common instant with both tracking controllers active")
    identical,delta,seq=max(choices)
    tracking_figure(runs,seq,calibration,a.out,identical,a.font)
    moving=[(diff,k,same) for same,diff,k in choices if k!=seq and all(run["host"][k]["body_forward_mps"]>.6 for run in runs)]
    second=None
    if moving:
        _,second,same=max(moving);dst=a.out/"moving";dst.mkdir()
        tracking_figure(runs,second,calibration,dst,same,a.font)
    run_figure(runs,a.out,a.font)
    summary=dict(runs=evidence,frame_used_for_figure=seq,two_raw_images_identical=identical,angular_difference_radps=delta,
                 secondary_moving_frame=second)
    (a.out/"results.json").write_text(json.dumps(summary,allow_nan=False),encoding="utf8")
    print([(x["mode"],x["score"]["progress_m"],x["source_frame_counts"]) for x in evidence],seq,identical)


if __name__=="__main__":
    main()
