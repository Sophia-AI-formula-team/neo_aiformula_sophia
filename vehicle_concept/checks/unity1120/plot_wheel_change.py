"""Compare actual camera, body motion and CAN steps for matched policy versions."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from PIL import Image
from plot_turning import LABELS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data",type=Path,required=True)
    parser.add_argument("--roots",type=Path,nargs="+",required=True)
    parser.add_argument("--font",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--cases",nargs="+",choices=LABELS)
    a=parser.parse_args()
    plt.rcParams.update({"font.family":font_manager.FontProperties(fname=str(a.font)).get_name(),
                         "axes.unicode_minus":False,"font.size":9,"axes.edgecolor":"#c9d2d6"})
    groups=json.loads(a.data.read_bytes())
    if a.cases:
        groups=[g for name in a.cases for g in groups if g[-1]["run"].split("-",1)[1]==name]
    if not groups:raise ValueError("No matched tests")
    pair=len(groups[0])==2
    if any(len(g)!=(2 if pair else 3) for g in groups):
        raise ValueError("Expected groups of two or three matched versions")
    newest="协调收油" if pair else "有界"
    height=2.08*len(groups)+1.52
    fig=plt.figure(figsize=(16.5,height),facecolor="white")
    grid=fig.add_gridspec(len(groups),6,width_ratios=[1.15,1.64,1.3,1.3,1.32,1.32],
                          top=1.-1./height,bottom=.48/height,left=.027,right=.99,wspace=.30,hspace=.40)
    styles=[dict(color="#365c7a",lw=2.8),dict(color="#d26439",lw=2.1,ls="--"),
            dict(color="#11806f",lw=1.25,ls="-.",marker=".",markevery=8,ms=2.4)]
    headline="收油时左右轮怎样减速，车身还剩多少路面？" if pair else "相同的赛道，三版怎样收方向？"
    legend=("蓝实线 = 原版 V6.27.6     橙虚线 = 协调收油版" if pair else "蓝实线 = 原版 V6.27.6     橙虚线 = 快速撤回     绿点划线 = 有界撤回")
    fig.suptitle(headline,x=.024,y=1.-.03/height,ha="left",fontsize=20,fontweight="bold")
    fig.text(.026,1.-.60/height,legend+"。   轮差为右轮 CAN 减左轮 CAN；减小仍为正的轮差表示同向收力。",fontsize=11)
    for row,group in enumerate(groups):
        latest=group[-1];key=latest["run"].split("-",1)[1]
        folders=[]
        for run in group:
            paths=[r/run["run"] for r in a.roots if (r/run["run"]/"result.json").is_file()]
            if len(paths)!=1:raise ValueError("Missing or ambiguous path for "+run["run"])
            folders.append(paths[0])
        desc=fig.add_subplot(grid[row,0]);desc.axis("off")
        desc.text(0,.83,LABELS[key].replace(" · 延迟","\n延迟 180 ms"),fontsize=12,fontweight="bold")
        desc.text(0,.49,"最小轮距（m）\n"+" / ".join(f"{s['painted_wheel_center_m']['minimum']:.2f}" for s in group),fontsize=9)
        desc.text(0,.20,"最大轮差跳变（rpm）\n"+" / ".join(str(abs(s["largest_delta_right_minus_left_rpm"]["signed_rpm"])) for s in group),fontsize=9)
        desc.text(0,.01,newest+"版末停 "+f"{latest['terminal_physical_below_005_s']:.2f} s",fontsize=8,color="#654425")
        ev=[json.loads(l) for l in (folders[-1]/"exchange.jsonl").read_bytes().splitlines()]
        worst=group[0]["largest_delta_right_minus_left_rpm"]
        i=int(np.argmin(abs(np.array(latest["timeseries"]["s"])-worst["capture_s"]))) if abs(worst["signed_rpm"])>=100 else len(ev)-1
        ax=fig.add_subplot(grid[row,1])
        with Image.open(folders[-1]/ev[i]["camera_file"]) as image:ax.imshow(image)
        ax.set_axis_off()
        time_label=f"{ev[i]['frame']['simulation_time_s']:.1f} s"
        if pair and abs(worst["signed_rpm"])>=100:time_label+=" · 原版轮差跳变时刻"
        ax.text(.02,.035,time_label,transform=ax.transAxes,color="white",
                bbox=dict(fc="#14262c",ec="none",alpha=.86,pad=2))
        if row==0:ax.set_title(newest+"版的车载相机原图")
        charts=[fig.add_subplot(grid[row,k]) for k in (2,3,4,5)]
        for item,folder,style in zip(group,folders,styles):
            w=np.array(item["wheel_time_series"]);raw=np.array(json.loads((folder/"result.json").read_bytes())["rows"])
            curves=item["timeseries"];t=curves["s"]
            charts[0].plot(w[:,0],np.min(w[:,1:],axis=1),**style)
            charts[1].plot(raw[:,0],raw[:,4],**style)
            charts[2].plot(raw[:,0],-raw[:,14],**style)
            charts[3].step([*t,item["seconds"]],[*curves["diff_can_rpm"],curves["diff_can_rpm"][-1]],where="post",**style)
        for chart in charts:
            sec=latest["seconds"];chart.set_xlim(0,sec);chart.set_xticks([0,sec/2,sec])
            if pair and abs(worst["signed_rpm"])>=100:chart.axvline(worst["capture_s"],lw=.8,color="#6b7378",alpha=.6,ls=":")
            chart.tick_params(length=2,labelsize=8);chart.grid(axis="y",alpha=.23)
            chart.spines[["top","right"]].set_visible(False)
            if row==len(groups)-1: chart.set_xlabel("接管后模拟时间（s）",fontsize=8)
        charts[0].set_ylim(bottom=0);charts[1].set_ylim(-.1,2.6)
        if row==0:
            for chart,title in zip(charts,["前轮中心距线外缘（m）","实测车速（m/s）","实测角速度（rad/s，左正）","发送的左右轮差（rpm）"]):
                chart.set_title(title,fontsize=10)
    fig.text(.026,.12/height,"相同场次的第一张图逐字节一致。轮距取两只前胎中较近一只，包含白线宽度；停稳按实际车速低于 0.05 m/s。",fontsize=9)
    fig.savefig(a.out,dpi=143,facecolor="white");plt.close(fig)


if __name__=="__main__":
    main()
