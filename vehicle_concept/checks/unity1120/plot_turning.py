"""Plot the paired raw camera frames and measured traces in final-scores.json."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from PIL import Image


LABELS = {
    "straight": "沿外车道切线起步", "outer": "外缘 · 朝外 12°", "inner": "内缘 · 朝内 12°",
    "hard": "外缘 · 朝外 23°", "bend": "先直行、再入弯", "deepening": "转入后弯度加深",
    "bend-delay180": "先直行、再入弯 · 延迟", "hard-delay180": "外缘 23° · 延迟",
    "rolling-edge": "行驶中靠向外缘",
}
COLORS = ["#176181", "#c35126"]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scores",type=Path,required=True)
    parser.add_argument("--roots",type=Path,nargs="+",required=True)
    parser.add_argument("--font",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--cases",nargs="+",choices=LABELS)
    args=parser.parse_args()
    plt.rcParams.update({"font.family":font_manager.FontProperties(fname=str(args.font)).get_name(),
                         "axes.unicode_minus":False, "font.size":9, "axes.edgecolor":"#c9d2d6",
                         "text.color":"#172c34","axes.labelcolor":"#344c55"})
    pairs=json.loads(args.scores.read_bytes())
    if args.cases:
        pairs=[pair for name in args.cases for pair in pairs if pair[1]["run"].split("-",1)[1]==name]
    if not pairs:
        raise ValueError("No requested pairs")
    height=2.15*len(pairs)+1.47
    fig=plt.figure(figsize=(16,height),facecolor="white")
    grid=fig.add_gridspec(len(pairs),6,width_ratios=[.94,1.5,1.5,1.38,1.38,1.38],left=.02,right=.99,
                          top=1.-.93/height,bottom=.46/height,wspace=.26,hspace=.42)
    fig.suptitle("同一起点，车辆实际转了多少、离线还有多远？",x=.02,y=1.-.045/height,
                 ha="left",fontsize=20,fontweight="bold")
    fig.text(.022,1.-.56/height,"原图来自 Unity 车载左相机。蓝色为旧版，橙色为新版。方向图中粗线是发出的指令，细点线是车身实际角速度。两版本曲线重合表示测量相同。",fontsize=11)
    for row, pair in enumerate(pairs):
        code=pair[1]["run"].split("-",1)[1]
        folders=[]
        for item in pair:
            matches=[r/item["run"] for r in args.roots if (r/item["run"]/"result.json").is_file()]
            if len(matches)!=1:raise ValueError("Missing or ambiguous run "+item["run"])
            folders.append(matches[0])
        text=fig.add_subplot(grid[row,0]);text.axis("off")
        a,b=pair
        text.text(0,.8,LABELS[code].replace(" · 延迟","\n延迟 180 ms"),fontsize=12,fontweight="bold",va="center")
        text.text(0,.50,f"进度（旧 / 新）\n{a['progress_m']:.2f} / {b['progress_m']:.2f} m",va="center")
        text.text(0,.15,f"新版末段停稳\n{b['terminal_physical_below_005_s']:.2f} s",va="center",color="#814120")
        for i,folder in enumerate(folders):
            event=json.loads((folder/"exchange.jsonl").read_bytes().splitlines()[-1])
            ax=fig.add_subplot(grid[row,i+1])
            with Image.open(folder/event["camera_file"]) as image: ax.imshow(image)
            ax.set_axis_off()
            ax.text(.025,.04,f"{event['frame']['simulation_time_s']:.1f} s",transform=ax.transAxes,
                    color="white",bbox=dict(boxstyle="round,pad=.2",fc="#16252b",ec="none",alpha=.84))
            if row==0: ax.set_title(("旧版最后一张前视图","新版最后一张前视图")[i],fontsize=11)
        axes=[fig.add_subplot(grid[row,k]) for k in (3,4,5)]
        for i,(item,folder) in enumerate(zip(pair,folders)):
            style=dict(color=COLORS[i],linestyle=("-","--")[i],linewidth=(2,1.5)[i])
            wheels=np.array(item["wheel_time_series"]); body=np.array(json.loads((folder/"result.json").read_bytes())["rows"])
            time=item["timeseries"]["s"]; yaw=item["timeseries"]["cmd_ros_w"]
            axes[0].plot(wheels[:,0],wheels[:,1:].min(axis=1),**style)
            axes[1].plot(body[:,0],body[:,4],**style)
            axes[2].step([*time,item["seconds"]],[*yaw,yaw[-1]],where="post",**style)
            axes[2].plot(body[:,0],-body[:,14],color=COLORS[i],linestyle=":",linewidth=1.1)
        for ax in axes:
            ax.set_xlim(0,b["seconds"]);ax.set_xticks([0,b["seconds"]/2,b["seconds"]])
            ax.tick_params(labelsize=8,colors="#3c555e",length=2)
            ax.grid(axis="y",alpha=.24);ax.spines[["top","right"]].set_visible(False)
        axes[0].set_ylim(0,max(1.05,1.1*max(np.max(np.min(np.array(item["wheel_time_series"])[:,1:],axis=1)) for item in pair)))
        axes[1].set_ylim(-.1,2.6);axes[2].set_ylim(-.11,.52)
        if row==0:
            for ax, cap in zip(axes,("最近前轮中心到线外缘（m）","车身实际前进速度（m/s）","指令与实测转速（rad/s，左为正）")):
                ax.set_title(cap,fontsize=10)
        if row==len(pairs)-1:
            for ax in axes: ax.set_xlabel("接管后模拟时间（s）",fontsize=8)
    fig.text(.022,.09/height,"轮距曲线按绘制白线的外缘计算；一侧胎面完全越线才判负。停稳时间按车身实际速度低于 0.05 m/s 计算。",fontsize=9)
    fig.savefig(args.out,dpi=143,facecolor="white");plt.close(fig)


if __name__=="__main__":
    main()
