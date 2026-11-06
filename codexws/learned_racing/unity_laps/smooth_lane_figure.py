"""One recorded training example plus all held-training layout metrics."""
import json
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import font_manager
from smooth_lane_diagnosis import ROOT,AHEAD,load_trial,clean,curvature

def main():
    plt.rcParams.update({"font.family":font_manager.FontProperties(fname="C:/Windows/Fonts/msyh.ttc").get_name(),
                         "axes.spines.right":False,"axes.spines.top":False,"axes.unicode_minus":False})
    result=json.loads((ROOT/"lane_diagnosis.json").read_bytes());held=result["withheldTrainingLayouts"]
    lim=[np.inf if q is None else q for q in result["thresholdsM"]];pick=None;score=-1
    for path in sorted((ROOT/"runs").glob("tr???_a?.json")):
        if int(path.stem[2:5])<=64:continue
        run=load_trial(path);f,h=clean(run,lim)
        kt=curvature(run["truth"]);kr=curvature(run["raw"]);kf=curvature(f)
        gain=abs(kr-kt)-abs(kf-kt);i=int(np.nanargmax(gain))
        if gain[i]>score:pick=(run,f,h,kt,kr,kf,i);score=gain[i]
    run,f,h,kt,kr,kf,i=pick;t=run["time"];lo,hi=max(0,i-6),min(i+7,len(t));window=slice(lo,hi)
    fig=plt.figure(figsize=(12,6.7),layout="constrained",facecolor="white")
    grid=fig.add_gridspec(2,3,width_ratios=[1.45,1.05,1.1],height_ratios=[1,.28])
    a=fig.add_subplot(grid[0,0]);b=fig.add_subplot(grid[0,1]);c=fig.add_subplot(grid[0,2])
    black="#263743";orange="#be520b";blue="#126587";gray="#59717e"
    for name,series,color,ls in [("赛道实际几何",kt,black,"--"),("原始 A4",kr,orange,"-"),("加车身运动补偿",kf,blue,"-")]:
        a.plot(t[window],series[window],label=name,color=color,ls=ls,lw=2.1)
    a.axvline(t[i],color="#9facb3",ls=":");a.legend(frameon=False,fontsize=9);a.set_title("同一条真实轨迹的道路曲率",loc="left",fontsize=13)
    a.set(xlabel="行驶时间（s）",ylabel="前方道路曲率（1/m）");a.grid(axis="y",alpha=.17)
    for arr,color,kind,tag in [(run["truth"],black,"--","真值"),(run["raw"],orange,"-","A4"),(f,blue,"-","补偿后")]:
        b.plot(AHEAD,arr[i],ls=kind,marker="o",ms=4,lw=2,color=color,label=tag)
    b.set_title(f"第 {t[i]:.1f} 秒的四个点",loc="left",fontsize=13);b.set(xlabel="车前距离（m）",ylabel="横向位置（m）",xticks=AHEAD);b.grid(axis="y",alpha=.17)
    cat=["直道被误判为弯","真弯漏判或方向相反"];indices=np.arange(2);width=.32
    for data,label,x,color in [(held["raw"],"原始 A4",-width/2,orange),(held["filtered"],"补偿后",width/2,blue)]:
        vals=[data["straightLookaheadFalseAbove0035"],data["trueBendWrongSignOrBelow0035"]]
        bars=c.bar(indices+x,vals,width,label=label,color=color);c.bar_label(bars,fontsize=10,padding=3)
    c.set_title("16 个留出训练布局的汇总",loc="left",fontsize=13);c.set_xticks(indices,cat,fontsize=9);c.set(ylabel="判断帧数",ylim=(0,135));c.grid(axis="y",alpha=.17);c.legend(frameon=False,fontsize=9)
    fig.suptitle("用实测车速和偏航区分道路弯曲与单帧车线跳变",x=.04,ha="left",fontsize=19,fontweight="bold",color=black)
    foot=fig.add_subplot(grid[1,:]);foot.axis("off")
    foot.text(0,.83,f"左、中图为留出轨迹中单帧曲率纠错幅度最大的样例  ·  {run['name']}。整段按已记录的实际车身运动离线回放。",fontsize=11,color=black)
    foot.text(0,.37,"右图含 112 条轨迹，共 8,912 帧；4,350 帧为直道，3,749 帧为已达到判据的实际弯道。各动作可能共用部分相同进弯观测。",fontsize=10,color=gray)
    fig.savefig(ROOT/"lane_diagnosis.png",dpi=135);plt.close(fig)
if __name__=="__main__":main()
