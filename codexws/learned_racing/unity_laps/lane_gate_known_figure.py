"""Known-case diagnostic figure from actual independent Unity rollouts."""
import json
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import font_manager
from lane_gate_trials import ROOT

def main():
    root=ROOT/"known_b01";p=json.loads((root/"known_case_explanation.json").read_bytes())
    runs=[json.loads((root/"runs"/("b01_"+arm+".json")).read_bytes())for arm in ["raw","gate"]]
    plt.rcParams.update({"font.family":font_manager.FontProperties(fname="C:/Windows/Fonts/msyh.ttc").get_name(),
                         "axes.spines.right":False,"axes.spines.top":False,"axes.unicode_minus":False})
    fig,axes=plt.subplots(2,1,figsize=(10.5,6.6),sharex=True,layout="constrained",gridspec_kw={"height_ratios":[1.3,1]})
    colors=["#c65b20","#147495"];labels=["原始 A4 四点","加车身运动补偿"]
    for i,(r,c,label)in enumerate(zip(runs,colors,labels)):
        v=np.array(r["rows"]);d=r["decisions"];s=np.interp([j["t"]for j in d],v[:,0],v[:,7])
        style="--"if i==0 else "-"
        axes[0].plot(v[:,7],v[:,4],style,color=c,lw=2.5,label=label+" · 实测速")
        axes[1].plot(s,[q["curvature"] for q in d],style,color=c,lw=1.8,label=label)
        if i==1:
            mask=np.array([q["laneGate"]["mask"]!=0 for q in d])
            axes[1].scatter(s[mask],np.array([q["curvature"] for q in d])[mask],marker="o",s=44,
                            facecolor="white",edgecolor=c,zorder=5,label="本帧替换过一个或多个近点")
    for ax in axes:
        ax.axvspan(12,34,color="#ecf2f4",alpha=.8,zorder=-1);ax.axvline(12,color="#9aa8b0",lw=.8);ax.axvline(34,color="#9aa8b0",lw=.8)
        ax.grid(axis="y",alpha=.20);ax.legend(frameon=False,loc="upper left",fontsize=9,ncol=2)
    axes[0].axhline(4,ls=":",lw=1.3,color="#46535c");axes[0].text(35,4.04,"4 m/s",fontsize=9)
    axes[0].set(ylabel="实测车身前进速度（m/s）",ylim=(-.1,5.4))
    a,b=(p[k]["gateMps"]for k in ["newRaw","gated"])
    axes[0].set_title(f"固定 12–34 m 直道区间  ·  原始 {a:.2f} m/s  ·  补偿后 {b:.2f} m/s",loc="left",fontsize=12)
    axes[1].set(ylabel="驾驶控制器收到的前视曲率（1/m）",xlabel="该次起步后的赛道进度（m）")
    fig.suptitle("已知失败直道 B01：两个闭环实跑的实际速度与道路观测",x=.05,ha="left",fontsize=17,fontweight="bold")
    fig.savefig(root/"speed_and_lane.png",dpi=135);plt.close(fig)
if __name__=="__main__":main()
