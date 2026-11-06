"""Plot all three observed outcomes of each registered opening scenario."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager, pyplot as plt
from matplotlib.patches import FancyBboxPatch

root=Path(__file__).parent
report=json.loads((root/"gap_holdout/results.json").read_bytes())
font=Path("C:/Windows/Fonts/msyh.ttc")
if font.exists():
    font_manager.fontManager.addfont(str(font))
    plt.rcParams["font.family"]=font_manager.FontProperties(fname=str(font)).get_name()
bg,ink="#f5f7f3","#173d3a"
fig,ax=plt.subplots(figsize=(12,7.6),facecolor=bg)
fig.subplots_adjust(left=.045,right=.97,bottom=.10,top=.76)
ax.set_xlim(0,4);ax.set_ylim(0,8);ax.axis("off")
fig.text(.045,.94,"双侧与连续障碍：三组真实闭环",fontsize=23,weight="bold",color=ink)
fig.text(.045,.88,"每格是一个完整场景。时间为整段用时；碰撞记录到终止，不和通过时间比较。",
         fontsize=11,color="#4d655f")
for x,title in enumerate(("原控制","只提速","提速 + 寻缝"),start=1):
    ax.text(x+.47,8.16,title,ha="center",va="bottom",fontsize=12,weight="bold",color=ink)
labels={"static":"静态双桶","moving":"移动双桶","double":"两排障碍","mixed-triple":"三排动静混合"}
for i,case in enumerate(report["pairs"]):
    y=7-i
    family=labels.get(case["family"],case["family"])
    road="弯道" if case["road"]=="bend" else "直道"
    ax.text(.03,y+.51,case["id"].upper(),fontsize=13,weight="bold",color=ink)
    ax.text(.03,y+.24,f"{road} · {family}",fontsize=9,color="#506c62")
    for x,arm in enumerate(("base","pace","learned"),start=1):
        rec=case[arm];ok=rec["physicalSuccess"]
        color="#d9eee5" if ok else "#f7ddd4"
        ax.add_patch(FancyBboxPatch((x+.025,y+.08),.93,.80,
                     boxstyle="round,pad=.01,rounding_size=.05",linewidth=0,facecolor=color))
        label=f"通过  {rec['elapsedS']:.2f} s" if ok else "碰撞终止"
        ax.text(x+.49,y+.58,label,ha="center",va="center",fontsize=11,weight="bold",color=ink if ok else "#8c3826")
        secondary=(f"最小净距 {rec['minConeM']:.2f} m" if ok else
                   f"已行驶 {rec['progressM']:.1f} m")
        ax.text(x+.49,y+.32,secondary,ha="center",va="center",fontsize=9,color="#52685f")
fig.text(.045,.035,"实验使用相同 A4、车辆物理与障碍布局。通过不代表整条赛道或实车已经验证。",fontsize=10,color="#506c62")
fig.savefig(root/"long_training_evidence/gap_holdout.png",dpi=150,facecolor=bg)
