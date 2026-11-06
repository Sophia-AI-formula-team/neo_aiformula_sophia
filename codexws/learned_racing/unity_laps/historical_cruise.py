"""Retrospective, descriptive cruise audit on fixed previous Unity trials."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib import font_manager
import numpy as np
from scipy.spatial import cKDTree

from scenario_shots import geometry


HERE = Path(__file__).parent
RECORDS = [
    "risk_revision/runs/frozen_20318_clear.json",
    "risk_revision/runs/frozen_93677_clear.json",
    "learned_revision/runs/h01_base.json",
    "learned_revision/runs/full01_base.json",
]


def longest(mask, elapsed, distance):
    starts = np.flatnonzero(mask & ~np.r_[False,mask[:-1]])
    ends = np.flatnonzero(mask & ~np.r_[mask[1:],False])+1
    return max([(elapsed[a:b].sum(),distance[a:b].sum()) for a,b in zip(starts,ends)],
               default=(0.,0.))


def measure(run, tree, curvature):
    r = np.asarray(run["rows"])
    dt = np.diff(r[:,0])
    ds = np.maximum(0,np.diff(r[:,7]))
    idx = tree.query((r[:-1,1:3]+r[1:,1:3])*.5)[1]
    straight = (r[:-1,7]>=10.) & (abs(curvature[idx])<=.015)
    for prop in run["props"]:
        straight &= abs(r[:-1,7]-prop[-1])>=15.
    sustained = straight & (r[:-1,4]>=4) & (r[1:,4]>=4)
    best_time,best_m = longest(sustained,dt,ds)
    return r,straight,dict(id=run["id"], status=run["status"], distanceM=ds[straight].sum(),
        timeS=dt[straight].sum(), meanProgressMps=ds[straight].sum()/dt[straight].sum(),
        distanceFractionAtOrAbove4=ds[sustained].sum()/ds[straight].sum(),
        longestContinuousS=best_time, correspondingDistanceM=best_m)


def main():
    center,_,_,curvature,_ = geometry()
    tree = cKDTree(center)
    data = [measure(json.loads((HERE/p).read_bytes()), tree, curvature) for p in RECORDS]
    dest = HERE/"long_training_evidence"
    dest.mkdir(exist_ok=True)
    (dest/"historical_cruise.json").write_bytes((json.dumps({
        "scope":"Previous recorded trials, descriptive, distinct from the preregistered new cruise gates.",
        "rule":"Start after 10 m. Geometric local curvature at midpoint no more than 0.015/m. At least 15 m from every obstacle along course progress. Continuous spans require speed at both endpoints >=4. No filtering by commands.",
        "runs":[m for _,_,m in data]}, ensure_ascii=False, indent=2)+"\n").encode())
    font_path = Path("C:/Windows/Fonts/msyh.ttc")
    if font_path.is_file():
        font_manager.fontManager.addfont(str(font_path))
        plt.rcParams["font.family"]=font_manager.FontProperties(fname=str(font_path)).get_name()
    bg,ink,green="#f6f6ee","#173d3e","#00766a"
    fig,axes=plt.subplots(2,1,figsize=(10,5.5),sharex=True,facecolor=bg)
    fig.subplots_adjust(left=.09,right=.97,bottom=.15,top=.78,hspace=.57)
    fig.text(.09,.92,"两次空场整圈，直路能持续跑到 4 m/s",color=ink,size=21,weight="bold")
    fig.text(.09,.85,"绿色为符合几何条件的直路，灰色为起步或转弯。曲线为实测车速。",color="#516665",size=11)
    for ax,(r,straight,result) in zip(axes,data[:2]):
        ax.set_facecolor(bg)
        ax.axhline(4,ls="--",lw=1,c="#b15832",zorder=1)
        ax.plot(r[:,7],r[:,4],color="#b9c1b9",lw=1.45)
        ax.plot(r[:,7],np.where(np.r_[straight,False],r[:,4],np.nan),color=green,lw=2)
        ax.set_title(f"起点 {result['id'].split('_')[1]}    直路均速 {result['meanProgressMps']:.2f} m/s"
                     f"    4 m/s 以上里程 {result['distanceFractionAtOrAbove4']:.0%}",
                     loc="left",fontsize=11,color=ink)
        ax.set_ylim(0,5.2);ax.set_yticks([0,2,4]);ax.set_ylabel("m/s",rotation=0,labelpad=12)
        ax.spines[["right","top"]].set_visible(False)
        ax.spines[["left","bottom"]].set_color("#a6b4ae");ax.tick_params(colors="#486461")
    axes[-1].set_xlabel("从各自起点的行驶进度（米）")
    fig.savefig(dest/"historical_cruise.png",dpi=140,facecolor=bg);plt.close(fig)


if __name__=="__main__":
    main()
