"""Render independent, post-race gate rulings alongside the sim vehicle's logged proposal."""
import json
import math

from referee_inset import position
from three_view import label,digest

ROUTE={
    "middle":"从两桶中间通过",
    "outside_inner_lane_cone":"从内道一侧绕过",
    "outside_outer_lane_cone":"从外道一侧绕过",
    "straddle_or_uncertain":"跨界或无法判定",
    "unfinished":"未完全过门",
    "not_reached":"未到该排",
}


def number(v):
    return "—" if v is None else f"{v:.2f}"


class GateView:
    """Ground truth is consumed only by this offline film generator."""
    def __init__(self,report_path,records):
        src=json.loads(report_path.read_bytes())
        case,arm=records[0].path.stem.rsplit("_",1)
        other,arm2=records[1].path.stem.rsplit("_",1)
        if other!=case:raise ValueError("Pair is from different gate layouts")
        pair=next(p for p in src["pairs"] if p["id"]==case)
        self.arms=[pair[arm]["gates"],pair[arm2]["gates"]]
        self.props=records[0].run["props"]
        self.records=records
        for r,k in zip(records,(arm,arm2)):
            if digest(r.path)!=pair["runHashes"][k]:raise ValueError("Gate judge and physical film have different source runs")
        self.indices=[i for g in self.arms[0] for i in g["propIndices"]]
        if len(set(self.indices))!=len(self.indices):raise ValueError("One prop appears in multiple gates")
        for left,right in zip(*self.arms):
            ids=left["propIndices"]
            if ids!=right["propIndices"]:raise ValueError("Compared gates differ")
            if any(math.dist(self.props[i][:2],left[key])>.001 for i,key in zip(ids,("leftHomeXZ","rightHomeXZ"))):
                raise ValueError("Judge gate center does not match raw simulator coordinates")
        self.provenance={"case":case,"compared":[arm,arm2],"judgeSHA256":digest(report_path),"arms":self.arms,
            "predictionSource":"decisions[].gap, projected ranges and travelTime; not a ground-truth measurement",
            "groundSource":"real logged prop positions, vehicle footprints at both axes; post-race only"}

    def draw_gates(self,d,project,t):
        for idx,g in enumerate(self.arms[0],1):
            a,b=(project(*position(self.props[i],t)) for i in g["propIndices"])
            d.line((a,b),fill="#74a6a9",width=4)
            x,y=(a if a[1]<b[1] else b)
            d.rounded_rectangle((x-20,y-30,x+20,y-10),4,fill="#25454c")
            label(d,(x,y-20),f"{idx} 排",13,"#ffffff",True,anchor="mm")

    def current_gate(self,j,t):
        gates=self.arms[j]
        active=0
        for n,g in enumerate(gates[1:],1):
            prior=gates[n-1]["rearClearS"]
            enter=g["frontEnterS"]
            if enter is not None and t>=enter:active=n
            elif prior is not None and t>=prior+.55:active=n
        return active,gates[active]

    def stats(self,d,j,t,name,color):
        n,g=self.current_gate(j,t)
        enter,leave=g["frontEnterS"],g["rearClearS"]
        finished=leave is not None and t>=leave
        started=enter is not None and t>=enter
        terminated=t>=self.records[j].run["seconds"]-.00002
        state=ROUTE[g["route"]] if finished or terminated else "车身正在过门" if started else "正在接近"
        y=914+151*j
        label(d,(1410,y),f"{name}   ·   第 {n+1} 排   {state}",22,color,True)
        v1=g["entryBodyMps"] if started else None
        v2=g["exitBodyMps"] if finished else None
        label(d,(1411,y+41),f"实测进 / 出门速度    {number(v1)} / {number(v2)}  m/s",20,"#264550")
        if finished or (terminated and started):
            label(d,(1411,y+81),f"最窄开口 {number(g.get('gateMinFreeOpeningM'))} m   两侧余量 {number(g.get('minLeftProjectedM'))} / {number(g.get('minRightProjectedM'))} m",18,"#49636b")
        else:
            label(d,(1411,y+81),"车尾清空后显示整段最窄开口与两侧投影余量。",17,"#537079")

    def footer(self,d,samples):
        proposal=samples[1]["decision"].get("gap") or {}
        if proposal.get("state")=="visible_opening":
            rows=len(proposal.get("crossings") or [])
            txt=f"车上规划   {rows} 排 · 估计 {proposal['travelTime']:.2f} s · 目标 {proposal['speed']:.2f} m/s"
        else:
            txt="本次感知周期未上报可见开口方案。"
        label(d,(1410,1237),txt,20,"#16333a",True)
