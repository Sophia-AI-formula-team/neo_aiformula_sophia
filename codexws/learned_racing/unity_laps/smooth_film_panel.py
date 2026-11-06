"""Visualize the logged frozen smooth controller without attributing V2 fallback to it."""
from bisect import bisect_right

from PIL import Image,ImageDraw
from learned_film_panel import HEIGHT, OUTCOMES

STATE={"clear":"常规行驶","observing":"等待足够的局部观测","idle":"常规控制","selected":"本周期选取",
       "holding":"继续通过","returning":"通过后回线","released":"结束此动作"}


def action_name(a,actions):
    if a==0:return "沿用冻结 V2"
    side,pad,speed=actions[a]
    if side==0:return f"沿参考线 · {speed:.1f} m/s"
    return f"{'左' if side<0 else '右'}侧通过 · {speed:.1f} m/s"


class SmoothFilmPanel:
    def __init__(self,run):
        self.run=run;cfg=run["request"].get("smooth") or {}
        if not cfg.get("forest") or cfg.get("mode")=="probe":
            raise ValueError("The long-training film requires a frozen fitted forest")
        self.model=cfg["modelId"];self.actions=cfg["actions"]
        self.selected=[i for i,d in enumerate(run["decisions"]) if (d.get("smooth") or {}).get("state")=="selected"]
        for i in self.selected:
            ev=run["decisions"][i]["smooth"]
            if ev["modelId"]!=self.model or len(ev["actionScores"])!=len(self.actions):
                raise ValueError("Invalid frozen model decision")

    def render(self,i,label):
        im=Image.new("RGB",(2048,HEIGHT),"#172f37");d=ImageDraw.Draw(im)
        rec=self.run["decisions"][i];ev=rec.get("smooth") or {};pos=bisect_right(self.selected,i)-1
        label(d,(42,35),"长训策略",30,"#ffffff",True)
        label(d,(212,36),STATE.get(ev.get("state"),ev.get("state") or "常规控制"),23,"#bbdcd6")
        label(d,(2005,36),"本趟结果   "+OUTCOMES.get(self.run["status"],self.run["status"]),26,"#eaf1cf",True,anchor="rm")
        d.line((42,65,2004,65),fill="#46616a",width=2)
        if pos>=0:
            last=self.run["decisions"][self.selected[pos]];le=last["smooth"];a=le["chosen"]
            label(d,(42,103),f"{last['t']:.2f} s 选取   {action_name(a,self.actions)}",29,"#dff4ad",True)
            if ev.get("chosen",-1)>0:
                text=f"观测建议横移 {ev['observedGoal']:+.2f} m    滤波后目标 {ev['targetOffset']:+.2f} m"
            else:text="此刻上方显示车辆实际使用的目标点、速度及偏航指令。"
            label(d,(42,156),text,22,"#e1e9ea")
        else:
            label(d,(42,103),"当前还没有选取避障动作",27,"#dff4ad",True)
            label(d,(42,153),"本周期使用道路观测和已有控制器生成指令。",22,"#d3e2e2")
        label(d,(1046,100),f"车身实际横摆 {rec['yaw']:+.2f} rad/s",25,"#e3eeed",True)
        label(d,(1536,100),f"目标横摆 {rec['yawCmd']:+.2f} rad/s",25,"#e3eeed",True)
        signed=rec["leftCan"]-rec["rightCan"]
        label(d,(1046,157),f"左−右目标轮速 {signed:+d} rpm",23,"#b9d7d5")
        label(d,(1536,157),"余量奖励上限 0.50 m",23,"#b9d7d5")
        return im
