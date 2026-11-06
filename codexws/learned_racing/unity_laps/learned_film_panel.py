"""Use recorded kernel-regression action returns, issued choices and referee outcomes in a film."""
from bisect import bisect_right

from PIL import Image, ImageDraw


HEIGHT = 216
ACTION_STATES = {"observing":"等待可评估目标", "selected":"本周期选择动作",
                 "holding":"继续执行所选动作", "clear":"所选动作结束", "new_object":"新目标进入，交还常规避让"}
OUTCOMES = {"segment":"跑完本段", "lap":"完成一圈", "obstacle":"碰桶终止", "boundary":"触边终止",
            "stalled":"进度停滞", "lost_lane":"道路观测丢失"}


def action_text(index,actions):
    if index==0:return "沿用常规避让"
    offset,speed=actions[index]
    side="直行" if abs(offset)<.01 else f"{'左' if offset<0 else '右'}移 {abs(offset):.1f} m"
    return f"{side} · {speed:.1f} m/s"


class LearnedPanel:
    def __init__(self,run):
        self.run=run
        cfg=run["request"].get("encounter") or {}
        if not cfg.get("actions") or cfg.get("mode")=="probe":
            raise ValueError("Learning panel requires a fitted policy, not a forced training action")
        self.actions=cfg["actions"]
        self.decisions=run["decisions"]
        self.selected=[i for i,d in enumerate(self.decisions) if (d.get("encounter") or {}).get("state")=="selected"]
        self.model=cfg["modelId"]
        for i in self.selected:
            e=self.decisions[i]["encounter"]
            if e.get("modelId")!=self.model or len(e.get("actionScores") or [])!=len(self.actions):
                raise ValueError("Incomplete scores or mismatched decision weights")
        self.outcome=OUTCOMES.get(run["status"],run["status"])

    def sample(self,i):
        e=self.decisions[i].get("encounter") or {}
        last=bisect_right(self.selected,i)-1
        source=self.decisions[self.selected[last]] if last>=0 else None
        return e,source

    def render(self,i,label):
        surface=Image.new("RGB",(2048,HEIGHT),"#152f39");d=ImageDraw.Draw(surface)
        current,source=self.sample(i)
        state=ACTION_STATES.get(current.get("state"),"当前没有策略评估")
        label(d,(43,35),"学习策略",30,"#ffffff",True)
        label(d,(204,37),state,23,"#c2dcdb")
        end="本趟结果  "+self.outcome+f"   /   最小桶净距 {self.run['minGap']:.2f} m"
        label(d,(2004,34),end,24,"#ffd7a1" if self.run["status"] in ("obstacle","boundary") else "#c0e8d4",True,anchor="rm")
        d.line((43,67,2004,67),fill="#44616a",width=2)
        if source is None:
            label(d,(43,111),"尚未触发动作评分",31,"#ffffff",True)
            label(d,(44,168),"此时画面上方显示常规控制器根据道路和锥桶观测发出的指令。",23,"#c2dcdb")
            return surface
        decision=source["encounter"]
        chosen=decision["chosen"];features=decision["features"]
        label(d,(43,105),f"{source['t']:.2f} s 选择   {action_text(chosen,self.actions)}",32,"#dff4ac",True)
        label(d,(44,156),f"选取时   前距 {features[3]:.1f} m    横向速度估计 {features[1]:+.2f} m/s    {int(features[4])} 个跟踪目标",22,"#e0e9ea")
        label(d,(44,190),"正横向速度朝道路右侧。这里显示实际选取时的观测。",18,"#a7c1c4")
        scores=decision["actionScores"]
        best=sorted(range(len(scores)),key=lambda k:(-scores[k],k))[:3]
        label(d,(1024,94),f"动作估计回报 · 前 {len(best)} 项",21,"#c2dcdb",True)
        for k,action in enumerate(best):
            x=1024+k*330
            color="#ddf4aa" if action==chosen else "#e1eaea"
            label(d,(x,133),action_text(action,self.actions),21,color,action==chosen)
            label(d,(x,175),f"{scores[action]:+.2f}",29,color,True,True)
        return surface
