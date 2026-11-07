"""Inspectable full-loop replays of the procedural pretraining environment."""
import json
from pathlib import Path
import numpy as np
from v3_model import KINDS, DEFAULT_PARAMETERS, make_scenario, rollout

def make_preview(out,parameters,generation,version_name="V3.1"):
    cases=[]
    for i,kind in enumerate(KINDS):
        scenario=make_scenario(990000000+i,kind)
        results={}
        for version,p in (("V3.0",DEFAULT_PARAMETERS),(version_name,parameters)):
            r=rollout(scenario,p,120000+i,record=True)
            results[version]=r
        cases.append(dict(kind=kind,center=scenario.center.tolist(),width=scenario.width,obstacles=scenario.obstacles,results=results))
    payload=dict(generation=generation,cases=cases)
    (out/"preview_data.json").write_text(json.dumps(payload,allow_nan=False),encoding="utf8")
    page='''<!doctype html><meta charset="utf-8"><title>V3 预训练驾驶回放</title>
<style>body{font:16px system-ui;background:#101725;color:#e7edf4;margin:24px}h1{font-size:26px}button,select{font:inherit;padding:7px;background:#29384c;color:white;border:1px solid #64748b;border-radius:6px}canvas{width:48%;background:#172338;border-radius:10px;margin:10px .5%}input{width:94%}#info{white-space:pre-wrap}a{color:#7dd3fc}</style>
<h1>V3.0 / V3.1 · 程序化预训练回放</h1><p>这是检查点的独立 CPU 评估回放，不是 Unity 录像，也不计入 GPU 训练次数。左：裁判全图，仅供观看；右：车辆当时可见的局部道路、锥桶估计与候选轨迹。模型输入没有左图。</p>
<select id="case"></select> <select id="version"><option>V3.0</option><option>V3.1</option></select> <button id="play">播放 / 暂停</button> <span id="label"></span>
<div><canvas id="world" width="650" height="530"></canvas><canvas id="sensor" width="650" height="530"></canvas></div>
<input id="scrub" type="range" min="0" step="1"><p id="info"></p><a href="index.html">返回训练进度</a>
<script>const data=DATA;const choice=document.getElementById('case');const names={A:'A：中间可钻',B:'B：中间不足，需要绕行',C:'C：中间和外侧均可通行',sequence:'连续障碍',moving:'移动障碍',mixed:'动静混合',bend:'弯中障碍',clear:'空场整圈',missing:'观测缺失'};data.cases.forEach((s,i)=>choice.add(new Option(names[s.kind],i)));let playing=false;const W=world.getContext('2d'),S=sensor.getContext('2d');function draw(){const c=data.cases[+choice.value],r=c.results[version.value],row=r.trace[+scrub.value]||r.trace.at(-1);if(!row)return;scrub.max=r.trace.length-1;label.textContent='检查点 '+data.generation+' · '+row.t.toFixed(1)+' s';W.clearRect(0,0,650,530);S.clearRect(0,0,650,530);const scale=8,xy=p=>[325+p[0]*scale,265-p[1]*scale];function poly(ctx,pts,col,width=2){ctx.strokeStyle=col;ctx.lineWidth=width;ctx.beginPath();pts.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.stroke()}poly(W,[...c.center,c.center[0]].map(xy),'#475569',c.width*scale);poly(W,r.trace.slice(0,+scrub.value+1).map(d=>xy(d.pos)),'#2dd4bf',3);for(const o of c.obstacles){const p=c.center[o[0]],a=c.center[(o[0]+1)%c.center.length],b=c.center[(o[0]+c.center.length-1)%c.center.length],dx=a[0]-b[0],dy=a[1]-b[1],n=Math.hypot(dx,dy),off=o[1]+o[2]*Math.sin(row.t*o[3]+o[4]),q=xy([p[0]+dy/n*off,p[1]-dx/n*off]);W.fillStyle=o[2]?'#fb7185':'#fbbf24';W.beginPath();W.arc(...q,4,0,7);W.fill()}const p=xy(row.pos);W.save();W.translate(...p);W.rotate(row.heading);W.fillStyle='#60a5fa';W.fillRect(-3,-6,6,12);W.restore();const loc=(z,x)=>[325+x*43,490-z*38];if(row.road.length){S.fillStyle='#334155';S.beginPath();row.road.forEach((q,i)=>i?S.lineTo(...loc(q[0],q[1])):S.moveTo(...loc(q[0],q[1])));[...row.road].reverse().forEach(q=>S.lineTo(...loc(q[0],q[2])));S.closePath();S.fill()}for(const q of row.objects){S.fillStyle='#fbbf24';S.beginPath();S.arc(...loc(q[0],q[1]),14,0,7);S.fill();poly(S,[loc(q[0],q[1]),loc(q[0]+q[2],q[1]+q[3])],'#fb7185',2)}if(row.decision.path)poly(S,row.decision.path.map(q=>loc(q[0],q[1])),'#2dd4bf',3);S.fillStyle='#60a5fa';S.fillRect(309,479,32,49);info.textContent='整圈结果：'+r.status+' · 用时 '+r.time.toFixed(1)+' s · 完成 '+(100*r.progress).toFixed(1)+'%\n当前车速 '+row.speed.toFixed(2)+' m/s；目标速度 '+row.command[0].toFixed(2)+'；转向指令 '+row.command[1].toFixed(2)+'\n决策：'+row.decision.reason+'；可行候选 '+(row.decision.feasible??0)+'；不确定性扣除后余量 '+(row.decision.margin?.toFixed(2)??'未知')+' m\n灰色区域是本帧观测范围，范围外为未知。失败不会从回放中删除。'}choice.onchange=version.onchange=()=>{scrub.value=0;draw()};scrub.oninput=draw;play.onclick=()=>playing=!playing;setInterval(()=>{if(playing){scrub.value=(+scrub.value+1)%(+scrub.max+1);draw()}},100);draw();</script>'''
    (out/"preview.html").write_text(page.replace("V3.1",version_name).replace("DATA",json.dumps(payload,allow_nan=False)),encoding="utf8")
    # An exportable static artifact accompanies the interactive local replay.
    try:import matplotlib
    except ModuleNotFoundError:
        # The CUDA environment intentionally has only training dependencies.
        # The complete interactive replay above has no plotting dependency.
        (out/"static_plot_unavailable.txt").write_text("Interactive preview.html is complete. PNG export requires the host Python matplotlib environment.",encoding="utf8")
        return payload
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    fig,axes=plt.subplots(3,3,figsize=(13,11),facecolor="#111827")
    for ax,c in zip(axes.flat,cases):
        ax.set_facecolor("#172338");center=np.array(c["center"])
        tangent=np.roll(center,-1,axis=0)-np.roll(center,1,axis=0)
        tangent/=np.linalg.norm(tangent,axis=1)[:,None]
        normal=np.c_[tangent[:,1],-tangent[:,0]]
        outer=center+normal*c["width"]/2;inner=center-normal*c["width"]/2
        ax.add_patch(Polygon(np.r_[outer,outer[:1],inner[:1],inner[::-1]],color="#475569"))
        for st,offset,amp,hz,phase in c["obstacles"]:
            at=center[st]+normal[st]*(offset+amp*np.sin(phase))
            ax.plot(*at,"o",color="#fbbf24",markersize=4)
            if amp:
                ends=np.array([center[st]+normal[st]*(offset-amp),center[st]+normal[st]*(offset+amp)])
                ax.plot(*ends.T,":",color="#fb7185",linewidth=1)
        for version,col in (("V3.0","#fb7185"),(version_name,"#2dd4bf")):
            r=c["results"][version];points=np.array([d["pos"] for d in r["trace"]])
            if len(points):ax.plot(*points.T,color=col,lw=1.6,label=f"{version}: {r['status']} {r['time']:.1f}s")
        ax.set_title(c["kind"],color="white");ax.axis("equal");ax.tick_params(colors="#94a3b8");ax.legend(fontsize=7)
    fig.suptitle(f"V3 procedural pretraining | checkpoint {generation} | NOT Unity footage",color="white")
    fig.tight_layout();fig.savefig(out/"preview.png",dpi=130);plt.close(fig)
    return payload

if __name__=="__main__":
    import argparse
    p=argparse.ArgumentParser();p.add_argument("out",type=Path);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    model=json.loads((a.out/"model.json").read_bytes())
    make_preview(a.out,model["parameters"],model["generation"],model["version"])
