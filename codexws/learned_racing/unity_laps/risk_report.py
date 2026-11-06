"""Write an offline report for paired A4 runs, retaining failed and unreached zones."""
import argparse
from html import escape
import json
from pathlib import Path
import re

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from risk_metrics import APPROACH_M, RECOVERY_M, compare, measure


STATUS = {"lap": "完圈", "boundary": "触边", "obstacle": "碰桶", "lost_lane": "丢线", "stalled": "停滞"}
KIND = {"static": "单桶", "pair": "连续桶", "moving": "横移桶"}
BLUE = "#187781"
ORANGE = "#b15423"


def fmt(v, places=2):
    return "—" if v is None else f"{v:.{places}f}"


def result(m):
    return f'{m["lapSeconds"]:.2f} s' if m["finished"] else f'{STATUS.get(m["status"], m["status"])} · {m["progressM"]:.1f} m'


def timeline(run, group, obs):
    """Use independent time axes relative to the first sample in the evaluation window."""
    rows = np.asarray(run["rows"])
    available = rows[(rows[:, 7] >= group["stationM"] - APPROACH_M) & (rows[:, 7] <= group["endStationM"] + RECOVERY_M)]
    if not len(available):
        return None
    first, last = available[0, 0], available[-1, 0]
    # Keep a sample even for an immediate termination.
    samples = available[::max(1, len(available)//240)]
    traces = dict(actual=[[round(t-first, 2), round(v, 3)] for t, v in samples[:, [0, 4]]],
                  target=[[round(t-first, 2), round(v, 3)] for t, v in samples[:, [0, 5]]])
    selected = [d for d in run.get("decisions", []) if first - .001 <= d["t"] <= last + .001]
    if selected:
        traces["target"] = [[round(d["t"] - first, 2), round(d["speedCmd"], 3)] for d in selected]
        traces["cruise"] = [[round(d["t"] - first, 2), round(d["cruise"], 3)] for d in selected]
    risk = next((r for r in group.get("motionRisk", []) if r and r["conflict"]), None)
    snapshots = []
    for d in selected:
        snapshots.append({k: d.get(k) for k in ("t", "reason", "conflictRange", "speed", "yaw", "speedCmd", "yawCmd", "look", "target", "preferred",
                           "offset", "curvature", "ratio", "guardCap", "cruise", "observed", "tracks", "leftCan", "rightCan")})
    return {"from": float(first), "until": float(last), "traces": traces, "decisions": snapshots,
            "risk": risk, "label": obs, "passed": group["passed"], "gap": group["coneGapM"]}


def svg_plot(views):
    w, h, l, r, t, b = 820, 244, 52, 18, 21, 38
    valid = [v for v in views if v]
    if not valid:
        return '<p class="muted">两趟均未到达该区域。</p>'
    seconds = max(1, max(v["until"] - v["from"] for v in valid))
    vmax = max(1, int(np.ceil(max(y for v in valid for vals in v["traces"].values() for _, y in vals))))
    def xy(x, y):
        return (l + x * (w-l-r) / seconds, h-b - y * (h-t-b) / vmax)
    parts = [f'<svg role="img" aria-label="实测车速与轮速指令对应的目标车速，随区域内时间比较" viewBox="0 0 {w} {h}">']
    for value in np.linspace(0, vmax, 4):
        _, y = xy(0, value)
        parts.append(f'<line x1="{l}" y1="{y}" x2="{w-r}" y2="{y}" stroke="#ccd5cf"/><text x="{l-8}" y="{y+4}" text-anchor="end">{value:.1f}</text>')
    for value in np.linspace(0, seconds, 5):
        x, y = xy(value, 0)
        parts.append(f'<text x="{x}" y="{y+18}" text-anchor="middle">{value:.1f}</text>')
    parts.append(f'<text x="{l}" y="13">m/s</text><text x="{w/2}" y="{h-3}" text-anchor="middle">进入评价区后的仿真秒数</text>')
    for side, (v, color) in enumerate(zip(views, (ORANGE, BLUE))):
        if not v:
            continue
        for key in ("actual", "target"):
            # Preserve held commands as steps between controller updates.
            arr = v["traces"][key]
            if key == "target":
                arr = [arr[0]] + [p for a, z in zip(arr, arr[1:]) for p in ((z[0], a[1]), z)]
            points = " ".join(f"{X:.1f},{Y:.1f}" for x, y in arr for X, Y in [xy(x, y)])
            dash = 'stroke-dasharray="5 4"' if key == "target" else ""
            parts.append(f'<polyline fill="none" stroke="{color}" stroke-width="{1.8 if dash else 2.5}" {dash} points="{points}"/>')
        if v["risk"]:
            risk_t = v["risk"]["refereeOnsetS"] - v["from"]
            x, y = xy(risk_t, 0)
            parts.append(f'<path d="M{x-4:.1f},{y:.1f} l4,-6 4,6 z" fill="{color}"><title>{v["label"]} 首次裁判冲突估计</title></path>')
    return "".join(parts) + "</svg>"


def seed_summary(records, output):
    """Native-resolution static figure, with the same matched-distance denominator printed."""
    font_path = Path("C:/Windows/Fonts/msyh.ttc")
    bold_path = Path("C:/Windows/Fonts/msyhbd.ttc")
    def font(size, bold=False):
        return ImageFont.truetype(str(bold_path if bold else font_path), size)
    width, height = 1280, 146 + 84*len(records) + 68
    im = Image.new("RGB", (width, height), "#f6f7f0")
    g = ImageDraw.Draw(im)
    def txt(x,y,s,size=19,color="#22312d",bold=False):
        g.text((x,y),s,font=font(size,bold),fill=color,anchor="lt")
    txt(30,20,"稳定性与自由路段速度",35,bold=True)
    txt(30,74,"同起点、同布局比较。自由速度只取双方都走到的路段，排除每组障碍前 12 m 至最后一桶后 10 m。",18,"#52665e")
    txt(30,123,"起点 / 布局",17,bold=True);txt(245,123,"旧策略 → 新策略",17,bold=True);txt(707,123,"自由路段进度速度　m/s",17,bold=True)
    max_mps = max((s["mps"] or 0 for c in records for s in c["matchedFreeRoad"]), default=1) or 1
    for i,c in enumerate(records):
        y=163+i*84;a,b=c["before"],c["after"]
        label="路障" if a["zones"] else "空场"
        txt(30,y,f'{a["seed"]}  {label}',20,bold=True)
        txt(245,y,result(a)+"   →   "+result(b),18)
        if a["zones"]:
            n=lambda m:sum(x["passed"] for x in m["zones"])
            txt(245,y+29,f'障碍区通过  {n(a)}/{len(a["zones"])}  →  {n(b)}/{len(b["zones"])}',15,"#51665d")
        for j,(v,color) in enumerate(zip(c["matchedFreeRoad"],(ORANGE,BLUE))):
            yy=y+j*24; speed=v["mps"] or 0;right=712+int(350*speed/max_mps)
            if speed>0:g.rounded_rectangle((712,yy+3,right+1,yy+15),5,fill=color)
            txt(right+9,yy,fmt(v["mps"]),16,color)
        g.line((30,y+63,1250,y+63),fill="#d6ded6")
    y=height-38;txt(30,y,"橙色为旧策略，青色为新策略。失败记录显示判停原因与位置，空场圈速仅列完圈结果。",16,"#51665d")
    im.save(output)


def practice_report(training):
    """Optionally provide {phase_label: runs} to keep practice and qualification separate."""
    phases = training if isinstance(training, dict) else {"训练": training}
    sections = []
    for phase, runs in phases.items():
        measurements = [measure(r) for r in runs]
        if not measurements:
            continue
        candidates = {}
        for m in measurements:
            name = re.sub(r"_"+str(m["seed"])+r"(_clear|_traffic)?$", "", m["id"])
            candidates.setdefault(name, []).append(m)
        heading = escape(str(phase))
        sections.append(f'<h2>{heading}</h2><div class="scroll"><table><thead><tr><th>候选</th><th>完圈</th><th>障碍区通过</th><th>最差试次完成距离</th><th>完圈自由速度 m/s</th></tr></thead><tbody>')
        for name, group in candidates.items():
            done = [m for m in group if m["finished"]]
            secs = sum(m["freeRoad"]["seconds"] for m in done)
            pace = sum(m["freeRoad"]["meters"] for m in done) / secs if secs else None
            zones = [z for m in group for z in m["zones"]]
            minimum = min(min(1., m["progressM"]/m["courseM"]) for m in group)
            sections.append(f'<tr><td>{escape(name)}</td><td>{len(done)}/{len(group)}</td><td>{sum(z["passed"] for z in zones)}/{len(zones)}</td>'
                            f'<td><meter min="0" max="1" value="{minimum:.5f}"></meter> {minimum:.0%}</td><td>{fmt(pace)}</td></tr>')
        sections.append(f'</tbody></table></div><details><summary>展开{heading}全部 {len(measurements)} 趟记录</summary><div class="scroll"><table>'
                        '<thead><tr><th>试次</th><th>终止结果</th><th>通过区</th><th>路边最小余量 m</th><th>已行驶自由速度 m/s</th></tr></thead><tbody>')
        for m in measurements:
            sections.append(f'<tr><td>{escape(m["id"])}</td><td>{result(m)}</td><td>{sum(z["passed"] for z in m["zones"])}/{len(m["zones"])}</td>'
                            f'<td>{fmt(m["minRoadM"])}</td><td>{fmt(m["freeRoad"]["mps"])}</td></tr>')
        sections.append('</tbody></table></div></details>')
    return ''.join(sections)


def write_report(pairs, output_path, training=()):
    """pairs is an ordered iterable of (reference, challenger); training may be grouped by phase."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True,exist_ok=True)
    records=[]; blocks=[]; options=[];snapshots=[]
    for a,b in pairs:
        c=compare(a,b);records.append(c)
        for z,(ga,gb) in enumerate(zip(c["before"]["zones"],c["after"]["zones"])):
            views=[timeline(a,ga,"旧策略"),timeline(b,gb,"新策略")]
            k=len(blocks);name=f'种子 {c["before"]["seed"]} · {KIND[ga["kind"]]}'
            options.append(f'<option value="{k}">{name}</option>')
            desc=[]
            for g in (ga,gb):
                state="已通过" if g["passed"] else "区内终止" if g["entered"] else "未到计时区"
                desc.append(f'{state}　·　净距 {fmt(g["coneGapM"])} m')
            action=gb.get("controller",{})
            yield_action=action.get("lowestYieldTarget")
            reason=(f'<p>该区记录到 {action["yieldUpdates"]} 次让行指令、{action["recoveryUpdates"]} 次恢复指令。'
                    + (f'让行目标最低 {yield_action["speedCmd"]:.2f} m/s，同次曲率巡航值 {yield_action["cruise"]:.2f} m/s。' if yield_action else "")+"</p>") if action else ""
            blocks.append(f'<section class="encounter" data-key="{k}"><p class="legend"><span>旧策略　{desc[0]}</span><b>新策略　{desc[1]}</b></p>{svg_plot(views)}{reason}</section>')
            snapshots.append(views[1]["decisions"] if views[1] else [])
    if not records:raise ValueError("At least one matched pair is required")
    image_path=output_path.with_suffix(".png")
    seed_summary(records,image_path)
    traffic=[c for c in records if c["before"]["zones"]];clean=[c for c in records if not c["before"]["zones"]]
    def completed(seq,side):return sum(c[side]["finished"] for c in seq)
    practice=practice_report(training)
    table=[]
    for c in records:
        a,b=c["before"],c["after"]
        zones=[]
        for ga,gb in zip(a["zones"],b["zones"]):
            def time(x):return fmt(x["seconds"]) if x["passed"] else "中止" if x["entered"] else "未到"
            zones.append(f'{KIND[ga["kind"]]} {time(ga)} → {time(gb)}')
        table.append(f'<tr><td>{a["seed"]}</td><td>{result(a)}</td><td>{result(b)}</td><td>{fmt(c["matchedFreeRoad"][0]["meters"],1)}</td>'
                     f'<td>{fmt(c["matchedFreeSpeedDeltaMps"],2)}</td><td>{escape("；".join(zones) or "空场")}</td></tr>')
    chart = ("<h2>障碍出现时的动作</h2><p>实线为实测车速，虚线为控制器目标。两次试车按进入同一评价区的时刻对齐。"
        "三角标出裁判离线估算的首次冲突：未来 3 s 内若保持当前航向、车速与桶速，车身到桶的预计净距小于 0.25 m。"
        "这个估计只用来标注轨迹。</p><label>查看路段 <select id='zone'>"+"".join(options)+"</select></label>"
        +"".join(blocks)+"<h3>新策略逐次发令</h3><label id='stepLabel'>指令序号 <input type='range' id='step' min='0' value='0'></label>"
        "<div id='command' aria-live='polite'></div>") if blocks else ""
    head = f'加设路障时，旧策略完圈 {completed(traffic,"before")}/{len(traffic)}，新策略 {completed(traffic,"after")}/{len(traffic)}。' if traffic else ""
    if clean:head+=f'空场配对 {len(clean)} 组，分别完圈 {completed(clean,"before")} 和 {completed(clean,"after")} 次。'
    html="""<!doctype html><html lang="zh-CN"><meta charset="utf-8"><link rel="icon" href="data:,"><meta name="viewport" content="width=device-width,initial-scale=1"><title>A4 驾驶考核 · 路障与自由速度</title><style>
*{box-sizing:border-box}body{background:#f6f7f0;color:#22312d;font:16px/1.65 "Segoe UI","Microsoft YaHei",sans-serif;max-width:1180px;padding:20px 24px;margin:auto}h1{font-size:clamp(26px,4vw,42px);margin:14px 0}h2{border-top:1px solid #c6d3c6;padding-top:20px;margin-top:26px}h3{margin-bottom:6px}p{margin:5px 0 14px}img,svg{max-width:100%;height:auto}svg{width:100%;max-height:280px;background:#fff;border-radius:8px}svg text{fill:#4e635a;font:12px sans-serif}.scroll{overflow-x:auto}table{border-collapse:collapse;width:100%;text-align:left;white-space:nowrap}th,td{padding:7px 9px;border-bottom:1px solid #d2ded5}td{font-variant-numeric:tabular-nums}select,input{font:inherit;max-width:100%}.legend{display:flex;flex-wrap:wrap;gap:18px}.legend span{color:#a2421d}.legend b{color:#14656e}.muted,small{color:#50675b}summary{cursor:pointer}input[type=range]{width:55%;vertical-align:middle}#command{margin:10px 0;padding:12px 15px;border-left:3px solid #187781;background:#eef2e9}#command p:last-child{margin:0}@media(max-width:600px){body{padding:12px}.legend{gap:5px;font-size:14px}}
</style><main>"""
    html+=f'<h1>A4 驾驶考核</h1><p>{head}</p><img src="{escape(image_path.name)}" alt="各起点完圈和自由区速度对比"><div class="scroll"><table><thead><tr><th>起点</th><th>旧策略</th><th>新策略</th><th>共同自由距离 m</th><th>速度差 m/s</th><th>分区用时 s（旧 → 新）</th></tr></thead><tbody>{"".join(table)}</tbody></table></div><small>分区秒数用于描述驾驶动作。遇到横移桶的让行耗时不进入速度评分。部分路线的共同区间速度不能替代完圈成绩。</small>{chart}{practice}</main>'
    dataset=json.dumps(snapshots,allow_nan=False,separators=(",",":")).replace("<","\\u003c")
    html+='<script type="application/json" id="actions">'+dataset+'</script><script>'+JS+'</script></html>'
    output_path.write_text(html,encoding="utf8")
    return records


JS="""
const by=id=>document.getElementById(id),zone=by('zone');
if(zone){
  const frames=JSON.parse(by('actions').textContent),step=by('step');
  const number=(n,k=2)=>n==null?'—':Number(n).toFixed(k);
  function draw(){
    const d=frames[Number(zone.value)][Number(step.value)],box=by('command');
    box.replaceChildren();by('stepLabel').hidden=!d;
    if(!d){box.textContent='该路段无可用的逐次决策日志。';return}
    const guides=(d.observed||[]).map(x=>number(x)).join(' / ');
    const reason=({yield:'路障让行',recover:'避让后恢复',road:'正常行驶',insufficient_lane:'车道观测不足',start:'起步'})[d.reason]||d.reason||'未记录';
    const nearest=(d.tracks||[]).filter(o=>o[0]>0).sort((a,b)=>a[0]-b[0])[0];
    const lines=[
      '仿真 '+number(d.t)+' s · '+reason+'。前方 2 / 3.5 / 5.5 / 8 m 车道横向观测为 '+guides+' m · 局部路障轨迹 '+(d.tracks||[]).length+' 条。',
      nearest?'最近前方目标：纵向 '+number(nearest[0])+' m，横向 '+number(nearest[1])+' m，估计横向速度 '+number(nearest[2])+' m/s。':'当前这一帧没有前方目标轨迹。',
      '预瞄 '+number(d.look)+' m，目标横移 '+number(d.offset)+' m，预瞄目标横坐标 '+number(d.target)+' m。曲率 '+number(d.curvature,3)+' /m。',
      '巡航目标 '+number(d.cruise)+' m/s，候选速度比例 '+number(d.ratio)+'，风险限速 '+(Number(d.guardCap)>20?'未限制':number(d.guardCap)+' m/s')+'；下发 '+number(d.speedCmd)+' m/s，实测 '+number(d.speed)+' m/s。',
      '目标偏航 '+number(d.yawCmd)+' rad/s，实测 '+number(d.yaw)+' rad/s；左轮指令 '+number(d.leftCan,0)+' RPM，右轮指令 '+number(d.rightCan,0)+' RPM。'
    ];
    for(const msg of lines){let p=document.createElement('p');p.textContent=msg;box.append(p)}
  }
  function change(){document.querySelectorAll('.encounter').forEach(el=>el.hidden=el.dataset.key!==zone.value);const actions=frames[Number(zone.value)];step.max=Math.max(0,actions.length-1);const lowest=actions.reduce((best,d,i)=>d.reason==='yield'&&(best<0||d.speedCmd<actions[best].speedCmd)?i:best,-1);step.value=Math.max(0,lowest);draw()}
  const moving=[...zone.options].find(o=>o.textContent.includes('横移')&&frames[Number(o.value)].some(d=>d.reason==='yield'));if(moving)zone.value=moving.value;
  step.oninput=draw;zone.onchange=change;change();
}
"""


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pair",action="append",nargs=2,required=True,type=Path)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--training",type=Path,nargs="*",default=[])
    args=p.parse_args()
    load=lambda f:json.loads(f.read_bytes())
    write_report([(load(a),load(b)) for a,b in args.pair],args.output,[load(f) for f in args.training])
