"""Publish measured pairs, a command example and physical/score acceptance checks."""
import hashlib
import json
from pathlib import Path
import numpy as np
import av
from command_replay import crop_window
from risk_trials import BLIND,CHECK,PRACTICE,ROOT,GUARD_FIELDS,CORRIDOR_FIELDS,write
from risk_select import load_run
from risk_metrics import free_steering
from risk_report import write_report

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extra_evidence(ablation, steering):
    rows = []
    for pair in ablation["pairs"]:
        seed = pair["after"]["seed"]
        cells = []
        for key in ("before", "after"):
            zone = next(z for z in pair[key]["zones"] if z["kind"] == "moving")
            cells.append(f'{zone["coneGapM"]:.2f} m / {zone["seconds"]:.2f} s' if zone["passed"] else "碰桶")
        rows.append(f'<tr><td>{seed}</td><td>{cells[0]}</td><td>{cells[1]}</td></tr>')
    def describe(side):
        runs = [s[side] for s in steering]
        rms = np.sqrt(sum(r["squaredRateTime"] for r in runs) / sum(r["seconds"] for r in runs))
        returns = 100 * sum(r["rapidReturns"] for r in runs) / sum(r["meters"] for r in runs)
        return f"{rms:.2f} rad/s²", f"{returns:.2f} 次/100 m"
    a, b = describe("before"), describe("after")
    return (f'<h2>车身有没有少摇晃</h2><p>共同跑到的自由路段中，实测偏航角加速度 RMS 为 {a[0]} → {b[0]}。'
            f'一秒内快速回摆为 {a[1]} → {b[1]}。回摆只计偏航越过 ±0.08 rad/s 后又在一秒内反向的事件，'
            '排除车速小于 1 m/s 的样本。两项都受车速与弯形影响。</p>'
            '<h2>单独放宽避让限速，会怎样？</h2><p>保留相同感知、路径评分与轮速转换。'
            '同种子再跑，只改通过与恢复的速度上限。每列列出横移桶最小净距和通过时间。</p>'
            '<div class="scroll"><table><thead><tr><th>起点</th><th>上限 4.5 / 4.5 m/s</th><th>冻结策略 1.4 / 1.9 m/s</th>'
            '</tr></thead><tbody>'+''.join(rows)+'</tbody></table></div>'
            '<p>这四个布局在选参后作机理对照。<a href="cap_ablation.json">对照原始读数</a></p>'
            '<h2>20318 为什么留下来</h2><p>冻结策略在 68.6 秒时看到桶在左前 6.5 m，估计桶在向左移。'
            '到 69.8 秒，桶已在左前 3.6 m，估计改为向右移。此时车才选右绕，最终在 236.0 m 碰桶。'
            '将所有桶的未来位置都向两侧扩展，另一次试验反而在 49.3 m 的静态桶前触边。'
            '这项改动未入选。<a href="../reversal_diagnostic/README.md">实验记录</a> · '
            '<a href="../reversal_diagnostic/protocol.json">试验协议</a></p>')

def moving_example(r):
    # Match an actual relative observation to the injected moving cone offline,
    # then choose the nearest observation to its scorer-defined approach gate.
    o=r["props"][3]
    rows=np.asarray(r["rows"])
    candidates=[]
    for d in r["decisions"]:
        if not d["reason"].startswith("yield") or d["speedCmd"]>=d["cruise"]-.3:continue
        i=np.abs(rows[:,0]-d["t"]).argmin()
        pos=rows[i,1:3];h=rows[i,3]
        obj=np.array(o[:2])+o[4]*np.sin(o[5]+d["t"]*o[6])*np.array(o[2:4])
        dx,dz=obj-pos
        local=np.array([dx*np.sin(h)+dz*np.cos(h),dx*np.cos(h)-dz*np.sin(h)])
        if not (1.5<local[0]<9):continue
        for track in d["tracks"]:
            err=float(np.linalg.norm(local-track[:2]))
            if err<.80:
                candidates.append((abs(local[0]-5)+err,d,track,local.tolist(),err))
    if not candidates:return None
    _,decision,observed,truth,error=min(candidates,key=lambda x:x[0])
    return dict(trial=r["id"],decision=decision,matchedTrack=observed,refereeRelative=truth,associationErrorM=error)

def main():
    frozen=json.loads((ROOT/"selected.json").read_bytes())
    film=json.loads((ROOT/"film_source.json").read_bytes())
    training={k:[] for k in ("练习","资格复验")}
    for prefix in ("fit_stable_","fit_balanced_","fit_fast_","corridor_"):
        for s in (*PRACTICE,*CHECK):
            if (ROOT/"runs"/f"{prefix}{s}.json").exists():
                training["练习" if s in PRACTICE else "资格复验"].append(load_run(f"{prefix}{s}"))
    results={}
    steering=[]
    for layout,seeds,filename in (("traffic",BLIND,"index.html"),("clear",BLIND[:2],"clear.html")):
        pairs=[(load_run(f"prior_{s}_{layout}"),load_run(f"{frozen['prefix']}_{s}_{layout}")) for s in seeds]
        records=write_report(pairs,ROOT/filename,training if layout=="traffic" else ())
        if layout == "traffic":
            for (old, new), record in zip(pairs, records):
                end = record["matchedUntilM"]
                steering.append(dict(seed=record["before"]["seed"],before=free_steering(old,end),after=free_steering(new,end)))
        for old,new in pairs:
            assert new["request"]["solver"]==frozen["solver"] and new["request"]["weights"]==frozen["weights"]
            for r in (old,new):
                assert (r["status"]=="lap")==bool(r["gate"]==32 and r["progress"]>=r["length"])
                if r["status"]=="lap":assert r["minRoad"]>=0 and r["minGap"]>0
        results[layout]=records
    replay=load_run("decision_film");original=load_run(film["source"])
    assert original["rows"]==replay["rows"] and film["identicalCommands"]
    example=moving_example(original)
    fields=CORRIDOR_FIELDS if len(frozen["weights"])==13 else GUARD_FIELDS
    out=dict(selection=frozen,parameters=dict(zip(fields,frozen["weights"])),evaluation=results,example=example,film=film,
             practiceRuns=sum(map(len,training.values())), matchedFreeSteering=steering,
             runHashes={p.name:sha(p) for p in sorted((ROOT/"runs").glob("*.json"))})
    write(ROOT/"results.json",out)
    page=ROOT/"index.html";html=page.read_text(encoding="utf8")
    details='<p><a href="clear.html">同起点无新增路障圈速</a> · <a href="results.json">原始指标与指令样本</a> · <a href="selected.json">冻结参数</a></p>'
    if (ROOT/"commands.mp4").is_file():
        start, _ = crop_window(replay)
        with av.open(str(ROOT/"commands.mp4")) as video:
            stream = video.streams.video[0]
            time = example["decision"]["t"] - start if example else 0.
            for frame in video.decode(stream):
                if frame.time >= time:
                    frame.to_image().save(ROOT/"command-example.jpg", quality=93)
                    break
        details+='<video controls playsinline preload="metadata" poster="command-example.jpg" style="width:100%;max-width:980px" src="commands.mp4"></video>'
    details+=('<p>录像来自种子 '+str(film["seed"])+'。车每 0.2 秒更新一次。A4 从前视画面估计车线与桶的位置，'
              '规划器比较十种横向目标与三档速度。它根据桶的运动、车身来得及横移的距离、'
              '路边剩余空间和上次指令选出动作，再将预瞄点和实测偏航换成左右轮目标 RPM。'
              '候选动作、代价公式和电机转换由人编写；这次用 Unity 练习和复验，从预先给定的参数组合中选定弯速、收油与恢复设置。'
              '裁判单独使用整图计分。</p>')
    html=html.replace('<h1>A4 驾驶考核</h1>','<h1>A4 驾驶考核</h1>'+details)
    caps = ROOT/"cap_ablation.json"
    if caps.is_file():
        html=html.replace("</main>",extra_evidence(json.loads(caps.read_bytes()),steering)+"</main>")
    page.write_text(html,encoding="utf8")
    print("Measured pairs", [(layout,[(r["before"]["status"],r["after"]["status"]) for r in records]) for layout,records in results.items()])
if __name__=="__main__":main()
