"""Publish measured pairs, a command example and physical/score acceptance checks."""
import hashlib
import json
from pathlib import Path
import numpy as np
from risk_trials import BLIND,CHECK,PRACTICE,ROOT,GUARD_FIELDS,CORRIDOR_FIELDS,write
from risk_select import load_run
from risk_metrics import compare,measure
from risk_report import write_report

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

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
    for layout,seeds,filename in (("traffic",BLIND,"index.html"),("clear",BLIND[:2],"clear.html")):
        pairs=[(load_run(f"prior_{s}_{layout}"),load_run(f"{frozen['prefix']}_{s}_{layout}")) for s in seeds]
        records=write_report(pairs,ROOT/filename,training if layout=="traffic" else ())
        for old,new in pairs:
            assert new["request"]["solver"]==frozen["solver"] and new["request"]["weights"]==frozen["weights"]
            for r in (old,new):
                assert (r["status"]=="lap")==bool(r["gate"]==32 and r["progress"]>=r["length"])
                if r["status"]=="lap":assert r["minRoad"]>=0 and r["minGap"]>0
        results[layout]=records
    replay=load_run("decision_film");original=load_run(film["source"])
    assert original["rows"]==replay["rows"] and original["decisions"]==replay["decisions"]
    example=moving_example(original)
    fields=CORRIDOR_FIELDS if len(frozen["weights"])==13 else GUARD_FIELDS
    out=dict(selection=frozen,parameters=dict(zip(fields,frozen["weights"])),evaluation=results,example=example,film=film,
             practiceRuns=sum(map(len,training.values())),
             runHashes={p.name:sha(p) for p in sorted((ROOT/"runs").glob("*.json"))})
    write(ROOT/"results.json",out)
    page=ROOT/"index.html";html=page.read_text(encoding="utf8")
    details='<p><a href="clear.html">同起点无新增路障圈速</a> · <a href="results.json">原始指标与指令样本</a> · <a href="selected.json">冻结参数</a></p>'
    if (ROOT/"commands.mp4").is_file():
        details+='<video controls playsinline preload="metadata" style="width:100%;max-width:980px" src="commands.mp4"></video>'
    html=html.replace('<h1>A4 驾驶考核</h1>','<h1>A4 驾驶考核</h1>'+details)
    page.write_text(html,encoding="utf8")
    print("Measured pairs", [(layout,[(r["before"]["status"],r["after"]["status"]) for r in records]) for layout,records in results.items()])
if __name__=="__main__":main()
