"""Resumable 30,000-attempt CEM curriculum. CPU only; no vehicle or Unity IO."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import time
import traceback
from contextlib import nullcontext
import numpy as np
from v3_model import DEFAULT_PARAMETERS, PARAMETER_BOUNDS, KINDS, make_scenario, rollout

HERE=Path(__file__).resolve().parent
def save(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+".tmp")
    temp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf8")
    temp.replace(path)

def run_trial(job):
    index,seed,kind,parameters=job
    result=rollout(make_scenario(seed,kind),parameters,seed+100000000)
    result.pop("trace",None)
    return dict(index=index,scenario_seed=seed,parameters=parameters,**result)

def rank(rows):
    # A fast collision can never outrank a completed trial. Normalise by course
    # length; each population shares two cases and has fresh robustness cases.
    return (sum(not r["completed"] for r in rows),
            float(np.mean([r["time"]/r["lap_length"] if r["completed"] else
                           20+10*(1-min(r["progress"],1)) for r in rows])),
            float(np.mean([r["steering_tv"]/r["lap_length"] for r in rows])))

def dashboard(out,state,history):
    data=dict(attempts=state["done"],target=state["target"],generation=state["generation"],
              status=state["status"],version=state["version"],backend=state["backend"],history=history[-200:],
              parameters=state["best"],unique_scenarios=state.get("unique_scenarios",0),elapsed=time.time()-state["started"],pid=os.getpid())
    save(out/"status.json",data)
    document='''<!doctype html><meta charset="utf-8"><meta http-equiv="refresh" content="30">
<title>V3.1 程序化驾驶预训练</title><style>body{background:#111827;color:#e5e7eb;font:17px system-ui;max-width:1100px;margin:35px auto}h1{font-size:28px}.cards{display:flex;gap:24px}.card{background:#1f2937;padding:20px;border-radius:12px;flex:1}b{font-size:30px}canvas{background:#1f2937;width:100%;margin-top:24px;border-radius:12px}pre{font-size:14px}a{color:#60a5fa}</style>
<h1>V3.1 · 程序化驾驶预训练</h1><p>显式轨迹预测 + 试错学习参数。此处是代理动力学仿真，Unity A4 闭环成绩另计。</p>
<div class="cards"><div class="card">实际试次<br><b id="trials"></b></div><div class="card">参数更新轮次<br><b id="gen"></b></div><div class="card">运行状态<br><b id="state"></b></div></div>
<p>每轮共享 2 个配对场景，其余场景独立生成。曲线随场景难度变化，不能直接当作盲测提升。</p>
<canvas id="chart" width="1100" height="300"></canvas><p>青色：每轮训练完成率；红色：碰撞率。失败均保留。</p>
<p>场景：A 中间可通过 · B 中间不可通过需绕行 · C 两者可行 · 连续 · 移动 · 混合 · 弯中 · 空场 · 观测缺失</p>
<p><a href="preview.html">查看当前检查点的传感器、规划与行驶回放</a> · <a href="model.json">参数模型</a> · <a href="manifest.json">来源与计数规则</a></p><pre id="params"></pre>
<script>const d=DATA;trials.textContent=d.attempts+' / '+d.target;gen.textContent=d.generation;state.textContent=d.status;params.textContent=JSON.stringify(d.parameters,null,2);const c=chart.getContext('2d');c.strokeStyle='#475569';for(let j=0;j<5;j++){let y=25+j*60;c.beginPath();c.moveTo(35,y);c.lineTo(1080,y);c.stroke();c.fillStyle='#aaa';c.fillText((1-j*.25).toFixed(2),2,y)}for(const [key,col]of [['success','#2dd4bf'],['collision','#fb7185']]){c.strokeStyle=col;c.lineWidth=3;c.beginPath();d.history.forEach((v,i)=>{let x=35+i*1045/Math.max(1,d.history.length-1),y=265-v[key]*240;i?c.lineTo(x,y):c.moveTo(x,y)});c.stroke()}</script>'''
    (out/"index.html").write_text(document.replace("V3.1",state["version"]).replace("DATA",json.dumps(data,ensure_ascii=False)),encoding="utf8")

def main():
    p=argparse.ArgumentParser();p.add_argument("--out",type=Path,required=True)
    p.add_argument("--attempts",type=int,default=30000);p.add_argument("--seed",type=int,default=731901)
    p.add_argument("--workers",type=int,default=4);p.add_argument("--population",type=int,default=12)
    p.add_argument("--cases",type=int,default=3);p.add_argument("--resume",action="store_true")
    p.add_argument("--backend",choices=("cpu","cuda"),default="cuda")
    p.add_argument("--smoke",action="store_true");a=p.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
    if a.attempts<1 or not 1<=a.workers<=6 or a.population<4 or a.cases<1:raise ValueError("Invalid budget")
    names=list(DEFAULT_PARAMETERS);low=np.array([PARAMETER_BOUNDS[n][0] for n in names]);high=np.array([PARAMETER_BOUNDS[n][1] for n in names])
    source={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in ("v3_model.py","v3_train.py","v3_visual.py","v3_gpu.py","v3_evidence/plant_fit.json")}
    checkpoint=out/"checkpoint.json";journal=out/"attempts.jsonl";history=[]
    if a.resume:
        state=json.loads(checkpoint.read_bytes())
        if state["sources"]!=source:raise ValueError("Source differs from checkpoint; create a new version/run")
        if state["target"]!=a.attempts or state["population"]!=a.population or state["cases"]!=a.cases or state["backend"]!=a.backend:raise ValueError("Resume budget/layout mismatch")
        if (out/"history.json").exists():history=json.loads((out/"history.json").read_bytes())
    else:
        if checkpoint.exists() or journal.exists():raise FileExistsError("Use --resume or new output")
        state=dict(version="V3.2" if a.backend=="cuda" else "V3.1",backend=a.backend,sources=source,target=a.attempts,done=0,generation=0,status="starting",started=time.time(),
                   population=a.population,cases=a.cases,best=DEFAULT_PARAMETERS,mean=list(DEFAULT_PARAMETERS.values()),
                   std=((high-low)*.22).tolist(),rng=np.random.default_rng(a.seed).bit_generator.state,plan=None)
        for name,digest in source.items():
            content=(HERE/name).read_bytes()
            if hashlib.sha256(content).hexdigest()!=digest:raise RuntimeError("Source changed during snapshot")
            dest=out/"sources"/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(content)
        save(out/"manifest.json",dict(version=state["version"],sources=source,seed=a.seed,attempts=a.attempts,
          environment="procedural surrogate, not Unity",physics="explicit lag model fitted on 45 archived Unity runs; 15 separate one-step checks; perturbations not real-car validation",
          planner="explicit finite-horizon footprint constraints",learned="CEM parameter search; no neural driving policy",
          sensors="local symbolic camera with visibility/noise/dropout, wheel error, gyro; no global pose/map",
          counts="training rollouts only; candidate path evaluations and heldout tests excluded",workers=a.workers,
          sampling="2 paired scenarios per population; remaining scenarios independently generated per candidate",
          holdout_seed_start=990000000,stop="create STOP file; latest complete attempt is retained",backend=a.backend,
          parallel_environments=a.population*a.cases if a.backend=="cuda" else a.workers,
          acceleration="CUDA batched trajectory prediction; CPU plant/sensors and CEM" if a.backend=="cuda" else "CPU process pool"))
        save(checkpoint,state)
    # Completed records are authoritative across a crash between append and checkpoint.
    records={}
    if journal.exists():
        for line in journal.read_text(encoding="utf8").splitlines():
            r=json.loads(line)
            if r["index"] in records:raise ValueError("Duplicate attempt index")
            records[r["index"]]=r
    state["done"]=len(records);rng=np.random.default_rng();rng.bit_generator.state=state["rng"]
    if a.backend=="cuda":
        import torch
        from v3_gpu import run_jobs
        if not torch.cuda.is_available():raise RuntimeError("CUDA unavailable")
        torch.set_num_threads(2)
        save(out/"gpu.json",dict(name=torch.cuda.get_device_name(0),torch=torch.__version__,device_count=torch.cuda.device_count(),batch=a.population*a.cases))
    with (ProcessPoolExecutor(max_workers=a.workers) if a.backend=="cpu" else nullcontext(None)) as pool:
        try:
            while state["done"]<a.attempts:
                if (out/"STOP").exists():state["status"]="stopped";break
                if state["plan"] is None:
                    mean=np.array(state["mean"]);std=np.array(state["std"])
                    sampled=np.clip(rng.normal(mean,std,(a.population,len(names))),low,high)
                    sampled[0]=[state["best"][n] for n in names]
                    paired=rng.integers(1,800000000,min(2,a.cases)).tolist()
                    kinds=[KINDS[(state["generation"]*a.cases+i)%len(KINDS)] for i in range(a.cases)]
                    start=state["done"];plan=[]
                    for c,vector in enumerate(sampled):
                        params=dict(zip(names,vector.tolist()))
                        seeds=paired+rng.integers(1,800000000,max(0,a.cases-2)).tolist()
                        for j,(seed,kind) in enumerate(zip(seeds,kinds)):
                            index=start+c*a.cases+j
                            if index<a.attempts:plan.append([index,seed,kind,params])
                    state["plan"]=plan;state["rng"]=rng.bit_generator.state;save(checkpoint,state)
                pending=[job for job in state["plan"] if job[0] not in records]
                for result in (pool.map(run_trial,pending) if pool else run_jobs(pending)):
                    with journal.open("a",encoding="utf8") as f:f.write(json.dumps(result,allow_nan=False)+"\n");f.flush();os.fsync(f.fileno())
                    records[result["index"]]=result;state["done"]=len(records);state["status"]="training"
                    save(checkpoint,state)
                batch=[records[j[0]] for j in state["plan"]]
                groups=[batch[i:i+a.cases] for i in range(0,len(batch),a.cases) if len(batch[i:i+a.cases])==a.cases]
                if groups:
                    groups.sort(key=rank);elite=groups[:max(2,len(groups)//4)]
                    vectors=np.array([[g[0]["parameters"][n] for n in names] for g in elite])
                    state["mean"]=(.65*vectors.mean(axis=0)+.35*np.array(state["mean"])).tolist()
                    state["std"]=np.maximum(vectors.std(axis=0),.035*(high-low)).tolist()
                    state["best"]=groups[0][0]["parameters"]
                state["generation"]+=1;state["plan"]=None
                state["unique_scenarios"]=len({r["scenario_seed"] for r in records.values()})
                summary=dict(generation=state["generation"],attempts=state["done"],success=float(np.mean([r["completed"] for r in batch])),
                             collision=float(np.mean([r["contact"] for r in batch])),mean_speed=float(np.mean([r["mean_speed"] for r in batch])))
                history.append(summary);save(out/"history.json",history)
                model=dict(version=state["version"],trained_attempts=state["done"],generation=state["generation"],parameters=state["best"],sources=source,
                           acceptance="training checkpoint only, independent Unity validation required")
                save(out/"model.json",model);save(out/"checkpoints"/f"{state['generation']:04d}.json",model)
                save(checkpoint,state);dashboard(out,state,history)
                print(json.dumps(summary),flush=True)
                if state["generation"]==1 or state["generation"]%25==0:
                    from v3_visual import make_preview
                    make_preview(out,state["best"],state["generation"],state["version"])
                if a.smoke:state["status"]="smoke_complete";break
            else:state["status"]="complete"
        except BaseException:
            state["status"]="error";(out/"error.txt").write_text(traceback.format_exc(),encoding="utf8");raise
        finally:save(checkpoint,state);dashboard(out,state,history)

if __name__=="__main__":main()
