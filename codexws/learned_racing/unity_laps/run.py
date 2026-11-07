"""Run a seeded curriculum against an already open Unity editor. No ROS/CAN transport."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.ndimage import gaussian_filter1d

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from course import outer_verge, resample
sys.path.insert(0,str(HERE.parent/"unity_cones"))
from experiment import write, invoke_editor

BASE=[1.55,8.,6.4,3.2,.17,5.0,.85,0.,0.]
NAMES=["pace","curve_lift","steer","lookahead","obstacle_margin","shift_distance","motion_forecast","near_line","far_line"]
SIM=Path("C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer")

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

class Lab:
    def __init__(self,sim,root,solver=None):
        self.sim=sim.resolve();self.root=root.resolve();self.work=self.root/".work";self.work.mkdir(exist_ok=True,parents=True)
        self.raw=self.root/"runs";self.raw.mkdir(exist_ok=True)
        sys.path.insert(0,str(self.sim/"workspace/tools"))
        from invoke_unity_bridge import invoke
        self.bridge=invoke
        private=os.environ.get("AIFORMULA_LAP_COPY")
        self.queue=None;proj=self.sim/"Simulator"
        if private:
            proj=Path(private).resolve();self.queue=proj/"LabQueue"
            owner=json.loads((self.queue/"owner.json").read_bytes())
            if Path(owner["source"]).resolve()!=self.sim/"Simulator":raise ValueError("Unexpected source project")
            if not (self.queue/"ready").exists():raise RuntimeError("Private Unity is importing")
            panel=dict(assets=(self.queue/"ready").read_text(encoding="utf-8-sig"),editor=str(Path(owner["engine"]).parent/"Data"),scene="private batch preview",dirty=False,play=False)
        else:
            panel=invoke_editor(invoke,'return new {assets=Application.dataPath, editor=EditorApplication.applicationContentsPath, scene=UnityEngine.SceneManagement.SceneManager.GetActiveScene().path,dirty=UnityEngine.SceneManagement.SceneManager.GetActiveScene().isDirty,play=EditorApplication.isPlaying};')
        if Path(panel["assets"]).resolve()!=proj/"Assets" or panel["dirty"] or panel["play"]:raise RuntimeError(panel)
        self.panel=panel
        eng=Path(panel["editor"]);managed=eng/"Managed"
        sources=sorted(HERE.glob("*.cs"));self.build=hashlib.sha256(b"".join(p.read_text(encoding="utf8").encode("utf8") for p in sources)).hexdigest()[:14]
        self.dll=self.work/f"LapLab_{self.build}.dll"
        if solver is not None and solver!=self.build:
            self.build=solver;self.dll=self.work/f"LapLab_{solver}.dll"
            previous=(HERE/"evidence/.work")/self.dll.name
            if not self.dll.is_file() and previous.is_file():
                self.dll=previous
            if not self.dll.is_file():
                revisions={"db4aecaf3e5c7d":"e7af050", "f25947e8af56df":"43423aed", "27d298ede25138":"743e85f", "ca509cc0c9c239":"6e96acd","788eb7af5abf9f":"ca33d0b","82c17a6e48068f":"7a99e84"}
                if solver not in revisions:raise RuntimeError("No recorded source revision for "+solver)
                source_dir=self.work/"sources"/solver
                source_dir.mkdir(exist_ok=True,parents=True)
                tree=revisions[solver]+":codexws/learned_racing/unity_laps"
                names=subprocess.check_output(["git","ls-tree","--full-tree","--name-only",tree],cwd=HERE,text=True).splitlines()
                sources=[source_dir/name for name in sorted(names) if name.endswith(".cs")]
                for source in sources:
                    (source_dir/source.name).write_bytes(subprocess.check_output(["git","show",tree+"/"+source.name],cwd=HERE))
                digest=hashlib.sha256(b"".join(p.read_text(encoding="utf8").encode("utf8") for p in sources)).hexdigest()[:14]
                if digest!=solver:raise RuntimeError("Historical source hash mismatch")
        if not self.dll.exists():
            mono=eng/"MonoBleedingEdge"
            refs=[managed/"UnityEngine/UnityEngine.dll",managed/"UnityEngine/UnityEditor.CoreModule.dll",
                  managed/"Newtonsoft.Json.dll",proj/"Library/ScriptAssemblies/Assembly-CSharp.dll",
                  mono/"lib/mono/4.7.2-api/Facades/netstandard.dll",*(managed/"UnityEngine").glob("UnityEngine.*Module.dll")]
            command=[str(mono/"bin/mono.exe"),str(mono/"lib/mono/4.5/mcs.exe"),"-target:library","-langversion:latest",
                     "-out:"+str(self.dll),*["-r:"+str(p) for p in refs],*map(str,sources)]
            p=subprocess.run(command,capture_output=True,text=True)
            (self.work/"compile.log").write_text(p.stdout+p.stderr,encoding="utf8")
            if p.returncode:raise RuntimeError(p.stdout+p.stderr)
        surface=proj/"Assets/TrainingEnvironment/Surface/track_surface.json"
        curb=resample(outer_verge(json.loads(surface.read_bytes())["meshes"]),.35)
        self.edge=gaussian_filter1d(curb,1.,axis=0,mode="wrap").round(5).tolist()
        self.asset_hashes={p.name:sha(p) for p in [surface,proj/"Assets/Scenes/MotorControllerValidation.unity",
            proj/"Assets/AIFormula/Runtime/Physics/AIFormulaDifferentialDrive.cs",proj/"Assets/AIFormula/Runtime/Physics/AIFormulaTireContact.cs",
            proj/"Assets/AIFormula/Runtime/Physics/AIFormulaArticulatedContact.cs",proj/"Assets/TrainingEnvironment/Prefabs/TrafficCone_75cm.prefab"]}
        description={"editor":panel,"solverHash":self.build,"sources":self.asset_hashes,
            "features":NAMES,"columns":"t x z heading v v_command yaw_command progress lane_offset road_clearance cone_clearance target_offset right_can_command_rpm left_can_command_rpm yaw_rate".split()}
        write(self.root/"baseline.json",description)
        write(self.root/"solvers"/(self.build+".json"),description)
        self.event("session",build=self.build,editor=panel)
    def event(self,kind,**extra):
        with (self.root/"events.jsonl").open("a",encoding="utf8") as fp:
            fp.write(json.dumps({"utc":time.strftime("%Y-%m-%dT%H:%M:%SZ",time.gmtime()),"kind":kind,**extra},ensure_ascii=False)+"\n")
    def run(self,name,weights,seed=41021,obstacles=False,vision=0,media=False,limit=1.,capture=.5,model="local_semantic",*,obstacle_specs=None,timing=None,encounter=None,smooth=None,lane_gate=None,external_control=False,departure_rule=None):
        rng=np.random.default_rng(seed);start=rng.random(); offset=rng.uniform(-.30,.30); angle=rng.uniform(-3.2,3.2)
        specs=[]
        if obstacle_specs is not None:
            if obstacles:
                raise ValueError("Choose either explicit or seeded obstacle placement")
            for spec in obstacle_specs:
                if len(spec)!=5 or not all(np.isfinite(spec)) or not 0<spec[0]<1 or spec[2]<0 or spec[3]<0:
                    raise ValueError("Invalid [station fraction, lateral m, amplitude m, angular frequency, phase]")
                specs.append([float(x) for x in spec])
        elif obstacles:
            station=rng.uniform(.15,.22);pair=rng.uniform(.43,.55);moving=rng.uniform(.72,.85)
            specs=[[station,rng.uniform(-.22,.22),0,0,0],[pair,-.38,0,0,0],[pair+.014,.70,0,0,0],
                   [moving,0,1.55,.57,rng.uniform(-np.pi,np.pi)]]
        w=[round(float(x),5) for x in weights]
        params=dict(weights=w,seed=int(seed),start=float(start),offset=float(offset),yaw=float(angle),obstacles=specs,
                    limit=limit,vision=bool(vision),model=model,captureInterval=capture,solver=self.build,media=media)
        if external_control:params["externalControl"]=True
        if departure_rule is not None:
            if departure_rule not in ("body_edge","all_wheel_centers"):raise ValueError("Unknown departure rule")
            params["departureRule"]=departure_rule
        if timing is not None:
            if timing["mode"] not in ("both","yield_only") or not 0<=timing["approach"]<=6:
                raise ValueError("Unknown timing action or invalid approach speed")
            params["timing"]=timing
        if encounter is not None:params["encounter"]=encounter
        if smooth is not None:params["smooth"]=smooth
        if lane_gate is not None:
            if len(lane_gate)!=3 or not all(np.isfinite(lane_gate)) or min(lane_gate)<=0:raise ValueError("Invalid lane innovation limits")
            params["laneGateThresholdsM"]=list(lane_gate)
        cache=self.raw/(name+".json")
        if cache.exists():
            old=json.loads(cache.read_bytes())
            if old["request"]!=params:raise ValueError("Trial ID already belongs to different settings: "+name)
            return old
        jobpath=self.work/(name+".request.json");out=self.work/(name+".result.json")
        cfg=dict(start=start,offset=offset,yaw=angle,weights=w,obstacles=specs,edge=self.edge,visionPort=vision,
                 output=str(out),media=str(self.work/"frames"/name) if media else "",limit=limit,captureInterval=capture)
        if external_control:cfg["externalControl"]=True
        if departure_rule is not None:cfg["departureRule"]=departure_rule
        if timing is not None:
            cfg.update(timingMode=timing["mode"],approachCap=timing["approach"])
        if encounter is not None:cfg["encounter"]=encounter
        if smooth is not None:cfg["smooth"]=smooth
        if lane_gate is not None:cfg["laneGateThresholdsM"]=params["laneGateThresholdsM"]
        if out.exists():
            previous=json.loads(jobpath.read_bytes())
            old_port=previous.pop("visionPort")
            comparable=dict(cfg);comparable.pop("visionPort")
            starts=[e for e in map(json.loads,(self.root/"events.jsonl").read_text(encoding="utf8").splitlines())
                    if e.get("kind")=="start" and e.get("trial")==name]
            if previous!=comparable or bool(old_port)!=bool(vision) or not starts or starts[-1]["params"]!=params:
                raise ValueError("Unmatched unclaimed Unity result for "+name)
            self.event("resume_finished_output",trial=name)
        else:
            if jobpath.exists():raise RuntimeError("An unfinished Unity request still owns "+name)
            write(jobpath,cfg);self.event("start",trial=name,params=params)
            if self.queue:
                q=self.queue/"request"
                if q.exists():raise RuntimeError("Private queue already has a request")
                temp=q.with_suffix(".tmp");temp.write_text(str(self.dll)+"\n"+str(jobpath),encoding="utf8");temp.replace(q)
            else:
                code="var asm=AppDomain.CurrentDomain.GetAssemblies().FirstOrDefault(a=>a.GetName().Name=="+json.dumps(self.dll.stem)+")??Assembly.Load(System.IO.File.ReadAllBytes("+json.dumps(str(self.dll))+"));return asm.GetType(\"LapLab.UnityRace\").GetMethod(\"Schedule\").Invoke(null,new object[]{"+json.dumps(str(jobpath))+"});"
                invoke_editor(self.bridge,code)
        began=time.monotonic()
        while True:
            try:
                result=json.loads(out.read_bytes())
                break
            except (FileNotFoundError,PermissionError,json.JSONDecodeError):
                pass  # Windows publishes the filename before the writer releases its file handle.
            if self.queue and (self.queue/"error").exists():
                raise RuntimeError((self.queue/"error").read_text(encoding="utf-8-sig"))
            if time.monotonic()-began>520:raise TimeoutError(name+" saw no terminal result")
            time.sleep(.16)
        result["request"]=params;result["id"]=name
        temp_cache=cache.with_suffix(".json.tmp")
        write(temp_cache,result);temp_cache.replace(cache)
        out.unlink();jobpath.unlink()
        self.event("finish",trial=name,status=result["status"],seconds=result["seconds"],progress=result["progress"],minRoad=result["minRoad"],minGap=result["minGap"])
        print(f"{name} {result['status']} {result['seconds']:.2f}s {result['progress']:.1f}/{result['length']:.1f} gap={result['minGap']:.2f} road={result['minRoad']:.2f}",flush=True)
        if result["status"].startswith("error:"):raise RuntimeError(result["status"])
        return result

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--sim",type=Path,default=SIM);p.add_argument("--root",type=Path,default=HERE/"evidence")
    p.add_argument("--name",default="probe");p.add_argument("--weights",type=float,nargs=len(BASE),default=BASE);p.add_argument("--seed",type=int,default=41021)
    p.add_argument("--obstacles",action="store_true");p.add_argument("--limit",type=float,default=1.);p.add_argument("--media",action="store_true")
    args=p.parse_args();Lab(args.sim,args.root).run(args.name,args.weights,args.seed,args.obstacles,media=args.media,limit=args.limit)
