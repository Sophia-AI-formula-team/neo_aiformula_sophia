"""Run the Foxy command loop on the user's explicitly selected private Concept Unity host."""
import argparse
from contextlib import contextmanager
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import time


HERE = Path(__file__).resolve().parent


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError("Unexpected Concept solver source at " + old[:70])
    return text.replace(old, new)


def build(lab_root, out, source_dir):
    harness = lab_root / "research/learned_racing/unity_laps/run.py"
    module = importlib.util.spec_from_file_location("concept_lap_host", harness)
    runtime = importlib.util.module_from_spec(module)
    module.loader.exec_module(runtime)
    # This private snapshot contains old planner code and track geometry. Never
    # redistribute it with the adapter; use the team's independently held copy.
    files=sorted(source_dir.glob("*.cs"))
    digest=hashlib.sha256(b"".join(f.read_text(encoding="utf8").encode("utf8") for f in files)).hexdigest()
    if digest!="46a827df6272e4b6e2806d155c81149c6cd5ba4293d9860bba1b8639281bb934":
        raise RuntimeError("Missing or different private V6.26 solver sources: " + str(source_dir))
    solver = out / "solver"
    solver.mkdir()
    for path in files:
        shutil.copy2(path,solver/path.name)
    path = solver / "UnityRace.cs"
    src = path.read_text(encoding="utf8")
    src = replace_once(src, "public float approachCap;", "public float approachCap, testSeconds;")
    src = replace_once(src, "readonly Setup cfg;", "FoxyExchange foxy; readonly Setup cfg;")
    src = replace_once(src, "cameraMountCalibrationStart=new[]",
                       "foxy=new FoxyExchange(cfg.output); foxy.Mount(d,feed); cameraMountCalibrationStart=new[]")
    start = src.index("    Perception Detect(){")
    end = src.index("    void Tick(){", start)
    src = src[:start] + src[end:]
    # Keep only the camera-to-Foxy route. The track is still used separately
    # for spawn, wheel-boundary refereeing and scoring.
    start = src.index("        float?[] observed=null,filtered=null;")
    end = src.index("        if(d.MotorFault.Length",start)
    src = src[:start] + """        if(k%20==0){
            var obs=foxy.Frame(net,feed,d,t);
            policy.speedCmd=obs.control[0]; policy.yawCmd=obs.control[1];
            decisions.Add(new {t,version="Foxy",obs.decision,policy.speedCmd,policy.yawCmd});
            seenFrames++;
        }
        foxy.Apply(d);d.StepDrive(dt);scene.GetPhysicsScene().Simulate(dt);k++;
""" + src[end:]
    src = replace_once(src, 'if(t>260||t-lastAdvance>9)',
                       'if(t>=cfg.testSeconds-.0005f){Finish("duration");return;}if(t>260||t-lastAdvance>9)')
    src = replace_once(src, "net.ReceiveTimeout=20000;", "net.ReceiveTimeout=120000;")
    src = replace_once(src,"if(net!=null)net.Close();",
                       "try{if(d && foxy!=null)foxy.Release(d);}finally{if(net!=null)net.Close();}")
    path.write_text(src,encoding="utf8")
    shutil.copy2(HERE / "FoxyExchange.cs", solver)
    runtime.HERE = solver
    return runtime


@contextmanager
def private_queue(host):
    queue = host / "LabQueue"
    owner = json.loads((queue / "owner.json").read_bytes())
    if host.name != "UnityLab-v623-02" or Path(owner["project"]).resolve() != host:
        raise RuntimeError("Select the independently assigned Concept UnityLab-v623-02")
    if Path((queue / "ready").read_text(encoding="utf-8-sig")).resolve() != host / "Assets":
        raise RuntimeError("Host reported a different Unity project")
    lock = queue / "foxy1120.lock"
    with lock.open("x", encoding="utf8") as stream:
        stream.write(str(os.getpid()))
    try:
        for marker in ("request", "error", "stop"):
            if (queue / marker).exists():
                raise RuntimeError("Host has a pending " + marker)
        yield queue, owner
    finally:
        lock.unlink()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--concept", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--referee-source", type=Path, help="Private frozen V6.26 solver source directory")
    parser.add_argument("--start", type=float, default=.87, help="Referee-only spawn fraction")
    parser.add_argument("--offset", type=float, default=0., help="Right of outer lane centre (m)")
    parser.add_argument("--heading", type=float, default=0., help="Yaw right from track tangent (deg)")
    parser.add_argument("--seconds", type=float, default=5.)
    parser.add_argument("--port", type=int, default=16826)
    parser.add_argument("--compile-only", action="store_true")
    args=parser.parse_args()
    if not 0<=args.start<1 or not 0<args.seconds<=60 or not 0<args.port<65536:
        parser.error("Invalid spawn fraction, run duration or port")
    lab_root=args.concept.resolve()
    host=lab_root / ".runtime/UnityLab-v623-02"
    out=args.out.resolve()
    out.mkdir(parents=True,exist_ok=False)
    with private_queue(host) as (queue, owner):
        os.environ["AIFORMULA_LAP_COPY"]=str(host)
        os.environ["AIFORMULA_SIM_ROOT"]=str(Path(owner["source"]).resolve().parent)
        source = args.referee_source or lab_root/".runtime/v626_time/eval-inner-check/solver"
        runtime=build(lab_root,out,source.resolve())
        lab=runtime.Lab(runtime.SIM,out)
        provenance={"host":owner,"solver_hash":lab.build,
                    "unity_scene_sha256":runtime.sha(host/"Assets/Scenes/MotorControllerValidation.unity"),
                    "drive_assembly_sha256":runtime.sha(host/"Library/ScriptAssemblies/Assembly-CSharp.dll"),
                    "bridge_source_sha256":hashlib.sha256((HERE/"FoxyExchange.cs").read_bytes()).hexdigest()}
        (out/"host.json").write_text(json.dumps(provenance,indent=2),encoding="utf8")
        if args.compile_only:
            print(lab.dll, flush=True)
            return
        result=out/"result.json"
        config=out/"request.json"
        config.write_text(json.dumps(dict(start=args.start,offset=args.offset,yaw=args.heading,
            weights=runtime.BASE,obstacles=[],edge=lab.edge,visionPort=args.port,output=str(result),
            media="",limit=1.,captureInterval=.2,externalControl=True,departureRule="one_side_tread",
            externalYawLimit=8.,testSeconds=args.seconds)),encoding="utf8")
        staged=queue/"foxy1120.request.tmp"
        with staged.open("x",encoding="utf8") as stream:
            stream.write(str(lab.dll)+"\n"+str(config))
        staged.rename(queue/"request")
        began=time.monotonic()
        while not result.exists():
            if (queue/"error").is_file():
                raise RuntimeError((queue/"error").read_text(encoding="utf-8-sig"))
            if time.monotonic()-began>900:
                raise TimeoutError("Unity has not reported a result")
            time.sleep(.18)
        data=json.loads(result.read_bytes())
        print(data["status"], data["seconds"], data["progress"],data["seenFrames"],flush=True)
        if data["status"].startswith("error:"):
            raise RuntimeError(data["status"])


if __name__=="__main__":
    main()
