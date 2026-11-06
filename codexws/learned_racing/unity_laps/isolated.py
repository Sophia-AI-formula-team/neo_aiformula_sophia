"""Start a private, windowless Unity batch project for parallel simulator experiments."""
import argparse,json,os,shutil,subprocess
from pathlib import Path
from run import HERE,SIM,write

ENGINE=Path("E:/Unity/Editor/2022.3.22f1/Editor/Unity.exe")
def main():
    p=argparse.ArgumentParser();p.add_argument("--sim",type=Path,default=SIM);p.add_argument("--copy",type=Path,required=True);p.add_argument("--restart-owned",action="store_true");args=p.parse_args()
    target=args.copy.resolve();src=(args.sim/"Simulator").resolve()
    queue=target/"LabQueue"
    if args.restart_owned:
        owner=json.loads((queue/"owner.json").read_bytes())
        if Path(owner["project"])!=target or Path(owner["source"])!=src or (target/"Temp/UnityLockfile").exists():raise RuntimeError("Project ownership or active lock")
        for marker in ("ready","stop","error"):(queue/marker).unlink(missing_ok=True)
    else:
        target.mkdir(parents=True)
        for key in ("Assets","ProjectSettings","Packages"):shutil.copytree(src/key,target/key)
    # The separate batch job has a file queue and must not register on the live editor's MCP port.
    for file in ("manifest.json","packages-lock.json"):
        manifest=target/"Packages"/file;data=json.loads((src/"Packages"/file).read_bytes());data["dependencies"].pop("com.coplaydev.unity-mcp",None)
        if file=="manifest.json":data["dependencies"]["com.unity.nuget.newtonsoft-json"]="3.2.1"
        else:data["dependencies"]["com.unity.nuget.newtonsoft-json"]["depth"]=0
        write(manifest,data)
    cache=src/"Library/PackageCache"
    shutil.copytree(cache,target/"Library/PackageCache",dirs_exist_ok=True,ignore=shutil.ignore_patterns("com.coplaydev.unity-mcp*"))
    # Original live bridge menu depends on the deliberately disconnected MCP server.
    live=target/"Assets/AIFormula/Editor/Experiments/AIFormulaReproductionLive.cs"
    if live.is_file():live.rename(live.with_suffix(".cs.disabled"))
    editor=target/"Assets/Editor";editor.mkdir(exist_ok=True);shutil.copyfile(HERE/"BatchHost.cs.txt",editor/"LapExperimentHost.cs")
    queue.mkdir(exist_ok=True);env={**os.environ,"AIFORMULA_EXPERIMENT_QUEUE":str(queue)}
    proc=subprocess.Popen([str(ENGINE),"-batchmode","-force-d3d11","-projectPath",str(target),"-executeMethod","LapExperimentHost.Start","-logFile",str(queue/"editor.log")],
                           env=env,creationflags=getattr(subprocess,"CREATE_NO_WINDOW",0))
    write(queue/"owner.json",{"pid":proc.pid,"project":str(target),"source":str(src),"engine":str(ENGINE),"queue":str(queue)})
    print(target,proc.pid)
if __name__=="__main__":main()
