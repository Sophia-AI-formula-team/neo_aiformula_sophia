"""Run the same V3 Python planner against isolated Unity RGB/virtual sensors."""
import argparse
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import time
from run import HERE,Lab,SIM,BASE

def main():
    p=argparse.ArgumentParser();p.add_argument("--name",required=True);p.add_argument("--seed",type=int,default=771301)
    p.add_argument("--model",type=Path);p.add_argument("--limit",type=float,default=.12);p.add_argument("--obstacles",action="store_true")
    a=p.parse_args();lab=Lab(SIM,HERE/"v3_unity_evidence")
    folder=lab.work/a.name;folder.mkdir(parents=True,exist_ok=True);ready=folder/"ready.json"
    if ready.exists():raise FileExistsError("New name required")
    cfg=json.loads((SIM/"Simulator/UserSettings/AIFormulaRoadDetector.json").read_bytes())
    args=[cfg["python"],"-B",str(HERE/"v3_vision.py"),str(SIM/"Simulator/Assets/StreamingAssets/RoadDetector/a4"),str(ready)]
    if a.model:args.extend(["--model",str(a.model.resolve())])
    with (folder/"server.log").open("w",encoding="utf8") as log:
        proc=subprocess.Popen(args,stdout=log,stderr=log,creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            started=time.monotonic()
            while not ready.exists():
                if proc.poll() is not None or time.monotonic()-started>100:raise RuntimeError("Vision server did not start; see "+str(folder))
                time.sleep(.2)
            meta=json.loads(ready.read_bytes())
            lab.run(a.name,BASE,a.seed,a.obstacles,meta["port"],True,limit=a.limit,capture=.2,
                    model=meta["version"]+"/A4/"+meta["weights"]+"/"+meta["controlSha"],external_control=True)
        finally:
            # Terminate only the child process owned by this invocation. The
            # server may still be waiting on the previous connection.
            if proc.poll() is None:proc.terminate()
            proc.wait(timeout=10)
if __name__=="__main__":main()
