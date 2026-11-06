"""Independently reconstruct the installed Archived1030 right/left integer CAN command."""
import json
import math
import os
import struct
from pathlib import Path
import av
from risk_trials import BLIND,ROOT,SIM,write
from risk_results import sha
from risk_select import load_run


def wheel_rpm(v, yaw_right):
    # Unity's JSON uses the shortest decimal that round-trips to the C# single.
    # Recover that value before the motor controller's double-precision inverse.
    v, yaw_right = struct.unpack("<ff", struct.pack("<ff", v, yaw_right))
    y=-yaw_right
    length,k,x,c=(1.53908994,3.15496243,3.36664054,-.0621119) if y>=0 else (1.68283261,2.91627673,-3.22124235,1.6954783)
    y=max(-c+1e-6,min(length-c-1e-6,y))
    ros=x-math.log(length/(y+c)-1)/k
    v=max(-4.5,min(4.5,v))
    def wheel(rate,a,b):
        corrected=0 if rate==0 else math.copysign(abs(rate)/a+b,rate)
        return round(corrected*60/(2*math.pi)*1.1)
    return (wheel(v/.127+.6/.254*ros,.834,2.76),wheel(v/.127-.6/.254*ros,.844,2.81))


def main():
    f=json.loads((ROOT/"selected.json").read_bytes());count=0;discrepancies=[]
    for layout,seeds in (("traffic",BLIND),("clear",BLIND[:2])):
        for s in seeds:
            run=load_run(f"{f['prefix']}_{s}_{layout}")
            assert run["request"]["weights"]==f["weights"] and run["request"]["solver"]==f["solver"]
            for d in run["decisions"]:
                got=(d["rightCan"],d["leftCan"])
                want=wheel_rpm(d["speedCmd"],d["yawCmd"])
                if got!=want:discrepancies.append((run["id"],d["t"],got,want))
                count+=1
    priv=Path(os.environ["AIFORMULA_LAP_COPY"]);live=SIM/"Simulator"
    files=[Path(x) for x in ("Assets/Scenes/MotorControllerValidation.unity","Assets/AIFormula/Runtime/Control/AIFormulaMotorController.cs",
                              "Assets/TrainingEnvironment/Surface/track_surface.json","Assets/AIFormula/Runtime/Perception/AIFormulaRgbCameraTopic.cs")]
    files+=sorted(p.relative_to(priv) for p in (priv/"Assets/AIFormula/Runtime/Physics").rglob("*.cs"))
    physical={f.as_posix():sha(priv/f) for f in files}
    assert all(physical[f.as_posix()]==sha(live/f) for f in files)
    with av.open(str(ROOT/"commands.mp4")) as video:
        stream=video.streams.video[0];frames=sum(1 for _ in video.decode(stream))
        assert frames>=150 and stream.width>=1000
    out=dict(decisionPairs=count,identicalCanPairs=count-len(discrepancies),canDifferences=discrepancies,
             sourceHashes=physical,decodedFrames=frames,videoSha256=sha(ROOT/"commands.mp4"))
    write(ROOT/"audit.json",out);print(count,"commands,",len(discrepancies),"mismatches;",frames,"frames")
    if discrepancies:
        raise ValueError("One or more integer wheel targets differ from the archived motor map")

if __name__=="__main__":
    main()
