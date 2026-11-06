"""Matched observation/forecast ablations on the first complete direct-camera blind run."""
import json
import subprocess
import sys
from run import HERE,write
from camera_training import BLIND

def main():
    root=HERE/"evidence";load=lambda name:json.loads((root/"runs"/(name+".json")).read_bytes())
    cases=[load(f"rgb_test_{seed}_learned") for seed in BLIND]
    reference=next((r for r in cases if r["status"]=="lap"),None)
    if reference is None:
        reference=load("final_a4_48133");policy=root/"joint_policy.json"
    else:policy=root/"camera_policy.json"
    seed=reference["request"]["seed"];cases={"normal":reference["id"]}
    for key,flag in (("muted","--ignore-objects"),("zeroForecast","--zero-forecast")):
        id_="rgb_ablate_"+key
        subprocess.run([sys.executable,"-B",str(HERE/"camera_runs.py"),"--release","a4","--name",id_,"--seed",str(seed),
                        "--obstacles","--policy",str(policy),"--capture",".25",flag],check=True)
        cases[key]=id_
    write(root/"ablation.json",dict(seed=seed,trials=cases))
if __name__=="__main__":main()
