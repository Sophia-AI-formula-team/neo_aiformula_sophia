"""Re-evaluate the same practice layouts after changing close-range recovery."""
import json
from pathlib import Path
import shutil
import sys

from gap_trials import ROOT, load, run_cases, sha, write

old=Path(sys.argv[1])
model=load(Path(sys.argv[2]))
ROOT.mkdir(parents=True,exist_ok=True)
cases=load(old/"practice.json")
menu={"r"+name[1:]:value for name,value in load(old/"menu.json").items()}
write(ROOT/"practice.json",cases)
write(ROOT/"menu.json",menu)
sources={}
for case in cases["cases"]:
    name=case["id"]+"_base.json"
    source=old/"runs"/name
    target=ROOT/"runs"/name
    target.parent.mkdir(parents=True,exist_ok=True)
    if not target.exists():shutil.copyfile(source,target)
    if sha(source)!=sha(target):raise ValueError("Baseline replay changed")
    sources[name]=sha(source)
write(ROOT/"reused_baselines.json",sources)
run_cases(cases["cases"],{k:dict(model,gap=v) for k,v in menu.items()},False)
