"""Restart the practice/development campaign from the first missing recorded job.

Run only when you exclusively own the private editor. Blind evaluation is a separate
command after recording its development freeze and receiving the registered matrix.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

from smooth_trials import HERE,ROOT

def main():
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path,default=ROOT);a=p.parse_args()
    if not os.environ.get("AIFORMULA_LAP_COPY"):raise RuntimeError("Set AIFORMULA_LAP_COPY to the exclusively owned private editor")
    root=a.output.resolve()
    def invoke(script,*args):
        subprocess.run([sys.executable,"-B",str(HERE/script),*args,"--output",str(root)],check=True,cwd=HERE)
    def stage(size,start):
        invoke("smooth_trials.py","train","--start",str(start),"--end",str(size))
        invoke("smooth_fit.py",str(size))
    def dev(tag):
        invoke("smooth_trials.py","eval","--tag",tag,"--arm","learned","--model",str(root/"checkpoints"/tag/"model.json"))
    stage(32,1);stage(64,33)
    invoke("smooth_trials.py","parity");invoke("smooth_trials.py","eval","--tag","dev","--arm","base")
    dev("ck32");dev("ck64")
    stage(80,65);dev("ck80")
    invoke("smooth_policytree.py","80");dev("ck80p")
    invoke("smooth_select.py","calibrate_actions","--size","80")
    if json.loads((root/"checkpoints/ck80m/calibration.json").read_bytes())["selected"]["changed"]>0:dev("ck80m")
    invoke("smooth_select.py","fixed","--size","80")
    # The operator signs the final immutable freeze once all physical reports
    # have been reviewed; this script never reveals or executes the blind suite.
if __name__=="__main__":main()
