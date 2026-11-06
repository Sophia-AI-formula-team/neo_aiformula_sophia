"""Replay an existing A4 development segment in this independently owned Unity copy."""
import json
from pathlib import Path
import sys
from smooth_trials import Lab, SIM, detector, run_one, write

source=Path(sys.argv[1])
reference=json.loads((source/"runs"/"dev_dc01_base.json").read_bytes())
matrix=json.loads((source/"development_layouts.json").read_bytes())
case=next(c for c in matrix["cases"] if c["id"]=="dc01")
destination=Path(__file__).parent/"gap_revision"
lab=Lab(SIM,destination)
with detector(lab,"outer-edge","copy_parity") as (port,release):
    result=run_one(lab,"copy_parity",case,None,port,release,False)
physics=result["rows"]==reference["rows"]
decisions=[{k:v for k,v in d.items() if k!="gap"} for d in result["decisions"]]
same=(physics and decisions==reference["decisions"])
report=dict(reference=reference["id"],physicsIdentical=physics,allDecisionsIdentical=same,
            result=result["status"],sourceSolver=reference["request"]["solver"],copySolver=result["request"]["solver"])
write(destination/"copy_parity.json",report)
print(report,flush=True)
if not same:raise ValueError("Copy does not reproduce the reference physics and observations")
