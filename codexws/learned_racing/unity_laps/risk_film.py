"""Film the first declared completed holdout; use a labelled practice run only if necessary."""
import json
import numpy as np
from risk_trials import BLIND, ROOT, Lab, SIM, detector, write
from risk_select import load_run

def main():
    final=json.loads((ROOT/"selected.json").read_bytes())
    lab=Lab(SIM,ROOT,final["solver"])
    records=[load_run(f"{final['prefix']}_{seed}_traffic") for seed in BLIND]
    case=next((r for r in records if r["status"]=="lap"),None)
    if case is None:case=load_run(final["source"]+"41021")
    with detector(lab,"outer-edge","decision_film",True) as (port,model):
        assert model == case["request"]["model"]
        film=lab.run("decision_film",final["weights"],case["request"]["seed"],True,port,True,capture=.10,model=model)
    if film["rows"]!=case["rows"] or len(film["decisions"])!=len(case["decisions"]):
        raise RuntimeError("Filmed vehicle trajectory differs")
    delta=np.zeros(4);range_delta=0.
    for live,scored in zip(film["decisions"],case["decisions"]):
        if any(live[k]!=scored[k] for k in scored if k not in ("tracks","conflictRange")):
            raise RuntimeError("Filmed controller inputs or commands differ")
        if len(live["tracks"])!=len(scored["tracks"]):raise RuntimeError("Track counts differ")
        if live["tracks"]:
            delta=np.maximum(delta,np.abs(np.array(live["tracks"])-scored["tracks"]).max(axis=0))
        range_delta=max(range_delta,abs(live["conflictRange"]-scored["conflictRange"]))
    if max(delta.max(),range_delta)>.001:raise RuntimeError("Camera track discrepancy exceeds tolerance")
    write(ROOT/"film_source.json",dict(source=case["id"],replay=film["id"],seed=case["request"]["seed"],
          identicalTrace=True,identicalCommands=True,maxTrackDelta_z_x_vx_vz=delta.tolist(),maxConflictRangeDeltaM=range_delta))
if __name__=="__main__":
    main()
