"""Fit local pace and yielding parameters with fixed layouts, then freeze before the blind set."""
import hashlib
import json
import numpy as np
from risk_trials import BLIND, CHECK, PRACTICE, ROOT, INITIAL, Lab, SIM, HERE, detector, write
from risk_metrics import rank

def main():
    lab = Lab(SIM, ROOT, "f25947e8af56df")
    # Shared local feedback architecture. Each candidate can drive its own line and choose to yield.
    candidates = {
        "stable": [4.15, 8.7, 6.4, 3.5, .35, 4.8, 1.18, .55, -.35, 1.25, 1.70, .95],
        "balanced": [4.45, 7.4, 6.8, 3.4, .27, 5., 1.1, .8, -.5, 1.40, 1.90, 1.15],
        "fast": [4.50, 5.6, 7., 3.65, .29, 4.5, 1.45, 1.2, -.8, 1.70, 2.30, 1.25],
    }
    write(ROOT/"protocol.json",dict(practice=PRACTICE, qualification=CHECK, blind=BLIND, candidates=candidates,
          policyInputs="camera sections and tracked cone observations, body speed and yaw rate",
          selection="completion count, accepted clearances, zones passed, worst/mean progress, free-road pace; top two to qualification"))
    groups={}
    with detector(lab,"outer-edge","practice") as (port,model):
        def race(name, seed):
            return lab.run(f"fit_{name}_{seed}",candidates[name],seed,True,port,model=model,capture=.15)
        for name in candidates:
            groups[name]=[race(name,s) for s in PRACTICE]
        finalists=sorted(candidates,key=lambda k:rank(groups[k]),reverse=True)[:2]
        for name in finalists:
            groups[name].extend(race(name,s) for s in CHECK)
        chosen=max(finalists,key=lambda k:rank(groups[k]))
        scores={k:list(rank(v)) for k,v in groups.items()}
        observerSha=hashlib.sha256((HERE/"vision.py").read_bytes()).hexdigest()
        write(ROOT/"frozen.json",dict(name=chosen,weights=candidates[chosen],solver=lab.build,model=model,
              observerSha=observerSha,trainingScores=scores,qualification=CHECK,blind=BLIND))
        lab.event("freeze",chosen=chosen,scores=scores,blind=BLIND)
        for seed,traffic in [(s,True) for s in BLIND]+[(s,False) for s in BLIND[:2]]:
            name=f"frozen_{seed}_{'traffic' if traffic else 'clear'}"
            lab.run(name,candidates[chosen],seed,traffic,port,model=model,capture=.15)
if __name__=="__main__":
    main()
