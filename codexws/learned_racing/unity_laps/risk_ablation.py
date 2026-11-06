"""Diagnostic after freezing: raise the two passing/recovery speed caps on the same four layouts."""
import json
from risk_trials import BLIND, ROOT, Lab, SIM, detector, write
from risk_select import load_run
from risk_metrics import compare

def main():
    frozen=json.loads((ROOT/"selected.json").read_bytes())
    lab=Lab(SIM,ROOT,frozen["solver"])
    weights=list(frozen["weights"])
    weights[9:11]=[4.5,4.5]
    pairs=[]
    with detector(lab,"outer-edge","cap_ablation") as (port,model):
        if model != frozen["model"]:
            raise ValueError("Perception differs from the frozen policy")
        for s in BLIND:
            fitted=load_run(f"{frozen['prefix']}_{s}_traffic")
            relaxed=lab.run(f"caps_raised_{s}",weights,s,True,port,model=model,capture=.15)
            pairs.append(compare(relaxed,fitted))
    write(ROOT/"cap_ablation.json",dict(raisedPassingAndRecoveryMps=4.5,seeds=BLIND,pairs=pairs))
if __name__=="__main__":
    main()
