"""Film the first declared completed holdout; use a labelled practice run only if necessary."""
import json
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
    if film["rows"]!=case["rows"] or film["decisions"]!=case["decisions"]:
        raise RuntimeError("Filmed rerun differs from scored run")
    write(ROOT/"film_source.json",dict(source=case["id"],replay=film["id"],seed=case["request"]["seed"],identicalTrace=True))
if __name__=="__main__":
    main()
