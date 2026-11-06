"""Compare corridor commitment to the first guarded finalist using only practice/qualification outcomes."""
import json
from risk_trials import BLIND,CHECK,PRACTICE,HERE,ROOT,Lab,SIM,detector,write
from risk_metrics import rank

CORRIDOR=[4.5,6.2,7.0,3.30,.24,4.8,1.10,.8,-.5,1.85,2.15,1.25,.48]

def load_run(name):
    return json.loads((ROOT/"runs"/(name+".json")).read_bytes())

def main():
    previous=json.loads((ROOT/"frozen.json").read_bytes())
    seeds=(*PRACTICE,*CHECK)
    old=[load_run(f"fit_{previous['name']}_{s}") for s in seeds]
    lab=Lab(SIM,ROOT,"27d298ede25138")
    with detector(lab,"outer-edge","corridor_practice") as (port,model):
        challengers=[lab.run(f"corridor_{s}",CORRIDOR,s,True,port,model=model,capture=.15) for s in seeds]
        if rank(challengers)>rank(old):
            final=dict(previous,name="corridor",weights=CORRIDOR,solver=lab.build,model=model,prefix="validated",source="corridor_")
        else:
            final=dict(previous,prefix="frozen",source="fit_"+previous["name"]+"_")
        final["selectionComparison"]={"corridor":rank(challengers),previous["name"]:rank(old)}
        write(ROOT/"selected.json",final);lab.event("final_selection",name=final["name"],basis=list(seeds))
        if final["prefix"]=="validated":
            for s,traffic in [(s,True) for s in BLIND]+[(s,False) for s in BLIND[:2]]:
                lab.run(f"validated_{s}_{'traffic' if traffic else 'clear'}",CORRIDOR,s,traffic,port,model=model,capture=.15)

if __name__=="__main__":
    main()
