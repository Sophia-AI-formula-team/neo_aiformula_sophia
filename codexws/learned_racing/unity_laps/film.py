"""Record a fresh Unity camera replay of a previously measured ideal-observation trial."""
import argparse
import json
from run import HERE,SIM,Lab

p=argparse.ArgumentParser();p.add_argument("source");p.add_argument("name");p.add_argument("--capture",type=float,default=.16)
args=p.parse_args();root=HERE/"evidence";original=json.loads((root/"runs"/(args.source+".json")).read_bytes())
q=original["request"]
if q["vision"]:raise ValueError("Use camera_runs.py for an RGB policy")
lab=Lab(SIM,root,q["solver"]);r=lab.run(args.name,q["weights"],q["seed"],bool(q["obstacles"]),media=True,capture=args.capture)
lab.event("film_comparison",source=original["id"],trial=r["id"],sameStatus=r["status"]==original["status"],
          secondsDelta=r["seconds"]-original["seconds"],progressDelta=r["progress"]-original["progress"])
