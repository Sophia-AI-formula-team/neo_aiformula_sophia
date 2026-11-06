"""Compare the same frozen controller at equal capture settings in the old and new solver."""
from collections import Counter
import json
import numpy as np
from smooth_trials import FROZEN,HERE,Lab,SIM,V2,detector,write

def report(a,b):
    assert len(a["decisions"])==len(b["decisions"])
    fields=Counter();max_xyzv=np.zeros(4);different_track_count=0
    for u,v in zip(a["decisions"],b["decisions"]):
        for key in u:
            if key!="smooth" and u[key]!=v[key]:fields[key]+=1
        if len(u["tracks"])!=len(v["tracks"]):different_track_count+=1
        elif u["tracks"]:max_xyzv=np.maximum(max_xyzv,np.max(abs(np.array(u["tracks"])-np.array(v["tracks"])),axis=0))
    commands=("speedCmd","yawCmd","leftCan","rightCan")
    return dict(original=a["id"],replay=b["id"],captureMatches=a["request"]["media"]==b["request"]["media"],
                physicsRows=len(a["rows"]),physicsExact=a["rows"]==b["rows"],totalDecisions=len(a["decisions"]),
                finalCommandsExact=all(c not in fields for c in commands),observedLaneExact="observed"not in fields,
                allHistoricalDecisionFieldsExact=not fields,differences=dict(fields),trackCountDifferences=different_track_count,
                maxAbsPairedTrackDelta_z_x_vx_vz=max_xyzv.tolist(),sameGates=a["gate"]==b["gate"],sameOutcome=a["status"]==b["status"])

def check(root):
    archive=json.loads((HERE/"learned_risk_revision/runs/diag_full01_learned.json").read_bytes())
    req=archive["request"];new=Lab(SIM,root);old=Lab(SIM,root,req["solver"])
    def run(lab,id_,film):
        cached=lab.raw/(id_+".json")
        if cached.exists():return json.loads(cached.read_bytes())
        with detector(lab,"outer-edge",id_,film=film) as (port,release):
            return lab.run(id_,FROZEN["weights"],req["seed"],vision=port,limit=req["limit"],media=film,
                           capture=req["captureInterval"],model=release,obstacle_specs=req["obstacles"],timing=req["timing"],encounter=V2)
    initial=run(new,"default_v2_parity",False)
    old_current=run(old,"parity_old_current_no_film",False)
    matched=run(new,"parity_new_matched_film",True)
    base=dict(reference=archive["id"],previousSolver=req["solver"],solver=new.build,
              archivedFilmVsInitialNoFilm=report(archive,initial),sameCurrentNoFilm=report(old_current,initial),
              sameArchivedAndCurrentFilm=report(archive,matched))
    base["entirePhysicsAndVisionAndCommandsExact"]=base["sameCurrentNoFilm"]["allHistoricalDecisionFieldsExact"] and base["sameCurrentNoFilm"]["physicsExact"]
    write(root/"parity.json",base)
    # Keep baseline.json describing the present solver after reconstructing an older DLL.
    Lab(SIM,root)
    assert base["entirePhysicsAndVisionAndCommandsExact"],base

if __name__=="__main__":
    from smooth_trials import ROOT
    check(ROOT)
