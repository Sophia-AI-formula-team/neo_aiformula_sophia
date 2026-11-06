"""Inventory distinct physical trials and keep development, replays and blind arms separate."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

from smooth_trials import ROOT,write

def classify(name):
    patterns=[(r"tr\d{3}_a[0-6]","training"),(r"(dev|ck32|ck64|ck80|ck80p|ck80m|ck64p)_d(c|l)?\d+_(base|learned)","development"),
              (r"[bl]\d{2}_(base|learned)","blind"),(r"film_tr\d{3}_a[0-6]","filmedTrainingReplay"),
              (r"(filmdev|filmck32)_d\d{3}_(base|learned)","filmedDevelopmentReplay"),(r"[bl]\d{2}_fixed","additionalFixedArm")]
    for pattern,label in patterns:
        if re.fullmatch(pattern,name):return label
    if name=="fixed_zero_reference_check":return "fixedActionEquivalence"
    if name in ("default_v2_parity","parity_old_current_no_film","parity_new_matched_film"):return "solverParity"
    raise ValueError("Unclassified trial "+name)

def main():
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path,default=ROOT);a=p.parse_args();root=a.output
    raw=root/"runs";events=[json.loads(l)for l in(root/"events.jsonl").read_text().splitlines()]
    starts=Counter(e["trial"]for e in events if e["kind"]=="start")
    finishes=Counter(e["trial"]for e in events if e["kind"]=="finish")
    reports={};groups=Counter();outcomes={};links={}
    for path in sorted(raw.glob("*.json")):
        name=path.stem;kind=classify(name);groups[kind]+=1
        if name not in finishes or starts[name]!=1 or finishes[name]!=1:raise AssertionError(name+" incomplete event history")
        content=path.read_bytes();r=json.loads(content)
        outcomes.setdefault(kind,Counter())[r["status"]]+=1
        reports[name]=dict(group=kind,sha256=hashlib.sha256(content).hexdigest(),status=r["status"],seconds=r["seconds"],
                          progressM=r["progress"],roadM=r["minRoad"],coneM=r["minGap"]if r["props"] else None)
        if kind=="blind" and r["props"] and name.endswith("_base"):
            links[name.removesuffix("_base")+"_fixed"]=name
    if set(reports)!=set(finishes):raise AssertionError("Event ledger contains a missing trial")
    expected={"training":560,"development":133,"blind":64,"filmedTrainingReplay":4,"filmedDevelopmentReplay":3,
              "fixedActionEquivalence":1,"solverParity":3}
    assert dict(groups)==expected and len(links)==26  # 20 local obstacle layouts plus six mixed obstacle laps
    local_links={k:v for k,v in links.items() if k.startswith("b")}
    assert len(local_links)==20
    write(root/"campaign_receipt.json",dict(groups=dict(groups),outcomes={k:dict(z) for k,z in outcomes.items()},
          totalNewUnityRuns=len(reports),frozen=json.loads((root/"frozen.json").read_bytes())["modelFileSHA256"],
          reusedFixedArmReferences=local_links,additionalFixedArmRuns=0,
          recoveredOutputs=[e["trial"]for e in events if e["kind"]=="resume_finished_output"],rawTrials=reports))
if __name__=="__main__":main()
