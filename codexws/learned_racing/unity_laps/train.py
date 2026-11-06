"""Cross-entropy policy search. Unity scores every sampled policy through a full trial."""
import json
from pathlib import Path
import numpy as np

from run import BASE,HERE,Lab,NAMES,SIM,write

LOW=np.array([1.4,1.0,3.5,2.0,.05,2.3,.10,-5.,-5.])
HIGH=np.array([5.7,15.,11.,4.8,.72,7.,1.8,5.,5.])
TRAIN_SEEDS=(41021,5948)
TEST_SEEDS=(7116,9227,3019)


def ranking(run):
    complete=run["status"]=="lap"
    return (complete, -run["seconds"] if complete else run["progress"]/run["length"])


def summarize(run,stage,generation,candidate):
    return dict(trial=run["id"],stage=stage,generation=generation,candidate=candidate,
                status=run["status"],seconds=run["seconds"],progress=run["progress"],length=run["length"],
                seed=run["request"]["seed"],weights=run["request"]["weights"],minRoad=run["minRoad"],
                minGap=run["minGap"],zoneStart=run["zoneStart"],zoneEnd=run["zoneEnd"])


def main():
    lab=Lab(SIM,HERE/"evidence");rng=np.random.default_rng(110616)
    record=[];report=lab.root/"training.json"
    def log(run,stage,g,c):
        record.append(summarize(run,stage,g,c));write(report,record);return run
    base=log(lab.run("learn_base",BASE,TRAIN_SEEDS[0]),"clean",0,0)
    mean=(HIGH+LOW)*.5;spread=(HIGH-LOW)*.32
    anchor=base
    for generation in (1,2):
        candidates=[anchor["request"]["weights"],*rng.normal(mean,spread,size=(6,len(mean)))]
        group=[]
        for c,p in enumerate(candidates):
            w=np.clip(p,LOW,HIGH)
            trial=lab.run(f"learn_c{generation}_{c}",w,TRAIN_SEEDS[0])
            group.append(log(trial,"clean",generation,c))
        group.sort(key=ranking,reverse=True);anchor=group[0]
        elite=np.array([r["request"]["weights"] for r in group[:3]])
        mean=elite.mean(0);spread=np.maximum(elite.std(0), (HIGH-LOW)*.12)
    write(lab.root/"clean_policy.json",dict(weights=anchor["request"]["weights"],source=anchor["id"],fields=NAMES))
    # Start obstacle practice from the clean policy. Rank failure by position and completion by time.
    # Validate the finalists on a second practice layout before freezing the weights.
    first=log(lab.run("learn_obase",BASE,TRAIN_SEEDS[0],True),"obstacles",0,0)
    mean=np.array(anchor["request"]["weights"]);spread=(HIGH-LOW)*np.array([.22,.13,.13,.20,.38,.32,.33,.19,.19])
    pool=[]
    for generation in (1,2):
        incumbent=mean if not pool else max(pool,key=ranking)["request"]["weights"]
        group=[]
        for c,p in enumerate([incumbent,*rng.normal(mean,spread,size=(8,len(mean)))]):
            w=np.clip(p,LOW,HIGH)
            trial=lab.run(f"learn_o{generation}_{c}",w,TRAIN_SEEDS[0],True)
            group.append(log(trial,"obstacles",generation,c))
        group.sort(key=ranking,reverse=True);pool.extend(group)
        elite=np.array([r["request"]["weights"] for r in group[:3]])
        mean=elite.mean(0);spread=np.maximum(elite.std(0),(HIGH-LOW)*.13)
    unique={tuple(r["request"]["weights"]):r for r in [first,*pool]}
    finalists=sorted(unique.values(),key=ranking,reverse=True)[:5]
    checked=[]
    for i,original in enumerate(finalists):
        w=original["request"]["weights"]
        second=log(lab.run("learn_cross_"+str(i),w,TRAIN_SEEDS[1],True),"cross",3,i)
        pair=(original,second)
        score=(sum(r["status"]=="lap" for r in pair),min(r["progress"]/r["length"] for r in pair),-sum(r["seconds"] for r in pair))
        # A sub-tick finish distance must not outrank elapsed time.
        score=(score[0],min(1.,score[1]),score[2]);checked.append((score,original,second))
    _,winner,repeat=max(checked,key=lambda x:x[0]);frozen=winner["request"]["weights"]
    write(lab.root/"policy.json",dict(weights=frozen,fields=NAMES,source=winner["id"],cross=repeat["id"],
                                       trainingSeeds=TRAIN_SEEDS,holdoutSeeds=TEST_SEEDS))
    for seed in TEST_SEEDS:
        for layout in ("clear","traffic"):
            for label,w in (("initial",BASE),("learned",frozen)):
                run=lab.run(f"test_{seed}_{layout}_{label}",w,seed,layout=="traffic")
                log(run,"test",-1,label)
if __name__=="__main__":main()
