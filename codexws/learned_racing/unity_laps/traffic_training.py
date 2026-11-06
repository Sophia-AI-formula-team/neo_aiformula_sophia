"""Second curriculum: optimise both training layouts together, then freeze for new seeds."""
import numpy as np
from train import LOW,HIGH,summarize
from run import BASE,HERE,Lab,NAMES,SIM,write

PRACTICE=(41021,5948)
HOLDOUT=(48133,81640,25004)


def main():
    lab=Lab(SIM,HERE/"evidence");rng=np.random.default_rng(61342);report=[];pool=[]
    centered=np.array([3.4,7.,7.,3.4,.37,4.7,1.,.3,-.8])
    spread=(HIGH-LOW)*.24;mean=centered.copy()
    def pair(weights,gen,index):
        results=[]
        for k,seed in enumerate(PRACTICE):
            trial=lab.run(f"joint_g{gen}_{index}_s{k}",weights,seed,True)
            report.append(summarize(trial,"joint",gen,index));write(lab.root/"joint_training.json",report);results.append(trial)
        passed=sum(r["status"]=="lap" for r in results)
        fractions=[min(1.,r["progress"]/r["length"]) for r in results]
        score=(passed,min(fractions),sum(fractions),-sum(r["seconds"] for r in results))
        return score,results[0]["request"]["weights"],[r["id"] for r in results]
    pool.append(pair(BASE,0,0))
    for generation in (1,2,3):
        samples=[centered if generation==1 else max(pool,key=lambda x:x[0])[1]]
        samples.extend(rng.normal(mean,spread,size=(7,len(BASE))))
        group=[pair(np.clip(p,LOW,HIGH),generation,c) for c,p in enumerate(samples)]
        pool.extend(group);elite=np.array([item[1] for item in sorted(pool,key=lambda x:x[0],reverse=True)[:4]])
        mean=elite.mean(0);spread=np.maximum(elite.std(0),(HIGH-LOW)*.13)
    score,w,source=max(pool,key=lambda x:x[0])
    write(lab.root/"joint_policy.json",dict(weights=w,fields=NAMES,source=source[0],cross=source[1],
                                             trainingSeeds=PRACTICE,holdoutSeeds=HOLDOUT,trainingLaps=score[0]))
    for seed in HOLDOUT:
        for layout in ("clear","traffic"):
            for label,weights in (("initial",BASE),("learned",w)):
                r=lab.run(f"joint_test_{seed}_{layout}_{label}",weights,seed,layout=="traffic")
                report.append(summarize(r,"joint_test",-1,label));write(lab.root/"joint_training.json",report)
if __name__=="__main__":main()
