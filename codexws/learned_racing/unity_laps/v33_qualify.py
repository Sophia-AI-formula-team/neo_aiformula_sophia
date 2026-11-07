"""Preregistered fresh starts; stop on a failure instead of spending a long run."""
import argparse
import hashlib
import subprocess
import sys
import numpy as np
from v33_trials import HERE,ROOT,read,save

def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','run']);p.add_argument('--stage',choices=['clear','disturbance','obstacles'],default='clear');p.add_argument('--name',default='V3_3_4');p.add_argument('--seed',type=int,default=33420261108);a=p.parse_args()
    plan_path=ROOT/('qualification_'+a.name+'.json')
    files=['v33_road.py','v33_controller.py','v33_audit.py','v33_vision.py','UnityRace.cs','run.py']
    hashes={f:hashlib.sha256((HERE/f).read_bytes()).hexdigest() for f in files}
    if a.action=='prepare':
        if plan_path.exists():raise FileExistsError('Never overwrite qualification plan')
        rng=np.random.default_rng(a.seed)
        seeds=rng.choice(np.arange(3300200000,3300300000),28,replace=False).tolist()
        save(plan_path,dict(sources=hashes,version=a.name.replace('_','.'),pace=2.8,yaw_limit=1.55,
             clear=seeds[:8],disturbance=seeds[8:12],obstacles=[dict(case=c,seed=s) for c,s in zip(['05','06','07','11','12','13','14','16']*2,seeds[12:])],
             stop='First failure aborts the cohort; untouched seeds do not count as passed. Training remains disabled.'))
        return
    plan=read(plan_path)
    if hashes!=plan['sources']:raise RuntimeError('Source changed after qualification freeze')
    outcomes=[]
    for i,s in enumerate(plan[a.stage]):
        if {f:hashlib.sha256((HERE/f).read_bytes()).hexdigest() for f in files}!=plan['sources']:raise RuntimeError('Source changed during qualification')
        seed=s['seed'] if isinstance(s,dict) else s;case=s['case'] if isinstance(s,dict) else '01'
        name=f'qualification_{a.name}_{a.stage}_{i+1:02d}'
        cmd=[sys.executable,'-B',str(HERE/'v33_trials.py'),name,'--cases',case,'--seed',str(seed),'--pace',str(plan['pace']),'--version',plan['version']]
        if a.stage!='obstacles':cmd+=['--full']
        if a.stage=='disturbance':cmd+=['--disturbance']
        subprocess.run(cmd,check=True)
        r=read(ROOT/name/'summary.json')[0];outcomes.append(dict(seed=seed,**r))
        passed=r['status'] in ('lap','segment') and (a.stage=='obstacles' or r['progress']/r['seconds']>=2.)
        save(ROOT/(a.name+'_'+a.stage+'_qualification.json'),dict(stage=a.stage,outcomes=outcomes,total=len(plan[a.stage]),all_passed=passed and len(outcomes)==len(plan[a.stage]),training_started=False))
        if not passed:print('GATE FAILED; remaining runs and training not started',flush=True);return
    print('STAGE PASSED: '+a.stage,flush=True)
if __name__=='__main__':main()
