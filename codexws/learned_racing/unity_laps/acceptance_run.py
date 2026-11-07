"""Pre-register and run paired held-out V3 Unity acceptance; never tune weights."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import numpy as np
from run import HERE, SIM, Lab, BASE

ROOT = HERE / 'v3_acceptance'
TRAIN = HERE.parents[3] / 'lap_driving_v3_wt/codexws/learned_racing/unity_laps/.work/v3_2_long_30000_20261107'


def read(p): return json.loads(p.read_bytes())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p, data):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf8')


def prepare():
    from scenario_shots import geometry
    from v3_model import DEFAULT_PARAMETERS
    from acceptance_audit import parity
    if (ROOT/'plan.json').exists():
        raise FileExistsError('Plan is immutable. Resume with run; do not regenerate after seeing outcomes.')
    model = read(TRAIN/'model.json')
    assert model['trained_attempts'] == 30000 and model['generation'] == 118
    assert read(TRAIN/'status.json')['status'] == 'complete'
    for rel, digest in model['sources'].items():
        assert sha(TRAIN/'sources'/rel) == digest, rel
    assert sha(HERE/'v3_model.py') == model['sources']['v3_model.py']
    frozen=ROOT/'frozen'; frozen.mkdir(parents=True,exist_ok=True)
    shutil.copy2(TRAIN/'model.json', frozen/'V3_2.json')
    save(frozen/'V3_0.json',dict(version='V3.0',parameters=DEFAULT_PARAMETERS,trained_attempts=0))
    for filename in ('history.json','status.json'):
        shutil.copy2(TRAIN/filename, frozen/filename)
    seen=set(); stats=dict(attempts=0,completed=0,contact=0,departure=0)
    with (TRAIN/'attempts.jsonl').open(encoding='utf8') as stream:
        for line in stream:
            r=json.loads(line);stats['attempts']+=1
            # Journal outcome includes the scenario seed and status.
            seen.add(r.get('seed'))
            for key in ('completed','contact','departure'):stats[key]+=bool(r.get(key,False))
    assert stats['attempts']==30000
    save(ROOT/'instrumentation_parity.json',parity(model['parameters']))
    _,stations,_,curvature,length=geometry()
    rng=np.random.default_rng(20261108118)
    def seed_for(curved=None, distance=18., straight_length=0.):
        for _ in range(100000):
            seed=int(rng.integers(900000000,980000000))
            if seed in seen:continue
            start=np.random.default_rng(seed).random()*length
            idx=np.searchsorted(stations,(start+distance)%length).clip(0,len(stations)-1)
            if curved is not None and (abs(curvature[idx])>.025)!=curved:continue
            if straight_length:
                inds=np.searchsorted(stations,(start+np.arange(1,straight_length))%length).clip(0,len(stations)-1)
                if np.max(abs(curvature[inds]))>.018:continue
            return seed
        raise RuntimeError('No qualifying held-out spawn')
    cases=[]
    def add(id_,title,kind,props=None,limit=40,curved=None,straight=0,full=False):
        cases.append(dict(id=id_,title=title,kind=kind,seed=seed_for(curved,18,straight),
                          props=props or [],limitM=length if full else limit,full_lap=full,
                          geometry_selected_only_by_referee=True))
    add('01','随机起点：空场完整一圈','full_clear',full=True)
    add('02','另一随机起点：空场完整一圈','full_clear',full=True)
    add('03','动静混合：完整一圈','full_traffic',[[22,0,0,0,0],[88,-.4,0,0,0],[92,.5,1.1,.7,.6],[196,-.3,1.2,.55,2.1]],full=True)
    add('04','无障碍直路：实际提速与摆动','cruise',limit=38,straight=36)
    for repeat in range(2):
        dist=18.+repeat*2
        add(f'{5+repeat*3:02}','A：两桶中缝可通行','gap_A',[[dist,-1.8,0,0,0],[dist,.7,0,0,0]],limit=dist+12,curved=False)
        add(f'{6+repeat*3:02}','B：中缝太窄，必须绕行','gap_B',[[dist,-.9,0,0,0],[dist,.3,0,0,0]],limit=dist+12,curved=False)
        add(f'{7+repeat*3:02}','C：可钻缝也可绕行，自选路线','gap_C',[[dist,-2.35,0,0,0],[dist,-.05,0,0,0]],limit=dist+12,curved=False)
    add('11','弯中静止障碍','bend_static',[[18,0,0,0,0]],limit=32,curved=True)
    add('12','连续双桶空隙：衔接下一个入口','sequence',[[14,-1.8,0,0,0],[14,.7,0,0,0],[23,-3.0,0,0,0],[23,-.5,0,0,0],[31,-1.8,0,0,0],[31,.7,0,0,0]],limit=42,curved=False)
    add('13','弯中连续避障','bend_sequence',[[15,.3,0,0,0],[21,-.8,0,0,0],[27,.3,0,0,0]],limit=38,curved=True)
    add('14','移动桶横穿：相位一','moving',[[18,-.5,1.25,.7,.1]],limit=32,curved=False)
    add('15','移动桶横穿：相位二','moving',[[18,-.5,1.25,.7,2.4]],limit=32,curved=False)
    add('16','弯中动静混合','bend_mixed',[[17,.25,0,0,0],[22,-.4,1.2,.65,1.4]],limit=34,curved=True)
    plan=dict(created_utc=datetime.now(timezone.utc).isoformat(),base='bf63f2044af1addebf5acf65972a4f1164f6283c',
              versions=['V3_0','V3_2'],course_length_m=length,cases=cases,
              criteria=dict(completion='full lap or complete declared local segment without contact or body-edge departure',
                            contact='1.16 m x 0.76 m body rectangle intersects cone circle radius 0.34 m',
                            departure='any body corner beyond two-lane road: stricter than competition four wheels out',
                            comparison='same seed and layout; paired elapsed times only if both complete; otherwise completion first',
                            first_lap='camera RGB, A4 lane/object output, motor RPM proxy, raw gyro; no map or global pose',
                            risk='uncertainty margin, NOT calibrated probability',
                            limits='static geometry and ideal virtual sensors; not real-car acceptance'),
              training=stats)
    save(ROOT/'plan.json',plan)
    files=['v3_model.py','v3_vision.py','acceptance_audit.py','acceptance_run.py','v3_evidence/plant_fit.json','UnityRace.cs','Road.cs']
    save(ROOT/'freeze.json',dict(sources={f:sha(HERE/f) for f in files},models={v:sha(frozen/(v+'.json'))for v in plan['versions']},
                                 plan_sha256=sha(ROOT/'plan.json'),training_journal_sha256=sha(TRAIN/'attempts.jsonl')))
    print(json.dumps(dict(cases=len(cases),runs=len(cases)*2,parity=read(ROOT/'instrumentation_parity.json'))))


def run():
    frozen=read(ROOT/'freeze.json');plan=read(ROOT/'plan.json')
    for f,h in frozen['sources'].items():assert sha(HERE/f)==h, f
    assert sha(ROOT/'plan.json')==frozen['plan_sha256']
    for v,h in frozen['models'].items():assert sha(ROOT/'frozen'/(v+'.json'))==h
    if not os.environ.get('AIFORMULA_LAP_COPY'):raise RuntimeError('Dedicated Unity copy required')
    lab=Lab(SIM,ROOT);cfg=read(SIM/'Simulator/UserSettings/AIFormulaRoadDetector.json')
    for number,case in enumerate(plan['cases']):
        versions=plan['versions'][::1 if number%2==0 else -1]
        for version in versions:
            name=case['id']+'_'+version
            if (ROOT/'runs'/(name+'.json')).exists():continue
            folder=lab.work/name;folder.mkdir(parents=True,exist_ok=True);ready=folder/'ready.json'
            if ready.exists():raise RuntimeError('Incomplete existing run: '+name)
            args=[cfg['python'],'-B',str(HERE/'v3_vision.py'),str(SIM/'Simulator/Assets/StreamingAssets/RoadDetector/a4'),str(ready),
                  '--model',str(ROOT/'frozen'/(version+'.json')),'--audit']
            with (folder/'server.log').open('w',encoding='utf8') as log:
                proc=subprocess.Popen(args,stdout=log,stderr=log,creationflags=subprocess.CREATE_NO_WINDOW)
                save(ROOT/'progress.json',dict(status='running',case=case['id'],version=version,total=32))
                try:
                    began=time.monotonic()
                    while not ready.exists():
                        if proc.poll() is not None or time.monotonic()-began>100:raise RuntimeError('Vision startup failed '+name)
                        time.sleep(.2)
                    meta=read(ready)
                    result=lab.run(name,BASE,case['seed'],vision=meta['port'],media=True,
                        limit=case['limitM']/plan['course_length_m'],capture=.2,
                        model=meta['version']+'/A4/'+meta['weights']+'/'+meta['controlSha'],external_control=True,
                        obstacle_specs=[[p[0]/plan['course_length_m'],*p[1:]]for p in case['props']])
                    assert abs(result['length']-plan['course_length_m'])<.1
                finally:
                    if proc.poll() is None:proc.terminate()
                    proc.wait(timeout=10)
    save(ROOT/'progress.json',dict(status='complete',runs=32))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','run']);args=p.parse_args()
    prepare() if args.action=='prepare' else run()
