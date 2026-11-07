"""Small, versioned Unity trials. No automatic training before qualification."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
from run import HERE, SIM, Lab, BASE

ROOT=HERE/'v33_evidence'
PREVIOUS=HERE.parents[3]/'lap_v3_acceptance_20261108_wt/codexws/learned_racing/unity_laps/v3_acceptance'

def read(p):return json.loads(p.read_bytes())
def save(p,data):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--cases',default='01')
    p.add_argument('--pace',type=float);p.add_argument('--yaw-limit',type=float,default=1.55)
    p.add_argument('--spatial',action='store_true');p.add_argument('--disturbance',action='store_true')
    p.add_argument('--old-parameters',action='store_true');p.add_argument('--full',action='store_true')
    p.add_argument('--seed',type=int);p.add_argument('--original',action='store_true');p.add_argument('--version',default='V3.3.4')
    args=p.parse_args()
    if not os.environ.get('AIFORMULA_LAP_COPY'):raise RuntimeError('Private Unity required')
    root=ROOT/args.name
    if root.exists():raise FileExistsError('Each experiment has a new ID: '+args.name)
    root.mkdir(parents=True)
    plan=read(PREVIOUS/'plan.json');cfg=read(SIM/'Simulator/UserSettings/AIFormulaRoadDetector.json')
    sources=['v33_road.py','v33_controller.py','v33_vision.py','v33_audit.py','UnityRace.cs','run.py','v3_model.py','v3_evidence/plant_fit.json']
    digest={f:hashlib.sha256((HERE/f).read_bytes()).hexdigest() for f in sources}
    for f in sources:
        target=root/'.work/sources'/f;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes((HERE/f).read_bytes())
    save(root/'manifest.json',dict(arguments=vars(args),sources=digest,development=True,training_started=False))
    lab=Lab(SIM,root)
    cases=[c for c in plan['cases'] if c['id'] in args.cases.split(',')]
    results=[]
    for case in cases:
        name=case['id'];folder=lab.work/name;folder.mkdir(parents=True);ready=folder/'ready.json'
        command=[cfg['python'],'-B',str(HERE/'v33_vision.py'),str(SIM/'Simulator/Assets/StreamingAssets/RoadDetector/a4'),str(ready),'--audit','--yaw-limit',str(args.yaw_limit),'--version',args.version]
        if args.pace is not None:command+=['--pace',str(args.pace)]
        if args.spatial:command+=['--spatial']
        if args.disturbance:command+=['--disturbance']
        if args.old_parameters:command+=['--model',str(PREVIOUS/'frozen/V3_2.json')]
        if args.original:
            from v3_model import DEFAULT_PARAMETERS
            record=dict(version='V3.0',parameters={**DEFAULT_PARAMETERS,**({'pace':args.pace} if args.pace else {})})
            save(folder/'model.json',record)
            command=[cfg['python'],'-B',str(HERE/'v3_vision.py'),str(SIM/'Simulator/Assets/StreamingAssets/RoadDetector/a4'),str(ready),'--audit','--model',str(folder/'model.json')]
        with (folder/'server.log').open('w',encoding='utf-8') as log:
            proc=subprocess.Popen(command,stdout=log,stderr=log,creationflags=subprocess.CREATE_NO_WINDOW)
            try:
                start=time.monotonic()
                while not ready.exists():
                    if proc.poll() is not None or time.monotonic()-start>100:raise RuntimeError('Vision startup: '+str(folder/'server.log'))
                    time.sleep(.2)
                meta=read(ready)
                result=lab.run(name,BASE,args.seed if args.seed is not None else case['seed'],vision=meta['port'],media=True,
                    limit=1. if args.full else case['limitM']/plan['course_length_m'],capture=.2,
                    model=meta['version']+'/'+meta['controlSha']+'/'+hashlib.sha256(json.dumps(digest).encode()).hexdigest(),
                    external_control=True,departure_rule='all_wheel_centers',external_yaw_limit=args.yaw_limit if args.yaw_limit>1.55 else None,
                    obstacle_specs=[[o[0]/plan['course_length_m'],*o[1:]] for o in case['props']])
                row=dict(id=name,status=result['status'],seconds=result['seconds'],progress=result['progress'],length=result['length'],minGap=result['minGap'],minRoad=result['minRoad'])
                results.append(row);print(json.dumps(row),flush=True);save(root/'summary.json',results)
            finally:
                if proc.poll() is None:proc.terminate()
                proc.wait(timeout=10)
    save(root/'complete.json',dict(complete=True,count=len(results)))

if __name__=='__main__':main()
