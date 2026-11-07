"""Separate all-wheel-centers-out cohort; retain earlier stricter cohort intact."""
import subprocess
import time
from acceptance_run import HERE,SIM,ROOT,read,save,sha,BASE,Lab


def main():
    plan=read(ROOT/'plan.json');freeze=read(ROOT/'freeze.json')
    assert read(ROOT/'progress.json')['status']=='complete', 'Strict cohort must finish first'
    for f,h in freeze['sources'].items():
        if f=='UnityRace.cs':
            assert sha(ROOT/'frozen/strict_referee'/f)==h
        else:assert sha(HERE/f)==h, f
    for v,h in freeze['models'].items():assert sha(ROOT/'frozen'/(v+'.json'))==h
    root=ROOT/'race_rule';root.mkdir(exist_ok=True)
    manifest=dict(rule='all_wheel_centers',actual_wheel_count=3,
                  wheel_locations='frontLeft and frontRight WheelCollider transforms; rearWheelBody.position',
                  scope='All three installed virtual wheel centers must be outside the road; contact patch width is not modeled. Same frozen models and original 16 layouts.',
                  reason='Earlier body-edge stop is stricter than the user rule. Separate additional cohort, not overwritten evidence.',
                  changed_sources={f:sha(HERE/f)for f in ['UnityRace.cs','run.py','acceptance_race.py']},
                  plan_sha256=sha(ROOT/'plan.json'),models=freeze['models'])
    if (root/'protocol.json').exists():assert read(root/'protocol.json')==manifest
    else:save(root/'protocol.json',manifest)
    lab=Lab(SIM,root);cfg=read(SIM/'Simulator/UserSettings/AIFormulaRoadDetector.json')
    for i,case in enumerate(plan['cases']):
        for version in plan['versions'][::1 if i%2==0 else -1]:
            name=case['id']+'_'+version
            if (root/'runs'/(name+'.json')).exists():continue
            folder=lab.work/name;folder.mkdir(parents=True,exist_ok=True);ready=folder/'ready.json'
            if ready.exists():raise RuntimeError('Incomplete attempt '+name)
            args=[cfg['python'],'-B',str(HERE/'v3_vision.py'),str(SIM/'Simulator/Assets/StreamingAssets/RoadDetector/a4'),str(ready),
                  '--model',str(ROOT/'frozen'/(version+'.json')),'--audit']
            with (folder/'server.log').open('w',encoding='utf8') as log:
                proc=subprocess.Popen(args,stdout=log,stderr=log,creationflags=subprocess.CREATE_NO_WINDOW)
                save(root/'progress.json',dict(status='running',case=case['id'],version=version,total=32))
                try:
                    begin=time.monotonic()
                    while not ready.exists():
                        if proc.poll() is not None or time.monotonic()-begin>100:raise RuntimeError('Vision startup failed '+name)
                        time.sleep(.2)
                    meta=read(ready)
                    lab.run(name,BASE,case['seed'],vision=meta['port'],media=True,limit=case['limitM']/plan['course_length_m'],capture=.2,
                            model=meta['version']+'/A4/'+meta['weights']+'/'+meta['controlSha'],external_control=True,
                            obstacle_specs=[[p[0]/plan['course_length_m'],*p[1:]]for p in case['props']],departure_rule='all_wheel_centers')
                finally:
                    if proc.poll() is None:proc.terminate()
                    proc.wait(timeout=10)
    save(root/'progress.json',dict(status='complete',runs=32))


if __name__=='__main__':main()
