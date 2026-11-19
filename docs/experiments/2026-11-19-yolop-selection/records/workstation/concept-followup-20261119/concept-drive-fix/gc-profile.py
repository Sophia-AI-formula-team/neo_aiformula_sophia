"""Measure CPython GC pauses during steady-state frozen driving with Torch imported."""
import gc, json, sys, time
from pathlib import Path

import cv2, numpy as np, torch, torchvision
from concept_drive.core import Drive
from concept_drive.transport import native_sensor
from concept_road import _markings

root=Path.home()/'aiformula_runtime/concept-field-20261119/deployed-src/vehicle_concept/src/concept_drive/test/data/moving-lap-start'
ref=json.loads((root/'expected.json').read_text())
fixture=[]
for f in ref:
    p=root/('%05d'%f['frame'])
    bgr=cv2.imread(str(p)+'_rgb.png')
    with np.load(str(p)+'_perception.npz') as arr:
        fixture.append((bgr,{k:arr[k] for k in arr.files},f['sensor']))
cv2.setNumThreads(1);cv2.cvtColor(np.zeros((1,1,3),np.uint8),cv2.COLOR_BGR2LAB)
driver=Drive(4.)
if len(sys.argv)>1 and sys.argv[1]=='freeze':gc.freeze()
gen_start={};current=[];events=[];duration=[]
def collect(phase,info):
    gen=info['generation']
    if phase=='start':gen_start[gen]=time.perf_counter()
    else:
        dt=1000*(time.perf_counter()-gen_start.pop(gen))
        events.append([gen,dt,info['collected']]);current.append(dt)
gc.callbacks.append(collect)
for i in range(64):
    bgr,arr,s=fixture[i%len(fixture)]
    sensor=native_sensor(dict(s,gyro=-s['gyro']),i*.2)
    current.clear();t=time.perf_counter()
    out=driver.step(bgr,arr['raw'],arr['lane'],arr,sensor)
    json.dumps(out,allow_nan=False,separators=(',',':'))
    duration.append([i,(time.perf_counter()-t)*1000,sum(current)])
gc.callbacks.remove(collect)
print(json.dumps(dict(count=len(duration),times_ms=duration,gc2=[r for r in events if r[0]==2],worst=sorted(duration,key=lambda r:-r[1])[:10]),indent=2))
