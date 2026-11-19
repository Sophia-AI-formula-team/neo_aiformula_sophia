"""Fit forward yaw time constant and bounded gains to manual episodes; compare held-out signed steps."""
import gzip,json,math,statistics as st
from pathlib import Path
base=Path(__file__).parent
rows=[json.loads(l) for l in gzip.open(base/'control.jsonl.gz')]
# Only intervals that have held a single nonzero drive command and steering for >=0.35s.
segments=json.loads((base/'segments.json').read_text())
train=[];test=[]
for s in segments:
 mode,v,w=s['k'];n=len(s['gyro']);duration=s['t1']-s['t0']
 if mode!='manual' or v<=0 or duration<.35 or n<10:continue
 if abs(w)<.03:continue
 c={'span':round(duration,3),'speed':round(v,3),'yaw_cmd':round(w,3),'first':round(s['gyro'][0],3),'late':round(st.median(s['gyro'][-max(3,n//3):]),3)}
 (train if s['t0']<60 else test).append(c)
print(json.dumps(dict(training=train,holdout=test),indent=2))
