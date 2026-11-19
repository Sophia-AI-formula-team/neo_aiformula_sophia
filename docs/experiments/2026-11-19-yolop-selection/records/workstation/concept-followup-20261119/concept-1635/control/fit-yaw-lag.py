"""Fit gain-fixed body-yaw lag to sustained forward manual inputs; hold out final time block."""
import gzip,json
from pathlib import Path
import numpy as np
data=Path(__file__).parent
r=[json.loads(s) for s in gzip.open(data/'control.jsonl.gz')]
ts=lambda k:np.array([t/1e9 for t,key,*v in r if key==k])
tm=ts('m');m=np.array([v for _,k,*v in r if k=='m'])
tg=ts('g');modes=np.array([v[0] for _,k,*v in r if k=='g'])
tw=np.array([t/1e9 for t,k,l,r,w,valid in (row for row in r if row[1]=='w') if valid])
yaw=np.array([w for t,k,l,r,w,valid in (row for row in r if row[1]=='w') if valid])
grid=np.arange(tm[0]+.3,tm[-1],.05)
i=np.searchsorted(tm,grid,'right')-1;v,u=m[i].T
g=modes[np.searchsorted(tg,grid,'right')-1]
ys=np.interp(grid,tw,yaw)
# Start at measured gyro; predict through full one-second command snippets.
windows=[j for j in range(0,len(grid)-12,4) if all(g[j:j+13]=='manual') and min(v[j:j+13])>0.2 and max(abs(u[j:j+13]))>.15]
res=[]
for tau in np.arange(.05,1.01,.025):
 a=np.exp(-.05/tau);err=[[],[]]
 for j in windows:
  w=ys[j]
  for t in range(j+1,j+13):
   w=a*w+(1-a)*u[t-1];err[int(grid[j]-tm[0]>32)].append(w-ys[t])
 res.append((tau,*[np.sqrt(np.mean(np.square(e))) for e in err]))
old=min(res,key=lambda a:abs(a[0]-.103));best=min(res,key=lambda a:a[1])
print(json.dumps(dict(training_windows=int(sum(grid[j]-tm[0]<=32 for j in windows)),test_windows=int(sum(grid[j]-tm[0]>32 for j in windows)),unity_tau_identity_yaw_rmse=old,field_tau_identity_yaw_rmse=best),indent=2))
