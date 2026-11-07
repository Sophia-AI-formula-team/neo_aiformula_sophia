"""Same-frame causal replay; report support as well as temporal residuals."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from v3_vision import VisibleRoad
from v33_road import EvidenceRoad
from v33_trials import ROOT,save,read

def residual(previous,current,speed,yaw,dt):
    if previous is None or len(previous[0])<3 or len(current[0])<3:return [],0
    pr,pp=previous;road,pixels=current
    vals=[];eligible=0;angle=yaw*dt;c,s=np.cos(angle),np.sin(angle)
    for side,col,px in ((-1,1,0),(1,2,1)):
        old_ok=np.array([p[px]>3 if side==-1 else p[px]<636 for p in pp])
        z=pr[old_ok,0]-speed*dt;x=pr[old_ok,col]
        zz=c*z+s*x;xx=-s*z+c*x
        order=np.argsort(zz);zz=zz[order];xx=xx[order]
        if len(zz)<2:continue
        for depth in np.linspace(1.2,3.,7):
            row=np.array([depth,*[np.interp(depth,road[:,0],road[:,k]) for k in range(1,4)]])
            p=pixels[int(np.argmin(abs(road[:,0]-depth)))]
            observed=p[px]>3 if side==-1 else p[px]<636
            if not (observed and 1<=row[0]<=3 and zz[0]<=row[0]<=zz[-1]):continue
            eligible+=1;vals.append(abs(float(row[col]-np.interp(row[0],zz,xx))))
    return vals,eligible

def main():
    p=argparse.ArgumentParser();p.add_argument('experiment');p.add_argument('--version',default='V3.3.4');a=p.parse_args()
    root=ROOT/a.experiment;reports=[]
    for path in sorted((root/'runs').glob('*.json')):
        run=read(path);models=[VisibleRoad(),EvidenceRoad(False),EvidenceRoad(True)]
        prior=[None]*3;errors=[[] for _ in models];counts=[0]*3;visible=[0]*3;frames=0;paired=[[] for _ in models]
        for i,d in enumerate(run['decisions']):
            sensor=d['decision']['sensor'];speed=d['decision']['speedEstimate'];dt=.2 if i==0 else d['t']-run['decisions'][i-1]['t']
            stamps=np.asarray(run['frames'])[:,0];frame=int(np.argmin(abs(stamps-d['t'])))
            image=cv2.imread(str(root/'.work/frames'/run['id']/f'{frame:04d}f.jpg'))
            mask=cv2.imread(str(root/'.work'/run['id']/'frames'/f'{i:05d}_mask.png'),0)
            if image is None or mask is None:raise RuntimeError('Missing original replay input')
            this=[]
            for k,m in enumerate(models):
                out=m.read(image,mask) if k==0 else m.read(image,mask,speed,sensor['gyro'],dt)
                road,pixels=out[:2]
                er,n=residual(prior[k],(road,pixels),speed,sensor['gyro'],dt)
                errors[k]+=er;counts[k]+=n
                this.append(er)
                visible[k]+=sum(1 for row,p in zip(road,pixels) if 1<=row[0]<=3 and (p[0]>3 or p[1]<636))
                prior[k]=(road,pixels)
            if len({len(e) for e in this})==1:
                for k,e in enumerate(this):paired[k]+=e
            frames+=1
        stats=[]
        for k,name in enumerate(['V3.0',a.version+' spatial',a.version+' temporal']):
            e=errors[k]
            stats.append(dict(version=name,p50_m=float(np.median(e)) if e else None,p95_m=float(np.quantile(e,.95)) if e else None,
                              p99_m=float(np.quantile(e,.99)) if e else None,max_m=max(e) if e else None,
                              compared_edges=len(e),near_visible_rows=visible[k],frames=frames,
                              equal_count_frames_p95_m=float(np.quantile(paired[k],.95)) if paired[k] else None,
                              equal_count_comparisons=len(paired[k])))
        reports.append(dict(run=run['id'],methods=stats))
    save(root/('perception_replay_'+a.version+'.json'),dict(results=reports,limits='Camera-derived correspondence and planar metric; not ground-truth road accuracy. All methods use the same 7 depths from 1.2 to 3m and replay the identical JPEG RGB and saved mask; original inference used PNG RGB. Counts and support differ and are reported. Equal-count subset is diagnostic, not guaranteed same physical-edge identity or a qualification pass.'))
    print(json.dumps(reports,ensure_ascii=False),flush=True)
if __name__=='__main__':main()
