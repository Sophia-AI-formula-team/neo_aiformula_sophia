"""Fit a small response model to archived Unity training traces, not holdouts."""
import hashlib
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
def main():
    files=sorted((HERE/"smooth_revision/runs").glob("*.json"))[:60]
    sets=[];sources=[]
    for p in files:
        data=json.loads(p.read_bytes());a=np.asarray(data["rows"],float)
        if len(a)<15:continue
        sets.append(a);sources.append(dict(file=str(p.relative_to(HERE)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    result={}
    for name,state,command in (("speed",4,5),("yaw",14,6)):
        fits=[]
        for lag in range(4):
            xx=[];yy=[]
            for a in sets[:45]:
                dt=np.diff(a[:,0]);valid=(dt>.08)&(dt<.12)&(a[:-1,4]>.4)
                idx=np.flatnonzero(valid);idx=idx[idx>=lag]
                xx.append(np.c_[a[idx-lag,command],a[idx,state],np.ones(len(idx))])
                yy.append((a[idx+1,state]-a[idx,state])/dt[idx])
            x=np.concatenate(xx);y=np.concatenate(yy)
            coeff=np.linalg.lstsq(x,y,rcond=None)[0]
            if coeff[0]<=0 or coeff[1]>=0:continue
            error=np.mean((x@coeff-y)**2)
            fits.append((error,lag,coeff))
        if not fits:raise ValueError("No stable fit for "+name)
        error,lag,b=min(fits,key=lambda x:x[0]);pred=[];actual=[]
        for a in sets[45:]:
            idx=np.arange(lag,len(a)-1);dt=a[idx+1,0]-a[idx,0]
            good=(dt>.08)&(dt<.12);idx=idx[good];dt=dt[good]
            x=np.c_[a[idx-lag,command],a[idx,state],np.ones(len(idx))]
            pred.extend(a[idx,state]+(x@b)*dt);actual.extend(a[idx+1,state])
        result[name]=dict(tau=float(-1/b[1]),gain=float(-b[0]/b[1]),bias=float(-b[2]/b[1]),delay=lag*.1,
                          fit_acceleration_rmse=float(np.sqrt(error)),heldout_step_rmse=float(np.sqrt(np.mean((np.array(pred)-actual)**2))))
    rows=np.concatenate(sets[:45]);bands=[]
    for speed in (1.,2.,3.,4.):
        mask=(abs(rows[:,6])>.3)&(rows[:,4]>=speed)&(rows[:,4]<speed+1)
        bands.append(float(np.clip(np.median(rows[mask,14]/rows[mask,6]),.005,1.)))
    result["yaw_gain_by_speed"]=dict(speed=[1.5,2.5,3.5,4.5],gain=bands,method="median observed yaw / command per speed band; descriptive fit")
    result.update(source="archived Unity training traces",fit_runs=45,validation_runs=len(sets)-45,sources=sources,
                  limitation="One-step linear lag fit only, not long-rollout equivalence or real-car calibration",body_speed_cap=4.5)
    dest=HERE/"v3_evidence";dest.mkdir(exist_ok=True)
    (dest/"plant_fit.json").write_text(json.dumps(result,indent=2),encoding="utf8")
    print(json.dumps({k:v for k,v in result.items() if k!="sources"},indent=2))
if __name__=="__main__":main()
