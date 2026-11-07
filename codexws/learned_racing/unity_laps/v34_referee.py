"""Offline near-field diagnostic. World geometry is NEVER imported by policy.

The installed referee corridor is projected onto a flat ground plane. This is
not a hand-labeled image benchmark: camera/body pitch and occlusions limit it.
"""
import argparse
import numpy as np
from scipy.spatial import cKDTree
from render import track
from vision import CX,FOCAL
from v33_trials import ROOT,read,save

def score(path):
    r=read(path);rows=np.asarray(r['rows']);geom=track()
    center=(geom[0]+geom[1])/2;right=geom[0]-geom[1]
    lane=float(np.median(np.linalg.norm(right,axis=1)));right/=np.linalg.norm(right,axis=1)[:,None]
    tree=cKDTree(center);errors=[];free=bad=truth_count=covered=0;opportunities=observed=0
    worst=[]
    for d in r['decisions']:
        q=d['decision'];road=np.asarray(q['road'])
        if len(road)<2:continue
        row=rows[int(np.argmin(abs(rows[:,0]-d['t'])))];f=np.array([np.sin(row[3]),np.cos(row[3])]);side=np.array([f[1],-f[0]])
        for z in np.linspace(1.2,3.,7):
            # Wide truth cross-section then intersect with the actual camera FOV.
            xx=np.arange(-10.,10.001,.02);world=row[1:3]+z*f+xx[:,None]*side
            idx=tree.query(world)[1];off=np.sum((world-center[idx])*right[idx],axis=1)
            truth=(off<=lane*.5)&(off>=-lane*1.5)
            groups=[g for g in np.split(np.flatnonzero(truth),np.flatnonzero(np.diff(np.flatnonzero(truth))>1)+1) if len(g)]
            if not groups:continue
            g=min(groups,key=lambda g:min(abs(xx[g])));tl,tr=xx[g[0]],xx[g[-1]]
            fov_l=-CX*z/FOCAL;fov_r=(639-CX)*z/FOCAL
            lo,hi,sigma=[float(np.interp(z,road[:,0],road[:,k])) for k in (1,2,3)]
            vis=(xx>=fov_l)&(xx<=fov_r)
            predicted=vis&(xx>=lo)&(xx<=hi)
            free+=int(predicted.sum());bad+=int((predicted&~truth).sum())
            truth_count+=int((truth&vis).sum());covered+=int((truth&predicted).sum())
            for true,estimate in ((tl,lo),(tr,hi)):
                if fov_l+.04<true<fov_r-.04:
                    opportunities+=1
                    if fov_l+.04<estimate<fov_r-.04:
                        observed+=1;err=abs(true-estimate);errors.append(err)
                        worst.append((err,d['t'],float(z)))
    return dict(id=r['id'],status=r['status'],visible_edge_opportunities=opportunities,estimated_edges=observed,
                coverage=observed/opportunities if opportunities else None,
                absolute_edge_error_p95_m=float(np.quantile(errors,.95)) if errors else None,
                false_free_fraction=bad/free if free else None,road_recall=covered/truth_count if truth_count else None,
                worst_frames=sorted(worst,reverse=True)[:5])

def main():
    p=argparse.ArgumentParser();p.add_argument('experiment');a=p.parse_args();root=ROOT/a.experiment
    results=[score(p) for p in sorted((root/'runs').glob('*.json'))]
    save(root/'offline_road_referee.json',dict(results=results,qualification_pass=False,
         limits='Offline planar diagnostic only, 2 cm sampling of installed referee corridor. World pose and map are scoring inputs only; pitch/roll and occlusion are not modeled. Not a substitute for same-physical-edge temporal acceptance or manual image labels.'))
    for r in results:print(r,flush=True)
if __name__=='__main__':main()
