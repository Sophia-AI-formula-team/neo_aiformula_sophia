"""Runtime source extracted from the accepted V6.26 freeze: experiments/v625_boundary/onboard625.py. """

from itertools import combinations

import numpy as np

HALF_TRACK=.30

HALF_TREAD=.055

HALF_PAINT=.0735

def curve(track,z):
    return np.polynomial.polynomial.polyval(z,track['coef'])

class LocalLimits:
    def __init__(self):
        self.history={}

    def update(self,road,tracks,lines,speed,yaw,dt):
        road=np.array(road,dtype=float,copy=True)
        z=road[:,0]
        used={}
        # Integrate the last local sightings into the new body frame.
        ang=yaw*dt
        for side,(stored,err) in self.history.items():
            points=stored[:,:2]; ages=stored[:,2]+dt
            p=points-[speed*dt*np.sinc(ang/np.pi),speed*dt*.5*ang*np.sinc(.5*ang/np.pi)**2]
            p=p@np.array([[np.cos(ang),-np.sin(ang)],[np.sin(ang),np.cos(ang)]])
            good=np.r_[True,np.diff(p[:,0])>.01]
            good &= ages<=1.000001
            if good.sum()>=2 and np.all(np.diff(p[good,0])>0):
                used[side]=(np.column_stack((p[good],ages[good])),err)
        paint={line['track'] for line in lines if len(line.get('points',[]))>=3}
        available=[t for t in tracks if t['id'] in paint and t['age']<=.65]
        hypotheses=[]
        for triple in combinations(available,3):
            lo=max(t['zmin'] for t in triple); hi=min(t['zmax'] for t in triple)
            if hi-lo<1.25:continue
            zz=np.linspace(lo,hi,8)
            tri=sorted(triple,key=lambda t:curve(t,(lo+hi)/2))
            xs=np.array([curve(t,zz) for t in tri])
            # Compare widths normal to the centre marking; a bend can appear wider in x.
            slope=np.gradient(xs[1],zz)
            widths=np.diff(xs,axis=0)/np.hypot(1,slope)
            if widths.min()<2.35 or widths.max()>5.1:continue
            discrepancy=np.mean(abs(widths[0]-widths[1]))+sum(np.std(w) for w in widths)
            for j,side in [(0,'left'),(2,'right')]:
                if side in used:
                    p,_=used[side]; shared=(zz>=p[0,0])&(zz<=p[-1,0])
                    if np.any(shared):
                        discrepancy+=.6*np.median(abs(xs[j,shared]-np.interp(zz[shared],p[:,0],p[:,1])))
            hypotheses.append((discrepancy,tri))
        evidence={}; ids=None
        if hypotheses:
            hypotheses.sort(key=lambda h:h[0])
            if len(hypotheses)==1 or hypotheses[1][0]-hypotheses[0][0]>.20 or (
                    hypotheses[0][1][1]['id']==hypotheses[1][1][1]['id']):
                tri=hypotheses[0][1];ids=[t['id'] for t in tri]
                for side,t in zip(('left','right'),[tri[0],tri[2]]):
                    zz=np.arange(max(-1.2,t['zmin']),min(12,t['zmax'])+.01,.12)
                    if len(zz)<2:continue
                    pts=np.column_stack([zz,curve(t,zz),np.zeros_like(zz)])
                    # Preserve old near-field support only where the camera no longer sees it.
                    if side in used:
                        old=used[side][0]
                        if old[0,0]<=zz[0]<=old[-1,0] and abs(np.interp(zz[0],old[:,0],old[:,1])-pts[0,1])<.35:
                            pts=np.vstack((old[old[:,0]<zz[0]-.03],pts))
                    used[side]=(pts,float(t['error']))
        self.history=used
        for j,side in [(1,'left'),(2,'right')]:
            if side not in used:continue
            pts,err=used[side];valid=(z>=pts[0,0])&(z<=pts[-1,0])
            projected=np.interp(z,pts[:,0],pts[:,1]);sign=1 if j==1 else -1
            limit=projected-sign*HALF_PAINT
            tighter=valid&(sign*(limit-road[:,j])>0)
            road[tighter,j]=limit[tighter]
            road[tighter,j+2]=np.maximum(.04,err+.025*np.interp(z[tighter],pts[:,0],pts[:,2]))
            evidence[side]=pts[::2,:2].round(3).tolist()
        return road,dict(ordered_paint=ids,edges=evidence)

def tread_clearance(road,z,x,heading,uncertainty_scale=1.):
    # For each side tyre at least one point on its tread must touch legal paint/road.
    z,x,h=np.broadcast_arrays(z,x,heading);c=np.cos(h);s=np.sin(h)
    def margins(a):
        qz=z[...,None]-s[...,None]*a; qx=x[...,None]+c[...,None]*a
        left=qx-np.interp(qz,road[:,0],road[:,1]);right=np.interp(qz,road[:,0],road[:,2])-qx
        lower=np.minimum(left-uncertainty_scale*np.interp(qz,road[:,0],road[:,3]),
                         right-uncertainty_scale*np.interp(qz,road[:,0],road[:,min(4,road.shape[1]-1)]))
        return np.minimum(left,right),lower,(qz>=road[0,0])&(qz<=road[-1,0])
    sides=[];bounds=[];known=[]
    for offset in (-HALF_TRACK,HALF_TRACK):
        # Interior samples make this a conservative contact test on a curved edge.
        g,l,k=margins(offset+np.linspace(-HALF_TREAD,HALF_TREAD,5))
        sides.append(g.max(-1));bounds.append(l.max(-1));known.append(k.all(-1))
    return np.minimum(*sides),np.minimum(*bounds),np.logical_and(*known)
