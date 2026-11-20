"""Runtime source extracted from the accepted V6.26 freeze: experiments/v67_road/road.py. """

import cv2

import numpy as np

from concept_road.vision import FOCAL, CX, CY, HEIGHT

from concept_road.material import MaterialRoad

try:
    from ._markings import trace_paint
except ImportError:  # Direct Python source checkout, without colcon build.
    trace_paint = None

DEPTHS=np.linspace(.95,12.,45)
DEPTH_PIXEL_ROWS = np.asarray([round(CY + HEIGHT * FOCAL / z) for z in DEPTHS])


def _weighted_quadratic(design, lateral, weights, rcond, depth):
    # Match numpy.polynomial.polyfit's weighted, column-scaled solve. The
    # Vandermonde matrix is the same at every robust reweighting iteration.
    lhs = design * weights
    rhs = lateral * weights
    scale = np.sqrt(np.square(lhs).sum(axis=1))
    scale[scale == 0] = 1.
    coefficients, _, rank, _ = np.linalg.lstsq(lhs.T / scale, rhs, rcond)
    if rank != 3:
        return np.polynomial.polynomial.polyfit(depth, lateral, 2, w=weights)
    return coefficients / scale

def fit(points):
    p=np.asarray(points,float)
    if len(p)<5 or np.ptp(p[:,0])<1.:return None
    z,x=p[:,0],p[:,1]
    design = np.polynomial.polynomial.polyvander(z+0., 2).T
    fit_lateral = x+0.
    rcond = len(z) * np.finfo(z.dtype).eps
    weights=np.exp(-p[:,2]/.35)/(1+.04*z*z)
    coefficient=_weighted_quadratic(design,fit_lateral,np.sqrt(weights),rcond,z)
    for iteration in range(5):
        residual=x-np.polynomial.polynomial.polyval(z,coefficient)
        # The initial least-squares solution is already a fixed point when
        # every sample receives exactly unit robust weight.
        if iteration==0 and np.all(abs(residual)<=.10):
            break
        robust=np.minimum(1.,.10/np.maximum(abs(residual),1e-6))
        coefficient=_weighted_quadratic(design,fit_lateral,np.sqrt(weights*robust),rcond,z)
    residual=abs(x-np.polynomial.polynomial.polyval(z,coefficient))
    accepted=residual<.25
    if accepted.sum()<5 or np.ptp(z[accepted])<1.:return None
    error=float(np.quantile(residual[accepted],.8))
    if error>.20:return None
    return dict(coef=coefficient.tolist(),error=error,zmin=float(z[accepted].min()),
                zmax=float(z[accepted].max()),points=p[accepted].tolist(),
                age=float(np.average(p[accepted,2],weights=weights[accepted])))

def val(curve,z):
    coefficients = curve['coef']
    if len(coefficients) != 3 or isinstance(z, (tuple, list)):
        return np.polynomial.polynomial.polyval(z, coefficients)
    intercept, slope, quadratic = coefficients
    return intercept + z * (slope + z * quadratic)

def tangent(curve,z):return np.arctan(curve['coef'][1]+2*curve['coef'][2]*z)

class RoadMap:
    def __init__(self,mode):
        assert mode in ('current','lanes','fused','guided')
        self.mode=mode;self.tracks=[];self.serial=0;self.last={};self.material=None
        self.material_road=MaterialRoad() if mode in ('fused','guided') else None

    def transport(self,speed,yaw,dt):
        if not 0<dt<=.5:return []
        a=yaw*dt;c,s=np.cos(a),np.sin(a)
        dz=speed*dt*np.sinc(a/np.pi)
        dx=speed*dt*(1-np.cos(a))/a if abs(a)>1e-5 else 0.
        old=[]
        for track in self.tracks:
            source=np.asarray(track['points'], dtype=float)
            z=source[:,0]-dz
            x=source[:,1]-dx
            zz=c*z+s*x
            age=source[:,2]+dt
            retained=(age<=.8)&(zz>.2)&(zz<14.)
            curve=fit(np.column_stack((zz[retained],(-s*z+c*x)[retained],age[retained])))
            if curve is not None:curve['id']=track['id'];old.append(curve)
        return old

    def markings(self,lane):
        if trace_paint is not None:
            tracks = trace_paint(np.ascontiguousarray(lane, dtype=np.uint8),
                                 lane.shape[0]-4, int(CY+HEIGHT*FOCAL/12),
                                 CY, CX, HEIGHT*FOCAL, FOCAL)
            return [curve for points in tracks if (curve := fit(points)) is not None]
        tracks=[]
        # Scan actual pixel rows once; duplicate distant pixel rows must not
        # masquerade as many independent measurements.
        for v in range(lane.shape[0]-4,int(CY+HEIGHT*FOCAL/12),-1):
            z=HEIGHT*FOCAL/(v-CY)
            active=np.flatnonzero(lane[v])
            groups=[g for g in np.split(active,np.flatnonzero(np.diff(active)>2)+1)
                    if len(g)>1 and g[0]>2 and g[-1]<lane.shape[1]-3 and len(g)*z/FOCAL<.45]
            xs=[(float(np.mean(g))-CX)*z/FOCAL for g in groups]
            used=set()
            for x in xs:
                choices=[]
                for i,points in enumerate(tracks):
                    if i in used or z-points[-1][0]>4.:continue
                    recent=np.array(points[-8:]);prediction=recent[-1,1]
                    if len(recent)>2 and np.ptp(recent[:,0])>.15:
                        slope=np.clip(np.polyfit(recent[:,0],recent[:,1],1)[0],-2.,2.)
                        prediction+=slope*(z-recent[-1,0])
                    gap=abs(x-prediction)
                    if gap<.20+.10*(z-points[-1][0])+.012*z:choices.append((gap,i))
                if choices:
                    _,i=min(choices);tracks[i].append([z,x,0.]);used.add(i)
                else:tracks.append([[z,x,0.]]);used.add(len(tracks)-1)
        return [curve for points in tracks if (curve:=fit(points)) is not None]

    @staticmethod
    def road_rows(raw):
        n,labels=cv2.connectedComponents(raw.astype(np.uint8),connectivity=8)
        seed=labels[260:350,200:460];counts=np.bincount(seed.ravel(),minlength=n);counts[0]=0
        component=labels==int(counts.argmax()) if counts.max() else np.zeros_like(raw,bool)
        observed = (DEPTH_PIXEL_ROWS >= 2) & (DEPTH_PIXEL_ROWS < raw.shape[0] - 2)
        pixels = DEPTH_PIXEL_ROWS[observed]
        # Two of five pixels are exactly the old .4 support threshold.
        support = component[pixels[:, None] + np.arange(-2, 3)].sum(axis=1) >= 2
        valid = support.sum(axis=1) >= 12
        if not np.any(valid):
            return np.empty((0, 5)), component
        depth, visible = DEPTHS[observed][valid], support[valid]
        left = visible.argmax(axis=1)
        right = visible.shape[1] - 1 - visible[:, ::-1].argmax(axis=1)
        # Use the exterior envelope. Internal holes still do not form edges.
        return np.column_stack((depth, (left-CX)*depth/FOCAL, (right-CX)*depth/FOCAL,
                                left, right)), component

    def read(self,image,raw,lane,boxes=(),speed=0.,yaw=0.,dt=.2):
        previous=self.transport(speed,yaw,dt)
        fresh=self.markings(lane)
        matched=set();tracks=[]
        for curve in sorted(fresh,key=lambda c:c['zmax']-c['zmin'],reverse=True):
            choices=[]
            for i,old in enumerate(previous):
                if i in matched:continue
                lo=max(curve['zmin'],old['zmin']);hi=min(curve['zmax'],old['zmax'])
                if hi-lo<.7:continue
                z=np.linspace(lo,hi,12);error=float(np.median(abs(val(curve,z)-val(old,z))))
                angle=float(np.median(abs(tangent(curve,z)-tangent(old,z))))
                if error<.55 and angle<.35:choices.append((error,i))
            if choices:
                _,i=min(choices);matched.add(i);old=previous[i]
                joined=fit(curve['points']+old['points'])
                if joined is not None:curve=joined
                curve['id']=old['id']
            else:self.serial+=1;curve['id']=self.serial
            tracks.append(curve)
        tracks.extend(old for i,old in enumerate(previous) if i not in matched)
        # Same marking can be split into fragments; keep the stronger fit when
        # two fits agree over a substantial common span.
        unique=[]
        for curve in sorted(tracks,key=lambda c:(c['zmax']-c['zmin'])/(.05+c['error']),reverse=True):
            duplicate=False
            for old in unique:
                lo=max(curve['zmin'],old['zmin']);hi=min(curve['zmax'],old['zmax'])
                if hi-lo>1.:
                    z=np.linspace(lo,hi,8)
                    if np.median(abs(val(curve,z)-val(old,z)))<.23:duplicate=True;break
            if not duplicate:unique.append(curve)
        self.tracks=unique
        anchors,component=self.road_rows(raw)
        pairs=[]
        for i,left in enumerate(unique):
            for j,right in enumerate(unique):
                if i==j:continue
                lo=max(1.7,left['zmin'],right['zmin']);hi=min(9.,left['zmax'],right['zmax'])
                if hi-lo<1.5:continue
                z=np.linspace(lo,hi,20);l=val(left,z);r=val(right,z);width=r-l
                if width.min()<1.2 or width.max()>6.5:continue
                # A lane is between adjacent, noncrossing observed markings.
                if any(k not in (i,j) and other['zmin']<=lo+.5 and other['zmax']>=hi-.5 and
                       np.mean((val(other,z)>l+.2)&(val(other,z)<r-.2))>.7 for k,other in enumerate(unique)):continue
                if np.median(abs(tangent(left,z)-tangent(right,z)))>.45:continue
                supported=anchors[(anchors[:,0]>=lo)&(anchors[:,0]<=hi)]
                score=0.
                for a,bl,br,*_ in supported:
                    ll,rr=val(left,a),val(right,a)
                    score+=max(0.,min(br,rr)-max(bl,ll))/max(.1,br-bl)
                if len(supported):score/=len(supported)
                pairs.append(dict(left=i,right=j,score=score,width=float(np.median(width)),zmin=lo,zmax=hi))
        current=max(pairs,key=lambda p:p['score'],default=None)
        selected=[]
        if current and current['score']>.45:
            selected=[current]
            if self.mode!='current':
                changed=True
                while changed:
                    changed=False
                    for pair in pairs:
                        if pair in selected:continue
                        adjacent=any(pair['left']==p['right'] or pair['right']==p['left'] for p in selected)
                        if adjacent and .55< pair['width']/current['width'] <1.6:
                            selected.append(pair);changed=True
        leftcurve=rightcurve=None
        current_curves=None
        if selected:
            current_curves=(unique[current['left']],unique[current['right']])
            leftcurve=unique[min(selected,key=lambda p:val(unique[p['left']],4.))['left']]
            rightcurve=unique[max(selected,key=lambda p:val(unique[p['right']],4.))['right']]
        # When paint is absent, fit only B's exterior envelope. Missing interior
        # pixels never become extra road edges or inflate edge-fit uncertainty.
        if leftcurve is None and len(anchors):
            leftcurve=fit([[z,l,0.] for z,l,r,u,v in anchors if u>3])
            rightcurve=fit([[z,r,0.] for z,l,r,u,v in anchors if v<636])
        material_rows=None
        rows=[];pixels=[];head=[];row_sources=[]
        for z in DEPTHS:
            if leftcurve is None or rightcurve is None:continue
            lc,rc=leftcurve,rightcurve
            if current_curves:
                lc,rc=current_curves
                # Preserve the current lane at short range. An outer marking
                # observed only farther away must not delete all near rows.
                usable=[p for p in selected if all(c['zmin']-2.<=z<=c['zmax']+.6 for c in [unique[p['left']],unique[p['right']]])]
                if usable:
                    connected=[current] if current in usable else []
                    changed=True
                    while changed:
                        changed=False
                        for p in usable:
                            if p not in connected and any(p['left']==q['right'] or p['right']==q['left'] for q in connected):
                                connected.append(p);changed=True
                    if connected:
                        lc=unique[min(connected,key=lambda p:val(unique[p['left']],z))['left']]
                        rc=unique[max(connected,key=lambda p:val(unique[p['right']],z))['right']]
            if any(z>c['zmax']+.6 or z<c['zmin']-2. for c in [lc,rc]):continue
            l,r=val(lc,z),val(rc,z)
            if not .8<r-l<12:continue
            sigma=max(.045+c['error']+.04*c['age']+.05*max(c['zmin']-z,z-c['zmax'],0) for c in [lc,rc])
            rows.append([z,float(l),float(r),sigma]);head.append(float((tangent(lc,z)+tangent(rc,z))/2))
            row_sources.append([z,lc.get('id'),rc.get('id')])
            v=int(round(CY+HEIGHT*FOCAL/z))
            pixels.append([int(np.clip(CX+l*FOCAL/z,0,639)),int(np.clip(CX+r*FOCAL/z,0,639)),v])
        occupied_rows = component[component.any(axis=1)]
        widths = (component.shape[1] - occupied_rows.argmax(axis=1) -
                  occupied_rows[:, ::-1].argmax(axis=1))
        envelope_negative = int(widths.sum() - occupied_rows.sum())
        self.last=dict(version={'current':'V6.7.1','lanes':'V6.8.1','fused':'V6.9.2','guided':'V6.13.1'}[self.mode],
                       tracks=unique,lane_bands=pairs,current_lane=current,allowed_bands=selected,
                       adjacent_direction='unknown; lane-changing permitted by experiment rule',
                       left_curve=leftcurve,right_curve=rightcurve,road_rows=len(rows),
                       memory_max_age=.8,history_points=sum(p[2]>0 for c in unique for p in c['points']),
                       envelope_negative_pixels=envelope_negative,row_sources=row_sources,
                       source='B ego lane anchor + observed marking curves + causal wheel/gyro history',
                       lane_count_prior=None,material_support=material_rows is not None)
        if self.mode in ('fused','guided'):
            material_rows,pixels,material_head=self.material_road.read(image,raw,lane,speed,yaw,dt,boxes)
            if self.mode=='guided' and len(rows)>=3:
                guidance=np.asarray(rows)
                supported=(material_rows[:,0]>=guidance[0,0])&(material_rows[:,0]<=guidance[-1,0])
                material_head[supported]=np.interp(material_rows[supported,0],guidance[:,0],head)
            self.last.update(material_support=True,material_evidence=self.material_road.last,
                             lane_geometry_rows=rows,road_rows=len(material_rows),
                             guidance_used=self.mode=='guided' and len(rows)>=3,
                             source='B ego lane identifies material/component; RGB + lane restore free surface; causal boundary history')
            return material_rows,pixels,material_head
        return np.asarray(rows).reshape(-1,4),pixels,np.asarray(head)
