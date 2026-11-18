"""Runtime source extracted from the accepted V6.26 freeze: experiments/v67_road/material.py. """

import cv2

import numpy as np

from concept_road.vision import FOCAL, CX, CY, HEIGHT

try:
    from ._markings import material_sections as native_sections
except ImportError:
    native_sections = None


def road_sections(road, depths):
    if native_sections is not None:
        return native_sections(road.view(np.uint8), depths, CX, CY, HEIGHT*FOCAL, FOCAL)
    sections = []; pixels = []; edges = []
    for z in depths:
        v = round(CY+HEIGHT*FOCAL/z)
        if not 2 <= v < road.shape[0]-2: continue
        active = np.flatnonzero(road[v-2:v+3].mean(axis=0) >= .6)
        groups = [g for g in np.split(active, np.flatnonzero(np.diff(active)>1)+1) if len(g)>8]
        if not groups: continue
        g = min(groups, key=lambda g: max(g[0]-CX, CX-g[-1], 0))
        left, right = (g[[0,-1]].astype(float)-CX)*z/FOCAL
        if right-left < .3: continue
        sections.append([z,left,right,.045+.004*z])
        pixels.append([int(g[0]),int(g[-1]),v])
        if g[0] > 3: edges.append([z,left,-1.,0.])
        if g[-1] < road.shape[1]-4: edges.append([z,right,1.,0.])
    return sections, pixels, edges


def local_edge_error(points, coef, at, depth_step, fallback):
    """Weighted local residual; require distinct depth bins and current evidence."""
    local=points[np.abs(points[:,0]-at)<=5.*depth_step]
    bins=np.unique(np.rint((local[:,0]-at)/depth_step)) if len(local) else []
    if len(bins)<3 or not np.any(local[:,3]<1e-6):
        return fallback
    errors=np.abs(local[:,1]-np.polynomial.polynomial.polyval(local[:,0],coef))
    weight=np.exp(-local[:,3]/.3)/(1.+.02*local[:,0]**2)
    order=np.argsort(errors);cumulative=np.cumsum(weight[order])
    return float(errors[order[np.searchsorted(cumulative,.8*cumulative[-1])]])

class MaterialRoad:
    fit_boundaries = True

    def __init__(self, temporal=True):
        self.temporal = temporal
        self.pavement = None
        self.history = []
        self.last = {}
        self.lane_width = None
        self.lane_width_samples = 0
        self.chroma_squared = None

    def read(self, image, raw, lane_mask, speed=0., yaw=0., dt=.2, object_boxes=()):
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        self.chroma_squared = None
        if self.pavement is None:
            seeds=raw.astype(bool)&~lane_mask.astype(bool)
            seeds[:235]=False
            for x0,y0,x1,y1 in object_boxes:
                seeds[max(0,int(y0)):int(y1)+1,max(0,int(x0)):int(x1)+1]=False
            if seeds.sum()<200:
                return np.zeros((0,4)),[],np.zeros(0)
            self.pavement = np.median(lab[seeds], axis=0)
        # Lab pixels are bytes; even half-integer median seeds and the sum of
        # these two squared differences are represented exactly in float32.
        delta_a = lab[:, :, 1].astype(np.float32) - np.float32(self.pavement[1])
        delta_b = lab[:, :, 2].astype(np.float32) - np.float32(self.pavement[2])
        chroma_squared = delta_a * delta_a + delta_b * delta_b
        self.chroma_squared = chroma_squared
        # Achromatic paint, cracks and shadows share pavement chroma.
        # Luminance cuts incorrectly made zebra stripes and seams impassable.
        pavement = (chroma_squared < 12**2)
        # Paint is traversable material, not a hole. Keep detected markings
        # only beside pavement; do not let false line pixels open distant road.
        near = cv2.dilate(pavement.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        paint = (lane_mask > 0) | ((chroma_squared < 10**2) & (lab[:, :, 0] > self.pavement[0]))
        road = (pavement | (paint & near)).astype(np.uint8)
        road[:int(CY)+2] = 0
        # A cone silhouette is occupied pavement, not a physical road verge.
        # Bridge only its observed image box where pavement exists on BOTH
        # sides. The separate obstacle constraint still forbids the cone.
        # Do not bridge to the image border or across an unobserved outer edge.
        occluded=np.zeros_like(road)
        for x0,y0,x1,y1 in object_boxes:
            left=max(1,int(x0)-3);right=min(image.shape[1]-2,int(x1)+3)
            for v in range(max(int(CY)+3,int(y0)),min(image.shape[0]-1,int(y1)+2)):
                if road[v,max(0,left-3):left].any() and road[v,right+1:right+4].any():
                    road[v,left:right+1]=1;occluded[v,left:right+1]=1
        road = cv2.morphologyEx(road, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(road, 8)
        seeds = labels[raw.astype(bool)]
        counts = np.bincount(seeds.ravel(), minlength=n); counts[0] = 0
        road = (labels == int(np.argmax(counts))) if counts.max() else np.zeros_like(road, bool)
        depths = np.linspace(.95, 12., 45)
        sections, pixels, edges = road_sections(road, depths)
        old=[]
        if self.temporal and 0 < dt <= .5:
            angle=yaw*dt;c,s=np.cos(angle),np.sin(angle)
            # Exact constant-twist translation over this sensor interval.
            dz=speed*dt*np.sinc(angle/np.pi)
            dx=speed*dt*(1-np.cos(angle))/angle if abs(angle)>1e-5 else 0.
            for z,x,side,age in self.history:
                z-=dz;x-=dx
                zz=c*z+s*x;xx=-s*z+c*x
                if age+dt <= .8 and .2 < zz < 12: old.append([zz,xx,side,age+dt])
        # Fresh observations always enter the fit. Historical evidence has an
        # explicit age and can neither outlive 0.8 s nor preload the first frame.
        self.history = edges + old
        rows=np.asarray(sections,float).reshape(-1,4)
        heading=np.zeros(len(rows)); fits=[]
        for side in (-1,1):
            points=np.asarray([e for e in self.history if e[2]==side],float).reshape(-1,4)
            if len(points)<6 or np.ptp(points[:,0])<2.: continue
            z,x,age=points[:,0],points[:,1],points[:,3]
            weights=np.exp(-age/.3)/(1+.02*z*z)
            coef=np.polynomial.polynomial.polyfit(z,x,2,w=np.sqrt(weights))
            for iteration in range(3):
                residual=x-np.polynomial.polynomial.polyval(z,coef)
                if iteration==0 and np.all(abs(residual)<=.12):
                    break  # Repeating the same weighted fit gives identical coefficients.
                robust=np.minimum(1.,.12/np.maximum(abs(residual),1e-6))
                coef=np.polynomial.polynomial.polyfit(z,x,2,w=np.sqrt(weights*robust))
            residual=x-np.polynomial.polynomial.polyval(z,coef)
            err=float(np.quantile(abs(residual),.8))
            if err>.35: continue
            fits.append(dict(side=side,coef=coef.tolist(),residual=err,points=len(points),zmin=float(z.min()),zmax=float(z.max())))
        if len(fits)>1:
            anchor=max(fits,key=lambda f:(f['zmax']-f['zmin'])/(.04+f['residual']))
            aa=np.asarray(anchor['coef']);stable=[]
            for f in fits:
                zz=np.linspace(max(f['zmin'],anchor['zmin']),min(f['zmax'],anchor['zmax']),12)
                bb=np.asarray(f['coef'])
                disagreement=np.median(abs(np.arctan(aa[1]+2*aa[2]*zz)-np.arctan(bb[1]+2*bb[2]*zz)))
                if f is anchor or disagreement<.18:stable.append(f)
            fits=stable
        if fits and len(rows):
            # Physical edge tangent only. A clipped pixel boundary is excluded.
            all_heading=[np.arctan(f['coef'][1]+2*f['coef'][2]*rows[:,0]) for f in fits]
            weights=np.array([(f['zmax']-f['zmin'])/(.04+f['residual'])**2 for f in fits])
            heading=np.average(all_heading,axis=0,weights=weights)
        if fits and len(rows) and self.fit_boundaries:
            for i,row in enumerate(rows):
                for f in fits:
                    col=1 if f['side']==-1 else 2
                    zmin=min(e[0] for e in self.history if e[2]==f['side'])
                    zmax=max(e[0] for e in self.history if e[2]==f['side'])
                    # Never extrapolate beyond actually observed history.
                    if zmin<=row[0]<=zmax:
                        estimate=np.polynomial.polynomial.polyval(row[0],f['coef'])
                        raw=row[col]
                        row[col]=estimate
                        # A disagreement becomes uncertainty, not invented free
                        # space. Footprint feasibility subtracts this margin.
                        points=np.asarray([e for e in self.history if e[2]==f['side']],float)
                        # Five observed depth steps on either side: uncertainty
                        # belongs to this part of the edge, not a distant bend.
                        local_error=local_edge_error(points,f['coef'],row[0],float(np.median(np.diff(depths))),f['residual'])
                        row[3]=max(row[3],local_error+.04,abs(raw-estimate))
        self.last=dict(physical_edges=fits,history_points=len(old),fresh_points=len(edges),
                       max_age_s=max([e[3] for e in self.history]+[0]),
                       heading_source='observed material boundaries; excludes FOV clipping',
                       unknown_heading=not bool(fits),lane_count_prior=None,
                       material_seed='current B ego-road pixels excluding markings and detections',
                       uncertainty_source='V6.11.1 local same-edge residual; 3 native depth bins and current support, 5 depth steps each side; 80 percent weighted residual plus .04; max raw-fit disagreement')
        return rows,pixels,heading
