"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/map_semantics618.py. """

from collections import Counter
from functools import lru_cache

import cv2

import numpy as np

from concept_road.runtime_map import CELL, transform

STEP = .10

FORWARD = np.arange(1.2, 12.01, STEP)

RIGHT = np.arange(-13., 13.01, STEP)

GRID_Z, GRID_X = np.meshgrid(FORWARD, RIGHT, indexing='ij')

def runs(indices):
    return [g for g in np.split(indices, np.flatnonzero(np.diff(indices)>1)+1) if len(g)]

@lru_cache(maxsize=4)
def pavement_projection(rows, columns, projection):
    focal,cx,cy,height=projection
    u=np.rint(cx+focal*GRID_X/GRID_Z).astype(int)
    v=np.rint(cy+height*focal/GRID_Z).astype(int)
    valid=(u>=3)&(u<columns-3)&(v>=int(cy)+4)&(v<rows-3)
    return np.clip(u,0,columns-1),np.clip(v,0,rows-1),valid

def pavement_grid(image, raw, lane, boxes, projection):
    u,v,visible=pavement_projection(*image.shape[:2],tuple(projection))
    valid=visible.copy()
    for l,t,r,b in boxes:
        valid &= ~((u>=l-2)&(u<=r+2)&(v>=t-2)&(v<=b+2))
    lab=cv2.cvtColor(image[v,u],cv2.COLOR_BGR2LAB).astype(np.float32)
    b_support=raw[v,u]&valid
    seed=lab[b_support & ~lane[v,u] & (GRID_Z<5.0)]
    road=np.zeros_like(valid);paint=np.zeros_like(valid);pale=np.zeros_like(valid)
    if len(seed)>=100:
        colour=np.median(seed,axis=0)
        chroma=np.linalg.norm(lab[:,:,1:]-colour[1:],axis=2)
        lum=lab[:,:,0]-colour[0]
        road=valid&(chroma<=12)&(lum>=-36)&(lum<=31)
        pale=valid&(chroma<=13)&(lum>=24)
        beside=cv2.dilate(road.astype(np.uint8),np.ones((3,3),np.uint8))>0
        # A genuine white line needs reflectance evidence as well as proximity
        # to the network's paint prediction. Painted asphalt remains asphalt.
        network_near=cv2.dilate(lane.astype(np.uint8),np.ones((3,3),np.uint8))>0
        paint=pale & network_near[v,u]
        road |= pale & beside
    return road,paint,pale,valid

def part_grid(parts, kinds):
    out=np.zeros(GRID_Z.shape,np.uint8)
    for p in parts:
        if p['kind'] not in kinds:
            continue
        a=np.asarray(p['rows'])
        pts=np.vstack((a[:,[1,0]],a[::-1][:,[2,0]]))
        pts=np.rint((pts-[RIGHT[0],FORWARD[0]])/STEP).astype(np.int32)
        cv2.fillPoly(out,[pts],1)
    return out>0

def road_fills(parts, measured):
    result=[]
    all_known=part_grid(parts,{'ego','adjacent','ego_mask'})
    adjacent=part_grid(parts,{'adjacent'})
    for kind in ('ego','adjacent'):
        original=part_grid(parts,{kind})
        closed=cv2.morphologyEx(original.astype(np.uint8),cv2.MORPH_CLOSE,
                               np.ones((9,5),np.uint8)).astype(bool)
        fill=closed & ~original & measured
        # For ego, the pale B-pixel fallback can already cover the missing
        # marking-defined band. Retain the distinction as a repair overlay.
        if kind=='adjacent':
            fill &= ~all_known
        else:
            fill &= ~adjacent
        y,x=np.nonzero(fill)
        result.append((kind,np.c_[FORWARD[y],RIGHT[x]]))
    return result

def marking_lines(evidence, paint, rgb_pale, asphalt, valid):
    result=[]
    for track in evidence['tracks']:
        coef=track['coef']
        z=FORWARD[(FORWARD>=max(1.35,track['zmin']))&(FORWARD<=min(11.,track['zmax']))]
        if len(z)<5:
            continue
        x=np.polynomial.polynomial.polyval(z,coef)
        offsets=np.arange(-.30,.301,STEP)
        zi=np.rint((z-FORWARD[0])/STEP).astype(int)
        xi_raw=np.rint((x[:,None]+offsets-RIGHT[0])/STEP).astype(int)
        xi=np.clip(xi_raw,0,len(RIGHT)-1)
        view=valid[zi[:,None],xi]&(xi_raw==xi)
        road=asphalt[zi[:,None],xi]
        bright=paint[zi[:,None],xi]&view
        no_visible_paint=~(rgb_pale[zi[:,None],xi[:,1:-1]]&view[:,1:-1]).any(axis=1)
        # Reject detections far from this line's fresh or remembered support.
        samples=np.asarray(track['points'])
        supported=np.min(abs(z[:,None]-samples[:,0]),axis=1)<=.35
        present=np.any(bright,axis=1)&supported
        visible_unpainted=np.all(view[:,1:-1],axis=1)&(road.mean(axis=1)>=.6)&no_visible_paint
        lit=runs(np.flatnonzero(present))
        marks=[g for g in lit if len(g)>=3]
        if not marks:
            continue
        distance=np.r_[0.,np.cumsum(np.hypot(np.diff(z),np.diff(x)))]
        styles=np.full(len(z),'unclassified',dtype='<U12')
        for g in marks:
            if distance[g[-1]]-distance[g[0]]>=2.8:
                styles[g]='solid'
        gap_list=[]
        for a,b in zip(marks,marks[1:]):
            gap=np.arange(a[-1]+1,b[0])
            if not len(gap) or not (.60 <= distance[b[0]]-distance[a[-1]] <= 5.0):
                continue
            if (visible_unpainted[gap]&~present[gap]).mean()<.85:
                continue
            # A long solid stretch followed by a short strip can be a
            # transition at a mouth, rather than two dashes of one pattern.
            if max(distance[g[-1]]-distance[g[0]] for g in (a,b))>2.8:
                continue
            styles[a]=styles[b]='dashed'
            gap_list.append(np.c_[z[gap],x[gap]].round(3).tolist())
        for g in marks:
            segment=np.c_[z[g],x[g]]
            result.append(dict(track=track['id'],kind=str(styles[g[0]]),
                               points=segment.round(3).tolist()))
        if gap_list:
            result.append(dict(track=track['id'],kind='observed_gap',gaps=gap_list))
    return result

def inside_openings(evidence, asphalt, valid):
    """Find directly observed pavement extending into the inner side of the circuit."""
    current=evidence.get('current_lane')
    if not current or current['score']<.7:
        return []
    lines=evidence['tracks']
    left,right=[lines[current[side]] for side in ('left','right')]
    z=FORWARD
    shared=(z>=max(2.1,left['zmin'],right['zmin']))&(z<=min(10.5,left['zmax'],right['zmax']))
    l,r=[np.polynomial.polynomial.polyval(z,edge['coef']) for edge in (left,right)]
    shared &= (r-l >= 2.5) & (r-l <= 5.1)
    offsets=np.arange(.30,-3.61,-STEP)
    ids=np.rint((l[:,None]+offsets-RIGHT[0])/STEP).astype(int)
    xc=np.clip(ids,0,len(RIGHT)-1)
    zr=np.arange(len(z))[:,None]
    usable=(xc==ids)&valid[zr,xc]
    observed=asphalt[zr,xc]&usable
    # Allow isolated sampling misses, but require physically connected
    # pavement at the threshold and further into the side entrance.
    required=offsets>=-2.8
    reach=shared & (usable[:,required].mean(axis=1)>=.95) & (observed[:,required].mean(axis=1)>=.88)
    candidates=[]
    for group in runs(np.flatnonzero(reach)):
        if z[group[-1]]-z[group[0]]<1.4:
            continue
        rows=[]
        for i in group:
            # Map only a contiguous part of the observed opening. Do not
            # draw across a color gap to remote asphalt.
            occupied=np.flatnonzero(observed[i])
            stop=0
            for j in occupied:
                if j-stop>3:
                    break
                stop=j
            if offsets[stop]>-2.8:
                continue
            rows.append([z[i],l[i]+offsets[stop],l[i]])
        if len(rows)<12:
            continue
        mid=rows[len(rows)//2]
        candidates.append(dict(mouth=[float(mid[0]),float(mid[2])],
                               rows=np.round(rows,3).tolist()))
    return candidates

class SemanticMap:
    def __init__(self, projection):
        self.projection=projection
        self.markings={}
        self.repairs={}
        self.gaps=set()
        self.junctions=[]

    def grid_coordinates(self, points, pose):
        return np.unique(np.floor(transform(points,pose)/CELL).astype(int),axis=0)

    def update(self, image, raw, lane, boxes, evidence, snapshot):
        pose,t=snapshot['pose'],snapshot['t']
        asphalt,paint,rgb_pale,valid=pavement_grid(image,raw,lane,boxes,self.projection)
        repairs=road_fills(snapshot['parts'],asphalt)
        changed_repairs=[]
        for kind,points in repairs:
            for z,x in self.grid_coordinates(points,pose):
                k=int(z),int(x)
                if self.repairs.get(k)!=kind:
                    self.repairs[k]=kind
                    changed_repairs.append([*k,kind])
        lines=marking_lines(evidence,paint,rgb_pale,asphalt,valid)
        votes={}
        changed_gaps=[]
        for line in lines:
            if line['kind']=='observed_gap':
                for gap in line['gaps']:
                    for z,x in self.grid_coordinates(gap,pose):
                        key=int(z),int(x)
                        if key not in self.gaps:
                            self.gaps.add(key)
                            changed_gaps.append(list(key))
                continue
            for z,x in self.grid_coordinates(line['points'],pose):
                k=int(z),int(x)
                votes.setdefault(k,set()).add(line['kind'])
        changed_markings=[]
        for k,kinds in votes.items():
            vote=self.markings.setdefault(k,Counter())
            vote.update(kinds)
            best=max(('solid','dashed'),key=lambda label:vote[label])
            name=best if vote[best]>=2 and vote[best]>vote['dashed' if best=='solid' else 'solid'] else 'unclassified'
            changed_markings.append([*k,name])
        openings=inside_openings(evidence,asphalt,valid)
        for opening in openings:
            anchor=transform(opening['mouth'],pose)
            nearest=min(self.junctions,key=lambda j:np.linalg.norm(j['position']-anchor),default=None)
            if nearest is None or np.linalg.norm(nearest['position']-anchor)>4.:
                nearest=dict(position=anchor.copy(), frames=0, first=t, last=t, start=pose[:2], travel=0.,cells=Counter())
                self.junctions.append(nearest)
            if nearest['frames'] and nearest['last']==t:
                continue
            nearest['frames']+=1
            nearest['last']=t
            distance=np.linalg.norm(np.asarray(pose[:2])-nearest['start'])
            nearest['travel']=max(nearest['travel'],float(distance))
            nearest['position']+= (anchor-nearest['position'])/nearest['frames']
            pixels=part_grid([dict(kind='branch',rows=opening['rows'])],{'branch'})&asphalt
            iz,ix=np.nonzero(pixels)
            nearest['cells'].update(tuple(p) for p in self.grid_coordinates(np.c_[FORWARD[iz],RIGHT[ix]],pose))
        confirmed=[dict(id=i+1,position=j['position'].round(3).tolist(),frames=j['frames'],
                        first_s=j['first'],last_s=j['last'],baseline_m=j['travel'],
                        observed_cells=[[int(z),int(x)] for (z,x),count in sorted(j['cells'].items()) if count>=2])
                   for i,j in enumerate(self.junctions) if j['frames']>=4 and j['travel']>=.7]
        return dict(lines=lines,repairs=[dict(kind=k,points=p.round(3).tolist()) for k,p in repairs],
                    openings=openings, junctions=confirmed, changed_repairs=changed_repairs,
                    changed_markings=changed_markings,changed_gaps=changed_gaps)
