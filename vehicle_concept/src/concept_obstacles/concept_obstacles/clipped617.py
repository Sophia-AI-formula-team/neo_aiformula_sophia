"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/clipped617.py. """

import numpy as np

from concept_road.vision import FOCAL, CX, CY, HEIGHT

def censored_support(track,detection,dt):
    left,top,right,bottom=detection['clipped']
    if not bottom or not (left != right) or top or track.hits<3 or track.presence_age>2*dt+1e-5:
        return None
    observed=np.asarray(detection['box'],float)
    det_area=np.maximum(0.,observed[2:]-observed[:2]).prod()
    if det_area<=0:return None
    nodes,weights=np.polynomial.hermite.hermgauss(3)
    estimates=[]
    for mean,cov in [(track.static_state,track.static_covariance),(track.state[:2],track.covariance[:2,:2])]:
        root=np.linalg.cholesky(cov+np.eye(2)*1e-12);mass=0.
        for a,wa in zip(nodes,weights):
            for b,wb in zip(nodes,weights):
                z,x=mean+root@np.array([a,b])*np.sqrt(2)
                if z<=.1:continue
                u,v=CX+x*FOCAL/z,CY+HEIGHT*FOCAL/z
                w,h=track.physical_size*FOCAL/z
                box=np.array([u-w/2,v-h,u+w/2,v])
                area=np.maximum(0.,np.minimum(box[2:],observed[2:])-np.maximum(box[:2],observed[:2])).prod()
                if area/det_area>=.5:mass+=wa*wb/np.pi
        estimates.append(float(mass))
    mode=np.asarray(estimates);weight=np.array([track.static_probability,1-track.static_probability])
    score=float(weight@mode)
    # One of the 3-point quadrature's side lobes must support this fragment.
    return dict(score=score,mode_support=mode>=1/6-1e-9,mode_scores=mode.tolist()) if score>=1/6-1e-9 else None
