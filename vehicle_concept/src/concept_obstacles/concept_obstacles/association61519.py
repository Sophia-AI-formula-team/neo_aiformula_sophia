"""Runtime source extracted from the accepted V6.26 freeze: experiments/v615_recovery/association61519.py. """

import numpy as np

from scipy.optimize import linear_sum_assignment

from scipy.special import logsumexp

from concept_road.vision import FOCAL, CX, CY, HEIGHT

from concept_obstacles.ground import contact_point

def visible_probability(mean,covariance,size,center_offset=0.):
    nodes,weights=np.polynomial.hermite.hermgauss(3)
    root=np.linalg.cholesky(covariance+np.eye(2)*1e-12)
    probability=0.
    for a,wa in zip(nodes,weights):
        for b,wb in zip(nodes,weights):
            z,x=contact_point(mean+root@np.array([a,b])*np.sqrt(2), center_offset)
            if not .1<z<12:continue
            u,v=CX+x*FOCAL/z,CY+HEIGHT*FOCAL/z
            w,h=size*FOCAL/z
            if 2<u-w/2 and u+w/2<638 and 2<v-h and v<358:
                probability+=wa*wb/np.pi
    return float(probability)

def assign(tracks,detections,recall=.8,expected_new=1.):
    # Ground wedge whose complete cone feet may be detected. Units m^2.
    minimum_z=HEIGHT*FOCAL/(358-CY)
    area=640/(2*FOCAL)*(12**2-minimum_z**2)
    intensity=expected_new/area
    costs=np.full((len(tracks),len(detections)+len(tracks)),np.inf)
    for i in range(len(tracks)):costs[i,len(detections)+i]=0.
    details={};audit=[]
    for i,track in enumerate(tracks):
        means=[track.static_state,track.state[:2]]
        covariances=[track.static_covariance,track.covariance[:2,:2]]
        weights=np.array([track.static_probability,1-track.static_probability])
        pd=recall*np.array([visible_probability(m,p,track.physical_size,track.center_offset) for m,p in zip(means,covariances)])
        missed=max(1e-12,1-float(weights@pd))
        for j,det in enumerate(detections):
            if any(det['clipped']):continue
            r=det['R']*(4. if det['score']<.5 else 1.)
            logs=[];mahas=[]
            for m,p in zip(means,covariances):
                variance=p+r;innovation=det['point']-m
                maha=float(innovation@np.linalg.solve(variance,innovation));mahas.append(maha)
                logs.append(-np.log(2*np.pi)-.5*(np.linalg.slogdet(variance)[1]+maha))
            log_terms=np.log(np.maximum(weights*pd,1e-300))+logs
            log_likelihood=float(logsumexp(log_terms))
            cost=-(log_likelihood-np.log(missed*intensity))
            # Existing 99% 2D geometric gate remains a search bound. It is not
            # a reason to force this match if the missed/new explanation wins.
            if min(mahas)<9.21 and cost<0:costs[i,j]=cost
            info=dict(id=track.number,detection=j,p_static=float(weights[0]),pd=pd.tolist(),
                      missed_probability=missed,new_clutter_intensity_per_m2=intensity,
                      mode_mahalanobis2=mahas,mode_density=np.exp(logs).tolist(),
                      mode_match_contribution=np.exp(log_terms).tolist(),cost=cost,
                      accepted_gate=bool(np.isfinite(costs[i,j])))
            audit.append(info);details[i,j]=dict(info=info,pd=pd,R=r,maha=min(mahas))
    chosen=[]
    if len(tracks):
        ii,jj=linear_sum_assignment(costs)
        chosen=[(int(i),int(j),details[i,j]) for i,j in zip(ii,jj) if j<len(detections)]
    return chosen,audit,dict(recall=recall,expected_new_clutter_per_frame=expected_new,
        ground_area_m2=area,intensity_per_m2=intensity,calibrated=False,
        visibility='3x3 positive Gaussian quadrature of complete projected box')
