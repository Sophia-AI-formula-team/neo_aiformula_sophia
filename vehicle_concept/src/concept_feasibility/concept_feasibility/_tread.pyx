"""Evaluate the ten frozen tyre contact samples, including support at the road ends."""
import numpy as np
from libc.math cimport sin,cos
cimport cython

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.cdivision(True)
def clearance(const double[:,:] road,const double[:] z,const double[:] x,const double[:] yaw,
              double uncertainty,const double[:] offsets):
    cdef Py_ssize_t n=z.shape[0],m=road.shape[0],last=road.shape[1]-1,i,j,side,low,high,mid
    if not m or last<3 or x.shape[0]!=n or yaw.shape[0]!=n or offsets.shape[0]!=10:
        raise ValueError("invalid tyre clearance input")
    bounds=np.empty((2,n));visible=np.empty(n,dtype=np.uint8)
    cdef double[:,::1] b=bounds
    cdef unsigned char[::1] known=visible
    cdef double c,s,depth,lateral,left,right,sigma_l,sigma_r,dz,delta,g,r,side_gap,side_lower
    cdef Py_ssize_t sr=min(last,4)
    with nogil:
        for i in range(n):
            c=cos(yaw[i]);s=sin(yaw[i]);known[i]=1
            for side in range(2):
                for j in range(5*side,5*side+5):
                    depth=z[i]-s*offsets[j];lateral=x[i]+c*offsets[j]
                    if depth<road[0,0] or depth>road[m-1,0]:
                        known[i]=0
                    if depth<=road[0,0]:
                        low=0;high=0
                    elif depth>=road[m-1,0]:
                        low=m-1;high=low
                    else:
                        low=0;high=m-1
                        while high-low>1:
                            mid=(low+high)//2
                            if road[mid,0]<=depth:low=mid
                            else:high=mid
                    left=road[low,1];right=road[low,2];sigma_l=road[low,3];sigma_r=road[low,sr]
                    if low!=high:
                        dz=road[high,0]-road[low,0];delta=depth-road[low,0]
                        left+=((road[high,1]-left)/dz)*delta
                        right+=((road[high,2]-right)/dz)*delta
                        sigma_l+=((road[high,3]-sigma_l)/dz)*delta
                        sigma_r+=((road[high,sr]-sigma_r)/dz)*delta
                    left=lateral-left;right=right-lateral
                    g=min(left,right);r=min(left-uncertainty*sigma_l,right-uncertainty*sigma_r)
                    if j==5*side:
                        side_gap=g;side_lower=r
                    else:
                        side_gap=max(side_gap,g);side_lower=max(side_lower,r)
                if side==0:
                    b[0,i]=side_gap;b[1,i]=side_lower
                else:
                    b[0,i]=min(b[0,i],side_gap);b[1,i]=min(b[1,i],side_lower)
    return bounds[0],bounds[1],visible.view(np.bool_)
