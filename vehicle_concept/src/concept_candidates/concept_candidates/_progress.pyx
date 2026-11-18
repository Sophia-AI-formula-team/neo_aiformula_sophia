"""Project every candidate onto all reference segments with the frozen ambiguity test."""
import numpy as np
from libc.math cimport sqrt, pow, fabs, INFINITY
cimport cython

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.cdivision(True)
def project(const double[:, :] points, const double[:, :] edges, const double[:] length,
            const double[:] arc, const double[:] z, const double[:] x,
            const double[:] coef, double sigma):
    cdef Py_ssize_t n=z.shape[0], m=length.shape[0], i,j,best
    if x.shape[0] != n or not m or points.shape[0] != m+1 or points.shape[1]!=2 or edges.shape[0]!=m or edges.shape[1]!=2 or arc.shape[0]!=m+1 or coef.shape[0]!=3:
        raise ValueError("invalid reference polyline")
    value_np=np.empty(n, dtype=np.float64); valid_np=np.empty(n,dtype=np.uint8)
    work=np.empty((3,m), dtype=np.float64)
    cdef double[:] value=value_np
    cdef unsigned char[:] valid=valid_np
    cdef double[:,::1] buf=work
    cdef double rz,rx,l2,t,b,dist,closest,f,zc,xc,slope,norm,curv,offset,refarc
    with nogil:
        for i in range(n):
            best=0
            closest=INFINITY
            for j in range(m):
                rz=z[i]-points[j,0]; rx=x[i]-points[j,1]
                l2=length[j]*length[j]
                t=(rz*edges[j,0]+rx*edges[j,1])/max(l2,1e-12)
                b=min(1.,max(0.,t))
                rz=z[i]-(points[j,0]+b*edges[j,0])
                rx=x[i]-(points[j,1]+b*edges[j,1])
                dist=rz*rz+rx*rx
                buf[0,j]=dist;buf[1,j]=t;buf[2,j]=b
                if dist<closest:
                    best=j;closest=dist
            b=buf[2,best];t=buf[1,best]
            refarc=arc[best]+b*length[best]
            value[i]=refarc
            valid[i]=not ((best==0 and t<=0) or (best==m-1 and t>=1))
            if not valid[i]:
                continue
            for j in range(m):
                dist=buf[0,j]
                if (j==0 or dist<=buf[0,j-1]) and (j==m-1 or dist<=buf[0,j+1]):
                    if fabs((arc[j]+buf[2,j]*length[j])-refarc)>.16 and sqrt(dist)<=sqrt(closest)+sigma:
                        valid[i]=0
                        break
            if not valid[i]:
                continue
            zc=points[best,0]+b*edges[best,0];xc=points[best,1]+b*edges[best,1]
            slope=coef[1]+2*coef[2]*zc
            norm=sqrt(1+slope*slope)
            curv=2*coef[2]/pow(1+slope*slope,1.5)
            offset=(z[i]-zc)*(-slope/norm)+(x[i]-xc)*(1/norm)
            valid[i]=1-curv*offset>fabs(curv)*sigma
    return value_np,valid_np.view(np.bool_)
