"""All-times obstacle risk test. Stops only after proving the same ANY comparison."""
import numpy as np
cimport cython
from libc.math cimport sqrt, fabs
from cpython.pycapsule cimport PyCapsule_GetName, PyCapsule_GetPointer
from scipy.special import cython_special

# ndtr is a pure numeric C function. Obtain SciPy's double-precision Cython
# entry point with its precise signature; avoid taking Python's GIL for each CDF.
ctypedef double (*Cdf)(double, int) noexcept nogil
cdef Cdf gaussian
for name,capsule in cython_special.__pyx_capi__.items():
    if name.startswith("__pyx_fuse_") and name.endswith("ndtr") and "log" not in name and PyCapsule_GetName(capsule) == b"double (double, int __pyx_skip_dispatch)":
        gaussian = <Cdf>PyCapsule_GetPointer(capsule, PyCapsule_GetName(capsule))
        break
else:
    raise ImportError("Unsupported scipy.special double ndtr ABI")

cdef inline double ndtr(double x) noexcept nogil:
    return gaussian(x, 1)

from cython.parallel cimport prange


@cython.cdivision(True)
cdef inline double rect(double z, double x, double sz, double sx, double rho,
                        double a, double b, double w) noexcept nogil:
    cdef double pa,pb,res,sd,m0,m1,closest,hx,hz,k
    if a>b or w<0:
        return 0
    pa=max(0.,ndtr((b-z)/sz)-ndtr((a-z)/sz))
    pb=max(0.,ndtr((w-x)/sx)-ndtr((-w-x)/sx))
    res=max(0.,1.-rho*rho)
    if res<=1e-12:
        return min(pa,pb)
    sd=sqrt(res)
    k=rho*sx/sz
    m0=x+k*(a-z);m1=x+k*(b-z)
    closest=max(min(m0,m1),min(max(m0,m1),0.))
    k=max(sx*sd,1e-12)
    hx=max(0.,ndtr((w-closest)/k)-ndtr((-w-closest)/k))
    k=rho*sz/sx
    m0=z+k*(-w-x);m1=z+k*(w-x)
    closest=max(min(m0,m1),min(max(m0,m1),(a+b)*.5))
    k=max(sz*sd,1e-12)
    hz=max(0.,ndtr((b-closest)/k)-ndtr((a-closest)/k))
    return min(pa,pb,pa*hx,pb*hz)


@cython.boundscheck(False)
@cython.wraparound(False)
cdef double margin(const double[:,:,:,:] terms, const unsigned char[:,:] enabled,
                   Py_ssize_t c, double limit, double tail, double front, double rear, double width) noexcept nogil:
    cdef Py_ssize_t i,t,k,m=terms.shape[2]
    cdef double lo=-8.,hi=12.,mid,r,a,b,w,z,x,sz,sx,total
    cdef bint decisive,possible
    for i in range(20):
        mid=(lo+hi)*.5
        decisive=False
        for t in range(terms.shape[1]):
            if not enabled[t,c]: continue
            possible=False
            for k in range(m):
                r=terms[c,t,k,5]+mid
                a=-rear-r;b=front+r;w=width+r
                if a>b or w<0:continue
                z=terms[c,t,k,1];x=terms[c,t,k,2];sz=terms[c,t,k,3];sx=terms[c,t,k,4]
                # A univariate Gaussian tail bounds the entire rectangle. With
                # every component outside, summed upper bounds are below limit.
                if a-tail*sz<=z<=b+tail*sz and -w-tail*sx<=x<=w+tail*sx:
                    possible=True
                    break
            if not possible:continue
            total=0.
            for k in range(m):
                r=terms[c,t,k,5]+mid
                total=total + terms[c,t,k,0]*rect(terms[c,t,k,1],terms[c,t,k,2],
                    terms[c,t,k,3],terms[c,t,k,4],terms[c,t,k,6],-rear-r,front+r,width+r)
                if total>limit:
                    decisive=True
                    break
            if decisive:break
        if decisive:hi=mid
        else:lo=mid
    return lo


@cython.boundscheck(False)
@cython.wraparound(False)
def bisect(const double[:,:,:,::1] terms,const unsigned char[:,::1] enabled,
           double limit,double tail,double front,double rear,double width,int workers):
    cdef Py_ssize_t n=terms.shape[0],i
    if enabled.shape[1]!=n or enabled.shape[0]!=terms.shape[1] or terms.shape[3]!=7 or workers<1:
        raise ValueError('invalid forecast batch')
    result=np.empty(n)
    cdef double[:] out=result
    for i in prange(n,nogil=True,num_threads=workers,schedule='static'):
        out[i]=margin(terms,enabled,i,limit,tail,front,rear,width)
    return result
