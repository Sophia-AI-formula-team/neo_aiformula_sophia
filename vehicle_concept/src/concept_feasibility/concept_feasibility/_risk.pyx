"""All-times obstacle risk test. Stops only after proving the same ANY comparison."""
import numpy as np
cimport cython
from libc.math cimport sqrt, fabs, sin, cos, hypot
from cpython.pycapsule cimport PyCapsule_GetName, PyCapsule_GetPointer
from scipy.special import cython_special

# ndtr is a pure numeric C function. Obtain SciPy's double-precision Cython
# entry point with its precise signature; avoid taking Python's GIL for each CDF.
ctypedef double (*Cdf)(double, int) nogil except? -1e300
cdef Cdf gaussian
for name,capsule in cython_special.__pyx_capi__.items():
    if name.startswith("__pyx_fuse_") and name.endswith("ndtr") and "log" not in name and PyCapsule_GetName(capsule) == b"double (double, int __pyx_skip_dispatch)":
        gaussian = <Cdf>PyCapsule_GetPointer(capsule, PyCapsule_GetName(capsule))
        break
else:
    raise ImportError("Unsupported scipy.special double ndtr ABI")

# Linear interpolation of Phi at multiples of 1/512 has absolute error
# <= max|Phi''| * h*h/8 < 1.16e-7; the omitted 9-sigma tails are < 2e-19.
cdef double cdf_knots[9217]
for knot in range(9217):
    cdf_knots[knot] = gaussian((knot-4608)/512., 1)

cdef inline double cdf(double x,bint approximate) nogil except? -1e300:
    cdef double index,fraction
    cdef int i
    if not approximate:return gaussian(x,1)
    if x<=-9.:return 0.
    if x>=9.:return 1.
    index=x*512.+4608.
    i=<int>index
    if i>=9216:return cdf_knots[9216]
    fraction=index-i
    return cdf_knots[i]+fraction*(cdf_knots[i+1]-cdf_knots[i])


from cython.parallel cimport prange
cdef extern from "omp.h" nogil:
    int omp_get_thread_num()


@cython.cdivision(True)
cdef inline double rect(double z, double x, double sz, double sx, double rho,
                        double a, double b, double w,bint approximate) nogil except? -1e300:
    cdef double pa,pb,res,sd,m0,m1,closest,hx,hz,k
    if a>b or w<0:
        return 0
    pa=max(0.,cdf((b-z)/sz,approximate)-cdf((a-z)/sz,approximate))
    pb=max(0.,cdf((w-x)/sx,approximate)-cdf((-w-x)/sx,approximate))
    if pa==0. or pb==0.:return 0.
    if rho==0.:return pa*pb
    res=max(0.,1.-rho*rho)
    if res<=1e-12:
        return min(pa,pb)
    sd=sqrt(res)
    k=rho*sx/sz
    m0=x+k*(a-z);m1=x+k*(b-z)
    closest=max(min(m0,m1),min(max(m0,m1),0.))
    k=max(sx*sd,1e-12)
    hx=max(0.,cdf((w-closest)/k,approximate)-cdf((-w-closest)/k,approximate))
    k=rho*sz/sx
    m0=z+k*(-w-x);m1=z+k*(w-x)
    closest=max(min(m0,m1),min(max(m0,m1),(a+b)*.5))
    k=max(sz*sd,1e-12)
    hz=max(0.,cdf((b-closest)/k,approximate)-cdf((a-closest)/k,approximate))
    return min(pa,pb,pa*hx,pb*hz)


@cython.boundscheck(False)
@cython.wraparound(False)
cdef inline double decision_at(const double[:,:,:,:] terms,Py_ssize_t slot,Py_ssize_t t,double mid,
                               double limit,double front,double rear,double width) nogil except? -1e300:
    cdef Py_ssize_t k
    cdef double r,a,b,w,z,x,sz,sx,distz,distx,weight,total=0.,upper_tail=0.,approx_error=0.
    for k in range(terms.shape[2]):
        weight=terms[slot,t,k,0]
        if weight==0.:continue
        r=terms[slot,t,k,5]+mid
        a=-rear-r;b=front+r;w=width+r
        if a>b or w<0:continue
        z=terms[slot,t,k,1];x=terms[slot,t,k,2];sz=terms[slot,t,k,3];sx=terms[slot,t,k,4]
        # A rectangle is bounded by either marginal; frozen rectangle_upper
        # explicitly includes both. Round all three tail bounds upwards.
        distz=max(a-z,z-b);distx=fabs(x)-w
        if distz>6.*sz or distx>6.*sx:upper_tail+=weight*1.e-9
        elif distz>4.*sz or distx>4.*sx:upper_tail+=weight*.000032
        elif distz>3.*sz or distx>3.*sx:upper_tail+=weight*.00135
        else:
            total+=weight*rect(z,x,sz,sx,terms[slot,t,k,6],a,b,w,True)
            # Differences of CDFs add at most 2*eps, multiplying two unit
            # probabilities adds at most 4*eps. min/max never increase it.
            approx_error+=weight*5.e-7
    if total-approx_error>limit+1e-12 or total+approx_error+upper_tail<limit-1e-12:return total
    total=0.
    for k in range(terms.shape[2]):
        r=terms[slot,t,k,5]+mid
        total+=terms[slot,t,k,0]*rect(terms[slot,t,k,1],terms[slot,t,k,2],
              terms[slot,t,k,3],terms[slot,t,k,4],terms[slot,t,k,6],-rear-r,front+r,width+r,False)
    return total


@cython.boundscheck(False)
@cython.wraparound(False)
cdef double margin(const double[:,:,:,:] terms,const unsigned char[:,:] enabled,Py_ssize_t c,Py_ssize_t slot,
                   double limit,double front,double rear,double width,double[:,:] verified) nogil except? -1e300:
    # Legacy bisection [-8,12] with 20 steps returns the greatest accepted
    # point on this exact dyadic grid (0 if every point is rejected).
    # The greatest point safe at all times is the smallest timewise result.
    cdef int maximum=1048575,lo,hi,trial
    cdef double unit=20./1048576., value,worst,threshold
    cdef Py_ssize_t t,limiting=-1,nt=terms.shape[1]
    for t in range(nt):
        if enabled[t,c]:limiting=t
    while limiting>=0:
        lo=0;hi=maximum+1
        while hi-lo>1:
            trial=(lo+hi)//2
            if decision_at(terms,slot,limiting,-8.+trial*unit,limit,front,rear,width)>limit:hi=trial
            else:lo=trial
        maximum=lo
        if maximum==0:break
        # Only a time that rejects this point can lower the result. Choose the
        # strongest witness first, but verify every remaining enabled time.
        verified[c,limiting]=maximum
        threshold=-8.+maximum*unit; worst=limit;limiting=-1
        for t in range(nt):
            if not enabled[t,c] or verified[c,t]>=maximum:continue
            value=decision_at(terms,slot,t,threshold,limit,front,rear,width)
            if value<=limit:verified[c,t]=maximum
            elif value>worst:worst=value;limiting=t
    return -8.+maximum*unit


@cython.boundscheck(False)
@cython.wraparound(False)
@cython.cdivision(True)
cdef double forecast(const double[:,:,:] models,const double[:,:,:] centers,const double[:,:] z,
                     const double[:,:] x,const double[:,:] heading,const double[:,:] travel,
                     const double[:,:] turn,const unsigned char[:,:] active,Py_ssize_t i,
                     double[:,:,:,:] scratch,Py_ssize_t slot,double front,double rear,double width) nogil except? -1e300:
    cdef Py_ssize_t t,j,m,o
    cdef double c,s,dz,dx,al,side,gap,min_gap=100.,zz,xx,xz,budget,extra,sz,sx,cross,cl,r
    for t in range(z.shape[0]):
        if not active[t,i]:continue
        c=cos(heading[t,i]);s=sin(heading[t,i])
        for j in range(centers.shape[1]):
            dz=centers[t,j,0]-z[t,i];dx=centers[t,j,1]-x[t,i]
            al=dz*c+dx*s;side=dx*c-dz*s
            gap=hypot(al-max(-rear,min(front,al)),side-max(-width,min(width,side)))-centers[t,j,2]
            min_gap=min(min_gap,gap)
        for m in range(models.shape[1]):
            dz=models[t,m,1]-z[t,i];dx=models[t,m,2]-x[t,i]
            zz=models[t,m,3];xz=models[t,m,4];xx=models[t,m,5]
            if models[t,m,7]!=0.:
                budget=.01*(models[t,m,8]+travel[t,i])+.02*(models[t,m,9]+turn[t,i])*(models[t,m,10]+travel[t,i])
                extra=max(0.,budget*budget-models[t,m,11])
                zz=zz+extra;xx=xx+extra
            sz=sqrt(max(c*c*zz+2*c*s*xz+s*s*xx,1e-12))
            sx=sqrt(max(s*s*zz-2*c*s*xz+c*c*xx,1e-12))
            cross=c*s*(xx-zz)+(c*c-s*s)*xz
            scratch[slot,t,m,0]=models[t,m,0];scratch[slot,t,m,1]=dz*c+dx*s
            scratch[slot,t,m,2]=dx*c-dz*s;scratch[slot,t,m,3]=sz;scratch[slot,t,m,4]=sx
            scratch[slot,t,m,5]=models[t,m,6];scratch[slot,t,m,6]=max(-1.,min(1.,cross/(sz*sx)))
    return min_gap


@cython.boundscheck(False)
@cython.wraparound(False)
def path(const double[:,:,::1] models,const double[:,:,::1] centers,const double[:,:] z,const double[:,:] x,
         const double[:,:] heading,const double[:,:] travel,const double[:,:] turn,const unsigned char[:,::1] enabled,
         double limit,double front,double rear,double width,int workers):
    cdef Py_ssize_t nt=z.shape[0],n=z.shape[1],m=models.shape[1],i
    if models.shape[0]!=nt or models.shape[2]!=12 or centers.shape[0]!=nt or centers.shape[1]*2!=m or centers.shape[2]!=3 or workers<1:
        raise ValueError('invalid shared forecast geometry')
    for item in (x,heading,travel,turn,enabled):
        if item.shape[0]!=nt or item.shape[1]!=n:raise ValueError('invalid trajectory dimensions')
    results=np.empty((2,n))
    cdef double[:,:] out=results
    cdef double[:,:] verified=np.full((n,nt),-1e300)
    cdef double[:,:,:,:] scratch=np.empty((workers,nt,m,7))
    for i in prange(n,nogil=True,num_threads=workers,schedule='static'):
        out[0,i]=forecast(models,centers,z,x,heading,travel,turn,enabled,i,scratch,omp_get_thread_num(),front,rear,width)
        out[1,i]=margin(scratch,enabled,i,omp_get_thread_num(),limit,front,rear,width,verified)
    return results[0],results[1]
