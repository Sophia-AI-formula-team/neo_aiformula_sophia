"""Fitted motor integration and exact integer-command interval search for V6.26."""
import numpy as np
from libc.math cimport log, rint, fabs, exp, sin, cos
cimport cython

cdef inline double sign(double v) noexcept nogil:
    return (v>0)-(v<0)

@cython.cdivision(True)
cdef inline void encoded(double v, double steer, double *common, double *ratio) noexcept nogil:
    cdef double w = -<double><float>steer
    cdef double limit, gain, offset, bias, bounded, ros, right, left
    v=<double><float>v
    if w>=0:
        limit=1.53908994;gain=3.15496243;offset=3.36664054;bias=-.0621119
    else:
        limit=1.68283261;gain=2.91627673;offset=-3.22124235;bias=1.6954783
    bounded=max(-bias+1e-6,min(limit-bias-1e-6,w))
    ros=offset-log(limit/(bounded+bias)-1)/gain
    right=v/.127+.6/.254*ros;left=v/.127-.6/.254*ros
    right=rint(sign(right)*(fabs(right)/.834+2.76)*(60/(2*3.141592653589793)*1.1))
    left=rint(sign(left)*(fabs(left)/.844+2.81)*(60/(2*3.141592653589793)*1.1))
    common[0]=(right+left)*.5
    ratio[0]=(right-left)/common[0] if fabs(common[0])>1 else 0.

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.cdivision(True)
cdef inline double response(double common,double ratio,const double[:] p) noexcept nogil:
    cdef double hinge=max(-p[5],min(p[5],ratio))-p[4]
    cdef double scale=common/100
    return scale*p[1]+(scale*min(hinge,0.))*p[2]+(scale*max(hinge,0.))*p[3]

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.cdivision(True)
def advance(const double[:] speed,const double[:] yaw,const double[:] drive,const double[:] steer,
            double dt,const double[:] age,const double[:] pv,const double[:] pw,double delay,double braking):
    cdef Py_ssize_t n=drive.shape[0],i
    if speed.shape[0]!=n or yaw.shape[0]!=n or steer.shape[0]!=n or age.shape[0]!=n or pv.shape[0]!=6 or pw.shape[0]!=6:
        raise ValueError("invalid motor trajectory batch")
    out=np.empty((3,n))
    cdef double[:,::1] result=out
    cdef double decay_v=exp(-dt/pv[0]),decay_w=exp(-dt/pw[0]),common,ratio,vg,wg,active
    with nogil:
        for i in range(n):
            if drive[i]>0:
                encoded(drive[i],steer[i],&common,&ratio)
                vg=max(0.,response(common,ratio,pv));wg=response(common,ratio,pw)
                result[0,i]=vg+(speed[i]-vg)*decay_v
                result[1,i]=wg+(yaw[i]-wg)*decay_w
                result[2,i]=0.
            else:
                active=max(0.,age[i]+dt-delay)-max(0.,age[i]-delay)
                result[0,i]=max(0.,speed[i]-braking*active)
                result[1,i]=yaw[i]*max(0.,min(1.,result[0,i]/max(speed[i],1e-8))) if speed[i]>1e-8 else 0.
                result[2,i]=age[i]+dt
    return out[0],out[1],out[2]

@cython.boundscheck(False)
@cython.wraparound(False)
def closest(const double[:] commands,const double[:] predicted,const double[:] desired,
            const double[:] lower,const double[:] upper,const double[:] at_lo,const double[:] at_hi):
    cdef Py_ssize_t n=desired.shape[0],k=commands.shape[0],i,j
    if predicted.shape[0]!=k or lower.shape[0]!=n or upper.shape[0]!=n or at_lo.shape[0]!=n or at_hi.shape[0]!=n:
        raise ValueError("invalid reachable steering table")
    result=np.empty(n)
    cdef double[:] output=result
    cdef double best,choice,error,l,h,d
    with nogil:
        for i in range(n):
            d=desired[i];l=lower[i];h=upper[i];best=1e300;choice=l
            for j in range(k):
                if l<=commands[j]<=h:
                    error=fabs(predicted[j]-d)
                    if error<best:
                        best=error;choice=commands[j]
            error=fabs(at_lo[i]-d)
            if error<best:
                best=error;choice=l
            if fabs(at_hi[i]-d)<best:
                choice=h
            output[i]=choice
    return result
