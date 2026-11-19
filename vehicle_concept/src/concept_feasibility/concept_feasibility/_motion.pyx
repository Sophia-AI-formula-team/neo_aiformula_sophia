"""Fitted motor integration and exact integer-command interval search for V6.26."""
import numpy as np
from libc.math cimport log, rint, fabs, exp, sin, cos
cimport cython

cdef inline double sign(double v) nogil except? -1e300:
    return (v>0)-(v<0)

@cython.cdivision(True)
cdef inline int encoded(double v, double steer, bint physical, double *common, double *ratio) nogil except -1:
    cdef double w = -steer if physical else -<double><float>steer
    cdef double limit, gain, offset, bias, bounded, ros, right, left
    if not physical:
        v=<double><float>v
    if w>=0:
        limit=1.53908994;gain=3.15496243;offset=3.36664054;bias=-.0621119
    else:
        limit=1.68283261;gain=2.91627673;offset=-3.22124235;bias=1.6954783
    bounded=max(-bias+1e-6,min(limit-bias-1e-6,w))
    if physical:
        ros=offset-(1./gain)*log(limit/(bounded+bias)-1)
        if fabs(w)<=1e-9:
            ros=0.
            if fabs(v)<=1e-9:
                v=0.
    else:
        ros=offset-log(limit/(bounded+bias)-1)/gain
    right=v/.127+.6/.254*ros;left=v/.127-.6/.254*ros
    right=sign(right)*(fabs(right)/.834+2.76)
    left=sign(left)*(fabs(left)/.844+2.81)
    if physical:
        right=rint(right*(60/(2*3.141592653589793))*1.1)
        left=rint(left*(60/(2*3.141592653589793))*1.1)
    else:
        right=rint(right*(60/(2*3.141592653589793)*1.1))
        left=rint(left*(60/(2*3.141592653589793)*1.1))
    common[0]=(right+left)*.5
    ratio[0]=(right-left)/common[0] if fabs(common[0])>1 else 0.
    return 0

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.cdivision(True)
cdef inline double response(double common,double ratio,const double[:] p) nogil except? -1e300:
    cdef double hinge=max(-p[5],min(p[5],ratio))-p[4]
    cdef double scale=common/100
    return scale*p[1]+(scale*min(hinge,0.))*p[2]+(scale*max(hinge,0.))*p[3]

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.cdivision(True)
def advance(const double[:] speed,const double[:] yaw,const double[:] drive,const double[:] steer,
            double dt,const double[:] age,const double[:] pv,const double[:] pw,double delay,double braking,bint physical=False):
    cdef Py_ssize_t n=drive.shape[0],i
    if speed.shape[0]!=n or yaw.shape[0]!=n or steer.shape[0]!=n or age.shape[0]!=n or pv.shape[0]!=6 or pw.shape[0]!=6:
        raise ValueError("invalid motor trajectory batch")
    out=np.empty((3,n))
    cdef double[:,::1] result=out
    cdef double decay_v=exp(-dt/pv[0]),decay_w=exp(-dt/pw[0]),common,ratio,vg,wg,active
    with nogil:
        for i in range(n):
            if drive[i]>0:
                encoded(drive[i],steer[i],physical,&common,&ratio)
                vg=max(0.,response(common,ratio,pv));wg=steer[i] if physical else response(common,ratio,pw)
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
cdef inline Py_ssize_t search(const double[:] a,double value,bint inclusive) nogil except -1:
    cdef Py_ssize_t low=0,hi=a.shape[0],mid
    while low<hi:
        mid=(low+hi)//2
        if a[mid]<value or (inclusive and a[mid]==value):low=mid+1
        else:hi=mid
    return low


@cython.boundscheck(False)
@cython.wraparound(False)
def closest(const double[:] commands,const double[:] predicted,const long long[:] original,
            const long long[:] group_of,const long long[:] starts,Py_ssize_t zero,
            const double[:] desired,const double[:] lower,const double[:] upper,
            const double[:] at_lo,const double[:] at_hi):
    """Search every constant-response interval that intersects the steering bounds.

    A plateau contributes its first legal command in the original abs/yaw order.
    Explicitly retain that original index to reproduce np.argmin on exact ties.
    """
    cdef Py_ssize_t n=desired.shape[0],k=commands.shape[0],i,j,first,end,g,start,stop,best_index,p
    if not k or original.shape[0]!=k or group_of.shape[0]!=k or starts.shape[0]!=predicted.shape[0]+1 or lower.shape[0]!=n or upper.shape[0]!=n or at_lo.shape[0]!=n or at_hi.shape[0]!=n:
        raise ValueError("invalid steering intervals")
    result=np.empty(n)
    cdef double[:] output=result
    cdef double best,choice,error,l,h,d
    with nogil:
        for i in range(n):
            d=desired[i];l=lower[i];h=upper[i];best=1e300;choice=l;best_index=k
            first=search(commands,l,False);end=search(commands,h,True)
            if first<end:
                for g in range(group_of[first],group_of[end-1]+1):
                    error=fabs(predicted[g]-d)
                    if error>best:continue
                    start=max(first,starts[g]);stop=min(end,starts[g+1])
                    if zero<=start:p=start
                    elif zero>=stop:p=stop-1
                    elif original[zero-1]<original[zero]:p=zero-1
                    else:p=zero
                    if error<best or original[p]<best_index:
                        best=error;choice=commands[p];best_index=original[p]
            error=fabs(at_lo[i]-d)
            if error<best:best=error;choice=l
            if fabs(at_hi[i]-d)<best:choice=h
            output[i]=choice
    return result


@cython.boundscheck(False)
@cython.wraparound(False)
@cython.cdivision(True)
def integrate(const double[:,::1] initial, const double[:,:,:] controls, double dt,
              const double[:] pv, const double[:] pw, double delay, double braking,bint physical=False):
    """Integrate the full checked action sequence without crossing the Python boundary per 50 ms step.

    State columns are forward, right, heading, speed, gyro, distance, turn and brake age.
    """
    cdef Py_ssize_t steps=controls.shape[0], n=controls.shape[1], t,i,j
    if initial.shape[0]!=n or initial.shape[1]!=8 or controls.shape[2]!=2 or pv.shape[0]!=6 or pw.shape[0]!=6:
        raise ValueError('invalid controls or state')
    data=np.empty((steps+1,n,8))
    data[0]=initial
    cdef double[:,:,::1] out=data
    cdef double decay_v=exp(-dt/pv[0]),decay_w=exp(-dt/pw[0]),common,ratio,velocity,angular,vg,wg
    cdef double speed,yaw,age,elapsed,mid,travel,rotation
    with nogil:
        for t in range(steps):
            for i in range(n):
                speed=out[t,i,3];yaw=out[t,i,4];age=out[t,i,7]
                if controls[t,i,0]>0:
                    encoded(controls[t,i,0],controls[t,i,1],physical,&common,&ratio)
                    vg=max(0.,response(common,ratio,pv));wg=controls[t,i,1] if physical else response(common,ratio,pw)
                    velocity=vg+(speed-vg)*decay_v;angular=wg+(yaw-wg)*decay_w
                    elapsed=0.
                else:
                    elapsed=age+dt
                    velocity=max(0.,speed-braking*(max(0.,elapsed-delay)-max(0.,age-delay)))
                    angular=yaw*max(0.,min(1.,velocity/max(speed,1e-8))) if speed>1e-8 else 0.
                mid=out[t,i,2]+.25*(yaw+angular)*dt
                travel=(speed+velocity)*.5
                rotation=(yaw+angular)*.5
                out[t+1,i,0]=out[t,i,0]+travel*cos(mid)*dt
                out[t+1,i,1]=out[t,i,1]+travel*sin(mid)*dt
                out[t+1,i,2]=out[t,i,2]+rotation*dt
                out[t+1,i,3]=velocity;out[t+1,i,4]=angular
                out[t+1,i,5]=out[t,i,5]+(fabs(speed)+fabs(velocity))*.5*dt
                out[t+1,i,6]=out[t,i,6]+(fabs(yaw)+fabs(angular))*.5*dt
                out[t+1,i,7]=elapsed
    return data
