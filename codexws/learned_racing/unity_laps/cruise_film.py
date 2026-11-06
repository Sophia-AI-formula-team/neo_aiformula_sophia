"""Measure a registered clear straight directly from the logged, monotone physical progress."""
import numpy as np

from three_view import label


class StraightGate:
    def __init__(self,records,bounds):
        self.lo,self.hi=bounds;self.signals=[];self.provenance=[]
        for r in records:
            rows=np.asarray(r.rows)
            if r.run["props"] or np.any(np.diff(rows[:,7])<-.00001):raise ValueError("Straight counter requires a clear, monotonically traversed gate")
            ts,s,v=rows[:,0],rows[:,7],rows[:,4]
            if not s[0]<=self.lo<self.hi<=s[-1]:raise ValueError("Recording did not cover the registered gate")
            start,end=np.interp([self.lo,self.hi],s,ts);t=np.r_[start,ts[(ts>start)&(ts<end)],end]
            travel=np.interp(t,ts,s);speed=np.interp(t,ts,v)
            self.signals.append((t,travel,speed))
            high=self.measure(r.run["seconds"],len(self.signals)-1)[1]
            mean=float(((speed[:-1]+speed[1:])*.5)@np.diff(t)/(end-start))
            self.provenance.append(dict(index=r.path.stem,fromM=self.lo,toM=self.hi,entryS=start,exitS=end,
                actualMps=mean,above4M=high,fractionPlannedAtLeast4=high/(self.hi-self.lo)))

    def measure(self,t,j):
        time,progress,speeds=self.signals[j]
        if t<=time[0]:return 0.,0.
        t=min(t,time[-1]);inside=time<t
        s=np.r_[progress[inside],np.interp(t,time,progress)];v=np.r_[speeds[inside],np.interp(t,time,speeds)]
        a,b=v[:-1],v[1:]
        widths=np.where((a>=4.)&(b>=4.),1.,0.)
        cross=(a<4.)!=(b<4.)
        frac=np.divide(4.-a,b-a,out=np.zeros_like(a),where=cross)
        widths[cross]=np.where(b[cross]>=4.,1-frac[cross],frac[cross])
        return float(s[-1]-s[0]),float(widths@np.diff(s))

    def stats(self,d,j,t,name,color,decision):
        dist,above=self.measure(t,j);y=920+j*149
        label(d,(1410,y),name,22,color,True)
        label(d,(1987,y),f"估计路面曲率 {decision['curvature']:+.3f} /m",17,"#49636b",anchor="rm")
        label(d,(1410,y+45),f"达到 4.0 m/s 的里程    {above:.2f} / {self.hi-self.lo:.0f} m",22,"#16333a",True)
        x,y0,w=1412,y+75,565;span=self.hi-self.lo
        d.rectangle((x,y0,x+w,y0+14),fill="#dbe7e0")
        if dist:d.rectangle((x,y0,x+dist/span*w,y0+14),fill="#c0916c")
        if above:d.rectangle((x,y0,x+above/span*w,y0+14),fill="#168582")

    def footer(self,d):
        label(d,(1410,1227),f"固定沿程 {self.lo:g}–{self.hi:g} m   ·   青色累计达标里程",20,"#49636b")
