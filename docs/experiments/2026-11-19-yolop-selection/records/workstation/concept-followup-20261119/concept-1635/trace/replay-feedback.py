"""Reconstruct state and causality using ROS bag receipt times and original capture stamps."""
import collections
import json
import math
import sqlite3
from datetime import datetime
from pathlib import Path
from can_msgs.msg import Frame
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from rclpy.serialization import deserialize_message
import concept_vehicle_state.core as honda
from concept_drive.transport import Feedback
p,=Path.home().glob('rosbags/concept/*163533*/*.db3')
db=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
topics={'/aiformula_sensing/vehicle_info':Frame,'/aiformula_sensing/vectornav/imu':Imu,'/concept/vehicle_state':String,'/concept/decision':String}
raw=[];dec=[]
def stamp(d):
 return d['sec']+d['nanosec']*1e-9 if isinstance(d,dict) else d.sec+d.nanosec*1e-9
for topic,typ in topics.items():
 for t,b in db.execute('select timestamp,data from messages join topics on topics.id=topic_id where topics.name=? order by timestamp',(topic,)):
  m=deserialize_message(b,typ)
  data=(t/1e9,topic,m)
  if typ==String:
   j=json.loads(m.data)
   if topic=='/concept/decision':
    if j['stamp']:
     dec.append((stamp(j['emitted_stamp'])-j['processing_ms']*.001,stamp(j['stamp']),j))
    continue
   data=(t/1e9,topic,j)
  if typ==Frame and m.id!=0x711: continue
  raw.append(data)
raw.sort(key=lambda e:e[0]); dec.sort()
def states(bounds):
 honda.MIN_WHEEL_RPM,honda.MAX_WHEEL_RPM=bounds
 v=honda.VehicleState(); reps=[]
 for t,tp,m in raw:
  if tp.endswith('vehicle_info'):
   v.wheels(m.data,stamp(m.header.stamp),t,t,frame_id=m.id,dlc=m.dlc,is_rtr=m.is_rtr,is_extended=m.is_extended,is_error=m.is_error)
  elif tp.endswith('imu'):
   if m.angular_velocity_covariance[0]<0 or not m.header.frame_id:
    v.gyros.clear()
   else:v.gyro(m.angular_velocity.z,stamp(m.header.stamp),t,t)
  else:reps.append((t,m,v.sample(t,t)))
 return reps
def clk(t):return datetime.fromtimestamp(t).astimezone().isoformat(timespec='milliseconds')
old=states((-255,341)); envelope=math.ceil((6.+.3*1.55)*60/(2*math.pi*.127)); new=states((-envelope,envelope))
idx=next(i for i,(t,s,q) in enumerate(old) if s['reason']=='unsupported_can_rpm_range')
tf=old[idx][0]
def short(s):return dict((k,s.get(k)) for k in ('valid','reason','leftRpm','rightRpm','t','gyro_t'))
def valid_series(which):
 fb=Feedback(); reports=[]
 i=0; sess=None; last=None; locked=False
 for cb,cap,d in dec:
  while i<len(old) and old[i][0]<=cb:
   if which=='recorded':s=old[i][1]
   elif which=='fixed_counterfactual':s=new[i][2] if i==idx else old[i][1]
   elif which=='reconstructed_old':s=old[i][2]
   else:s=new[i][2]
   fb.add(s);i+=1
  # Start all four variants at the same recorded successful pre-fault step;
  # callback order before this local experiment is not reconstructible exactly.
  if cb < tf-.85 or cb > tf+2.:
   continue
  if sess!=d['session']:
   sess=d['session']; last=cap; locked=False
   assert d['status']=='ok'
  try:
   state=fb.at(cap,.15); dt=round((cap-state['t'])*1e3,2);ok=True
  except ValueError:
   dt=None;ok=False
  if last is not None and cap-last>.5:locked=True
  if ok and not locked:last=cap
  if tf-.85<cb<tf+2.:
   reports.append(dict(image_arrival=clk(cb),capture=clk(cap),delta_ms=dt,has_causal_feedback=ok,continuity_locked=locked,observed_status=d['status'],observed_reason=d['reason']))
 return reports
matched=sum(s['reason']==q['reason'] for _,s,q in old)
actual=old[idx]; cf=new[idx]
print(json.dumps(dict(wheel_range=[-envelope,envelope],window_evidence=[dict(t=clk(t),recorded=short(s),old_from_raw=short(q),new_from_raw=short(new[i][2])) for i,(t,s,q) in enumerate(old) if tf-.08<t<tf+.13],all_state_report_reason_agreement=dict(n=len(old),matched=matched),timeseries={w:valid_series(w) for w in ('recorded','reconstructed_old','fixed_counterfactual','reconstructed_new')},method='All variants start at the same recorded successful step immediately before the fault. Captured receipts from bag approximate callback times. Original accepted state stream is preserved in fixed_counterfactual except one invalid RPM sample. Simulated lock only tests elapsed capture time and original causal feedback age/clearing policy.'),indent=2))
