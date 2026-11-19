import base64,collections,gzip,json,os,time
from datetime import datetime
from pathlib import Path
import cv2,numpy as np,rclpy
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist,TwistStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu,Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool,Trigger
from concept_yolop.frame_codec import encode_frame
from concept_yolop.geometry import HONDA
assert os.environ['ROS_DOMAIN_ID']=='207' and os.environ['ROS_LOCALHOST_ONLY']=='1'
def stamp(t):return datetime.fromtimestamp(t).astimezone().isoformat(timespec='milliseconds')
# Original serialized wheel, gyro and idle joystick samples captured together in the bag.
topics=[('/aiformula_sensing/vehicle_info',Frame),('/aiformula_sensing/vectornav/imu',Imu),('/aiformula_control/joy_node/joy_l2_throttle',Joy)]
with gzip.open(Path(__file__).with_name('stationary-inputs.json.gz'),'rt') as source:
 evidence=json.load(source)
t0=evidence['start_ns']
period=evidence['period_ns']/1e9
rows=[];original=collections.Counter()
for row in evidence['messages']:
 topic=row['topic'];kind=dict(topics)[topic];m=deserialize_message(base64.b64decode(row['cdr_b64']),kind)
 if kind is Frame:assert m.id==0x711 and not any(m.data)
 if kind is Imu:assert abs(m.angular_velocity.z)<.05
 if kind is Joy:assert not any(m.buttons)
 observed=m.header.stamp.sec*10**9+m.header.stamp.nanosec
 rows.append((row['offset_ns']/1e9,topic,m,max(0,t0+row['offset_ns']-observed)))
 original[topic]+=1
assert len(original)==3 and min(original.values())>=7
base=Path.home()/'aiformula_runtime/concept-field-1635/fixture'
pack=[]
for p in sorted(base.glob('*.png')):
 with np.load(p.with_suffix('.npz')) as f:
  pack.append(json.loads(encode_frame(0,0,'zed_left_camera_optical_frame',cv2.imread(str(p)),f['road'],f['lane'],f,HONDA)))
assert len(pack)==5
rclpy.init();n=rclpy.create_node('isolated_stationary_gate_probe')
pub={topic:n.create_publisher(kind,topic,qos_profile_sensor_data) for topic,kind in topics}
frames=n.create_publisher(String,'/concept/perception/frame',1)
states=[];gate=[];mot=[];can=[];auto=[];dec=[]
for topic,kind,fn in [
 ('/concept/vehicle_state',String,lambda m:states.append((time.time(),json.loads(m.data)))),
 ('/concept/gate/status',String,lambda m:gate.append((time.time(),json.loads(m.data)))),
 ('/concept/decision',String,lambda m:dec.append((time.time(),json.loads(m.data)['reason']))),
 ('/concept/auto_cmd',TwistStamped,lambda m:auto.append((time.time(),m.twist.linear.x,m.twist.angular.z))),
 ('/concept/internal/motor_cmd',Twist,lambda m:mot.append((time.time(),m.linear.x,m.angular.z))),
 ('/concept/internal/motor_can',Frame,lambda m:can.append((time.time(),int.from_bytes(m.data[:4],'little',signed=True),int.from_bytes(m.data[4:],'little',signed=True))))]:
 n.create_subscription(kind,topic,fn,qos_profile_sensor_data)
start=time.monotonic();sample=0;cycle=0;next_image=start+.3;img=0;publish_images=True
def pump():
 global sample,cycle,img,next_image
 now=time.monotonic()
 while now>=start+cycle+rows[sample][0]:
  offset,tp,msg,age=rows[sample]
  # Keep original each message's sample-to-receipt age; time-shift entire stationary segment.
  ns=time.time_ns()-int((now-start-cycle-offset)*1e9)-age
  msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,10**9)
  pub[tp].publish(msg);sample+=1
  if sample==len(rows):sample=0;cycle+=period
 if now>=next_image:
  if publish_images:
   msg=pack[img%5]; sec,ns=divmod(time.time_ns(),10**9); msg['stamp']=dict(sec=sec,nanosec=ns)
   frames.publish(String(data=json.dumps(msg,separators=(',',':'))));img+=1
  next_image=now+.2
 rclpy.spin_once(n,timeout_sec=.003)
def wait(fn,timeout):
 end=time.monotonic()+timeout
 while not fn() and time.monotonic()<end:pump()
 assert fn(),(len(states),len(gate),len(mot),len(auto),dec[-4:],gate[-1:] )
def run(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:pump()
def service(typ,name,req):
 cli=n.create_client(typ,'/concept/gate/'+name);wait(cli.service_is_ready,4);future=cli.call_async(req);wait(future.done,4)
 assert future.result().success,(name,future.result())
def select(log,t0,t1):return [r for r in log if t0<=r[0]<t1]
report=dict(version='recorded stationary sensors and five captured Honda frames, real-clock shifted, no plant dynamics and no detector rerun',bag=evidence['bag'],source_segment=stamp(t0/1e9),raw_messages_per_2s=original,loop_frames=5,windows=[])
try:
 wait(lambda:len(auto)>=4 and len(states)>2 and gate and states[-1][1]['valid'],22)
 assert not n.count_subscribers('/aiformula_control/motor_controller/reference_signal')
 assert not n.count_publishers('/aiformula_control/motor_controller/reference_signal')
 assert [s.node_name for s in n.get_subscriptions_info_by_topic('/concept/replay/reference_signal')]==['virtual_receiver']
 assert not any(name in ('zed_node','socket_can_sender','socket_can_receiver') for name,ns in n.get_node_names_and_namespaces())
 service(SetBool,'enable_output',SetBool.Request(data=True))
 for window in range(2):
  fresh=time.time()
  service(Trigger,'resume',Trigger.Request());run(7)
  end=time.time()
  r=select(mot,fresh,end); nz=[v for v in r if v[1] or v[2]]
  sent=select(auto,fresh,end); status=select(gate,fresh+.65,end)
  subset=select(states,fresh,end); cans=select(can,fresh,end); nonzero_can=[v for v in cans if v[1] or v[2]]
  report['windows'].append(dict(start=stamp(fresh),end=stamp(end),first_positive=[dict(t=stamp(stage[0]),v=stage[1],omega=stage[2]) for num,stage in enumerate(nz) if num==0 or stage[1:]!=nz[num-1][1:]][:7],
     peak_v=max([v for _,v,w in r],default=0.),peak_abs_yaw=max([abs(w) for _,v,w in r],default=0.),nonzero_motor=len(nz),can_frames=len(cans),nonzero_can=len(nonzero_can),first_nonzero_can=stamp(nonzero_can[0][0]) if nonzero_can else None,
     gate_modes=dict(collections.Counter(st['mode'] for _,st in status)),min_v=min([v for _,v,w in r],default=0),
     state_reasons=dict(collections.Counter(st['reason'] for _,st in subset)),gate_reasons=dict(collections.Counter(st['reason'] for _,st in status)),
     proposal_count=len(sent),state_count=len(subset),stationary_valid=sum(st['valid'] and st['leftRpm']==st['rightRpm']==0 for _,st in subset)))
  if window==0:
   service(Trigger,'pause',Trigger.Request());run(.7)
 publish_images=False;lost=time.time();preloss=gate[-1][1];run(1.45)
 following=select(gate,lost,time.time()); first_stopped=next((v for t,v in following if v['reason']=='auto_timeout'),None); last_nonzero=max((t for t,v,w in mot if (v or w) and t>=lost-.3),default=None)
 report['loss']=dict(start=stamp(lost),first_timeout=stamp(first_stopped['t']) if first_stopped else None,timeout_observed_ms=round(1000*(first_stopped['t']-lost),1) if first_stopped else None,last_nonzero_motor=stamp(last_nonzero) if last_nonzero else None,can_after_700ms=len(select(can,lost+.7,time.time())),before=preloss,last_gate=gate[-1][1],internal_after_700ms=len(select(mot,lost+.7,time.time())),nonzero_after_700ms=sum(bool(v or w) for _,v,w in select(mot,lost+.7,time.time())),can_nonzero_after_700ms=sum(bool(v or w) for _,v,w in select(can,lost+.7,time.time())))
 service(SetBool,'enable_output',SetBool.Request(data=False))
 report['decisions']=dict(collections.Counter(v for _,v in dec))
 print(json.dumps(report,indent=2,ensure_ascii=False),flush=True)
 assert all(w['first_positive'] and w['peak_v']==.5 and set(w['gate_modes'])=={'auto'} and w['stationary_valid']==w['state_count'] and w['nonzero_can']>0 for w in report['windows']), 'normal stationary windows'
 assert preloss['mode']=='auto' and report['loss']['last_gate']['reason']=='auto_timeout' and report['loss']['internal_after_700ms'] and report['loss']['can_after_700ms'] and not (report['loss']['nonzero_after_700ms'] or report['loss']['can_nonzero_after_700ms']), 'camera loss watchdog'
finally:
 n.destroy_node();rclpy.shutdown()
