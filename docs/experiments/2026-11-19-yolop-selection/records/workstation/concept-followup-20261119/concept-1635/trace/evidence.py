import bisect,collections,json,math,sqlite3,struct
from datetime import datetime
from pathlib import Path
from geometry_msgs.msg import Twist, TwistStamped
from can_msgs.msg import Frame
from std_msgs.msg import String
from rclpy.serialization import deserialize_message
p,=Path.home().glob('rosbags/concept/*163533*/*.db3')
db=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
def rows(topic,typ):
 for t,b in db.execute('select timestamp,data from messages join topics on topics.id=topic_id where topics.name=? order by timestamp',(topic,)):
  m=deserialize_message(b,typ)
  yield t/1e9,json.loads(m.data) if typ==String else m
def clk(t):return datetime.fromtimestamp(t).astimezone().isoformat(timespec='milliseconds')
gs=list(rows('/concept/gate/status',String));d=list(rows('/concept/decision',String));st=list(rows('/concept/vehicle_state',String));jt=[t for t,_ in gs]
can=[(t,struct.unpack('<ii',bytes(m.data))) for t,m in rows('/aiformula_sensing/vehicle_info',Frame) if m.id==0x711 and m.dlc==8]; wt=[t for t,_ in can]
cmd=list(rows('/concept/auto_cmd',TwistStamped)); motors=list(rows('/concept/internal/motor_cmd',Twist)); mt=[t for t,m in motors]
def stats(v):
 v=sorted(v);return dict(n=len(v), min=round(v[0],3), median=round(v[len(v)//2],3),p95=round(v[math.ceil(len(v)*.95)-1],3),max=round(v[-1],3)) if v else dict(n=0)
def mode(t):
 i=bisect.bisect_right(jt,t)-1
 return gs[i][1]['mode'] if i>=0 and t-jt[i]<.24 else 'unknown'
def nearest(ts,t):return max(0,min(len(ts)-1,bisect.bisect_right(ts,t)-1))
rej=collections.defaultdict(lambda:collections.Counter());pct=collections.Counter();track_src=collections.Counter()
examples=[]
phase={}
for t,r in d:
 why=r.get('planner_reason') or r['reason'];rm=rej[why];rm['decisions']+=1
 bad=r.get('rejected',{})
 rm['with_obstacle_rejected']+=bool(bad.get('obstacle',0));rm['with_road_rejected']+=bool(bad.get('road',0))
 for k,v in bad.items():rm[k]+=v
 if r.get('candidates'):rm['candidates']+=len(r['candidates'])
 for tr in r.get('tracks',[]):track_src[tr['source']]+=1
 if why!='feasible' and r.get('tracks') and len(examples)<16:
  examples.append([clk(t),why,bad,[(b['id'],b['source'],*[round(x,2) for x in b['position']],round(b['last_observation_age_s'],2)) for b in r['tracks']]])
 key=(r['session'],why)
 if key not in phase:phase[key]=[clk(t),clk(t),0]
 phase[key][1]=clk(t);phase[key][2]+=1
g_groups=[];prev=None
for t,g in gs:
 key=(g['mode'],g['reason'])
 if not g_groups or key!=prev:g_groups.append([t,t,key])
 g_groups[-1][1]=t; prev=key
speeds=collections.defaultdict(list)
for t,(rw,lw) in can:speeds[mode(t)].append((rw+lw)*math.pi*.127/60)
sample=[]
for t,s in st:
 if not s['valid']:
  sample.append(dict(time=clk(t),reason=s['reason'],gate=mode(t),nearby_can=[(round(1000*(tw-t),1),rpm) for tw,rpm in can if abs(t-tw)<.13],gate_status=[(round(ti-t,2),q['mode'],q['reason']) for ti,q in gs if abs(t-ti)<.24]))
p95=collections.defaultdict(list)
for t,r in d:
 if r['status']=='ok':
  for k,v in r['camera_timing_ms'].items():p95[k].append(v)
speed_break=collections.defaultdict(collections.Counter)
for t,m in motors:
 if mode(t)=='auto':speed_break[str(round(m.linear.x,2))][str(round(m.angular.z,2))]+=1
autom=[(t,m) for t,m in motors if mode(t)=='auto' and abs(m.linear.x)>1e-5]
print(json.dumps(dict(rejection_by_reason=rej,track_sources=track_src,track_examples=examples,real_wheel_derived_mps_by_gate_mode={k:stats(v) for k,v in speeds.items()},auto_motor_commands={k:dict(v) for k,v in speed_break.items()},gate_intervals_at_least_0p35s=[[clk(a),clk(b),round(b-a+.1,2),*key] for a,b,key in g_groups if b-a>.25],invalid_feedback=sample,camera_timing_ok_ms={k:stats(v) for k,v in p95.items()},phase=[[session,*v,why] for (session,why),v in phase.items()],auto_motor_nonzero=stats([m.linear.x for _,m in autom])),indent=2,ensure_ascii=False))
