"""Summarize a finalized Concept bag without changing it or decoding camera images."""
import collections
import datetime
import json
import math
import sqlite3
import sys
from pathlib import Path

from can_msgs.msg import Frame
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Joy
from std_msgs.msg import String

bag = Path(sys.argv[1]).resolve()
db = sqlite3.connect(bag.as_uri() + '?mode=ro&immutable=1', uri=True)
def read(name, typ):
    for t, b in db.execute('SELECT timestamp,data FROM messages JOIN topics ON topic_id=topics.id WHERE topics.name=? ORDER BY timestamp',(name,)):
        m = deserialize_message(b, typ)
        yield t, json.loads(m.data) if typ == String else m
def ts(n):
    return datetime.datetime.fromtimestamp(n/1e9).astimezone().isoformat(timespec='milliseconds')
def counts(target):
    return dict(collections.Counter(target))
def span(values):
    values=sorted(values)
    if not values: return {}
    return dict(min=values[0], median=values[len(values)//2],p95=values[math.ceil(.95*len(values))-1],max=values[-1])
def stat(m):
    return (m['mode'],m['reason'],m['output_enabled'],m['paused'],m['manual_held'],m['estop'])
g=list(read('/concept/gate/status',String))
a=list(read('/concept/auto_cmd',TwistStamped))
d=list(read('/concept/decision',String))
w=list(read('/concept/vehicle_state',String))
j=list(read('/aiformula_control/joy_node/joy_l2_throttle',Joy))
m=list(read('/concept/internal/motor_cmd',Twist))
c=list(read('/concept/internal/motor_can',Frame))
events=[]
prev=None
for t,s in g:
    if stat(s)!=prev:
        events.append([ts(t),stat(s)])
    prev=stat(s)
def delta_ms(t,msg):
    return (t-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e6
out=dict(bag=bag.parent.name,gate=counts(str(stat(s)) for t,s in g),gate_transitions=events,
         range=[ts(n) for n in db.execute('SELECT min(timestamp),max(timestamp) FROM messages').fetchone()],
         decisions=counts(str((s['status'],s['reason'],s.get('geometry_profile'))) for _,s in d),
         auto_cmd=len(a),positive=sum(msg.twist.linear.x>0 for _,msg in a),
         nonzero=sum(bool(msg.twist.linear.x or msg.twist.angular.z) for _,msg in a),
         auto_speed_mps=span([msg.twist.linear.x for _,msg in a]),gate_limits=counts(s['max_auto_speed_mps'] for _,s in g),
         auto_capture_to_bag_ms=span([delta_ms(t,msg) for t,msg in a]),
         auto_gap_ms=span([(t-u)/1e6 for (u,_),(t,_) in zip(a,a[1:])]),
         feedback=counts(str((s['valid'],s['reason'])) for _,s in w),
         wheels=counts(str((s['leftRpm'],s['rightRpm'])) for _,s in w if s['valid']),
         joy=len(j),joy_pressed=counts(str([i for i,v in enumerate(msg.buttons) if v]) for _,msg in j),
         joy_age_ms=span([delta_ms(t,msg) for t,msg in j]),motor_cmd=len(m),
         motor_nonzero=sum(bool(msg.linear.x or msg.angular.z) for _,msg in m),
         motor_can=counts(str((hex(msg.id),bytes(msg.data).hex())) for _,msg in c))
capture = lambda msg: msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
transitions = [(t,s) for (u,b),(t,s) in zip(g,g[1:]) if b['reason']!=s['reason']]
pulses = []
streak = []
for t,msg in m:
    if msg.linear.x or msg.angular.z:
        streak.append(t)
    elif streak:
        pulses.append(dict(start=ts(streak[0]), samples=len(streak), until_zero_ms=(t-streak[0])/1e6))
        streak=[]
out['motor_command_pulses']=pulses
out['before_timeout']=[]
for t,s in transitions:
    if s['reason']!='auto_timeout': continue
    active = max((u for u,b in g if u<t and b['reason']=='auto_command_received'), default=None)
    if active is None: continue
    last = max(((u,v) for u,v in a if u<=active),key=lambda pair:pair[0], default=None)
    if last is None: continue
    following = next(((u,v) for u,v in a if u>active),None)
    out['before_timeout'].append(dict(last_active=ts(active), first_paused=ts(t), cmd_arrival=ts(last[0]),
                                    last_capture=ts(capture(last[1])), last_capture_expires=ts(capture(last[1])+500000000),
                                    following_cmd_arrival=ts(following[0]) if following else None))
ok=[s for _,s in d if s['status']=='ok']
out['successful_decision_ms']=dict(driver_processing=span([s['processing_ms'] for s in ok]),
    capture_to_driver_callback=span([s['capture_age_s']*1000-s['processing_ms'] for s in ok]))
out['uncovered_capture_expiry_intervals']=sum(t>capture(x)+500000000 for (u,x),(t,y) in zip(a,a[1:]))
out['final_can_topic_recorded']=bool(db.execute("select 1 from topics where name='/aiformula_control/motor_controller/reference_signal'").fetchone())
out['planner_reasons']=counts(d.get('planner_reason') or d['reason'] for _,d in d)
out['positive_by_gate_mode']=counts(s['mode'] for _,s in g if s.get('linear_mps',0.)>0)
out['hardware_working_samples']=dict(valid=sum(bool(x['valid']) for _,x in w),
      nonzero=sum(bool(x['leftRpm'] or x['rightRpm']) for _,x in w if x['valid']))
print(json.dumps(out,ensure_ascii=False,indent=2))
