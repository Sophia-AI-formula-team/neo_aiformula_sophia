"""Summarize observed body speed by gate mode and inspect 4 m/s candidates."""
import bisect
import collections
import json
import math
import sqlite3
import struct
from pathlib import Path
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

def dist(a):
    a=sorted(a)
    return {k:round(a[math.ceil(len(a)*p)-1],3) for k,p in [('p50',.5),('p95',.95),('peak',1)]} if a else {}

def audit(path):
    db=sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)
    db.execute('begin')
    def read(topic,typ):
        for t,data in db.execute('select timestamp,data from messages join topics on topics.id=topic_id where topics.name=? order by timestamp',(topic,)):
            m=deserialize_message(data,typ)
            yield t,json.loads(m.data) if typ==String else m
    g=list(read('/concept/gate/status',String))
    gt=[t for t,m in g]
    d=list(read('/concept/decision',String))
    ac=list(read('/concept/auto_cmd',TwistStamped))
    req=list(read('/concept/internal/motor_cmd',Twist))
    wheel=[(t,struct.unpack('<ii',bytes(m.data))) for t,m in read('/aiformula_sensing/vehicle_info',Frame) if m.id==0x711 and m.dlc==8 and not (m.is_error or m.is_rtr or m.is_extended)]
    states=list(read('/concept/vehicle_state',String))
    modes=collections.defaultdict(list)
    over=[]
    for i,(t,(r,l)) in enumerate(wheel):
        j=bisect.bisect_right(gt,t)-1
        mode=g[j][1]['mode'] if j>=0 and 0<=t-gt[j]<250_000_000 else 'unknown'
        modes[mode].append((r+l)*math.pi*.127/60)
        if max(l,r)>255:over.append((i,t,r,l,mode))
    h=[r for t,r in d if r.get('candidates')]
    def ccount(row,key):
        return sorted(set(round(float(c['speed_mps']),3) for c in row['candidates'] if key is None or c[key]))
    def grouped(col):
        return dict(collections.Counter(str(row) for row in col))
    stats=dict(bag=path.parent.name, wheel_body_mps={k:dist(v) for k,v in modes.items()},over255=len(over),max_rpm=wheel and max(max(m) for _,m in wheel),
        drive_target=grouped(r.get('target_speed_mps') for _,r in d),effective=grouped(r.get('effective_speed_mps') for _,r in d),gate_cap=grouped(q.get('max_auto_speed_mps') for _,q in g),
        auto=grouped(round(v.twist.linear.x,3) for _,v in ac),motor=grouped(round(q.linear.x,3) for _,q in req) if '155042' in str(path) else dist([q.linear.x for _,q in req]),
        highest_offered=grouped(max(ccount(r,None)) for r in h), highest_allowed=grouped(max(ccount(r,'allowed'),default=None) for r in h),
        highest_selectable=grouped(max(ccount(r,'selectable'),default=None) for r in h),reject=grouped(r.get('rejected') for r in h) if '155042' in str(path) else None)
    if over:
        i=over[0][0]
        stats['first_crossing']=[(t,r,l) for t,(r,l) in wheel[max(i-4,0):i+7]]
        stats['peak_pair']=max(over,key=lambda x:max(x[2],x[3]))
        stats['high_modes']=grouped(x[-1] for x in over)
        stats['state_reasons']=grouped(q.get('reason') for _,q in states)
    return stats
print(json.dumps([audit(p) for p in sorted((Path.home()/'rosbags/concept').glob('*/*.db3'))],indent=2))
