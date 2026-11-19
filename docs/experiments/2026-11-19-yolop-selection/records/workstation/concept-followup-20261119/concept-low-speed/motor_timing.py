"""Pair the first recorded matching motor request with each changed auto command."""
import bisect
import json
import math
import sqlite3
import sys
from pathlib import Path
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.serialization import deserialize_message

db=sqlite3.connect(Path(sys.argv[1]).resolve().as_uri()+'?mode=ro',uri=True)
def read(topic, typ):
    for t,b in db.execute('SELECT timestamp,data FROM messages JOIN topics ON topics.id=topic_id '
                          'WHERE topics.name=? ORDER BY timestamp',(topic,)):
        yield t,deserialize_message(b,typ)
def value(m):
    return m.linear.x,m.angular.z
motor=list(read('/concept/internal/motor_cmd',Twist))
motor_time=[t for t,_ in motor]
auto=list(read('/concept/auto_cmd',TwistStamped))
source_to_motor=[]
dds_to_motor=[]
for index,(arrival,cmd) in enumerate(auto):
    u=value(cmd.twist)
    if u==(0.,0.):
        continue
    i=bisect.bisect_left(motor_time,arrival)
    if i and value(motor[i-1][1])==u:
        continue
    source=cmd.header.stamp.sec*10**9+cmd.header.stamp.nanosec
    end=min(auto[index+1][0] if index+1<len(auto) else arrival+100_000_000,arrival+100_000_000)
    for t,m in motor[i:]:
        if t>=end:break
        if value(m)==u:
            source_to_motor.append((t-source)/1e6)
            dds_to_motor.append((t-arrival)/1e6)
            break
def summary(x):
    v=sorted(x)
    return {k:round(v[math.ceil(q*len(v))-1],2) for k,q in [('median',.5),('p95',.95),('max',1)]} if v else {}
print(json.dumps(dict(paired_changed_nonzero=len(source_to_motor),capture_to_internal_motor_ms=summary(source_to_motor),
                      proposal_to_internal_motor_ms=summary(dds_to_motor)),indent=2))
