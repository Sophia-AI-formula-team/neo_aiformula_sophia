"""Read-only summary of small-speed commands and raw wheel feedback."""
import bisect
import collections
import json
import sqlite3
import struct
import sys
from datetime import datetime
from pathlib import Path

from can_msgs.msg import Frame
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

path = Path(sys.argv[1]).resolve()
db = sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)
db.execute('BEGIN')

def read(topic, kind):
    cursor = db.execute('SELECT timestamp,data FROM messages JOIN topics ON topic_id=topics.id '
                        'WHERE name=? ORDER BY timestamp', (topic,))
    for time_ns, raw in cursor:
        msg = deserialize_message(raw, kind)
        yield time_ns, json.loads(msg.data) if kind is String else msg

def stamp(t):
    return datetime.fromtimestamp(t / 1e9).astimezone().isoformat(timespec='milliseconds')

def band(speed):
    return ('reverse' if speed < 0 else 'forward') + (
        ' <=0.21' if abs(speed) <= .21 else ' 0.21–0.5' if abs(speed) <= .5 else ' >0.5'
    ) if speed else 'zero longitudinal'

def motion(messages):
    return collections.Counter(band(s.linear.x) for s in messages)

decisions = list(read('/concept/decision', String))
requests = list(read('/concept/internal/motor_cmd', Twist))
converter = [(t, struct.unpack('<ii', bytes(m.data))) for t,m in read('/concept/internal/motor_can', Frame)
             if m.id == 0x210 and m.dlc == 8]
wheel = [(t,m) for t,m in read('/aiformula_sensing/vehicle_info', Frame) if m.id == 0x711 and m.dlc == 8]
states = list(read('/concept/vehicle_state', String))
gates = list(read('/concept/gate/status', String))
cmd_times = [t for t,_ in requests]
raw_patterns = collections.Counter()
feedback = collections.defaultdict(collections.Counter)

for t,m in wheel:
    payload = bytes(m.data)
    if any(payload[i] for i in (1,2,3,5,6,7)):
        raw_patterns[payload.hex()] += 1

for t,state in states:
    i = bisect.bisect_right(cmd_times,t)
    j = bisect.bisect_left(cmd_times,t-180_000_000)
    if not i or j==0 or t-cmd_times[i-1] > 60_000_000:
        continue
    recent = [msg for _,msg in requests[j-1:i]]
    if len(recent)<4 or any(band(m.linear.x)!=band(recent[0].linear.x) or
                            abs(m.angular.z)>.05 for m in recent):
        continue
    feedback[band(recent[-1].linear.x)][
        'invalid' if not state['valid']
        else 'rpm_nonzero' if state['leftRpm'] or state['rightRpm'] else 'both_rpm_zero'
    ] += 1

blocked = collections.Counter()
for _,row in decisions:
    speed = (row.get('command') or {}).get('linear_x_mps')
    if speed is not None and speed<0:
        blocked[(round(speed,3), row['status'], row['reason'])] += 1

span = db.execute('SELECT min(timestamp),max(timestamp) FROM messages').fetchone()
print(json.dumps(dict(bag=path.parent.name, span=[stamp(t) for t in span if t is not None],
    planner_reverse=[[list(k),v] for k,v in blocked.items()],
    planning_status=collections.Counter(row['status']+' '+row['reason'] for _,row in decisions),
    exact_longitudinal=collections.Counter(f'{m.linear.x:.3f}' for _,m in requests),
    yaw_only=sum(abs(m.angular.z)>1e-8 and abs(m.linear.x)<1e-8 for _,m in requests),
    opposed_rpm_words=sum(a*b<0 for _,(a,b) in converter),
    both_rpm_negative=sum(a<0 and b<0 for _,(a,b) in converter),
    max_motor_yaw_rps=max((abs(m.angular.z) for _,m in requests),default=0),
    auto_speed=motion(m.twist for _,m in read('/concept/auto_cmd', TwistStamped)),
    motor_requests=motion(m for _,m in requests),
    motor_nonzero_frames=sum(any(v) for _,v in converter),
    negative_motor_frames=sum(a<0 or b<0 for _,(a,b) in converter),
    raw_wheel_frames=len(wheel),
    nonzero_higher_byte_frames=sum(raw_patterns.values()),
    top_higher_byte_patterns=[dict(hex=k,count=v,interpreted_as_int32_le=struct.unpack('<ii',bytes.fromhex(k)))
                              for k,v in raw_patterns.most_common(16)],
    paired_feedback=dict(feedback),
    gate=collections.Counter(s['mode']+' '+s['reason'] for _,s in gates)
), ensure_ascii=False,indent=2))
