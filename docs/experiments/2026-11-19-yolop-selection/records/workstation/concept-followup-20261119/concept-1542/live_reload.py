import collections
import json
import os
import signal
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

rclpy.init()
node = rclpy.create_node('concept_paused_reload')
latest = {}
decisions = []
for topic in ('gate/status', 'vehicle_state', 'decision'):
    def receive(msg, topic=topic):
        latest[topic] = json.loads(msg.data)
        if topic == 'decision':
            decisions.append(latest[topic])
    node.create_subscription(String, '/concept/' + topic, receive, qos_profile_sensor_data)

def wait(predicate, duration):
    end = time.monotonic()+duration
    while not predicate() and time.monotonic()<end:
        rclpy.spin_once(node, timeout_sec=.1)
    if not predicate():
        raise RuntimeError('Timed out: '+repr({k:(v.get('mode'), v.get('status'), v.get('reason')) for k,v in latest.items()}))

def fresh_stop(disabled=False):
    gate=latest.get('gate/status', {})
    wheels=latest.get('vehicle_state', {})
    now=node.get_clock().now().nanoseconds/1e9
    return (gate.get('paused') is True and gate.get('manual_held') is False
            and gate.get('linear_mps') == gate.get('yaw_rps') == 0
            and wheels.get('valid') is True and wheels.get('leftRpm') == wheels.get('rightRpm') == 0
            and 0 <= now-gate.get('t', 0) < .3 and 0 <= now-wheels.get('t', 0) < .3
            and (not disabled or gate.get('output_enabled') is False))

def service(kind, topic, request):
    c=node.create_client(kind,topic)
    assert c.wait_for_service(timeout_sec=3)
    f=c.call_async(request)
    wait(f.done, 4.)
    assert f.result().success, f.result()
    return f.result().message

try:
    wait(lambda: fresh_stop() and 'decision' in latest, 8.)
    print(service(SetBool,'/concept/gate/enable_output',SetBool.Request(data=False)),flush=True)
    wait(lambda: fresh_stop(disabled=True), 3.)
    old=latest['decision']['session']
    proc=Path('/proc/64960/cmdline')
    if proc.exists():
        assert b'/lib/concept_drive/drive\0' in proc.read_bytes()
        os.kill(64960,signal.SIGINT)
    wait(lambda: latest['decision']['session']!=old and latest['decision']['status']=='ok', 25.)
    healthy=latest['decision']['session']
    start=time.monotonic()
    print('new driver restart:',service(Trigger,'/concept/drive/restart',Trigger.Request()),flush=True)
    latency_ms=round((time.monotonic()-start)*1000)
    wait(lambda: latest['decision']['session']!=healthy and latest['decision']['status']=='ok', 12.)
    session=latest['decision']['session']
    start=len(decisions)
    deadline=time.monotonic()+8.
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    print(json.dumps(dict(restart_round_trip_ms=latency_ms,
          new_session=session, decision_status=collections.Counter(d['status']+':'+d['reason'] for d in decisions[start:]),
          gate=latest['gate/status']),indent=2),flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
