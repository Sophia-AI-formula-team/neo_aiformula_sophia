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
    print(json.dumps(dict(gate=latest['gate/status'], wheels=latest['vehicle_state']),ensure_ascii=False),flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
