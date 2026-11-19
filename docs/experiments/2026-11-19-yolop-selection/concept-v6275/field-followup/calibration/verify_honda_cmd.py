"""Compare private-domain proposals with live-camera decisions using stationary test feedback."""
import collections
import json
import os
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

assert os.environ['ROS_DOMAIN_ID'] == '198' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
rclpy.init()
node = rclpy.create_node('honda_proposal_probe')
states = node.create_publisher(String, '/concept/vehicle_state', 10)
decisions, commands = {}, {}
statuses = collections.Counter()
reasons = collections.Counter()

def state():
    data = dict(t=time.time(), gyro=0., leftRpm=0., rightRpm=0., wheelRadius=.127,
                valid=True, source='isolated_standstill_test', rpm_encoding='forward_only_uint8')
    states.publish(String(data=json.dumps(data)))

def decision(message):
    result = json.loads(message.data)
    statuses[result['status']] += 1
    reasons[result['reason']] += 1
    if result['stamp'] and result.get('geometry_profile') == 'honda-v1.4.0-ground' and result['status'] == 'ok':
        stamp = result['stamp']
        decisions[stamp['sec'], stamp['nanosec']] = result

def command(message):
    stamp = message.header.stamp
    commands[stamp.sec, stamp.nanosec] = (message.header.frame_id, message.twist.linear.x, message.twist.angular.z)

node.create_timer(.025, state)
node.create_subscription(String, '/concept/decision', decision, qos_profile_sensor_data)
node.create_subscription(TwistStamped, '/concept/auto_cmd', command, qos_profile_sensor_data)
try:
    end = time.monotonic() + 40.
    while time.monotonic() < end and len(commands.keys() & decisions.keys()) < 25:
        rclpy.spin_once(node, timeout_sec=.05)
    common = sorted(commands.keys() & decisions.keys())
    for key in common:
        d = decisions[key]['command']
        assert commands[key] == ('base_footprint', d['linear_x_mps'], d['angular_z_radps'])
    speeds = [commands[k][1] for k in common]
    print(json.dumps(dict(feedback='synthetic stationary wheel and gyro', real_clock=True,
                          matched=len(common), published_observed=len(commands), positive=sum(v > 0 for v in speeds),
                          speed_range_mps=[min(speeds), max(speeds)] if speeds else [], status= dict(statuses),
                          reasons=dict(reasons), max_capture_age_s=max((decisions[k]['capture_age_s'] for k in common),default=None),
                          first_stamp=common[0] if common else None, last_stamp=common[-1] if common else None)), flush=True)
    assert common
finally:
    node.destroy_node()
    rclpy.shutdown()
