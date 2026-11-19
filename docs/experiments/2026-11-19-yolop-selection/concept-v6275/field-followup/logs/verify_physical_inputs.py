"""Read-only ROS probe. Match the physical inputs, state, image decision and output."""
from collections import Counter
import json
import os
import time

import rclpy
from can_msgs.msg import Frame
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

assert os.environ['ROS_DOMAIN_ID'] == '198' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
rclpy.init()
node = rclpy.create_node('verify_physical_inputs')
wheels, gyros, states, decisions, commands = {}, {}, {}, {}, {}
status, reasons, state_status = Counter(), Counter(), Counter()

def stamp(msg):
    return msg.sec + msg.nanosec * 1e-9

def wheel(msg):
    if msg.id == 0x711 and msg.dlc == 8:
        wheels[stamp(msg.header.stamp)] = (int(msg.data[4]), int(msg.data[0]))

def imu(msg):
    gyros[stamp(msg.header.stamp)] = msg.angular_velocity.z

def state(msg):
    d = json.loads(msg.data)
    state_status[d['reason']] += 1
    if d['valid'] and d['source'] == 'can_0x711_low_byte_vectornav_ros_z':
        states[d['t']] = d

def decision(msg):
    d = json.loads(msg.data)
    status[d['status']] += 1
    reasons[d['reason']] += 1
    if d['status'] == 'ok' and d.get('geometry_profile') == 'honda-v1.4.0-ground':
        decisions[d['stamp']['sec'], d['stamp']['nanosec']] = d

def cmd(msg):
    commands[msg.header.stamp.sec, msg.header.stamp.nanosec] = (msg.twist.linear.x, msg.twist.angular.z)

for typ, topic, fn in [(Frame,'/aiformula_sensing/vehicle_info',wheel), (Imu,'/aiformula_sensing/vectornav/imu',imu),
                       (String,'/concept/vehicle_state',state), (String,'/concept/decision',decision),
                       (TwistStamped,'/concept/auto_cmd',cmd)]:
    node.create_subscription(typ, topic, fn, qos_profile_sensor_data)

try:
    end = time.monotonic() + 48
    while time.monotonic() < end and len(commands.keys() & decisions.keys()) < 30:
        rclpy.spin_once(node, timeout_sec=.1)
    chain = []
    matches = [k for k, d in states.items() if wheels.get(k) == (d['leftRpm'], d['rightRpm']) and
               gyros.get(d['gyro_t']) == d['gyro']]
    for k in commands.keys() & decisions.keys():
        d = decisions[k]
        capture = (k[0] * 10**9 + k[1]) / 1e9
        if (any(abs(capture - s - d['sensor_age_s']) < 1.e-6 for s in matches) and
                commands[k] == (d['command']['linear_x_mps'], d['command']['angular_z_radps'])):
            chain.append(k)
    print(json.dumps(dict(can711=len(wheels), imu=len(gyros), valid_state_samples=len(states),
                          state_reasons=dict(state_status), state_matching_both_raw=len(matches),
                          decision_status=dict(status), planner_reasons=dict(reasons),
                          auto_cmd=len(commands), complete_chain=len(chain),
                          positive=sum(commands[k][0] > 0 for k in chain),
                          wheel_rpm_pairs=sorted(set(wheels.values()))[:10],
                          gyro_z_range_rad_s=[min(gyros.values()),max(gyros.values())] if gyros else [])), flush=True)
    assert chain
finally:
    node.destroy_node()
    rclpy.shutdown()
