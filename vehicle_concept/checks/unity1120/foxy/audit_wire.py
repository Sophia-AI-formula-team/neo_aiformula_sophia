"""Cross-check the saved host/ROS logs without using any judge data as model input."""
import argparse
import json
from pathlib import Path

from bridge import FRAME_KEYS, SCHEMA, verify_applied


def load(path):
    return [json.loads(line) for line in path.open()]


def check(host, ros):
    actual = load(host / 'exchange.jsonl')
    matched = load(ros / 'frames.jsonl')
    if not actual or len(actual) != len(matched):
        raise ValueError('host and ROS frame counts differ or input is empty')
    previous = None
    action_count = 0
    for seq, (row, saved) in enumerate(zip(actual, matched)):
        header, reply, decision = row['frame'], row['command'], saved['decision']
        if (header != saved['input'] or reply != saved['reply'] or set(header) != FRAME_KEYS
                or header['seq'] != seq or reply['seq'] != seq or header['schema'] != SCHEMA):
            raise ValueError('host and ROS records disagree at frame %d' % seq)
        verify_applied(header, seq, previous)
        if row['set_right_can'] != reply['right_can_rpm'] or row['set_left_can'] != reply['left_can_rpm']:
            raise ValueError('Unity setpoint register disagrees at frame %d' % seq)
        if decision['geometry_profile'] != 'honda-v1.4.0-ground':
            raise ValueError('wrong camera geometry')
        if decision['status'] == 'ok':
            cmd = decision['command']
            if (cmd['linear_x_mps'], cmd['angular_z_radps']) != (reply['linear_mps'], reply['angular_z_radps']):
                raise ValueError('ROS selected command differs at frame %d' % seq)
            action_count += 1
        previous = reply
    print(json.dumps(dict(scene=host.name, session=ros.name, frames=len(actual), setpoint_matches=len(actual),
                          next_frame_CAN_matches=len(actual)-1, decisions_with_twist=action_count,
                          h12_active=sum(r['decision']['ranking_active'] for r in matched))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('unity_output', type=Path)
    parser.add_argument('ros_session', type=Path)
    args = parser.parse_args()
    check(args.unity_output, args.ros_session)
