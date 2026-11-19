"""Summarize MapAssist state and command wiring from DDS or a finalized recording."""
import argparse
import collections
from datetime import datetime
import json
from pathlib import Path
import sqlite3
import time

from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message
from std_msgs.msg import Bool, Int32, String

ROOT = '/semantic_drive/map_assist_045'
A3 = '/aiformula_perception/yolop_a3'
MOTOR = '/aiformula_control/motor_controller/gate_status'
TOPICS = {
    ROOT + '/status': (String, 'map'),
    '/semantic_drive/map_status': (String, 'legacy'),
    ROOT + '/stop_wait_candidate': (String, 'legacy'),
    '/semantic_drive/motor_session': (String, 'session'),
    ROOT + '/pixel_stop_status': (String, 'pixel'),
    MOTOR: (String, 'motor'),
    A3 + '/traffic_light/state': (String, 'light'),
    A3 + '/red_pixels_count': (Int32, 'number'),
    '/flag_stop': (Bool, 'bool'),
    '/point_flag': (Bool, 'bool'),
    ROOT + '/diagnostic_cmd_vel': (Twist, 'twist'),
    ROOT + '/lya_cmd_vel': (Twist, 'twist'),
    '/aiformula_control/game_pad/cmd_vel': (Twist, 'twist'),
}
FIELDS = {
    'map': ('phase', 'lap', 'reason', 'recorded_samples', 'recorded_distance_m',
            'reference_length_m', 'odom_reference_ready', 'fusion_pending', 'route_matched',
            'progress_m', 'recording_points_valid', 'point_age_sec', 'odom_age_sec',
            'avoidance_active', 'branch_active', 'traffic_light_raw', 'traffic_light',
            'traffic_vote', 'traffic_mode', 'traffic_reason', 'traffic_stop_flag',
            'traffic_stop_committed', 'traffic_event_consumed', 'traffic_stop_flag_role',
            'smooth_plan_available', 'smooth_plan_reason', 'auto_trigger_reason',
            'velocity_gate_reason', 'path_output_allowed', 'ros_time_sec'),
    'legacy': ('traffic_policy', 'traffic_mode', 'traffic_stop_required', 'traffic_reason', 'ros_time_sec'),
    'session': ('active', 'sequence', 'stamp_ns'),
    'pixel': ('threshold', 'latest_pixels', 'age_ms', 'counts', 'detections_publishers', 'flag_publishers'),
    'motor': ('reason', 'semantic_active', 'manual_held', 'command', 'can_data'),
}
TRANSITIONS = {
    'map': ('phase', 'lap', 'reason', 'odom_reference_ready', 'fusion_pending', 'route_matched',
            'recording_points_valid', 'avoidance_active', 'branch_active', 'traffic_light',
            'traffic_mode', 'traffic_reason', 'traffic_stop_flag', 'smooth_plan_available',
            'velocity_gate_reason'),
    'legacy': ('traffic_mode', 'traffic_reason', 'traffic_stop_required'),
    'motor': ('reason', 'semantic_active', 'manual_held'),
    'session': ('active',),
}


def stamp(ns):
    return datetime.fromtimestamp(ns/1e9).astimezone().isoformat(timespec='milliseconds')


def decode(msg, kind):
    if kind == 'twist':
        return [msg.linear.x, msg.angular.z]
    if kind not in FIELDS:
        return msg.data
    try:
        data = json.loads(msg.data)
        selected = {key: data[key] for key in FIELDS[kind] if key in data}
        if kind == 'map':
            f = data.get('map_fusion') or {}
            selected['fusion'] = {key: f[key] for key in ('available', 'reason', 'state') if key in f}
        return selected
    except (ValueError, TypeError):
        return {'invalid_json': True}


class Collector:
    def __init__(self):
        self.states = {}

    def add(self, topic, received, msg):
        kind = TOPICS[topic][1]
        value = decode(msg, kind)
        state = self.states.setdefault(topic, dict(count=0, first=[stamp(received), value],
                                                  last=None, transitions=[], totals={}))
        state['count'] += 1
        state['last'] = [stamp(received), value]
        totals = state['totals']
        if kind == 'map' and not value.get('invalid_json'):
            for key in ('recorded_samples', 'recorded_distance_m', 'progress_m'):
                x = value.get(key)
                if x is not None:
                    totals['max_' + key] = max(x, totals.get('max_' + key, x))
            for key in ('odom_reference_ready', 'route_matched', 'traffic_stop_committed'):
                totals[key + '_true_samples'] = totals.get(key + '_true_samples', 0) + int(value.get(key) is True)
        if kind in TRANSITIONS:
            label = {key: value.get(key) for key in TRANSITIONS[kind]}
        elif kind in ('number', 'pixel'):
            pixel = value if kind == 'number' else value.get('latest_pixels')
            if pixel is not None:
                totals['max_pixels'] = max(pixel, totals.get('max_pixels', pixel))
                totals['at_or_above_600'] = totals.get('at_or_above_600', 0) + int(pixel >= 600)
            return
        elif kind == 'twist':
            totals['nonzero_samples'] = totals.get('nonzero_samples', 0) + int(any(value))
            totals['peak_linear_abs_mps'] = max(abs(value[0]), totals.get('peak_linear_abs_mps', 0))
            totals['peak_yaw_abs_rps'] = max(abs(value[1]), totals.get('peak_yaw_abs_rps', 0))
            return
        else:
            label = value
        timeline = state['transitions']
        if len(timeline) < 180 and (not timeline or label != timeline[-1][1]):
            timeline.append([stamp(received), label])


def from_recording(path):
    path = Path(path).resolve()
    collector = Collector()
    with sqlite3.connect(path.as_uri() + '?mode=ro', uri=True) as db:
        query = 'SELECT topics.name, timestamp, data FROM messages JOIN topics ON topics.id=topic_id'
        names = [row[0] for row in db.execute('SELECT name FROM topics')]
        selected = [topic for topic in names if topic in TOPICS]
        if selected:
            query += ' WHERE topics.name IN (' + ','.join('?' for _ in selected) + ') ORDER BY timestamp'
            for topic, t, raw in db.execute(query, selected):
                msg = deserialize_message(raw, TOPICS[topic][0])
                collector.add(topic, t, msg)
    return dict(bag=path.parent.name, topics=names, samples=collector.states)


def sample_live(seconds):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('mapassist_field_readback')
    collector = Collector()
    for topic, (message, _) in TOPICS.items():
        node.create_subscription(message, topic,
            lambda msg, topic=topic: collector.add(topic, time.time_ns(), msg), qos_profile_sensor_data)

    def names(infos):
        return sorted(x.node_namespace.rstrip('/') + '/' + x.node_name for x in infos
                      if x.node_name != 'mapassist_field_readback')

    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        endpoints = {topic: dict(publishers=names(node.get_publishers_info_by_topic(topic)),
                                 subscribers=names(node.get_subscriptions_info_by_topic(topic)))
                     for topic in TOPICS}
        return dict(collected=stamp(time.time_ns()), duration_s=seconds,
                    endpoints=endpoints, samples=collector.states)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag')
    parser.add_argument('--seconds', type=float, default=4)
    args = parser.parse_args()
    result = from_recording(args.bag) if args.bag else sample_live(args.seconds)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
