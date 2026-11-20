"""Reconstruct auto-command handoffs from a Concept field rosbag2 sqlite file."""
import argparse
import bisect
import csv
import json
import sqlite3
from pathlib import Path


def messages(connection, topic):
    query = ('SELECT timestamp, data FROM messages WHERE topic_id IN '
             '(SELECT id FROM topics WHERE name=? AND type=?) ORDER BY timestamp,id')
    for time_ns, blob in connection.execute(query, (topic, 'std_msgs/msg/String')):
        if blob[:2] != b'\x00\x01':
            raise ValueError('Expected little-endian CDR for ' + topic)
        length = int.from_bytes(blob[4:8], 'little')
        yield time_ns, json.loads(blob[8:8 + length - 1])


def capture_ns(record):
    stamp = record.get('stamp')
    if stamp is not None:
        return stamp['sec'] * 10**9 + stamp['nanosec']


def nearest(times, stamp):
    at = bisect.bisect_left(times, stamp)
    candidates = times[max(0, at - 1):at + 1]
    if candidates:
        found = min(candidates, key=lambda t: abs(t - stamp))
        # Legacy gate stamps were converted to epoch seconds in a float.
        if abs(found - stamp) <= 3000:
            return found


def trace(connection):
    decisions = {}
    for _, decision in messages(connection, '/concept/decision'):
        source = capture_ns(decision)
        if decision.get('auto_published') and source is not None:
            controller = decision.get('tracking_controller', {})
            decisions[source] = dict(plan_reason=decision.get('planner_reason'),
                                     selectable=sum(bool(c.get('selectable')) for c in decision['candidates']),
                                     requested_mps=decision['command']['linear_x_mps'],
                                     follower_vref=controller.get('reference_speed_mps'),
                                     owner=controller.get('owner'),
                                     publish_age_ms=decision.get('command_age_at_publish_ms'))
    sources = sorted(decisions)
    wheel_messages = iter(messages(connection, '/concept/vehicle_state'))
    next_wheel = next(wheel_messages, None)
    wheel = {}
    current = prev_ns = None
    rows = []
    for bag_ns, gate in messages(connection, '/concept/gate/status'):
        while next_wheel and next_wheel[0] <= bag_ns:
            _, wheel = next_wheel
            next_wheel = next(wheel_messages, None)
        reason = gate['reason']
        if gate['mode'] == 'auto' and reason == 'auto_command_received':
            if gate.get('converter_ready') is False:
                reason = 'waiting_converter_ack'
            elif gate.get('converter_ready') is None and gate['linear_mps'] == gate['yaw_rps'] == 0:
                reason = 'accepted_zero_or_converter_wait'
            elif gate['linear_mps'] == 0 and gate['yaw_rps'] == 0:
                reason = 'accepted_zero'
            else:
                reason = 'dispatch_nonzero'
        key = (gate['mode'], reason)
        if current is not None and 0 <= bag_ns - prev_ns <= 300_000_000:
            current['observed_ms'] += (bag_ns - prev_ns) / 1e6
        else:
            current = None
        prev_ns = bag_ns
        if current is not None and key == (current['mode'], current['reason']):
            current['last_status_ns'] = bag_ns
            current['reports'] += 1
            continue
        age = gate.get('auto_capture_age_s')
        clocks = gate.get('clock_trace') or {}
        exact = clocks.get('accepted') or {}
        source = exact.get('capture_ns') if gate['mode'] == 'auto' and reason != 'waiting_new_auto' else None
        if source is None and age is not None:
            source = nearest(sources, round((gate['t'] - age) * 1e9))
        can = clocks.get('can_publish') or {}
        state_age = None if wheel.get('t') is None else (gate['t']-wheel['t'])*1000
        current = dict(mode=key[0], reason=reason, start_bag_ns=bag_ns, last_status_ns=bag_ns,
                       observed_ms=0., reports=1, gate_start_ros_s=gate['t'], capture_ns=source,
                       capture_age_ms=None if age is None else round(age*1000, 2),
                       receipt_age_ms=None if gate.get('auto_receipt_age_s') is None else
                       round(gate['auto_receipt_age_s']*1000, 2),
                       applied_start_mps=gate['linear_mps'], ready_at_start=gate.get('converter_ready'),
                       gate_received_ns=exact.get('received_ns') if source == exact.get('capture_ns') else None,
                       gate_dispatch_ns=exact.get('first_dispatch_ns') if source == exact.get('capture_ns') else None,
                       last_can_publish_ns=can.get('stamp_ns'), last_can_payload_hex=can.get('data_hex'),
                       wheel_source_s=wheel.get('t'), wheel_sample_age_ms=state_age,
                       wheel_report_reason=wheel.get('reason'), left_rpm=wheel.get('leftRpm'),
                       right_rpm=wheel.get('rightRpm'),
                       **decisions.get(source, {}))
        rows.append(current)
    if not rows:
        raise ValueError('No gate status messages in this recording')
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    with sqlite3.connect(args.bag.resolve().as_uri() + '?mode=ro', uri=True) as db:
        rows = trace(db)
    columns = list(dict.fromkeys(k for row in rows for k in row))
    with args.output.open('w', newline='', encoding='utf-8-sig') as out:
        writer = csv.DictWriter(out, columns)
        writer.writeheader()
        writer.writerows(rows)
    print(f'{len(rows)} status intervals -> {args.output}')


if __name__ == '__main__':
    main()
