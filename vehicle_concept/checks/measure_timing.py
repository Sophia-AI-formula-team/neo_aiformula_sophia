#!/usr/bin/env python3
"""Passively measure published camera, perception, decision and Twist timing on a ROS host."""
import argparse
from collections import Counter
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


CAMERA = '/aiformula_sensing/zed_node/left_image/undistorted'
NS = 1_000_000_000


def timestamp(sec, nanosec):
    if type(sec) is int and type(nanosec) is int and sec >= 0 and 0 <= nanosec < NS:
        value = sec * NS + nanosec
        if value > 0:
            return value
    raise ValueError('invalid source stamp')


def percentiles(values):
    if not len(values):
        return None
    p50, p95, p99, maximum = np.percentile(values, [50, 95, 99, 100])
    return dict(p50=float(p50), p95=float(p95), p99=float(p99), maximum=float(maximum))


class Stream:
    def __init__(self):
        self.first = {}
        self.duplicates = self.out_of_order = 0
        self.latest = -1
        self.forward = {}

    def observe(self, stamp, receipt_ros, receipt_wall):
        if stamp in self.first:
            self.duplicates += 1
        else:
            if stamp > self.latest:
                self.forward[stamp] = receipt_ros, receipt_wall
            else:
                self.out_of_order += 1
            self.first[stamp] = receipt_ros, receipt_wall
            self.latest = max(self.latest, stamp)

    def summary(self):
        stamps = sorted(self.first)
        span_s = (stamps[-1]-stamps[0])/NS if len(stamps) > 1 else 0.
        return dict(unique=len(stamps), duplicates=self.duplicates, out_of_order=self.out_of_order,
                    advancing=len(self.forward), source_hz=(len(stamps)-1)/span_s if span_s else None,
                    source_intervals_s=percentiles(np.diff(stamps)/NS),
                    first_source_ns=stamps[0] if stamps else None, last_source_ns=stamps[-1] if stamps else None)


class Measurement:
    def __init__(self):
        self.camera, self.perception, self.decisions, self.twists = (Stream() for _ in range(4))
        self.statuses, self.reasons, self.invalid = Counter(), Counter(), Counter()
        self.status_by_stamp = {}
        self.decision_stamps, self.decision_ages, self.processing = [], [], []
        self.rate_counters = {}
        self.rate_skipped = 0
        self.rate_pairs = 0
        self.planner_rates = set()

    def decision(self, data, now, mono):
        status, reason = data['status'], data['reason']
        if not isinstance(status, str) or not isinstance(reason, str):
            raise ValueError('invalid decision status')
        self.statuses[status] += 1
        self.reasons[reason] += 1
        session, counter = data.get('session'), data.get('rate_skipped_total')
        if isinstance(session, str) and type(counter) is int and counter >= 0:
            previous = self.rate_counters.get(session)
            if previous is not None and counter >= previous:
                self.rate_skipped += counter - previous
                self.rate_pairs += 1
            self.rate_counters[session] = counter
        hz = data.get('planning_hz')
        if type(hz) in (int, float) and math.isfinite(hz) and hz > 0:
            self.planner_rates.add(hz)
        if data.get('stamp') is not None:
            stamp = timestamp(**data['stamp'])
            self.decisions.observe(stamp, now, mono)
            self.decision_stamps.append(stamp)
            self.status_by_stamp.setdefault(stamp, set()).add(status)
        age = data.get('capture_age_s')
        if type(age) in (int, float) and math.isfinite(age) and age >= 0:
            self.decision_ages.append(age)
        cost = data.get('processing_ms')
        if data.get('command') is not None and type(cost) in (int, float) and math.isfinite(cost):
            self.processing.append(cost)

    def report(self, seconds, planning_hz, clock_info, end_mono):
        sources = self.camera.first
        twists = self.twists.forward
        matched = sorted(twists.keys() & sources.keys())
        clock_deltas = [(twists[s][0]-s)/NS for s in matched]
        valid_ages = [x for x in clock_deltas if x >= 0]
        arrivals = sorted(wall for _, wall in twists.values())
        intervals = np.diff(arrivals)/NS
        # This metric is observable even when source stamps use a different clock.
        # Count complete time bins starting at the first image this probe saw.
        missing = bins = None
        if sources:
            start = min(wall for _, wall in sources.values())
            bins = int((end_mono-start)*planning_hz/NS)
            occupied = {int((t-start)*planning_hz/NS) for t in arrivals if t >= start}
            missing = bins - sum(0 <= k < bins for k in occupied)
        seen = self.perception.first.keys()
        decision = self.decisions.first.keys()
        eligible = (clock_info['operator_confirmed_vehicle_clock'] and not clock_info['use_sim_time']
                    and not clock_info['clock_publishers'] and clock_info['zed_publisher_present']
                    and bool(matched) and all(x >= 0 for x in clock_deltas))
        return dict(window_s=seconds, messages=sum(self.statuses.values()), statuses=self.statuses,
                    reasons=self.reasons, capture_to_decision_s=percentiles(self.decision_ages),
                    processing_ms=percentiles(self.processing),
                    capture_interval_s=percentiles(np.diff(self.decision_stamps)/NS),
                    age_over_half_s=sum(x > .5 for x in self.decision_ages),
                    clock=dict(clock_info, eligible_for_vehicle_timing_evaluation=eligible),
                    camera=self.camera.summary(), perception=self.perception.summary(), decisions=self.decisions.summary(),
                    auto_cmd=self.twists.summary(), invalid_messages=dict(self.invalid),
                    observed_new_twist_hz=(len(arrivals)-1)*NS/(arrivals[-1]-arrivals[0])
                    if len(arrivals)>1 and arrivals[-1]>arrivals[0] else None,
                    camera_without_perception=len(sources.keys()-seen), perception_without_camera=len(seen-sources.keys()),
                    perception_without_decision=len(seen-decision),
                    rate_skipped_confirmed=self.rate_skipped if self.rate_pairs else None,
                    rate_counter_pairs=self.rate_pairs, driver_planning_hz=sorted(self.planner_rates),
                    status_unique_stamps={s: sum(s in v for v in self.status_by_stamp.values()) for s in self.statuses},
                    ok_decisions_without_twist=sum('ok' in ss and stamp not in twists
                                                   for stamp, ss in self.status_by_stamp.items()),
                    cmd_with_camera=len(matched), cmd_without_camera=len(twists)-len(matched),
                    cmd_with_negative_age=len(clock_deltas)-len(valid_ages),
                    capture_to_observed_twist_s=percentiles(valid_ages),
                    capture_to_twist_over_200ms=sum(v > .200 for v in valid_ages),
                    observed_twist_intervals_s=percentiles(intervals),
                    twist_intervals_over_200ms=int(np.sum(intervals > .200)),
                    planning_window_hz=planning_hz, complete_periods=bins, periods_without_new_twist=missing)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--planning-hz', type=float, default=5., help='Expected Twist update rate for time bins')
    parser.add_argument('--vehicle-clock-confirmed', action='store_true',
                        help='Operator verified physical ZED capture and ROS host share the vehicle clock')
    args = parser.parse_args()
    if not all(math.isfinite(x) and x > 0 for x in (args.seconds, args.planning_hz)):
        parser.error('duration and planning rate must be positive and finite')
    rclpy.init(args=[])
    node = rclpy.create_node('concept_measure_timing')
    data = Measurement()

    def collect(kind, msg):
        wall, now = time.monotonic_ns(), node.get_clock().now().nanoseconds
        try:
            if kind == 'decisions':
                data.decision(json.loads(msg.data), now, wall)
                return
            if kind == 'perception':
                stamp = timestamp(**json.loads(msg.data)['stamp'])
            else:
                stamp = timestamp(msg.header.stamp.sec, msg.header.stamp.nanosec)
                if kind == 'twists' and not all(math.isfinite(v) for v in (msg.twist.linear.x, msg.twist.angular.z)):
                    raise ValueError('invalid Twist')
            getattr(data, kind).observe(stamp, now, wall)
        except (AttributeError, KeyError, TypeError, ValueError):
            data.invalid[kind] += 1

    for kind, msg_type, topic in [('camera', Image, CAMERA), ('perception', String, '/concept/perception/frame'),
                                  ('decisions', String, '/concept/decision'), ('twists', TwistStamped, '/concept/auto_cmd')]:
        node.create_subscription(msg_type, topic, lambda msg, k=kind: collect(k, msg), qos_profile_sensor_data)
    try:
        end = time.monotonic() + args.seconds
        next_graph = 0.
        camera_sources = set()
        sim_clock, clock_publishers = False, 0
        while time.monotonic() < end:
            if time.monotonic() >= next_graph:
                camera_sources.update(info.node_namespace.rstrip('/') + '/' + info.node_name
                                      for info in node.get_publishers_info_by_topic(CAMERA))
                clock_publishers = max(clock_publishers, node.count_publishers('/clock'))
                sim_clock = sim_clock or bool(node.get_parameter('use_sim_time').value)
                next_graph = time.monotonic() + .5
            rclpy.spin_once(node, timeout_sec=min(.2, max(0., end-time.monotonic())))
        end_mono = time.monotonic_ns()
        clock_info = dict(operator_confirmed_vehicle_clock=args.vehicle_clock_confirmed,
                          use_sim_time=sim_clock, clock_publishers=clock_publishers,
                          camera_publishers=sorted(camera_sources),
                          zed_publisher_present=camera_sources == {'/aiformula_sensing/zed_node'})
        print(json.dumps(data.report(args.seconds, args.planning_hz, clock_info, end_mono),
                         indent=2, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # Preserve historical collection-success exit code. This is not a hardware acceptance result.
    return 0 if data.processing else 1


if __name__ == '__main__':
    raise SystemExit(main())
