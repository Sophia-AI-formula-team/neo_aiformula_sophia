#!/usr/bin/env python3
"""Observe onboard decision timing on the ROS wall clock, with motor output disabled."""
import argparse
from collections import Counter
import json
import math
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.)
    seconds = parser.parse_args().seconds
    if not math.isfinite(seconds) or seconds <= 0:
        parser.error('duration must be positive and finite')
    rclpy.init(args=[])
    node = rclpy.create_node('concept_measure_timing')
    statuses, reasons = Counter(), Counter()
    ages, planning, stamps = [], [], []

    def collect(msg):
        data = json.loads(msg.data)
        statuses[data['status']] += 1
        reasons[data['reason']] += 1
        stamp = data.get('stamp')
        if stamp:
            stamps.append(stamp['sec'] + stamp['nanosec'] * 1e-9)
        age = data.get('capture_age_s')
        if age is not None and age >= 0:
            ages.append(age)
        if data.get('command') is not None:
            planning.append(data['processing_ms'])

    node.create_subscription(String, '/concept/decision', collect, qos_profile_sensor_data)
    try:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.2, max(0., end - time.monotonic())))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    def pct(values):
        return dict(p50=float(np.median(values)), p95=float(np.percentile(values, 95)),
                    maximum=max(values)) if values else None
    print(json.dumps(dict(window_s=seconds, messages=sum(statuses.values()), statuses=statuses,
                          reasons=reasons, capture_to_decision_s=pct(ages), processing_ms=pct(planning),
                          capture_interval_s=pct(np.diff(stamps).tolist()), age_over_half_s=sum(t>.5 for t in ages)),
                     indent=2, ensure_ascii=False))
    return 0 if planning else 1


if __name__ == '__main__':
    raise SystemExit(main())
