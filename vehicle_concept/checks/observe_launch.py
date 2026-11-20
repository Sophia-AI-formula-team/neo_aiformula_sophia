"""Find comparable, straight, from-rest command intervals in a recorded Concept bag."""
import argparse
from bisect import bisect_left, bisect_right
import csv
import sqlite3
from pathlib import Path

from trace_gate_clock import messages


def launch_trials(db):
    gates = [g for _, g in messages(db, '/concept/gate/status')]
    # Use the sensor stamp (not rosbag arrival) and retain invalid readings as gaps.
    unique = {}
    for bag_ns, wheel in messages(db, '/concept/vehicle_state'):
        stamp = wheel['t'] if wheel.get('t') is not None else bag_ns/1e9
        if stamp not in unique or not wheel['valid']:
            unique[stamp] = wheel
    wheels = sorted(unique.items())
    times = [t for t, _ in wheels]
    rest = prev = pulse = None
    for g in gates:
        t = g['t']
        v, yaw = g['linear_mps'], g['yaw_rps']
        mode = g['mode']
        continuous = (prev is not None and prev['mode'] == mode and 0 < t-prev['t'] <= .25)
        if not continuous or mode not in ('manual', 'auto'):
            pulse = rest = None
        if pulse is not None and (v <= 0 or abs(yaw) > .05):
            start, peak, area = pulse
            area += prev['linear_mps']*(t-prev['t'])
            prior = wheels[bisect_left(times, start-.25):bisect_right(times, start)]
            window = wheels[bisect_right(times, start):bisect_left(times, t)]
            if (len(prior) >= 3 and prior[-1][0]-prior[0][0] >= .12 and
                    all(w['valid'] and w['leftRpm'] == w['rightRpm'] == 0 for _, w in prior)):
                sequence = prior + window
                stamps = [stamp for stamp, _ in sequence] + [t]
                coverage = bool(len(window) >= 2 and all(w['valid'] for _, w in window)
                                and max(b-a for a, b in zip(stamps, stamps[1:])) <= .15)
                onset = None
                if coverage:
                    for (at, a), (_, b) in zip(window, window[1:]):
                        if min(a['leftRpm'], a['rightRpm'], b['leftRpm'], b['rightRpm']) > 0:
                            onset = round((at-start)*1000, 1)
                            break
                yield dict(mode=mode, start_ros_s=start, duration_ms=round((t-start)*1000, 1),
                           peak_mps=round(peak, 3), mean_mps=round(area/(t-start), 3),
                           feedback_covered=coverage, both_wheels_onset_ms=onset,
                           valid_wheel_samples=sum(w['valid'] for _, w in window))
            pulse = None
        if mode in ('manual', 'auto') and v == 0 and yaw == 0:
            if rest is None:
                rest = t
        else:
            if continuous and v > 0 and abs(yaw) <= .05:
                if pulse is not None:
                    start, peak, area = pulse
                    pulse = (start, max(peak, v), area + prev['linear_mps']*(t-prev['t']))
                elif rest is not None and t-rest >= .25:
                    pulse = (t, v, 0.)
            rest = None
        prev = g


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('out', type=Path)
    args = parser.parse_args()
    with sqlite3.connect(args.bag.resolve().as_uri()+'?mode=ro', uri=True) as db:
        rows = list(launch_trials(db))
    fields = ['mode','start_ros_s','duration_ms','peak_mps','mean_mps','feedback_covered',
              'both_wheels_onset_ms','valid_wheel_samples']
    with args.out.open('w', newline='', encoding='utf-8-sig') as out:
        writer = csv.DictWriter(out, fields)
        writer.writeheader()
        writer.writerows(rows)
    print('from rest:', len(rows), 'covered:', sum(r['feedback_covered'] for r in rows),
          'with two consecutive positive readings on both wheels:',
          sum(r['both_wheels_onset_ms'] is not None for r in rows))
