"""Isolate H12's 11/19 hold using original scores, selected IDs and trajectories.

Run with the original bag DB, the overlay replay JSON, and a result JSON path.
The recorder omitted the brake audit and the provenance of visible road rows.
Only accept regenerated brake evidence when the best forward option matches
the original action, full path, and risk margin. Overlay-derived visible_z is
explicitly approximate; no success in this calculation establishes a new lap.
"""
import argparse
from bisect import bisect_left
from collections import Counter
import json
from pathlib import Path
import sqlite3
import struct

import numpy as np

from concept_candidates.planner625 import MarkedPlanner
from concept_candidates.rear_reference617 import RecoveryReference
from concept_feasibility.response61515 import motor_profile
from concept_ranker.supported_launch import SupportedLaunch


def messages(db, topic):
    rows = db.execute('SELECT data FROM messages m JOIN topics t ON t.id=m.topic_id '
                      'WHERE t.name=? ORDER BY m.timestamp,m.id', (topic,))
    for (data,) in rows:
        size, = struct.unpack_from('<I', data, 4)
        yield json.loads(data[8:7+size])


def ns(d):
    return d['stamp']['sec']*10**9 + d['stamp']['nanosec']


def replay(bag, overlay_replay):
    with sqlite3.connect(bag.resolve().as_uri() + '?immutable=1', uri=True) as db:
        decisions = list(messages(db, '/concept/decision'))
        sensors = sorted((s['t'], s) for s in messages(db, '/concept/vehicle_state') if s['valid'])
    sensor_times = [s[0] for s in sensors]
    overlay = {r['capture_ns']: r for r in json.loads(overlay_replay.read_bytes())}
    planner = MarkedPlanner(dict(pace=2., uncertainty=1., yaw_limit=1.55))
    results = []
    for i, d in enumerate(decisions):
        if not d.get('command') or d['command']['linear_x_mps'] != 0:
            continue
        cand = d['candidates']
        r = {'i': i, 'capture_ns': ns(d)}
        results.append(r)
        if not d.get('ranking_active') or d['selected_index'] is None:
            r['result'] = 'original_no_ranker_or_no_selection'
            continue
        allowed = np.array([c['allowed'] and c['selectable'] for c in cand])
        movable = [c['index'] for c in cand if allowed[c['index']] and c['speed_mps'] > 0]
        if not movable:
            r['result'] = 'no_original_forward'
            continue
        top = max(movable, key=lambda j: cand[j]['score'])
        r['h12_top_forward'] = top
        context = overlay[ns(d)]
        if 'replay' not in context or len(d['objects']) or d['speed_estimate_mps'] >= .04:
            r['result'] = ('obstacle_present' if len(d['objects']) else
                           'not_stopped' if d['speed_estimate_mps'] >= .04 else 'missing_overlay')
            continue
        if any(m.get('ground_forward_right_m') is None or m['within_tracking_range']
               for m in d.get('obstacle_measurements', [])):
            r['result'] = 'obstacle_detection_pending'
            continue
        sensor_t = ns(d)/1e9 - d['sensor_age_s']
        pos = bisect_left(sensor_times, sensor_t)
        _, sample = min(sensors[max(0, pos-1):pos+1], key=lambda v: abs(v[0]-sensor_t))
        dt = (ns(d)-ns(decisions[i-1]))/1e9
        obs = RecoveryReference(np.array(d['road_rows']), np.empty((0, 77)),
                                d['speed_estimate_mps'], -sample['gyro'], dt,
                                np.array(d['heading']), None, None, reference=None)
        planner.p['pace'] = d['effective_speed_mps']
        # H12 at rest after a hold. Positive drive resets its own stop-age
        # during rollout. The original target guidance was not recorded.
        planner.stop_age = 30.
        applied = context['applied']
        with motor_profile(True, applied):
            planner.step(obs)
        audit = planner.last['audit']
        # The new policy can select a lower-scored forward option. Apply the
        # same replay identity requirement to all candidates it could use.
        matched = np.zeros(len(cand), dtype=bool)
        for j in movable:
            choice = cand[j]
            row = audit['table'][j]
            matched[j] = (np.array_equal(audit['paths'][j], choice['path'])
                          and np.allclose(row[:2], [choice['speed_mps'], choice['yaw_right_radps']],
                                          rtol=0, atol=1e-6)
                          and abs(row[5] - choice['clearance_m']) <= 1e-6
                          and bool(row[6]) == bool(choice['allowed']))
        if not matched[top]:
            r['result'] = 'missing_original_brake_audit_unmatched_rollout'
            continue
        r['matched_forward_ids'] = np.flatnonzero(matched).tolist()
        audit['nominal_feasible'] = np.asarray(audit['nominal_feasible'], dtype=bool) & matched
        # Use the exact recorded H12 inputs to the policy (scores, controls,
        # eligibility and baseline). Only the unlogged checked-rollout facts
        # for the top moving candidate come from the matched regeneration.
        table = np.array(audit['table'])
        for j, choice in enumerate(cand):
            table[j, [0, 1, 5]] = (choice['speed_mps'], choice['yaw_right_radps'], choice['clearance_m'])
        audit['table'] = table
        # road615.py gives inferred rows strictly positive allowance. An
        # applied paint edge has at least 0.04 m allowance (limits6251.py),
        # so an unchanged 0/0 within the blind section implies measured
        # material even in frames with paint edges further down the track.
        near = [z for z, _, _, sl, sr in d['road_rows']
                if -1.200001 <= z < .95 and sl == sr == 0]
        evidence = dict(stamp_s=ns(d)/1e9, applied_speed_mps=applied[0], obstacle_clear=True,
                        observed_near_z=near, visible_z=context['replay']['field_road']['visible_z'])
        pick, _, why = SupportedLaunch().decide(d['selected_index'], np.array([v['score'] for v in cand]),
                                               allowed, d['whitebox_index'], audit, obs, evidence)
        r.update(result=why['reason'], pick=pick, original_pick=d['selected_index'],
                 command_mps=float(table[pick, 0]), brake_margin_m=audit['brake_lower'][pick],
                 road_margin_m=cand[pick]['clearance_m'], visible_z=evidence['visible_z'])
    return dict(method='Original stationary scores and recorded trajectories; matched regenerated brake audit; '
                       'visible_z from JPEG reconstruction. Each frame starts from its real wheel/gate state.',
                counts=dict(Counter(r['result'] for r in results)), frames=results)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('bag', 'overlay_replay', 'output'):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    result = replay(args.bag, args.overlay_replay)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(result['counts'])
