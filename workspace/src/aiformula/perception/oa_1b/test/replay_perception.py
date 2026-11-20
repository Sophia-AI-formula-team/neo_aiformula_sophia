"""Run the delivered node's model tap + state machine against the original three outer-circle captures."""
import argparse
from collections import Counter
import gzip
import json
from pathlib import Path

import cv2
from yolop_a3_runtime.color_semantics import classify_traffic_light
from yolop_a3_runtime.shared_runtime import SharedFPNPerception

from oa_1b.perception import ModelTap
from oa_1b.traffic_signal import SignalTracker


def replay(run,tap):
    track = SignalTracker()
    body = [json.loads(s) for s in (run/'trial/observations.jsonl').read_text().splitlines()]
    phases = {r['event']:r['utc_ms'] for r in body if r['event']!='sample'}
    events = [(r['utc_ms']*10**6,'body',r) for r in body]
    with gzip.open(run/'trial/topic-events.jsonl.gz','rt') as stream:
        events.extend((t,k,v) for t,k,v in map(json.loads,stream) if k in ('state','detector_frame'))
    counts = Counter()
    for t,k,v in sorted(events,key=lambda e:e[0]):
        if k=='body':
            # These are the sparse causal Unity observations retained by the run, not a recorded odom bag.
            track.odometry(v['x'],v['z'],v['speed'],t*1e-9)
        elif k=='state':
            track.status(v,t*1e-9)
        else:
            stamp=v['header']['stamp']
            ns=stamp['sec']*10**9+stamp['nanosec']
            ms=ns//10**6
            image=cv2.imread(str(run/'raw'/('%d.png'%ms)))
            assert image is not None
            base=tap.predict(image).detections_payload()
            counts['frames']+=1
            assert base['traffic_light_state']==v['traffic_light_state'] and base['red_pixels_count']==v['red_pixels_count']
            if not track.accept_stamp(ns,t):
                counts['stale']+=1
                continue
            fixed=track.correct(image,base,tap.candidates,ns,t*1e-9,classify_traffic_light)
            counts['after_stop_tracked']+=int(fixed['oa_1b']['tracked'] and ms>=phases['first_stop'])
            counts['distance_threshold_crossings_removed']+=int(base['red_pixels_count']>=50>fixed['red_pixels_count'])
            if ms>=phases['second_red']:
                counts['second_red_frames']+=1
                counts['stock_red']+=int(base['traffic_light_state']=='red')
                counts['oa1b_red']+=int(fixed['traffic_light_state']=='red')
            if phases['short_off']<=ms<phases['reset_red'] or phases['long_off']<=ms<phases['second_red']:
                counts['off']+=1
                counts['added_red_while_off']+=int(fixed['oa_1b']['rescued'])
    return counts


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('capture',type=Path)
    cli.add_argument('weights',type=Path)
    args=cli.parse_args()
    tap=ModelTap(SharedFPNPerception(args.weights,device='cuda',score_threshold=.5))
    try:
        output={r.name:replay(r,tap) for r in sorted(args.capture.glob('run[123]'))}
        assert sum(s['frames'] for s in output.values())==420
        print(json.dumps(output,indent=2))
    finally:
        tap.hook.remove()
