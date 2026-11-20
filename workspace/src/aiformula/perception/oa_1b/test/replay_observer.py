"""Run the delivered node's model tap + state machine against the original three outer-circle captures."""
import argparse
from collections import Counter
import gzip
import json
import time

import torch
from pathlib import Path

import cv2
from yolop_a3_runtime.color_semantics import classify_traffic_light

from oa_1b.observer import Candidates, check_runtime
from oa_1b.traffic_signal import SignalTracker


def replay(run, model):
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
            base = v
            counts['frames'] += 1
            if not track.accept_stamp(ns,t):
                counts['stale']+=1
                continue
            def infer():
                began = time.monotonic()
                boxes = model.infer(image)
                inference_ms.append((time.monotonic() - began) * 1000)
                counts['inference_calls'] += 1
                return boxes

            if base['traffic_light_state'] == 'unknown':
                candidates = infer
            else:
                candidates = [(b['box_xyxy'], b['score']) for b in base['detections'] if b['class_id'] == 10]
            fixed = track.correct(image, base, candidates, ns, t * 1e-9, classify_traffic_light)
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
    check_runtime(args.weights)
    model = Candidates(args.weights)
    first_image = cv2.imread(str(next((args.capture / "run1/raw").glob("*.png"))))
    began = time.monotonic()
    model.infer(first_image)
    warmup_ms = (time.monotonic() - began) * 1000
    inference_ms = []
    output = {r.name: replay(r, model) for r in sorted(args.capture.glob('run[123]'))}
    assert sum(s['frames'] for s in output.values()) == 420
    output['gpu'] = {
        'warmup_ms': round(warmup_ms, 2),
        'inference_count': len(inference_ms),
        'p50_ms': round(sorted(inference_ms)[len(inference_ms) // 2], 2),
        'max_ms': round(max(inference_ms), 2),
        'torch_peak_allocated_mib': round(torch.cuda.max_memory_allocated() / 1024**2, 1),
    }
    print(json.dumps(output, indent=2))
