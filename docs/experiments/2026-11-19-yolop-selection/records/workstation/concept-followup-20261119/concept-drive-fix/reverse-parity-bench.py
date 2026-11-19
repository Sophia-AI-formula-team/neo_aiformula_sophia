"""Offline comparison using the 15:50:42 bag's road, wheels, gyro and selected paths."""
import argparse
from contextlib import redirect_stdout
import importlib.util
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parent
def load(name, source):
    spec = importlib.util.spec_from_file_location(name, str(source))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

def main():
    frames = json.loads((ROOT / 'reverse-field-inputs.json').read_text())
    new = load('new_candidates', ROOT / 'backtrack_forward617.py')
    new.advance_reverse = load('new_response', ROOT / 'reverse_model617.py').step
    old = load('old_candidates', ROOT / 'baseline_backtrack.py')
    old.advance_reverse = load('old_response', ROOT / 'baseline_reverse.py').step
    timings = {k: [] for k in ('before', 'after')}
    comparisons = recorded_paths = obstacle_cases = rear_cases = 0
    release_ends = []
    def evaluate(fn, obs, frame):
        horizon = len(frame['path']) * .2
        durations = [1., 0., .2, .4, 3*.2, .8, 1.]
        return [fn(obs, dur, 0., horizon, frame['previous_yaw'], steer)
                for steer in (-.18, 0., .18) for dur in durations]

    def compare(before, after):
        assert before.keys() == after.keys()
        for key in before:
            np.testing.assert_array_equal(before[key], after[key], err_msg=key)
    for i, frame in enumerate(frames):
        obs = SimpleNamespace(road=np.asarray(frame['road']), objects=np.asarray(frame['objects']),
                              speed=frame['speed'], yaw=frame['yaw'], dt=frame['dt'],
                              rear_observed_limit=frame['road'][0][0])
        results = {}
        for label in (('before', 'after') if i % 2 else ('after', 'before')):
            step = {'before':old.checked_retreat, 'after':new.checked_retreat}[label]
            t = time.perf_counter()
            results[label] = evaluate(step, obs, frame)
            timings[label].append(1000 * (time.perf_counter()-t))
        for before, after in zip(results['before'], results['after']):
            compare(before, after)
            comparisons += 1
        sel = results['after'][7]
        assert sel['valid'] and frame['command']['linear_x_mps'] == sel['controls'][0,0]
        np.testing.assert_array_equal(sel['path'], frame['path'])
        recorded_paths += 1
        if i == 0:
            obs.rear_observed_limit = None
            for duration in (0.,1.):
                before,after = (m.checked_retreat(obs, duration, .5, 4.2, .11, -.18) for m in (old,new))
                compare(before, after)
                assert after['valid'] == (duration==0.)
                rear_cases += 1
            obs.rear_observed_limit=obs.road[0,0]
            obj=np.zeros((1,77));obj[0,4]=.3;obj[0,25]=.75;obj[0,66:68]=.02
            for offset in (30,50):obj[0,offset:offset+16]=(.002*np.eye(4)).ravel()
            obs.objects=obj
            for z in (0.,8.):
                for start in (0,26,46):obj[0,start]=z
                before,after=(m.checked_retreat(obs,1.,.1,4.2,.0,.1) for m in (old,new))
                compare(before, after)
                assert after['valid'] == (z==8.)
                obstacle_cases+=1
    print(json.dumps(dict(field_frames=len(frames), all_fields_identical=comparisons,
                          recorded_selected_paths_identical=recorded_paths,
                          rear_visibility_equivalence=rear_cases, obstacle_equivalence=obstacle_cases,
                          ms={k:dict(median=float(np.median(v)),p95=float(np.percentile(v,95)),max=max(v))
                              for k,v in timings.items()}, samples_ms=timings),indent=2))

if __name__ == '__main__':
    main()
