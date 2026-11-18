"""Stream archived RGB, masks, detections and measured wheel/gyro through the installable driver.

Pass one or more run directories containing frames/ and decisions.jsonl. The recorded
decisions and positions are only compared after step() and never enter the driver.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from concept_drive.core import Drive


def replay(directory, limit):
    engine = Drive()
    frames = 0
    reverses = 0
    ranked = 0
    requested = 0
    evaluated = 0
    first_failure = None
    maxima = dict(control=0., road=0., objects=0., path=0., score=0.,
                  candidate_paths=0., candidate_table=0.)
    with (directory / 'decisions.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            i = row['frame']
            if limit is not None and i >= limit:
                break
            prefix = directory / 'frames' / f'{i:05d}'
            image = cv2.imread(str(prefix) + '_rgb.png', cv2.IMREAD_COLOR)
            detfile = Path(str(prefix) + '_perception.npz')
            if image is None or not detfile.exists():
                raise FileNotFoundError(f'Incomplete stream at frame {i}. All intervening frames are required.')
            old = row['decision']
            with np.load(detfile) as det:
                new = engine.step(image, det['raw'], det['lane'], det, old['sensor'])
            audit = old['audit']
            current_audit = engine.builder.planner.last['audit']
            evaluation = new['candidate_evaluation']
            requested += evaluation['input_count']
            evaluated += evaluation['evaluated_count']
            checks = dict(pick=new['selected_index'] == audit.get('selected'),
                          exact=evaluation['mode'] == 'exact',
                          whitebox=new['whitebox_index'] == audit.get('whiteboxSelected', audit.get('selected')),
                          allowed=np.array_equal([c['allowed'] for c in new['candidates']],
                                                 [bool(c[6]) for c in audit.get('table', [])]))
            for key, current, expected in (
                ('control', new['command_native'], row['control']), ('road', new['road_rows'], old['road']),
                ('objects', new['objects'], old['objects']), ('path', new['selected_path'], old['path']),
                ('candidate_paths', current_audit.get('paths', []), audit.get('paths', [])),
                ('candidate_table', current_audit.get('table', []), audit.get('table', [])),
                ('score', [c['score'] for c in new['candidates']],
                 old['wholeLap']['final_scores'] if old.get('wholeLap') else [c[7] for c in audit.get('table', [])])):
                a, b = np.asarray(current), np.asarray(expected)
                err = float(np.max(np.abs(a-b), initial=0.)) if a.shape == b.shape else None
                checks[key] = err is not None and err <= 1e-5
                if err is not None:
                    maxima[key] = max(maxima[key], err)
            if not all(checks.values()) and first_failure is None:
                first_failure = dict(frame=i, different=[key for key in checks if not checks[key]])
            ranked += int(new['ranking_active'])
            reverses += int(new['command_native'][0] < 0)
            frames += 1
    if not frames:
        raise ValueError(f'No comparable frames in {directory}')
    return dict(run=directory.name, frames=frames, ranks=ranked, reversing_frames=reverses,
                candidate_requests=requested, simulated_candidates=evaluated,
                max_abs_error=maxima, first_difference=first_failure)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--limit', type=int)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    results = [replay(run, args.limit) for run in args.runs]
    print(json.dumps(results, indent=2))
    raise SystemExit(any(row['first_difference'] for row in results))
