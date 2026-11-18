import json
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from concept_drive.core import Drive


def test_native_selection_and_every_slot_against_evaluating_each_request(monkeypatch):
    # Old laboratory settings cannot turn on near-target or cross-speed merging.
    monkeypatch.setenv('AIFORMULA_CANDIDATE_MERGE', 'tolerance')
    monkeypatch.setenv('AIFORMULA_CANDIDATE_TOL_M', '1')
    monkeypatch.setenv('AIFORMULA_CANDIDATE_TOL_MPS', '1')
    path = Path(__file__).parent / 'data' / 'moving-lap-start'
    rows = json.loads((path / 'expected.json').read_text())
    baseline, production = Drive(3.4), Drive(3.4)
    cv2.setNumThreads(1)

    def every_slot(targets, commands):
        slots = np.arange(len(commands))
        return slots, slots

    for row in rows:
        frame = path / ('%05d' % row['frame'])
        image = cv2.imread(str(frame) + '_rgb.png')
        with np.load(str(frame) + '_perception.npz') as det:
            args = image, det['raw'], det['lane'], det, row['sensor']
            with patch('concept_candidates.curve_road617.distinct_requests', every_slot):
                all_slots = baseline.step(*args)
            reused = production.step(*args)
        baseline_counts = all_slots.pop('candidate_evaluation')
        counts = reused.pop('candidate_evaluation')
        assert baseline_counts['evaluated_count'] == baseline_counts['input_count'] == 75
        assert counts == dict(mode='exact', batches=1, input_count=75,
                              evaluated_count=[55, 60, 60, 55, 55][row['frame']])
        assert reused['selected_index'] == row['selected']
        assert reused == all_slots
        old_audit = dict(baseline.builder.planner.last['audit'])
        audit = dict(production.builder.planner.last['audit'])
        assert audit.pop('candidate_evaluation') == counts
        old_audit.pop('candidate_evaluation')
        assert audit == old_audit
        old_pending, pending = baseline.planner.pending, production.planner.pending
        assert pending.keys() == old_pending.keys()
        for key in pending:
            np.testing.assert_array_equal(pending[key], old_pending[key])
