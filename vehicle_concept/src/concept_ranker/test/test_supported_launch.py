"""11/19 logged H12 scores plus explicit extra evidence omitted by that recorder."""
from types import SimpleNamespace

import numpy as np
import pytest

from concept_ranker.supported_launch import SupportedLaunch


def logged_case():
    # 17:08:30.241 JST. Original candidate IDs 0, 4 and whitebox 39. ID 4's
    # polyline here uses every fifth logged point. Margins and scores are from
    # decision JSON. Runtime-only positive brake and vision certificates are
    # supplied separately to test both possible outcomes on the same choices.
    scores = np.array([.9157079845519238, -.37082571127353475, -.4274160612412217])
    table = np.zeros((3, 12))
    table[:, 0] = [0, .2, .2]
    table[1, 1] = -.062209
    table[1, 5] = .513974
    table[:, 6] = 1
    table[1:, 8] = .3
    path = [[.0272, 0], [.457, -.0081], [.9584, -.0285], [1.4614, -.0574], [1.9642, -.0883]]
    audit = dict(table=table, previous_command=[0, 0], paths=[[], path, path],
                 nominal_feasible=[True]*3, no_observation_brake_feasible=[True]*3,
                 brake_lower=[1.]*3)
    evidence = dict(visible_z=[.95, 3.2], observed_near_z=np.arange(-1.2, .95, .16),
                    stamp_s=2., applied_speed_mps=0., obstacle_clear=True)
    return dict(pick=0, scores=scores, allowed=np.ones(3, bool), baseline=2, audit=audit,
                obs=SimpleNamespace(objects=[], dt=.2, speed=0.), evidence=evidence)


def test_logged_hold_only_displaced_with_measured_corridor_and_brake():
    args = logged_case()
    pick, scores, why = SupportedLaunch().decide(**args)
    assert pick == scores.argmax() == 1 and why['applied'] and not why['continued']
    assert scores[1] == args['scores'][1]
    assert why['certified_forward_indices'] == [1]
    np.testing.assert_allclose(scores[[0, 2]] + why['noncertified_score_reduction'],
                               args['scores'][[0, 2]])
    # The next logged frame has only 0.424476 m of robust clearance.
    args['audit']['table'][1, 5] = .424476
    pick, scores, why = SupportedLaunch().decide(**args)
    assert pick == 0 and why['reason'] == 'insufficient_driving_or_braking_margin'


@pytest.mark.parametrize('case', [
    'unknown_near', 'missing_near_row', 'unseen_gap', 'short_view', 'no_camera',
    'no_track_yet', 'moving_obstacle', 'whitebox_hold', 'cannot_brake',
    'narrow_braking_margin', 'retimed_only', 'forbidden', 'reversing', 'already_driving',
    'no_progress',
])
def test_no_override_without_positive_evidence(case):
    args = logged_case()
    audit, evidence, obs = args['audit'], args['evidence'], args['obs']
    if case == 'unknown_near':
        evidence['observed_near_z'] = []
    elif case == 'missing_near_row':
        evidence['observed_near_z'] = np.delete(evidence['observed_near_z'], 8)
    elif case == 'unseen_gap':
        evidence['visible_z'][0] = 1.5
    elif case == 'short_view':
        evidence['visible_z'][1] = 2.
    elif case == 'no_camera':
        evidence['visible_z'] = None
    elif case == 'no_track_yet':
        evidence['obstacle_clear'] = False
    elif case == 'moving_obstacle':
        obs.objects = [[2., 0., 0., 1.]]
    elif case == 'whitebox_hold':
        args['baseline'] = 0
    elif case == 'cannot_brake':
        audit['no_observation_brake_feasible'][1] = False
    elif case == 'narrow_braking_margin':
        audit['brake_lower'][1] = .1
    elif case == 'retimed_only':
        audit['nominal_feasible'][1] = False
    elif case == 'forbidden':
        args['allowed'][1] = False
    elif case == 'reversing':
        obs.speed = -.3
    elif case == 'already_driving':
        obs.speed = 1.
    elif case == 'no_progress':
        audit['table'][1, 8] = 0.
    pick, scores, why = SupportedLaunch().decide(**args)
    assert pick == 0 and not why['applied']
    np.testing.assert_array_equal(scores, args['scores'])


@pytest.mark.parametrize('interrupt', [None, 'paused_gate', 'missed_frame', 'new_hazard'])
def test_continuation_requires_actual_applied_command_and_fresh_observation(interrupt):
    args = logged_case()
    launch = SupportedLaunch()
    assert launch.decide(**args)[0] == 1
    args['obs'].speed = .17
    args['evidence'].update(stamp_s=2.2, applied_speed_mps=.2)
    args['audit']['previous_command'] = [.2, -.062209]
    if interrupt == 'paused_gate':
        args['evidence']['applied_speed_mps'] = 0.
    elif interrupt == 'missed_frame':
        args['evidence']['stamp_s'] += .3
    elif interrupt == 'new_hazard':
        args['evidence']['obstacle_clear'] = False
    pick, _, why = launch.decide(**args)
    assert pick == (1 if interrupt is None else 0)
    assert why.get('continued') or not why['applied']


def test_frozen_sim_and_native_h12_motion_keep_scores_and_clear_episode():
    args = logged_case()
    launch = SupportedLaunch()
    assert launch.decide(**args)[0] == 1
    args['pick'] = 2
    assert launch.decide(**args)[0] == 2
    args.update(pick=0, evidence=None)
    pick, scores, why = launch.decide(**args)
    assert why is None and pick == 0
    np.testing.assert_array_equal(scores, args['scores'])


@pytest.mark.parametrize('blocked', ['margin', 'brake'])
def test_real_second_choice_only_with_its_own_certificate(blocked):
    args = logged_case()
    # 17:06, field replay frame 169: incumbent highest moving score belonged
    # to slot 59. Slot 4 had these recorded counterfactual costs and clearances.
    args['scores'] = np.array([1.2, .8269419348338949, .688204054805746])
    audit = args['audit']
    audit['table'][2, 5] = .517425
    audit['brake_lower'][2] = 2.955
    if blocked == 'margin':
        audit['table'][1, 5] = .49
    else:
        audit['no_observation_brake_feasible'][1] = False
    original_allowed = args['allowed'].copy()
    pick, scores, why = SupportedLaunch().decide(**args)
    assert pick == scores.argmax() == why['certified_index'] == 2
    assert why['forward_index'] == 1 and why['top_forward_rejection']
    assert why['h12_top_forward_score'] == args['scores'][1]
    assert why['certified_forward_indices'] == [2]
    assert scores[2] == why['h12_forward_score'] == args['scores'][2]
    assert len(why['rejection_counts']) == 1
    np.testing.assert_array_equal(args['allowed'], original_allowed)
    # An uncertain detection vetoes the whole start, including the alternate.
    args['evidence']['obstacle_clear'] = False
    decision, unchanged, why = SupportedLaunch().decide(**args)
    assert decision == 0 and why['reason'] == 'obstacle_or_unresolved_detection'
    np.testing.assert_array_equal(unchanged, args['scores'])


def test_no_certificate_for_any_alternative_keeps_model_scores_and_stop():
    args = logged_case()
    args['audit']['brake_lower'][1] = .4
    args['audit']['no_observation_brake_feasible'][2] = False
    pick, scores, why = SupportedLaunch().decide(**args)
    assert pick == 0 and not why['applied'] and why['certified_index'] is None
    assert why['certified_forward_indices'] == []
    assert why['rejection_counts'] == {
        'insufficient_driving_or_braking_margin': 1,
        'continuous_drive_or_immediate_brake_unavailable': 1,
    }
    np.testing.assert_array_equal(scores, args['scores'])
