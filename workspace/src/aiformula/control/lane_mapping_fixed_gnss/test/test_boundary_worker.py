"""Real mask extraction + worker routing; no ROS transport or route acceptance."""
from contextlib import nullcontext
from dataclasses import replace
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / 'lane_mapping_lya_reference'))
from lane_mapping_fixed_gnss import worker as module
from lane_mapping_fixed_gnss.attitude import AttitudeSample, AttitudeSnapshot
from lane_mapping_lya_reference.mapping_core import GroundLookup
from lane_mapping_lya_reference.paint_geometry import BoundaryObservation
from test_boundary_bundle import snapshot, reference, graph_reference, branch_subset_reference


def setup_worker(mode='legacy_flat'):
    # Exercise actual methods without creating a background scheduling thread;
    # queue ownership/guard timing is independently covered by test_worker.
    worker = object.__new__(module.MapWorker)
    worker.motion_attitude_mode = mode
    worker.lookup = worker.calibration = worker.matcher = worker.snapshot = None
    worker.last_export_ns = time.monotonic_ns()
    worker.guard = lambda job: nullcontext(True)
    worker.engine = SimpleNamespace(process=Mock(return_value={'accepted': True}),
                                   repeat_localizer=Mock(), freeze=Mock())
    transform = np.eye(4)
    transform[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
    transform[:3, 3] = [.2, -.1, .65]
    calibration = dict(width=13, height=9, camera_matrix=[[8., .23, 6.2], [0., 7., 2.1], [0., 0., 1.]],
                       base_from_camera=transform.tolist())
    image = np.zeros((9, 13), dtype=np.uint8)
    image[4:8, 3:10] = 255
    source = AttitudeSample(90, 190, (0., 0., 1.), (0., 0., 0.), (0., 0., 0.), None, .25)
    attitude = AttitudeSnapshot(source, 100, 200, (0., 0., 1.))
    job = dict(kind='teach', mask=image, calibration=calibration, stamp_ns=100,
               prior_stamp_ns=95, prior_gyro_stamp_ns=90, prior_receipt_ns=200, receipt_ns=210,
               prior=[0., 0., 0.], attitude=attitude)
    return worker, job


@pytest.mark.parametrize('mode', ['legacy_flat', 'raw_imu_tilt'])
def test_actual_full_mask_extracts_boundary_without_changing_filled_projection(mode):
    worker, job = setup_worker(mode)
    original = job['mask'].copy()
    local, sensitivity = worker._project(job)
    observation = job['boundary_observation']
    assert isinstance(observation, BoundaryObservation)
    assert observation.source_stamp_ns == job['stamp_ns']
    assert len(observation.points) > 0
    expected = GroundLookup(13, 9, job['calibration']['camera_matrix'],
                            np.asarray(job['calibration']['base_from_camera']), .5)
    points, scores, _ = expected.project_mask_with_sensitivity(original, 127)
    assert local.tobytes() == points.tobytes()
    assert sensitivity.tobytes() == scores.tobytes()
    np.testing.assert_array_equal(job['mask'], original)
    with pytest.raises(ValueError):
        observation.points.setflags(write=True)


@pytest.mark.parametrize('kind', ['teach', 'repeat'])
def test_real_projected_observation_reaches_matching_only_and_not_job_log(kind):
    worker, job = setup_worker()
    job['kind'] = kind
    result = SimpleNamespace(accepted=False, correction_se2=(0., 0., 0.))
    worker.matcher = SimpleNamespace(match=Mock(return_value=result), discard_pending=Mock())
    output = worker._compute(job)
    call = worker.engine.process if kind == 'teach' else worker.matcher.match
    observation = call.call_args.kwargs['boundary_observation']
    assert isinstance(observation, BoundaryObservation)
    assert observation.source_stamp_ns == job['stamp_ns']
    assert call.call_args.kwargs['motion_source_stamp_ns'] == job['prior_stamp_ns']
    assert 'boundary_observation' not in output['job']
    if kind == 'repeat':
        assert output['committed'] is False
        worker.matcher.discard_pending.assert_called_once()


def test_bad_extraction_does_not_reuse_previous_observation_or_call_engine(monkeypatch):
    worker, job = setup_worker()
    job['boundary_observation'] = object()
    def fail(*args, **kwargs):
        raise ValueError('boundary input rejected')
    monkeypatch.setattr(module, 'extract_boundary_observation', fail)
    with pytest.raises(ValueError, match='boundary input rejected'):
        worker._compute(job)
    assert 'boundary_observation' not in job
    worker.engine.process.assert_not_called()


@pytest.mark.parametrize('field,value', [('stamp_ns', 101), ('prior_gyro_stamp_ns', 91),
    ('prior_receipt_ns', 201), ('receipt_ns', 199)])
def test_selected_attitude_timing_is_still_strict(field, value):
    worker, job = setup_worker('raw_imu_tilt')
    job[field] = value
    with pytest.raises(ValueError, match='selected causal wheel attitude'):
        worker._compute(job)
    worker.engine.process.assert_not_called()


@pytest.mark.parametrize('member', ['points_xy', 'normals', 'support_segments', 'segment_owner',
                                  'votes', 'reliable_votes', 'record_ids', 'missing'])
def test_bundle_rejects_changed_reference_even_if_other_metadata_claims_original(monkeypatch, member):
    worker, _ = setup_worker()
    worker.snapshot = original = snapshot()
    worker.engine.freeze.return_value = original
    changed = None
    if member != 'missing':
        array = getattr(original.boundary_reference, member).copy()
        if member == 'normals':
            array *= -1.
        elif member == 'segment_owner':
            array[0] = 1
        elif member in ('points_xy', 'support_segments'):
            array.flat[0] += .01
        else:
            array[0] += 1
        changed = replace(original.boundary_reference, **{member: array})
    metadata = dict(snapshot_geometry_sha256=module.geometry_sha256(original),
                    source_frontier_ns=100, boundary_reference=changed)
    monkeypatch.setattr(module, 'build_bundle', lambda *args, **kwargs: Path('unused-bundle.json'))
    monkeypatch.setattr(module, 'load_bundle', lambda path: ({}, {}, metadata, original.consensus_xy, {}))
    with pytest.raises(ValueError, match='does not match'):
        worker._compute(dict(kind='bundle', root='unused'))
    worker.engine.repeat_localizer.assert_not_called()


def test_bundle_exact_boundary_reference_hands_off_existing_engine_state(monkeypatch):
    worker, _ = setup_worker()
    worker.snapshot = original = snapshot()
    worker.engine.freeze.return_value = original
    metadata = dict(snapshot_geometry_sha256=module.geometry_sha256(original),
                    source_frontier_ns=100, boundary_reference=reference())
    monkeypatch.setattr(module, 'build_bundle', lambda *args, **kwargs: Path('unused-bundle.json'))
    monkeypatch.setattr(module, 'load_bundle', lambda path: ({}, {}, metadata, original.consensus_xy, {}))
    result = worker._compute(dict(kind='bundle', root='unused'))
    worker.engine.repeat_localizer.assert_called_once_with()
    assert worker.matcher is worker.engine.repeat_localizer.return_value
    assert 'boundary_reference' not in result


@pytest.mark.parametrize('name',['directed_edges','edge_frame_votes','edge_reliable_votes','edge_last_stamp_ns',
                                 'full_in_degree','full_out_degree','missing'])
def test_bundle_worker_rejects_graph_difference_with_identical_points(monkeypatch,name):
    worker,_=setup_worker()
    worker.snapshot=original=replace(snapshot(),boundary_reference=graph_reference())
    worker.engine.freeze.return_value=original
    changed=reference()
    if name!='missing':
        array=getattr(original.boundary_reference,name).copy()
        if name=='directed_edges': array[0]=[0,2]
        else: array[0]+=1
        changes={name:array}
        if name=='directed_edges':
            changes.update(full_in_degree=None,full_out_degree=None)
        changed=replace(original.boundary_reference,**changes)
    metadata=dict(snapshot_geometry_sha256=module.geometry_sha256(original),source_frontier_ns=100,
                  boundary_reference=changed)
    monkeypatch.setattr(module,'build_bundle',lambda *args,**kwargs:Path('unused-bundle.json'))
    monkeypatch.setattr(module,'load_bundle',lambda path:({}, {},metadata,original.consensus_xy,{}))
    with pytest.raises(ValueError,match='does not match'):
        worker._compute(dict(kind='bundle',root='unused'))
    worker.engine.repeat_localizer.assert_not_called()


def test_worker_rejects_locally_valid_but_falsely_lowered_full_branch_degree(monkeypatch):
    worker,_=setup_worker()
    worker.snapshot=original=replace(snapshot(),boundary_reference=branch_subset_reference())
    worker.engine.freeze.return_value=original
    degrees=original.boundary_reference.full_out_degree.copy()
    degrees[1]=1
    changed=replace(original.boundary_reference,full_out_degree=degrees)
    metadata=dict(snapshot_geometry_sha256=module.geometry_sha256(original),source_frontier_ns=100,
                  boundary_reference=changed)
    monkeypatch.setattr(module,'build_bundle',lambda *args,**kwargs:Path('unused-bundle.json'))
    monkeypatch.setattr(module,'load_bundle',lambda path:({}, {},metadata,original.consensus_xy,{}))
    with pytest.raises(ValueError,match='does not match'):
        worker._compute(dict(kind='bundle',root='unused'))
    worker.engine.repeat_localizer.assert_not_called()
