"""Focused synthetic adapter tests; no Unity, models, image files or DDS.

Uses real monotonic time for the red/green test. All positions, detector fields
and commands are synthetic protocol fixtures, never physical/lap acceptance.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import time
import unittest

import semantic_worker as worker
from sim_checkpoint_adapter import validate_case_path

OPTIONS = None


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.case_root = Path(OPTIONS.output_root) / 'checkpoints'
        self.path = self.case_root / self._testMethodName / 'topo.json'
        self.transports = []
        self.number = 0
        self.clock = 20.0
        self.transport = self.make_transport()

    def tearDown(self):
        for transport in self.transports:
            transport.close()

    def make_transport(self):
        value = worker.SemanticTransport(OPTIONS.source_root, OPTIONS.vision_root,
            OPTIONS.motor_gate_source, str(self.case_root), str(self.path))
        self.transports.append(value)
        return value

    def request(self, color='unknown', age=.18, points=True, x=0.0):
        self.number += 1
        self.clock += .1
        return dict(type='tick', sequence=self.number, sim_time=self.clock, lane_epoch=1,
            odom=dict(x=x, y=0.0, yaw=0.0, v=0.0, w=0.0),
            vision=dict(sequence=self.number, epoch=1, sim_time=self.clock-age, age=age,
                supported=True, selected_points=[[1.,0.],[2.,0.],[3.,0.]] if points else None,
                corridor=None, traffic_light_state=color, vision_commit=self.transport.vision_commit,
                controller=dict(v=2., w=.1, target=[2.,0.,0.])))

    def pulse(self, color, duration=.7, points=False):
        end = time.monotonic() + duration
        records = []
        while time.monotonic() < end:
            records.append(self.transport.tick(self.request(color=color, points=points)))
            time.sleep(.06)
        return records

    def record_segment(self):
        for index in range(14):
            self.transport.tick(self.request(age=0., x=index*.25))
        self.transport.close()
        return json.loads(self.path.read_text(encoding='utf-8'))

    def test_nonzero_source_age_startup_forwards_original_command(self):
        result = self.transport.tick(self.request(age=.18))
        self.assertEqual((result['v'], result['w']), (2., .1))
        self.assertEqual(result['command_gate']['reason'], 'semantic_go')
        self.assertGreaterEqual(result['command_gate']['source_age_sec'], .18)
        self.assertLess(result['command_gate']['remaining_sec'], .33)

    def test_nonzero_age_recovers_after_empty_tick_timeout(self):
        request = self.request()
        request['vision'] = None
        result = self.transport.tick(request)
        self.assertEqual(result['command_gate']['reason'], 'command_timeout')
        time.sleep(.01)
        result = self.transport.tick(self.request(age=.2))
        self.assertEqual(result['v'], 2.)

    def test_red_green_nonzero_age_release_without_private_abc(self):
        red = self.pulse('red')
        self.assertTrue(red[-1]['status']['traffic_stop_required'])
        self.assertEqual((red[-1]['v'], red[-1]['w']), (0.,0.))
        self.assertFalse(red[-1]['status']['recording_points_valid'])
        green = self.pulse('green')
        self.assertFalse(green[-1]['status']['traffic_stop_required'])
        self.assertEqual(green[-1]['status']['traffic_reason'], 'green_resume')
        self.assertEqual((green[-1]['v'], green[-1]['w']), (2.,.1))
        self.assertEqual(green[-1]['vision_result'], 'command_accepted_no_topo_abc')
        self.assertFalse(green[-1]['status']['recording_points_valid'])

    def test_atomic_color_preserves_capture_age_without_distance(self):
        result = self.transport.tick(self.request(color='red', age=.45))
        self.assertTrue(self.transport.map.light_input_is_atomic)
        self.assertAlmostEqual(self.transport.map.signal_observation.age_sec, .45, places=6)
        self.assertIsNone(self.transport.map.signal_observation.distance_m)
        self.assertIsNone(result['status']['signal_approach']['distance_m'])
        self.assertFalse(result['status']['signal_approach']['enabled'])

    def test_delayed_green_ages_out_before_vote_ready(self):
        red = self.pulse('red')
        self.assertTrue(red[-1]['status']['traffic_stop_required'])
        # Let the old vote window expire while the real red latch remains.
        time.sleep(.53)
        began = time.monotonic()
        for offset in (0., .12, .24, .45):
            delay = began + offset - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            result = self.transport.tick(self.request(color='green', age=.45))
        self.assertTrue(result['status']['traffic_stop_required'], 'setup must finish before the new .5s window matures')
        delay = began + .53 - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        request = self.request()
        request['vision'] = None
        result = self.transport.tick(request)
        self.assertTrue(result['status']['traffic_vote']['ready'])
        self.assertEqual(result['status']['traffic_light'], 'green')
        self.assertTrue(result['status']['traffic_stop_required'])
        self.assertEqual((result['v'],result['w']), (0.,0.))
        fresh = self.transport.tick(self.request(color='green', age=.1))
        self.assertFalse(fresh['status']['traffic_stop_required'])
        self.assertEqual(fresh['status']['traffic_reason'], 'green_resume')
        self.assertEqual(fresh['v'], 2.)

    def test_duplicate_vision_does_not_vote_or_refresh_command(self):
        request = self.request()
        first = self.transport.tick(request)
        request['sequence'] += 1
        request['sim_time'] += .1
        result = self.transport.tick(request)
        self.assertEqual(result['vision_votes_total'], first['vision_votes_total'])
        self.assertFalse(result['command_new'])
        self.assertEqual(result['vision_result'], 'duplicate_or_out_of_order')

    def test_stale_source_rejected_even_at_fresh_arrival(self):
        result = self.transport.tick(self.request(age=.51))
        self.assertEqual(result['vision_result'], 'stale')
        self.assertEqual(result['vision_votes_total'], 0)
        self.assertEqual(result['v'], 0.)

    def test_source_ttl_expires_without_repeated_frame_refresh(self):
        self.transport.tick(self.request(age=.45))
        time.sleep(.07)
        request = self.request()
        request['vision'] = None
        result = self.transport.tick(request)
        self.assertEqual(result['command_gate']['reason'], 'source_command_timeout')
        self.assertEqual((result['v'], result['w']), (0.,0.))
        self.assertEqual(result['command_gate']['remaining_sec'], 0.)

    def test_signal_range_cannot_enter_stop_wait(self):
        request = self.request()
        request['vision']['signal_range'] = dict(valid=True, distance_m=8.)
        with self.assertRaisesRegex(ValueError, 'separate distance trial'):
            self.transport.tick(request)

    def test_unsupported_lane_still_votes_but_never_drives(self):
        request = self.request(color='red')
        request['vision']['supported'] = False
        result = self.transport.tick(request)
        self.assertEqual(result['vision_votes_total'], 1)
        self.assertEqual(result['v'], 0.)

    def test_mismatched_lya_provenance_rejected(self):
        request = self.request()
        request['vision']['vision_commit'] = 'wrong'
        with self.assertRaisesRegex(ValueError, 'provenance'):
            self.transport.tick(request)

    def test_nonfinite_command_rejected(self):
        request = self.request()
        request['vision']['controller']['v'] = float('nan')
        with self.assertRaisesRegex(ValueError, 'finite'):
            self.transport.tick(request)

    def test_epoch_change_cannot_reset_map_implicitly(self):
        self.transport.tick(self.request())
        request = self.request()
        request['lane_epoch'] = 2
        with self.assertRaisesRegex(ValueError, 'implicit map reset'):
            self.transport.tick(request)

    def test_simulation_clock_rewind_rejected(self):
        self.transport.tick(self.request())
        request = self.request()
        request['sim_time'] = 1.
        with self.assertRaisesRegex(ValueError, 'rewind'):
            self.transport.tick(request)

    def test_use_sim_time_reported_honestly(self):
        result = self.transport.tick(self.request())
        self.assertTrue(result['status']['use_sim_time'])
        self.assertAlmostEqual(result['status']['ros_time_sec'], self.clock, places=6)
        self.assertTrue(result['checkpoint_scope']['production_sim_guard_unchanged'])

    def test_checkpoint_roundtrip_uses_production_checksum(self):
        envelope = self.record_segment()
        from semantic_local_planner_ros.topo_checkpoint import digest
        self.assertEqual(envelope['sha256'], digest(envelope['state']))
        self.assertGreater(len(envelope['state']['samples']), 10)
        self.assertEqual(envelope['schema_version'], 1)

    def test_same_case_second_writer_rejected(self):
        second = self.make_transport()
        self.assertTrue(second.map.restore_blocked)
        self.assertEqual(second.map.persistence_status['error'], 'checkpoint_in_use')

    def test_semantic_restart_restores_prefix_and_actual_restore_event(self):
        before = self.record_segment()
        old_boot = self.transport.worker_boot_id
        self.transport = self.make_transport()
        self.assertIsNotNone(self.transport.map.restore_pending)
        result = self.transport.tick(self.request(age=.1, x=3.25))
        self.assertNotEqual(result['worker_boot_id'], old_boot)
        self.assertTrue(result['restore_event']['observed'])
        self.assertEqual(result['restore_event']['persistence']['state'], 'restored')
        rows = [[s.x,s.y,s.yaw,s.excluded] for s in self.transport.map.samples]
        self.assertEqual(rows[:len(before['state']['samples'])], before['state']['samples'])
        self.assertGreaterEqual(result['status']['recorded_samples'], len(before['state']['samples']))
        self.assertFalse(result['status']['signal_approach']['enabled'])

    def test_recovery_pose_mismatch_preserves_saved_file(self):
        self.record_segment()
        before = self.path.read_bytes()
        self.transport = self.make_transport()
        result = self.transport.tick(self.request(x=30.))
        self.assertEqual(result['status']['persistence']['error'], 'odom_position_mismatch')
        self.assertIsNone(result['restore_event'])
        self.transport.close()
        self.assertEqual(before, self.path.read_bytes())

    def test_recovery_time_rewind_preserves_saved_file(self):
        self.record_segment()
        before = self.path.read_bytes()
        self.transport = self.make_transport()
        self.clock = 1.
        result = self.transport.tick(self.request(x=3.25))
        self.assertEqual(result['status']['persistence']['error'], 'odom_time_rewind')
        self.transport.close()
        self.assertEqual(before, self.path.read_bytes())

    def test_checkpoint_corruption_rejected_without_overwrite(self):
        self.record_segment()
        envelope = json.loads(self.path.read_text(encoding='utf-8'))
        envelope['sha256'] = '0'*64
        self.path.write_text(json.dumps(envelope), encoding='utf-8')
        before = self.path.read_bytes()
        self.transport = self.make_transport()
        self.assertEqual(self.transport.map.persistence_status['error'], 'checksum_mismatch')
        self.transport.close()
        self.assertEqual(before, self.path.read_bytes())

    def test_checkpoint_path_escape_and_vehicle_ros_rejected(self):
        with self.assertRaises(ValueError):
            validate_case_path(str(self.case_root), str(self.case_root.parent / 'case/topo.json'))
        with self.assertRaisesRegex(ValueError, 'vehicle'):
            validate_case_path(str(self.case_root), str(self.case_root / '.ros/topo.json'))


class LedgerResult(unittest.TextTestResult):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ledger = []

    def addSuccess(self, test):
        self.ledger.append(dict(test=test.id(), status='PASS'))
        super().addSuccess(test)

    def addFailure(self, test, error):
        self.ledger.append(dict(test=test.id(), status='FAIL', detail=self._exc_info_to_string(error, test)))
        super().addFailure(test, error)

    def addError(self, test, error):
        self.ledger.append(dict(test=test.id(), status='ERROR', detail=self._exc_info_to_string(error, test)))
        super().addError(test, error)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', required=True)
    parser.add_argument('--vision-root', required=True)
    parser.add_argument('--motor-gate-source', required=True)
    parser.add_argument('--output-root', required=True)
    OPTIONS = parser.parse_args()
    output = Path(OPTIONS.output_root)
    output.mkdir(parents=True, exist_ok=False)
    pinned = [Path(__file__), Path(worker.__file__), Path(worker.__file__).with_name('sim_checkpoint_adapter.py')]
    before = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in pinned}
    started = time.monotonic()
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(TransportTests)
    result = unittest.TextTestRunner(verbosity=2, resultclass=LedgerResult, failfast=True).run(suite)
    after = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in pinned}
    receipt = dict(kind='synthetic_stdio_transport_and_windows_checkpoint_unit_tests',
        actual_unity=False, actual_model=False, native_dds=False, tests_run=result.testsRun,
        successful=result.wasSuccessful(), failures=len(result.failures), errors=len(result.errors),
        skipped=len(result.skipped), elapsed_sec=time.monotonic()-started, ledger=result.ledger,
        source_sha256_before=before, source_sha256_after=after, source_unchanged=before==after)
    (output/'receipt.json').write_text(json.dumps(receipt, indent=2), encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() and before == after else 1)
