"""Disk-format/atomicity tests; no ROS, DDS or vehicle operations."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from semantic_local_planner_ros.topo_checkpoint import CheckpointStore, digest, validate_state


class CheckpointTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.path = Path(tmp.name) / 'nested' / 'map.json'
        self.store = CheckpointStore(self.path, 'config1', 100)
        self.addCleanup(self.store.close)
        self.state = dict(samples=[[0.0, 0.0, 0.0, False], [1.0, 0.0, 0.0, True]],
                          recorded_distance=1.0, left_start_area=False, excluded_since_sample=True,
                          closed_at=None, progress=0.0, last_pose=[1.0, 0.0, 0.0],
                          odom_frame='odom', child_frame='base_footprint', odom_stamp=1.0)

    def test_roundtrip_empty_then_partial_and_no_temporary_files(self):
        self.assertIsNone(self.store.load())
        self.store.save(self.state)
        self.assertEqual(self.store.load(), self.state)
        self.assertEqual(list(self.path.parent.glob('*.tmp')), [])

    def test_unknown_schema_checksum_and_configuration_rejected(self):
        self.store.save(self.state)
        valid = json.loads(self.path.read_text())
        for key, value, error in [('schema_version', 99, 'schema_mismatch'),
                                  ('sha256', 'bad', 'checksum_mismatch'),
                                  ('fingerprint', 'other', 'map_or_config_mismatch')]:
            with self.subTest(key=key):
                envelope = dict(valid)
                envelope[key] = value
                self.path.write_text(json.dumps(envelope))
                with self.assertRaisesRegex(ValueError, error):
                    self.store.load()

    def test_invalid_samples_numbers_types_and_bounds_rejected(self):
        changes = [dict(samples=[[0, 0, float('nan'), False]]), dict(samples=[[0, 0, 0, 'false']]),
                   dict(samples=[[0, 0, 0, False]] * 101), dict(recorded_distance=-1),
                   dict(progress=float('inf')), dict(left_start_area='yes'), dict(odom_stamp=True),
                   dict(last_pose=None), dict(odom_frame=''), dict(closed_at=[0, 0, 0])]
        for change in changes:
            with self.subTest(change=change):
                state = copy.deepcopy(self.state)
                state.update(change)
                with self.assertRaises(ValueError):
                    validate_state(state, 100)

    def test_bounded_read_rejects_large_file(self):
        self.path.write_bytes(b' ' * 1025)
        with patch('semantic_local_planner_ros.topo_checkpoint.MAX_BYTES', 1024):
            with self.assertRaisesRegex(ValueError, 'checkpoint_too_large'):
                self.store.load()

    def test_failed_fsync_keeps_previous_snapshot(self):
        self.store.save(self.state)
        previous = self.path.read_bytes()
        self.state['progress'] = 2
        with patch('semantic_local_planner_ros.topo_checkpoint.os.fsync', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                self.store.save(self.state)
        self.assertEqual(self.path.read_bytes(), previous)
        self.assertEqual(list(self.path.parent.glob('*.tmp')), [])

    def test_lock_released_after_close(self):
        with self.assertRaisesRegex(ValueError, 'checkpoint_in_use'):
            CheckpointStore(self.path, 'config1', 100)
        self.store.close()
        other = CheckpointStore(self.path, 'config1', 100)
        other.close()
