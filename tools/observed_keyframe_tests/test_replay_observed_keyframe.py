"""Optional NumPy/Pillow evidence checks, separate from standard-library handoff tests."""
import json
import math
from pathlib import Path
import tempfile
import unittest
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from replay_observed_keyframe import map_prior, pose_digest, source_hashes, source_ns, write_new


class ObservedFixtureHelpers(unittest.TestCase):
    def test_source_time_keeps_original_rounding_and_positive_epoch(self):
        self.assertEqual(source_ns(0.), 1_000_000_000)
        self.assertEqual(source_ns(5.659999873), 6_659_999_873)

    def test_teach_origin_is_a_rigid_frame_not_an_odometry_reset(self):
        origin = np.array([3., 4., math.pi / 2])
        raw = np.array([3., 6., math.pi / 2 + .2])
        np.testing.assert_allclose(map_prior(raw, origin), [2., 0., .2], atol=1e-14)
        np.testing.assert_array_equal(raw, [3., 6., math.pi / 2 + .2])
        np.testing.assert_allclose(map_prior(origin, origin), [0., 0., 0.], atol=1e-14)

    def test_pose_digest_ignores_signed_zero_not_significant_changes(self):
        self.assertEqual(pose_digest([0., 1., 2.]), pose_digest([-0., 1. + 1e-12, 2.]))
        self.assertNotEqual(pose_digest([0., 1., 2.]), pose_digest([0., 1.001, 2.]))

    def test_line_endings_are_explicit_and_other_content_is_not_ignored(self):
        lf, crlf = source_hashes(b"a\nb\n"), source_hashes(b"a\r\nb\r\n")
        self.assertNotEqual(lf["raw"], crlf["raw"])
        self.assertEqual(lf["lf"], crlf["lf"])
        self.assertEqual(lf["crlf"], crlf["raw"])
        self.assertNotEqual(lf["lf"], source_hashes(b"a\nb \n")["lf"])

    def test_outputs_are_new_and_numpy_values_are_serialized(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            output = Path(directory) / "evidence.json"
            write_new(output, {"points": np.array([[1., 2.]]), "count": np.int64(1)})
            before = output.read_bytes()
            self.assertEqual(json.loads(before), {"points": [[1., 2.]], "count": 1})
            with self.assertRaises(FileExistsError):
                write_new(output, {"replaced": True})
            self.assertEqual(output.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
