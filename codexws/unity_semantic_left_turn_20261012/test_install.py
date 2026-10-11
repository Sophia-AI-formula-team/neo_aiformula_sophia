"""Isolated installer safety tests; never read or write the live Simulator.

Every project/payload below is created in a fresh TemporaryDirectory. These
tests establish file-tool behaviour, not Unity compilation or driving results.
"""
import contextlib
import copy
import io
import json
import os
from pathlib import Path
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import install


class InstallerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="semantic-left-installer-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.bundle = self.root / "bundle"
        self.project = self.root / "project"
        self.bundle.mkdir()
        (self.project / "Assets").mkdir(parents=True)
        self.rows = []
        for index, name in enumerate(sorted(install.ALLOWED)):
            row = dict(path=name)
            for side in ("before", "after"):
                data = (side + " " + name + "\r\n").encode() if side == "after" or index % 3 == 0 else None
                row[side + "_sha256"] = None if data is None else install.digest(data)
                row[side] = None if data is None else dict(kind="file", path=("before/" if side == "before" else "overlay/") + name)
                if data is not None:
                    self.write(self.bundle / row[side]["path"], data)
                if side == "before" and data is not None:
                    self.write(self.project / name, data)
            self.rows.append(row)
        self.prerequisite = "Assets/StreamingAssets/SemanticPlanning/source/map_assist.py"
        self.prerequisite_data = b"# frozen native core; never an overlay destination\r\n"
        self.write(self.project / self.prerequisite, self.prerequisite_data)
        self.unrelated = "Assets/Tools/LaneTeachRepeat/peer.cs"
        self.write(self.project / self.unrelated, b"peer-owned\r\n")
        self.manifest = dict(format=install.FORMAT, before_semantic_commit=install.BEFORE,
            frozen_semantic_commit=install.AFTER, integration_base_commit=install.BASE,
            files=self.rows, prerequisites={self.prerequisite: install.digest(self.prerequisite_data)})
        self.save_manifest()

    @staticmethod
    def write(path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)

    def save_manifest(self):
        data = json.dumps(self.manifest, indent=2).encode()
        (self.bundle / "manifest.json").write_bytes(data)
        (self.bundle / "manifest.sha256").write_text(install.digest(data) + "  manifest.json\n", encoding="ascii")

    def project_bytes(self):
        return {p.relative_to(self.project).as_posix(): p.read_bytes()
                for p in self.project.rglob("*") if p.is_file()}

    def execute(self, action="verify-payload", **kwargs):
        return install.execute(self.bundle, self.project, action, **kwargs)

    def apply(self):
        result = self.execute("apply", write=True, unity_closed=True)
        self.assertTrue(result["ok"], result)
        return result

    def test_default_cli_only_verifies_payload_and_has_zero_writes(self):
        before = self.project_bytes()
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            code = install.main(["--bundle", str(self.bundle)])
        result = json.loads(stream.getvalue())
        self.assertEqual(code, 0)
        self.assertEqual(result["action"], "verify-payload")
        self.assertEqual(result["writes"], 0)
        self.assertEqual(before, self.project_bytes())

    def test_write_requires_both_explicit_flags(self):
        before = self.project_bytes()
        for action in ("apply", "rollback"):
            for flags in ({}, {"write": True}, {"unity_closed": True}):
                with self.subTest(action=action, flags=flags), self.assertRaisesRegex(ValueError, "--write --unity-closed"):
                    self.execute(action, **flags)
        self.assertEqual(before, self.project_bytes())

    def test_readonly_actions_reject_write_flags(self):
        for action in ("inspect", "verify-payload", "verify-before", "verify-after"):
            with self.subTest(action=action), self.assertRaises(ValueError):
                self.execute(action, write=True, unity_closed=True)

    def test_payload_verification_precedes_all_writes(self):
        before = self.project_bytes()
        self.write(self.bundle / self.rows[-1]["after"]["path"], b"corrupted late payload")
        with self.assertRaisesRegex(ValueError, "Payload bytes differ"):
            self.execute("apply", write=True, unity_closed=True)
        self.assertEqual(before, self.project_bytes())

    def test_before_payload_also_verified_before_apply(self):
        before = self.project_bytes()
        row = next(row for row in reversed(self.rows) if row["before"])
        self.write(self.bundle / row["before"]["path"], b"bad rollback bytes")
        with self.assertRaisesRegex(ValueError, "Payload bytes differ"):
            self.execute("apply", write=True, unity_closed=True)
        self.assertEqual(before, self.project_bytes())

    def test_manifest_digest_and_pin_mismatch_rejected(self):
        raw = (self.bundle / "manifest.json").read_bytes()
        (self.bundle / "manifest.json").write_bytes(raw + b" ")
        with self.assertRaisesRegex(ValueError, "checksum"):
            self.execute()
        for key in ("format", "before_semantic_commit", "frozen_semantic_commit", "integration_base_commit"):
            original = self.manifest[key]
            self.manifest[key] = "wrong"
            self.save_manifest()
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "frozen revisions"):
                self.execute()
            self.manifest[key] = original

    def test_manifest_duplicate_keys_rejected(self):
        raw = (self.bundle / "manifest.json").read_bytes()
        raw = raw.replace(b'"format":', b'"format":"duplicate", "format":', 1)
        (self.bundle / "manifest.json").write_bytes(raw)
        (self.bundle / "manifest.sha256").write_text(install.digest(raw), encoding="ascii")
        with self.assertRaisesRegex(ValueError, "Duplicate JSON key"):
            self.execute()

    def test_path_traversal_absolute_reserved_and_aliases_rejected(self):
        for value in ("../outside", "/absolute", "C:/outside", "Assets\\file", "Assets//file",
                      "Assets/./file", "Assets/NUL", "Assets/name.", "Assets/name ", "Assets/x\nfile"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                install.safe_name(value)
        original = self.rows[0]["after"]["path"]
        self.rows[0]["after"]["path"] = "overlay/../../outside"
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "exact file"):
            self.execute()
        self.rows[0]["after"]["path"] = original

    def test_core_weights_scenes_settings_upstream_and_peer_paths_refused(self):
        for name in (self.prerequisite, "Assets/StreamingAssets/RoadDetector/model.pt",
                     "Assets/Scenes/MotorControllerValidation.unity", "Assets/preset.asset",
                     "Assets/StreamingAssets/VisionLaneFollower/upstream/lane.py", self.unrelated,
                     "Assets/AIFormula/Runtime/Control/New.cs"):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "allowlist"):
                install.managed_name(name)

    def test_full_allowlist_and_unique_rows_required(self):
        removed = self.rows.pop()
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "entire fixed"):
            self.execute()
        self.rows.append(removed)
        self.rows.append(copy.deepcopy(self.rows[0]))
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "Duplicate/case"):
            self.execute()

    def test_git_recipe_and_absent_both_sides_refused(self):
        self.rows[0]["after"] = dict(kind="git_blob", path="whatever", commit=install.AFTER)
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "file payload"):
            self.execute()
        self.rows[0].update(before=None, before_sha256=None, after=None, after_sha256=None)
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "absent on both"):
            self.execute()

    def test_missing_or_overlapping_prerequisite_refused(self):
        self.manifest["prerequisites"] = {}
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "prerequisites are required"):
            self.execute()
        self.manifest["prerequisites"] = {self.rows[0]["path"]: "0" * 64}
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "overlapping"):
            self.execute()

    def test_precondition_drift_produces_zero_writes(self):
        self.write(self.project / self.prerequisite, b"changed core")
        before = self.project_bytes()
        result = self.execute("apply", write=True, unity_closed=True)
        self.assertFalse(result["ok"])
        self.assertEqual(result["writes"], 0)
        self.assertEqual(before, self.project_bytes())

    def test_destination_drift_produces_zero_writes(self):
        self.write(self.project / self.rows[-1]["path"], b"unrelated newer edit")
        before = self.project_bytes()
        result = self.execute("apply", write=True, unity_closed=True)
        self.assertFalse(result["ok"])
        self.assertEqual(result["writes"], 0)
        self.assertEqual(before, self.project_bytes())

    def test_apply_verify_and_exact_rollback_preserve_peer_and_core(self):
        before = self.project_bytes()
        self.assertTrue(self.execute("verify-before")["ok"])
        result = self.apply()
        self.assertEqual(result["writes"], len(self.rows))
        self.assertEqual(result["completed"], [row["path"] for row in self.rows])
        self.assertTrue(self.execute("verify-after")["ok"])
        result = self.execute("rollback", write=True, unity_closed=True)
        self.assertTrue(result["ok"], result)
        self.assertEqual(before, self.project_bytes())
        self.assertFalse(list(self.project.rglob(".semantic-left-*.tmp")))

    def test_added_file_rollback_requires_exact_after_hash(self):
        self.apply()
        row = next(row for row in self.rows if row["before"] is None)
        self.write(self.project / row["path"], b"user edit after installation")
        before = self.project_bytes()
        result = self.execute("rollback", write=True, unity_closed=True)
        self.assertFalse(result["ok"])
        self.assertEqual(result["writes"], 0)
        self.assertEqual(before, self.project_bytes())

    def test_mixed_state_cannot_apply_or_rollback(self):
        row = self.rows[0]
        self.write(self.project / row["path"], (self.bundle / row["after"]["path"]).read_bytes())
        inspection = self.execute("inspect")
        self.assertTrue(inspection["mixed"])
        self.assertFalse(inspection["ok"])
        for action in ("apply", "rollback"):
            result = self.execute(action, write=True, unity_closed=True)
            self.assertFalse(result["ok"])
            self.assertEqual(result["writes"], 0)

    def test_partial_atomic_failure_receipt_lists_completed_and_refuses_retry(self):
        original = install.os.replace
        calls = []
        def replace(source, destination):
            calls.append(str(destination))
            if len(calls) == 2:
                raise OSError("fixture interruption")
            return original(source, destination)
        with patch.object(install.os, "replace", replace):
            result = self.execute("apply", write=True, unity_closed=True)
        self.assertFalse(result["ok"])
        self.assertEqual(result["writes"], 1)
        self.assertEqual(result["completed"], [self.rows[0]["path"]])
        self.assertEqual(result["in_progress"], self.rows[1]["path"])
        self.assertTrue(self.execute("inspect")["mixed"])
        self.assertFalse(self.execute("apply", write=True, unity_closed=True)["ok"])
        self.assertFalse(self.execute("rollback", write=True, unity_closed=True)["ok"])
        self.assertFalse(list(self.project.rglob(".semantic-left-*.tmp")))

    def test_prerequisite_change_between_files_stops_with_receipt(self):
        original = install.os.replace
        def replace(source, destination):
            original(source, destination)
            self.write(self.project / self.prerequisite, b"changed after first replacement")
        with patch.object(install.os, "replace", replace):
            result = self.execute("apply", write=True, unity_closed=True)
        self.assertFalse(result["ok"])
        self.assertEqual(result["completed"], [self.rows[0]["path"]])
        self.assertEqual(result["writes"], 1)

    def test_noop_rows_are_not_rewritten(self):
        row = self.rows[0]
        data = (self.bundle / row["before"]["path"]).read_bytes()
        self.write(self.bundle / row["after"]["path"], data)
        row["after_sha256"] = row["before_sha256"]
        self.save_manifest()
        result = self.apply()
        self.assertEqual(result["unchanged"], [row["path"]])
        self.assertEqual(result["writes"], len(self.rows)-1)

    def test_payload_hardlink_is_refused(self):
        path = self.bundle / self.rows[0]["after"]["path"]
        os.link(path, self.root / "outside-hardlink")
        with self.assertRaisesRegex(ValueError, "Linked path"):
            self.execute()

    def test_destination_hardlink_is_refused_without_touching_external_file(self):
        path = self.project / self.rows[0]["path"]
        external = self.root / "outside-hardlink"
        os.link(path, external)
        expected = external.read_bytes()
        with self.assertRaisesRegex(ValueError, "Linked path"):
            self.execute("apply", write=True, unity_closed=True)
        self.assertEqual(external.read_bytes(), expected)

    def test_symlinked_root_is_refused_before_resolve(self):
        link = self.root / "project-link"
        try:
            link.symlink_to(self.project, target_is_directory=True)
        except (OSError, NotImplementedError) as error:
            self.skipTest("Host does not permit directory symlink creation: " + str(error))
        self.addCleanup(lambda: link.unlink() if link.is_symlink() else None)
        with self.assertRaisesRegex(ValueError, "Linked path"):
            install.execute(self.bundle, link, "verify-before")

    def test_windows_reparse_attribute_is_refused_on_older_python(self):
        value = SimpleNamespace(st_mode=stat.S_IFDIR, st_file_attributes=0x400, st_nlink=1)
        with patch.object(Path, "lstat", return_value=value), self.assertRaisesRegex(ValueError, "Linked path"):
            install._unlinked(self.project)


if __name__ == "__main__":
    unittest.main(verbosity=2)
