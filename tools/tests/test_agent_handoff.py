"""Offline handoff tests: no Git, network, ROS or hardware dependencies."""

import contextlib
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock
import zipfile


SOURCE = Path(__file__).resolve().parents[1] / "agent_handoff.py"
SPEC = importlib.util.spec_from_file_location("agent_handoff", str(SOURCE))
handoff = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(handoff)


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        # The implementation canonicalizes session/output directories. Windows
        # runners may supply an 8.3 TEMP alias (for example RUNNER~1); use the
        # same canonical spelling so fault-injection Path comparisons hit.
        self.root = Path(self.temporary.name).resolve(strict=True)
        self.session = self.root / "lab-001"
        handoff.init_session(self.root, "lab-001", "experiment", "a" * 40, "plan-001")

    def metadata(self, **changes):
        path = self.session / "session.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        data.update(changes)
        path.write_text(json.dumps(data), encoding="utf-8")
        return data

    def seal(self, result="partial"):
        data = self.metadata(status="handed_off", result=result,
                             publication_review={"secrets_checked": True, "privacy_checked": True})
        contents = {
            "Objective and scope": "Offline checks only. No vehicle or remote access was authorized.",
            "Input handoff": "Consumed plan-001; reviewed the supplied plan and source snapshot.",
            "Source and working tree": "Base is " + data["base_code_sha"] + "; dirty tree: local patch retained. Dependency source: Python standard library.",
            "Work performed": "Ran python -B tools/agent_handoff.py validate --session lab-001; vehicle tests were not run.",
            "Evidence and result": "Result is " + result + "; one offline check completed, no vehicle acceptance claimed.",
            "Risks and blockers": "Lab hardware is unavailable. Images and notes were manually reviewed.",
            "Next agent actions": "Review evidence, reproduce offline checks, then ask the operator before any hardware action.",
        }
        document = "# Session report\n\n" + "\n".join(
            "## " + heading + "\n\n" + contents[heading] + "\n" for heading in handoff.SECTIONS)
        (self.session / "SESSION.md").write_text(document, encoding="utf-8")

    def evidence(self, name="check.txt", data=b"offline check completed\n"):
        target = self.session / "evidence" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        return target

    def test_aliased_temp_root_preserves_all_four_fault_injections(self):
        # A real '..' alias is portable and exposes the same lexical-vs-
        # canonical mismatch without requiring Windows 8.3 names or symlinks.
        # Reuse the original tests and their full exception/content assertions.
        tests = ("test_racing_output_is_not_replaced_and_temps_are_removed",
                 "test_racing_sidecar_rolls_back_only_our_zip",
                 "test_source_mutation_during_snapshot_fails",
                 "test_windows_reparse_junction_attribute_is_rejected")
        for index, name in enumerate(tests):
            with self.subTest(injection=name):
                case_root = self.root / ("alias-case-" + str(index))
                case_root.mkdir()
                intermediate = case_root / "intermediate"
                intermediate.mkdir()
                alias = intermediate / ".."
                self.assertNotEqual(alias, case_root)
                self.assertEqual(alias.resolve(strict=True), case_root)
                fixture = HandoffTests(name)
                temporary = SimpleNamespace(name=str(alias), cleanup=lambda: None)
                try:
                    with mock.patch.object(tempfile, "TemporaryDirectory", return_value=temporary):
                        fixture.setUp()
                    self.assertEqual(fixture.root, case_root)
                    getattr(fixture, name)()
                finally:
                    fixture.doCleanups()

    def test_init_is_exclusive_and_draft_not_publishable(self):
        summary = handoff.validate_session(self.session)
        self.assertEqual(summary["status"], "draft")
        self.assertEqual(summary["result"], "not_run")
        self.assertFalse(summary["pack_eligible"])
        self.assertTrue((self.session / "evidence").is_dir())
        with self.assertRaises(FileExistsError):
            handoff.init_session(self.root, "lab-001", "planner", "b" * 40, "plan-002")
        with self.assertRaisesRegex(handoff.HandoffError, "handed_off"):
            handoff.pack_session(self.session, self.root / "draft.zip", True)

    def test_init_rejects_unsafe_ids_and_sha_before_writing(self):
        for identity in ("../escape", "/absolute", "a/b", ".hidden", "CON", "trailing.", "a..b"):
            with self.subTest(identity=identity), self.assertRaises(handoff.HandoffError):
                handoff.init_session(self.root, identity, "planner", "c" * 40, "input")
        with self.assertRaises(handoff.HandoffError):
            handoff.init_session(self.root, "bad-sha", "planner", "not-a-sha", "input")
        self.assertFalse((self.root / "bad-sha").exists())

    def test_handed_off_requires_completed_report(self):
        self.metadata(status="handed_off")
        with self.assertRaisesRegex(handoff.HandoffError, "placeholders"):
            handoff.validate_session(self.session)
        (self.session / "SESSION.md").write_text("# A report without sections", encoding="utf-8")
        with self.assertRaisesRegex(handoff.HandoffError, "completed section"):
            handoff.validate_session(self.session)
        (self.session / "SESSION.md").write_text("   \n", encoding="utf-8")
        with self.assertRaisesRegex(handoff.HandoffError, "empty"):
            handoff.validate_session(self.session)

    def test_partial_blocked_not_run_remain_honest_machine_results(self):
        for result in ("partial", "blocked", "not_run", "pass"):
            with self.subTest(result=result):
                self.seal(result)
                output = self.root / (result + ".zip")
                report = handoff.pack_session(self.session, output, True)
                self.assertEqual(report["result"], result)
                with zipfile.ZipFile(str(output)) as archive:
                    meta = json.loads(archive.read("session.json"))
                    manifest = json.loads(archive.read("hashmanifest.json"))
                    self.assertEqual(meta["result"], result)
                    self.assertEqual(manifest["result"], result)
                    self.assertIn(b"dirty tree: local patch retained", archive.read("SESSION.md"))

    def test_confirm_flag_and_both_manual_checks_required(self):
        self.seal()
        with self.assertRaisesRegex(handoff.HandoffError, "confirm-reviewed"):
            handoff.pack_session(self.session, self.root / "no-confirm.zip")
        for field in ("privacy_checked", "secrets_checked"):
            checks = {"secrets_checked": True, "privacy_checked": True}
            checks[field] = False
            self.metadata(publication_review=checks)
            with self.assertRaisesRegex(handoff.HandoffError, "publication_review"):
                handoff.pack_session(self.session, self.root / "unchecked.zip", True)
        self.assertFalse((self.root / "unchecked.zip").exists())

    def test_packing_is_deterministic_and_manifest_hashes_every_input(self):
        self.seal()
        self.evidence("nested/result.json", b'{"tests":2,"failed":0}\n')
        first = handoff.pack_session(self.session, self.root / "first.zip", True)
        second = handoff.pack_session(self.session, self.root / "second.zip", True)
        self.assertEqual(first["sha256"], second["sha256"])
        self.assertEqual(Path(first["output"]).read_bytes(), Path(second["output"]).read_bytes())
        with zipfile.ZipFile(first["output"]) as archive:
            self.assertEqual(archive.namelist(), sorted(archive.namelist()))
            manifest = json.loads(archive.read("hashmanifest.json"))
            self.assertEqual(len(manifest["files"]), len(archive.namelist()) - 1)
            self.assertEqual(manifest["base_code_sha"], "a" * 40)
            for item in manifest["files"]:
                data = archive.read(item["path"])
                self.assertEqual(item["bytes"], len(data))
                self.assertEqual(item["sha256"], hashlib.sha256(data).hexdigest())
                self.assertFalse(item["path"].startswith("/"))
            self.assertNotIn(str(self.root).encode(), archive.read("session.json"))
        self.assertEqual(Path(first["checksum"]).read_text().strip(), first["sha256"] + "  first.zip")

    def test_duplicate_zip_or_sidecar_never_overwritten(self):
        self.seal()
        output = self.root / "existing.zip"
        output.write_bytes(b"original")
        with self.assertRaisesRegex(handoff.HandoffError, "already exists"):
            handoff.pack_session(self.session, output, True)
        self.assertEqual(output.read_bytes(), b"original")
        other = self.root / "other.zip"
        sidecar = Path(str(other) + ".sha256")
        sidecar.write_bytes(b"original checksum")
        with self.assertRaisesRegex(handoff.HandoffError, "already exists"):
            handoff.pack_session(self.session, other, True)
        self.assertFalse(other.exists())
        self.assertEqual(sidecar.read_bytes(), b"original checksum")

    def test_racing_output_is_not_replaced_and_temps_are_removed(self):
        self.seal()
        output = self.root / "race.zip"
        real_link = os.link

        def competing_link(source, target):
            if Path(target) == output:
                output.write_bytes(b"competing writer")
            return real_link(source, target)

        with mock.patch.object(handoff.os, "link", side_effect=competing_link):
            with self.assertRaises(FileExistsError):
                handoff.pack_session(self.session, output, True)
        self.assertEqual(output.read_bytes(), b"competing writer")
        self.assertFalse(list(self.root.glob("handoff-tmp-*")))

    def test_racing_sidecar_rolls_back_only_our_zip(self):
        self.seal()
        output = self.root / "race-sidecar.zip"
        sidecar = Path(str(output) + ".sha256")
        real_link = os.link

        def competing_link(source, target):
            if Path(target) == sidecar:
                sidecar.write_bytes(b"competing checksum")
            return real_link(source, target)

        with mock.patch.object(handoff.os, "link", side_effect=competing_link):
            with self.assertRaises(FileExistsError):
                handoff.pack_session(self.session, output, True)
        self.assertFalse(output.exists())
        self.assertEqual(sidecar.read_bytes(), b"competing checksum")
        self.assertFalse(list(self.root.glob("handoff-tmp-*")))

    def test_rejects_secret_text_even_with_review_flags(self):
        self.seal()
        secrets = (b"password = unsafe-example", b"api_key: unsafe-example",
                   b"-----BEGIN RSA PRIVATE KEY-----\nexample\n",
                   ("ghp_" + "x" * 36).encode(), b"Authorization: Bearer abcdefghijklmnop",
                   b"https://user:unsafe-example@example.invalid/file")
        for secret in secrets:
            with self.subTest(secret=secret[:12]):
                self.evidence(data=secret)
                with self.assertRaisesRegex(handoff.HandoffError, "secret"):
                    handoff.validate_session(self.session)

    def test_redacted_text_is_allowed(self):
        self.seal()
        self.evidence(data=b"password: [REDACTED]\napi_key=<redacted>\n")
        self.assertTrue(handoff.validate_session(self.session)["valid"])

    def test_gps_requires_explicit_privacy_review(self):
        self.evidence(data=b'{"latitude_deg":35.0,"longitude_deg":139.0}\n')
        with self.assertRaisesRegex(handoff.HandoffError, "GPS"):
            handoff.validate_session(self.session)
        self.metadata(publication_review={"secrets_checked": False, "privacy_checked": True})
        result = handoff.validate_session(self.session)
        self.assertEqual(result["gps_coordinate_files"], ["evidence/check.txt"])
        self.assertIn("cannot guarantee", result["warning"])

    def test_csv_coordinates_also_require_review(self):
        self.evidence("positions.csv", b"time,latitude,longitude\n1,35.0,139.0\n")
        with self.assertRaisesRegex(handoff.HandoffError, "GPS"):
            handoff.validate_session(self.session)

    def test_hidden_unknown_rawbag_and_private_key_paths_rejected(self):
        for name in (".env", ".hidden/log.txt", "record.db3", "record.mcap", "record.bag",
                     "private.pem", "id_rsa.txt", "script.py", "hashmanifest.json.exe"):
            with self.subTest(name=name):
                path = self.evidence(name)
                with self.assertRaises(handoff.HandoffError):
                    handoff.validate_session(self.session)
                path.unlink()
                if name.startswith(".hidden/"):
                    path.parent.rmdir()

    def test_extra_root_files_and_input_manifest_are_rejected(self):
        for name in ("extra.txt", "hashmanifest.json"):
            path = self.session / name
            path.write_text("manual data", encoding="utf-8")
            with self.assertRaisesRegex(handoff.HandoffError, "unapproved"):
                handoff.validate_session(self.session)
            path.unlink()

    def test_symlink_file_and_escape_directory_are_rejected(self):
        outside = self.root / "outside"
        outside.mkdir()
        target = outside / "secret.txt"
        target.write_text("nonsecret sample", encoding="utf-8")
        link = self.session / "evidence" / "link.txt"
        try:
            link.symlink_to(target)
        except (OSError, NotImplementedError) as exc:
            self.skipTest("symlink creation not permitted on this platform: " + str(exc))
        with self.assertRaisesRegex(handoff.HandoffError, "symlinks"):
            handoff.validate_session(self.session)
        link.unlink()
        link = self.session / "evidence" / "escape"
        link.symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(handoff.HandoffError, "symlinks"):
            handoff.validate_session(self.session)

    def test_windows_reparse_junction_attribute_is_rejected(self):
        target = self.evidence()
        original = Path.lstat

        def simulate_reparse(path, *args, **kwargs):
            value = original(path, *args, **kwargs)
            if path == target:
                return SimpleNamespace(st_mode=value.st_mode, st_file_attributes=0x400)
            return value

        with mock.patch.object(Path, "lstat", simulate_reparse):
            with self.assertRaisesRegex(handoff.HandoffError, "junctions"):
                handoff.validate_session(self.session)

    def test_output_cannot_be_inside_input_or_have_wrong_extension(self):
        self.seal()
        for output in (self.session / "report.zip", self.root / "report.txt", self.root / ".report.zip"):
            with self.subTest(output=output), self.assertRaises(handoff.HandoffError):
                handoff.pack_session(self.session, output, True)

    def test_disguised_text_image_cannot_skip_scans(self):
        self.evidence("screen.png", b"password=unsafe-example")
        with self.assertRaisesRegex(handoff.HandoffError, "image signature"):
            handoff.validate_session(self.session)

    def test_binary_images_are_not_falsely_treated_as_utf8_text(self):
        self.seal()
        self.evidence("screen.png", b"\x89PNG\r\n\x1a\n\xff\x00\xfe")
        self.assertTrue(handoff.validate_session(self.session)["valid"])

    def test_file_total_and_count_bounds(self):
        oversized = self.evidence(data=b"x" * (handoff.MAX_FILE_BYTES + 1))
        with self.assertRaisesRegex(handoff.HandoffError, "oversized"):
            handoff.validate_session(self.session)
        oversized.unlink()
        with mock.patch.object(handoff, "MAX_TOTAL_BYTES", 200):
            with self.assertRaisesRegex(handoff.HandoffError, "total byte"):
                handoff.validate_session(self.session)
        with mock.patch.object(handoff, "MAX_FILES", 2):
            with self.assertRaisesRegex(handoff.HandoffError, "too many files"):
                handoff.validate_session(self.session)

    def test_non_utf8_text_and_nonfinite_or_duplicate_json_rejected(self):
        path = self.evidence(data=b"\xff\xfe")
        with self.assertRaisesRegex(handoff.HandoffError, "UTF-8"):
            handoff.validate_session(self.session)
        path.unlink()
        metadata = self.session / "session.json"
        original = metadata.read_text().rstrip()
        metadata.write_text(original[:-1] + ', "id":"duplicate"}', encoding="utf-8")
        with self.assertRaisesRegex(handoff.HandoffError, "duplicate"):
            handoff.validate_session(self.session)
        metadata.write_text(original[:-1] + ', "extra":NaN}', encoding="utf-8")
        with self.assertRaisesRegex(handoff.HandoffError, "nonfinite"):
            handoff.validate_session(self.session)

    def test_stale_handoff_acknowledgement_rejected(self):
        self.metadata(consumed_handoff_id="plan-older")
        with self.assertRaisesRegex(handoff.HandoffError, "acknowledge"):
            handoff.validate_session(self.session)

    def test_session_id_must_match_directory(self):
        self.metadata(id="another-id")
        with self.assertRaisesRegex(handoff.HandoffError, "directory name"):
            handoff.validate_session(self.session)

    def test_source_mutation_during_snapshot_fails(self):
        self.seal()
        evidence = self.evidence()
        original = handoff._read_file

        def mutate_after_read(path, root):
            result = original(path, root)
            if path == evidence:
                evidence.write_bytes(b"changed after read, before archive construction")
            return result

        with mock.patch.object(handoff, "_read_file", side_effect=mutate_after_read):
            with self.assertRaisesRegex(handoff.HandoffError, "changed during validation"):
                handoff.pack_session(self.session, self.root / "changed.zip", True)
        self.assertFalse((self.root / "changed.zip").exists())

    def test_remote_references_require_hash_size_and_access_note_no_download(self):
        artifact = {"url": "https://example.invalid/private/report.zip", "sha256": "b" * 64,
                    "bytes": 123, "access_note": "Private lab repository; recipient needs project access."}
        self.metadata(external_artifacts=[artifact])
        self.assertEqual(handoff.validate_session(self.session)["external_artifacts"], 1)
        for key, bad in (("sha256", "unknown"), ("bytes", -1), ("access_note", ""),
                         ("url", "file:///private/report.zip"), ("url", "https://user:password@example.invalid/a"),
                         ("url", "https://example.invalid/file.zip?sv=2026-01-01&sig=fake_signed_credential"),
                         ("url", "https://example.invalid/file.zip?X-Amz-Signature=fake_signed_credential"),
                         ("url", "https://example.invalid/file.zip#private-access-code")):
            with self.subTest(key=key):
                altered = dict(artifact)
                altered[key] = bad
                self.metadata(external_artifacts=[altered])
                with self.assertRaises(handoff.HandoffError):
                    handoff.validate_session(self.session)

    def test_validate_cli_json_and_nonzero_failure(self):
        output, error = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(error):
            status = handoff.main(["validate", "--session", str(self.session)])
        self.assertEqual(status, 0)
        self.assertTrue(json.loads(output.getvalue())["valid"])
        output, error = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(error):
            status = handoff.main(["pack", "--session", str(self.session), "--output", str(self.root / "missing-review.zip")])
        self.assertNotEqual(status, 0)
        self.assertFalse(json.loads(error.getvalue())["valid"])


if __name__ == "__main__":
    unittest.main()
