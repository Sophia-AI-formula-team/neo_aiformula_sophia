"""ROS-free mock contract for smoke-only failure diagnostics.

This imports the local smoke module but never creates a node or imports its ROS
runtime dependencies. DDS behavior is a separate integration test.
"""
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).with_name("runtime_smoke.py")
SPEC = importlib.util.spec_from_file_location("reviewed_runtime_smoke", SOURCE)
SMOKE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SMOKE)


class Clock:
    def __init__(self):
        self.ticks = 0
        self.pumps = []

    def now(self):
        return self.ticks / 100.

    def pump(self, duration):
        self.pumps.append(duration)
        self.ticks += 1


class SmokeDiagnosticTests(unittest.TestCase):
    def test_already_satisfied_does_not_collect_diagnostics(self):
        clock = Clock()
        SMOKE._wait_for(lambda: True, "five commits", clock.pump,
                        lambda: self.fail("unexpected diagnostic read"), clock=clock.now)
        self.assertEqual(clock.pumps, [])

    def test_four_commits_and_terminal_is_failure_with_original_reason(self):
        clock, details = Clock(), {"engine_counts": {"accepted": 4}}
        with self.assertRaises(SMOKE.SmokeFailure) as caught:
            SMOKE._wait_for(lambda: 4 >= 5, "five commits", clock.pump, lambda: details,
                bootstrap_terminal_reason=lambda: "bootstrap_anchor_not_established", clock=clock.now)
        self.assertIn("terminal engine: bootstrap_anchor_not_established", str(caught.exception))
        self.assertIs(caught.exception.diagnostics, details)
        self.assertEqual(clock.pumps, [])

    def test_original_five_commit_predicate_is_not_bypassed(self):
        clock = Clock()
        SMOKE._wait_for(lambda: clock.ticks >= 5, "five commits", clock.pump,
                        lambda: self.fail("success read diagnostics"), clock=clock.now)
        self.assertEqual(clock.pumps, [.01] * 5)

    def test_timeout_still_fails_and_has_diagnostics(self):
        clock, details = Clock(), {"worker_failure": None, "last_completed_camera": None}
        with self.assertRaises(SMOKE.SmokeFailure) as caught:
            SMOKE._wait_for(lambda: False, "DDS discovery", clock.pump, lambda: details,
                            timeout=.03, clock=clock.now)
        self.assertTrue(str(caught.exception).startswith("timeout: DDS discovery;"))
        self.assertEqual(clock.pumps, [.01] * 3)
        self.assertEqual(caught.exception.diagnostics, details)

    def test_diagnostic_exception_cannot_replace_original_failure(self):
        def unavailable():
            raise OSError("journal unavailable")
        with self.assertRaises(SMOKE.SmokeFailure) as caught:
            SMOKE._wait_for(lambda: False, "five commits", lambda _: self.fail("pump"), unavailable,
                bootstrap_terminal_reason=lambda: "bootstrap_anchor_not_established", clock=lambda: 0.)
        self.assertIn("bootstrap_anchor_not_established", str(caught.exception))
        self.assertEqual(caught.exception.diagnostics, {"diagnostics_unavailable": "journal unavailable"})

    def test_numeric_rotation_and_partial_write_keep_latest_complete_event(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            old = dict(event="teach_mask", source_ns=1, compute_ms=999., result=dict(reason="old"))
            new = dict(event="teach_mask", source_ns=20, compute_ms=123.,
                       result=dict(reason="processing_deadline", processing_ms=90., counts={"rejected": 1}))
            (path / "commands-9.jsonl").write_text(json.dumps(old) + "\n", encoding="utf-8")
            (path / "commands-10.jsonl").write_bytes(
                (json.dumps(new) + '\n{"event":"teach_mask"').encode("utf-8"))
            evidence = SMOKE._last_completed_camera(path)
            self.assertEqual(evidence["journal_file"], "commands-10.jsonl")
            self.assertEqual(evidence["reason"], "processing_deadline")
            self.assertEqual(evidence["compute_ms"], 123.)
            self.assertEqual(evidence["counts_at_result"], {"rejected": 1})

    def test_bounded_tail_does_not_claim_older_truncated_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            row = json.dumps(dict(event="teach_mask", source_ns=1, compute_ms=1.)) + "\n"
            commands = '{"event":"command"}\n' * 15000
            (Path(directory) / "commands-0.jsonl").write_text(row + commands, encoding="utf-8")
            self.assertIsNone(SMOKE._last_completed_camera(directory))

    def test_source_age_is_at_diagnostic_not_relabelled_as_commit_age(self):
        with tempfile.TemporaryDirectory() as directory:
            row = dict(event="teach_mask", source_ns=2000000, compute_ms=42.,
                       result=dict(reason="bootstrap_odometry", processing_ms=21., counts={"accepted": 1}))
            (Path(directory) / "commands-0.jsonl").write_text(json.dumps(row) + "\n", encoding="utf-8")
            engine = SimpleNamespace(phase="invalid", _fault="bootstrap_anchor_not_established",
                                     counts={"accepted": 1, "rejected": 2})
            node = SimpleNamespace(worker=SimpleNamespace(engine=engine, failure=None,
                    thread=SimpleNamespace(is_alive=lambda: True)), last_mask_ns=9000000,
                journal=SimpleNamespace(directory=directory),
                safety=SimpleNamespace(snapshot=lambda: {"state": "TEACH"}),
                get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10000000)))
            evidence = SMOKE._runtime_diagnostics(node)
            self.assertEqual(evidence["engine_counts"], {"accepted": 1, "rejected": 2})
            self.assertEqual(evidence["latest_received_mask_age_ms"], 1.)
            self.assertEqual(evidence["last_completed_camera"]["source_age_at_diagnostic_ms"], 8.)
            self.assertEqual(evidence["last_completed_camera"]["compute_ms"], 42.)
            self.assertIn("not commit-time ages", evidence["evidence_semantics"])
            evidence["engine_counts"]["accepted"] = 9
            self.assertEqual(engine.counts["accepted"], 1)


if __name__ == "__main__":
    unittest.main()
