"""Repository handoff integrity checks, without Git/network/ROS dependencies."""

import hashlib
import json
from pathlib import Path
import re
import sys
import tempfile
import unittest
from urllib.parse import unquote, urlsplit
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import agent_handoff


ROOT = Path(__file__).resolve().parents[2]
CONTEXT = ROOT / "docs" / "agent-context"


class RepositoryContextTests(unittest.TestCase):
    def test_canonical_repository_and_vehicle_layout_are_explicit(self):
        instructions = (ROOT / "AGENTS.md").read_text(encoding="utf-8")
        self.assertIn("Sophia-AI-formula-team/neo_aiformula_sophia", instructions)
        self.assertIn("1303517209", instructions)
        context = (ROOT / "AGENT_CONTEXT.md").read_text(encoding="utf-8")
        self.assertNotIn("pid_ws/src/", context)
        control = ROOT / "workspace" / "src" / "aiformula" / "control"
        for package in ("lane_mapping_lya_reference", "lane_mapping_fixed", "lane_mapping_fixed_gnss"):
            self.assertTrue((control / package / "package.xml").is_file())
        workflow = (ROOT / ".github" / "workflows" / "agent-handoff.yml").read_text(encoding="utf-8")
        self.assertIn("GITHUB_REPOSITORY_ID", workflow)
        self.assertIn("1303517209", workflow)
        self.assertRegex(workflow, r"(?m)^\s+\.github/workflows\s*$")

    def test_entrypoints_and_local_links(self):
        paths = [ROOT / "AGENTS.md", ROOT / "AGENT_CONTEXT.md", ROOT / "README.md"]
        paths.extend(CONTEXT.rglob("*.md"))
        self.assertTrue((CONTEXT / "STATUS.md").is_file())
        self.assertTrue((CONTEXT / "handoffs").is_dir())
        # Only our new entrypoints/context: the historical README may contain
        # unrelated links whose availability is not part of this check.
        for path in paths:
            document = path.read_text(encoding="utf-8")
            if path == ROOT / "README.md":
                document = document.split("## Repository layout")[0]
            for target in re.findall(r"\[[^\]\n]*\]\(([^)\s]+)\)", document):
                parsed = urlsplit(target)
                if parsed.scheme or not parsed.path:
                    continue
                resolved = (path.parent / unquote(parsed.path)).resolve()
                with self.subTest(document=str(path.relative_to(ROOT)), target=target):
                    resolved.relative_to(ROOT)
                    self.assertTrue(resolved.exists(), "broken local context link")

    def test_published_sessions_are_reviewed_and_sealed(self):
        sessions = sorted((CONTEXT / "sessions").glob("*/session.json"))
        self.assertTrue(sessions, "a seed planner record is required")
        for metadata in sessions:
            with self.subTest(session=metadata.parent.name):
                result = agent_handoff.validate_session(metadata.parent)
                self.assertTrue(result["valid"])
                self.assertTrue(result["pack_eligible"], "draft belongs outside published sessions")

    def test_bundles_match_readable_records_and_hashes(self):
        for metadata in sorted((CONTEXT / "sessions").glob("*/session.json")):
            session = metadata.parent
            archive_path = CONTEXT / "bundles" / (session.name + ".zip")
            checksum_path = Path(str(archive_path) + ".sha256")
            with self.subTest(session=session.name):
                data = archive_path.read_bytes()
                digest = hashlib.sha256(data).hexdigest()
                self.assertEqual(checksum_path.read_text(encoding="utf-8"),
                                 digest + "  " + archive_path.name + "\n")
                with zipfile.ZipFile(archive_path) as archive:
                    manifest = json.loads(archive.read("hashmanifest.json"))
                    paths = [entry["path"] for entry in manifest["files"]]
                    self.assertEqual(sorted(archive.namelist()), sorted(paths + ["hashmanifest.json"]))
                    for entry in manifest["files"]:
                        payload = archive.read(entry["path"])
                        self.assertEqual(payload, (session / entry["path"]).read_bytes())
                        self.assertEqual(len(payload), entry["bytes"])
                        self.assertEqual(hashlib.sha256(payload).hexdigest(), entry["sha256"])
                # Repack independently to catch extra files, changed metadata or
                # platform-specific checkout line-ending changes as well.
                with tempfile.TemporaryDirectory(prefix="context-test-") as temp:
                    repacked = Path(temp) / archive_path.name
                    agent_handoff.pack_session(session, repacked, confirm_reviewed=True)
                    self.assertEqual(data, repacked.read_bytes())

    def test_no_orphan_bundles(self):
        names = {path.parent.name for path in (CONTEXT / "sessions").glob("*/session.json")}
        expected = {name + suffix for name in names for suffix in (".zip", ".zip.sha256")}
        actual = {path.name for path in (CONTEXT / "bundles").iterdir()}
        self.assertEqual(expected, actual)


if __name__ == "__main__":
    unittest.main()
