"""Linux tests for dependency migration and deletion of the previous installs."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from legacy_install import installed_paths, migrate_dependencies, python_value, retire


class LegacyInstall(unittest.TestCase):
    def setUp(self):
        work = tempfile.TemporaryDirectory()
        self.addCleanup(work.cleanup)
        self.home = Path(work.name).resolve()
        self.new = self.home / "concept_1121_ws"
        self.new.mkdir()
        self.old = self.home / "concept_v6275_ws"
        self.old.mkdir()
        self.second = self.home / "concept_1120_v6278_ws"
        self.second.mkdir()

    def test_copied_dependency_works_after_old_paths_are_removed(self):
        venv = self.old / "venv"
        subprocess.run([sys.executable, "-m", "venv", "--without-pip", str(venv)], check=True)
        site = Path(python_value(venv / "bin/python3", "import sysconfig; print(sysconfig.get_path('purelib'))"))
        (site / "fixture_dep.py").write_text("VERSION = 1121\n")
        (self.new / "venv").mkdir()
        (self.new / ".copying-legacy-venv").touch()
        migrate_dependencies(self.home, self.new)
        self.assertFalse((self.new / ".copying-legacy-venv").exists())

        repo = self.home / "src/neo-concept-v6275"
        package = repo / "vehicle_concept/src/concept/package.xml"
        package.parent.mkdir(parents=True)
        package.write_text("<package/>")
        git = ["git", "-C", str(repo)]
        subprocess.run(git + ["init", "-q"], check=True)
        subprocess.run(git + ["remote", "add", "origin",
                              "https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git"], check=True)
        subprocess.run(git + ["add", "."], check=True)
        subprocess.run(git + ["-c", "user.name=Fixture", "-c", "user.email=fixture@example.invalid",
                              "commit", "-qm", "fixture"], check=True)
        with self.assertRaisesRegex(ValueError, "save its local changes"):
            package.write_text("<edited/>")
            list(installed_paths(self.home))
        package.write_text("<package/>")

        command = [str(self.new / "venv/bin/python3"), "-I", "-c",
                   "import fixture_dep; from pathlib import Path; "
                   "assert fixture_dep.VERSION == 1121; "
                   f"assert not any(Path(p).exists() for p in {list(map(str, (self.old, self.second, repo)))!r})"]
        retire(self.home, self.new, command)
        self.assertFalse(any(path.exists() for path in (self.old, self.second, repo)))
        self.assertTrue((self.new / ".legacy-retired").exists())
        retire(self.home, self.new, command)

    def test_failed_verification_restores_previous_workspaces(self):
        with self.assertRaises(subprocess.CalledProcessError):
            retire(self.home, self.new, [sys.executable, "-c", "raise SystemExit(4)"])
        self.assertTrue(self.old.is_dir() and self.second.is_dir())
        self.assertFalse((self.new / ".legacy-retired").exists())

    def test_unrecognized_data_and_symlinks_block_removal(self):
        extra = self.old / "my-recording"
        extra.mkdir()
        with self.assertRaisesRegex(ValueError, "my-recording"):
            list(installed_paths(self.home))
        extra.rmdir()
        self.old.rmdir()
        self.old.symlink_to(self.second, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "without symlink"):
            list(installed_paths(self.home))


if __name__ == "__main__":
    unittest.main()
