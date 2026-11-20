"""Check that the 1121 upgrade and its rollback work over a real 1120 receipt."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "2026-11-20-mapassist/concept"))
from shortcuts import Release, V6278, migrate


class Upgrade(unittest.TestCase):
    def test_rollback_restores_1120_command_and_icons(self):
        with tempfile.TemporaryDirectory() as directory, patch("shortcuts.validate_runtime"):
            home = Path(directory)
            before_desktop = home / "Desktop first"
            after_desktop = home / "Desktop after"
            menu = home / ".local/share/applications"
            release = Release(HERE, "1121 experiment", "concept-shortcuts-1121", previous=V6278)
            migrate(home, before_desktop, menu, "apply")
            command = home / ".local/bin/concept-live"
            alias = command.with_name("renamed-concept")
            alias.write_bytes(command.read_bytes())
            alias.chmod(0o741)
            protected = command.with_name("lya")
            protected.write_text("vehicle LYA")
            original_paths = [command, alias, before_desktop / "Concept.desktop", menu / "Concept.desktop"]
            originals = {path: (path.read_bytes(), path.stat().st_mode) for path in original_paths}
            migrate(home, after_desktop, menu, "apply", release)
            migrate(home, after_desktop, menu, "check", release)
            for path in (command, alias):
                self.assertEqual(path.read_bytes(), (HERE / "concept-live").read_bytes())
            self.assertIn("1121 experiment", original_paths[2].read_text())
            migrate(home, None, menu, "rollback", release)
            for path, (data, mode) in originals.items():
                self.assertEqual((path.read_bytes(), path.stat().st_mode), (data, mode))
            self.assertFalse((after_desktop / "Concept.desktop").exists())
            self.assertEqual(protected.read_text(), "vehicle LYA")
            migrate(home, before_desktop, menu, "check")


if __name__ == "__main__":
    unittest.main()
