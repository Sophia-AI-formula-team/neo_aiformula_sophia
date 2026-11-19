"""Linux regression for migrating the vehicle user's active shortcuts."""
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import shortcuts


class ShortcutUpgrade(unittest.TestCase):
    def test_retire_duplicates_and_preserve_current_concept(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            desktop = home / "Desktop"
            menu = home / ".local/share/applications"
            commands = home / ".local/bin"
            env_bin = home / "test-bin"
            for path in (desktop, menu, commands, env_bin):
                path.mkdir(parents=True)
            runtime = home / "aiformula_runtime/2026-11-20-mapassist-av314"
            (runtime / "install").mkdir(parents=True)
            (runtime / "install/local_setup.bash").touch()
            selector = home / "aiformula_runtime/2026-11-19-yolop-selection/yolop.launch"
            selector.parent.mkdir(parents=True)
            selector.touch()
            (commands / "yolop-launch").symlink_to(selector)
            (commands / "yolop").symlink_to("yolop-launch")
            (commands / "old-selector-copy").symlink_to(selector)
            for path in (commands / "concept-live", desktop / "Concept.desktop"):
                path.write_text("owned by current Concept release\n")

            xdg = env_bin / "xdg-user-dir"
            xdg.write_text('#!/bin/sh\necho "$HOME/Desktop"\n')
            xdg.chmod(0o755)
            env = {"HOME": str(home), "XDG_DATA_HOME": str(menu.parent),
                   "PATH": os.pathsep.join((str(env_bin), str(commands), "/usr/bin", "/bin"))}
            with patch.dict(os.environ, env):
                shortcuts.migrate(home, desktop, True)
                self.assertFalse(os.path.lexists(commands / "old-selector-copy"))
                self.assertFalse(os.path.lexists(commands / "yolop-launch"))
                for index, name in enumerate(("allnodes", "yolop", "mapassist", "lya"), start=1):
                    entry = (desktop / ("1120-" + name + ".desktop")).read_text()
                    self.assertIn(f"Name=1120 {index} {name}\n", entry)
                    self.assertIn(f'Exec="{commands / name}"\n', entry)
                shutil.copy2(desktop / "1120-mapassist.desktop", menu / "old-mapassist.desktop")
                shutil.copy2(commands / "mapassist", commands / "old-mapassist")
                with self.assertRaises(SystemExit):
                    shortcuts.migrate(home, desktop, False, check=True)
                shortcuts.migrate(home, desktop, True)
                shortcuts.migrate(home, desktop, False, check=True)
                self.assertFalse((commands / "old-mapassist").exists())
                self.assertFalse((menu / "old-mapassist.desktop").exists())
                for path in (commands / "concept-live", desktop / "Concept.desktop"):
                    self.assertEqual(path.read_text(), "owned by current Concept release\n")

                def shell_check(prefix):
                    return subprocess.run(
                        ["bash", "--noprofile", "--norc", "-ic", prefix + 'source "$1"',
                         "bash", str(shortcuts.HERE / "verify-shortcuts.bash")],
                        capture_output=True, text=True)
                clean = shell_check("")
                self.assertEqual(clean.returncode, 0, clean.stderr)
                self.assertNotEqual(shell_check("alias lya='old-lya'; ").returncode, 0)
                self.assertNotEqual(shell_check("semantic() { true; }; ").returncode, 0)


if __name__ == "__main__":
    unittest.main()
