"""Linux regression for migrating the vehicle user's active shortcuts."""
import os
import hashlib
import json
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import shortcuts


class ShortcutUpgrade(unittest.TestCase):
    def test_original_concept_entries_remain_available_until_concept_upgrade(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            commands, desktop = home / ".local/bin", home / "Desktop"
            commands.mkdir(parents=True)
            desktop.mkdir()
            wrapper = b'#!/usr/bin/env bash\nset -e\nsource "$HOME/concept_v6275_ws/activate_concept.bash"\nexec ros2 run concept live "$@"\n'
            entry = (b'[Desktop Entry]\nVersion=1.0\nType=Application\nName=Concept\n'
                     b'Comment=Launch the Concept panel and RViz\nExec="/home/control/.local/bin/concept-live"\n'
                     b'Icon=applications-engineering\nTerminal=true\n')
            old = json.loads((shortcuts.HERE / "legacy-sha256.json").read_text())
            for path, content in ((commands / "concept-live", wrapper), (desktop / "Concept.desktop", entry)):
                self.assertEqual(hashlib.sha256(content).hexdigest(), old[path.name])
                path.write_bytes(content)
            selector = home / "aiformula_runtime/2026-11-19-yolop-selection/yolop.launch"
            selector.parent.mkdir(parents=True)
            selector.touch()
            setup = home / "aiformula_runtime/2026-11-20-mapassist-av314/install/local_setup.bash"
            setup.parent.mkdir(parents=True)
            setup.touch()
            with patch.dict(os.environ, {"XDG_DATA_HOME": str(home / ".local/share")}):
                shortcuts.migrate(home, desktop, True)
                shortcuts.migrate(home, desktop, False, check=True)
            self.assertEqual((commands / "concept-live").read_bytes(), wrapper)
            self.assertEqual((desktop / "Concept.desktop").read_bytes(), entry)

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
                map_icon = desktop / "1120-mapassist.desktop"
                self.assertIn(f'Exec="{commands / "mapassist"}" inner 450\n', map_icon.read_text())
                # Upgrade a previously installed plain 1120 MapAssist desktop entry.
                map_icon.write_text(map_icon.read_text().split("Actions=Inner450;")[0])
                shortcuts.migrate(home, desktop, True)
                self.assertIn("[Desktop Action Inner450]", map_icon.read_text())
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
