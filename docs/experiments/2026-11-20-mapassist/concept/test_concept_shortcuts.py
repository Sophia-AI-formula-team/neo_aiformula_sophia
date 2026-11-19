"""Linux integration tests. Hardware / ROS launch are replaced by a recording executable."""
import fcntl
import json
import os
from pathlib import Path
import pty
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import shortcuts


OLD_COMMAND = b'#!/usr/bin/env bash\nset -e\nsource "$HOME/concept_v6275_ws/activate_concept.bash"\nexec ros2 run concept live "$@"\n'
OLD_ICON = (b'[Desktop Entry]\nVersion=1.0\nType=Application\nName=Concept\n'
            b'Comment=Launch the Concept panel and RViz\nExec="/home/control/.local/bin/concept-live"\n'
            b'Icon=applications-engineering\nTerminal=true\n')


def put(path, content, mode=0o755):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content if isinstance(content, bytes) else content.encode())
    path.chmod(mode)


class VehicleConceptShortcuts(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(prefix="concept user ")
        self.addCleanup(tmp.cleanup)
        self.home = Path(tmp.name)
        self.desktop = self.home / "Localized Desktop"
        self.hidden = self.home / ".xdg-hidden/Desktop/Concept.desktop"
        self.menu = self.home / ".local/share/applications/Concept.desktop"
        self.command = self.home / ".local/bin/concept-live"
        self.state = self.home / ".local/state/aiformula/concept-shortcuts-v6278/current.json"
        self.workspace = self.home / "concept_1120_v6278_ws"
        self.repo = self.home / "src/neo-concept-1120-v6278"
        self.tools = self.home / "fixture-bin"
        self.pin = "f305d5f7acea5c87af0e2003bec13bdc06199d8f"
        self.env = dict(os.environ, HOME=str(self.home), XDG_DATA_HOME=str(self.menu.parent.parent),
                        PATH=os.pathsep.join([str(self.tools), "/usr/bin", "/bin"]),
                        EXAMPLE_PIN=self.pin)
        self.launch = self.home / "fake-launch.json"
        put(self.tools/"xdg-user-dir", '#!/bin/sh\nprintf "%s\\n" "$HOME/Localized Desktop"\n')
        put(self.tools/"git", '#!/bin/sh\ncase "$3" in rev-parse) echo "$EXAMPLE_PIN";; diff) exit 0;; *) exit 10;; esac\n')
        put(self.repo/"vehicle_concept/src/concept/package.xml", "<package/>")
        put(self.workspace/"activate_concept.bash", 'export FIXTURE_FOXY_PYTHON_ENV=active\n')
        put(self.repo/"vehicle_concept/checks/check_overlay.py",
            'import os,sys\nfrom pathlib import Path\n'
            'assert os.environ["FIXTURE_FOXY_PYTHON_ENV"] == "active"\n'
            'assert Path(sys.argv[1]) == Path.home()/"concept_1120_v6278_ws"\n')
        put(self.tools/"ros2", '#!/usr/bin/env python3\nimport os,sys,json\nfrom pathlib import Path\n'
            '(Path.home()/"fake-launch.json").write_text(json.dumps([sys.argv[1:],os.isatty(0)]))\n')
        put(self.hidden, OLD_ICON, 0o751)
        put(self.command, OLD_COMMAND, 0o750)
        self.assertEqual(shortcuts.digest(OLD_COMMAND), shortcuts.LEGACY["concept-live"])
        self.assertEqual(shortcuts.digest(OLD_ICON), shortcuts.LEGACY["Concept.desktop"])
        for name in ("allnodes", "yolop", "mapassist", "lya"):
            put(self.command.with_name(name), "unchanged " + name)
            put(self.desktop / ("1120-"+name+".desktop"), "unchanged desktop " + name)

    def args(self, *args):
        return [sys.executable, str(shortcuts.HERE / "shortcuts.py"), *args]

    def cli(self, *args, ok=True, env=None):
        result = subprocess.run(self.args(*args), env=dict(self.env, **(env or {})),
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertEqual(result.returncode == 0, ok, (result.stdout, result.stderr))
        return result

    def test_installs_in_place_and_restores_archive_bytes_modes_and_absent_entries(self):
        self.cli()
        self.assertEqual(self.command.read_bytes(), OLD_COMMAND)
        self.assertFalse(self.state.exists())
        self.cli("--apply")
        self.cli("--check")
        original_receipt = self.state.read_bytes()
        self.cli("--apply")
        self.assertEqual(self.state.read_bytes(), original_receipt)
        self.assertFalse(self.launch.exists())  # installation and checking never launch ROS
        for p in (self.desktop/"Concept.desktop", self.hidden, self.menu):
            self.assertIn('Exec="' + str(self.command) + '"', p.read_text())
            self.assertIn("Terminal=true\n", p.read_text())
        master, slave = pty.openpty()
        try:
            subprocess.run([str(self.command), "tracking_controller:=lya", "panel:=false", "a b"],
                           stdin=slave, stdout=slave, stderr=slave, env=self.env, check=True)
        finally:
            os.close(master)
            os.close(slave)
        self.assertEqual(json.loads(self.launch.read_text()), [["run","concept","live","tracking_controller:=lya","panel:=false","a b"], True])
        self.cli("--rollback")
        self.cli("--rollback")
        self.assertEqual(shortcuts.snapshot(self.command), [shortcuts.LEGACY["concept-live"],0o750])
        self.assertEqual(shortcuts.snapshot(self.hidden), [shortcuts.LEGACY["Concept.desktop"],0o751])
        self.assertFalse(self.menu.exists())
        self.assertFalse((self.desktop/"Concept.desktop").exists())
        self.assertFalse(self.state.exists())
        for name in ("allnodes","yolop","mapassist","lya"):
            self.assertEqual(self.command.with_name(name).read_text(), "unchanged " + name)
            self.assertEqual((self.desktop/("1120-"+name+".desktop")).read_text(), "unchanged desktop " + name)

    def test_refuses_unrecognized_shortcut_stale_pin_and_modified_rollback(self):
        put(self.menu, "other application")
        self.cli("--apply", ok=False)
        self.assertEqual(self.command.read_bytes(), OLD_COMMAND)
        self.menu.unlink()
        self.cli("--apply", ok=False, env={"EXAMPLE_PIN":"wrong"})
        self.assertEqual(self.command.read_bytes(), OLD_COMMAND)
        self.cli("--apply")
        self.menu.write_text("operator change after installation")
        self.cli("--rollback", ok=False)
        self.assertEqual(self.menu.read_text(), "operator change after installation")
        self.assertNotEqual(self.command.read_bytes(), OLD_COMMAND)
        self.menu.unlink()
        self.cli("--rollback")
        self.assertEqual(self.command.read_bytes(), OLD_COMMAND)

    def test_renamed_terminal_and_desktop_entries_and_symlink_are_restored(self):
        alias = self.command.with_name("concept-live-old")
        old_link = self.command.with_name("use-concept")
        copied = self.desktop/"Concept copy.desktop"
        foreign = self.desktop/"custom.desktop"
        put(alias, OLD_COMMAND, 0o741)
        old_link.symlink_to("concept-live")
        put(copied, OLD_ICON, 0o641)
        put(foreign, b'[Desktop Entry]\nExec=different-app\n')
        self.cli("--apply")
        for path in (alias, old_link):
            self.assertEqual(path.read_bytes(), self.command.read_bytes())
        self.assertEqual(copied.read_bytes(), self.menu.read_bytes())
        self.assertEqual(foreign.read_bytes(), b'[Desktop Entry]\nExec=different-app\n')
        self.cli("--apply")
        self.cli("--check")
        self.cli("--rollback")
        self.assertEqual(shortcuts.snapshot(alias), [shortcuts.LEGACY["concept-live"], 0o741])
        self.assertEqual(shortcuts.snapshot(copied), [shortcuts.LEGACY["Concept.desktop"], 0o641])
        self.assertTrue(old_link.is_symlink())
        self.assertEqual(os.readlink(old_link), "concept-live")

    def test_xdg_desktop_disabled_does_not_create_an_icon_in_home(self):
        put(self.tools/"xdg-user-dir", '#!/bin/sh\necho "$HOME"\n')
        self.cli("--apply")
        self.cli("--check")
        self.assertTrue(self.hidden.is_file() and self.menu.is_file())
        self.assertFalse((self.home/"Concept.desktop").exists())
        self.cli("--rollback")
        self.assertEqual(self.hidden.read_bytes(), OLD_ICON)
        self.assertFalse(self.menu.exists())

    def test_missing_xdg_utility_selects_existing_standard_desktop_and_rollback_needs_no_xdg(self):
        fallback = self.home/"Desktop"
        with mock.patch.object(subprocess,"check_output",side_effect=FileNotFoundError):
            self.assertIsNone(shortcuts.xdg_desktop(self.home))
            fallback.mkdir()
            self.assertEqual(shortcuts.xdg_desktop(self.home), fallback)
        # Equivalent to a missing/failed helper in the CLI's reduced PATH.
        put(self.tools/"xdg-user-dir", '#!/bin/sh\nexit 127\n')
        self.cli("--apply")
        self.assertTrue((fallback/"Concept.desktop").is_file())
        self.assertTrue(self.menu.is_file() and self.hidden.is_file())
        put(self.tools/"xdg-user-dir", '#!/bin/sh\ntouch "$HOME/xdg-was-run"\nexit 3\n')
        self.cli("--rollback")
        self.assertFalse((self.home/"xdg-was-run").exists())
        self.assertEqual(self.hidden.read_bytes(), OLD_ICON)
        self.assertFalse((fallback/"Concept.desktop").exists())

    def test_finds_archived_hidden_icon_after_an_older_mapassist_migration(self):
        self.hidden.unlink()
        archive = self.home / "aiformula_runtime/2026-11-20-mapassist-av314/shortcut-backup-test"
        saved = archive / self.hidden.relative_to(self.home)
        put(saved, OLD_ICON)
        self.cli("--apply")
        self.assertIn("V6.27.8", self.hidden.read_text())
        self.cli("--rollback")
        self.assertFalse(self.hidden.exists())
        self.assertEqual(saved.read_bytes(), OLD_ICON)

    def test_refuses_foreign_symlink_and_bad_overlay(self):
        self.menu.parent.mkdir(parents=True, exist_ok=True)
        self.menu.symlink_to(self.command)
        self.cli("--apply", ok=False)
        self.menu.unlink()
        put(self.repo/"vehicle_concept/checks/check_overlay.py", "raise SystemExit(7)")
        self.cli("--apply",ok=False)
        self.assertFalse(self.state.exists())
        self.assertEqual(self.command.read_bytes(), OLD_COMMAND)

    def test_shared_mapassist_lock_serializes_command(self):
        lockpath = self.home/".cache/aiformula-1120-shortcuts.lock"
        lockpath.parent.mkdir()
        with lockpath.open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            child = subprocess.Popen(self.args("--apply"), env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                with self.assertRaises(subprocess.TimeoutExpired):
                    child.communicate(timeout=.35)
                self.assertEqual(self.command.read_bytes(), OLD_COMMAND)
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)
            out, err = child.communicate(timeout=15)
            self.assertEqual(child.returncode, 0, (out,err))

    def test_interrupted_install_can_recover_from_the_persistent_receipt(self):
        original = shortcuts.replace_file
        def interrupted(path, data, mode):
            if path == self.hidden:
                raise RuntimeError("simulated power loss")
            return original(path, data, mode)
        with mock.patch.object(shortcuts, "validate_runtime"), mock.patch.object(shortcuts,"replace_file",interrupted):
            with self.assertRaises(RuntimeError):
                shortcuts.migrate(self.home, self.desktop, self.menu.parent, "apply")
        self.assertTrue(self.state.exists())
        self.cli("--apply")
        self.cli("--check")
        self.cli("--rollback")
        self.assertEqual(self.hidden.read_bytes(), OLD_ICON)


if __name__ == "__main__":
    unittest.main()
