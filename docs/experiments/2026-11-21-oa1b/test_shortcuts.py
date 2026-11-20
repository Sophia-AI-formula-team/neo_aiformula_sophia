"""Exercise the 1121 independent launcher installer in a disposable Linux HOME."""
import json
import os
from pathlib import Path
import shlex
import stat
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(os.environ.get("OA1B_SHORTCUTS", Path(__file__).with_name("shortcuts.py"))).resolve()
STAGES = {"allnodes":"oa1b-allnodes", "yolop":"oa1b-yolop", "oa1b":"oa1b",
          "lya":"oa1b-lya", "stop":"oa1b-stop", "verify":"oa1b-verify"}


def executable(path, data):
    path.write_text(data, encoding="utf8")
    path.chmod(0o755)


def snapshot(paths):
    result = {}
    for path in paths:
        info = path.lstat()
        data = os.readlink(path) if path.is_symlink() else path.read_bytes()
        result[str(path)] = (info.st_ino, info.st_mtime_ns, info.st_mode, data)
    return result


class ShortcutTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.home = Path(self.temp.name) / "实验 用户"
        self.bin = self.home/".local/bin"
        self.desktop = self.home/"1121 桌面"
        self.release = self.home/"aiformula_runtime/2026-11-21-oa1b/oa1b-1.0"
        for path in (self.bin, self.desktop, self.release):
            path.mkdir(parents=True)
        self.ready = self.release/"READY"
        self.ready.write_text(self.release.name+"\n")
        self.call_log = self.home/"captured.jsonl"
        executable(self.release/"entry", "#!"+sys.executable+"\n"
                   "import json,os,sys\n"
                   "with open(os.environ['CALL_LOG'],'a') as f: f.write(json.dumps(sys.argv[1:])+'\\n')\n")
        helper = Path(self.temp.name)/"helpers"
        helper.mkdir()
        executable(helper/"xdg-user-dir", '#!/bin/sh\ntest "$1" = DESKTOP && printf "%s\\n" "$TEST_DESKTOP"\n')
        self.env = dict(os.environ, HOME=str(self.home), TEST_DESKTOP=str(self.desktop),
                        CALL_LOG=str(self.call_log), PATH=str(helper)+":"+os.environ["PATH"])
        for name in ("allnodes","yolop","mapassist","lya","concept"):
            executable(self.bin/name, "#!/bin/sh\nexit 17\n")
        (self.bin/"concept-live").symlink_to(self.bin/"concept")
        for name in ("Allnodes","YOLOP","MapAssist","LYA","Concept"):
            (self.desktop/(name+".desktop")).write_text("[Desktop Entry]\nName="+name+"\n")
        self.legacy = [*self.bin.iterdir(), *self.desktop.iterdir()]
        self.before = snapshot(self.legacy)
        self.owned = [self.bin/name for name in STAGES.values()]
        self.desktops = [(stage,self.desktop/f"1121-oa1b-{n}-{stage}.desktop")
                         for n,stage in enumerate(("allnodes","yolop","oa1b","lya"),1)]
        self.owned += [path for _,path in self.desktops]

    def tearDown(self):
        self.assertEqual(snapshot(self.legacy), self.before)

    def cli(self, action, *args, ok=True):
        proc = subprocess.run([sys.executable,str(SCRIPT),action,str(self.release),*map(str,args)],
                              env=self.env,capture_output=True,text=True)
        if ok:
            self.assertEqual(proc.returncode,0,proc.stderr)
        else:
            self.assertNotEqual(proc.returncode,0)
        return proc

    def test_preview_check_install_invocation_and_remove(self):
        preview = self.cli("preview")
        self.assertEqual({line.split(" ",1)[1] for line in preview.stdout.splitlines()},
                         {str(p) for p in self.owned})
        self.assertFalse(any(os.path.lexists(p) for p in self.owned))
        self.cli("check", ok=False)
        self.cli("install")
        self.cli("check", "--desktop", self.desktop)
        installed = snapshot(self.owned)
        self.cli("install")
        self.assertEqual(snapshot(self.owned), installed)
        self.assertTrue(all(stat.S_IMODE(p.stat().st_mode)==0o755 for p in self.owned))

        probe_args = ["inner", "带 空格", "$(false); *"]
        expected = []
        for stage,name in STAGES.items():
            subprocess.run([str(self.bin/name),*probe_args],env=self.env,check=True)
            expected.append([stage,*probe_args])
        for stage,path in self.desktops:
            lines=path.read_text().splitlines()
            self.assertIn("Terminal=true",lines)
            entries=[shlex.split(v[5:]) for v in lines if v.startswith("Exec=")]
            permitted=[[str(self.bin/STAGES[stage])]]
            if stage=="oa1b":
                self.assertIn("[Desktop Action Inner]",lines)
                permitted.append([str(self.bin/"oa1b"),"inner"])
            self.assertEqual(entries,permitted)
            for command in entries:
                subprocess.run(command,env=self.env,check=True)
                expected.append([stage,*command[1:]])
        self.assertEqual([json.loads(s) for s in self.call_log.read_text().splitlines()],expected)
        self.assertEqual(subprocess.run([self.bin/"allnodes"], env=self.env).returncode,17)
        self.cli("remove")
        self.cli("remove")
        self.cli("check",ok=False)
        self.assertFalse(any(os.path.lexists(p) for p in self.owned))

    def test_partially_prepared_build_is_rejected(self):
        self.ready.write_text("another-version")
        for action in ("preview","install","check"):
            self.cli(action,ok=False)
        self.assertFalse(any(os.path.lexists(p) for p in self.owned))

    def test_foreign_regular_file_blocks_even_partial_removal(self):
        self.cli("install")
        path=self.desktop/"1121-oa1b-4-lya.desktop"
        path.write_text("foreign shortcut\n")
        before=snapshot(self.owned)
        for action in ("preview","install","check","remove"):
            self.cli(action,ok=False)
            self.assertEqual(snapshot(self.owned),before)

    def test_existing_and_dangling_symlinks_are_never_followed(self):
        for dst in (self.bin/"mapassist", self.home/"nonexistent"):
            with self.subTest(dst=dst):
                path=self.bin/"oa1b"
                path.symlink_to(dst)
                before=snapshot([path])
                for action in ("install","remove"):
                    self.cli(action,ok=False)
                    self.assertEqual(snapshot([path]),before)
                    self.assertFalse(any(os.path.lexists(v) for v in self.owned if v!=path))
                path.unlink()

    def test_external_desktop_symlink_is_rejected(self):
        outside=Path(self.temp.name)/"other-desktop"
        outside.mkdir()
        shortcut=self.home/"external"
        shortcut.symlink_to(outside,target_is_directory=True)
        self.cli("install","--desktop",shortcut,ok=False)
        self.assertEqual(list(outside.iterdir()),[])

    def test_symlinked_bin_directory_is_never_written(self):
        outside=Path(self.temp.name)/"foreign commands"
        self.bin.rename(outside)
        self.bin.symlink_to(outside,target_is_directory=True)
        before=snapshot([self.bin,*outside.iterdir()])
        for action in ("preview","install","check","remove"):
            self.cli(action,ok=False)
            self.assertEqual(snapshot([self.bin,*outside.iterdir()]),before)
            self.assertFalse(any(path.exists() for _,path in self.desktops))


if __name__=="__main__":
    unittest.main(verbosity=2)
