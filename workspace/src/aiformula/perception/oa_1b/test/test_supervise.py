"""Linux process-tree tests. These launch only local dummy processes."""
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest


SUPERVISOR = Path(__file__).resolve().parents[1] / "oa_1b/supervise.py"
STAGES = ("allnodes", "yolop", "oa1b", "lya")
WORKER = """
import os, signal, subprocess, sys, time
from pathlib import Path

role = sys.argv[1]
stage = sys.argv[2]
directory = Path.cwd()
if role == 'leaf':
    if os.environ.get('STUBBORN') == 'yes':
        for s in (signal.SIGINT, signal.SIGTERM):
            signal.signal(s, signal.SIG_IGN)
    (directory/(stage+'.leaf')).write_text(str(os.getpid()))
    time.sleep(60)
elif role == 'orphan':
    subprocess.Popen([sys.executable, __file__, 'leaf', stage], start_new_session=True)
else:
    subprocess.run([sys.executable, __file__, 'orphan', stage], check=True)
    # This process is the launch parent. Its grandchild is now a detached orphan.
    (directory/(stage+'.parent')).write_text(str(os.getpid()))
    (directory/(stage+'.env')).write_text(os.environ['TEST_STAGE'])
    while not (directory/(stage+'.exit')).exists():
        time.sleep(.02)
    sys.exit(int((directory/(stage+'.exit')).read_text()))
"""


def wait_for(condition, seconds=7):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(.025)
    raise AssertionError("Timed out waiting for process evidence")


class LifecycleTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.selector = self.root / "aiformula_runtime/2026-11-19-yolop-selection"
        self.selector.mkdir(parents=True)
        self.worker = self.root / "fake_ros.py"
        self.worker.write_text(WORKER)
        self.env = dict(os.environ, HOME=str(self.root), XDG_RUNTIME_DIR=str(self.root), ROS_DOMAIN_ID="221")
        self.processes = []
        self.neighbor = subprocess.Popen([sys.executable, "-c", "import time;time.sleep(90)"])

    def exec_stage(self, args, **extra):
        return subprocess.Popen([sys.executable, str(SUPERVISOR)] + args,
                                env=dict(self.env, **extra), cwd=self.root,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                start_new_session=True)

    def stage(self, name, stubborn=False):
        proc = self.exec_stage([name, "--", sys.executable, str(self.worker), "root", name],
                               TEST_STAGE=name, STUBBORN="yes" if stubborn else "no")
        self.processes.append(proc)
        wait_for(lambda:(self.root/(name+".leaf")).exists() and (self.root/(name+".parent")).exists())
        self.assertEqual((self.root/(name+".env")).read_text(), name)
        return proc

    def start_all(self):
        return [self.stage(n) for n in STAGES]

    def assert_clean(self):
        for proc in self.processes:
            proc.wait(timeout=10)
        for path in list(self.root.glob("*.parent")) + list(self.root.glob("*.leaf")):
            self.assertFalse(Path("/proc",path.read_text()).exists(), f"Process {path} survived")
        self.assertIsNone(self.neighbor.poll())

    def tearDown(self):
        for proc in self.processes:
            if proc.poll() is None:
                proc.terminate()
        try:
            for proc in self.processes:
                proc.wait(timeout=10)
        finally:
            self.neighbor.terminate()
            self.neighbor.wait()
            self.temp.cleanup()

    def test_ctrl_c_secondary_with_detached_orphan_and_reuse(self):
        stages = self.start_all()
        stages[2].send_signal(signal.SIGINT)
        self.assert_clean()
        # Old launchers can immediately lock their original, unchanged filenames.
        for file in [self.root/f"aiformula-1120-221-{s}.lock" for s in ("allnodes","mapassist","lya")] + [
                self.selector/"selector-221.lock"]:
            with file.open("r+") as handle:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for path in self.root.glob("*.parent"):
            path.unlink()
        for path in self.root.glob("*.leaf"):
            path.unlink()
        self.stage("allnodes")
        self.assertEqual(self.exec_stage(["stop"]).wait(timeout=8), 0)
        self.assert_clean()

    def test_hup_on_first_terminal(self):
        stages = self.start_all()
        stages[0].send_signal(signal.SIGHUP)
        self.assert_clean()

    def test_term_on_last_terminal(self):
        stages = self.start_all()
        stages[-1].send_signal(signal.SIGTERM)
        self.assert_clean()

    def test_loss_of_secondary_terminal_socket(self):
        stages = self.start_all()
        stages[1].kill()  # client died without sending a shutdown request
        self.assert_clean()

    def test_forcible_loss_of_first_terminal(self):
        stages = self.start_all()
        stages[0].kill()
        self.assert_clean()

    def test_missing_stage_command_stops_session(self):
        self.stage("allnodes")
        failed = self.exec_stage(["yolop", "--", "/nonexistent/test-oa1b"])
        self.assertNotEqual(failed.wait(timeout=4), 0)
        self.assert_clean()

    def test_failed_launch_parent_reaps_stubborn_orphan(self):
        self.stage("allnodes")
        self.stage("yolop", stubborn=True)
        (self.root/"yolop.exit").write_text("7")
        self.assert_clean()
        self.assertEqual([p.returncode for p in self.processes], [7,7])

    def test_zero_exit_of_launch_parent_stops_others(self):
        self.start_all()
        (self.root/"lya.exit").write_text("0")
        self.assert_clean()
        self.assertEqual([p.returncode for p in self.processes], [0,0,0,0])

    def test_legacy_locks_and_stage_order(self):
        legacy = self.selector/"selector-221.lock"
        with legacy.open("a+") as old:
            fcntl.flock(old, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.assertNotEqual(self.exec_stage(["allnodes","--","true"]).wait(timeout=2), 0)
        self.stage("allnodes")
        denied = [["allnodes","--","true"],["lya","--","true"],["oa1b","--","true"]]
        for cmd in denied:
            self.assertNotEqual(self.exec_stage(cmd).wait(timeout=3), 0)
        self.stage("yolop")
        self.assertNotEqual(self.exec_stage(["yolop","--","true"]).wait(timeout=3), 0)
        for file in (legacy, self.root/"aiformula-1120-221-mapassist.lock"):
            with file.open("a+") as old:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(old, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.assertEqual(self.exec_stage(["stop"]).wait(timeout=8), 0)
        self.assert_clean()
        self.assertEqual(self.exec_stage(["stop"]).wait(timeout=2), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
