import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from tempfile import TemporaryDirectory
import unittest


RUNNER = Path(__file__).resolve().parents[1] / "shellscript/run_obstacle_avoidance.py"

SUPERVISOR = r"""
import importlib.util
from pathlib import Path
import sys

spec = importlib.util.spec_from_file_location("oa_runner", sys.argv[1])
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
work = Path(sys.argv[2])
runner.reset_lane_source = lambda: (work / "reset").touch()
child = "import os, pathlib, signal, sys; signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); pathlib.Path(sys.argv[1]).write_text(str(os.getpid())); signal.pause()"
sys.exit(runner.run_child([sys.executable, "-c", child, str(work / "oa_pid")]))
"""

LAUNCHER = r"""
from pathlib import Path
import signal
import subprocess
import sys

child = subprocess.Popen(sys.argv[2:])
(Path(sys.argv[1]) / "runner_pid").write_text(str(child.pid))
signal.pause()
"""


def wait_for(condition):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(0.025)
    assert condition()


def read_pid(path):
    wait_for(lambda: path.is_file() and path.stat().st_size > 0)
    return int(path.read_text())


def alive(pid):
    try:
        status = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
        return status not in {"Z", "X"}
    except FileNotFoundError:
        return False


def stop_if_running(pid):
    if pid is not None and alive(pid):
        os.kill(pid, signal.SIGKILL)


@unittest.skipUnless(sys.platform.startswith("linux"), "Linux parent-death signals")
class ObstacleAvoidanceShutdownTests(unittest.TestCase):
    def test_terminal_hangup_stops_oa_and_restores_lane_source(self):
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        tmp_path = Path(tmp.name)
        supervisor = subprocess.Popen([sys.executable, "-c", SUPERVISOR, str(RUNNER), str(tmp_path)])
        oa = None
        try:
            oa = read_pid(tmp_path / "oa_pid")
            supervisor.send_signal(signal.SIGHUP)
            supervisor.wait(timeout=5)
            assert not alive(oa)
            assert (tmp_path / "reset").is_file()
        finally:
            stop_if_running(supervisor.pid)
            stop_if_running(oa)
            supervisor.wait(timeout=5)

    def test_launcher_exit_stops_runner_and_oa(self):
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        tmp_path = Path(tmp.name)
        supervisor_command = [sys.executable, "-c", SUPERVISOR, str(RUNNER), str(tmp_path)]
        launcher = subprocess.Popen([sys.executable, "-c", LAUNCHER, str(tmp_path), *supervisor_command])
        runner = oa = None
        try:
            runner = read_pid(tmp_path / "runner_pid")
            oa = read_pid(tmp_path / "oa_pid")
            launcher.kill()
            launcher.wait(timeout=5)
            wait_for(lambda: not alive(runner) and not alive(oa))
            assert (tmp_path / "reset").is_file()
        finally:
            stop_if_running(launcher.pid)
            stop_if_running(runner)
            stop_if_running(oa)
            launcher.wait(timeout=5)

    def test_killed_runner_does_not_leave_oa(self):
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        tmp_path = Path(tmp.name)
        supervisor = subprocess.Popen([sys.executable, "-c", SUPERVISOR, str(RUNNER), str(tmp_path)])
        oa = None
        try:
            oa = read_pid(tmp_path / "oa_pid")
            supervisor.kill()
            supervisor.wait(timeout=5)
            wait_for(lambda: not alive(oa))
        finally:
            stop_if_running(supervisor.pid)
            stop_if_running(oa)
            supervisor.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
