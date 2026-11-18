"""Exercise the real panel exit path with a slow, isolated bag subprocess."""
import os
import signal
import subprocess
import sys
import time

import pytest


def test_launch_signals_during_startup_and_slow_finalize(tmp_path):
    if os.name != 'posix':
        pytest.skip('The Foxy recorder uses POSIX process groups')
    pytest.importorskip('rclpy')
    from concept_panel import recorder

    fake_ros = tmp_path / 'ros2'
    fake_ros.write_text("""#!/usr/bin/env python3
import json, os, signal, sqlite3, sys, time
from pathlib import Path
bag = Path(sys.argv[sys.argv.index('-o')+1])
bag.mkdir(parents=True)
requested = []
signal.signal(signal.SIGINT, lambda *_: requested.append(True))
(bag/'pending-pid').write_text(str(os.getpid()))
(bag/'pending-pid').rename(bag/'starting')
while not (bag/'open').exists(): time.sleep(.03)
db = sqlite3.connect(str(bag/'bag.db3'))
db.executescript("CREATE TABLE topics(id INTEGER, name TEXT);"
                 "INSERT INTO topics VALUES(1,'/concept/decision');"
                 "CREATE TABLE messages(topic_id INTEGER); INSERT INTO messages VALUES(1);")
while not requested: time.sleep(.03)
(bag/'flushing').touch()
time.sleep(6.5)
db.close()
info = dict(storage_identifier='sqlite3', relative_file_paths=['bag.db3'],message_count=1,
            topics_with_message_count=[dict(topic_metadata=dict(name='/concept/decision'),message_count=1)])
(bag/'metadata.yaml').write_text(json.dumps(dict(rosbag2_bagfile_information=info)))
""", encoding='utf8')
    fake_ros.chmod(0o755)
    env = dict(os.environ, PATH=str(tmp_path)+os.pathsep+os.environ['PATH'],
               CONCEPT_BAG_DIR=str(tmp_path/'bags'), ROS_LOCALHOST_ONLY='1',
               ROS_DOMAIN_ID='88', QT_QPA_PLATFORM='offscreen')
    code = """
from concept_panel import main
original = main.Window.show
def show(window):
    original(window)
    main.QtCore.QTimer.singleShot(0, window.bag_toggle.click)
main.Window.show = show
main.main()
"""
    panel = subprocess.Popen([sys.executable, '-c', code], env=env, start_new_session=True)
    bag_pid = None

    def wait_for(pattern, timeout=12):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            files = list(tmp_path.glob(pattern))
            if files:
                return files[0]
            assert panel.poll() is None, 'Panel exited before '+pattern
            time.sleep(.05)
        pytest.fail('Timed out waiting for '+pattern)

    try:
        marker = wait_for('bags/*/starting')
        bag = marker.parent
        bag_pid = int(marker.read_text())
        panel.send_signal(signal.SIGINT)
        time.sleep(.25)
        (bag/'open').touch()
        wait_for('bags/*/flushing')
        panel.send_signal(signal.SIGINT)
        time.sleep(5.1)
        assert panel.poll() is None
        panel.send_signal(signal.SIGTERM)
        assert panel.wait(timeout=15) == 0
        assert recorder._verified_messages(bag) == 1
        with pytest.raises(ProcessLookupError):
            os.killpg(bag_pid, 0)
    finally:
        if panel.poll() is None:
            panel.kill()
            panel.wait()
        if bag_pid is not None:
            try:
                os.killpg(bag_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
