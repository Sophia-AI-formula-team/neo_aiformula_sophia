import os
from pathlib import Path
import time

import pytest

from concept_panel import recorder


EXCLUDED = {
    '/concept/perception/frame', '/concept/view/camera',
    '/aiformula_control/motor_controller/reference_signal', '/concept/replay/reference_signal',
}


def test_registration_and_topic_selection(tmp_path, monkeypatch):
    monkeypatch.setenv('CONCEPT_BAG_DIR', str(tmp_path))
    bag = recorder.BagRecorder()
    assert bag.root == tmp_path
    assert bag.status() == recorder.RecorderStatus()
    assert bag.stop() == bag.close() == bag.start() == recorder.RecorderStatus()
    assert recorder.DECISION_TOPIC == '/concept/decision'
    assert recorder.DECISION_TOPIC in recorder.TOPICS
    assert {'/concept/internal/motor_cmd', '/concept/internal/motor_can'} <= set(recorder.TOPICS)
    assert not EXCLUDED.intersection(recorder.TOPICS)
    assert len(recorder.TOPICS) == len(set(recorder.TOPICS)) == 10


@pytest.fixture
def fake_ros(tmp_path, monkeypatch):
    if os.name != 'posix':
        pytest.skip('Foxy production target uses POSIX process groups')
    binary = tmp_path / 'bin' / 'ros2'
    binary.parent.mkdir()
    binary.write_text(r"""#!/usr/bin/env python3
import json, os, signal, sqlite3, sys, time
from pathlib import Path
mode = os.environ.get('FAKE_BAG_MODE', 'normal')
if mode == 'fail':
    print('intentional CLI failure', flush=True)
    sys.exit(3)
args = sys.argv[1:]
path = Path(args[args.index('-o') + 1])
qos = Path(args[args.index('--qos-profile-overrides-path') + 1]).read_text()
path.mkdir()
(path / 'args.json').write_text(json.dumps(args))
(path / 'qos.yaml').write_text(qos)
stopped = False
def stop(sig, frame):
    global stopped
    stopped = True
signal.signal(signal.SIGINT, stop)
if mode == 'delayed':
    time.sleep(.15)
if mode == 'hang':
    signal.signal(signal.SIGINT, signal.SIG_IGN)
db = sqlite3.connect(str(path / 'one.db3'))
db.execute('CREATE TABLE topics (id INTEGER, name TEXT)')
db.execute('CREATE TABLE messages (id INTEGER, topic_id INTEGER)')
db.execute("INSERT INTO topics VALUES (1, '/concept/decision')")
if mode != 'empty':
    db.execute('INSERT INTO messages VALUES (1,1)')
db.commit()
while not stopped:
    time.sleep(.01)
db.close()
if mode != 'no_metadata':
    count = 2 if mode == 'bad_count' else (0 if mode == 'empty' else 1)
    topic = dict(topic_metadata={'name':'/concept/decision'}, message_count=count)
    info = dict(storage_identifier='sqlite3', relative_file_paths=['one.db3'],
                message_count=count, topics_with_message_count=[topic])
    (path/'metadata.yaml').write_text(json.dumps(dict(rosbag2_bagfile_information=info)))
sys.exit(2)
""")
    binary.chmod(0o755)
    monkeypatch.setenv('PATH', str(binary.parent) + os.pathsep + os.environ['PATH'])
    monkeypatch.setattr(recorder, '_POLL', .01)
    monkeypatch.setattr(recorder, '_FINALIZE_TIMEOUT', 1.5)
    monkeypatch.setattr(recorder, '_TERM_TIMEOUT', .5)
    return recorder.BagRecorder(tmp_path / 'bags with spaces #')


def await_status(bag, state):
    finish = time.monotonic() + 5.
    while time.monotonic() < finish:
        s = bag.status()
        if s.state == state:
            return s
        if s.state == 'error' and state != 'error':
            pytest.fail(s.error)
        time.sleep(.015)
    pytest.fail('Timed out waiting for %s: %r' % (state, bag.status()))


def test_regular_record_stop_repeat_close(fake_ros):
    import json
    import yaml
    bag = fake_ros
    one = bag.start()
    assert one.state == 'starting'
    assert bag.start().path == one.path
    await_status(bag, 'recording')
    path = Path(one.path)
    args = json.loads((path/'args.json').read_text())
    assert args[:2] == ['bag', 'record'] and args[-len(recorder.TOPICS):] == list(recorder.TOPICS)
    assert args[args.index('--max-cache-size')+1] == '0'
    assert args[args.index('-s')+1] == 'sqlite3'
    assert '-a' not in args and '--all' not in args and '--no-discovery' not in args
    qos = yaml.safe_load((path/'qos.yaml').read_text())
    assert set(qos) == set(recorder.TOPICS)
    assert all(profile == dict(history='keep_last', depth=1, reliability='best_effort',
                               durability='volatile') for profile in qos.values())
    assert bag.stop().state == 'stopping'
    assert await_status(bag, 'saved').messages == 1
    two = bag.start()
    assert two.path != one.path
    await_status(bag, 'recording')
    assert bag.close().state == 'stopping'
    result = await_status(bag, 'saved')
    assert result.path == two.path and result.messages == 1
    assert bag.start() == result


def test_stop_during_startup_is_owned_and_returns_immediately(fake_ros, monkeypatch):
    monkeypatch.setenv('FAKE_BAG_MODE', 'delayed')
    bag = fake_ros
    bag.start()
    assert bag.close().state == 'stopping'
    assert bag.status().state == 'stopping'
    assert await_status(bag, 'saved').messages == 1


@pytest.mark.parametrize('mode,expected', [('fail', 'intentional CLI failure'),
        ('no_metadata','metadata.yaml'), ('bad_count','SQLite'), ('empty','没有录到消息'),
        ('hang','SIGINT')])
def test_failure_is_reported_and_reusable(fake_ros, monkeypatch, mode, expected):
    bag = fake_ros
    monkeypatch.setenv('FAKE_BAG_MODE', mode)
    bag.start()
    if mode != 'fail':
        await_status(bag, 'recording')
        bag.stop()
    failed = await_status(bag, 'error')
    assert expected in failed.error
    assert failed.messages is None
    monkeypatch.delenv('FAKE_BAG_MODE')
    next_bag = bag.start()
    assert next_bag.path != failed.path
    await_status(bag, 'recording')
    bag.close()
    assert await_status(bag, 'saved').messages == 1


def test_stop_never_signals_another_recording(fake_ros):
    first = fake_ros
    second = recorder.BagRecorder(first.root)
    try:
        assert first.start().path != second.start().path
        await_status(first, 'recording')
        await_status(second, 'recording')
        first.stop()
        assert await_status(first, 'saved').messages == 1
        assert second.status().state == 'recording'
    finally:
        first.close()
        second.close()
    assert await_status(second, 'saved').messages == 1
