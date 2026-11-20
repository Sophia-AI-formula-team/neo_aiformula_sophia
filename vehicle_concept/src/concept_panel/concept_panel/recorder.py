"""Nonblocking ownership of one Foxy ros2 bag process group per panel."""
from collections import Counter
from contextlib import closing
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import tempfile
import threading
import time
from typing import Optional
from uuid import uuid4

import yaml


DECISION_TOPIC = '/concept/decision'
CAMERA_TOPIC = '/concept/perception/overlay/compressed'
PANEL_TOPIC = '/concept/panel/screenshot/compressed'
TOPICS = (
    DECISION_TOPIC, CAMERA_TOPIC, PANEL_TOPIC, '/concept/vehicle_state', '/concept/auto_cmd',
    '/concept/gate/status', '/concept/gate/estop', '/concept/internal/motor_cmd',
    '/concept/internal/motor_can', '/aiformula_sensing/vehicle_info',
    '/aiformula_sensing/vectornav/imu', '/aiformula_control/joy_node/joy_l2_throttle',
)
_ACTIVE = ('starting', 'recording', 'stopping')
_POLL = .05
_STARTUP_TIMEOUT = 15.
_FINALIZE_TIMEOUT = 15.
_TERM_TIMEOUT = 3.


@dataclass(frozen=True)
class RecorderStatus:
    state: str = 'idle'
    path: str = ''
    error: str = ''
    messages: Optional[int] = None


def _verified_messages(path):
    data = yaml.safe_load((path / 'metadata.yaml').read_text(encoding='utf8'))
    info = data['rosbag2_bagfile_information']
    if info.get('storage_identifier') != 'sqlite3':
        raise ValueError('录包元数据没有声明 SQLite 存储')
    expected = info['message_count']
    if type(expected) is not int or expected < 1:
        raise ValueError('没有录到消息，请检查所选 ROS 域和话题发布者')
    expected_topics = {}
    for item in info['topics_with_message_count']:
        topic = item['topic_metadata']['name']
        count = item['message_count']
        if topic not in TOPICS or topic in expected_topics or type(count) is not int or count < 0:
            raise ValueError('录包元数据中的话题或消息数无效')
        expected_topics[topic] = count
    actual = Counter()
    seen = set()
    for name in info['relative_file_paths']:
        db = (path / name).resolve(strict=True)
        if db.parent != path.resolve() or db in seen or not db.is_file():
            raise ValueError('录包元数据中的 SQLite 文件路径无效')
        seen.add(db)
        with closing(sqlite3.connect(db.as_uri() + '?mode=ro', uri=True)) as connection:
            for topic, count in connection.execute(
                    'SELECT topics.name, COUNT(*) FROM messages '
                    'LEFT JOIN topics ON topics.id=messages.topic_id GROUP BY topics.name'):
                actual[topic] += count
    if sum(actual.values()) != expected or dict(actual) != {t: n for t, n in expected_topics.items() if n}:
        raise ValueError('SQLite 各话题实际消息数与 metadata.yaml 不一致')
    return expected


def _signal_group(process, sig):
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def _group_alive(process):
    process.poll()  # Reap our child before querying its group.
    try:
        os.killpg(process.pid, 0)
        return True
    except ProcessLookupError:
        return False


def _wait_group(process, seconds):
    end = time.monotonic() + seconds
    while _group_alive(process):
        if time.monotonic() >= end:
            return False
        time.sleep(_POLL)
    return True


class BagRecorder:
    """start, stop, close, and status never wait for disk, DDS, or the child.

    A non-daemon supervisor finishes a requested close even after Qt stops
    processing events. The object can be reused after saved or error.
    """
    def __init__(self, root=None):
        location = root if root is not None else os.environ.get('CONCEPT_BAG_DIR') or '~/rosbags/concept'
        self.root = Path(location).expanduser().absolute()
        self._lock = threading.Lock()
        self._current = RecorderStatus()
        self._stop = threading.Event()
        self._closed = False

    def status(self):
        with self._lock:
            return self._current

    def start(self):
        with self._lock:
            if self._closed or self._current.state in _ACTIVE:
                return self._current
            name = datetime.now().strftime('concept_%Y%m%d_%H%M%S_') + uuid4().hex
            path = self.root / name
            self._stop = threading.Event()
            self._current = RecorderStatus('starting', str(path))
            worker = threading.Thread(target=self._run, args=(path, self._stop),
                                      name='concept-bag-recorder', daemon=False)
            try:
                worker.start()
            except RuntimeError as exc:
                self._current = RecorderStatus('error', str(path), str(exc))
            return self._current

    def _request_stop(self):
        if self._current.state in _ACTIVE:
            self._current = RecorderStatus('stopping', self._current.path)
            self._stop.set()
        return self._current

    def stop(self):
        with self._lock:
            return self._request_stop()

    def close(self):
        with self._lock:
            self._closed = True
            return self._request_stop()

    def _run(self, path, stop):
        try:
            if os.name != 'posix':
                raise OSError('Foxy 面板录包需要支持 POSIX 进程组的系统')
            self.root.mkdir(parents=True, exist_ok=True)
            if path.exists():
                raise FileExistsError(str(path))
            # Foxy's DDS QoS depth and rosbag's SQLite write cache are independent.
            profile = {topic: dict(history='keep_last', depth=1, reliability='best_effort',
                                   durability='volatile') for topic in TOPICS}
            with tempfile.TemporaryDirectory(prefix='concept-bag-qos-') as scratch, tempfile.TemporaryFile() as log:
                qos = Path(scratch) / 'qos.yaml'
                qos.write_text(yaml.safe_dump(profile), encoding='utf8')
                command = ['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', str(path),
                           '--max-cache-size', '0', '--qos-profile-overrides-path', str(qos), *TOPICS]
                process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    self._supervise(process, path, stop)
                except Exception as exc:
                    size = log.seek(0, os.SEEK_END)
                    log.seek(max(0, size-800))
                    tail = log.read().decode('utf8', errors='replace').strip()
                    raise RuntimeError(str(exc) + ('\n' + tail if tail else '')) from exc
                finally:
                    if _group_alive(process):
                        _signal_group(process, signal.SIGTERM)
                        if not _wait_group(process, _TERM_TIMEOUT):
                            _signal_group(process, signal.SIGKILL)
                            _wait_group(process, _TERM_TIMEOUT)
            messages = _verified_messages(path)
            outcome = RecorderStatus('saved', str(path), messages=messages)
        except Exception as exc:
            outcome = RecorderStatus('error', str(path), str(exc))
        with self._lock:
            self._current = outcome

    def _supervise(self, process, path, stop):
        deadline = time.monotonic() + _STARTUP_TIMEOUT
        opened = False
        while process.poll() is None:
            if not opened and any(path.glob('*.db3')):
                opened = True
                with self._lock:
                    if not stop.is_set():
                        self._current = RecorderStatus('recording', str(path))
            if stop.is_set() and opened:
                _signal_group(process, signal.SIGINT)
                if not _wait_group(process, _FINALIZE_TIMEOUT):
                    raise RuntimeError('录包进程收到 SIGINT 后未在时限内完成收尾')
                if process.returncode not in (0, -signal.SIGINT, int(signal.SIGINT), 130):
                    raise RuntimeError('录包进程收尾失败，退出码 %s' % process.returncode)
                return
            if not opened and time.monotonic() >= deadline:
                _signal_group(process, signal.SIGINT)
                _wait_group(process, _TERM_TIMEOUT)
                raise RuntimeError('录包进程未能及时打开 SQLite 文件')
            time.sleep(_POLL)
        raise RuntimeError('录包进程意外退出，退出码 %s' % process.returncode)
