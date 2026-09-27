"""Supervisor method tests: no ROS process or vehicle is started here."""
import ast
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace as Obj

import pytest


@pytest.fixture
def supervisor():
    source = Path(__file__).parents[1] / 'lane_mapping_lya_reference' / 'supervisor.py'
    tree = ast.parse(source.read_text(encoding='utf-8'), feature_version=(3, 8))
    tree.body = [item for item in tree.body if isinstance(item, ast.ClassDef)]
    namespace = {'Node': object, 'json': json, 'time': Obj(monotonic=lambda: 10.0),
                 'os': os, 'signal': signal, 'subprocess': subprocess}
    exec(compile(tree, str(source), 'exec'), namespace)
    cls = namespace['TeacherSupervisor']
    node = cls.__new__(cls)
    node.events = []
    node.stops = []
    node._control_seen = None
    node._event = lambda event, **fields: node.events.append(event)
    node._request_stop = lambda reason: node.stops.append(reason)
    node.get_clock = lambda: Obj(now=lambda: Obj(nanoseconds=10_000_000_000))
    return node


@pytest.mark.parametrize('payload', [None, [], 12, 'x', {}, {'stamp_ns': 'nan'}])
def test_malformed_state_never_renews_heartbeat(supervisor, payload):
    supervisor._control(Obj(data=json.dumps(payload)))
    assert supervisor._control_seen is None
    assert not supervisor.stops


@pytest.mark.parametrize('stamp', [0, 8_000_000_000, 11_000_000_000])
def test_stale_or_future_state_does_not_stop_owned_teacher(supervisor, stamp):
    supervisor._control(Obj(data=json.dumps(dict(stamp_ns=stamp,
        safety_mode='fixed_only', teacher_enabled=False))))
    assert supervisor._control_seen is None
    assert not supervisor.stops


def test_reference_state_does_not_shutdown_teacher(supervisor):
    supervisor._control(Obj(data=json.dumps(dict(stamp_ns=10_000_000_000,
        safety_mode='lya_reference', teacher_enabled=False))))
    assert supervisor._control_seen == 10.0
    assert not supervisor.stops


def test_fixed_handoff_stops_only_owned_teacher(supervisor):
    supervisor._control(Obj(data=json.dumps(dict(stamp_ns=10_000_000_000,
        safety_mode='fixed_only', teacher_enabled=False))))
    assert supervisor.stops == ['fixed_route_handoff']


@pytest.mark.skipif(os.name != 'posix', reason='Target Linux process-group test')
def test_owned_process_shutdown_leaves_unrelated_process_alive(supervisor):
    # Harmless sleep processes only. No ROS teacher or motor is launched.
    owned = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                             start_new_session=True)
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                                 start_new_session=True)
    supervisor._process = owned
    supervisor._expected_stop = False
    try:
        assert supervisor._group_alive()
        type(supervisor)._request_stop(supervisor, 'unit_test_handoff')
        owned.wait(timeout=3.0)
        assert not supervisor._group_alive()
        assert unrelated.poll() is None
        # An already exited process is also safe to signal, without a race error.
        supervisor._signal_group(force=True)
    finally:
        if owned.poll() is None:
            os.killpg(owned.pid, signal.SIGKILL)
        owned.wait(timeout=3.0)
        if unrelated.poll() is None:
            os.killpg(unrelated.pid, signal.SIGKILL)
        unrelated.wait(timeout=3.0)
