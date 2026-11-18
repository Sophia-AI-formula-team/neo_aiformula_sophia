#!/usr/bin/env python3
"""Exercise the installed Foxy panel on a private DDS domain under Xvfb.

Source Foxy, the neo underlay and the built Concept overlay, then run
  python3 vehicle_concept/checks/panel_bag_smoke.py --output /tmp/qa_panel_bag --domain 89
A unique child of --output retains both bags, CLI output and evidence.json.
Exit codes: 0 passed, 1 failed, 77 missing runtime dependency.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import re
import shutil
import signal
import sqlite3
import subprocess
import sys
import tempfile
import time
import traceback

# Independent expectations, also used to advertise every probe through DDS.
DECISION = '/concept/decision'
STATE = '/concept/vehicle_state'
AUTO = '/concept/auto_cmd'
GATE = '/concept/gate/status'
CAMERA = '/concept/view/camera'
JPEG = '/concept/perception/overlay/compressed'
FRAME = '/concept/perception/frame'
LIVE_CAN = '/aiformula_control/motor_controller/reference_signal'
REPLAY_CAN = '/concept/replay/reference_signal'
CAN = 'can_msgs/msg/Frame'
STRING = 'std_msgs/msg/String'
IMAGE = 'sensor_msgs/msg/Image'
EXPECTED = {
    JPEG: 'sensor_msgs/msg/CompressedImage',
    DECISION: STRING, STATE: STRING, AUTO: 'geometry_msgs/msg/TwistStamped', GATE: STRING,
    '/concept/gate/estop': 'std_msgs/msg/Bool', '/concept/internal/motor_cmd': 'geometry_msgs/msg/Twist',
    '/concept/internal/motor_can': CAN, '/aiformula_sensing/vehicle_info': CAN,
    '/aiformula_sensing/vectornav/imu': 'sensor_msgs/msg/Imu',
    '/aiformula_control/joy_node/joy_l2_throttle': 'sensor_msgs/msg/Joy',
}
EXCLUDED = {
    FRAME: STRING, '/concept/perception/overlay': IMAGE, CAMERA: IMAGE,
    '/aiformula_sensing/zed_node/left_image/undistorted': IMAGE,
    '/concept/view/local': 'visualization_msgs/msg/MarkerArray',
    '/concept/view/map': 'visualization_msgs/msg/MarkerArray',
    LIVE_CAN: CAN, REPLAY_CAN: CAN, '/qa/unlisted': STRING,
}
SENSOR_INPUTS = {'/aiformula_sensing/vehicle_info', '/aiformula_sensing/vectornav/imu',
                 '/aiformula_control/joy_node/joy_l2_throttle'}
PIXEL_SECRET = 'QA_image_payload_must_never_enter_bags'
PNG = 'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+a4WYAAAAASUVORK5CYII='


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def recorders(root):
    """Identify our recorder by its real argv and unique bag output prefix."""
    found = []
    for item in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            args = [s.decode(errors='replace') for s in item.read_bytes().split(b'\0') if s]
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if (any(arg.startswith(str(root) + '/') for arg in args)
                and ('record' in args or any(Path(arg).name == 'record' for arg in args))):
            found.append(int(item.parent.name))
    return sorted(found)


class Sources:
    def __init__(self):
        import cv2
        import numpy as np
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message

        self.node = Node('panel_bag_qa_sources')
        self.pubs = {}
        self.messages = {}
        for topic, msg_type in {**EXPECTED, **EXCLUDED}.items():
            cls = get_message(msg_type)
            qos = qos_profile_sensor_data if msg_type == IMAGE or topic == JPEG or topic in SENSOR_INPUTS else 10
            self.pubs[topic] = self.node.create_publisher(cls, topic, qos)
            self.messages[topic] = cls()
            msg = self.messages[topic]
            if msg_type == IMAGE:
                msg.encoding, msg.height, msg.width, msg.step = 'rgb8', 8, 8, 24
                msg.data = (PIXEL_SECRET.encode() + bytes(192))[:192]
            elif topic == JPEG:
                ok, encoded = cv2.imencode('.jpg', np.full((360, 640, 3), (30, 143, 255), dtype=np.uint8))
                require(ok, 'Could not encode fixture JPEG')
                msg.format, msg.data = 'bgr8; jpeg compressed bgr8', encoded.tobytes()
            elif topic == FRAME:
                # A String with actual encoded image bytes; no planner is run in this test.
                msg.data = json.dumps({'schema': 'concept.perception.v1', 'bgr_png': PNG,
                                       'road_png': PNG, 'lane_png': PNG, 'frame_id': PIXEL_SECRET})
            elif topic == '/qa/unlisted':
                msg.data = PIXEL_SECRET
        self.seq = 0

    def assert_exclusions(self):
        counts = {topic: self.pubs[topic].get_subscription_count() for topic in EXCLUDED}
        unexpected = {t: n for t, n in counts.items() if n > (1 if t == CAMERA else 0)}
        require(not unexpected, 'Unexpected DDS subscribers: ' + repr(unexpected))
        return counts

    def publish(self, session):
        self.seq += 1
        stamp = self.node.get_clock().now().to_msg()
        t = stamp.sec + stamp.nanosec * 1e-9
        for topic, msg in self.messages.items():
            if hasattr(msg, 'header'):
                msg.header.stamp = stamp
                msg.header.frame_id = 'base_footprint'
            if topic == DECISION:
                index = 17 if session == 1 else 49
                msg.data = json.dumps(dict(stamp=t, status='shadow', selected_index=index,
                    feasible_count=1, candidates=[{'index': index, 'feasible': True}],
                    reason='qa_panel_bag', qa_session=session, qa_seq=self.seq))
            elif topic == STATE:
                msg.data = json.dumps(dict(t=t, valid=True, leftRpm=0, rightRpm=0, gyro=0,
                                           qa_session=session, qa_seq=self.seq))
            elif topic == GATE:
                msg.data = json.dumps(dict(t=t, mode='disabled', reason='output_disabled'))
            self.pubs[topic].publish(msg)


def drive_panel(run):
    from concept_panel.main import main
    from concept_panel.qt import QtCore, QtWidgets
    if QtCore.__name__.startswith('PySide'):
        from PySide6.QtTest import QTest
    else:
        from PyQt5.QtTest import QTest

    app = QtWidgets.QApplication(sys.argv[:1])
    timer = QtCore.QTimer()
    root = run / 'bags'
    events, sessions = [], []
    src = window = None
    phase, batch, tick_count = 'open', 1, 0
    previous = failure = None
    start = changed = time.monotonic()

    def stage(name):
        nonlocal phase, changed
        phase, changed = name, time.monotonic()

    def tick():
        nonlocal window, src, previous, failure, batch, tick_count
        try:
            now = time.monotonic()
            require(now - changed < 18, 'Timeout in phase ' + phase)
            if window is None:
                window = next((w for w in app.topLevelWidgets() if w.isVisible()
                              and w.findChild(QtWidgets.QPushButton, 'bag_toggle')), None)
                if window is None:
                    return
                src = Sources()
                app.aboutToQuit.connect(src.node.destroy_node)
            button = window.findChild(QtWidgets.QPushButton, 'bag_toggle')
            label = window.findChild(QtWidgets.QLabel, 'bag_status')
            status = label.text()
            path = Path(window.findChild(QtWidgets.QLineEdit, 'bag_path').text())
            if (status, str(path)) != previous:
                events.append(dict(seconds=round(now - start, 3), status=status, path=str(path)))
                previous = (status, str(path))
            require('失败' not in status, label.toolTip())
            if phase == 'open' and now - start > 1.4:
                require(status == '未录制' and path == root, 'Unexpected initial panel state')
                other = set(src.node.get_node_names()) - {'panel_bag_qa_sources', 'concept_panel'}
                require(not other, 'Choose an unused ROS domain: ' + repr(other))
                require(src.pubs[CAMERA].get_subscription_count() == 1, 'Missing panel camera subscriber')
                src.assert_exclusions()
                stage('start')
            if phase == 'start':
                require(button.isEnabled() and button.text() == '开始录包', 'Start button unavailable')
                QTest.mouseClick(button, QtCore.Qt.LeftButton)
                stage('record')
            elif phase == 'record':
                counts = src.assert_exclusions()
                if status != '录制中' or not all(src.pubs[t].get_subscription_count() for t in EXPECTED):
                    return
                src.publish(batch)
                tick_count += 1
                if tick_count < 15:
                    return
                pids = recorders(root)
                require(pids and path.parent == root, 'Recorder PID or visible bag path missing')
                sessions.append(dict(batch=batch, path=str(path), active_pids=pids,
                                     excluded_subscribers=counts, publish_ticks=tick_count))
                if batch == 1:
                    require(button.isEnabled() and button.text() == '停止录包', 'Stop button unavailable')
                    QTest.mouseClick(button, QtCore.Qt.LeftButton)
                    stage('saved')
                else:
                    stage('close')
                    window.close()
            elif phase == 'saved' and status.startswith('已保存'):
                match = re.fullmatch(r'已保存\s*·\s*(\d+)\s*条消息', status)
                require(match and path == Path(sessions[0]['path']) and (path / 'metadata.yaml').is_file(),
                        'Saved status missing verified path or count: ' + status)
                require(not recorders(root), 'Recorder still running after panel says saved')
                sessions[0]['panel_saved_count'] = int(match.group(1))
                batch, tick_count = 2, 0
                stage('start')
        except Exception:
            failure = traceback.format_exc()
            timer.stop()
            app.quit()

    timer.timeout.connect(tick)
    timer.start(100)
    try:
        main(args=[])
    finally:
        timer.stop()
        write_json(run / 'panel.json', dict(events=events, sessions=sessions, error=failure))
    require(not failure and phase == 'close' and len(sessions) == 2, failure or 'Panel exited before close test')


def verify_bag(session, root):
    import cv2
    import numpy as np
    import yaml
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    bag = Path(session['path'])
    require(bag.parent == root and bag.is_dir(), 'Missing visible bag directory')
    info = subprocess.run(['ros2', 'bag', 'info', str(bag)], capture_output=True, text=True, timeout=15)
    (bag.parent.parent / ('info-%s.txt' % session['batch'])).write_text(info.stdout + info.stderr, encoding='utf-8')
    require(info.returncode == 0 and all(topic in info.stdout for topic in EXPECTED), 'ros2 bag info failed')
    meta = yaml.safe_load((bag / 'metadata.yaml').read_text(encoding='utf-8'))['rosbag2_bagfile_information']
    require(meta['storage_identifier'] == 'sqlite3', 'Unexpected storage')
    meta_counts = {row['topic_metadata']['name']: row['message_count'] for row in meta['topics_with_message_count']}
    require(set(meta_counts) == set(EXPECTED), 'Unexpected/missing topics in metadata: ' + repr(meta_counts))
    counts, sequences = Counter(), []
    files = [bag / p for p in meta['relative_file_paths']]
    require(set(files) == set(bag.glob('*.db3')) and files, 'Missing/unlisted database')
    for item in files:
        require(item.parent == bag, 'Invalid bag file path')
        with sqlite3.connect(item.as_uri() + '?mode=ro', uri=True) as db:
            require(db.execute('PRAGMA quick_check').fetchone() == ('ok',), 'Damaged SQLite file')
            for name, typename in db.execute('SELECT name, type FROM topics'):
                require(EXPECTED.get(name) == typename, 'Forbidden topic or wrong type: ' + name)
            for name, data in db.execute('SELECT t.name, m.data FROM messages m JOIN topics t ON m.topic_id=t.id ORDER BY m.id'):
                require(PIXEL_SECRET.encode() not in data and PNG.encode() not in data, 'Image payload in ' + name)
                counts[name] += 1
                if name == JPEG:
                    msg = deserialize_message(data, CompressedImage)
                    frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                    require(frame is not None and frame.shape == (360, 640, 3), 'Invalid recorded JPEG')
                    require(msg.format == 'bgr8; jpeg compressed bgr8' and msg.header.stamp.sec > 0,
                            'Missing JPEG format or camera timestamp')
                if name in (DECISION, STATE):
                    fields = json.loads(deserialize_message(data, String).data)
                    require(fields['qa_session'] == session['batch'], 'Mixed recording sessions')
                    if name == DECISION:
                        idx = 17 if session['batch'] == 1 else 49
                        require(fields['selected_index'] == idx and fields['candidates'] == [{'index': idx, 'feasible': True}],
                                'Original decision fields changed')
                        sequences.append(fields['qa_seq'])
    total = sum(counts.values())
    require(dict(counts) == meta_counts and all(counts[t] > 0 for t in EXPECTED), 'Incomplete topic counts')
    require(total == meta['message_count'] == session.get('panel_saved_count', total), 'Message totals disagree')
    require(len(set(sequences)) >= 2, 'Missing distinct selected decisions')
    return dict(path=str(bag), messages=total, topics=dict(counts), decision_sequences=sequences,
                selected_index=17 if session['batch'] == 1 else 49, db_bytes=sum(p.stat().st_size for p in files))


def supervise(run, domain):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1', QT_QPA_PLATFORM='xcb',
               ROS_HOME=str(run / 'ros-home'), ROS_LOG_DIR=str(run / 'ros-log'), CONCEPT_BAG_DIR=str(run / 'bags'))
    args = ['xvfb-run', '-a', '-s', '-screen 0 1440x900x24', sys.executable, str(Path(__file__).resolve()),
            '--worker', str(run)]
    report = dict(domain=domain, passed=False)
    with (run / 'panel.log').open('w') as log:
        proc = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            require(proc.wait(timeout=90) == 0, 'Panel worker failed; see panel.log and panel.json')
            panel = json.loads((run / 'panel.json').read_text(encoding='utf-8'))
            report['panel'] = panel
            root = run / 'bags'
            paths = [Path(s['path']) for s in panel['sessions']]
            require(len(set(paths)) == 2 and set(paths) == {p.parent for p in root.rglob('metadata.yaml')},
                    'Expected two separate finalized bags')
            report['leftover_recorders'] = recorders(root)
            require(not report['leftover_recorders'], 'Recorder still running after closing Qt')
            report['bags'] = [verify_bag(s, root) for s in panel['sessions']]
            report['passed'] = True
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            for pid in recorders(run / 'bags'):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            write_json(run / 'evidence.json', report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(tempfile.gettempdir()) / 'qa-panel-bag')
    parser.add_argument('--domain', type=int, default=89, help='unused local ROS_DOMAIN_ID, 1 to 101')
    parser.add_argument('--worker', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        drive_panel(args.worker)
        return 0
    if not 1 <= args.domain <= 101:
        parser.error('choose a spare domain from 1 to 101')
    missing = [c for c in ('ros2', 'xvfb-run', 'Xvfb', 'xauth') if not shutil.which(c)]
    if not missing:
        try:
            import yaml
            from concept_panel.qt import QtCore
            if QtCore.__name__.startswith('PySide'):
                from PySide6.QtTest import QTest
            else:
                from PyQt5.QtTest import QTest
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
            for typename in set(EXPECTED.values()) | set(EXCLUDED.values()):
                get_message(typename)
            if subprocess.run(['ros2', 'bag', 'record', '--help'], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, timeout=10).returncode:
                missing.append('ros2 bag record')
        except (ImportError, subprocess.TimeoutExpired) as exc:
            missing.append(str(exc))
    if missing:
        print('PENDING: source Foxy and the built overlay; runtime dependency missing: ' + ', '.join(missing))
        return 77
    args.output.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix='panel-', dir=str(args.output))).resolve()
    print('Evidence: ' + str(run), flush=True)
    try:
        supervise(run, args.domain)
    except Exception:
        traceback.print_exc()
        return 1
    print('PASS: two separate SQLite bags, eleven selected topics including decodable JPEG, no remaining recorder')
    return 0


if __name__ == '__main__':
    sys.exit(main())
