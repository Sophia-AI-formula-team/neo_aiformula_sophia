"""Replay archived Unity camera/feedback through Drive and save the live Qt map."""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Trigger

from concept_drive.node import DriverNode
from concept_panel.main import Controller
from concept_panel.qt import QtGui, QtWidgets
from concept_panel.window import Window
from concept_yolop.frame_codec import encode_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path, help='Unity run directory containing decisions.jsonl and frames/')
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') not in ('85', '86') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('use an isolated software domain 85 or 86 with ROS_LOCALHOST_ONLY=1')
    args.output.mkdir(parents=True, exist_ok=False)
    app = QtWidgets.QApplication([])
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    executor = SingleThreadedExecutor()
    driver, feeder, gui = DriverNode(), Node('lap_map_archive'), Node('lap_map_test_panel')
    for node in (driver, feeder, gui):
        executor.add_node(node)
    window = Window()
    control = Controller(gui, window)
    window.show()
    clock = feeder.create_publisher(Clock, '/clock', 10)
    wheels = feeder.create_publisher(String, '/concept/vehicle_state', 10)
    camera = feeder.create_publisher(String, '/concept/perception/frame', 10)
    decision, received = None, 0
    commands = []

    def receive(message):
        nonlocal decision, received
        decision = json.loads(message.data)
        received += 1

    feeder.create_subscription(String, '/concept/decision', receive, 10)
    feeder.create_subscription(TwistStamped, '/concept/auto_cmd', commands.append, 10)
    map_client = feeder.create_client(Trigger, '/concept/drive/lap_map')

    def until(condition):
        deadline = time.monotonic() + 12.
        while not condition():
            if time.monotonic() > deadline:
                raise TimeoutError('timed out during archive replay, decision count %d' % received)
            executor.spin_once(timeout_sec=.02)
            control.update()
            app.processEvents()

    try:
        until(lambda: all(p.get_subscription_count() for p in (clock, wheels, camera, driver.decisions))
              and map_client.service_is_ready())
        cold = map_client.call_async(Trigger.Request())
        until(cold.done)
        assert not cold.result().success

        for index, line in enumerate((args.archive / 'decisions.jsonl').read_text(encoding='utf8').splitlines()):
            row = json.loads(line)
            assert row['frame'] == index
            capture = 100_000_000_000 + 200_000_000*index
            sec, nano = divmod(capture, 10**9)
            clock.publish(Clock(clock=Time(sec=sec, nanosec=nano)))
            until(lambda: driver.get_clock().now().nanoseconds == capture)
            sample = row['decision']['sensor']
            sample = dict(sample, valid=True, t=capture/1e9, gyro=-sample['gyro'])
            wheels.publish(String(data=json.dumps(sample)))
            until(lambda: driver.feedback.samples and driver.feedback.samples[-1]['t'] == sample['t'])
            prefix = args.archive / 'frames' / f'{index:05d}'
            bgr = cv2.imread(str(prefix)+'_rgb.png')
            with np.load(str(prefix)+'_perception.npz') as perception:
                wire = encode_frame(sec, nano, 'unity_archive', bgr, perception['raw'], perception['lane'],
                                    perception, 'frozen-unity-v626')
            camera.publish(String(data=wire))
            until(lambda: received > index)
            if decision['map'] is None:
                raise ValueError(decision['reason'])
            if index == 20:
                window.map_button.click()
                until(lambda: window.lap_map.save_button.isEnabled())

        # Close the map while the driver remains idle, then request the full
        # authoritative snapshot again in the same session.
        window.lap_map.close()
        window.map_button.click()
        final_stamp = decision['stamp']
        until(lambda: window.lap_map.key is not None and window.lap_map.key[1] == final_stamp)
        data = window.lap_map.canvas.data
        assert data['session'] == decision['session']
        assert len(data['track']) == received == len(driver.driver.mapper.path)
        assert np.allclose(data['pose'], decision['map']['pose'])
        assert len(data['road']) >= decision['map']['cells']

        path = args.output / 'track.png'
        dialog = QtWidgets.QFileDialog
        previous = dialog.getSaveFileName
        dialog.getSaveFileName = lambda *_: (str(path), 'PNG')
        try:
            window.lap_map.save_button.click()
        finally:
            dialog.getSaveFileName = previous
        assert not QtGui.QImage(str(path)).isNull()

        result = dict(frames=received, commands=len(commands), distance_m=data['distance_m'],
                      cells=len(data['road']), lines=Counter(str(int(k)) for k in data['lines'][:, 2]),
                      start_finish_gap_m=float(np.linalg.norm(data['pose'][:2])),
                      static_records=len(data['static']), moving_records=len(data['moving']),
                      openings=len(data['openings']), png_bytes=path.stat().st_size)
        (args.output / 'result.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
        print(json.dumps(result))
    finally:
        window.lap_map.close()
        window.close()
        control.recording.recorder.close()
        executor.shutdown()
        for node in (gui, feeder, driver):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
