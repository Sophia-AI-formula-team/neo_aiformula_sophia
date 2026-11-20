"""Replay one Unity exchange through the vehicle's Foxy gate and stock motor node."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from std_srvs.srv import SetBool, Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from foxy_gate_dds import Fixture, LOG, require


def compare(fixture, rows):
    fixture.launch()
    fixture.until(lambda: fixture.feedback.get('valid') and fixture.status.get('mode') == 'disabled'
                  and fixture.enable.service_is_ready(), 'vehicle nodes not ready', 15)
    enable = SetBool.Request(data=True)
    fixture.until(lambda: fixture.service(fixture.enable, enable, success=None) == 'output_enabled',
                  'could not enable isolated software receiver', 8)
    fixture.mode('paused', 'output_enabled')
    # Startup discovery of the joystick and feedback can lag the enable service.
    # Pump real messages until the unmodified gate accepts the resume request.
    fixture.until(lambda: fixture.service(fixture.resume, Trigger.Request(), success=None) == 'waiting_new_auto',
                  'software gate did not receive its startup interlocks', 8)
    fixture.mode('auto', 'waiting_new_auto')

    results = []
    for index, row in enumerate(rows):
        cmd, header = row['command'], row['frame']
        require(index == header['seq'] == cmd['seq'], 'missing or unordered camera response')
        if index + 1 < len(rows):
            interval = rows[index + 1]['frame']['simulation_time_s'] - header['simulation_time_s']
        else:
            interval = header['simulation_time_s'] - rows[index - 1]['frame']['simulation_time_s']
        require(.06 <= interval <= .5, 'capture interval outside the Unity protocol range')
        message = TwistStamped()
        message.header.stamp = fixture.probe.get_clock().now().to_msg()
        message.twist.linear.x, message.twist.angular.z = cmd['linear_mps'], cmd['angular_z_radps']
        require(all(map(math.isfinite, (message.twist.linear.x, message.twist.angular.z))), 'invalid Twist')
        before, start = len(fixture.frames), time.monotonic()
        fixture.auto.publish(message)
        fixture.dwell(interval)
        wire = fixture.frames[before:]
        expected = (cmd['right_can_rpm'], cmd['left_can_rpm'])
        # The previous target may remain on the wire for one converter tick.
        # Require convergence for the remainder of this command, not just one coincidental match.
        last_wrong = max((i for i, (_, value) in enumerate(wire) if value != expected), default=-1)
        tail = wire[last_wrong + 1:]
        results.append(dict(seq=index, expected_right_left=expected, received_frames=len(wire),
                            final_right_left=wire[-1][1] if wire else None, stable_frames=len(tail),
                            match=len(tail) >= 2 and fixture.status.get('mode') == 'auto',
                            settled_after_ms=round(1000 * (tail[0][0] - start), 2) if tail else None))

    fixture.mode('paused', 'auto_timeout')
    before = len(fixture.frames)
    fixture.dwell(.20)
    stopped = fixture.frames[before:]
    zero = len(stopped) >= 3 and all(value == (0, 0) for _, value in stopped)
    return dict(software='vehicle gate + stock motor controller + DDS software CAN receiver',
                input='Unity Twist sequence, freshly timestamped at its simulated capture cadence',
                feedback='fixture wheel, IMU and idle joystick heartbeat', commands=len(results),
                matches=sum(r['match'] for r in results), watchdog_stop=zero,
                zero_frames_after_timeout=len(stopped), rows=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('exchange', type=Path)
    parser.add_argument('result', type=Path)
    args = parser.parse_args()
    require(Path('/.dockerenv').is_file() and os.environ.get('ROS_DOMAIN_ID') == '69'
            and os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'use an isolated Docker network and ROS domain')
    rows = [json.loads(line) for line in args.exchange.read_text().splitlines() if line.strip()]
    require(len(rows) >= 2, 'need two timestamped camera commands')
    LOG.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    fixture = Fixture()
    try:
        report = compare(fixture, rows)
        args.result.write_text(json.dumps(report, indent=2) + '\n')
        print('Matched {matches}/{commands} commands; silent publisher stopped: {watchdog_stop}'.format(**report))
        require(report['matches'] == report['commands'] and report['watchdog_stop'], 'gate trace mismatch')
    finally:
        fixture.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
