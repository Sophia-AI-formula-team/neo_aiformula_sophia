"""Unity stdio adapter: original LYA + neo stop_wait MotorGate + topo observer.

This is an in-process ROS shim, not DDS. Traffic votes and receipt timeouts use
real monotonic time; ROS/odom stamps use unchanged Unity simulation time.
signal_range and bbox-distance are deliberately not part of this protocol.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace as NS
import uuid

from sim_checkpoint_adapter import install_windows_fcntl_transport, build_simulation_map_class

sys.dont_write_bytecode = True
MAX_LINE = 1024 * 1024


class Pose2D:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta


class Twist:
    def __init__(self):
        self.linear, self.angular = NS(x=0.0, y=0.0, z=0.0), NS(x=0.0, y=0.0, z=0.0)


class DataMessage:
    def __init__(self, data=None):
        self.data = data


def header():
    return NS(frame_id='odom', stamp=NS(sec=0, nanosec=0))


def pose():
    return NS(position=NS(x=0.0, y=0.0, z=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))


class PoseStamped:
    def __init__(self):
        self.header, self.pose = header(), pose()


class Odometry:
    def __init__(self):
        self.header, self.child_frame_id = header(), 'base_link'
        self.pose = NS(pose=pose(), covariance=[0.0] * 36)
        self.twist = NS(twist=Twist(), covariance=[0.0] * 36)


class PathMessage:
    def __init__(self):
        self.header, self.poses = header(), []


class Publisher:
    def __init__(self):
        self.value, self.count = None, 0

    def publish(self, message):
        self.value = copy.deepcopy(message)
        self.count += 1


def import_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def hash_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(name + ' must be finite')
    return float(value)


def sequence(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(name + ' must be a nonnegative integer')
    return value


class SemanticTransport:
    def __init__(self, source_root, vision_root, motor_gate_source,
                 checkpoint_root, checkpoint_path, traffic_policy='stop_wait'):
        if traffic_policy != 'stop_wait':
            raise ValueError('This adapter accepts stop_wait only; bbox/legacy are separate')
        self.source_root, self.vision_root = Path(source_root).resolve(), Path(vision_root).resolve()
        self.motor_gate_source = Path(motor_gate_source).resolve()
        self.checkpoint_root, self.checkpoint_path = checkpoint_root, checkpoint_path
        self.sim_time, self.previous_sim_time = 0.0, None
        self.worker_boot_id = str(uuid.uuid4())
        self.last_tick_sequence, self.last_vision_sequence = -1, -1
        self.epoch = None
        self.total_vision_votes = 0
        self.last_controller = None
        self.point_corridor = None
        self.restore_event = None
        self.closed = False
        source = self.source_root / 'semantic_local_planner_ros/semantic_local_planner_ros/map_assist.py'
        self.source_hash = hash_file(source)
        source_files = sorted(p for p in self.source_root.rglob('*')
                              if p.is_file() and p.suffix in ('.py', '.yaml', '.osm')
                              and '__pycache__' not in p.parts)
        self.source_files = {p.relative_to(self.source_root).as_posix(): hash_file(p) for p in source_files}
        self.source_tree_hash = hashlib.sha256(json.dumps(self.source_files, sort_keys=True).encode()).hexdigest()
        self.motor_gate_hash = hash_file(self.motor_gate_source)
        manifest = json.loads((self.vision_root / 'upstream/provenance.json').read_text(encoding='utf-8'))
        self.vision_commit = manifest['commit']
        for entry in manifest['files']:
            if hash_file(self.vision_root / 'upstream' / entry['path']) != entry['sha256']:
                raise ValueError('Pinned LYA/LLP source mismatch: ' + entry['path'])
        self.vision_adapter_hash = hash_file(self.vision_root / 'upstream_adapter.py')
        self._install_ros_transport()
        install_windows_fcntl_transport()
        sys.path.insert(0, str(self.source_root / 'semantic_planner_core'))
        sys.path.insert(0, str(self.source_root / 'semantic_local_planner_ros'))
        self.map_module = import_file('semantic_trial_map_assist', source)
        self.motor_module = import_file('semantic_trial_motor_gate', self.motor_gate_source)
        map_class = build_simulation_map_class(self.map_module.MapAssist, checkpoint_root, checkpoint_path)
        self.map = map_class()
        self.gate = self.motor_module.MotorGate()

    def _install_ros_transport(self):
        transport = self
        overrides = dict(enable_traffic_control=True, enable_velocity_gate=True,
                         avoidance_initially_idle=True, use_sim_time=True,
                         traffic_policy='stop_wait', enable_signal_approach=False,
                         enable_correction=False, checkpoint_path=self.checkpoint_path)

        class Node:
            def __init__(self, name):
                self.name, self.parameters = name, {'use_sim_time': True}

            def declare_parameter(self, name, default):
                self.parameters[name] = overrides.get(name, default)

            def get_parameter(self, name):
                return NS(value=self.parameters[name])

            def create_publisher(self, *_):
                return Publisher()

            def create_subscription(self, message_type, topic, callback, qos):
                return NS(topic=topic, callback=callback)

            def create_timer(self, interval, callback, clock=None):
                return NS(interval=interval, callback=callback, clock=clock)

            def get_logger(self):
                def log(message):
                    print('[' + self.name + '] ' + message, file=sys.stderr)
                return NS(info=log, warn=log, warning=log, error=log, debug=lambda *_: None)

            def get_clock(self):
                def now():
                    nanos = int(transport.sim_time * 1e9)
                    return NS(nanoseconds=nanos, to_msg=lambda: NS(sec=nanos // 10**9, nanosec=nanos % 10**9))
                return NS(now=now)

        names = ('rclpy', 'rclpy.node', 'rclpy.clock', 'rclpy.qos', 'geometry_msgs', 'geometry_msgs.msg',
                 'nav_msgs', 'nav_msgs.msg', 'std_msgs', 'std_msgs.msg', 'ament_index_python', 'ament_index_python.packages')
        modules = {name: ModuleType(name) for name in names}
        modules['rclpy.node'].Node = Node
        modules['rclpy.clock'].Clock = lambda **kwargs: NS(**kwargs)
        modules['rclpy.clock'].ClockType = NS(STEADY_TIME=3)
        modules['rclpy.qos'].QoSProfile = lambda **kwargs: NS(**kwargs)
        modules['rclpy.qos'].DurabilityPolicy = NS(TRANSIENT_LOCAL=1)
        for name, cls in (('Pose2D', Pose2D), ('PoseStamped', PoseStamped), ('Twist', Twist)):
            setattr(modules['geometry_msgs.msg'], name, cls)
        modules['nav_msgs.msg'].Odometry, modules['nav_msgs.msg'].Path = Odometry, PathMessage
        modules['std_msgs.msg'].Bool = modules['std_msgs.msg'].String = DataMessage
        modules['ament_index_python.packages'].get_package_share_directory = lambda name: str(self.source_root / name)
        for name, module in modules.items():
            if '.' in name:
                parent, child = name.rsplit('.', 1)
                setattr(modules[parent], child, module)
        sys.modules.update(modules)

    def ready(self):
        return dict(type='ready', transport='in_process_ros_shim', control_tick_hz=20,
            worker_boot_id=self.worker_boot_id, source_hash=self.source_hash,
            source_tree_hash=self.source_tree_hash, source_files=self.source_files,
            vision_commit=self.vision_commit, vision_adapter_hash=self.vision_adapter_hash,
            motor_gate_hash=self.motor_gate_hash, traffic_policy='stop_wait', signal_approach=False,
            command_source='existing_detector_LaneControl_original_LYA',
            time_basis='real_monotonic_arrival', ros_time_basis='unchanged_Unity_simulation_time',
            odom_source='caller_simulated_odometry',
            checkpoint_scope=self.map.simulation_checkpoint_scope,
            semantic_revision='neo_mapassist_0.3.0_motor_gate_0.4.1_unity_stop_wait_v1')

    def _corridor_context(self, vision, image_time, age):
        value = vision.get('corridor')
        if value is None:
            return None
        if not isinstance(value, dict) or value.get('source') not in ('current_right_inferred', 'current_left_identity_checked_native'):
            raise ValueError('Unknown corridor source')
        stamp = finite(value['calibration_stamp'], 'corridor.calibration_stamp')
        calibration_sequence = sequence(value['calibration_sequence'], 'corridor.calibration_sequence')
        capture_age = finite(value['calibration_age_at_capture'], 'corridor.calibration_age_at_capture')
        maximum = finite(value['max_calibration_age_sec'], 'corridor.max_calibration_age_sec')
        width = finite(value['width_m'], 'corridor.width_m')
        if (stamp < 0 or stamp > image_time or calibration_sequence < 1
                or calibration_sequence > vision['sequence'] or capture_age < 0
                or abs(capture_age - (image_time - stamp)) > 1e-6 or maximum != 1.5 or width <= 0):
            raise ValueError('Invalid corridor calibration')
        received_age = capture_age + max(age, self.sim_time - image_time)
        if received_age > maximum:
            raise ValueError('Expired corridor calibration')
        return dict(source=value['source'], width_m=width, calibration_sequence=calibration_sequence,
            calibration_stamp=stamp, source_sequence=vision['sequence'], received_age=received_age,
            received_at=time.monotonic(), max_age_sec=maximum)

    def _corridor_guard(self):
        value = self.point_corridor
        if value is None:
            return None
        elapsed = time.monotonic() - value['received_at']
        age = max(self.sim_time - value['calibration_stamp'], value['received_age'] + elapsed)
        valid = elapsed >= 0 and 0 <= age <= value['max_age_sec']
        return dict(source=value['source'], source_sequence=value['source_sequence'],
            calibration_sequence=value['calibration_sequence'], width_m=value['width_m'],
            calibration_age_sec=age, max_age_sec=value['max_age_sec'], valid=valid,
            remaining_sec=max(0.0, value['max_age_sec'] - age) if valid else 0.0)

    def _receive_vision(self, vision):
        if vision is None:
            return 'none', None
        if 'signal_range' in vision:
            raise ValueError('signal_range belongs to the separate distance trial, not stop_wait')
        vision_sequence = sequence(vision['sequence'], 'vision.sequence')
        if vision_sequence <= self.last_vision_sequence:
            return 'duplicate_or_out_of_order', None
        self.last_vision_sequence = vision_sequence
        if vision.get('epoch') != self.epoch:
            raise ValueError('Vision epoch differs from current continuous odometry session')
        age = finite(vision['age'], 'vision.age')
        image_time = finite(vision['sim_time'], 'vision.sim_time')
        if age < 0 or image_time < 0 or image_time > self.sim_time + 0.001:
            raise ValueError('Invalid vision source age/time')
        source_age = max(age, self.sim_time - image_time)
        if source_age > self.map.p['point_timeout_sec']:
            return 'stale', None
        state = str(vision['traffic_light_state']).strip().lower()
        if state not in ('red', 'green', 'unknown'):
            raise ValueError('Invalid traffic light state')
        # Native stop_wait consumes an atomic color-only observation. Keeping
        # source age here prevents a delayed green frame becoming fresh again
        # while the real arrival-time vote window matures. No metric range is
        # inferred or used; the separate bbox protocol remains forbidden.
        observation = dict(state=state, sequence=vision_sequence, age_sec=source_age,
                           distance_m=None, uncertainty_m=0.0)
        self.map._on_signal_observation(DataMessage(json.dumps(observation, allow_nan=False)))
        self.total_vision_votes += 1
        if vision.get('supported') is not True:
            return 'light_only_unsupported_lane', None
        if vision.get('vision_commit') != self.vision_commit:
            raise ValueError('Original LYA command provenance differs from pinned source')
        try:
            corridor = self._corridor_context(vision, image_time, age)
        except (ValueError, TypeError, KeyError, OverflowError):
            return 'light_only_invalid_corridor', None
        controller = vision['controller']
        v, w = finite(controller['v'], 'controller.v'), finite(controller['w'], 'controller.w')
        target = controller.get('target')
        if not isinstance(target, list) or len(target) != 3:
            raise ValueError('Original LYA command requires its source target')
        target = [finite(item, 'target') for item in target]
        raw = dict(v=v, w=w, target=target, sequence=vision_sequence,
                   sim_time=image_time, source_age_sec=source_age, received_at=time.monotonic())
        self.last_controller = raw
        self.point_corridor = corridor
        # Topo observes the existing ABC. A missing private ABC is independent
        # of the already produced fresh LYA command, as in motor 0.4.1.
        points = vision.get('selected_points')
        if points is None:
            return 'command_accepted_no_topo_abc', raw
        if not isinstance(points, list) or len(points) != 3:
            raise ValueError('selected_points must contain three points or be null')
        validated = []
        for point in points:
            if not isinstance(point, list) or len(point) not in (2, 3):
                raise ValueError('ABC point requires x,y and optional theta')
            validated.append(Pose2D(*[finite(item, 'point') for item in point]))
        for label, point in zip('abc', validated):
            self.map._point_cb(label)(point)
        return 'accepted', raw

    def tick(self, request):
        if request.get('type') != 'tick':
            raise ValueError('Only type=tick is accepted')
        tick_sequence = sequence(request['sequence'], 'sequence')
        if tick_sequence <= self.last_tick_sequence:
            raise ValueError('Tick sequence must increase')
        self.last_tick_sequence = tick_sequence
        self.sim_time = finite(request['sim_time'], 'sim_time')
        if self.sim_time < 0 or (self.previous_sim_time is not None and self.sim_time < self.previous_sim_time):
            raise ValueError('Unity simulation time must not rewind')
        self.previous_sim_time = self.sim_time
        epoch = sequence(request['lane_epoch'], 'lane_epoch')
        if self.epoch is None:
            self.epoch = epoch
        elif epoch != self.epoch:
            raise ValueError('Lane epoch changed; refusing an implicit map reset')
        values = {name: finite(request['odom'][name], 'odom.' + name) for name in ('x', 'y', 'yaw', 'v', 'w')}
        odom = Odometry()
        nanos = int(self.sim_time * 1e9)
        odom.header.stamp = NS(sec=nanos // 10**9, nanosec=nanos % 10**9)
        odom.pose.pose.position.x, odom.pose.pose.position.y = values['x'], values['y']
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(values['yaw']/2), math.cos(values['yaw']/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = values['v'], values['w']
        # Register the session first so its initial auto_after cannot accept a
        # stale command. The raw source capture receipt is checked below.
        now = time.monotonic()
        self.gate.session_message(json.dumps(dict(session=self.worker_boot_id,
            sequence=tick_sequence, stamp_ns=nanos, active=True)), nanos, now)
        vision_result, raw = self._receive_vision(request.get('vision'))
        awaiting_restore = self.map.restore_pending is not None
        self.map._on_odom(odom)
        if (awaiting_restore and self.map.restore_pending is None and not self.map.restore_blocked
                and self.map.persistence_status['state'] == 'restored'):
            self.restore_event = dict(observed=True, tick_sequence=tick_sequence,
                odom_stamp=self.sim_time, worker_boot_id=self.worker_boot_id,
                persistence=copy.deepcopy(self.map.persistence_status),
                recorded_samples=len(self.map.samples), recorded_distance_m=self.map.recorded_distance,
                reference_length_m=self.map.reference.length_m if self.map.reference is not None else None)
        point_count_before = self.map.point_pubs['c'].count
        self.map._tick()
        # The shim has no ROS executor: explicitly call the production status
        # timer to drive asynchronous checkpoint submission and diagnostics.
        self.map._status_tick()
        self.map._publish_status()
        status = json.loads(self.map.status_pub.value.data)
        now = time.monotonic()
        self.gate.status(self.map.status_pub.value.data, nanos, now)
        if raw is not None:
            # Match production's ARRIVAL-time command contract. Source capture
            # TTL is separately enforced here and at Unity consumption.
            self.gate.command((raw['v'], raw['w']), time.monotonic())
        now = time.monotonic()
        v, w = self.gate.output(nanos, now)
        previous = self.last_controller
        source_age = (now-previous['received_at']+previous['source_age_sec']) if previous else None
        if source_age is not None and source_age > self.gate.command_timeout:
            v, w = self.gate.hold('source_command_timeout', now)
        corridor = self._corridor_guard()
        gate_reason = self.gate.reason
        if corridor is not None and corridor['valid'] is not True:
            v, w = self.gate.hold('corridor_width_prior_expired', now)
            gate_reason = self.gate.reason
        selected = [self.map.points[label] for label in 'abc']
        return dict(type='tick', sequence=tick_sequence, sim_time=self.sim_time,
            worker_boot_id=self.worker_boot_id, valid=True, v=float(v), w=float(w), status=status,
            semantic_v=float(v), semantic_w=float(w), corridor_guard=corridor,
            command_gate=dict(reason=gate_reason, policy='motor_0.4.1_traffic_only',
                traffic_stop_required=status['traffic_stop_required'],
                source_sequence=previous['sequence'] if previous else None,
                source_age_sec=source_age,
                remaining_sec=max(0.0, self.gate.command_timeout-source_age) if previous else 0.0,
                raw_command_new=raw is not None),
            selected_points=[[float(p.x), float(p.y)] for p in selected if p is not None],
            selected_points_new=raw is not None and request.get('vision', {}).get('selected_points') is not None,
            topo_selected_points_new=self.map.point_pubs['c'].count > point_count_before,
            target=previous['target'] if previous else [],
            controller_v=previous['v'] if previous else 0.0,
            controller_w=previous['w'] if previous else 0.0,
            command_new=raw is not None, vision_result=vision_result,
            vision_votes_total=self.total_vision_votes, last_vision_sequence=self.last_vision_sequence,
            epoch=self.epoch, source_hash=self.source_hash, source_tree_hash=self.source_tree_hash,
            motor_gate_hash=self.motor_gate_hash, vision_commit=self.vision_commit,
            restore_event=self.restore_event,
            checkpoint_scope=self.map.simulation_checkpoint_scope)

    def close(self):
        if not self.closed:
            self.map.close_checkpoint()
            self.closed = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', required=True)
    parser.add_argument('--vision-root', required=True)
    parser.add_argument('--motor-gate-source', required=True)
    parser.add_argument('--checkpoint-root', required=True)
    parser.add_argument('--checkpoint-path', required=True)
    parser.add_argument('--traffic-policy', default='stop_wait', choices=('stop_wait',))
    args = parser.parse_args()
    protocol, sys.stdout = sys.stdout, sys.stderr

    def send(value):
        protocol.write(json.dumps(value, separators=(',', ':'), allow_nan=False) + '\n')
        protocol.flush()

    transport, request = None, {}
    try:
        transport = SemanticTransport(args.source_root, args.vision_root, args.motor_gate_source,
                                      args.checkpoint_root, args.checkpoint_path, args.traffic_policy)
        send(transport.ready())
        while True:
            line = sys.stdin.buffer.readline(MAX_LINE + 1)
            if not line:
                return 0
            if len(line) > MAX_LINE:
                raise ValueError('Input exceeds transport limit')
            request = json.loads(line)
            if request.get('type') == 'shutdown':
                transport.close()
                send(dict(type='shutdown', clean=True, worker_boot_id=transport.worker_boot_id,
                          checkpoint_ok=(transport.map.persistence_status['state'] in ('saved', 'new_map')
                                         and transport.map.persistence_status.get('error') is None),
                          persistence=transport.map.persistence_status))
                return 0
            send(transport.tick(request))
    except Exception as error:
        send(dict(type='error', sequence=request.get('sequence') if isinstance(request, dict) else None,
                  valid=False, v=0.0, w=0.0, error=type(error).__name__ + ': ' + str(error)))
        return 1
    finally:
        if transport is not None:
            transport.close()


if __name__ == '__main__':
    raise SystemExit(main())
