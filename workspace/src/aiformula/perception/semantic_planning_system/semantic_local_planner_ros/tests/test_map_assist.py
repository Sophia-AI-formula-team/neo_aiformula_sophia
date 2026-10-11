"""Callback tests for map_assist with minimal ROS stubs, not a live ROS runtime.

The real callbacks and route algorithms run here. DDS, ROS scheduling and actual
vehicle motion are outside these tests. All injected ROS modules are restored.
"""

import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


class Pose2D:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class DataMessage:
    def __init__(self, data=None):
        self.data = data


def header():
    return types.SimpleNamespace(frame_id="", stamp=types.SimpleNamespace(sec=0, nanosec=0))


def pose():
    return types.SimpleNamespace(
        position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


class PoseStamped:
    def __init__(self):
        self.header, self.pose = header(), pose()


class Odometry:
    def __init__(self):
        self.header = header()
        self.child_frame_id = "base_link"
        self.pose = types.SimpleNamespace(pose=pose(), covariance=[0.0] * 36)


class PathMessage:
    def __init__(self):
        self.header, self.poses = header(), []


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class MapAssistTests(unittest.TestCase):
    def setUp(self):
        self.package = Path(__file__).resolve().parents[1]
        self.system = self.package.parent
        self.now = 100.0
        self.ros_now = None
        official = self.package / "config" / "aiformula_topomap.osm"
        self.assertTrue(official.is_file(), "The package must include the supplied official map fixture")
        overrides = {
            "topomap_path": str(official),
            "min_lap_distance_m": 10.0,
            "lap_closure_radius_m": 0.1,
            "lap_closure_heading_deg": 5.0,
            "progress_tolerance_m": 0.2,
            "persistence_frames": 3,
        }
        self.parameter_overrides = overrides
        test_case = self

        class FakeNode:
            def __init__(self, name):
                self.name = name
                self.parameters = {"use_sim_time": overrides.get("use_sim_time", False)}
                self.publishers, self.subscriptions, self.timers = {}, {}, []
                self.logs = []

            def declare_parameter(self, name, default):
                self.parameters[name] = overrides.get(name, default)

            def get_parameter(self, name):
                return types.SimpleNamespace(value=self.parameters[name])

            def create_publisher(self, message_type, topic, qos):
                publisher = Publisher()
                self.publishers[topic] = publisher
                return publisher

            def create_subscription(self, message_type, topic, callback, qos):
                self.subscriptions[topic] = callback
                return callback

            def create_timer(self, interval, callback, clock=None):
                self.timers.append((interval, callback, clock))
                return callback

            def get_logger(self):
                return types.SimpleNamespace(info=self.logs.append, warn=self.logs.append)

            def get_clock(self):
                def now():
                    seconds = test_case.now if test_case.ros_now is None else test_case.ros_now
                    return types.SimpleNamespace(
                        nanoseconds=int(seconds * 1e9),
                        to_msg=lambda: types.SimpleNamespace(sec=int(seconds), nanosec=0),
                    )
                return types.SimpleNamespace(now=now)

        modules = {}
        for name in ("rclpy", "rclpy.node", "rclpy.qos", "rclpy.clock", "geometry_msgs", "geometry_msgs.msg",
                     "nav_msgs", "nav_msgs.msg", "std_msgs", "std_msgs.msg"):
            modules[name] = types.ModuleType(name)
        modules["rclpy.node"].Node = FakeNode
        modules["rclpy.clock"].Clock = lambda **kwargs: types.SimpleNamespace(**kwargs)
        modules["rclpy.clock"].ClockType = types.SimpleNamespace(STEADY_TIME=3)
        modules["rclpy.qos"].QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
        modules["rclpy.qos"].DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
        modules["geometry_msgs.msg"].Pose2D = Pose2D
        modules["geometry_msgs.msg"].Twist = Twist
        modules["geometry_msgs.msg"].PoseStamped = PoseStamped
        modules["nav_msgs.msg"].Odometry = Odometry
        modules["nav_msgs.msg"].Path = PathMessage
        modules["std_msgs.msg"].Bool = DataMessage
        modules["std_msgs.msg"].String = DataMessage
        for parent, child in (("rclpy", "node"), ("rclpy", "qos"), ("rclpy", "clock"),
                              ("geometry_msgs", "msg"), ("nav_msgs", "msg"), ("std_msgs", "msg")):
            setattr(modules[parent], child, modules[parent + "." + child])
        path_patch = patch.object(sys, "path", [str(self.system / "semantic_planner_core")] + sys.path)
        path_patch.start()
        self.addCleanup(path_patch.stop)
        missing = object()
        previous_modules = {name: sys.modules.get(name, missing) for name in modules}

        def restore_modules():
            for name, previous in previous_modules.items():
                if previous is missing:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = previous

        sys.modules.update(modules)
        self.addCleanup(restore_modules)
        source = self.package / "semantic_local_planner_ros" / "map_assist.py"
        spec = importlib.util.spec_from_file_location("map_assist_under_test", source)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.module.time = types.SimpleNamespace(monotonic=lambda: self.now)
        self.node = self.module.MapAssist()

    def test_bare_startup_loads_installed_vehicle_profile(self):
        config = str(self.package / "config" / "map_assist.yaml")
        self.assertTrue(Path(config).is_file())
        with patch.object(self.module, "installed_config", return_value=config) as lookup:
            with patch.object(self.module.sys, "argv", ["map_assist"]):
                args = self.module.startup_args()
        lookup.assert_called_once_with("map_assist.yaml")
        self.assertEqual(args, ["--ros-args", "--params-file", config, "--"])

    def test_startup_keeps_explicit_overrides_after_vehicle_profile(self):
        args = ["--ros-args", "-p", "use_sim_time:=false", "-r", "__node:=map_check"]
        original = list(args)
        with patch.object(self.module, "installed_config", return_value="/installed/map_assist.yaml"):
            actual = self.module.startup_args(args)
        self.assertEqual(actual[-len(args):], args)
        self.assertEqual(actual[:4], ["--ros-args", "--params-file", "/installed/map_assist.yaml", "--"])
        self.assertEqual(args, original)

    def test_explicit_live_config_is_kept_without_injecting_another_profile(self):
        args = ["--ros-args", "--params-file", "/installed/map_assist.yaml", "-p", "enable_velocity_gate:=true"]
        with patch.object(self.module, "installed_config") as lookup:
            self.assertEqual(self.module.startup_args(args), args)
        lookup.assert_not_called()

    def test_explicit_bag_config_does_not_load_vehicle_profile(self):
        args = ["--ros-args", "--params-file", "/installed/map_assist_bag.yaml"]
        with patch.object(self.module, "installed_config") as lookup:
            self.assertEqual(self.module.startup_args(args), args)
        lookup.assert_not_called()

    def test_empty_map_path_loads_installed_official_map(self):
        self.parameter_overrides["topomap_path"] = ""
        official = str(self.package / "config" / "aiformula_topomap.osm")
        with patch.object(self.module, "installed_config", return_value=official) as lookup:
            node = self.module.MapAssist()
        lookup.assert_called_once_with("aiformula_topomap.osm")
        self.assertEqual(node.p["topomap_path"], official)
        self.assertIsNotNone(node.official)

    def test_explicit_map_path_keeps_priority(self):
        with patch.object(self.module, "installed_config") as lookup:
            node = self.module.MapAssist()
        lookup.assert_not_called()
        self.assertEqual(node.p["topomap_path"], self.parameter_overrides["topomap_path"])

    def test_startup_publishes_status_without_inputs(self):
        self.assertEqual(len(self.node.status_pub.messages), 1)
        status = json.loads(self.node.status_pub.messages[-1].data)
        self.assertEqual(status["phase"], "learning")
        self.assertEqual(status["reason"], "waiting_for_odom")
        self.assertIsNone(status["odom_age_sec"])
        self.assertEqual(status["point_age_sec"], dict.fromkeys("abc"))
        self.assertFalse(status["recording_points_valid"])
        self.assertEqual(self.output_count(), 0)

    def test_only_status_timer_uses_steady_time(self):
        control = next(timer for timer in self.node.timers if timer[1] == self.node._tick)
        status = next(timer for timer in self.node.timers if timer[1] == self.node._status_tick)
        self.assertIsNone(control[2], "Vehicle control must retain the node's ROS clock")
        self.assertEqual(status[2].clock_type, self.module.ClockType.STEADY_TIME)
        self.assertAlmostEqual(status[0], 0.2)

    def test_status_continues_without_clock_and_during_paused_bag(self):
        self.parameter_overrides["use_sim_time"] = True
        self.ros_now = 0.0
        self.node = self.module.MapAssist()
        for ros_time in (0.0, 1234.0):
            self.ros_now = ros_time
            before = len(self.node.status_pub.messages)
            for _ in range(3):
                self.advance(0.21)
                self.node.status_timer()
            self.assertEqual(len(self.node.status_pub.messages), before + 3)
            status = json.loads(self.node.status_pub.messages[-1].data)
            self.assertTrue(status["use_sim_time"])
            self.assertAlmostEqual(status["ros_time_sec"], ros_time)
            self.assertEqual(status["reason"], "waiting_for_odom")
        self.assertEqual(self.output_count(), 0)
        self.assertEqual(self.node.path_pub.messages, [])

    def test_status_reports_stale_inputs_when_ros_clock_is_paused(self):
        self.ros_now = 1234.0
        self.start_session()
        self.triplet()
        self.odom()
        self.node._status_tick()
        self.advance(1.0)
        self.node._status_tick()
        status = json.loads(self.node.status_pub.messages[-1].data)
        self.assertAlmostEqual(status["ros_time_sec"], 1234.0)
        self.assertAlmostEqual(status["odom_age_sec"], 1.0)
        for age in status["point_age_sec"].values():
            self.assertAlmostEqual(age, 1.0)
        self.assertFalse(status["recording_points_valid"])

    def test_status_timer_never_runs_control_or_velocity_gate(self):
        self.traffic_node()
        self.ready_visual_motion()
        self.node._on_controller_cmd(self.command())
        points_before = self.output_count()
        commands_before = len(self.node.cmd_pub.messages)
        used_seq_before = dict(self.node.used_seq)
        self.node.traffic_light = "red"
        self.node.traffic_mode = "wait"
        self.advance(1.0)
        with patch.object(self.node, "_update_traffic") as update:
            with patch.object(self.node, "_publish_velocity_stop_if_needed") as stop:
                self.node._status_tick()
        update.assert_not_called()
        stop.assert_not_called()
        self.assertEqual(self.output_count(), points_before)
        self.assertEqual(len(self.node.cmd_pub.messages), commands_before)
        self.assertEqual(self.node.used_seq, used_seq_before)
        self.assertEqual(self.node.traffic_mode, "wait")

    def test_status_rate_is_independent_of_control_tick(self):
        count = len(self.node.status_pub.messages)
        self.advance(0.1)
        self.node._status_tick()
        self.node._tick()
        self.assertEqual(len(self.node.status_pub.messages), count)
        self.advance(0.11)
        self.node._tick()
        self.assertEqual(len(self.node.status_pub.messages), count)
        self.node._status_tick()
        self.assertEqual(len(self.node.status_pub.messages), count + 1)

    def advance(self, seconds=0.05):
        self.now += seconds

    def start_session(self):
        self.node._on_avoidance(DataMessage(False))
        self.advance(self.node.p["resume_delay_sec"] + 0.01)
        # Start recording after the initial ownership state is known and settled.
        self.node._on_reset(DataMessage(True))

    @staticmethod
    def visual_points(angle_degrees=0.0):
        angle = math.radians(angle_degrees)
        return [Pose2D(distance * math.cos(angle), distance * math.sin(angle), 10.0 + index)
                for index, distance in enumerate((0.4, 0.8, 1.2))]

    def triplet(self, points=None):
        points = self.visual_points() if points is None else points
        for label, point in zip("abc", points):
            self.node._point_cb(label)(point)
        return points

    def odom(self, x=0.0, y=0.0, yaw=0.0, stamp=None):
        message = Odometry()
        message.header.frame_id = "odom"
        stamp = self.now if stamp is None else stamp
        message.header.stamp.sec = int(stamp)
        message.header.stamp.nanosec = int(round((stamp % 1.0) * 1e9))
        message.pose.pose.position.x, message.pose.pose.position.y = x, y
        message.pose.pose.orientation.z = math.sin(yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.node._on_odom(message)
        return message

    def output_count(self):
        counts = [len(self.node.point_pubs[label].messages) for label in "abc"]
        self.assertEqual(counts[0], counts[1])
        self.assertEqual(counts[0], counts[2])
        return counts[0]

    def assert_last_points_equal(self, expected):
        for label, point in zip("abc", expected):
            actual = self.node.point_pubs[label].messages[-1]
            self.assertAlmostEqual(actual.x, point.x)
            self.assertAlmostEqual(actual.y, point.y)
            self.assertEqual(actual.theta, point.theta)

    def learn_circle(self):
        """Exercise real recording/closure; this is not the supplied map's shape."""
        self.start_session()
        for step in range(121):
            self.advance(0.1)
            self.triplet()
            angle = 2.0 * math.pi * step / 120.0
            self.odom(3.0 * math.sin(angle), 3.0 * (1.0 - math.cos(angle)), angle)
            self.node._tick()
        self.assertIsNotNone(self.node.reference)
        self.assertGreater(self.node.reference.length_m, 18.0)
        self.assertLess(self.node.reference.length_m, 19.0)
        self.assertTrue(self.node.alignment["diagnostic_only"])
        self.assertTrue(self.node.path_pub.messages[-1].poses)
        self.assertFalse(self.node.matched, "A post-closure odom update must confirm the match")

    def seed_guard_reference(self):
        """Inject acceptance only to isolate heading/OA behavior from map fitting.

        The actual circle recording cannot establish correspondence with the
        supplied five-corner outline. Fusion acceptance is deliberately seeded
        here, not claimed as an outcome of the real fusion implementation.
        """
        self.learn_circle()
        self.assertFalse(self.node.fusion_status["available"])
        self.node.fusion_status = {
            "available": True, "mode": "fused", "reference_source": "odom_topology_fused",
            "reason": "synthetic_guard_fixture",
        }

    @staticmethod
    def dense_loop(points, spacing=0.3):
        """Record actual odometry geometry independently of the official input."""
        result = []
        for start, end in zip(points, points[1:]):
            count = max(1, math.ceil(math.dist(start, end) / spacing))
            for index in range(count):
                fraction = index / count
                result.append((start[0] + fraction * (end[0] - start[0]),
                               start[1] + fraction * (end[1] - start[1])))
        result.append(result[0])
        return result

    def complete_official_lap(self, *, traffic=False, official_transform=None, recorded_points=None):
        """Run the production lap-completion callback on a dense measured loop.

        Only raw odom samples and the already satisfied closure conditions are
        supplied. Reference building, fusion, anchors and graph construction
        are the production implementations and are never mocked.
        """
        self.parameter_overrides.update(enable_traffic_control=traffic, enable_velocity_gate=traffic)
        self.node = self.module.MapAssist()
        self.start_session()
        measured = recorded_points or self.dense_loop(self.node.official.points)
        if official_transform is not None:
            original = self.node.official
            self.node.official = self.module.OfficialRoute(
                original.node_ids, [official_transform(key, point)
                                    for key, point in zip(original.node_ids, original.points)],
            )
            graph = self.node.topology
            nodes = {key: official_transform(key, point) for key, point in graph.nodes.items()}
            adjacency = {key: {other: math.dist(nodes[key], nodes[other]) for other in neighbours}
                         for key, neighbours in graph.adjacency.items()}
            self.node.topology = self.module.TopologyGraph(nodes, adjacency, graph.closed_ways)
        self.node.samples = [self.module.RouteSample(x, y, 0.0) for x, y in measured[:-1]]
        self.node.recorded_distance = sum(math.dist(a, b) for a, b in zip(measured, measured[1:]))
        self.node.left_start_area = True
        self.node._try_complete_lap((*measured[0], 0.0), False)
        self.assertIsNotNone(self.node.reference)
        return self.node

    def conflicting_frame(self, points=None):
        self.advance()
        self.odom()
        points = self.triplet(self.visual_points(-40.0) if points is None else points)
        self.node._tick()
        return points

    def traffic_node(self):
        self.parameter_overrides.update(enable_traffic_control=True, enable_velocity_gate=True)
        self.node = self.module.MapAssist()
        self.start_session()

    def confirm_light(self, state, *, refresh_points=True, oa_command=None):
        """Feed real 10 Hz observations for a full window, without control ticks."""
        start = self.now
        for index in range(7):
            self.now = start + index * 0.1
            if self.node.odom is not None:
                self.odom(*self.node.odom)
            if refresh_points:
                self.triplet()
            if oa_command is not None:
                self.node._on_controller_cmd(oa_command)
            self.node._on_light_state(DataMessage(state))
        self.assertEqual(self.node.traffic_light, state)

    @staticmethod
    def command(speed=0.7, turn=0.2):
        message = Twist()
        message.linear.x, message.angular.z = speed, turn
        return message

    @staticmethod
    def twist_values(message):
        return tuple(getattr(vector, name) for vector in (message.linear, message.angular) for name in "xyz")

    def assert_stopped(self):
        self.assertTrue(self.node.cmd_pub.messages)
        self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]), (0.0,) * 6)

    def ready_visual_motion(self):
        points = self.triplet()
        self.odom()
        self.node._tick()
        self.assert_last_points_equal(points)

    def learned_official_loop(self, *, near_junction=True, available=True, official_transform=None):
        """Build the actual fused outer loop before exercising traffic callbacks."""
        self.complete_official_lap(traffic=True, official_transform=official_transform)
        self.assertTrue(self.node.fusion_status["available"], self.node.fusion_status)
        self.assertIsNotNone(self.node.mapped_topology)
        graph = self.node.mapped_topology
        if not available:
            self.node.official_to_odom = None
            self.node.mapped_topology = None
            self.node.fusion_status = {
                "available": False, "mode": "odom_only", "reference_source": "odom_only",
                "reason": "ambiguous_correspondence",
            }
            self.node.topology_alignment["available"] = False
            self.node.topology_alignment["reason"] = "ambiguous_correspondence"
        start = self.node.official.node_ids[0]
        forward = self.node.official.node_ids[1]
        alternative = graph.alternative_route(
            start, forward, require_internal=True, outer_node_ids=self.node.official.node_ids[:-1],
        )
        first, second = alternative.points[:2]
        yaw = math.atan2(second[1] - first[1], second[0] - first[0])
        if near_junction:
            x, y = first
            self.node.progress = self.node.fused_node_progress[start]
        else:
            x, y, _ = self.node.reference.point_at(10.0)
            self.node.progress = 10.0
        self.triplet()
        self.odom(x, y, yaw)
        self.assertTrue(self.node.matched)
        return alternative

    def complete_active_detour(self):
        """Drive the real tracker along its route until it releases to vision."""
        self.assertEqual(self.node.traffic_mode, "detour")
        route = list(self.node.detour_tracker.points)
        for start, end in zip(route, route[1:]):
            yaw = math.atan2(end[1] - start[1], end[0] - start[0])
            self.advance(0.6)
            self.odom(start[0], start[1], yaw)
            self.triplet()
            self.node._tick()
            steps = math.ceil(math.dist(start, end) / 0.5)
            for step in range(1, steps + 1):
                fraction = step / steps
                self.advance(0.2)
                self.odom(start[0] + fraction * (end[0] - start[0]),
                          start[1] + fraction * (end[1] - start[1]), yaw)
                self.triplet()
                self.node._tick()
                if self.node.traffic_mode == "normal":
                    return
        self.fail("Real detour tracker did not complete its mapped route")

    def emit_detour_completion(self):
        """Isolate light-event ordering; real route tracking is covered above."""
        completed = types.SimpleNamespace(available=False, done=True, reason="complete")
        with patch.object(self.node.detour_tracker, "update", return_value=completed):
            self.advance(0.1)
            self.odom(*self.node.odom)
            self.node._tick()

    def test_only_one_initial_false_is_needed_and_no_semantic_or_control_interfaces_exist(self):
        self.assertEqual(self.node.p["avoidance_flag_topic"], "/lane_stop_flag")
        self.assertAlmostEqual(self.node.p["resume_delay_sec"], 0.2)
        subscriptions = {self.node.p[f"input_point_{label}_topic"] for label in "abc"}
        subscriptions.update(self.node.p[key] for key in
                             ("odom_topic", "avoidance_flag_topic", "branch_active_topic", "reset_topic"))
        self.assertEqual(set(self.node.subscriptions), subscriptions)
        publishers = {self.node.p[f"output_point_{label}_topic"] for label in "abc"}
        publishers.update((self.node.p["status_topic"], self.node.p["reference_path_topic"]))
        self.assertEqual(set(self.node.publishers), publishers)
        self.assertNotIn("/lane_stop_flag", self.node.publishers)
        self.assertFalse(any("semantic" in name or "heartbeat" in name for name in self.node.p))
        self.seed_guard_reference()
        for _ in range(7):
            self.conflicting_frame()
        self.assertAlmostEqual(math.degrees(self.node.last_correction), 8.0,
                               msg="Event-based False must remain valid without heartbeat refresh")

    def test_unknown_initial_state_observes_odom_but_never_publishes_points(self):
        self.triplet()
        self.odom()
        self.node._tick()
        self.assertEqual(self.output_count(), 0)
        self.assertTrue(self.node.samples[-1].excluded)
        self.advance(0.1)
        self.triplet()
        self.odom(x=0.3)
        self.node._tick()
        self.assertEqual(self.output_count(), 0)
        self.assertEqual(len(self.node.samples), 2)
        self.assertTrue(all(sample.excluded for sample in self.node.samples))
        self.assertTrue(self.node.status_pub.messages, "Observation/status remain available")
        self.assertEqual(self.node.last_correction, 0.0)

    def test_explicit_initial_idle_allows_visual_output_but_true_still_takes_ownership(self):
        self.assertIsNone(self.node.avoidance_active)
        self.parameter_overrides["avoidance_initially_idle"] = True
        self.node = self.module.MapAssist()
        self.assertIs(self.node.avoidance_active, False)
        self.triplet()
        self.odom()
        self.node._tick()
        self.assertEqual(self.output_count(), 1)
        self.node._on_avoidance(DataMessage(True))
        self.triplet()
        self.node._tick()
        self.assertEqual(self.output_count(), 1)

    def test_only_new_complete_triplets_publish_and_stale_points_stop(self):
        self.start_session()
        points = self.visual_points()
        self.node._point_cb("a")(points[0])
        self.node._point_cb("b")(points[1])
        self.node._tick()
        self.assertEqual(self.output_count(), 0)
        self.node._point_cb("c")(points[2])
        self.node._tick()
        self.assertEqual(self.output_count(), 1)
        self.assert_last_points_equal(points)
        self.node._tick()
        self.assertEqual(self.output_count(), 1)
        self.node._point_cb("a")(points[0])
        self.node._tick()
        self.assertEqual(self.output_count(), 1)
        self.advance(0.6)
        self.node._tick()
        self.assertEqual(self.output_count(), 1)
        self.assertEqual(self.node.reason, "visual_points_stale")
        self.triplet(points)
        self.node._tick()
        self.assertEqual(self.output_count(), 2)

    def test_excessively_skewed_triplet_is_not_published(self):
        self.start_session()
        points = self.visual_points()
        self.node._point_cb("a")(points[0])
        self.advance(0.2)
        self.node._point_cb("b")(points[1])
        self.node._point_cb("c")(points[2])
        self.node._tick()
        self.assertEqual(self.output_count(), 0)
        self.assertEqual(self.node.reason, "visual_triplet_skew")

    def test_first_lap_and_bounded_second_lap_correction_do_not_mutate_or_recorrect_input(self):
        self.seed_guard_reference()
        source = self.visual_points(-40.0)
        before = [(point.x, point.y, point.theta) for point in source]
        corrections = []
        for _ in range(8):
            self.conflicting_frame(source)
            corrections.append(math.degrees(self.node.last_correction))
            count, violations = self.output_count(), self.node.guard.violation_count
            for _ in range(5):
                self.node._tick()
            self.assertEqual(self.output_count(), count)
            self.assertEqual(self.node.guard.violation_count, violations)
            for label, original in zip("abc", source):
                output = self.node.point_pubs[label].messages[-1]
                difference = math.degrees(math.atan2(output.y, output.x) - math.atan2(original.y, original.x))
                self.assertAlmostEqual(difference, corrections[-1], places=8)
                self.assertAlmostEqual(math.hypot(output.x, output.y), math.hypot(original.x, original.y))
                self.assertEqual(output.theta, original.theta)
        self.assertEqual(corrections[:2], [0.0, 0.0])
        self.assertAlmostEqual(corrections[2], 2.0)
        self.assertAlmostEqual(corrections[-1], 8.0)
        self.assertTrue(all(abs(value) <= 8.0 + 1e-9 for value in corrections))
        self.assertEqual(before, [(point.x, point.y, point.theta) for point in source])

    def test_zero_points_are_preserved_only_when_visual_control_is_allowed(self):
        self.seed_guard_reference()
        for _ in range(4):
            self.conflicting_frame()
        zeros = [Pose2D(0.0, 0.0, theta) for theta in (1.0, 2.0, 3.0)]
        self.conflicting_frame(zeros)
        self.assert_last_points_equal(zeros)
        self.assertEqual(self.node.reason, "visual_stop_preserved")
        self.assertEqual(self.node.guard.violation_count, 0)

    def test_true_immediately_releases_output_even_with_new_visual_or_zero_points_and_never_expires(self):
        self.seed_guard_reference()
        for _ in range(4):
            self.conflicting_frame()
        self.assertGreater(self.node.last_correction, 0.0)
        reference = self.node.reference
        self.node._on_avoidance(DataMessage(True))
        self.assertEqual(self.node.last_correction, 0.0)
        self.assertEqual(self.node.guard.violation_count, 0)
        count = self.output_count()
        for points in (self.visual_points(-40.0), [Pose2D(), Pose2D(), Pose2D()]):
            self.advance()
            self.odom()
            self.triplet(points)
            self.node._tick()
            self.assertEqual(self.output_count(), count, "Avoidance ownership forbids even zero-point output")
        self.advance(60.0)
        self.triplet()
        self.node._tick()
        self.assertEqual(self.output_count(), count, "True is an event state, not a heartbeat that can expire")
        self.assertIs(self.node.reference, reference)
        self.assertEqual(self.node.last_correction, 0.0)

    def test_false_discards_avoidance_cache_and_partial_groups_then_recounts_fresh_frames(self):
        self.seed_guard_reference()
        for _ in range(4):
            self.conflicting_frame()
        reference = self.node.reference
        self.node._on_avoidance(DataMessage(True))
        points = self.triplet(self.visual_points(-40.0))
        self.node._tick()
        count = self.output_count()
        self.node._on_avoidance(DataMessage(False))
        self.node._tick()
        self.assertEqual(self.output_count(), count)
        self.assertEqual(self.node.guard.violation_count, 0)
        # Repeated False cannot postpone the original 0.2-second release.
        self.advance(0.1)
        self.node._on_avoidance(DataMessage(False))
        self.node._tick()
        self.assertEqual(self.output_count(), count, "No points may publish during the resume delay")
        self.advance(0.11)
        self.odom()
        self.node._tick()
        self.assertEqual(self.output_count(), count, "The old group from avoidance must not replay")
        for label, point in zip("ab", points[:2]):
            self.node._point_cb(label)(point)
        self.node._tick()
        self.assertEqual(self.output_count(), count, "A partial group cannot reuse an old third point")
        self.node._point_cb("c")(points[2])
        self.node._tick()
        self.assertEqual(self.output_count(), count + 1)
        self.assert_last_points_equal(points)
        self.assertEqual(self.node.guard.violation_count, 1)
        self.conflicting_frame(points)
        self.assertEqual(self.node.last_correction, 0.0)
        self.conflicting_frame(points)
        self.assertAlmostEqual(math.degrees(self.node.last_correction), 2.0)
        self.assertIs(self.node.reference, reference)

    def test_complete_fresh_group_received_after_false_can_publish_once_when_delay_finishes(self):
        self.seed_guard_reference()
        self.node._on_avoidance(DataMessage(True))
        self.triplet(self.visual_points(-40.0))
        self.node._tick()
        count = self.output_count()
        self.node._on_avoidance(DataMessage(False))
        self.advance(0.1)
        points = self.triplet(self.visual_points(-40.0))
        self.node._tick()
        self.assertEqual(self.output_count(), count)
        self.advance(0.11)
        self.odom()
        self.node._tick()
        self.assertEqual(self.output_count(), count + 1,
                         "A fresh complete group after False need not arrive again after the delay")
        self.assert_last_points_equal(points)
        self.assertEqual(self.node.guard.violation_count, 1)
        self.node._tick()
        self.assertEqual(self.output_count(), count + 1)
        self.assertEqual(self.node.guard.violation_count, 1)

    def test_optional_branch_disables_correction_while_preserving_visual_output(self):
        self.seed_guard_reference()
        for _ in range(4):
            self.conflicting_frame()
        self.node._on_branch(DataMessage(True))
        self.assertEqual(self.node.last_correction, 0.0)
        self.assertEqual(self.node.guard.violation_count, 0)
        points = self.conflicting_frame()
        self.assert_last_points_equal(points)
        self.assertEqual(self.node.reason, "branch_active")
        self.node._on_branch(DataMessage(False))
        corrections = []
        for _ in range(3):
            self.conflicting_frame()
            corrections.append(math.degrees(self.node.last_correction))
        self.assertEqual(corrections[:2], [0.0, 0.0])
        self.assertAlmostEqual(corrections[2], 2.0)

    def test_reset_and_odom_jump_clear_learned_map_and_latched_path(self):
        self.learn_circle()
        self.node._on_reset(DataMessage(False))
        self.assertIsNotNone(self.node.reference)
        self.node._on_reset(DataMessage(True))
        self.assertIsNone(self.node.reference)
        self.assertIsNone(self.node.odom)
        self.assertEqual(self.node.samples, [])
        self.assertEqual(self.node.path_pub.messages[-1].poses, [])
        self.learn_circle()
        self.advance()
        self.triplet()
        self.odom(100.0, 0.0, 0.0)
        self.assertIsNone(self.node.reference)
        self.assertFalse(self.node.matched)
        self.assertEqual(self.node.reason, "odom_jump_reset")
        self.assertEqual(self.node.path_pub.messages[-1].poses, [])
        self.assertEqual(len(self.node.samples), 1)
        self.assertEqual(self.node.recorded_distance, 0.0)

    def test_repeated_short_circles_cannot_accumulate_minimum_lap_distance(self):
        self.parameter_overrides["min_lap_distance_m"] = 30.0
        self.node = self.module.MapAssist()
        self.start_session()
        early_returns = 0
        for step in range(361):
            self.advance(0.1)
            self.triplet()
            angle = 2.0 * math.pi * step / 120.0
            self.odom(3.0 * math.sin(angle), 3.0 * (1.0 - math.cos(angle)), angle)
            self.assertIsNone(self.node.reference)
            self.assertLess(self.node.recorded_distance, 19.0)
            if step and step % 120 == 0:
                early_returns += 1
                self.assertEqual(self.node.reason, "returned_before_min_lap_distance")
                self.assertEqual(self.node.recorded_distance, 0.0)
                self.assertEqual(len(self.node.samples), 1)
                self.assertFalse(self.node.left_start_area)
            self.node._tick()
        self.assertEqual(early_returns, 3)

    def test_abrupt_yaw_jump_clears_reference_even_without_translation(self):
        self.learn_circle()
        self.advance()
        self.triplet()
        self.odom(yaw=math.pi)
        self.assertIsNone(self.node.reference)
        self.assertEqual(self.node.reason, "odom_heading_jump_reset")
        self.assertFalse(self.node.matched)
        self.assertEqual(self.node.path_pub.messages[-1].poses, [])
        self.assertEqual(len(self.node.samples), 1)
        self.assertEqual(self.node.recorded_distance, 0.0)

    def test_large_header_time_rewind_restarts_and_accepts_subsequent_odom(self):
        self.learn_circle()
        self.advance()
        self.triplet()
        self.odom(stamp=1.0)
        self.assertIsNone(self.node.reference)
        self.assertEqual(self.node.reason, "odom_time_reset")
        self.assertEqual(self.node.odom_stamp, 1.0)
        self.assertEqual(self.node.path_pub.messages[-1].poses, [])
        self.advance(0.1)
        self.triplet()
        self.odom(x=0.3, stamp=1.1)
        self.assertAlmostEqual(self.node.odom_stamp, 1.1)
        self.assertAlmostEqual(self.node.odom[0], 0.3)
        self.assertAlmostEqual(self.node.recorded_distance, 0.3)
        self.assertEqual(len(self.node.samples), 2)

    def test_zero_nonfinite_or_backward_visual_points_exclude_recorded_section(self):
        self.start_session()
        cases = {
            "zero_stop": [Pose2D(), Pose2D(), Pose2D()],
            "nan_coordinate": [Pose2D(0.4, math.nan), Pose2D(0.8, 0), Pose2D(1.2, 0)],
            "infinite_theta": [Pose2D(0.4, 0, math.inf), Pose2D(0.8, 0), Pose2D(1.2, 0)],
            "backward_point": [Pose2D(-0.4, 0), Pose2D(0.8, 0), Pose2D(1.2, 0)],
        }
        for name, invalid_points in cases.items():
            with self.subTest(case=name):
                self.node._on_reset(DataMessage(True))
                self.advance(0.1)
                self.triplet()
                self.odom()
                self.assertFalse(self.node.samples[-1].excluded)
                self.advance(0.1)
                self.triplet(invalid_points)
                self.odom(x=0.3)
                self.assertTrue(self.node.samples[-1].excluded)
                self.advance(0.1)
                self.triplet()
                self.odom(x=0.6)
                self.assertTrue(self.node.samples[-1].excluded)
                self.advance(0.1)
                self.triplet()
                self.odom(x=0.9)
                self.assertFalse(self.node.samples[-1].excluded)

    def test_traffic_control_requires_a_velocity_gate_and_distinct_command_topics(self):
        self.parameter_overrides["enable_traffic_control"] = True
        with self.assertRaisesRegex(ValueError, "velocity gate"):
            self.module.MapAssist()
        self.parameter_overrides["enable_velocity_gate"] = True
        self.parameter_overrides["input_cmd_vel_topic"] = "/same"
        self.parameter_overrides["output_cmd_vel_topic"] = "/same"
        with self.assertRaisesRegex(ValueError, "input/output"):
            self.module.MapAssist()

    def test_first_lap_red_waits_unknown_stays_stopped_and_green_needs_new_points_and_command(self):
        self.traffic_node()
        self.ready_visual_motion()
        moving = self.command()
        self.node._on_controller_cmd(moving)
        self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]), self.twist_values(moving))
        count = self.output_count()
        self.confirm_light("red")
        self.assertEqual(self.node.traffic_mode, "wait")
        self.assert_stopped()
        stop_count = len(self.node.cmd_pub.messages)
        self.advance(0.1)
        self.triplet()
        self.node._tick()
        self.assertEqual(self.output_count(), count)
        self.assertGreater(len(self.node.cmd_pub.messages), stop_count)
        self.assert_stopped()
        self.node._on_light_state(DataMessage("unknown"))
        self.node._on_controller_cmd(moving)
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, "wait")
        self.assert_stopped()
        self.confirm_light("green", refresh_points=False)
        self.assertEqual(self.node.traffic_mode, "normal")
        self.node._on_controller_cmd(moving)
        self.assert_stopped()
        self.advance(0.21)
        self.node._tick()
        self.assertEqual(self.output_count(), count, "Green cannot replay points seen during red")
        self.node._on_controller_cmd(moving)
        self.assert_stopped()
        self.triplet()
        self.node._tick()
        self.assertEqual(self.output_count(), count + 1)
        self.assert_stopped()
        commands_before = len(self.node.cmd_pub.messages)
        self.node._tick()
        self.assertTrue(all(self.twist_values(msg) == (0.0,) * 6
                            for msg in self.node.cmd_pub.messages[commands_before:]))
        self.node._on_controller_cmd(moving)
        self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]), self.twist_values(moving))

    def test_velocity_gate_forwards_once_then_stops_stale_or_nonfinite_commands(self):
        self.traffic_node()
        self.ready_visual_motion()
        moving = self.command(0.8, -0.3)
        self.node._on_controller_cmd(moving)
        self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]), self.twist_values(moving))
        count = len(self.node.cmd_pub.messages)
        self.node._tick()
        self.assertEqual(len(self.node.cmd_pub.messages), count, "Timer must not replay moving commands")
        self.advance(0.6)
        self.triplet()
        self.node._tick()
        self.assert_stopped()
        self.assertEqual(self.node.velocity_gate_reason, "controller_command_timeout")
        for bad_value in (math.nan, math.inf):
            invalid = self.command()
            invalid.angular.z = bad_value
            self.node._on_controller_cmd(invalid)
            self.assert_stopped()
            self.assertEqual(self.node.velocity_gate_reason, "invalid_controller_command")
        self.node._on_controller_cmd(moving)
        self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]), self.twist_values(moving))

    def test_invalid_visual_group_revokes_previous_motion_readiness(self):
        self.traffic_node()
        self.ready_visual_motion()
        moving = self.command()
        self.node._on_controller_cmd(moving)
        self.advance(0.1)
        invalid_points = self.visual_points()
        invalid_points[1].y = math.nan
        count = self.output_count()
        self.triplet(invalid_points)
        self.node._tick()
        self.assertEqual(self.output_count(), count)
        self.node._on_controller_cmd(moving)
        self.assert_stopped()

    def test_skewed_visual_group_revokes_previous_motion_readiness(self):
        self.traffic_node()
        self.ready_visual_motion()
        moving = self.command()
        self.node._on_controller_cmd(moving)
        points = self.visual_points()
        count = self.output_count()
        self.advance(0.1)
        self.node._point_cb("a")(points[0])
        self.advance(0.2)
        self.node._point_cb("b")(points[1])
        self.node._point_cb("c")(points[2])
        self.node._tick()
        self.assertEqual(self.output_count(), count)
        self.assertEqual(self.node.reason, "visual_triplet_skew")
        self.node._on_controller_cmd(moving)
        self.assert_stopped()

    def test_detour_odom_reset_resumes_after_fresh_confirmed_green_but_holds_red_or_unknown(self):
        for light in ("green", "red", "unknown"):
            with self.subTest(light=light):
                self.learned_official_loop()
                self.confirm_light("red")
                self.node._tick()
                self.assertEqual(self.node.traffic_mode, "detour")
                self.confirm_light(light)
                self.assertEqual(self.node.traffic_mode, "detour")
                previous = self.node.odom
                self.advance(0.1)
                self.triplet()
                count = self.output_count()
                self.odom(previous[0] + 100.0, previous[1], previous[2])
                self.assertIsNone(self.node.reference)
                self.assertIsNone(self.node.detour_tracker)
                self.node._tick()
                self.assertEqual(self.output_count(), count)
                self.assert_stopped()
                self.advance(0.21)
                self.odom(*self.node.odom)
                self.triplet()
                self.node._tick()
                moving = self.command()
                self.node._on_controller_cmd(moving)
                if light == "green":
                    self.assertEqual(self.node.traffic_mode, "normal")
                    self.assertEqual(self.output_count(), count + 1,
                                     "A still-fresh confirmed green needs no extra green event after reset")
                    self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]),
                                     self.twist_values(moving))
                else:
                    self.assertEqual(self.node.traffic_mode, "wait")
                    self.assertEqual(self.output_count(), count)
                    self.assert_stopped()

    def test_oa_keeps_command_ownership_and_red_is_enforced_after_oa_release(self):
        self.traffic_node()
        self.ready_visual_motion()
        self.node._on_avoidance(DataMessage(True))
        points_count = self.output_count()
        oa_command = self.command(0.4, -0.6)
        self.node._on_controller_cmd(oa_command)
        count = len(self.node.cmd_pub.messages)
        self.confirm_light("red", oa_command=oa_command)
        self.triplet()
        self.node._tick()
        self.assertEqual(self.output_count(), points_count)
        self.assertEqual(len(self.node.cmd_pub.messages), count + 7,
                         "Each fresh OA command passes once; red cannot inject a stop")
        self.assertTrue(all(self.twist_values(message) == self.twist_values(oa_command)
                            for message in self.node.cmd_pub.messages[count:]))
        self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]), self.twist_values(oa_command))
        self.node._on_controller_cmd(oa_command)
        self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]), self.twist_values(oa_command))
        self.node._on_avoidance(DataMessage(False))
        self.assert_stopped()
        self.advance(0.21)
        self.triplet()
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, "wait")
        self.assertEqual(self.output_count(), points_count)
        self.node._on_controller_cmd(oa_command)
        self.assert_stopped()

    def test_inactive_visual_source_cannot_stop_fresh_oa_or_detour_commands(self):
        for owner in ("oa", "detour"):
            with self.subTest(owner=owner):
                if owner == "oa":
                    self.traffic_node()
                    self.ready_visual_motion()
                    self.node._on_avoidance(DataMessage(True))
                else:
                    self.learned_official_loop()
                    self.confirm_light("red")
                    self.node._tick()
                    self.assertEqual(self.node.traffic_mode, "detour")
                moving = self.command(0.5, -0.2)
                self.node._on_controller_cmd(moving)
                self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]), self.twist_values(moving))
                count = len(self.node.cmd_pub.messages)
                points_count = self.output_count()
                self.node._point_cb("b")(Pose2D(0.8, math.nan, 0.0))
                self.node._tick()
                self.assertEqual(len(self.node.cmd_pub.messages), count,
                                 "An inactive visual source cannot inject a stop over a fresh active command")
                self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]), self.twist_values(moving))
                self.assertEqual(self.output_count(), points_count)

    def test_red_automatically_avoids_forward_edge_and_emits_detour_only_on_new_odom(self):
        self.learned_official_loop()
        self.confirm_light("red")
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, "detour")
        route = self.node.detour_node_ids
        blocked = self.node.blocked_edge
        self.assertEqual(blocked, self.node.official.node_ids[:2])
        self.assertEqual((route[0], route[-1]), tuple(blocked))
        self.assertTrue(set(route) - set(self.node.official.node_ids))
        self.assertTrue(all(frozenset(edge) != frozenset(blocked) for edge in zip(route, route[1:])))
        self.assertEqual(self.output_count(), 1)
        for label in "abc":
            point = self.node.point_pubs[label].messages[-1]
            self.assertGreater(point.x, 0.0)
            self.assertTrue(math.isfinite(point.y))
        tracker = self.node.detour_tracker
        self.confirm_light("green")
        self.node._tick()
        self.assertIs(self.node.detour_tracker, tracker, "Green cannot abandon an already selected detour")
        self.assertEqual(self.output_count(), 2, "New odom received during voting emits one new path group")
        self.node._tick()
        self.assertEqual(self.output_count(), 2, "A timer without new odom cannot replay a path group")
        self.advance(0.1)
        self.odom(*self.node.odom)
        self.node._tick()
        self.assertEqual(self.output_count(), 3)
        moving = self.command()
        self.node._on_controller_cmd(moving)
        self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]), self.twist_values(moving))

    def test_red_waits_for_a_nearby_junction_and_an_unambiguous_alignment(self):
        for options, expected in (({"near_junction": False}, "red_wait_near_junction"),
                                  ({"available": False}, "red_wait_map_fusion")):
            with self.subTest(options=options):
                self.learned_official_loop(**options)
                self.confirm_light("red")
                self.node._tick()
                self.assertEqual(self.node.traffic_mode, "wait")
                self.assertEqual(self.node.traffic_reason, expected)
                self.assertEqual(self.output_count(), 0)
                self.assert_stopped()

    def test_detour_completion_reseeds_outer_progress_and_requires_fresh_visual_and_command(self):
        self.learned_official_loop()
        self.confirm_light("red")
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, "detour")
        reference = self.node.reference
        route_points = list(self.node.detour_tracker.points)
        expected_exit = self.node.detour_exit_progress
        self.confirm_light("green")
        last_position = self.node.odom
        completed = False
        for start, end in zip(route_points, route_points[1:]):
            yaw = math.atan2(end[1] - start[1], end[0] - start[0])
            # Rotate at the waypoint slowly enough for the odometry jump checks.
            self.advance(0.6)
            self.odom(start[0], start[1], yaw)
            self.triplet()
            self.node._tick()
            steps = math.ceil(math.dist(start, end) / 0.5)
            for step in range(1, steps + 1):
                fraction = step / steps
                last_position = (start[0] + fraction * (end[0] - start[0]),
                                 start[1] + fraction * (end[1] - start[1]), yaw)
                self.advance(0.2)
                self.odom(*last_position)
                self.triplet()
                self.node._tick()
                if self.node.traffic_mode == "normal":
                    completed = True
                    break
            if completed:
                break
        self.assertTrue(completed)
        self.assertIs(self.node.reference, reference)
        self.assertAlmostEqual(self.node.progress, expected_exit)
        self.assertFalse(self.node.matched)
        self.assertIsNone(self.node.detour_tracker)
        self.assertEqual(self.node.used_seq, self.node.point_seq)
        self.assert_stopped()
        count = self.output_count()
        self.advance(0.21)
        self.odom(*last_position)
        self.node._tick()
        self.assertEqual(self.output_count(), count)
        moving = self.command()
        self.node._on_controller_cmd(moving)
        self.assert_stopped()
        self.triplet()
        self.node._tick()
        self.assertEqual(self.output_count(), count + 1)
        self.node._on_controller_cmd(moving)
        self.assertEqual(self.twist_values(self.node.cmd_pub.messages[-1]), self.twist_values(moving))

    def test_final_reference_uses_official_node_shape_even_without_traffic_control(self):
        baseline = self.complete_official_lap()
        self.assertTrue(baseline.fusion_status["available"], baseline.fusion_status)
        self.assertFalse(baseline.p["enable_traffic_control"])
        self.assertIsNone(baseline.cmd_pub)
        baseline_points = [baseline.reference.point_at(baseline.reference.length_m * i / 200)[:2]
                           for i in range(200)]
        changed_id = baseline.official.node_ids[1]
        changed = self.complete_official_lap(
            official_transform=lambda key, point: (point[0] + 1.25, point[1] - 0.75)
            if key == changed_id else point,
        )
        self.assertTrue(changed.fusion_status["available"], changed.fusion_status)
        changed_points = [changed.reference.point_at(changed.reference.length_m * i / 200)[:2]
                          for i in range(200)]
        self.assertGreater(max(math.dist(a, b) for a, b in zip(baseline_points, changed_points)), 0.01,
                           "The official shape must alter the final reference, not only diagnostics")
        changed._publish_status()
        status = json.loads(changed.status_pub.messages[-1].data)
        self.assertEqual(status["phase"], "guarding")
        self.assertTrue(status["map_fusion"]["available"])

    def test_global_official_scale_does_not_resize_the_recorded_odom_route(self):
        baseline = self.complete_official_lap()
        self.assertTrue(baseline.fusion_status["available"], baseline.fusion_status)
        baseline_points = [baseline.reference.point_at(baseline.reference.length_m * i / 200)[:2]
                           for i in range(200)]
        # This is intentionally outside the former [0.5, 2] scale gate.
        scaled = self.complete_official_lap(
            official_transform=lambda key, point: (3.0 * point[0], 3.0 * point[1]),
        )
        self.assertTrue(scaled.fusion_status["available"], scaled.fusion_status)
        scaled_points = [scaled.reference.point_at(scaled.reference.length_m * i / 200)[:2]
                         for i in range(200)]
        self.assertAlmostEqual(scaled.reference.length_m, baseline.reference.length_m, delta=0.02)
        self.assertLess(max(math.dist(a, b) for a, b in zip(baseline_points, scaled_points)), 0.02)
        for key in baseline.fused_node_positions:
            self.assertLess(math.dist(baseline.fused_node_positions[key], scaled.fused_node_positions[key]), 0.02)

    def test_node_progress_and_detour_endpoints_use_final_fused_anchors(self):
        changed_id = self.node.official.node_ids[1]
        self.learned_official_loop(
            official_transform=lambda key, point: (point[0] + 1.25, point[1] - 0.75)
            if key == changed_id else point,
        )
        self.assertGreater(max(
            math.dist(position, self.node.official_to_odom.point(*self.node.topology.nodes[key]))
            for key, position in self.node.fused_node_positions.items()
        ), 0.05, "This fixture must distinguish fused anchors from a plain similarity transform")
        for key, progress in self.node.fused_node_progress.items():
            self.assertEqual(self.node._node_progress(key), progress)
            self.assertLess(math.dist(self.node.reference.point_at(progress)[:2],
                                      self.node.fused_node_positions[key]), 1e-7)
            self.assertEqual(self.node.mapped_topology.nodes[key], self.node.fused_node_positions[key])
        # A stored anchor is authoritative even when it differs from the old
        # phase/fraction calculation. Restore it before exercising detour.
        key = self.node.official.node_ids[1]
        progress = self.node.fused_node_progress[key]
        self.node.fused_node_progress[key] = progress + 1.234
        self.assertEqual(self.node._node_progress(key), progress + 1.234)
        self.node.fused_node_progress[key] = progress
        self.confirm_light("red")
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, "detour")
        route = self.node.detour_node_ids
        self.assertEqual(self.node.detour_tracker.points[0], self.node.fused_node_positions[route[0]])
        self.assertEqual(self.node.detour_tracker.points[-1], self.node.fused_node_positions[route[-1]])
        self.assertEqual(list(self.node.detour_tracker.points),
                         [self.node.mapped_topology.nodes[key] for key in route])

    def test_rejected_map_correspondence_is_odom_only_and_cannot_correct_or_detour(self):
        self.traffic_node()
        self.learn_circle()
        self.assertFalse(self.node.fusion_status["available"])
        self.assertEqual(self.node.fused_node_positions, {})
        self.assertEqual(self.node.fused_node_progress, {})
        self.assertIsNone(self.node.mapped_topology)
        for _ in range(8):
            points = self.conflicting_frame()
            self.assert_last_points_equal(points)
            self.assertEqual(self.node.last_correction, 0.0)
            self.assertEqual(self.node.guard.violation_count, 0)
            self.assertEqual(self.node.reason, "map_fusion_unavailable_passthrough")
        self.node._publish_status()
        status = json.loads(self.node.status_pub.messages[-1].data)
        self.assertEqual(status["phase"], "odom_only")
        self.assertFalse(status["map_fusion"]["available"])
        self.assertTrue(status["map_fusion"]["reason"])
        self.confirm_light("red")
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, "wait")
        self.assertEqual(self.node.traffic_reason, "red_wait_map_fusion")
        self.assertIsNone(self.node.detour_tracker)
        self.assert_stopped()

    def test_reset_after_completed_detour_does_not_ignore_a_later_red(self):
        self.learned_official_loop()
        self.confirm_light("red")
        self.node._tick()
        self.complete_active_detour()
        self.node._on_reset(DataMessage(True))
        self.assertIsNone(self.node.reference)
        self.assertEqual(self.node.fused_node_positions, {})
        self.assertEqual(self.node.fused_node_progress, {})
        self.assertIsNone(self.node.mapped_topology)
        self.confirm_light("red")
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, "wait")
        self.assertEqual(self.node.traffic_reason, "red_wait_first_lap")
        self.assert_stopped()

    def test_completed_detour_needs_no_green_before_later_red_can_stop(self):
        self.learned_official_loop()
        self.confirm_light("red")
        self.node._tick()
        self.complete_active_detour()
        self.assertEqual(self.node.traffic_mode, "normal")
        self.assertEqual(self.node.traffic_light, "unknown")
        self.advance(self.node.p["resume_delay_sec"] + 0.01)
        self.odom(*self.node.odom)
        self.triplet()
        self.node._tick()
        self.node._on_controller_cmd(self.command())
        self.assertGreater(self.node.cmd_pub.messages[-1].linear.x, 0.0)
        self.confirm_light("red")
        self.assertEqual(self.node.traffic_mode, "wait", "A later confirmed red must revoke motion without a green event")
        self.assert_stopped()

    def test_expired_red_during_detour_does_not_start_another_detour_at_exit(self):
        self.learned_official_loop()
        self.confirm_light("red")
        self.node._tick()
        self.advance(0.6)
        self.emit_detour_completion()
        self.assertEqual(self.node.traffic_mode, "normal")
        self.assertEqual(self.node.traffic_light, "unknown")
        self.assertIsNone(self.node.detour_tracker)
        self.advance(0.21)
        self.odom(*self.node.odom)
        self.triplet()
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, "normal")
        self.assertIsNone(self.node.detour_tracker)

    def test_fresh_red_during_detour_is_enforced_at_completion_without_green_event(self):
        self.learned_official_loop()
        self.confirm_light("red")
        self.node._tick()
        self.confirm_light("red")
        self.assertEqual(self.node.traffic_mode, "detour", "Light changes cannot abandon a committed route")
        self.emit_detour_completion()
        self.assertEqual(self.node.traffic_mode, "wait", "Fresh confirmed red remains effective at the exit")
        self.assertEqual(self.node.traffic_reason, "red_after_detour")
        self.assertIsNone(self.node.detour_tracker)
        self.node._on_controller_cmd(self.command())
        self.assert_stopped()

    def test_one_red_frame_or_timer_repetition_does_not_stop_normal_motion(self):
        self.traffic_node()
        self.ready_visual_motion()
        self.node._on_controller_cmd(self.command())
        self.node._on_light_state(DataMessage("red"))
        for _ in range(6):
            self.advance(0.1)
            self.odom(*self.node.odom)
            self.triplet()
            self.node._tick()
            self.node._on_controller_cmd(self.command())
        self.assertEqual(self.node.traffic_mode, "normal")
        self.assertEqual(self.node.traffic_light, "unknown")
        self.assertGreater(self.node.cmd_pub.messages[-1].linear.x, 0.0)

    def test_wait_survives_one_green_frame_and_detection_stream_loss(self):
        self.traffic_node()
        self.ready_visual_motion()
        self.confirm_light("red")
        self.node._on_light_state(DataMessage("green"))
        self.advance(0.6)
        self.odom(*self.node.odom)
        self.triplet()
        self.node._tick()
        self.node._on_controller_cmd(self.command())
        self.assertEqual(self.node.traffic_light, "unknown")
        self.assertEqual(self.node.traffic_mode, "wait")
        self.assert_stopped()

    def test_stale_green_cannot_release_detour_on_explicit_or_odom_reset(self):
        for reset_kind in ("explicit", "odom"):
            with self.subTest(reset_kind=reset_kind):
                self.learned_official_loop()
                self.confirm_light("red")
                self.node._tick()
                self.confirm_light("green")
                self.assertEqual(self.node.traffic_light, "green")
                self.advance(0.6)
                if reset_kind == "explicit":
                    self.node._on_reset(DataMessage(True))
                else:
                    x, y, yaw = self.node.odom
                    self.odom(x + 100.0, y, yaw)
                self.assertEqual(self.node.traffic_light, "unknown")
                self.assertEqual(self.node.traffic_mode, "wait")
                self.assertIsNone(self.node.detour_tracker)
                self.assertIsNone(self.node.reference)
                self.node._on_controller_cmd(self.command())
                self.assert_stopped()

    def test_status_reports_expired_vote_without_changing_control_state(self):
        self.traffic_node()
        self.ready_visual_motion()
        self.confirm_light("red")
        self.assertEqual(self.node.traffic_mode, "wait")
        self.advance(0.6)
        before_points = self.output_count()
        before_commands = len(self.node.cmd_pub.messages)
        with patch.object(self.node, "_update_light_vote") as update:
            self.node._status_tick()
        update.assert_not_called()
        status = json.loads(self.node.status_pub.messages[-1].data)
        self.assertEqual(status["traffic_light"], "unknown")
        self.assertEqual(status["traffic_light_raw"], "red")
        self.assertEqual(status["traffic_vote"]["sample_count"], 0)
        self.assertFalse(status["traffic_vote"]["ready"])
        self.assertEqual(self.node.traffic_mode, "wait")
        self.assertEqual(self.output_count(), before_points)
        self.assertEqual(len(self.node.cmd_pub.messages), before_commands)


if __name__ == "__main__":
    unittest.main()
