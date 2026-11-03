"""Expand the real map_drive launch using stubs; never starts ROS or vehicles."""

import ast
import importlib.util
from pathlib import Path
import sys
import types
import unittest


class LaunchDescription:
    def __init__(self, entities):
        self.entities = entities


class DeclareLaunchArgument:
    def __init__(self, name, default_value=None, **kwargs):
        self.name, self.default_value = name, default_value


class OpaqueFunction:
    def __init__(self, function):
        self.function = function


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return str(context[self.name])


class FindPackageShare:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return "/installed/share/" + self.name


class PathJoinSubstitution:
    def __init__(self, parts):
        self.parts = parts

    def perform(self, context):
        return "/".join(str(resolve(part, context)).rstrip("/") for part in self.parts)


class ParameterValue:
    def __init__(self, value, value_type=None):
        self.value, self.value_type = value, value_type


class Node:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


def resolve(value, context):
    if isinstance(value, ParameterValue):
        value = resolve(value.value, context)
        return value
    if hasattr(value, "perform"):
        return value.perform(context)
    return value


class MapDriveLaunchTests(unittest.TestCase):
    def setUp(self):
        self.path = Path(__file__).resolve().parents[1] / "launch" / "map_drive.launch.py"
        modules = {name: types.ModuleType(name) for name in (
            "launch", "launch.actions", "launch.substitutions", "launch_ros",
            "launch_ros.actions", "launch_ros.parameter_descriptions", "launch_ros.substitutions",
        )}
        modules["launch"].LaunchDescription = LaunchDescription
        modules["launch.actions"].DeclareLaunchArgument = DeclareLaunchArgument
        modules["launch.actions"].OpaqueFunction = OpaqueFunction
        modules["launch.substitutions"].LaunchConfiguration = LaunchConfiguration
        modules["launch.substitutions"].PathJoinSubstitution = PathJoinSubstitution
        modules["launch_ros.actions"].Node = Node
        modules["launch_ros.parameter_descriptions"].ParameterValue = ParameterValue
        modules["launch_ros.substitutions"].FindPackageShare = FindPackageShare
        for name, module in modules.items():
            if "." in name:
                parent, child = name.rsplit(".", 1)
                setattr(modules[parent], child, module)
        missing = object()
        previous = {name: sys.modules.get(name, missing) for name in modules}

        def restore():
            for name, value in previous.items():
                if value is missing:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = value

        sys.modules.update(modules)
        self.addCleanup(restore)
        spec = importlib.util.spec_from_file_location("map_drive_launch_under_test", self.path)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def expand(self, **overrides):
        description = self.module.generate_launch_description()
        context = {}
        nodes = []
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                context[entity.name] = overrides.get(entity.name, resolve(entity.default_value, context))
            elif isinstance(entity, OpaqueFunction):
                nodes.extend(entity.function(context))
            elif isinstance(entity, Node):
                nodes.append(entity)
            else:
                self.fail(f"Unexpected launch action: {type(entity).__name__}")
        return nodes, context

    def test_default_enables_exactly_the_original_chain_and_map_assist(self):
        nodes, context = self.expand()
        self.assertEqual([(node.kwargs["package"], node.kwargs["executable"]) for node in nodes], [
            ("lane_points", "lane_0529oa"),
            ("kalman_filter", "withoutkalman_0312"),
            ("trajectory_follower", "lya_follower_connected_omegat_global"),
            ("semantic_local_planner_ros", "map_assist"),
        ])
        # Legacy nodes keep their own internal node names and parameters.
        self.assertTrue(all("name" not in node.kwargs for node in nodes[:3]))
        self.assertEqual(nodes[0].kwargs["remappings"], [
            (f"/processed_point_{label}", f"/original_processed_point_{label}") for label in "abc"
        ])
        self.assertFalse(nodes[1].kwargs.get("remappings"))
        self.assertEqual(nodes[2].kwargs["remappings"], [
            ("/aiformula_control/game_pad/cmd_vel", "/map_assist/controller_cmd_vel")
        ])
        parameters = nodes[3].kwargs["parameters"]
        self.assertEqual(parameters[0], "/installed/share/semantic_local_planner_ros/config/map_assist.yaml")
        values = parameters[1]
        self.assertTrue(values["enable_traffic_control"])
        self.assertTrue(values["enable_velocity_gate"])
        self.assertTrue(values["enable_correction"])
        self.assertFalse(values["avoidance_initially_idle"])
        self.assertEqual(values["input_cmd_vel_topic"], "/map_assist/controller_cmd_vel")
        self.assertEqual(values["output_cmd_vel_topic"], "/aiformula_control/game_pad/cmd_vel")
        self.assertEqual(resolve(values["traffic_light_topic"], context), "/map_assist/traffic_light_input")
        self.assertEqual(resolve(values["topomap_path"], context),
                         "/installed/share/semantic_local_planner_ros/config/aiformula_topomap.osm")
        for label in "abc":
            self.assertEqual(values[f"input_point_{label}_topic"], f"/original_processed_point_{label}")
            self.assertEqual(values[f"output_point_{label}_topic"], f"/processed_point_{label}")

    def test_disabled_mode_has_three_original_nodes_and_no_remapping_or_map_parameters(self):
        nodes, _ = self.expand(use_map_assist="false")
        self.assertEqual([node.kwargs["executable"] for node in nodes], [
            "lane_0529oa", "withoutkalman_0312", "lya_follower_connected_omegat_global",
        ])
        for node in nodes:
            self.assertFalse(node.kwargs.get("remappings"))
            self.assertNotIn("parameters", node.kwargs)

    def test_blank_route_argument_preserves_configured_or_automatic_outer_route(self):
        nodes, context = self.expand(params_file="/vehicle/config/custom.yaml", route_node_ids=" ")
        parameters = nodes[-1].kwargs["parameters"]
        self.assertEqual(parameters[0], "/vehicle/config/custom.yaml")
        self.assertNotIn("route_node_ids", parameters[1])
        self.assertNotIn("detour_node_ids", parameters[1])
        self.assertNotIn("detour_node_ids", context)

    def test_explicit_overrides_are_forwarded_as_typed_strings_and_boolean(self):
        nodes, context = self.expand(
            enable_correction="false", traffic_light_topic="/detector/light_state",
            route_node_ids="0001,0002,0003,0001",
        )
        values = nodes[-1].kwargs["parameters"][1]
        self.assertFalse(values["enable_correction"])
        for name, expected in (("traffic_light_topic", "/detector/light_state"),
                               ("route_node_ids", "0001,0002,0003,0001")):
            self.assertIs(values[name].value_type, str)
            self.assertEqual(resolve(values[name], context), expected)

    def test_invalid_boolean_is_not_silently_treated_as_enabled(self):
        with self.assertRaisesRegex(ValueError, "use_map_assist"):
            self.expand(use_map_assist="typo")
        with self.assertRaisesRegex(ValueError, "enable_correction"):
            self.expand(enable_correction="typo")

    def test_initial_avoidance_idle_is_only_enabled_by_explicit_launch_argument(self):
        nodes, _ = self.expand(avoidance_initially_idle="true")
        self.assertTrue(nodes[-1].kwargs["parameters"][1]["avoidance_initially_idle"])
        with self.assertRaisesRegex(ValueError, "avoidance_initially_idle"):
            self.expand(avoidance_initially_idle="typo")

    def test_no_external_stack_process_or_avoidance_is_launched_in_either_mode(self):
        for enabled in ("true", "false"):
            with self.subTest(enabled=enabled):
                nodes, _ = self.expand(use_map_assist=enabled)
                packages = {node.kwargs["package"] for node in nodes}
                self.assertLessEqual(packages, {
                    "lane_points", "kalman_filter", "trajectory_follower", "semantic_local_planner_ros",
                })
        tree = ast.parse(self.path.read_text(encoding="utf-8"))
        imported = {alias.name for node in ast.walk(tree)
                    if isinstance(node, (ast.Import, ast.ImportFrom)) for alias in node.names}
        self.assertNotIn("IncludeLaunchDescription", imported)
        self.assertNotIn("ExecuteProcess", imported)


if __name__ == "__main__":
    unittest.main()
