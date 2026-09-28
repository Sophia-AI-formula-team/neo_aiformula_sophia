"""Offline source contracts against neo's actual launch/topic definitions.

Launch functions execute with inert action doubles: no ROS, processes or vehicle
nodes are started. These checks complement, not replace, native DDS validation.
"""

import ast
import copy
import math
import os.path as osp
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONTROL = PACKAGE.parent
AIFORMULA = CONTROL.parent
SHARED = PACKAGE / 'lane_mapping_lya_reference'
GNSS = CONTROL / 'lane_mapping_fixed_gnss'
PROFILE = CONTROL / 'trajectory_follower/trajectory_follower/lya_profile.py'
TOPICS = AIFORMULA / 'launchers/config/topic_list.yaml'
ALLNODES = AIFORMULA / 'launchers/launch/allnodes.launch.py'


def read_tree(path):
    return ast.parse(path.read_text(encoding='utf-8'), filename=str(path))


def profile_values():
    return {
        target.id: ast.literal_eval(node.value)
        for node in read_tree(PROFILE).body if isinstance(node, ast.Assign)
        for target in node.targets if isinstance(target, ast.Name)
    }


def topics():
    return yaml.safe_load(TOPICS.read_text(encoding='utf-8'))


def action(kind):
    def create(*args, **kwargs):
        return SimpleNamespace(kind=kind, positional_args=args, **kwargs)
    return create


def launch_namespace(path, function_names, profile_override=None):
    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    namespace = dict(profile_values())
    if profile_override is not None:
        namespace['REFERENCE_SPEED_MPS'] = profile_override
    namespace.update(
        math=math, Path=Path, osp=osp,
        get_package_share_directory=lambda package: str(CONTROL / package),
        get_frame_ids_and_topic_names=lambda: ({}, topics()),
        LaunchConfiguration=Configuration,
        LaunchDescription=lambda actions: actions,
    )
    for name in ('Node', 'DeclareLaunchArgument', 'OpaqueFunction',
                 'RegisterEventHandler', 'OnProcessExit', 'EmitEvent', 'Shutdown'):
        namespace[name] = action(name)
    selected = [copy.deepcopy(node) for node in read_tree(path).body
                if isinstance(node, ast.FunctionDef) and node.name in function_names]
    assert {node.name for node in selected} == set(function_names)
    for node in selected:
        node.returns = None
        for arg in node.args.args:
            arg.annotation = None
    module = ast.Module(body=selected, type_ignores=[])
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace


def defaults_from(actions):
    return {item.positional_args[0]: item.default_value for item in actions
            if item.kind == 'DeclareLaunchArgument'}


def node_overrides(nodes):
    return [node.parameters[-1] for node in nodes if node.kind == 'Node']


def test_actual_neo_road_detector_remaps_the_full_mask_topic():
    namespace = launch_namespace(ALLNODES, ['create_road_detector_node'])
    node, = namespace['create_road_detector_node']({})
    assert node.package == 'road_detector'
    assert node.executable == 'road_detector'
    remaps = dict(node.remappings)
    assert remaps['pub_mask_image'] == topics()['perception']['mask_image']
    assert remaps['pub_mask_image'] == '/aiformula_perception/road_detector/mask_image'
    assert remaps['sub_image'] == topics()['sensing']['zedx']['left_image']['undistorted']
    assert 'roi' not in remaps['pub_mask_image']


@pytest.mark.parametrize('package,filename', [
    ('lane_mapping_lya_reference', 'learning.yaml'),
    ('lane_mapping_fixed', 'learning.yaml'),
    ('lane_mapping_fixed_gnss', 'runtime.yaml'),
])
def test_yaml_defaults_use_actual_mask_and_lya_reference(package, filename):
    entries = yaml.safe_load((CONTROL / package / 'config' / filename).read_text(encoding='utf-8'))
    mask_users = 0
    for entry in entries.values():
        params = entry['ros__parameters']
        if 'mask_topic' in params:
            mask_users += 1
            assert params['mask_topic'] == topics()['perception']['mask_image']
        if 'reference_speed_mps' in params:
            pytest.fail('Default YAML must inherit the LYA reference, not copy it')
        if 'teacher_executable' in params:
            assert params['teacher_executable'] == 'lya_0221'
        if 'command_output_topic' in params:
            assert 'maximum_speed_mps' not in params
            assert params['command_output_topic'].startswith('/lane_learning')
            assert params['enable_vehicle_output'] is False
            assert params['hardware_stop_verified'] is False
            assert params['motor_zero_passthrough_verified'] is False
    assert mask_users >= 1


@pytest.mark.parametrize('mode', ['fixed_only', 'lya_reference'])
def test_shared_launch_passes_one_reference_to_recorder_follower_and_teacher(mode):
    namespace = launch_namespace(SHARED / 'launch_support.py',
                                 ['_boolean', '_nodes', 'generate_learning_launch'])
    defaults = defaults_from(namespace['generate_learning_launch'](mode))
    assert defaults['teacher_executable'] == 'lya_0221'
    assert defaults['mask_topic'] == topics()['perception']['mask_image']
    assert defaults['reference_speed_mps'] == ''
    for reference in ('', '1.75'):
        context = dict(defaults, reference_speed_mps=reference)
        overrides = node_overrides(namespace['_nodes'](context, mode))
        assert len(overrides) == 3
        if reference:
            assert all(item['reference_speed_mps'] == float(reference) for item in overrides)
        else:
            assert all('reference_speed_mps' not in item for item in overrides)
        assert all(item.get('mask_topic', defaults['mask_topic']) == defaults['mask_topic']
                   for item in overrides)
        assert overrides[0]['safety_mode'] == mode
        assert overrides[0]['enable_vehicle_output'] is False
        assert overrides[-1]['teacher_executable'] == 'lya_0221'


def test_gnss_launch_passes_same_reference_to_follower_and_private_teacher():
    namespace = launch_namespace(GNSS / 'launch/fixed_gnss.launch.py',
                                 ['nodes', 'generate_launch_description'])
    defaults = defaults_from(namespace['generate_launch_description']())
    assert defaults['reference_speed_mps'] == ''
    for reference in ('', '1.75'):
        actions = namespace['nodes'](dict(defaults, reference_speed_mps=reference))
        nodes = [item for item in actions if item.kind == 'Node']
        overrides = node_overrides(actions)
        assert len(overrides) == 2
        assert all(defaults['params_file'] in node.parameters for node in nodes)
        if reference:
            assert all(item['reference_speed_mps'] == float(reference) for item in overrides)
        else:
            assert all('reference_speed_mps' not in item for item in overrides)
        assert overrides[-1]['teacher_executable'] == 'lya_0221'
        assert overrides[-1]['teacher_command_topic'] == '/lane_learning/lya_cmd'
        assert overrides[0]['enable_vehicle_output'] is False


@pytest.mark.parametrize('source', [
    SHARED / 'recorder_node.py', SHARED / 'follower_node.py',
    SHARED / 'supervisor.py', GNSS / 'lane_mapping_fixed_gnss/node.py',
    CONTROL / 'trajectory_follower/trajectory_follower/lya_0221.py',
])
def test_runtime_default_imports_the_single_lya_profile(source):
    tree = read_tree(source)
    imports = [node for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
               and node.module in ('lya_profile', 'trajectory_follower.lya_profile')]
    assert any(alias.name == 'REFERENCE_SPEED_MPS'
               for node in imports for alias in node.names), str(source)
    # Runtime parameter dictionaries must not acquire an unrelated hard-coded
    # reference. YAML snapshots are checked against the source constant above.
    default_expressions = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value == 'reference_speed_mps':
                    assert not isinstance(value, ast.Constant), str(source)
                    if isinstance(value, ast.Name) and value.id == 'REFERENCE_SPEED_MPS':
                        default_expressions.append(value)
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter' and len(node.args) >= 2
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == 'reference_speed_mps'):
            default_expressions.append(node.args[1])
    assert default_expressions, 'No inherited runtime reference default checked in ' + str(source)
    for expression in default_expressions:
        # A future LYA-profile edit to 4 m/s must flow into the actual default
        # expression; the test neither edits production nor requests driving.
        assert eval(compile(ast.Expression(expression), str(source), 'eval'),
                    {'REFERENCE_SPEED_MPS': 4.0}) == 4.0


def test_profile_and_lya_runtime_dependencies_are_declared():
    values = profile_values()
    assert set(values) == {'REFERENCE_SPEED_MPS', 'MAX_YAW_RATE_RPS'}
    assert all(math.isfinite(value) and value > 0 for value in values.values())
    dependencies = lambda package: {
        child.text for child in ET.parse(CONTROL / package / 'package.xml').getroot()
        if child.tag in ('depend', 'exec_depend')
    }
    assert 'trajectory_follower' in dependencies('lane_mapping_lya_reference')
    assert 'lane_mapping_lya_reference' in dependencies('lane_mapping_fixed')
    assert 'lane_mapping_lya_reference' in dependencies('lane_mapping_fixed_gnss')
    assert {'tf2_ros', 'tf2_geometry_msgs', 'tf_transformations'} <= dependencies('trajectory_follower')
    entries = [node.value.replace(' ', '')
               for node in ast.walk(read_tree(CONTROL / 'trajectory_follower/setup.py'))
               if isinstance(node, ast.Constant) and isinstance(node.value, str)]
    assert 'lya_0221=trajectory_follower.lya_0221:main' in entries


@pytest.mark.parametrize('source', [
    SHARED / 'recorder_node.py', SHARED / 'follower_node.py',
    GNSS / 'lane_mapping_fixed_gnss/node.py',
])
def test_direct_node_mask_defaults_also_match_neo(source):
    defaults = []
    for node in ast.walk(read_tree(source)):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value == 'mask_topic' and isinstance(value, ast.Constant):
                    defaults.append(value.value)
    assert defaults, 'No mask default was checked in ' + str(source)
    assert all(value == topics()['perception']['mask_image'] for value in defaults)
