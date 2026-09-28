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


def runtime_defaults(source, profile_override=None):
    """Evaluate the actual production parameter dictionary, without its node."""
    namespace = profile_values()
    if profile_override is not None:
        namespace['REFERENCE_SPEED_MPS'] = profile_override
    candidates = []
    for node in ast.walk(read_tree(source)):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if (isinstance(key, ast.Constant) and key.value == 'reference_speed_mps'
                        and isinstance(value, ast.Name) and value.id == 'REFERENCE_SPEED_MPS'):
                    candidates.append(node)
    assert len(candidates) == 1, str(source)
    return eval(compile(ast.Expression(candidates[0]), str(source), 'eval'), namespace)


def subscript_key(node):
    """Normalize the Index wrapper used by Foxy's Python 3.8 AST."""
    value = node.slice
    if isinstance(value, ast.Index):
        value = value.value
    return value.value if isinstance(value, ast.Constant) else None


def merge_simple_parameter_fixture(node, declared_defaults):
    """Model only file wildcard/exact-node and later-dict precedence.

    This deliberately small offline fixture is NOT the ROS parameter parser.
    It checks the actual launch action ordering and does not claim transport or
    all ROS selector/type semantics have been exercised.
    """
    merged = dict(declared_defaults)
    for item in node.parameters:
        if isinstance(item, dict):
            merged.update(item)
        else:
            document = yaml.safe_load(Path(item).read_text(encoding='utf-8'))
            for selector in ('/**', node.name, '/' + node.name):
                merged.update(document.get(selector, {}).get('ros__parameters', {}))
    return merged


def production_teacher_speed_argument(parameters):
    """Execute actual supervisor argv assembly; never spawn the process."""
    source = SHARED / 'supervisor.py'
    tree = read_tree(source)
    owner = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                 and node.name == 'TeacherSupervisor')
    init = next(node for node in owner.body if isinstance(node, ast.FunctionDef)
                and node.name == '__init__')
    statements = []
    for node in init.body:
        if (isinstance(node, ast.Assign)
                and any(isinstance(target, ast.Name) and target.id == 'command'
                        for target in node.targets)):
            statements.append(node)
        if (isinstance(node, ast.If) and isinstance(node.test, ast.Compare)
                and isinstance(node.test.left, ast.Name)
                and node.test.left.id == 'executable'):
            statements.append(node)
    assert len(statements) == 2
    namespace = dict(binary=Path('/inert-fixture/lya_0221'),
                     executable='lya_0221', value=parameters.__getitem__)
    exec(compile(ast.Module(body=statements, type_ignores=[]), str(source), 'exec'), namespace)
    return next(item for item in namespace['command'] if item.startswith('reference_speed_mps:='))


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


def test_direct_recorder_topics_match_neo_launch_configuration():
    """Raw/common and CameraInfo are not keys in neo's topic_list.yaml."""
    direct = runtime_defaults(SHARED / 'recorder_node.py')
    shared = launch_namespace(SHARED / 'launch_support.py',
                              ['_boolean', '_nodes', 'generate_learning_launch'])
    for mode in ('fixed_only', 'lya_reference'):
        launch_defaults = defaults_from(shared['generate_learning_launch'](mode))
        package = 'lane_mapping_fixed' if mode == 'fixed_only' else 'lane_mapping_lya_reference'
        config = yaml.safe_load((CONTROL / package / 'config/learning.yaml').read_text(encoding='utf-8'))
        configured = config['lane_lap_recorder']['ros__parameters']
        for topic in ('vectornav_topic', 'camera_info_topic', 'mask_topic'):
            assert direct[topic] == launch_defaults[topic] == configured[topic]
    # Ground the common namespace in the real driver launch and topic list.
    vectornav = launch_namespace(AIFORMULA / 'launchers/launch/vectornav.launch.py',
                                 ['generate_launch_description'])
    driver = next(node for node in vectornav['generate_launch_description']()
                  if node.executable == 'vectornav')
    prefix = driver.namespace.rstrip('/') + '/' + driver.name
    assert topics()['sensing']['vectornav']['imu'] == prefix + '/imu'
    assert direct['vectornav_topic'] == prefix + '/raw/common'
    # The ZED camera_info convention is checked against the deployed node
    # namespace/name, not falsely attributed to a missing topic-list entry.
    zed_namespace = launch_namespace(ALLNODES, ['create_zed_node'])
    zed_namespace.update(check_zedx_available_fps=lambda *args: True,
                         IfCondition=action('IfCondition'))
    zed, = zed_namespace['create_zed_node'](
        dict(grab_resolution='HD1080', grab_frame_rate='15', pub_downscale_factor='3.0'))
    assert direct['camera_info_topic'] == zed.namespace.rstrip('/') + '/' + zed.name + '/left/camera_info'


@pytest.mark.parametrize('mode', ['fixed_only', 'lya_reference', 'gnss'])
@pytest.mark.parametrize('launch_override,expected', [('', 4.0), ('1.75', 1.75)])
def test_wildcard_yaml_reference_and_explicit_launch_precedence_offline(tmp_path, mode, launch_override, expected):
    """Synthetic merge model: /** sets 4; only an explicit launch arg wins."""
    config = tmp_path / 'reference_override.yaml'
    config.write_text('/**:\n  ros__parameters:\n    reference_speed_mps: 4.0\n', encoding='utf-8')
    if mode == 'gnss':
        namespace = launch_namespace(GNSS / 'launch/fixed_gnss.launch.py',
                                     ['nodes', 'generate_launch_description'])
        defaults = defaults_from(namespace['generate_launch_description']())
        actions = namespace['nodes'](dict(defaults, params_file=str(config),
                                          reference_speed_mps=launch_override))
    else:
        namespace = launch_namespace(SHARED / 'launch_support.py',
                                     ['_boolean', '_nodes', 'generate_learning_launch'])
        defaults = defaults_from(namespace['generate_learning_launch'](mode))
        actions = namespace['_nodes'](dict(defaults, params_file=str(config),
                                           reference_speed_mps=launch_override), mode)
    sources = {
        'lane_fixed_follower': SHARED / 'follower_node.py',
        'lane_lap_recorder': SHARED / 'recorder_node.py',
        'lane_teacher_supervisor': SHARED / 'supervisor.py',
        'lane_gnss_teacher_supervisor': SHARED / 'supervisor.py',
        'lane_endpoint_follower': GNSS / 'lane_mapping_fixed_gnss/node.py',
    }
    nodes = [item for item in actions if item.kind == 'Node']
    assert len(nodes) == (2 if mode == 'gnss' else 3)
    for node in nodes:
        merged = merge_simple_parameter_fixture(node, runtime_defaults(sources[node.name]))
        assert merged['reference_speed_mps'] == expected
        if node.executable == 'teacher_supervisor':
            # The actual argv-producing statements must carry the same value
            # to the managed LYA child, not just retain it inside the supervisor.
            assert production_teacher_speed_argument(merged) == 'reference_speed_mps:=' + str(expected)


@pytest.mark.parametrize('source,class_name,method', [
    (SHARED / 'follower_node.py', 'FixedFollower', '__init__'),
    (GNSS / 'lane_mapping_fixed_gnss/node.py', 'EndpointFollower', '_validate'),
])
@pytest.mark.parametrize('profile,override,explicit_cap,expected', [
    (4.0, None, 0.0, 4.0), (4.0, 1.75, 0.0, 1.75),
    (4.0, 1.75, 2.5, 2.5),
])
def test_actual_speed_cap_initialization_inherits_effective_reference(
        source, class_name, method, profile, override, explicit_cap, expected):
    defaults = runtime_defaults(source, profile_override=profile)
    assert defaults['maximum_speed_mps'] == 0.0
    params = dict(defaults, maximum_speed_mps=explicit_cap)
    if override is not None:
        params['reference_speed_mps'] = override
    owner = next(node for node in read_tree(source).body
                 if isinstance(node, ast.ClassDef) and node.name == class_name)
    function = next(node for node in owner.body
                    if isinstance(node, ast.FunctionDef) and node.name == method)
    # Execute the exact production if-statement, including its guard and RHS.
    # Other initialization may create ROS entities and is intentionally omitted.
    candidates = [node for node in function.body if isinstance(node, ast.If)
                  and isinstance(node.test, ast.Compare)
                  and isinstance(node.test.left, ast.Subscript)
                  and subscript_key(node.test.left) == 'maximum_speed_mps'
                  and any(isinstance(value, ast.Constant) and value.value == 0.0
                          for value in node.test.comparators)]
    assert len(candidates) == 1
    node = SimpleNamespace(p=params)
    exec(compile(ast.Module(body=candidates, type_ignores=[]), str(source), 'exec'), {'self': node})
    assert node.p['maximum_speed_mps'] == expected
