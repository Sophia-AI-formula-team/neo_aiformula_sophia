"""Run only production driver & gate from the installed launch's selected parameters."""
import runpy
from unittest.mock import patch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from concept import bringup

REAL_CAN = '/aiformula_control/motor_controller/reference_signal'


def settings_nodes(context):
    assert context.launch_configurations['onboard_camera'] == 'false'
    isolated = []
    def select(**params):
        if params['package'] in ('concept_drive', 'concept_control_gate', 'concept_vehicle_state','motor_controller','concept'):
            isolated.append(Node(**params))
        return None
    with patch.object(bringup, 'Node', select):
        bringup.build(context)
    assert context.launch_configurations['mode'] == 'replay' and len(isolated) == 5
    return isolated


def generate_launch_description():
    normal=runpy.run_path(bringup.share('concept','launch','concept.launch.py'))
    declarations=[a for a in normal['generate_launch_description']().entities if isinstance(a,DeclareLaunchArgument)]
    return LaunchDescription(declarations+[OpaqueFunction(function=settings_nodes)])
