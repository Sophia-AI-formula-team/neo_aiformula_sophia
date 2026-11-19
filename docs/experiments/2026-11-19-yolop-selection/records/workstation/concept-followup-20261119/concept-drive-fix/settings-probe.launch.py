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
        if params['package'] in ('concept_drive', 'concept_control_gate'):
            if params['package'] == 'concept_control_gate':
                others = [(src,dst) for src,dst in params.get('remappings', []) if src != REAL_CAN]
                params['remappings'] = others + [(REAL_CAN, '/settings_probe/unused_can')]
            isolated.append(Node(**params))
        return None
    with patch.object(bringup, 'Node', select):
        bringup.build(context)
    assert len(isolated) == 2
    return isolated


def generate_launch_description():
    normal=runpy.run_path(bringup.share('concept','launch','concept.launch.py'))
    declarations=[a for a in normal['generate_launch_description']().entities if isinstance(a,DeclareLaunchArgument)]
    return LaunchDescription(declarations+[OpaqueFunction(function=settings_nodes)])
