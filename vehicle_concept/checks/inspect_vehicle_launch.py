#!/usr/bin/env python3
"""Construct the real sensor graph without executing a driver or opening a device."""
import runpy
from unittest.mock import patch

from launch import LaunchContext
from launch_ros.actions import Node
from concept.bringup import sensor_actions, share


def main():
    context = LaunchContext()
    context.launch_configurations.update(grab_resolution='HD1080', grab_frame_rate='15',
                                         pub_downscale_factor='3.0', use_sim_time='false')
    functions = runpy.run_path(share('launchers', 'launch', 'allnodes.launch.py'))
    functions['wait_for_can_interface'] = lambda _: ()
    with patch('concept.bringup.runpy.run_path', return_value=functions):
        actions = sensor_actions(context)
    nodes = [context.perform_substitution(n.node_package[0]) for n in actions if isinstance(n, Node)]
    assert {'zed_wrapper', 'robot_state_publisher', 'joint_state_publisher', 'joy',
            'launchers', 'teleop_twist_joy', 'ros2_socketcan'} <= set(nodes), nodes
    assert nodes.count('ros2_socketcan') == 2
    print('Constructed sensor graph and chassis without devices:', ', '.join(nodes))


if __name__ == '__main__':
    main()
