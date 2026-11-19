"""Use production decision and gate nodes, physical inputs, and the software-only CAN sink."""
import os
import runpy
import shutil
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
import concept.bringup as b

def generate_launch_description():
    native = runpy.run_path(b.share('launchers','launch','allnodes.launch.py'))
    defaults = dict(can_interface='can0',receiver_interval_sec='0.1',enable_can_fd='false',use_sim_time='false',
                    grab_resolution='HD1080',grab_frame_rate=os.environ['TEST_CAMERA_HZ'],pub_downscale_factor='3.0',
                    config_common_path=b.share('vehicle','config','zedx','common.yaml'),
                    config_camera_path=b.share('vehicle','config','zedx','zedx.yaml'))
    if 'TEST_PUB_HZ' in os.environ:
        import yaml
        config=yaml.safe_load(Path(defaults['config_common_path']).read_text())
        config['/**']['ros__parameters']['general']['pub_frame_rate']=float(os.environ['TEST_PUB_HZ'])
        override=Path(os.environ['RUN_DIR'],'zed-common.yaml')
        override.write_text(yaml.safe_dump(config))
        defaults['config_common_path']=str(override)
    original_include=b.include
    def include(package, filename, **kwargs):
        if package=='concept_yolop' and os.environ.get('PROFILE_CAMERA')=='1':
            return ExecuteProcess(cmd=[shutil.which('python3'),str(Path(__file__).with_name('timed_camera.py')),
                '--ros-args','-p','calibration:='+kwargs['calibration']])
        return original_include(package,filename,**kwargs)
    b.include=include
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()] + [
        *native['create_socket_can_receiver'](), b.include('launchers','vectornav.launch.py'),
        OpaqueFunction(function=lambda ctx: [b.zed_actions(ctx,native,os.environ['RUN_DIR'])]
                       if os.environ.get('INSTALLED_ZED')=='1' else list(native['create_zed_node'](ctx))),
        Node(package='joy',executable='joy_node',name='joy_node',namespace='/aiformula_control',
             parameters=[b.share('launchers','config','joy.yaml')],remappings=[('joy','/aiformula_control/joy_node/joy')]),
        Node(package='launchers',executable='l2_throttle_joy_remap.py',name='l2_throttle_joy_remap',
             namespace='/aiformula_control',remappings=[('joy_in','/aiformula_control/joy_node/joy'),('joy_out','/aiformula_control/joy_node/joy_l2_throttle')]),
        b.include('concept','concept.launch.py',mode='replay',onboard_camera=os.environ.get('TEST_DIRECT','false'),panel=os.environ.get('TEST_PANEL','false'),rviz=os.environ.get('TEST_RVIZ','false'),
                  calibration=str(Path.home()/'.config/concept/honda-ground.json'))])
