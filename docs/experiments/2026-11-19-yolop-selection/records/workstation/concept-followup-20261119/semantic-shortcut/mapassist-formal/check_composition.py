"""Check launch composition without executing any vehicle actions."""
import importlib.util
from pathlib import Path
import sys

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

path = Path(sys.argv[1])
spec = importlib.util.spec_from_file_location('formal_launch_check',path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
context = LaunchContext()
context.launch_configurations['lane']='outer'
def resolve(value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))

arguments = [
    {resolve(key):resolve(value)
     for key,value in include.launch_arguments}
    for include in module.generate_launch_description().entities if isinstance(include,IncludeLaunchDescription)
]
assert arguments == [
    {'motor_cmd_topic':'/semantic_drive/map_assist_045/diagnostic_cmd_vel','launch_yolop_a3':'false'},
    {'lane':'outer','vehicle_output':'true','start_lya':'false'}
], arguments
print('Combined launch arguments:', arguments)

if "--occupied" in sys.argv:
    import rclpy
    rclpy.init()
    fake = rclpy.create_node("motor_controller")
    try:
        try:
            module.check_owners(context)
        except RuntimeError as error:
            assert "motor_controller" in str(error), error
            print("Existing motor owner rejected without executing vehicle actions.")
        else:
            raise AssertionError("Duplicate motor was not detected")
    finally:
        fake.destroy_node()
        rclpy.shutdown()
