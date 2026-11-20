"""One Foxy DDS lifecycle exercise with four real launch processes, no physical device."""
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy

supervisor = str(Path(__file__).resolve().parents[1] / "oa_1b/supervise.py")
stages = ("allnodes", "yolop", "oa1b", "lya")
launch_source = """
from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():
    name = os.environ["SMOKE_STAGE"]
    return LaunchDescription([Node(package='tf2_ros', executable='static_transform_publisher',
        name='test_oa1b_'+name, arguments=['0','0','0','0','0','0','map',name])])
"""


def execute():
    with tempfile.TemporaryDirectory() as home:
        root = Path(home)
        (root / "aiformula_runtime/2026-11-19-yolop-selection").mkdir(parents=True)
        active_launch = root / "smoke.launch.py"
        active_launch.write_text(launch_source)
        env = dict(os.environ, HOME=home, XDG_RUNTIME_DIR=home)
        rclpy.init()
        probe = rclpy.create_node("isolated_oa1b_lifecycle_probe")
        stages_started = []
        try:
            for stage in stages:
                proc = subprocess.Popen([sys.executable, supervisor, stage, "--", "ros2", "launch", str(active_launch)],
                                        env=dict(env, SMOKE_STAGE=stage))
                stages_started.append(proc)
                name = "test_oa1b_"+stage
                for _ in range(100):
                    if proc.poll() is not None:
                        raise RuntimeError(f"{stage} unexpectedly stopped with {proc.returncode}")
                    if name in probe.get_node_names():
                        print(stage, "discovered", name, flush=True)
                        break
                    time.sleep(.1)
                else:
                    raise TimeoutError(name)
            stages_started[2].send_signal(signal.SIGHUP)
            codes=[proc.wait(timeout=15) for proc in stages_started]
            print("Exit codes:", codes, flush=True)
            assert codes == [0]*4, codes
            for _ in range(60):
                survivors = [name for name in probe.get_node_names() if name.startswith("test_oa1b_")]
                if not survivors:
                    break
                time.sleep(.1)
            assert not survivors, survivors
            for stage in ("allnodes","mapassist","lya"):
                with (root / f"aiformula-1120-{env['ROS_DOMAIN_ID']}-{stage}.lock").open("r+") as lock:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            print("All four ROS nodes absent, old command locks reacquired, observer alive.",flush=True)
        finally:
            for p in stages_started:
                if p.poll() is None:
                    p.terminate()
                p.wait(timeout=10)
            probe.destroy_node()
            rclpy.shutdown()


if __name__=="__main__":
    execute()
