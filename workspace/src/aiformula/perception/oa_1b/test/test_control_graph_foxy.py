"""Run in an isolated Foxy container with patched PYTHONPATH and no CAN device."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


PREFIX = "/semantic_drive/map_assist_045"
PRIVATE = "/oa_1b/perception"
FIELD = "/aiformula_perception/yolop_a3"


def observe(prefix, lane):
    command = ["ros2","launch",str(prefix / "launch/oa1b.launch.py"),"lane:="+lane,
               "start_oa2:=true","oa_algorithm:=oa_2_1","vehicle_output:=false",
               "oa_initially_idle:=false","start_lya:=false","red_pixel_stop_threshold:=50",
               "lane_point_topic_prefix:=/original_processed_point_"]
    with tempfile.TemporaryFile() as log:
        launch = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        rclpy.init()
        probe = Node("oa1b_graph_test")
        old = probe.create_publisher(String,FIELD+"/detections",10)
        private = probe.create_publisher(String,PRIVATE+"/detections",10)
        stops, modes = [], []
        probe.create_subscription(Bool,"/flag_stop",lambda msg: stops.append(msg.data),10)
        probe.create_subscription(String,PREFIX+"/status",lambda msg: modes.append(json.loads(msg.data)),10)

        def ready(predicate, timeout=15):
            deadline = time.monotonic()+timeout
            while time.monotonic()<deadline:
                if launch.poll() is not None:
                    raise RuntimeError("ros2 launch exited before verification")
                rclpy.spin_once(probe,timeout_sec=.05)
                if predicate():
                    return
            raise AssertionError("DDS observation timed out")

        def owners(topic,sub=True):
            getter=probe.get_subscriptions_info_by_topic if sub else probe.get_publishers_info_by_topic
            return {endpoint.node_namespace.rstrip("/")+"/"+endpoint.node_name for endpoint in getter(topic)}

        def detection(publisher,count):
            sec, nsec = probe.get_clock().now().seconds_nanoseconds()
            publisher.publish(String(data=json.dumps({"header":{"frame_id":"zed_left_camera_optical_frame",
                "stamp":{"sec":sec,"nanosec":nsec}},"red_pixels_count":count})))

        try:
            ready(lambda: "/a3_pixel_stop_position" in owners(PRIVATE+"/detections") and
                  "/map_assist" in owners(PRIVATE+"/traffic_light/state") and modes and
                  owners(PREFIX+"/oa_triplet",False) and owners(PREFIX+"/oa_triplet"))
            assert "/map_assist" not in owners(FIELD+"/traffic_light/state")
            assert "/a3_pixel_stop_position" not in owners(FIELD+"/detections")
            assert owners("/flag_stop",False)=={"/a3_pixel_stop_position"}
            assert owners(PREFIX+"/diagnostic_cmd_vel",False)=={"/map_assist"}
            assert "/map_assist" in owners("/aiformula_control/motor_controller/manual_motion_started")
            assert "/map_assist" in owners(PREFIX+"/lya_cmd_vel")
            assert not owners(PREFIX+"/lya_cmd_vel",False),"third command must not start LYA"
            assert all("/map_assist" in owners("/original_processed_point_"+s) for s in "abc")
            detection(old,500)
            for _ in range(10): rclpy.spin_once(probe,timeout_sec=.04)
            assert not stops, "old detector controlled private pixel stop"
            detection(private,49)
            ready(lambda: stops==[False])
            detection(private,50)
            ready(lambda: stops==[False,True])
            assert modes[-1]["smooth_min_turn_radius_m"]=={"inner":6.0,"outer":5.5}[lane]
            print(json.dumps({"lane":lane,"stop_at_49_50":stops,"oa_pub":sorted(owners(PREFIX+"/oa_triplet",False)),
                "oa_sub":sorted(owners(PREFIX+"/oa_triplet")),"control_source":sorted(owners(PREFIX+"/diagnostic_cmd_vel",False)),
                "turn_radius":modes[-1]["smooth_min_turn_radius_m"]}))
        finally:
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
                launch.wait(timeout=20)
            remaining = set()
            for _ in range(50):
                remaining = set(probe.get_node_names())-{"oa1b_graph_test"}
                if not remaining:
                    break
                time.sleep(.10)
            probe.destroy_node()
            rclpy.shutdown()
            log.seek(0)
            print(log.read().decode(errors="replace")[-1500:])
            assert not remaining, f"Nodes left after launch exit: {remaining}"


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix",type=Path)
    args=parser.parse_args()
    for lane in ("inner","outer"):
        observe(args.prefix,lane)
