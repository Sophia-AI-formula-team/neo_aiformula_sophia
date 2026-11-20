"""Check DDS ownership in the vehicle domain, without starting the ROS CLI daemon."""
import argparse
import json
import time

import rclpy

RAW = "/aiformula_sensing/zed_node/left_image/undistorted"
ODOM = "/aiformula_sensing/gyro_odometry_publisher/odom"
PUBLIC = "/aiformula_perception/yolop_a3"
PRIVATE = "/oa_1b/perception"
PREFIX = "/semantic_drive/map_assist_045/"
MOTOR = "/aiformula_control/motor_controller"
MASK = "/aiformula_perception/road_detector/mask_image"
CAM_NODE = "/aiformula_perception/yolop_a3_perception"
POSE = "geometry_msgs/msg/Pose2D"
TWIST = "geometry_msgs/msg/Twist"
TEXT = "std_msgs/msg/String"
IMAGE = "sensor_msgs/msg/Image"

# topic -> (ROS type, sole expected publisher, required subscribers). None permits
# the actual hardware or simulator source for camera, odometry and vehicle-specific nodes.
SENSORS = {
    RAW: (IMAGE, None, ()),
    ODOM: ("nav_msgs/msg/Odometry", None, ()),
    MOTOR+"/manual_motion_started": ("builtin_interfaces/msg/Time", MOTOR, ()),
    MOTOR+"/reference_signal": ("can_msgs/msg/Frame", MOTOR, ()),
    **{"/original_processed_point_"+k:(POSE,None,()) for k in "abc"},
}
VISION = {
    PRIVATE+"/detections": (TEXT,CAM_NODE,()),
    PRIVATE+"/traffic_light/state": (TEXT,CAM_NODE,()),
    PUBLIC+"/detections": (TEXT,CAM_NODE,()),
    MASK: (IMAGE,CAM_NODE,()),
    "/cone_mask": (IMAGE,CAM_NODE,()),
}
PLANNER = {
    ODOM: ("nav_msgs/msg/Odometry", None, ("/map_assist", "/cone_mask_avoider", CAM_NODE)),
    PREFIX+"diagnostic_cmd_vel": (TWIST,"/map_assist",(MOTOR,)),
    PREFIX+"status": (TEXT,"/map_assist",(CAM_NODE,)),
    PREFIX+"oa_triplet": ("geometry_msgs/msg/PoseArray","/cone_mask_avoider",("/data_processing_node",)),
    PREFIX+"oa_ready": ("std_msgs/msg/Bool","/data_processing_node",("/map_assist",)),
    "/point_flag": ("std_msgs/msg/Bool","/cone_mask_avoider",("/map_assist","/data_processing_node")),
    "/flag_stop": ("std_msgs/msg/Bool","/a3_pixel_stop_position",("/map_assist",)),
    "/semantic_drive/motor_session": (TEXT,"/semantic_drive/motor_gate_session",(MOTOR,)),
    "/semantic_drive/map_status": (TEXT,"/map_assist_stop_wait_candidate",(MOTOR,)),
    PRIVATE+"/detections": (TEXT,CAM_NODE,("/a3_pixel_stop_position",)),
    PRIVATE+"/traffic_light/state": (TEXT,CAM_NODE,("/map_assist",)),
    "/cone_mask": (IMAGE,CAM_NODE,("/cone_mask_avoider",)),
    MOTOR+"/manual_motion_started": ("builtin_interfaces/msg/Time",MOTOR,("/map_assist",)),
    PREFIX+"filtered_lane_pose": (POSE,"/data_processing_node",()),
    PREFIX+"filtered_omega_t": (POSE,"/data_processing_node",()),
    **{"/original_processed_point_"+k:(POSE,None,("/map_assist",)) for k in "abc"},
    **{PREFIX+"point_"+k:(POSE,"/map_assist",("/data_processing_node",)) for k in "abc"},
}
FOLLOWER = {
    PREFIX+"lya_cmd_vel": (TWIST,"/trajectory_follower",("/map_assist",)),
    PREFIX+"filtered_lane_pose": (POSE,"/data_processing_node",("/trajectory_follower",)),
    PREFIX+"filtered_omega_t": (POSE,"/data_processing_node",("/trajectory_follower",)),
}


def endpoints(node, topic, publisher):
    lookup = node.get_publishers_info_by_topic if publisher else node.get_subscriptions_info_by_topic
    return [(info.node_namespace.rstrip("/")+"/"+info.node_name, info.topic_type) for info in lookup(topic)]


def inspect(node, stage, vehicle=False):
    required, vacant = {}, set()
    if stage=="allnodes":
        vacant.update((*SENSORS,*VISION,*PLANNER,*FOLLOWER))
    else:
        required.update(SENSORS)
        if stage=="yolop":
            vacant.update(VISION)
        else:
            required.update(VISION)
            required[RAW]=(IMAGE,None,(CAM_NODE,))
            if stage=="oa1b":
                vacant.update(k for k in PLANNER if k not in required)
                vacant.update(FOLLOWER)
            else:
                required.update(PLANNER)
                if stage=="lya":
                    vacant.add(PREFIX+"lya_cmd_vel")
                else:
                    required.update(FOLLOWER)
                    required[ODOM] = ("nav_msgs/msg/Odometry", None,
                                      ("/map_assist", "/cone_mask_avoider", CAM_NODE, "/trajectory_follower"))
    if vehicle and stage != "allnodes":
        required[MOTOR+"/reference_signal"] = (
            "can_msgs/msg/Frame", MOTOR, ("/aiformula_control/socket_can_sender",))
    errors,observed=[],{}
    for topic in sorted(set(required) | vacant):
        pubs=endpoints(node,topic,True)
        if topic in vacant:
            if pubs: errors.append(f"{topic} already has publishers {pubs}")
            continue
        kind, owner, needed=required[topic]
        subs=endpoints(node,topic,False)
        observed[topic]=dict(publishers=pubs,subscribers=subs)
        if len(pubs)!=1 or pubs[0][1]!=kind or (owner and pubs[0][0]!=owner):
            errors.append(f"{topic}: expected a single {kind} from {owner or 'vehicle'}, found {pubs}")
        for sub in needed:
            if (sub,kind) not in subs: errors.append(f"{topic}: missing subscriber {sub} ({kind})")
    if stage in ("lya","live"):
        for topic,name in ((PUBLIC+"/detections","/a3_pixel_stop_position"),(PUBLIC+"/traffic_light/state","/map_assist")):
            if any(n==name for n,_ in endpoints(node,topic,False)):
                errors.append(f"{name} also subscribes to baseline signal {topic}")
    return errors,observed


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage",choices=["allnodes","yolop","oa1b","lya","live"])
    parser.add_argument("--timeout",type=float,default=15)
    parser.add_argument("--vehicle", action="store_true",
                        help="Require the real Allnodes SocketCAN sender on the motor output")
    args=parser.parse_args()
    rclpy.init(args=[])
    observer=rclpy.create_node("oa1b_wiring_check",enable_rosout=False,start_parameter_services=False)
    try:
        start=time.monotonic()
        while True:
            rclpy.spin_once(observer,timeout_sec=.15)
            errors,graph=inspect(observer,args.stage,args.vehicle)
            if not errors and time.monotonic()-start>=1.5: break
            if time.monotonic()-start>=args.timeout:
                raise SystemExit("\n".join(errors) or "DDS discovery did not settle")
        if args.stage=="live": print(json.dumps(graph,indent=2))
    finally:
        observer.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
