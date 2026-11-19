"""Vehicle-local synthetic routing check with no motor or CAN readers/writers."""
from datetime import datetime
import json
import os
import time

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

PREFIX = "/semantic_drive/map_assist_045"
ODOM = "/aiformula_sensing/gyro_odometry_publisher/odom"
LIGHT = "/aiformula_perception/yolop_a3/traffic_light/state"
STOP = "/semantic_fixture/stop_distance"
FLAG = "/point_flag"


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "75" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Use localhost-only ROS domain 75")
    rclpy.init()
    node = rclpy.create_node("semantic_commissioning_probe")
    data = {name: [] for name in ("pose", "lya", "guarded", "state", "legacy")}

    def subscribe(name, msg, suffix, decode=lambda value: value):
        node.create_subscription(msg, PREFIX + suffix, lambda value: data[name].append(decode(value)), 10)

    subscribe("pose", Pose2D, "/filtered_lane_pose")
    subscribe("lya", Twist, "/lya_cmd_vel")
    subscribe("guarded", Twist, "/diagnostic_cmd_vel")
    subscribe("state", String, "/status", lambda value: json.loads(value.data))
    subscribe("legacy", String, "/stop_wait_candidate", lambda value: json.loads(value.data))
    pubs = {topic: node.create_publisher(kind, topic, 10) for topic, kind in (
        (ODOM, Odometry), (LIGHT, String), (STOP, Bool), (FLAG, Bool),
    )}
    for label in "abc":
        for prefix in ("/processed_point_", "/oa_point_"):
            pubs[prefix + label] = node.create_publisher(Pose2D, prefix + label, 10)

    def latest(name):
        return data[name][-1] if data[name] else None

    def drive_frame(*, oa=False, light="green", stop=False, oa_points="abc"):
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        pubs[ODOM].publish(odom)
        pubs[LIGHT].publish(String(data=light))
        pubs[STOP].publish(Bool(data=stop))
        pubs[FLAG].publish(Bool(data=oa))
        for i, label in enumerate("abc"):
            x = 1.0 + i
            pubs["/processed_point_" + label].publish(Pose2D(x=x, y=0.15))
            if label in oa_points:
                pubs["/oa_point_" + label].publish(Pose2D(x=x, y=-0.2))

    def wait_for(predicate, action=lambda: None, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            action()
            end = time.monotonic() + .09
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.015)
            if predicate():
                return
        raise AssertionError({
            "counts": {key: len(values) for key, values in data.items()},
            "state": latest("state"), "legacy": latest("legacy"),
        })

    def graph_has(topic, name):
        return any(i.node_name == name for i in node.get_subscriptions_info_by_topic(topic))

    def moving(msg):
        return msg is not None and (msg.linear.x != 0.0 or msg.angular.z != 0.0)

    def y_is(y):
        return latest("pose") is not None and abs(latest("pose").y - y) < 1e-6

    try:
        wait_for(lambda: y_is(.15) and moving(latest("lya")) and moving(latest("guarded"))
                 and latest("legacy") and not latest("legacy")["traffic_stop_required"],
                 drive_frame, timeout=20)
        assert graph_has(PREFIX + "/point_a", "data_processing_node")
        assert graph_has(PREFIX + "/filtered_lane_pose", "trajectory_follower")
        assert graph_has(PREFIX + "/lya_cmd_vel", "map_assist")
        map_speed = [latest("lya").linear.x, latest("lya").angular.z]

        wait_for(lambda: graph_has("/oa_point_c", "data_processing_node"),
                 lambda: drive_frame(oa=True, oa_points=""))
        pose_count = len(data["pose"])
        end = time.monotonic() + .4
        while time.monotonic() < end:
            drive_frame(oa=True, oa_points="c")
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=.02)
        assert len(data["pose"]) == pose_count, "Cached map ABC mixed with partial OA data"
        wait_for(lambda: y_is(-.2) and moving(latest("lya")), lambda: drive_frame(oa=True))
        oa_speed = [latest("lya").linear.x, latest("lya").angular.z]
        wait_for(lambda: y_is(.15), drive_frame)

        wait_for(lambda: latest("state") and latest("state")["traffic_mode"] == "wait"
                 and latest("legacy") and latest("legacy")["traffic_stop_required"]
                 and latest("guarded") is not None and not moving(latest("guarded")),
                 lambda: drive_frame(light="red", stop=True))
        wait_for(lambda: latest("state")["traffic_light"] == "not_red"
                 and latest("legacy")["traffic_mode"] == "normal" and moving(latest("guarded")),
                 lambda: drive_frame(light="unknown"))
        for topic in ("/aiformula_control/game_pad/cmd_vel",
                      "/aiformula_control/motor_controller/reference_signal",
                      "/semantic_drive/map_status", "/semantic_drive/motor_session"):
            assert node.count_publishers(topic) == 0, topic
        print(json.dumps(dict(
            time_jst=datetime.now().astimezone().isoformat(timespec="seconds"),
            original_lya_from_map=map_speed, original_lya_from_oa=oa_speed,
            switched_back_to_map=True, partial_triplet_forwarded=False,
            red_stopped_guarded_command=True, fresh_unknown_resumed=True,
            physical_publishers=0, ros_domain=75,
        ), indent=2), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
