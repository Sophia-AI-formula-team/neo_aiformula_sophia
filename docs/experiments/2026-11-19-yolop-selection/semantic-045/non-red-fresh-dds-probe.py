"""Check the official MapAssist diagnostic launch on localhost-only DDS."""
from datetime import datetime
import json
import os
import time

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

if (os.environ.get("ROS_DOMAIN_ID") != "203"
        or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"):
    raise SystemExit("Use isolated localhost ROS domain 203")

prefix = "/semantic_drive/map_assist_045"
inputs = {
    "/aiformula_sensing/gyro_odometry_publisher/odom": Odometry,
    "/aiformula_perception/yolop_a3/traffic_light/state": String,
    "/semantic_fixture/stop_distance": Bool,
    "/point_flag": Bool,
    "/semantic_drive/branch_active": Bool,
    "/aiformula_control/game_pad/cmd_vel": Twist,
    **{f"/processed_point_{label}": Pose2D for label in "abc"},
}
outputs = [prefix + "/" + name for name in ("status", "diagnostic_cmd_vel", "path", "detour_path")]
outputs.extend(prefix + f"/point_{label}" for label in "abc")
rclpy.init()
node = rclpy.create_node("semantic045_probe")
states, diagnostic, translated = [], [], []
node.create_subscription(String, prefix + "/status",
                         lambda msg: states.append(json.loads(msg.data)), 10)
node.create_subscription(Twist, prefix + "/diagnostic_cmd_vel",
                         lambda msg: diagnostic.append((msg.linear.x, msg.angular.z)), 10)
preview = prefix + "/stop_wait_preview"
node.create_subscription(String, preview, lambda msg: translated.append(json.loads(msg.data)), 10)
pubs = {name: node.create_publisher(kind, name, 10) for name, kind in inputs.items()}


def endpoint(topic, is_publisher):
    read = node.get_publishers_info_by_topic if is_publisher else node.get_subscriptions_info_by_topic
    return [entry for entry in read(topic) if entry.node_name == "map_assist"]


def await_state(predicate, step, sec=5):
    deadline = time.monotonic() + sec
    while time.monotonic() < deadline:
        step()
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.03)
        if states and predicate(states[-1]):
            return states[-1]
    raise RuntimeError(f"Expected state not observed. Latest state: {states[-1] if states else None}")


def frames(light, stop=None):
    odom = Odometry()
    odom.header.stamp = node.get_clock().now().to_msg()
    odom.header.frame_id = "odom"
    odom.pose.pose.orientation.w = 1.0
    pubs["/aiformula_sensing/gyro_odometry_publisher/odom"].publish(odom)
    for x, label in enumerate("abc", 2):
        pubs[f"/processed_point_{label}"].publish(Pose2D(x=float(x)))
    pubs["/point_flag"].publish(Bool(data=False))
    pubs["/semantic_drive/branch_active"].publish(Bool(data=False))
    pubs["/aiformula_control/game_pad/cmd_vel"].publish(Twist())
    pubs["/aiformula_perception/yolop_a3/traffic_light/state"].publish(String(data=light))
    if stop is not None:
        pubs["/semantic_fixture/stop_distance"].publish(Bool(data=stop))


try:
    initial = await_state(
        lambda state: state["traffic_planner_version"] == "ubs_auto_turn_v1",
        lambda: None, sec=12)
    assert initial["avoidance_active"] is None and initial["traffic_stop_flag"] is None
    assert initial["smooth_min_turn_radius_m"] == 6.0
    observed_inputs = {name: endpoint(name, False) for name in inputs}
    assert all(len(endpoints) == 1 for endpoints in observed_inputs.values())
    assert all(len(endpoint(name, True)) == 1 for name in outputs)
    assert not endpoint("/aiformula_control/game_pad/cmd_vel", True)
    assert not endpoint("/aiformula_control/motor_controller/reference_signal", True)
    no_flag = await_state(lambda state: state["traffic_reason"] == "red_wait_flag_unavailable",
                          lambda: frames("red"))
    stopped = await_state(lambda state: state["traffic_reason"] == "red_wait_first_lap"
                           and translated and translated[-1]["traffic_reason"] == "red_wait_first_lap",
                          lambda: frames("red", True))
    assert stopped["traffic_stop_flag"] is True and not stopped["avoidance_active"]
    assert diagnostic and diagnostic[-1] == (0.0, 0.0)
    assert translated and translated[-1]["traffic_stop_required"]
    silence = await_state(lambda state: state["traffic_light"] == "unknown",
                          lambda: None)
    assert silence["traffic_mode"] == "wait"
    unknown = await_state(lambda state: state["traffic_reason"] == "non_red_resume"
                          and state["traffic_light_raw"] == "unknown"
                          and translated and translated[-1]["traffic_mode"] == "normal",
                          lambda: frames("unknown", True))
    assert unknown["traffic_light"] == "not_red"
    await_state(lambda state: state["traffic_reason"] == "red_wait_first_lap"
                and translated and translated[-1]["traffic_stop_required"],
                lambda: frames("red", True))
    off = await_state(lambda state: state["traffic_reason"] == "red_off_resume"
                      and translated[-1]["traffic_reason"] == "red_off_resume",
                      lambda: frames("off", False))
    assert off["traffic_mode"] == "normal"
    await_state(lambda state: state["traffic_reason"] == "red_wait_first_lap"
                and translated[-1]["traffic_reason"] == "red_wait_first_lap",
                lambda: frames("red", True))
    yellow = await_state(lambda state: state["traffic_reason"] == "non_red_resume"
                         and translated[-1]["traffic_reason"] == "non_red_resume",
                         lambda: frames("yellow", False))
    assert yellow["traffic_mode"] == "normal"
    assert translated[-1]["traffic_policy"] == "stop_wait"
    assert translated[-1]["traffic_stop_required"] is False
    assert node.count_publishers("/semantic_drive/map_status") == 0
    assert node.count_publishers("/semantic_drive/motor_session") == 0
    assert yellow["traffic_stop_flag"] is False
    print(json.dumps({
        "time_jst": datetime.now().astimezone().isoformat(timespec="seconds"),
        "middleware": "ROS Jazzy desktop localhost domain 203",
        "planner": initial["traffic_planner_version"],
        "radius_m": initial["smooth_min_turn_radius_m"],
        "subscriptions": {name: {"type": entries[0].topic_type, "reliability": entries[0].qos_profile.reliability.name,
                                 "durability": entries[0].qos_profile.durability.name}
                          for name, entries in observed_inputs.items()},
        "diagnostic_outputs": outputs,
        "red_no_stop_publisher": no_flag["traffic_reason"],
        "red_stop_true": stopped["traffic_reason"],
        "silence_mode": silence["traffic_mode"],
        "fresh_unknown_mode": unknown["traffic_mode"],
        "off_reason": off["traffic_reason"], "yellow_reason": yellow["traffic_reason"],
        "physical_outputs": 0, "stop_wait_translation": "wait then normal",
    }, indent=2), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
