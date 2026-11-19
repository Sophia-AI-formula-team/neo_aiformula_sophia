"""Observe the formal MapAssist child in a private domain without starting LYA."""
import json
import os
import time

from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
import rclpy
from std_msgs.msg import Bool, String

assert os.environ.get('ROS_DOMAIN_ID') == '195' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
prefix = '/semantic_drive/map_assist_045'
rclpy.init()
node = rclpy.create_node('mapassist_desktop_no_lya_probe')
observed = {}
diagnostic = []
topics = ['/semantic_drive/map_status', '/semantic_drive/motor_session', prefix+'/status']
for topic in topics:
    def receive(message, topic=topic):
        observed[topic] = json.loads(message.data)
    node.create_subscription(String, topic, receive, 10)
node.create_subscription(Twist, prefix+'/diagnostic_cmd_vel',
                         lambda m: diagnostic.append([m.linear.x,m.angular.z]), 10)
odom = node.create_publisher(Odometry, '/aiformula_sensing/gyro_odometry_publisher/odom', 10)
light = node.create_publisher(String, '/aiformula_perception/yolop_a3/traffic_light/state', 10)
abc = [node.create_publisher(Pose2D, '/processed_point_'+k, 10) for k in 'abc']

try:
    end = time.monotonic()+12.
    next_at = 0.
    while time.monotonic()<end:
        now = time.monotonic()
        if now>=next_at:
            msg = Odometry()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id, msg.child_frame_id = 'odom', 'base_footprint'
            msg.pose.pose.orientation.w = 1.
            odom.publish(msg)
            light.publish(String(data='off'))
            for i,p in enumerate(abc):
                p.publish(Pose2D(x=1.+i,y=.1))
            next_at = now+.08
        rclpy.spin_once(node, timeout_sec=.04)
        if all(t in observed for t in topics) and observed[prefix+'/status'].get('recording_points_valid') is True:
            break

    nodes = sorted(node.get_node_names())
    graph = dict(legacy_stop_wait=observed['/semantic_drive/map_status']['traffic_policy'],
                 legacy_stop_required=observed['/semantic_drive/map_status']['traffic_stop_required'],
                 motor_session_active=observed['/semantic_drive/motor_session']['active'],
                 trajectory_follower_present='trajectory_follower' in nodes,
                 lya_endpoint_publishers=node.count_publishers(prefix+'/lya_cmd_vel'),
                 lya_endpoint_subscribers=[i.node_name for i in node.get_subscriptions_info_by_topic(prefix+'/lya_cmd_vel')],
                 oa_node_present='cone_mask_avoider' in nodes,
                 public_motor_command_publishers=node.count_publishers('/aiformula_control/game_pad/cmd_vel'),
                 physical_can_publishers=node.count_publishers('/aiformula_control/motor_controller/reference_signal'),
                 nonzero_output_without_lya=sum(any(c) for c in diagnostic))
    assert graph == dict(legacy_stop_wait='stop_wait',legacy_stop_required=True,motor_session_active=True,
                         trajectory_follower_present=False,lya_endpoint_publishers=0,
                         lya_endpoint_subscribers=['map_assist'],oa_node_present=False,
                         public_motor_command_publishers=0,physical_can_publishers=0,
                         nonzero_output_without_lya=0), graph
    print(json.dumps(graph, indent=2))
finally:
    node.destroy_node()
    rclpy.shutdown()
