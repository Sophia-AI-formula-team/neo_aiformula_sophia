"""AV314 ROS node with experiment-only traffic outputs. One model and one image subscription."""
import hashlib
import inspect
import json
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from yolop_a3_perception.node import YolopA3Node
from yolop_a3_runtime import color_semantics, shared_runtime, traffic_light_policy

from .traffic_signal import SignalTracker


WEIGHTS = 'a411c983b661d6b26217019a968251c89a6b66c95400e4e7cb78f04a644ca10c'
PINNED = (
    (inspect.getmodule(YolopA3Node), 'dca1bf2835e4302fcc6f2c9a18b833e132bc50c605e482f3592c86e0fbf6a139'),
    (shared_runtime, 'd5d43a905b05c6a8f02721485025f9077cebf5c81b3fe3f8e69897ee3e53601d'),
    (color_semantics, '483da35887ae28931cf1e473843faba74e49201634e1493ccdf709781492c285'),
    (traffic_light_policy,'8eb8fa7f0871d94946e59d15cd9e1ddf182c5238b1719005d024684fb0e982d6'),
)
OUTPUT = '/oa_1b/perception'


def digest(path):
    sha = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024), b''):
            sha.update(block)
    return sha.hexdigest()


def verify_field_runtime():
    for module, expected in PINNED:
        path = inspect.getfile(module)
        actual = digest(path)
        if actual != expected:
            raise RuntimeError(f'OA 1B expected {expected} for {path}, found {actual}')


class ModelTap:
    def __init__(self, runtime):
        self.runtime = runtime
        self.hook = runtime.model.register_forward_hook(self.capture)
        self.frame = self.candidates = None

    def capture(self, module, args, output):
        # The stock forward has completed NMS but has not yet applied the lamp shape filters.
        objects = output[1][0]
        select = objects['labels']==10
        boxes = objects['boxes'][select].detach().cpu().tolist()
        scores = objects['scores'][select].detach().cpu().tolist()
        self.candidates = list(zip(boxes,scores))

    def predict(self, image):
        self.frame = self.candidates = None
        result = self.runtime.predict(image)
        if self.candidates is None:
            raise RuntimeError('AV314 did not provide ROI-head results')
        self.frame = (image,result,self.candidates)
        return result


class Oa1bPerception(YolopA3Node):
    def __init__(self):
        verify_field_runtime()
        super().__init__()
        weights = Path(self.get_parameter('weights').value).expanduser()
        if digest(weights)!=WEIGHTS or self.model.score_threshold!=.5 or self.model.track_signal_board:
            raise RuntimeError('OA 1B requires the AV314 field weights, score 0.50 and native detector head')
        self.model = ModelTap(self.model)
        self.tracker = SignalTracker()
        self.private_detections = self.create_publisher(String,OUTPUT+'/detections',10)
        self.private_light = self.create_publisher(String,OUTPUT+'/traffic_light/state',10)
        self.create_subscription(String,'/semantic_drive/map_assist_045/status',self.on_status,10)
        self.create_subscription(Odometry,'/aiformula_sensing/gyro_odometry_publisher/odom',
                                 self.on_odometry,qos_profile_sensor_data)
        self.get_logger().info('OA 1B single-pass AV314 active; original outputs and '+OUTPUT)

    def on_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            payload = {}
        self.tracker.status(payload if isinstance(payload,dict) else {},time.monotonic())

    def on_odometry(self, msg):
        t = msg.header.stamp
        age = (self.get_clock().now().nanoseconds-(t.sec*10**9+t.nanosec))*1e-9
        if not 0<=age<=.5:
            return
        pos, velocity = msg.pose.pose.position, msg.twist.twist.linear
        self.tracker.odometry(pos.x,pos.y,(velocity.x**2+velocity.y**2)**.5,time.monotonic())

    def receive(self, msg):
        self.model.frame = None
        # Keep all public image publishers, encodings and original accepted detections stock.
        super().receive(msg)
        frame, self.model.frame = self.model.frame, None
        if frame is None:
            return
        t = msg.header.stamp
        stamp = t.sec*10**9+t.nanosec
        if msg.header.frame_id!='zed_left_camera_optical_frame' or not self.tracker.accept_stamp(
                stamp,self.get_clock().now().nanoseconds):
            return
        image, original, candidates = frame
        payload = self.tracker.correct(image, original.detections_payload(), candidates, stamp,
                                       time.monotonic(), color_semantics.classify_traffic_light)
        payload['header'] = dict(stamp=dict(sec=t.sec,nanosec=t.nanosec),frame_id=msg.header.frame_id)
        self.private_detections.publish(String(data=json.dumps(payload,allow_nan=False)))
        self.private_light.publish(String(data=payload['traffic_light_state']))

    def destroy_node(self):
        self.model.hook.remove()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Oa1bPerception()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
