import json
import math
from collections import OrderedDict, deque
from dataclasses import fields
from time import perf_counter

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

from .geometry import GroundCamera, rotation_matrix
from .inputs import (
    Cloud, RasterFrame, align_clouds, boxes_to_mask, cloud_xy, decode_detections, decode_raster,
    matched_clouds,
)
from .planner import PlanResult, PlannerConfig, plan, render_overlay


LANE_PREFIX = "/aiformula_perception/lane_line_publisher/lane_lines"
DETECTION_TOPIC = "/aiformula_perception/yolop_a3/detections"
CAMERA_INFO_TOPIC = "/aiformula_sensing/zed_node/left/camera_info"
ODOM_TOPIC = "/aiformula_sensing/gyro_odometry_publisher/odom"


def _stamp_ns(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec


def _quaternion(quaternion):
    return quaternion.x, quaternion.y, quaternion.z, quaternion.w


class ShadowPlanner(Node):
    def __init__(self):
        super().__init__("opti_oa_0_0")
        self.config = PlannerConfig(**{
            spec.name: self.declare_parameter(
                spec.name, -1.0 if spec.name == "vehicle_width_m" else spec.default
            ).value
            for spec in fields(PlannerConfig)
        })
        self.frame = self.declare_parameter("base_frame", "base_footprint").value
        self.roi = self.declare_parameter("roi_top_ratio", 0.5).value
        sync_slop = self.declare_parameter("sync_slop_sec", 0.22).value
        max_age = self.declare_parameter("max_detection_age_sec", 0.8).value
        max_odom_gap = self.declare_parameter("max_odom_gap_sec", 0.7).value
        self.config_error = ""
        try:
            self.config.validate()
            if not (0 <= self.roi < 1 and 0 < sync_slop < max_age < 10 and 0 < max_odom_gap < 10):
                raise ValueError("invalid_projection_or_timing_config")
        except ValueError as error:
            self.config_error = str(error)
        else:
            self.slop_ns, self.max_age_ns, self.max_odom_gap_ns = (
                int(1e9 * seconds) for seconds in (sync_slop, max_age, max_odom_gap)
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odometry = OrderedDict()
        self.clouds = {side: deque(maxlen=32) for side in ("center", "left", "right")}
        self.camera_info = {}
        self.detection = None
        self.mask = None
        self.backgrounds = deque(maxlen=4)
        self.input_error = ""
        self.last_signature = None
        output_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "~/path", output_qos)
        self.band_pub = self.create_publisher(Image, "~/path_band", output_qos)
        self.overlay_pub = self.create_publisher(Image, "~/overlay", output_qos)
        self.status_pub = self.create_publisher(String, "~/status", output_qos)

        qos = qos_profile_sensor_data
        self.create_subscription(String, self.declare_parameter("detections_topic", DETECTION_TOPIC).value,
                                 self._on_detection, qos)
        self.create_subscription(Image, self.declare_parameter("mask_topic", "/cone_mask").value,
                                 self._on_mask, qos)
        self.create_subscription(Image, self.declare_parameter("overlay_base_topic", "/cone_bbox_image").value,
                                 self._on_background, qos)
        self.create_subscription(CameraInfo, self.declare_parameter("camera_info_topic", CAMERA_INFO_TOPIC).value,
                                 self._on_camera_info, qos)
        self.create_subscription(Odometry, self.declare_parameter("odom_topic", ODOM_TOPIC).value,
                                 self._on_odom, qos)
        for side in self.clouds:
            topic = self.declare_parameter(side + "_topic", LANE_PREFIX + "/" + side).value
            self.create_subscription(PointCloud2, topic, lambda msg, key=side: self._on_cloud(key, msg), qos)
        self.create_timer(0.10, self._tick)
        self.get_logger().info("Camera-space corridor trial enabled with independent diagnostic outputs")

    def _on_camera_info(self, msg):
        self.camera_info[msg.header.frame_id] = msg

    def _on_detection(self, msg):
        try:
            self.detection = decode_detections(msg.data)
            self.input_error = ""
        except (ValueError, TypeError, KeyError, AttributeError):
            self.detection = None
            self.input_error = "invalid_detection_message"

    @staticmethod
    def _raster_frame(msg):
        frame = msg.header.frame_id
        stamp = _stamp_ns(msg.header)
        if not frame or stamp <= 0:
            raise ValueError("raster_header_required")
        return RasterFrame(stamp, frame, decode_raster(msg.data, msg.width, msg.height,
                                                       msg.step, msg.encoding))

    def _on_mask(self, msg):
        try:
            raster = self._raster_frame(msg)
            if raster.pixels.ndim != 2:
                raise ValueError("mono8_mask_required")
            self.mask = raster
        except (ValueError, TypeError, BufferError):
            self.mask = None
            self.input_error = "invalid_obstacle_mask"

    def _on_background(self, msg):
        try:
            raster = self._raster_frame(msg)
        except (ValueError, TypeError, BufferError):
            return
        if raster.pixels.ndim == 3:
            self.backgrounds.append(raster)

    def _background_for(self, stamp_ns, frame_id):
        return next((image.pixels for image in reversed(self.backgrounds)
                     if image.stamp_ns == stamp_ns and image.frame_id == frame_id), None)

    def _on_cloud(self, side, msg):
        stamp = _stamp_ns(msg.header)
        if stamp <= 0:
            self.clouds[side].clear()
            return
        try:
            if msg.header.frame_id != self.frame:
                raise ValueError("unexpected_lane_frame")
            points = cloud_xy(msg.data, {f.name: (f.offset, f.datatype) for f in msg.fields},
                              msg.width, msg.height, msg.row_step, msg.point_step, msg.is_bigendian)
        except (ValueError, TypeError, KeyError, BufferError):
            points = np.empty((0, 2))
        self.clouds[side].append(Cloud(stamp, points))

    def _on_odom(self, msg):
        stamp = _stamp_ns(msg.header)
        if msg.child_frame_id != self.frame or stamp <= 0:
            return
        position = msg.pose.pose.position
        try:
            rotation = rotation_matrix(_quaternion(msg.pose.pose.orientation))
        except ValueError:
            return
        pose = (position.x, position.y, math.atan2(rotation[1, 0], rotation[0, 0]))
        if not all(math.isfinite(value) for value in pose):
            return
        self.odometry[stamp] = pose
        if len(self.odometry) > 256:
            self.odometry.popitem(last=False)

    def _tick(self):
        if self.config_error:
            self._fail(self.config_error)
            return
        use_mask = self.mask is not None and (
            self.detection is None or self.mask.stamp_ns >= self.detection.stamp_ns
        )
        source = self.mask if use_mask else self.detection
        if source is None:
            self._fail(self.input_error or "waiting_for_obstacle_frame")
            return
        stamp = Time(nanoseconds=source.stamp_ns, clock_type=self.get_clock().clock_type)
        age = (self.get_clock().now() - stamp).nanoseconds
        if age > self.max_age_ns or age < -self.slop_ns:
            self._fail("obstacle_stamp_out_of_range", stamp, source.frame_id)
            return
        info = self.camera_info.get(source.frame_id)
        if info is None:
            self._fail("waiting_for_matching_camera_info", stamp, source.frame_id)
            return
        group = matched_clouds(self.clouds.values(), source.stamp_ns, self.slop_ns)
        if group is None:
            self._fail("waiting_for_synchronized_lane_clouds", stamp, source.frame_id)
            return
        if len(group[0].points) < 2 or any(len(cloud.points) < 2 for cloud in group[1:]):
            why = "missing_centerline" if len(group[0].points) < 2 else "missing_lane_boundary"
            self._fail(why, stamp, source.frame_id)
            return
        aligned = align_clouds(group, self.odometry, source.stamp_ns, self.max_odom_gap_ns)
        if aligned is None:
            self._fail("waiting_for_odometry_alignment", stamp, source.frame_id)
            return
        try:
            transform = self.tf_buffer.lookup_transform(self.frame, source.frame_id, stamp).transform
        except (LookupException, ConnectivityException, ExtrapolationException):
            self._fail("waiting_for_camera_tf", stamp, source.frame_id)
            return
        origin = transform.translation
        rotation = _quaternion(transform.rotation)
        background = self._background_for(source.stamp_ns, source.frame_id)
        signature = (source.stamp_ns, source.frame_id, use_mask, background is not None,
                     *(cloud.stamp_ns for cloud in group),
                     info.width, info.height, tuple(info.p), origin.x, origin.y, origin.z, rotation)
        if signature == self.last_signature:
            return
        start = perf_counter()
        try:
            camera = GroundCamera(info.width, info.height, info.p, (origin.x, origin.y, origin.z),
                                  rotation_matrix(rotation), self.roi)
            obstacles = source.pixels if use_mask else boxes_to_mask(source.detections, info.width, info.height)
            result = plan(*aligned, obstacles, self.config, camera)
        except (ValueError, TypeError, KeyError, np.linalg.LinAlgError) as error:
            result = PlanResult("invalid_planner_input", metrics={"detail": str(error)})
        result.metrics.update(planning_ms=(perf_counter() - start)*1000, obstacle_age_ms=age/1e6,
                              obstacle_source="mono8_mask" if use_mask else "rasterized_boxes",
                              sync_offset_ms=[(c.stamp_ns-source.stamp_ns)/1e6 for c in group])
        self.last_signature = signature
        self._publish(result, stamp, source.frame_id, background)

    def _fail(self, reason, stamp=None, camera_frame=""):
        background = self._background_for(stamp.nanoseconds, camera_frame) if stamp is not None else None
        signature = (reason, camera_frame, stamp.nanoseconds if stamp is not None else 0,
                     background is not None)
        if self.last_signature == signature:
            return
        self.last_signature = signature
        self._publish(PlanResult(reason), stamp or self.get_clock().now(), camera_frame, background)

    def _path(self, poses, stamp):
        path = Path()
        path.header.stamp = stamp.to_msg()
        path.header.frame_id = self.frame
        for x, y, yaw in poses:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw/2)
            pose.pose.orientation.w = math.cos(yaw/2)
            path.poses.append(pose)
        return path

    @staticmethod
    def _image(pixels, stamp, frame, encoding):
        message = Image()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = frame
        message.height, message.width = pixels.shape[:2]
        message.encoding = encoding
        message.step = message.width * (3 if encoding == "rgb8" else 1)
        message.data = pixels.tobytes()
        return message

    def _publish(self, result, stamp, camera_frame, background=None):
        self.path_pub.publish(self._path(result.path, stamp))
        info = self.camera_info.get(camera_frame)
        if not result.band.size and (info is not None or background is not None):
            shape = (info.height, info.width) if info is not None else background.shape[:2]
            result.band = np.zeros(shape, dtype=np.uint8)
            result.allowed = result.obstacles = np.zeros(shape, dtype=bool)
        self.band_pub.publish(self._image(result.band, stamp, camera_frame, "mono8"))
        self.overlay_pub.publish(self._image(render_overlay(result, background), stamp, camera_frame, "rgb8"))
        status = String()
        status.data = json.dumps(dict(reason=result.reason, has_candidate=bool(len(result.path)),
                                      stamp_ns=stamp.nanoseconds, mode="camera_shadow", **result.metrics),
                                 allow_nan=False)
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ShadowPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
