from dataclasses import dataclass


@dataclass(frozen=True)
class RosTopics:
    input_image: str = "/aiformula_sensing/zed_node/left_image/undistorted"
    lane_overlay: str = "pub_annotated_mask_image"
    lane_mask: str = "pub_mask_image"
    object_overlay: str = "/cone_bbox_image"
    obstacle_mask: str = "/cone_mask"
    red_pixels_count: str = "/red_pixels_count"
    traffic_light_state: str = "/traffic_light/state"
    detections_json: str = "/aiformula_perception/yolop_a2/detections"


ROS_TOPICS = RosTopics()

ROS_ENCODINGS = {
    "input_image": "bgr8",
    "lane_overlay": "bgr8",
    "lane_mask": "mono8",
    "object_overlay": "bgr8",
    "obstacle_mask": "mono8",
}
