from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "image_topic": "/CAM_FRONT/image_rect_compressed",
        "info_topic": "/CAM_FRONT/camera_info", "image_kind": "compressed",
        "body_frame": "base_link",
        "inference_provider": "auto", "inference_threads": "3",
        "use_sim_time": "true", "vehicle_width_m": "1.75", "planning_origin_x_m": "3.0",
        "analysis_width": "800", "scan_near_m": "2.0", "scan_far_m": "16.0",
        "lane_width_m": "3.35", "lookahead_m": "11.5", "clearance_m": "0.1",
        "orange_cone_candidates": "false", "camera_info_is_static": "false",
        "length_weight": "1.0", "curvature_weight": "0.25", "solid_weight": "0.70",
    }
    declarations = [DeclareLaunchArgument("model", description="Path to the YOLOP ONNX weights")]
    declarations.extend(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items())
    common = dict(use_sim_time=ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool))
    semantic = dict(common, **{k: LaunchConfiguration(k) for k in
                               ("model", "image_topic", "info_topic", "image_kind", "body_frame", "inference_provider")})
    semantic.update({k: ParameterValue(LaunchConfiguration(k), value_type=t) for k, t in
                     (("inference_threads", int), ("analysis_width", int), ("scan_near_m", float), ("scan_far_m", float),
                      ("lane_width_m", float), ("orange_cone_candidates", bool), ("camera_info_is_static", bool))})
    plan = dict(common, **{k: ParameterValue(LaunchConfiguration(k), value_type=float)
                          for k in ("vehicle_width_m", "planning_origin_x_m", "lookahead_m", "clearance_m",
                                    "length_weight", "curvature_weight", "solid_weight")})
    nodes = [Node(package="aiformula_frontend_demo", executable=name,
                  parameters=[options], output="screen")
             for name, options in (("semantic", semantic), ("opti_oa", plan))]
    return LaunchDescription(declarations+nodes)
