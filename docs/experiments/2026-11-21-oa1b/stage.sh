#!/usr/bin/env bash
set -eo pipefail
release=$(cd -- "$(dirname -- "$(readlink -f -- "$0")")" && pwd)
stage=$1
shift
base=/home/workspace/install
field="$HOME/aiformula_runtime/2026-11-20-mapassist-av314/install"
a3="$HOME/aiformula_runtime/yolop_a3_v14_20261012"
prefix=/semantic_drive/map_assist_045
control="$release/control"
checker="$release/install/share/oa_1b/tools/prepare_control.py"
source /opt/ros/foxy/setup.bash
source "$base/setup.bash"
if [[ $stage == oa1b ]]; then
    source "$field/local_setup.bash"
    for p in launchers semantic_local_planner_ros semantic_planner_core semantic_vehicle_wiring obsticle_avoidence; do
        [[ "$(ros2 pkg prefix "$p")" == "$field/$p" ]] || { echo "Unexpected field prefix: $p" >&2; exit 1; }
    done
fi
source "$release/install/local_setup.bash"
export PYTHONDONTWRITEBYTECODE=1
case "$stage" in
    allnodes)
        export PYTHONPATH="$control/python:${PYTHONPATH:-}"
        /usr/bin/python3 "$checker" verify --role motor --dest "$control"
        /usr/bin/python3 -m oa_1b.graph allnodes
        launch="$base/launchers/share/launchers/launch/allnodes.launch.py"
        exec ros2 launch "$launch" motor_cmd_topic:="$prefix/diagnostic_cmd_vel" \
            lane_point_topic_prefix:=/original_processed_point_ launch_yolop_a3:=false ;;
    yolop)
        # Stock Allnodes allows 120 seconds for can0 during a cold start.
        /usr/bin/python3 -m oa_1b.graph yolop --vehicle --timeout 150
        export PYTHONPATH="$a3/ros_install/yolop_a3_perception/lib/python3.8/site-packages:${PYTHONPATH:-}"
        export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
        if [[ $(uname -m) == aarch64 ]]; then export LD_PRELOAD="/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"; fi
        exec "$a3/venv/bin/python3" -m oa_1b.perception --ros-args -r __ns:=/aiformula_perception \
            -p "weights:=$a3/upstream/YOLOP_A3_Deploy/weights/shared_best.pth" -p device:=cuda -p score_threshold:=0.5 \
            -p input_image_topic:=/aiformula_sensing/zed_node/left_image/undistorted \
            -r pub_mask_image:=/aiformula_perception/road_detector/mask_image \
            -r pub_annotated_mask_image:=/aiformula_visualization/road_detector/annotated_mask_image \
            -r /traffic_light/state:=/aiformula_perception/yolop_a3/traffic_light/state \
            -r /red_pixels_count:=/aiformula_perception/yolop_a3/red_pixels_count ;;
    oa1b)
        export PYTHONPATH="$control/python:${PYTHONPATH:-}"
        /usr/bin/python3 "$checker" verify --role planner --dest "$control"
        /usr/bin/python3 -m oa_1b.graph oa1b --vehicle --timeout 90
        exec ros2 launch "$control/launch/oa1b.launch.py" lane:="${1:-outer}" red_pixel_stop_threshold:="${2:-50}" \
            vehicle_output:=true start_oa2:=true oa_algorithm:=oa_2_1 oa_initially_idle:=false start_lya:=false \
            lane_point_topic_prefix:=/original_processed_point_ ;;
    lya)
        /usr/bin/python3 -m oa_1b.graph lya --vehicle --timeout 45
        exec ros2 run trajectory_follower lya_follower_connected_omegat_global --ros-args \
            -r "/filtered_lane_pose:=$prefix/filtered_lane_pose" -r "/filtered_omega_t:=$prefix/filtered_omega_t" \
            -r "/aiformula_control/game_pad/cmd_vel:=$prefix/lya_cmd_vel" ;;
    verify) exec /usr/bin/python3 -m oa_1b.graph live --vehicle ;;
    *) exit 2 ;;
esac
