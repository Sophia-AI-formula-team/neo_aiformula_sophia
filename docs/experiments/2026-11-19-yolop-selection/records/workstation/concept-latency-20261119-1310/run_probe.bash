#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source "$HOME/concept_v6275_ws/activate_concept.bash"
export ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix concept)/share/concept/config/vehicle-fastdds.xml"
if pgrep -af '/(zed_wrapper|motor_controller|socket_can_sender_node_exe)($| )'; then
    exit 2
fi
label=${1:?supply a run label}
mkdir "$label"
children=()
stop_children() {
    for child in "${children[@]}"; do
        kill -INT -- "-$child" 2>/dev/null || true
    done
    for child in "${children[@]}"; do
        wait "$child" 2>/dev/null || true
    done
}
trap stop_children EXIT
date -Is
setsid ros2 launch "$PWD/latency_camera.launch.py" >"$label/zed.log" 2>&1 &
children+=("$!")
setsid timeout --signal=INT --kill-after=5 95 tegrastats --interval 500 >"$label/tegrastats.log" 2>&1 &
children+=("$!")
setsid timeout --signal=INT --kill-after=5 90 python3 latency_probe.py profile "$label/profile.json" >"$label/node.log" 2>&1 &
vision_pid=$!
children+=("$vision_pid")
sleep "${PROFILE_STARTUP_SECONDS:-12}"
kill -0 "$vision_pid"
if [[ "${PROFILE_PLANNER:-0}" == 1 ]]; then
    setsid timeout --signal=INT --kill-after=5 95 env OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
        ros2 run concept_drive drive --ros-args -p target_speed_mps:=1.0 >"$label/planner.log" 2>&1 &
    children+=("$!")
fi
timeout --signal=INT --kill-after=3 35 python3 latency_probe.py probe "$label/probe.json" --seconds "${PROFILE_SECONDS:-18}"
stop_children
children=()
python3 latency_probe.py summarize "$label" | tee "$label/summary.json"
date -Is
