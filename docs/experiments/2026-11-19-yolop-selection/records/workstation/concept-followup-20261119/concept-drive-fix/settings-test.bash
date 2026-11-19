#!/bin/bash
set -e
cd "$(dirname "$0")"
source ~/concept_v6275_ws/activate_concept.bash
source ~/aiformula_runtime/concept-drive-fix-20261119-1603/install/local_setup.bash
run=$(mktemp -d /tmp/concept-speed-settings.XXXXXXXX)
export XDG_CONFIG_HOME="$run/config" XDG_RUNTIME_DIR="$run/qt" QT_QPA_PLATFORM=offscreen
export ROS_LOG_DIR="$run/ros-logs" CONCEPT_BAG_DIR="$run/bags" ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix concept)/share/concept/config/foxy-fastdds.xml"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 SETTINGS_PROBE_LOG="$run"
mkdir -m 700 "$XDG_RUNTIME_DIR"
echo "$run"
group=''
stop_nodes() {
 if [ -n "$group" ]; then
   kill -INT -- "-$group" 2>/dev/null || true
   for attempt in {1..50}; do
     if ! kill -0 "$group" 2>/dev/null; then break; fi
     sleep .1
   done
   kill -TERM -- "-$group" 2>/dev/null || true
   wait "$group" || true
   group=''
 fi
}
trap stop_nodes EXIT
start_nodes() {
 local profile=$1
 shift
 setsid ros2 launch "$PWD/settings-probe.launch.py" mode:="$profile" onboard_camera:=false panel:=false rviz:=false "$@" > "$run/launch-$ROS_DOMAIN_ID.log" 2>&1 &
 group=$!
}
panel() {
 python3 -u settings-panel-probe.py "$@" | tee -a "$run/events.jsonl"
 test "${PIPESTATUS[0]}" -eq 0
}
export ROS_DOMAIN_ID=201
start_nodes vehicle
panel first-save 4.0 4.0 --persistent --save 2.3 1.7
panel panel-process-reopened 2.3 1.7 --persistent
stop_nodes
export ROS_DOMAIN_ID=202
start_nodes vehicle
panel driver-and-gate-restarted 2.3 1.7 --persistent
stop_nodes
export ROS_DOMAIN_ID=203
start_nodes vehicle target_speed_mps:=3.1
panel one-explicit-override 3.1 1.7 --persistent
stop_nodes
export ROS_DOMAIN_ID=204
start_nodes replay
panel replay-with-vehicle-files-present 4.0 4.0 --save 1.2 1.4
stop_nodes
export ROS_DOMAIN_ID=205
start_nodes vehicle
panel vehicle-after-replay 2.3 1.7 --persistent --save 4.0 4.0
stop_nodes
export ROS_DOMAIN_ID=206
start_nodes vehicle
panel restored-4mps-after-restart 4.0 4.0 --persistent
stop_nodes
date -Is
