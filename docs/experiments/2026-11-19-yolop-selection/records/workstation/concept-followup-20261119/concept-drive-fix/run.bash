#!/bin/bash
set -e
cd "$(dirname "$0")"
source ~/concept_v6275_ws/activate_concept.bash
source ~/aiformula_runtime/concept-drive-fix-20261119-1603/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=198 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix concept)/share/concept/config/vehicle-fastdds.xml"
export RUN_DIR="$PWD/$1" TEST_CAMERA_HZ="$2" RUN_SECS="$3" PROFILE_CAMERA="$4" ATTEMPTS="${5:-1}"
mkdir "$RUN_DIR"
if pgrep -af '/(zed_wrapper|motor_controller|socket_can_sender_node_exe|joy_node|vectornav)($| )'; then exit 2; fi
date -Is
setsid ros2 launch "$PWD/bench.launch.py" > "$RUN_DIR/launch.log" 2>&1 &
group=$!
cleanup() {
 kill -INT -- "-$group" 2>/dev/null || true
 sleep 4
 kill -TERM -- "-$group" 2>/dev/null || true
 wait || true
 date -Is
}
trap cleanup EXIT
python3 bench.py
