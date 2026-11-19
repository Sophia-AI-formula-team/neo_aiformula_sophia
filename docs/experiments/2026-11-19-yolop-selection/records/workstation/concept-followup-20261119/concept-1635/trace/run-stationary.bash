#!/bin/bash
set -e
cd "$(dirname "$0")"
source ~/concept_v6275_ws/activate_concept.bash
source ~/aiformula_runtime/concept-field-1635/install/local_setup.bash
export ROS_DOMAIN_ID=207 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix concept)/share/concept/config/vehicle-fastdds.xml"
setsid ros2 launch "$PWD/stationary-replay.launch.py" mode:=replay onboard_camera:=false > launch.log 2>&1 &
group=$!
cleanup(){ kill -INT -- "-$group" 2>/dev/null || true; sleep 2; kill -TERM -- "-$group" 2>/dev/null || true; }
trap cleanup EXIT
python3 stationary-replay.py
