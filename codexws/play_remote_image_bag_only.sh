#!/usr/bin/env bash
set -eo pipefail

readonly BAG_DIR="/media/control/R/OWEN/rosbag2_2026_06_29-17_48_44"
readonly IMAGE_TOPIC="/aiformula_sensing/zed_node/left_image/undistorted"

source /opt/ros/foxy/setup.bash
source /home/workspace/install/local_setup.bash
set -u

if pgrep -af "[r]os2 bag play" >/dev/null; then
  echo "Refusing to start: a ros2 bag play process is already running." >&2
  exit 1
fi

echo "Playing only: ${IMAGE_TOPIC}"
echo "Bag: ${BAG_DIR}"
echo "Excluded recorded topic: /aiformula_control/game_pad/cmd_vel"

exec ros2 bag play "${BAG_DIR}" \
  --topics "${IMAGE_TOPIC}" \
  --rate 1.0 \
  --loop
