#!/usr/bin/env bash
set -eo pipefail
here=$(cd "$(dirname "$0")" && pwd)
repo=$(git -C "$here" rev-parse --show-toplevel)
src="$repo/workspace/src/aiformula"
semantic="$src/perception/semantic_planning_system"
runtime="$HOME/aiformula_runtime/2026-11-20-mapassist-av314"
source /opt/ros/foxy/setup.bash
source /home/workspace/install/setup.bash
mkdir -p "$runtime"
cd "$runtime"
colcon build --base-paths "$semantic/semantic_planner_core" "$semantic/semantic_local_planner_ros" \
    "$src/control/semantic_vehicle_wiring" "$src/launchers" "$src/perception/obsticle_avoidence"
install -m 644 "$here/check_vehicle.py" "$runtime/check_vehicle.py"
source "$runtime/install/local_setup.bash"
for pkg in semantic_planner_core semantic_local_planner_ros semantic_vehicle_wiring launchers obsticle_avoidence; do
    [[ "$(ros2 pkg prefix "$pkg")" == "$runtime/install/$pkg" ]] || { echo "Unexpected $pkg prefix" >&2; exit 1; }
done
oa_module=$(/usr/bin/python3 -c 'from obsticle_avoidence import oa_2; print(oa_2.__file__)')
cmp "$src/perception/obsticle_avoidence/obsticle_avoidence/oa_2.py" "$oa_module"
base=/home/workspace/install/launchers/share/launchers/launch/allnodes.launch.py
grep -Fq lane_0927oa "$base" || { echo "Expected the 11/19 vehicle Allnodes" >&2; exit 1; }
ros2 launch "$base" --show-args | grep 'lane_point_topic_prefix'
ros2 pkg executables obsticle_avoidence | grep -Fx 'obsticle_avoidence oa_2'
ros2 pkg executables trajectory_follower | grep -Fx 'trajectory_follower lya_follower_connected_omegat_global'
ros2 launch launchers semantic_045_commissioning.launch.py --show-args
/usr/bin/python3 "$here/deployment_receipt.py" "$repo" "$runtime" "$oa_module"
python3 "$here/shortcuts.py" --apply
