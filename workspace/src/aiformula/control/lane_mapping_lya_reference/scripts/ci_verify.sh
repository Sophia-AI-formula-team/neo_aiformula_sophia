#!/usr/bin/env bash
# Run inside disposable ROS Foxy CI. The actual neo LYA is exercised only on
# private localhost DDS topics; no sensor drivers, inference or motors start.
set -eo pipefail
repo=$(cd "${1:?pass repository root}" && pwd)
ws=/tmp/lane_teach_repeat_ws
mkdir -p "$ws/evidence"
cd "$ws"
source /opt/ros/foxy/setup.bash
colcon build --event-handlers console_direct+ --base-paths \
  "$repo/dependencies/vectornav/vectornav_msgs" \
  "$repo/workspace/src/aiformula/control/trajectory_follower" \
  "$repo/workspace/src/aiformula/control/lane_mapping_lya_reference" \
  "$repo/workspace/src/aiformula/control/lane_mapping_fixed" \
  --packages-select vectornav_msgs trajectory_follower lane_mapping_lya_reference lane_mapping_fixed
source install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure \
  --base-paths "$repo/workspace/src/aiformula/control/lane_mapping_lya_reference" \
  "$repo/workspace/src/aiformula/control/lane_mapping_fixed" --packages-select \
  lane_mapping_lya_reference lane_mapping_fixed
colcon test-result --verbose
# An empty source discovery can make colcon report success with zero tests.
# Require the actual pytest result as well, not just a green command exit code.
python3 -c 'from pathlib import Path; import xml.etree.ElementTree as E; files=list(Path("build").rglob("*.xml")); count=sum(int(s.get("tests", "0")) for p in files for s in E.parse(p).getroot().iter("testsuite")); assert count >= 325, "Expected at least 325 tests, found {}".format(count); print("Verified recorded unit-test count:", count)'
ros2 pkg executables lane_mapping_lya_reference
ros2 pkg executables lane_mapping_fixed
ros2 pkg executables trajectory_follower
ros2 launch lane_mapping_lya_reference reference.launch.py --show-args
ros2 launch lane_mapping_fixed fixed.launch.py --show-args
# Exercise the real publication methods offline, with an explicitly doubled
# inference boundary. JUnit and stdout distinguish this from the DDS test below.
python3 -m pytest "$repo/workspace/src/aiformula/perception/road_detector/test/test_output_contract.py" \
  -q --junitxml="$ws/evidence/road_detector_header_unit.xml" \
  2>&1 | tee "$ws/evidence/road_detector_header_unit.log"
python3 "$repo/workspace/src/aiformula/control/lane_mapping_lya_reference/test/runtime_smoke.py" \
  --output "$ws/evidence/runtime.json"
python3 "$repo/workspace/src/aiformula/control/lane_mapping_lya_reference/test/neo_upstream_smoke.py" \
  --road-detector-source "$repo/workspace/src/aiformula/perception/road_detector/road_detector/road_detector.py" \
  --output "$ws/evidence/neo_upstream_runtime.json"
python3 "$repo/workspace/src/aiformula/control/lane_mapping_lya_reference/scripts/control_demo.py" \
  --output "$ws/evidence/control_demo"
