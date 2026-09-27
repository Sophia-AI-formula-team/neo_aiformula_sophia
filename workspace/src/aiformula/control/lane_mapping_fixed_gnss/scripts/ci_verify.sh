#!/usr/bin/env bash
# Isolated CI only; no hardware driver, real LYA or actuator subscription.
set -eo pipefail
repo=$(cd "${1:?repository root}" && pwd)
ws=/tmp/lane_endpoint_gnss_ws
mkdir -p "$ws/evidence"
cd "$ws"
source /opt/ros/foxy/setup.bash
colcon build --event-handlers console_direct+ --base-paths \
  "$repo/dependencies/vectornav/vectornav_msgs" \
  "$repo/workspace/src/aiformula/control/lane_mapping_lya_reference" \
  "$repo/workspace/src/aiformula/control/lane_mapping_fixed" \
  "$repo/workspace/src/aiformula/control/lane_mapping_fixed_gnss" \
  --packages-select vectornav_msgs lane_mapping_lya_reference lane_mapping_fixed lane_mapping_fixed_gnss
source install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure \
  --base-paths "$repo/workspace/src/aiformula/control/lane_mapping_lya_reference" \
  "$repo/workspace/src/aiformula/control/lane_mapping_fixed" "$repo/workspace/src/aiformula/control/lane_mapping_fixed_gnss" \
  --packages-select lane_mapping_lya_reference lane_mapping_fixed lane_mapping_fixed_gnss
colcon test-result --verbose
python3 -c 'from pathlib import Path; import xml.etree.ElementTree as E; files=list(Path("build").rglob("*.xml")); count=sum(int(s.get("tests", "0")) for p in files for s in E.parse(p).getroot().iter("testsuite")); assert count >= 470, "Expected real test results, got {}".format(count); print("Recorded unit tests:", count)'
ros2 pkg executables lane_mapping_fixed_gnss
ros2 launch lane_mapping_fixed_gnss fixed_gnss.launch.py --show-args
python3 "$repo/workspace/src/aiformula/control/lane_mapping_fixed_gnss/test/runtime_smoke.py" --output "$ws/evidence/runtime.json"
