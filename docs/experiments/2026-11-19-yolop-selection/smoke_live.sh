#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/foxy/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=191 PYTHONDONTWRITEBYTECODE=1
./yolop.launch a4 > a4.vehicle-topics.log 2>&1 &
detector=$!
trap 'kill -TERM "$detector" 2>/dev/null || true; wait "$detector" 2>/dev/null || true' EXIT
/usr/bin/python3 verify_live_routes.py --isolated
status=0
./yolop.launch a3 > duplicate-selection.log 2>&1 || status=$?
[[ "$status" == 1 ]]
rg -q 'already running' duplicate-selection.log
