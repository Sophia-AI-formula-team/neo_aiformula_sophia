#!/bin/bash
# Run inside a container with --network none, the checkout at /neo, and the exchange at /evidence.
set -e
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=69 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
export ROS_HOME=/tmp/concept-gate-1120/ros ROS_LOG_DIR=/tmp/concept-gate-1120/log
export PYTHONPATH="/neo/vehicle_concept/src/concept_control_gate:/neo/vehicle_concept/src/concept_vehicle_state:/neo/workspace/src/aiformula/control/motor_controller:${PYTHONPATH}"
exec python3 /neo/vehicle_concept/checks/unity1120/replay_gate.py "$@"
