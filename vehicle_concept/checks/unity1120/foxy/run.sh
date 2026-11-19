#!/bin/bash
set -eo pipefail
source /opt/ros/foxy/setup.bash
for package_dir in /opt/concept/src/concept*; do
    export PYTHONPATH="$package_dir:$PYTHONPATH"
done
if [ "$#" -gt 0 ]; then exec "$@"; fi
exec python3 /opt/concept/unity/bridge.py
