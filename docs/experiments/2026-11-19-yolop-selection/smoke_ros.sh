#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=187
for choice in a4 a4l; do
    status=0
    timeout -s INT -k 5s 22s ./yolop.launch "$choice" > "$choice.ros.log" 2>&1 || status=$?
    cat "$choice.ros.log"
    [[ "$status" == 124 ]]
    rg -qi 'single.frame' "$choice.ros.log"
done
