#!/bin/bash
set -euo pipefail

CAN_INTERFACE=${CAN_INTERFACE:-can0}
CAN_BITRATE=${CAN_BITRATE:-500000}

run_privileged() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    else
        sudo "$@"
    fi
}

run_privileged modprobe kvaser_usb

for _ in $(seq 1 20); do
    if ip link show "${CAN_INTERFACE}" >/dev/null 2>&1; then
        break
    fi
    sleep 0.5
done

if ! ip link show "${CAN_INTERFACE}" >/dev/null 2>&1; then
    echo "CAN interface ${CAN_INTERFACE} was not found" >&2
    exit 1
fi

run_privileged ip link set "${CAN_INTERFACE}" down >/dev/null 2>&1 || true
run_privileged ip link set "${CAN_INTERFACE}" type can bitrate "${CAN_BITRATE}"
run_privileged ip link set "${CAN_INTERFACE}" up
