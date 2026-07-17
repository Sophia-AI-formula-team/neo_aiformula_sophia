#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd $(dirname $0); pwd)
VECTORNAV_DEVICE=${VECTORNAV_DEVICE:-/dev/ttyUSB0}

run_privileged() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    else
        sudo "$@"
    fi
}

# CAN
bash ${SCRIPT_DIR}/can_bringup.sh

# IMU
for _ in $(seq 1 20); do
    if [ -e "${VECTORNAV_DEVICE}" ]; then
        break
    fi
    sleep 0.5
done

if ! [ -e "${VECTORNAV_DEVICE}" ]; then
    echo "VectorNav device ${VECTORNAV_DEVICE} was not found" >&2
    exit 1
fi

run_privileged chmod 666 "${VECTORNAV_DEVICE}"
