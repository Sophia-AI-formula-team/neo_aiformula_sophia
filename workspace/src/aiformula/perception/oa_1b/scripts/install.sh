#!/usr/bin/env bash
# Prepare the new package only. Invoke as the existing vehicle user.
set -eo pipefail
src=$(cd -- "$(dirname -- "$(readlink -f -- "$0")")/.." && pwd)
repo=$(git -C "$src" rev-parse --show-toplevel)
rev=$(git -C "$repo" rev-parse HEAD)
if [[ -n "$(git -C "$src" status --porcelain --untracked-files=all -- .)" ]]; then
    echo 'Use a clean OA 1B checkout.' >&2
    exit 1
fi
release="$HOME/aiformula_runtime/2026-11-21-mapassist-b/$rev"
a3="$HOME/aiformula_runtime/yolop_a3_v14_20261012"
field="$HOME/aiformula_runtime/2026-11-20-mapassist-av314/install"
mkdir -p -- "$release"
exec 9>"$release/install.lock"
flock -n 9 || { echo 'An OA 1B installation is already running.' >&2; exit 1; }
source /opt/ros/foxy/setup.bash
source /home/workspace/install/setup.bash
source "$field/local_setup.bash"
[[ "$(ros2 pkg prefix launchers)" == "$field/launchers" ]]
pins="$field/launchers/share/launchers"
sha256sum --check <<EOF
bf2246348605be59552e0f71a34125f0a8f2bccca6d3cbb0edde1f4277d6fcc1  $pins/launch/semantic_045_commissioning.launch.py
409e91fb39a57379fa3e23cbaf36bfddd6bf930a39c488930946b4971997106b  $pins/launch/semantic_vehicle_045.launch.py
EOF
if [[ ! -f "$release/READY" ]]; then
    (cd -- "$release" && colcon --log-base "$release/log" build --base-paths "$src" --packages-select oa_1b \
         --merge-install --build-base "$release/build" --install-base "$release/install")
    install -m 755 "$src/scripts/mapassist_b" "$release/mapassist_b"
fi
cmp "$release/mapassist_b" "$src/scripts/mapassist_b"
source "$release/install/local_setup.bash"
[[ "$(ros2 pkg prefix oa_1b)" == "$release/install" ]]
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
if [[ $(uname -m) == aarch64 ]]; then
    export LD_PRELOAD="/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
fi
"$a3/venv/bin/python3" -m oa_1b.observer --check --weights "$a3/upstream/YOLOP_A3_Deploy/weights/shared_best.pth"
printf '%s\n' "$rev" > "$release/READY"
python3 "$src/scripts/shortcut.py" "$release"
printf 'OA 1B ready: %s\n' "$release/mapassist_b"
