#!/usr/bin/env bash
# Install only OA 1B into a versioned prefix. The field agent separately creates its shortcuts.
set -eo pipefail
here=$(cd -- "$(dirname -- "$(readlink -f -- "$0")")" && pwd)
repo=$(git -C "$here" rev-parse --show-toplevel)
rel=workspace/src/aiformula/perception/oa_1b
src="$repo/$rel"
rev=$(git -C "$repo" rev-parse HEAD)
if [[ -n $(git -C "$repo" status --porcelain --untracked-files=all -- "$rel" "${here#"$repo"/}") ]]; then
    echo 'OA 1B source or installation instructions differ from the selected Git revision.' >&2
    exit 1
fi
base=/home/workspace/install
field="$HOME/aiformula_runtime/2026-11-20-mapassist-av314/install"
a3="$HOME/aiformula_runtime/yolop_a3_v14_20261012"
release="$HOME/aiformula_runtime/2026-11-21-oa1b/$rev"
source /opt/ros/foxy/setup.bash
source "$base/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
check_sha() { printf '%s  %s\n' "$1" "$2" | sha256sum -c -; }
check_sha af3dd1c8f990cd5d7fa1fdd886f682ce580e448757635b01db85c5a2d1e9fb64 "$base/launchers/share/launchers/launch/allnodes.launch.py"
check_sha b4f133d7f57e2b292b005a86133961db373948017c0440895bd9b336d4e42576 "$base/trajectory_follower/lib/python3.8/site-packages/trajectory_follower/lya_follower_connected_omegat_global.py"
check_sha a411c983b661d6b26217019a968251c89a6b66c95400e4e7cb78f04a644ca10c "$a3/upstream/YOLOP_A3_Deploy/weights/shared_best.pth"
[[ -x "$a3/venv/bin/python3" && -r "$field/local_setup.bash" ]]
[[ -r "$HOME/aiformula_runtime/2026-11-19-yolop-selection/yolop.launch" ]]
motor=$(/usr/bin/python3 -c 'from pathlib import Path; import motor_controller; print(Path(motor_controller.__file__).resolve().parent)')
mkdir -p -- "$release"
exec 9>"$release/install.lock"
flock -n 9 || { echo 'Another OA 1B preparation is running for this revision' >&2; exit 1; }
if [[ ! -f "$release/READY" ]]; then
    (cd "$release" && colcon --log-base "$release/log" build --base-paths "$src" --packages-select oa_1b \
        --merge-install --build-base "$release/build" --install-base "$release/install")
    /usr/bin/python3 "$release/install/share/oa_1b/tools/prepare_control.py" build \
        --field-install "$field" --motor-package "$motor" --dest "$release/control"
    install -m 755 "$here/entry" "$here/stage.sh" "$release"
fi
cmp "$here/entry" "$release/entry"
cmp "$here/stage.sh" "$release/stage.sh"
bash -e -c 'source "$1/install/local_setup.bash"; export PYTHONPATH="$1/control/python:$PYTHONPATH"; \
    /usr/bin/python3 "$1/install/share/oa_1b/tools/prepare_control.py" verify --dest "$1/control" --role motor' _ "$release"
bash -e -c 'source "$2/local_setup.bash"; source "$1/install/local_setup.bash"; export PYTHONPATH="$1/control/python:$PYTHONPATH"; \
    /usr/bin/python3 "$1/install/share/oa_1b/tools/prepare_control.py" verify --dest "$1/control" --role planner' _ "$release" "$field"
(
    source "$release/install/local_setup.bash"
    export PYTHONPATH="$a3/ros_install/yolop_a3_perception/lib/python3.8/site-packages:${PYTHONPATH:-}"
    export PYTHONNOUSERSITE=1
    if [[ $(uname -m) == aarch64 ]]; then export LD_PRELOAD="/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"; fi
    "$a3/venv/bin/python3" - "$release" <<'PY'
from pathlib import Path
import sys
from oa_1b import graph, perception, supervise

release = Path(sys.argv[1]).resolve()
for module in (graph, perception, supervise):
    if release / "install" not in Path(module.__file__).resolve().parents:
        sys.exit("Unexpected OA 1B module: " + module.__file__)
perception.verify_field_runtime()
print("OA 1B perception ready:", perception.__file__)
PY
)
printf '%s\n' "$rev" > "$release/READY"
printf 'OA 1B ready: %s\n' "$release/entry"
