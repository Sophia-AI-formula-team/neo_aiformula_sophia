#!/usr/bin/env bash
# Build and verify the pinned 11/20 Concept overlay. No ROS nodes are started.
set -eo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
url=https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git
branch=feat/concept-v626-live
pin=f305d5f7acea5c87af0e2003bec13bdc06199d8f
repo="$HOME/src/neo-concept-1120-v6278"
ws="$HOME/concept_1120_v6278_ws"
weights=
fail() { echo "Concept preparation: $*" >&2; exit 1; }
case $# in
    0) ;;
    1) if [[ $1 == --help || $1 == -h ]]; then
           echo "Usage: bash prepare-concept.sh [--weights B1.4-road-aligned-research.pth|V6.26.zip]"
           exit 0
       fi
       fail "Use --weights followed by the local model or V6.26 ZIP." ;;
    2) [[ $1 == --weights && -f $2 ]] || fail "Use --weights with an existing model or V6.26 ZIP."
       weights=$(realpath -- "$2") ;;
    *) fail "See --help." ;;
esac
for required in git flock python3 xdg-user-dir; do command -v "$required" >/dev/null || fail "Missing $required"; done
[[ -r "$here/shortcuts.py" && -r "$here/activate_concept.bash" ]] || fail "Keep the preparation files together."
mkdir -p "$HOME/src" "$ws"
exec 9>"$ws/.prepare-concept.lock"
flock -n 9 || fail "Preparation is already running in $ws"

if [[ ! -e "$repo" ]]; then
    staged=$(mktemp -d "$HOME/src/.neo-concept-1120-v6278.XXXXXX")
    git clone --depth 16 --no-checkout --single-branch --branch "$branch" --filter=blob:none "$url" "$staged"
    # A future branch tip may have moved beyond the initial shallow window.
    if ! git -C "$staged" merge-base --is-ancestor "$pin" "origin/$branch" 2>/dev/null; then
        git -C "$staged" fetch --unshallow --filter=blob:none origin "$branch"
    fi
    git -C "$staged" merge-base --is-ancestor "$pin" "origin/$branch" || fail "Pinned commit is absent from $branch; staging directory: $staged"
    git -C "$staged" sparse-checkout init --cone
    git -C "$staged" sparse-checkout set vehicle_concept workspace/src/aiformula/launchers \
        workspace/src/aiformula/vehicle workspace/src/aiformula/control/motor_controller
    git -C "$staged" checkout --detach "$pin"
    mv -T -- "$staged" "$repo"
fi
[[ $(git -C "$repo" rev-parse --show-toplevel) == "$(realpath "$repo")" ]] || fail "Unexpected source directory: $repo"
[[ $(git -C "$repo" rev-parse HEAD) == "$pin" ]] || fail "Existing $repo has a different HEAD; it was preserved."
remote=$(git -C "$repo" config --get remote.origin.url)
case "$remote" in "$url"|https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia|git@github.com:Sophia-AI-formula-team/neo_aiformula_sophia.git) ;; *) fail "Unexpected origin at $repo: $remote" ;; esac
changes=$(git -C "$repo" status --porcelain --untracked-files=normal)
[[ -z "$changes" ]] || fail "Existing checkout has local changes; preserved without building."
[[ -f "$repo/workspace/src/aiformula/launchers/CMakeLists.txt" && -f "$repo/vehicle_concept/checks/check_overlay.py" ]] ||
    fail "Existing sparse checkout is missing vehicle_concept or workspace/src/aiformula/launchers."

source "$here/activate_concept.bash" --base-for-build
command -v colcon >/dev/null || fail "Missing colcon in the Foxy build environment"
python3 -c 'import Cython, scipy, torch, torchvision; print("Python dependencies:", Cython.__version__,scipy.__version__,torch.__version__,torchvision.__version__)'
cd "$ws"
# 11/19 used a private launchers overlay over the vehicle's motor/vehicle packages.
colcon build --base-paths "$repo/vehicle_concept/src" "$repo/workspace/src/aiformula/launchers" --parallel-workers 2
source "$here/activate_concept.bash"
[[ $(ros2 pkg prefix launchers) == "$ws/install/launchers" ]] || fail "Unexpected launchers overlay"
python3 "$repo/vehicle_concept/checks/check_underlay.py"
python3 "$repo/vehicle_concept/checks/check_overlay.py" "$ws"
if [[ -n "$weights" ]]; then
    ros2 run concept_yolop concept_yolop_weights "$weights"
fi
python3 - <<'PY'
import hashlib
from pathlib import Path
import concept_ranker
from concept_yolop.weights import default_path, verify
h12 = Path(concept_ranker.__file__).with_name('model-h12.json')
if hashlib.sha256(h12.read_bytes()).hexdigest() != 'e87f84dd1a1e3ef4a7df21ca9b879f9e7b142573501937e1985e74632aec0df6':
    raise SystemExit(f'H12 checksum mismatch: {h12}')
print('H12 verified:', h12)
p = default_path()
if not p.is_file():
    raise SystemExit(f"Missing B1.4 at {p}; re-run prepare-concept.sh --weights /path/to/checkpoint-or-zip")
with p.open('rb') as source:
    verify(source)
print("B1.4 verified:", p)
PY
ros2 run concept_yolop concept_yolop_check "$repo/vehicle_concept/src/concept_drive/test/data/moving-lap-start/00000_rgb.png" --device cuda
python3 -c 'import torch; from cv_bridge import CvBridge; from concept_panel.qt import QtWidgets; print("Qt + Torch + cv_bridge imports OK")'
args=$(ros2 run concept live --show-args)
printf '%s\n' "$args"
for key in tracking_controller planning_hz cuda_fusion grab_frame_rate camera_info_topic; do
    grep -Fq "'$key':" <<<"$args" || fail "Installed launch is missing $key"
done
grep -Fq 'auto shows a terminal menu; pure_pursuit or lya selects directly' <<<"$args" || fail "Controller selection missing"
install -m 644 "$here/activate_concept.bash" "$ws/activate_concept.bash.tmp"
mv -T -- "$ws/activate_concept.bash.tmp" "$ws/activate_concept.bash"
python3 "$here/shortcuts.py" --apply
python3 "$here/shortcuts.py" --check
printf 'Ready: %s  %s\n' "$pin" "$ws"
