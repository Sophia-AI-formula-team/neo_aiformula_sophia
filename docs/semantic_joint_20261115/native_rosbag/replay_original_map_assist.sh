#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 run semantic_local_planner_ros map_assist --ros-args \
  --params-file "$task_dir/replay_map_assist.yaml" \
  -p "topomap_path:=$task_dir/aiformula_topomap.osm"
