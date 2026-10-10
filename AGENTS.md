# Vehicle integration handoff

## Scope and preservation

- Keep the vehicle-sourced `workspace/src/aiformula/perception/semantic_planning_system` contents unchanged during the 2026-10-05 YOLOP integration. It is an exact source snapshot, including its existing tests and documentation.
- `lane_points/lane_points/lane_0927oa.py` is an additional executable. Do not replace `lane_0215`, `lane_0529oa`, or other existing implementations with it.
- Preserve existing detector code and weights before deploying a new version. Use separate directories and reversible launch selection, and record exact backup and rollback paths.
- Do not use bag smoke tests for this integration. Acceptance is live camera input, lane mask Image output, compatible object mask/annotated boxes, then a separately launched obstacle-avoidance node producing its spline output.
- Maintain the integration status, commands, commit/weight hashes, validation results, and rollback instructions in `docs/agent_handoff_20261005.md`. Never describe a planned step or a static check as a passed live test.

## Deployment state to preserve

- YOLOP A2 comes from PR #11, publication commit `06fa28bdc09c731249428b7d81850d23e4e216a3`, imported into the PR #9 integration worktree. Preserve the original 31 publication files and checkpoint bytes; hashes are in `docs/yolop_a2_upstream_snapshot_20261005.json`. These and the 100 vehicle-source files match their snapshots on both hosts.
- The A2 runtime is deployed separately under `perception/road_detector/road_detector/YOLOP_A2_Deploy`; vehicle source and install legacy directories are named `yolop_legacy_20261005`. Do not remove them or replace the old detector script/weight. The archived legacy imports were verified with the original PyTorch environment.
- Vehicle backup root: `/home/control/aiformula_backups/20261005_yolop_integration`. It retains initial and camera-enabled allnodes versions plus `a2_switch.json` with before/after hashes. Check current hashes and use unused destinations before any reversible switch.
- Private Jetson environment: `/home/control/aiformula_runtime/yolop_a2_20261005`. Read `docs/jetson_a2_environment_20261005.md` before dependency changes. Its ROS Foxy/Python 3.8 profile is a verified target exception to upstream metadata. Do not use upstream `pip install -e .`, alter system Python/CUDA, or weaken restricted checkpoint loading.
- Existing separate `map_assist` and user RViz were retained. Verify process identities before stopping launches; do not kill unrelated processes.
- Live allnodes + A2 camera/mask output has been observed. Two manual OA observations had zero ROI counts and no OA triplets, so real-obstacle triggering and B-spline acceptance remain pending. OA has been stopped; allnodes and A2 remain running at the latest handoff.
- The OA wrapper's custom-context executor bug was corrected without editing OA algorithms, preserving the previous wrapper in the backup root. Actual restart/stop retest exited cleanly and delivered 39 `false` reset messages; retain both failed and successful evidence. This lifecycle check does not establish triggered B-spline output.

## Current interface contracts

- Camera input: `/aiformula_sensing/zed_node/left_image/undistorted` (`sensor_msgs/Image`).
- Lane mask: `/aiformula_perception/road_detector/mask_image` (`sensor_msgs/Image`, `mono8`, source image dimensions, 0/255).
- Obstacle mask: `/cone_mask` (`sensor_msgs/Image`, `mono8`, source dimensions, filled detector bounding boxes).
- Annotated object image: `/cone_bbox_image` (`sensor_msgs/Image`, `bgr8`, source dimensions).
- Avoidance output uses the existing `withoutkalman_0312` switching interface: OA three-point output to `/oa_point_a`, `/oa_point_b`, `/oa_point_c`; selection flag `/point_flag`.
- `allnodes.launch.py` keeps `lane_0215` as its original lane-points implementation. The legacy lane-only road detector is opt-in so a separately launched new detector can be the sole lane-mask publisher.
- `yolop_a2.launch.py` starts A2 independently. `obstacle_avoidance.launch.py` is the manual OA entry. Avoid duplicate lane-mask or `/cone_*` publishers.
- A2 traffic-light outputs use separate `/aiformula_perception/yolop_a2/...` names by default. Do not silently replace the semantic pipeline's light input.
- Native OA draws its B-spline on `/processed_image` and produces three Pose2D points; there is no existing `/bspline` Path topic. Check `/filtered_lane_pose` when validating complete wiring. Start observers before transitions because `/point_flag` is not latched.

Read the dated handoff before continuing vehicle work; it records what is actually deployed and what remains pending.
