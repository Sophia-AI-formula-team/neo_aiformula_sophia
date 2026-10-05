# 2026-10-05 vehicle integration

## Request

Copy the vehicle semantic packages and `lane_0927oa` into their corresponding repository locations without changing their contents. Enable the camera, preserve and bypass the old lane-only YOLOP, install the user's newer YOLOP with object detection, connect existing mask/detection/avoidance interfaces, and validate with live inputs. Do not use bag smoke, destructive actions, or discard existing versions.

## Baseline and source snapshot

- Repository baseline: `8914b613c7200709f7c8617aed2d408afd911a0d` on `main`.
- Work branch: `integration/live-yolop-20261005`.
- Vehicle source root: `/home/workspace/src/aiformula` on `control@192.168.0.197`.
- The 98 files in `perception/semantic_planning_system` were copied unchanged from the current vehicle source, excluding Python caches.
- `perception/lane_points/lane_points/lane_0927oa.py` and its existing additional console entry were copied unchanged. Existing lane-points implementations remain present.
- Source SHA-256 values are recorded in `docs/vehicle_source_snapshot_20261005.json`.
- The vehicle had an already-running `allnodes` launch, legacy road detector, and a separate `map_assist` process at initial inspection. This integration has not restarted or stopped those processes.

## Vehicle change completed

- Restored the `OpaqueFunction(function=create_zed_node)` action in the vehicle's `launchers/launch/allnodes.launch.py`. This is the only vehicle change so far; it affects the next launch, not the already-running process.
- Preserved the original file by moving it to `/home/control/aiformula_backups/20261005_yolop_integration/allnodes.launch.py.original`, then installed the camera-enabled copy. No original content was discarded.
- Original SHA-256: `e1a447d2f10f350eac3d5edbb73a04dcd28511f4c8fa3881f184e6b2ddcdb67d`.
- Camera-enabled SHA-256: `a7bdd440eb7b30d35e0ef01d7628e7409730034904e67ea6814aa11e4166601c`.
- The backup directory also contains `deployment.json` with these paths and hashes.

## Integration work prepared, not yet deployed

- The repository's camera startup was already enabled; the vehicle copy had commented it out. The prepared launch retains enabled camera startup.
- A `launch_legacy_road_detector` argument defaults to `false`. Setting it to `true` opts into the preserved lane-only detector, named `road_detector_legacy`. Do not run both detectors on the same lane-mask topic.
- The newer YOLOP repository/branch and associated AGENTS instructions have not yet been identified. Do not substitute the old lane-only model or claim the vehicle installation/acceptance is complete.
- The existing obstacle avoidance subscribes to `/cone_mask` and `/cone_bbox_image`. It computes a spline and renders it to `/processed_image`; its control interface is three `Pose2D` topics, not an existing `/bspline` Path topic.
- Added a separately invoked `ros2 launch launchers obstacle_avoidance.launch.py`, defaulting to the unchanged `oa_neo` B-spline implementation. `algorithm:=oa_neo_nurbs` and `algorithm:=oa_neo_ubs` select the other preserved algorithms.
- This launch remaps OA `/processed_point_a/b/c` outputs to `/oa_point_a/b/c`, and OA `/lane_stop_flag` to `/point_flag`. The existing `withoutkalman_0312` already switches between those sources, so the original `lane_0215` need not be replaced.
- A small process wrapper forwards launch termination signals and, after the OA process exits, publishes `/point_flag=false` for a bounded two-second discovery window. This is best-effort cleanup; SIGKILL, power loss, or an undiscovered subscriber can prevent confirmation. These new launch/helper files are repository preparation only and have not been deployed or live-tested on the vehicle.
- The existing filter retains its latest A/B/C values while rebuilding subscriptions, so a transition may briefly use stale points. No filter algorithm was changed in this task.

## Acceptance status

Passed locally: the 100 uploaded source files (98 semantic files, `lane_0927oa.py`, and its `setup.py`) match the captured vehicle SHA-256 values; Python source parsing and `git diff --check`.

Pending: newer YOLOP identity, checkpoint identity, reversible deployment, real camera mask output, detection output, and manual obstacle-avoidance spline output. No bag smoke was run. No live acceptance has passed yet.

## Rollback

The original repository remains available at the baseline commit. To revert the vehicle camera-only change without discarding the changed version, first verify the current source hash still equals `a7bdd440eb7b30d35e0ef01d7628e7409730034904e67ea6814aa11e4166601c`, move that source to a new unused filename in `/home/control/aiformula_backups/20261005_yolop_integration`, and copy `allnodes.launch.py.original` back to its source path. Do not overwrite a source changed by another operator. No restart is needed to restore the file; a running ROS launch retains the actions it loaded at startup.

The legacy detector has not yet been renamed or disconnected on the vehicle because the intended replacement repository is unconfirmed. No model or operating-system package has been installed.
