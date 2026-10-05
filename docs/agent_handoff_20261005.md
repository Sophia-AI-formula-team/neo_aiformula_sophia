# 2026-10-05 vehicle integration

## Current status

YOLOP A2 is deployed in a separate vehicle environment. Updated allnodes starts
the camera and bypasses the archived lane-only detector. Live camera input,
binary lane-mask Image output, object overlays, and detection JSON have been
observed. Manual obstacle avoidance received images and produced its debug
image, but neither observed indoor scene triggered avoidance. The manually
started OA session has now been stopped; allnodes and A2 remain running.

**Real-object triggering, source selection, and B-spline/three-point acceptance
remain pending.** The user has been asked to put a real cone in the avoidance
ROI and has not yet replied. Do not claim full acceptance. No bag smoke or bag
playback was run.

## Request and source identity

The user requested unchanged uploads of the vehicle semantic package and
`lane_0927oa`, camera startup, preservation and bypass of the old YOLOP,
installation of the handed-off version with object detection, compatible wiring,
and live acceptance. Existing versions must remain recoverable.

- Repository baseline: `8914b613c7200709f7c8617aed2d408afd911a0d` on `main`.
- Integration branch: `integration/live-yolop-20261005`,
  [PR #9](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/pull/9).
- A2 publication: `06fa28bdc09c731249428b7d81850d23e4e216a3`,
  [PR #11](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/pull/11).
  Its publication history is included in the PR #9 integration branch; use
  that PR's commit list for the integration revision and subsequent handoffs.
- Vehicle source root: `/home/workspace/src/aiformula` on
  `control@192.168.0.197`.
- The 98 files in `perception/semantic_planning_system` were copied into their
  corresponding repository location unchanged, excluding Python caches.
- `perception/lane_points/lane_points/lane_0927oa.py` and its existing additional
  `setup.py` console entry were copied unchanged. Existing lane implementations
  remain. allnodes still selects `lane_0215`.
- The original 31 A2 publication files, including runtime code and both weights,
  retain their bytes. All 131 preserved files (100 vehicle-source files plus
  31 upstream files) match their snapshots both locally and on the vehicle.

The new road-perception implementation is the independent ROS wrapper
`yolop_a2_perception`, under the sibling directory
`perception/road_detector/road_detector/YOLOP_A2_Deploy`. It uses separate lane
epoch-282 and object epoch-20 models. It is not a replacement of the original
`road_detector.py`. See [vehicle hashes](vehicle_source_snapshot_20261005.json),
[upstream hashes](yolop_a2_upstream_snapshot_20261005.json), and
[preservation evidence](validation_20261005/preservation.json).

## Vehicle deployment and preservation

Backup root: `/home/control/aiformula_backups/20261005_yolop_integration`.
Private environment root:
`/home/control/aiformula_runtime/yolop_a2_20261005`.

| allnodes version | Location | SHA-256 |
| --- | --- | --- |
| Initial vehicle file | Backup root + `allnodes.launch.py.original` | `e1a447d2f10f350eac3d5edbb73a04dcd28511f4c8fa3881f184e6b2ddcdb67d` |
| Intermediate camera-enabled file | Backup root + `allnodes.launch.py.camera_enabled.original` | `a7bdd440eb7b30d35e0ef01d7628e7409730034904e67ea6814aa11e4166601c` |
| Active integration file | `/home/workspace/src/aiformula/launchers/launch/allnodes.launch.py` | `b399ad26a1de18bb0d5014648c32768c3caf7c2e8dcd51b26a254e739b0d5388` |

Previous allnodes PID 12981 and its 16 children stopped normally with SIGINT
before the switch. The separate user `map_assist` and RViz processes were
retained. Updated allnodes uses `launch_rviz:=false` to retain that RViz.

The legacy directories were renamed without changing their contents:

| Location | Original path | Preserved path |
| --- | --- | --- |
| Source | `/home/workspace/src/aiformula/perception/road_detector/road_detector/yolop` | `/home/workspace/src/aiformula/perception/road_detector/road_detector/yolop_legacy_20261005` |
| Install | `/home/workspace/install/road_detector/lib/python3.8/site-packages/road_detector/yolop` | `/home/workspace/install/road_detector/lib/python3.8/site-packages/road_detector/yolop_legacy_20261005` |

All 38 source-directory files and 54 installed-directory files, including
existing caches, matched before/after hashes. The backup root's `a2_switch.json`
records their paths and hashes. The old detector script and `weights/123.pth`
were not replaced. Importing the old detector with the archived library path
and its original PyTorch `1.13.0a0+d0d6b1f2.nv22.10` succeeded; a complete live
rollback session has not been run.

New `launch/yolop_a2.launch.py`, `launch/obstacle_avoidance.launch.py`, and
`shellscript/run_obstacle_avoidance.py` were added under launchers source with
new install-space symlinks. Existing installed files were not replaced. The
existing allnodes install link resolves the updated source.

The private interpreter is `venv/bin/python3` under the environment root. The
A2 launch supplies runtime/wrapper source through `PYTHONPATH` and explicitly
preloads `/lib/aarch64-linux-gnu/libgomp.so.1`, as allnodes does. Its Python 3.8
profile preserves the ROS Foxy ABI and differs from upstream development
metadata. Do not use upstream `pip install -e .` or weaken restricted checkpoint
loading. CUDA headers were extracted privately for the TorchVision build; no
OS package installation or system Python/CUDA replacement occurred.

[The Jetson environment record](jetson_a2_environment_20261005.md) lists versions,
wheel/source/archive hashes, and build settings. [Device preflight](validation_20261005/preflight.json)
passed GPU NMS, GPU ROI Align, restricted checkpoint/model compatibility checks,
and one black `640 × 360` inference. The separate ROS wrapper build completed.
These checks do not establish live accuracy or avoidance acceptance.

## Wiring and ordinary startup

Source the existing ROS environment in each terminal:

```sh
source /opt/ros/foxy/setup.bash
source /home/workspace/install/local_setup.bash
```

Use three separate terminals, after checking equivalent launches are not already
running:

```sh
ros2 launch launchers allnodes.launch.py
ros2 launch launchers yolop_a2.launch.py
ros2 launch launchers obstacle_avoidance.launch.py
```

The third command is the manual OA step; allnodes does not start OA. If the
user's RViz is already open, add `launch_rviz:=false` to the first command. Do
not start a second camera/allnodes launch or duplicate mask/object publishers.

| Connection | Contract |
| --- | --- |
| Camera → A2 | `/aiformula_sensing/zed_node/left_image/undistorted`, `sensor_msgs/Image` |
| A2 lane mask | `/aiformula_perception/road_detector/mask_image`, `mono8`, source dimensions, 0/255 |
| A2 lane overlay | `/aiformula_visualization/road_detector/annotated_mask_image`, `bgr8` |
| A2 → OA obstacle mask | `/cone_mask`, `mono8`, source dimensions, selected filled bounding boxes |
| A2 → OA object overlay | `/cone_bbox_image`, `bgr8`, source dimensions |
| A2 detections | `/aiformula_perception/yolop_a2/detections`, JSON in `std_msgs/String` |
| OA three-point output | `/oa_point_a`, `/oa_point_b`, `/oa_point_c`, `geometry_msgs/Pose2D` |
| OA → filter selection | `/point_flag`, `std_msgs/Bool` |
| Existing filter output | `/filtered_lane_pose`, `geometry_msgs/Pose2D` |
| OA visualization | `/processed_image`, `bgr8`, including the computed B-spline |

A2 image outputs preserve camera headers. Its traffic-light outputs are isolated
as `/aiformula_perception/yolop_a2/traffic_light/state` and
`/aiformula_perception/yolop_a2/red_pixels_count` by default; they do not replace
the semantic pipeline's light source.

allnodes defaults `launch_legacy_road_detector` to `false`. Original `lane_0215`
continues its normal output. Manual OA defaults to unchanged `oa_neo`;
`algorithm:=oa_neo_nurbs` and `algorithm:=oa_neo_ubs` select the other preserved
algorithms. Its original `/processed_point_a/b/c` are remapped to `/oa_point_a/b/c`,
and `/lane_stop_flag` to `/point_flag`. Existing `withoutkalman_0312` already
switches these point sources. Native OA renders its B-spline and publishes
three Pose2D points; it does not publish a `/bspline` `nav_msgs/Path` topic.

The OA wrapper forwards termination signals and, after the algorithm exits,
publishes `/point_flag=false` for a bounded two-second discovery window. Its
first live shutdown exposed an executor/context mismatch: a custom rclpy
Context was used with `spin_once`'s default global executor. The helper was
corrected to use `SingleThreadedExecutor(context=context)` with explicit executor
shutdown; the OA algorithm files were not changed. The previous helper remains
at the backup root's `run_obstacle_avoidance.py.before_context_fix`.

Actual restart/stop retesting passed: OA launch PID 20678 exited with code 0,
wrapper PID 20680 finished cleanly, its log confirmed the `false` reset, and the
observer received 39 `false` messages before the publisher count returned to
zero. Retain the first failed shutdown observation as well as the successful
retest. Cleanup remains best effort under SIGKILL, power loss, or an undiscovered
subscriber. The unchanged filter retains latest A/B/C while rebuilding
subscriptions, so a transition can briefly use stale points.

## Sessions and evidence

These launch PIDs describe captured sessions, not permanent service identities.
Verify current command lines and descendants before stopping any process.

| Launch | Captured PID/state | Record/log under private environment root |
| --- | --- | --- |
| allnodes, `launch_rviz:=false` | 18215, retained | `live_processes_01.json`, `allnodes_live_01.log` |
| A2 | 18216, retained | `live_processes_01.json`, `yolop_a2_live_01.log` |
| Initial manual OA | 19393, stopped with SIGINT; first cleanup failed | `oa_process_01.json` |
| Fixed-wrapper OA retest | 20678 (wrapper 20680), stopped cleanly, code 0 | Lifecycle and shutdown evidence below |

OA is now stopped and remains a manually invoked step. allnodes and A2 continue.
The successful reset observation establishes cleanup delivery, not a real-object
avoidance trigger or a preceding switch back from an active OA path.
The final graph check found no legacy detector or OA node; lane mask and
`/cone_mask` each had one publisher, while `/point_flag` had zero publishers and
one filter subscriber.

Evidence is copied into [validation_20261005](validation_20261005). Current
records include `preflight.json`, `live_mask_01.json`, `live_oa_01.json`,
`cuda_header_sources.json`, and `preservation.json`. The second OA observation
is `live_oa_02.json`. Shutdown evidence preserves the initial failure in
`oa_stop_observation.json` and the successful retest in
`oa_stop_fixed_observation.json` and `oa_lifecycle_result.json`; both failed and
successful lifecycle evidence have been retrieved. `deployment_final.json`
records the selected launch/helper hashes and retained processes. Original JSON/images remain
under the vehicle private root, with local review copies at
`/home/nanato/aiformulaws/integration/evidence`.

`tools/verify_live_perception.py` is a read-only observer, also copied under the
vehicle private root. It receives images, JSON, ROI counts, selection flags,
OA points, and now `/filtered_lane_pose`; it publishes no inputs or commands.

## Live observations and remaining acceptance

[First live capture](validation_20261005/live_mask_01.json), 30 seconds with
allnodes plus A2:

| Check | Observation |
| --- | --- |
| Camera | 331 received frames |
| Lane mask | 170 frames; receive-span rate 6.22 Hz |
| Lane format/content | All 170 `640 × 360 mono8`, binary 0/255, nonempty; no decoding errors |
| Lane headers | 155 matched camera stamps, no frame-ID/dimension mismatch among matches; 15 unmatched |
| Lane header age | Mean 0.2801 s, maximum 0.5359 s in the observer ROS clock |
| Obstacle mask | 199 received binary mono8 frames, one observed nonempty mask |
| Object overlay | 95 received bgr8 frames |
| Detection JSON | 201 received messages |
| Latest inference timing | 117.875 ms total |

Each observed camera/A2 image topic had one publisher. Counts/rates are observer
measurements, not guaranteed publisher throughput; different streams can lose
different frames at the observer. The scene was an indoor desk area. Labels
included TV/chair and a `toilet` false positive. This verifies the live output
path, not race-scene segmentation/detection accuracy.

[First manual OA capture](validation_20261005/live_oa_01.json), 55 seconds,
received 745 debug frames and 893 ROI-count messages, all zero. No selection
flag or OA triplet was observed. A second 45-second capture received 1407
ROI-count messages, all zero, with no OA points or `/filtered_lane_pose`
messages. The indoor scene contains no track and no presented target obstacle.
These observations demonstrate OA image reception but cannot establish its
triggered B-spline output or complete point-source wiring.

Next acceptance steps:

1. Start the read-only observer before the obstacle transition. `/point_flag`
   is published on transitions and is not latched, so a late observer can miss
   its `true` value. Manually start OA if it is stopped.
2. Present a real obstacle, verify its correct box and mask within the OA
   trapezoid, and keep the original thresholds (entry requires ROI count >1000).
3. Confirm `/point_flag=true`, finite `/oa_point_a/b/c`, the visible B-spline on
   `/processed_image`, filter selection, and `/filtered_lane_pose`. Cyan pixels
   alone are insufficient because some detector boxes have the same color.
4. Remove the obstacle and observe completion/return behavior. Confirm stopping
   OA restores the ordinary lane source.
5. Append actual evidence; do not infer success from publisher presence, the
   black preflight image, or an empty-scene debug stream.

## Reversible rollback

A full live rollback has not been executed. Keep all source backups, A2 code,
private dependencies, downloaded archives, and logs. Do not delete/uninstall
them to select another detector.

For a perception-only return to legacy:

1. Stop manual OA and check its cleanup; stop A2 and confirm its process and
   publishers have disappeared. Use Ctrl-C in owning terminals or SIGINT to
   verified launch processes, not broad name-based kills.
2. Stop integration allnodes before restarting it to avoid duplicate camera and
   control nodes. Keep unrelated map_assist and the user's RViz.
3. Start `ros2 launch launchers allnodes.launch.py launch_legacy_road_detector:=true`
   (add `launch_rviz:=false` when retaining RViz). The updated launch supplies
   `yolop_legacy_20261005` through `legacy_yolop_path`/`PYTHONPATH` and keeps the
   old executable/weight. Do not run A2 simultaneously. Verify one lane-mask
   publisher.

For full restoration of original launch and detector paths:

1. Stop OA, A2, and integration allnodes as above; verify their children exited.
2. Check the active allnodes hash is still
   `b399ad26a1de18bb0d5014648c32768c3caf7c2e8dcd51b26a254e739b0d5388` and initial
   backup hash is `e1a447d2f10f350eac3d5edbb73a04dcd28511f4c8fa3881f184e6b2ddcdb67d`.
   If different, preserve and inspect the newly encountered version instead of
   overwriting another operator's changes.
3. Choose an unused backup filename, for example
   `allnodes.launch.py.a2_before_full_rollback`, under the backup root. Confirm
   it is absent, then move the active source allnodes there. Copy
   `allnodes.launch.py.original` to the now-vacant absolute source path in the
   deployment table. Retain both original backup and archived A2 launch.
4. At both source/install locations in the legacy-directory table, check
   `yolop` is absent and `yolop_legacy_20261005` matches `a2_switch.json`.
   Rename each archived directory back to `yolop`, replacing nothing. The old
   detector's relative imports require this name when using the original launch.
5. Verify restored allnodes and directory hashes before restarting. The initial
   launch had camera startup disabled; full rollback restores that state. The
   intermediate camera-enabled backup remains available if intentionally
   selecting that earlier state instead.

These steps preserve each version and require no system software changes.
