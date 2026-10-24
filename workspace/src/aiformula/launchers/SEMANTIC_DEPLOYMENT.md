# Native semantic shadow deployment

`semantic_shadow.launch.py` connects the existing neo ROS lane and LYA nodes to
MapAssist and `semantic_drive_bridge` on private topics. It does not start a
camera, detector, motor, CAN interface, teleop, obstacle avoidance or a metric
signal estimator. It has no live-driving mode. Existing launch files and their
defaults are unchanged.

Read the central [deployment guide](../perception/semantic_planning_system/DEPLOYMENT.md),
[integration issues](../perception/semantic_planning_system/INTEGRATION_ISSUES.md),
and [agent handoff](SEMANTIC_HANDOFF.md) before changing this entry.
The semantic algorithm baseline is commit `63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96`,
with `semantic_planner_core` and `semantic_local_planner_ros` at version 0.2.0.
The separately added bridge package is version 0.1.0. This entry does not alter
the semantic core, native LLP, native lane target selection, LYA gains or state1.

## What this deployment establishes

The launch constructs real ROS nodes and explicit remappings:

```text
external calibrated lane mask
  -> lane_line_publisher/lane_line_publisher
  -> lane_points/lane_0215
  -> MapAssist point gate
  -> kalman_filter/withoutkalman_0312
  -> trajectory_follower/lya_0221
  -> MapAssist velocity gate
  -> semantic_drive_bridge/command_watchdog
  -> /semantic_drive/shadow_cmd_vel

external stamped signal_capture -> signal_capture_bridge -> MapAssist
external Odometry -------------> LYA + MapAssist
external camera TF ------------> LLP
```

The original `map_drive.launch.py` uses `lane_0529oa` and
`lya_follower_connected_omegat_global`; it is not included here. Do not combine
this entry with that launch or use `allnodes.launch.py` as a sensing-only
launcher: allnodes also starts motor, teleop, lane target and filter nodes.
Run external sensing/perception components separately in a controlled ROS domain.

## Outstanding native acceptance gaps

These issues were identified before this integration; no semantic-core change
is made to conceal them.

| Gap | Consequence and boundary |
| --- | --- |
| Neo documents Ubuntu 20.04 / Foxy / Python 3.8, while vendored A3 requires Python >=3.10,<3.14. | The A3 ROS wrapper cannot simply be installed into stock Foxy's interpreter. A matching ROS/Python environment or an explicitly validated process bridge is required. This launch starts neither. |
| No calibrated native metric signal producer is deployed by this entry. | External perception must provide the capture bundle below. A plain color topic or detector bounding box is not a distance measurement. |
| Native `lane_0215` drops cloud headers and keeps side clouds independently; ABC are separate unstamped Pose2D messages. | Current-frame origin, side synchronization, and capture age are not preserved end-to-end. Receipt freshness and point counters cannot prove that a new command came from a new image. |
| Native LYA publishes unstamped Twist; the final watchdog sees receipt time and MapAssist status. | The watchdog limits missing/stale arrivals, but cannot reconstruct source-image provenance or reject every replayed upstream value. This is why the output remains shadow-only. |
| Existing motor_controller repeatedly publishes cached CAN data without a final Twist input-age timeout. | Even a bridge watchdog is not a downstream hardware stop guarantee if that watchdog dies. This launch never connects to that motor input. |
| Unity used additional RGB panel and current-corridor adapters. | They are not implemented by the unchanged native C++ LLP or this launch. Unity four-case success is not native DDS, Jetson, or vehicle acceptance. |

Stamped lane/ABC/command provenance, process-death stopping, command arbitration
and measured vehicle braking must be resolved before adding a separate live
entry. Do not promote this entry by remapping its shadow output to a motor topic.

## Required external inputs

All publishers must use the same ROS clock domain. With `use_sim_time:=true`,
every producer must use the same `/clock`; restart the chain after a clock reset.
Do not derive capture age from frame count, wall-clock guesses, or Unity truth.

1. A calibrated `sensor_msgs/msg/Image` lane mask, default
   `/semantic_drive/lane_mask`, retaining its original capture header. The
   intrinsics must describe the actual undistorted mask pixels and resolution.
   Native LLP currently requests reliable image QoS; verify compatibility with
   the external publisher using `ros2 topic info --verbose`.
2. A readable ROS parameter YAML with wildcard `/**: ros__parameters:` and the
   selected camera key (default `zedx`). Native LLP reads
   `<camera_name>.focal_length.x/y` and `<camera_name>.center_point.x/y`.
   The tracked vehicle intrinsics are examples for their named camera and
   resolution, not automatic authorization to reuse them for another setup.
3. Valid external TF between `base_link` and `camera_frame_id` before LLP starts.
   LLP loads the camera transform during initialization. Restart after changing
   calibration; it is not a live CameraInfo/extrinsics estimator.
4. `nav_msgs/msg/Odometry`, default
   `/aiformula_sensing/gyro_odometry_publisher/odom`: pose in `odom`, body frame
   `base_link`, measured finite body-forward speed and yaw rate, valid timestamps
   and covariance. These frames are required by the unchanged `lya_0221` code.
5. One atomic `std_msgs/msg/String` JSON bundle per real signal image, on
   `/semantic_drive/signal_capture`. `expected_frame_id` is required and must
   exactly match that source camera header. It is a frame identity check, not a
   substitute for metric calibration.

Signal bundle shape (illustrative numbers, not a publishable test command):

```json
{
  "header": {
    "stamp": {"sec": 123, "nanosec": 456000000},
    "frame_id": "verified_camera_optical_frame"
  },
  "sequence": 123456000000,
  "state": "red",
  "distance_m": 9.4,
  "uncertainty_m": 0.4
}
```

`sequence` equals `sec * 1000000000 + nanosec` and must increase for each new
capture. Use actual image capture time, not publish time; do not reset a local
counter when the producer restarts. Unknown color is an actual observation,
not permission to reuse a previous red/green vote. Missing distance is `null`,
not zero or a map-derived estimate. The bridge validates the timestamp and
rebases `age_sec` at reception before forwarding a new accepted frame to
`/semantic_drive/map_signal`. Its maximum capture age is 0.5 seconds.

Metric `distance_m` is measured at capture time from the vehicle front along the
current body-forward axis to the observed signal, with nonnegative measurement
uncertainty. It is neither a slant range nor a fixed-track-plane stopping metric.
The external producer must account for camera extrinsics and the actual vehicle
front offset. Do not substitute the Unity test's 0.216 m offset for an unmeasured
vehicle offset. A lamp-box bottom is not a ground stop line. Relevant-signal
selection and metric validity are responsibilities of that producer; the prior
Unity evidence covered one relevant signal, not arbitrary intersections.

Do not also publish A3's legacy color to MapAssist. Its legacy callback clears
the atomic range observation. This launch fixes that input to the unused
`/semantic_drive/unused_legacy_light` topic. The `red_pixel_traffic_light_adapter`
in the other semantic chain emits `unknown` for clear pixels and timer-repeats
state; it is not this atomic producer.

## Build and start on a ROS host

These are deployment instructions, not a claim that a ROS graph or vehicle has
been run on the Windows authoring host. From the repository root, after sourcing
the intended ROS installation:

```bash
rosdep install --from-paths dependencies workspace/src --ignore-src -r -y
colcon build --base-paths dependencies workspace/src --packages-up-to launchers
source install/setup.bash
ros2 pkg executables semantic_drive_bridge
ros2 launch launchers semantic_shadow.launch.py --show-args
python3 -m pytest workspace/src/aiformula/launchers/test/test_semantic_shadow_launch.py -q
```

The launch tests import actual ROS launch classes. They skip when ROS imports
are unavailable; the valid-graph construction case additionally requires the
installed packages. Construction and argument rejection do not start nodes or
prove DDS delivery, calibration, control behavior, or hardware safety.

Resolve declared and legacy runtime dependencies for the selected ROS distro,
including the original nodes' pandas, NumPy and TF imports. Do not use a model
fallback or silently change LYA/LLP parameters to make startup succeed. A3 has
its own [deployment guide](../perception/road_detector/road_detector/YOLOP_A3_Deploy/DEPLOYMENT.md)
and requires a separately matched interpreter/environment.

Start the external input publishers first. Then provide your verified values:

```bash
ros2 launch launchers semantic_shadow.launch.py \
  camera_params_file:=/absolute/path/to/verified_intrinsics.yaml \
  camera_frame_id:=verified_camera_optical_frame \
  expected_frame_id:=verified_camera_optical_frame \
  lane_mask_topic:=/semantic_drive/lane_mask \
  odom_topic:=/aiformula_sensing/gyro_odometry_publisher/odom \
  use_sim_time:=false
```

Empty frame names or a missing calibration file abort before node startup.
`shadow_output_topic` defaults to `/semantic_drive/shadow_cmd_vel` and only
accepts `/semantic_drive/shadow_*` topic names. In particular,
`/aiformula_control/game_pad/cmd_vel` and every `/aiformula_control/` topic are
explicitly rejected. There is no live-enable parameter and no motor/teleop node.

## Topic ownership and diagnostics

| Topic | Type | Producer -> consumer |
| --- | --- | --- |
| `/semantic_drive/lane_lines/{left,right,center}` | PointCloud2 | LLP -> lane_0215 |
| `/semantic_drive/original_point_{a,b,c}` | Pose2D | lane_0215 -> MapAssist |
| `/semantic_drive/processed_point_{a,b,c}` | Pose2D | MapAssist -> withoutkalman_0312 |
| `/semantic_drive/filtered_lane_pose`, `/semantic_drive/filtered_omega_t` | Pose2D | withoutkalman_0312 -> lya_0221 |
| `/semantic_drive/controller_cmd_vel` | Twist | lya_0221 -> MapAssist |
| `/semantic_drive/gated_cmd_vel` | Twist | MapAssist -> command_watchdog |
| `/semantic_drive/shadow_cmd_vel` | Twist | command_watchdog -> diagnostics/recording only |
| `/semantic_drive/signal_capture` | String JSON | external stamped metric perception -> signal_capture_bridge |
| `/semantic_drive/map_signal` | String JSON | signal_capture_bridge -> MapAssist |
| `/semantic_drive/map_status` | String JSON | MapAssist -> command_watchdog and diagnostics |
| `/semantic_drive/watchdog_status` | String JSON | command_watchdog -> diagnostics |

All seven node names are under `/semantic_drive`. LLP debug images and reference
paths also remain private. LYA creates its own `odom -> base_link` transform;
both its broadcaster and listener use `/semantic_drive/tf` and
`/semantic_drive/tf_static`, so it does not duplicate the externally owned global
TF tree. LLP alone consumes the external global camera TF.

This entry explicitly enables traffic control, metric approach and velocity
gating. It retains the core approach defaults: target lower bound 8.5 m, speed
cap 0.8 m/s, deceleration 1.0 m/s², latency 0.2 s. It does not apply the Unity
test overrides (8.0 m / 3.0 m/s / 1.5 m/s²) or promise an actual 8–9 m vehicle
stop. OA is absent and private unused OA topics prevent this shadow node from
joining live OA ownership; its cold-start OA state is explicitly idle.

Before recording a shadow run, inspect each automatic topic's publisher count
and require exactly the owner listed above. Capture masks/source headers,
signal_capture, map_signal, odometry, ABC, raw/gated/shadow commands and both
status streams in the same recording. Check native arrival ordering, source
age, duplicate/out-of-order handling, unknown/stale hold, fresh green resume,
loss of MapAssist/status, and watchdog timeout (0.25 s). These native checks are
outstanding until performed on the target ROS host; unit tests do not replace
them. A nonzero shadow Twist is a diagnostic result, not a vehicle command.

On exit, stop this launch and verify its nodes and private publishers disappear.
Keep external sensing and unrelated tasks under their existing owner. Roll back
the integration as a reviewed commit/worktree change; do not amend the core
history, overwrite model weights, or replace protected `biased_mppi` work.
