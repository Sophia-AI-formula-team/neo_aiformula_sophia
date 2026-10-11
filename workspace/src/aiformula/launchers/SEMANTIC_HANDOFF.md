# Agent handoff: native semantic shadow wiring

Read [SEMANTIC_DEPLOYMENT.md](SEMANTIC_DEPLOYMENT.md), the semantic root
[AGENTS.md](../perception/semantic_planning_system/AGENTS.md),
[AGENT_HANDOFF.md](../perception/semantic_planning_system/AGENT_HANDOFF.md), and
[INTEGRATION_ISSUES.md](../perception/semantic_planning_system/INTEGRATION_ISSUES.md)
before continuing. This handoff describes executable ROS wiring; it is not a
hardware acceptance record.

## Current ownership and scope

- New entry: `launch/semantic_shadow.launch.py`; installed by the existing launch
  directory rule. These two Markdown files are also installed with `launchers`.
- New nodes live in `control/semantic_drive_bridge` (version 0.1.0):
  `signal_capture_bridge` and `command_watchdog`.
- Semantic core baseline: `63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96`, version
  0.2.0 for core/local ROS packages. Do not bump unchanged core versions.
- `launchers` is now 0.1.0 (previously 0.0.0) for this added entry and docs.
- Native LLP, `lane_0215`, `withoutkalman_0312`, `lya_0221`, motor state1,
  perception models/weights and existing launch defaults are unchanged.
- Preserve the canonical checkout's unrelated untracked
  `workspace/src/aiformula/control/biased_mppi/`. Continue from the isolated
  integration branch rather than cleaning or resetting that checkout.

The launch owns exactly one private lane/target/controller/MapAssist/watchdog
chain. External calibrated perception, odometry, camera TF and any inference
process keep separate ownership. Do not add camera/motor/teleop startup through
an unqualified include of `allnodes.launch.py`. Do not include the separate
`semantic_vehicle_live.launch.py` primitive-planner/safety chain.

## Contract that must remain visible

1. All automatic ABC, target, raw and gated command topics are under
   `/semantic_drive/`; the sole final output is a restricted `shadow_*` topic.
   The live motor topic is explicitly rejected. No launch argument enables live
   control. This is a functional boundary while native provenance is incomplete.
2. The external producer supplies a real capture header, `sequence=source_ns`,
   actual color and calibrated metric distance/uncertainty in one signal bundle.
   The receiver derives age in the matching ROS clock domain. It does not
   estimate range, select a relevant light or relabel unknown as remembered red.
3. Atomic MapAssist signal input has one producer. Leave the private legacy color
   input unused; feeding both APIs clears the range in the legacy callback.
4. The native points are still unstamped Pose2D. Side-cloud caching, receipt-time
   ABC assembly and unstamped Twist prevent end-to-end source provenance. The
   status-aware watchdog is useful fault containment, not a repair of this gap.
5. `status_topic` on command_watchdog is its **input** `/semantic_drive/map_status`.
   `diagnostics_topic` is its **output** `/semantic_drive/watchdog_status`.
6. LLP projects into `base_link`, matching hard-coded native LYA assumptions.
   Keep LYA's own TF publisher/listener private. Validate external camera TF,
   calibration, mask resolution and source frame identity separately.

## Next work and evidence gates

Do not claim native deployment from Windows syntax tests, ROS stubs or prior
Unity videos. Record the actual ROS distro, Python ABI, Jetson/CUDA environment,
package commits, model manifest and calibration hashes before a native run.
Resolve the vendored A3 Python>=3.10 versus stock Foxy/Python3.8 mismatch explicitly.
No calibrated native signal-distance producer is delivered by this launch.

On a ROS host, first build and run a shadow graph with external recorded or live
inputs. Verify topic types/QoS and single publishers, capture-time rejection,
clock reset, process restart, transport delays and each stop/resume edge. Keep
offline/stub, replay, live ROS, Unity and hardware evidence in separate categories.
Do not interpret a received command as proof of execution.

Before a future live entry, design and validate stamped source identity through
cloud selection, complete ABC, target and command generation. Independent side
messages and time-of-receipt counters alone are insufficient. Preserve native
selection/LYA semantics or document any algorithm change before implementing it.
Also require downstream stop-on-command-loss, process-death behavior and a single
manual/autonomous arbiter. Existing motor_controller caches CAN commands; the
shadow watchdog must not be presented as a hardware emergency stop.

Changes to core behavior require a concrete issue report first, with the
failing input and preserved evidence. Do not tune pixel thresholds, source TTL,
vehicle dynamics or stopping parameters merely to pass a case. Default-disabled
core behavior and protected sources remain versioned review boundaries.

Use a new reviewed commit for integration work. Preserve the stacked core PR
history; do not amend it or mix Simulator media/source into this ROS package.
Coordinate Unity/CPU ownership separately; ROS documentation work is not
authorization to enter Play, use a model, or interrupt another task.
