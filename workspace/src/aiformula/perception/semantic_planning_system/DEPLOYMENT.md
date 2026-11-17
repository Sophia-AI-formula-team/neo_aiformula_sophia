# Deployment

## Current experiment 0.4.1 — 1117 supplier package

Use the [1117 deployment guide](../../../../../docs/semantic_joint_20261115/1117_部署说明.md)
and [wiring audit](../../../../../docs/semantic_joint_20261115/1117_接线审计.md).
The Unity shim assumes idle avoidance and no external branch event; these are
not live ROS subscriptions. See the trial report for red-detour and test failures.

## Historical experiment 0.4.0 — 2026-11-16

Use the [1115 source record](../../../../../docs/semantic_joint_20261115/README.md)
for the current package locations and compatibility boundary. This new-function
experiment retains the supplied traffic algorithm. Old motor, stop_wait, bbox,
race_map or checkpoint launches below must not be assumed compatible. Source
preparation does not deploy a vehicle or establish a new Unity result.

Build only the two selected ROS packages from the chosen neo worktree in a
compatible ROS environment: `colcon build --symlink-install --packages-select
semantic_planner_core semantic_local_planner_ros`. This command has not been run
as part of this source preparation. The supplied bare entry is
`ros2 run semantic_local_planner_ros map_assist`; its traffic control defaults
off. Simulation wiring must record its overrides and maintain a distinct export
and output directory. Preserve the archived ZIP and prior source pins for rollback.

For this source's MCP simulation, use the separately pinned
[Joint Unity deployment guide](../../../../../codexws/unity_semantic_joint_20261116/DEPLOYMENT.md)
and [actual results](../../../../../codexws/unity_semantic_joint_20261116/RESULTS.md).
Its calibrated RGB-panel-only signal mode is specific to the known Unity panel
and camera calibration; it is not an onboard traffic-light detector upgrade.
Keep the original LYA, official OSM, and unchanged 0.4.0 package algorithms.

## Historical 0.2.x integration below

The following commands, versions and acceptance records belong to the older
integration. They are retained for provenance and are not current 0.4.0 launch
or hardware acceptance instructions.

This repository offers two distinct deployment paths. The frozen Unity path
has four recorded closed-loop runs. The new native ROS path is a shadow wiring
entry and has no physical vehicle acceptance.

## Repository layout and versions

| Path relative to repository root | Purpose |
| --- | --- |
| `workspace/src/aiformula/perception/semantic_planning_system` | Semantic core and ROS packages, 0.2.0 for core/local planner |
| `workspace/src/aiformula/control/semantic_drive_bridge` | Native signal receiver and shadow command watchdog, 0.1.0 |
| `workspace/src/aiformula/launchers` | 0.1.0; dedicated `semantic_shadow.launch.py`; old allnodes defaults preserved |
| `codexws/unity_semantic_integration` | Versioned Unity source overlay, guarded installer and frozen evidence |

Core baseline: `63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96`.
Integration branch: `feat/semantic-neo-integration-20261012`, stacked on
`fix/semantic-distance-stop-20261012`. PR #15 imports the source, PR #16 fixes
distance approach, and the integration PR adds wiring/docs. No tag is moved.

## Unity deployment and replay

Follow [the Unity deployment guide](../../../../../codexws/unity_semantic_integration/DEPLOYMENT.md).
Use an existing compatible Simulator checkout with the original pinned
dependencies. The overlay is not a standalone Unity project. Verify the manifest
before applying; the installer refuses unexpected file drift and supports a
scoped rollback. Keep weights outside this Git delivery and verify their SHA.

Run through Unity MCP after exclusive editor ownership has been handed over.
Restore scene, Play state, focus override and worker ownership afterwards.
The target camera/light position remains fixed; inner and outer red/green starts
are separate cases. The trial stops at the end of the requested northeast
segment, rather than completing an entire circuit.

The frozen red cases stopped at 8.635939 m (inner) and 8.602903 m (outer), waited
for green, then continued. The video preserves recognition overlay, LLP/LYA,
semantic output and the vehicle view. These runs use ideal Unity odometry and
an in-process ROS transport shim. They do not test native DDS.

## Native ROS shadow deployment

Follow [launchers/SEMANTIC_DEPLOYMENT.md](../../launchers/SEMANTIC_DEPLOYMENT.md).
Build from the neo `workspace` with source roots restricted to the active
`src/aiformula` tree; do not build the entire parent project containing duplicate
packages. Configure the existing calibrated vehicle TF and LLP parameters, and
provide matching current mask, stamped signal capture and odometry producers.
Do not run `allnodes` or `map_drive` alongside the shadow entry.

The bridge itself targets Python >=3.8 with real rclpy and standard ROS messages.
The A3 inference package requires Python >=3.10 and its own compatible runtime.
A3 ROS wrapper availability does not prove stock Foxy compatibility. The new
launch does not install, download, or invent a metric signal producer.

Default final output is `/semantic_drive/shadow_cmd_vel`. The launch and bridge
reject the existing motor command topic. Keep the final output unconnected to
hardware. Read [INTEGRATION_ISSUES.md](INTEGRATION_ISSUES.md) for the missing source
provenance, final-consumption age and physical timeout requirements.

## Validation and rollback

Run scoped core tests and bridge policy tests. Record actual ROS distribution,
interpreter, package versions and commit for any native DDS smoke test; Windows
pure-policy tests are not a substitute. Check all resolved publishers using
`ros2 topic info -v`, verify fault/timeout zero output in the shadow topic, and
save the trace before claiming native acceptance.

For native rollback, stop the dedicated shadow launch and return to the pinned
pre-integration checkout/overlay. Never assume stopping a process stops hardware.
For Unity rollback, use the guarded overlay rollback against matching installed
hashes. Keep baseline recordings and tags immutable; record any new runs under a
new source revision instead of overwriting final63 evidence.
