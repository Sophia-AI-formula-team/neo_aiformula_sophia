# Deployment

## MapAssist 0.3.1 checkpoint patch — 2026-10-15

`semantic_local_planner_ros` is now 0.3.1. Its only runtime change allows finite
negative progress when a closed reference exists, matching the original route
matcher's signed seam coordinates. Negative travel distance and negative progress
before closure remain invalid. The semantic core and other package versions are
unchanged. See the [problem, source hashes and 118-test receipt](semantic_local_planner_ros/docs/checkpoint_signed_progress_20261015/README.md).

For a future authorized vehicle deployment, back up the current checkpoint and
installed package, then rebuild only `semantic_local_planner_ros` using the normal
vehicle deployment procedure. This local revision does not authorize vehicle
operation and is not a new ROS/DDS or physical durability result. Production
`use_sim_time=true` still disables checkpoints; the explicit Unity simulation
adapter is separate and must not be installed as native ROS lifecycle code.

Checkpoint schema remains 1. An older unpatched 0.3.0 reader still rejects a
negative seam value; preserve backups for rollback and never clamp or rewrite
progress merely to make an old reader accept it. Record closure, official-map
fusion and semantic restart as separate results. The triggering Unity run closed
its odometric lap but failed final checkpoint save and official fusion; it remains
a failed overall run under its original source revision.

> **2026-10-13 当前本地版本**：MapAssist 0.3.0新增闭环诊断和车辆入口自动存档/重启恢复，基于PR24；未部署到离线车辆。车上最后确认MapAssist 0.2.2 / motor 0.4.1。详见[最新变更、使用和验证说明](../../../../../docs/topo_checkpoint_20261013/README.md)。下文保留各历史阶段的版本和证据边界，不代表当前车辆仍为shadow-only或motor无超时。

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
