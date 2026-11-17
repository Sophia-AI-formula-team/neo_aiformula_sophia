> 当前迭代：1118 / 0.4.3；接线和问题分析位于仓库根目录 `codexws/unity_semantic_1118_20261118/`。下方旧版本记录保留为历史。

# Semantic planning integration

> **Current 0.4.1 experiment:** the 1117 supplier package and Unity match-only
> results are documented in the [1117 trial report](../../../../../docs/semantic_joint_20261115/SEMANTIC_1117_本地试跑.md),
> [wiring audit](../../../../../docs/semantic_joint_20261115/1117_接线审计.md) and
> [deployment guide](../../../../../docs/semantic_joint_20261115/1117_部署说明.md).
> Avoidance and external branch state are fixed in that simulation adapter; no
> native ROS or successful red detour is claimed. The 0.4.0 material below is historical.

> **Current 0.4.0 experiment, 2026-11-16:** the user-supplied 1115 source tests
> joint topo fusion and its own traffic/detour behavior. Start with the
> [source, compatibility and verification record](../../../../../docs/semantic_joint_20261115/README.md).
> Older results below remain historical and are not acceptance of this source.

The current Unity export and its guarded installer are in
[unity_semantic_joint_20261116](../../../../../codexws/unity_semantic_joint_20261116/README.md).
Read its [results](../../../../../codexws/unity_semantic_joint_20261116/RESULTS.md),
deployment and agent handoff before running this experiment. Its explicit Unity
signal input uses the existing calibrated RGB-panel detector; A4 detections stay
visible but do not vote on the signal. Native MapAssist/LYA algorithms remain
unchanged. Bbox distance and the older stop_wait/checkpoint modes are separate.

## Historical integration — not 0.4.0 acceptance

The planner remains in `perception/semantic_planning_system`, matching the neo
repository layout. ROS command adapters live in `control/semantic_drive_bridge`,
launch entry points in `launchers`, and the verified Unity integration in
`codexws/unity_semantic_integration` (excluded from colcon).

Start with [DEPLOYMENT.md](DEPLOYMENT.md). A future agent should read
[AGENTS.md](AGENTS.md), [AGENT_HANDOFF.md](AGENT_HANDOFF.md), and
[INTEGRATION_ISSUES.md](INTEGRATION_ISSUES.md) before changing wiring.

| Delivery | Entry | Evidence |
| --- | --- | --- |
| Unity RGB → A3 → LLP/LYA → MapAssist → simulated vehicle | [Frozen source overlay and installer](../../../../../codexws/unity_semantic_integration/README.md) | Four final63 runs; both red stops inside 8–9 m |
| Native ROS legacy lane chain + atomic signal receiver + command watchdog | `ros2 launch launchers semantic_shadow.launch.py` | Separate shadow output; read the native limitations before running |
| Existing costmap/primitive semantic planner | `semantic_bringup` | Separate planner; not the MapAssist simulation chain |

The core revision used by the four Unity runs is
`63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96`. The planner and local planner packages
remain version **0.2.0**. Integration adds a separately versioned bridge package;
it does not change the semantic algorithm, old launch defaults, or motor math.

Red does not mean unconditional LEFT. The primitive planner prefers a feasible
left candidate, while MapAssist needs a learned/aligned topology to detour. The
short, cold-start Unity runs verify red stop and green continuation.

Native MapAssist 0.2.1 now defaults to stable-red wait / stable-green resume,
with distance approach and automatic detour disabled. Topo learning is unchanged.
See [the scoped change record](SIMPLE_TRAFFIC_CHANGE.md); historical Unity results above remain tied to core63.
