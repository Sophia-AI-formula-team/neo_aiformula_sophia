# Semantic planning integration

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
