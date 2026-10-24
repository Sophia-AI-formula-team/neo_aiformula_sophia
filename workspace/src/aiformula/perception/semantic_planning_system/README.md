# Semantic planning integration

> **2026-10-15 Unity 验证完成**：固定 0.3.1 源码的内外圈红停绿行、闭环和两阶段语义进程恢复均通过；官方图融合仍未通过 RMSE 门限。见[结果与视频凭据](../../../../../codexws/unity_neo_topo_20261015/RESULTS.md)、[部署说明](../../../../../codexws/unity_neo_topo_20261015/DEPLOYMENT.md)和[最新交接](../../../../../codexws/unity_neo_topo_20261015/AGENT_HANDOFF.md)。这不代表实车或原生 DDS 验收。

> **2026-10-15 修订 0.3.1**：修复已闭合路线在起点附近产生合法负进度时的 checkpoint `negative_progress` 保存失败。仅修改持久化校验，核心几何、闭环阈值、官方图、LYA 和交通策略不变；118 项针对性回归通过，原始 Unity 失败与测试边界见[问题及验证记录](semantic_local_planner_ros/docs/checkpoint_signed_progress_20261015/README.md)。新版本尚无实车部署/验收结论。

> **2026-10-13 当前本地版本**：MapAssist 0.3.0新增闭环诊断和车辆入口自动存档/重启恢复，基于PR24；未部署到离线车辆。车上最后确认MapAssist 0.2.2 / motor 0.4.1。详见[最新变更、使用和验证说明](../../../../../docs/topo_checkpoint_20261013/README.md)。下文保留各历史阶段的版本和证据边界，不代表当前车辆仍为shadow-only或motor无超时。

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
