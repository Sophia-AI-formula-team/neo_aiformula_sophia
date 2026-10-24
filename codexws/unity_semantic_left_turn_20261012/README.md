# 左转诊断与待验证接线候选

版本 **0.1.0-candidate.1**。本目录是默认关闭、尚未安装和编译的 Unity 接线归档；标定模式的内外圈左转均未通过实跑验收。地图模式选择仍待用户确认。

原始 semantic 自动模式的左转验证失败，不能把红灯停车或外圈转弯称为路口绕行成功。原四场红绿灯结果保留在 [final63 归档](../unity_semantic_integration/README.md)，没有重写。原生 ROS 接线保留在 neo 的 `workspace/src/aiformula`，本目录有 `COLCON_IGNORE`，不参与 colcon 构建。

## 实测结论

| 实际实验 | 结果 | 未通过原因 |
| --- | --- | --- |
| exact_inner_red_detour | 首圈行驶 64.679 m 后停止；未产生首圈快照 | 新鲜车道 mask 无足够投影支持，保留失线停车守卫 |
| exact_outer_red_detour | 同一 worker 完成首圈，红灯前端距 8.643219 m 停车 | 原自动融合 RMSE 3.404463 m > 3.0 m；没有进入 detour |
| 较早 B 内/外圈学习 | 均完成真实绿色首圈 | 融合仍失败；不构成左转或红灯绕行证明 |

单独失败视频已输出：1280×960、15 fps、2441 帧、162.733333 s；主画面占高度 75%，包含真实识别、LLP/LYA 和 semantic 诊断。SHA256：`4a69d971f7209ace9e712b1e5b5670948e521f1282989f71b051d7c2f390c336`。视频和原始 RGB 不进入 Git；本机路径及完整解码记录见 [媒体 manifest](evidence/media/final_manifest.json)。

## 原因与最小修复边界

先读 [问题分析](PROBLEM_ANALYSIS.md) 和历史 [修复前报告](evidence/CALIBRATED_WIRING_PROBLEM_AND_PLAN.md)。官方五个外节点是图上的 junction，物理外环四个弯在这些节点之间；已有自动算法把拓扑折角当作真实轨迹可观测转角。调整 RMSE 门槛、把弯心直接冒充路口 ID 都不能解决这种输入语义差异。A3 只有车道和检测输出，不能从不存在的 drivable head 获得可行驶区或可靠侧路证据。

候选保持 core `63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96` 和原生 LLP/LYA 不变，只在显式开启时增加：

- 严格验证的仿真路口标定图，保留原自动融合失败结果和真实 reference/trust/长度。
- 连续弧长 detour 预瞄，替代此显式模式下的段内预瞄；默认模式继续使用原类。
- 新鲜且明确归属于地图 detour 的 ABC 接线；仅针对该模式处理视觉 ABC 无支持时的所有权，保留 RGB 时效、E-STOP、命令时效与控制所有权守卫。
- 可选真实首圈快照加载，逐项校验源文件、参数、闭合条件并重新计算原 reference；不能标为本轮新学习。

标定是**进入控制的显式场景先验**，不是视觉自动识别。逻辑 ID 绑定到定向 control portal，不能称为路口中心。只有正向 `02 → 06 → 03` 两条边，依次左入、右转、左出；反向或其他绕行缺边即拒绝。配置依赖场景知识，已有规划路径检查不是独立几何或真实车体净空证明。

## 版本与文件

基于 integration commit `434fdc43d00399808adf7e548d491a5dfdc5bffa`。core 仍为 0.2.0，native drive bridge 仍为 0.1.0；本候选不更改其版本，不创建 release/tag。

`manifest.json` 定义完整 **final63 → 当前左转采集接线 → 标定候选** 增量，共 14 个受管文件及 416 个不变前置文件。`before/` 是冻结 final63 的完整字节；`overlay/` 是完整候选；`patches/` 对比这两个端点。历史 C# 小 patch 的 before 是中间采集版本，不能把它当作 final63 安装包。

原失败记录中的已安装接线与候选不同；候选 `installed=false` 不是对历史采集代码的描述。只有两个已生成的任务 `.meta` 被管理；其他新 helper 的 `.meta` 由未来 Unity 导入产生，不改现有 GUID。

按 [部署说明](DEPLOYMENT.md) 核验，按 [验收清单](VALIDATION.md) 获取真实结果。下一 agent 先读 [交接指令](AGENTS.md)。
