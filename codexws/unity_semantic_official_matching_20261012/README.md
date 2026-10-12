# Official-map left-turn diagnostics 0.1.0

**红灯路口左转尚未通过。** 此版本交付默认关闭的 Unity 采集接线、真实失败证据与离线候选；不发布新的转向功能。官方 OSM、semantic core/local planner 0.2.0、原生 LLP/LYA 和车辆参数保持原字节。

沿用 neo 结构：原生 ROS shadow 接线在 `workspace/src/aiformula`，本目录在 `codexws` 并包含 `COLCON_IGNORE`。PR #17 是接线基线；PR #18 的仿真路口标定方案未被选择，仍为历史 draft，不启用。

本目录的 source overlay 是 frozen final63 到当前诊断接线的完整增量；其中包含此前 acquisition/route snapshot 接线，本次新采集变化只涉及 RoadDetector C#、其 worker 和独立试验 harness。新增 `EnableJunctionCaptureMetadata` 默认为 false，不属于保存的调参项；开启时只记录同次图像的实际 render-root 位姿与原始 mask 身份，不调用观察器或改变控制。

| 已验证项 | 证据与限制 |
| --- | --- |
| v2 采集协议 | 24 项 transport 测试通过；MCP 编译无错误。实际内外圈 724 个采集包的身份与原始 mask SHA 通过审计；各有 1 个停止 reset 后的旧 epoch 包，明确拒绝进入当前上下文。 |
| 外圈首圈 | 286.089m / 112.213s 完成真实 reference 学习；融合 RMSE 3.410266m 仍超过原 3.0m 门槛，无左转。 |
| v2 外圈 | 176.649m / 71.277s，采集后主动停止；不是路线验收。 |
| v2 内圈 | 64.875m / 29.591s，原 lane 支持丢失后停车；失败保留。 |
| 曲线观察器 | 18 项合成测试通过；69 张真实 A3 mask 得到 0 个支路双边界候选。远处一行像素会产生地面阶梯假横线，不能降低门槛认作支路。 |
| 官方节点关联候选 | 31 项合成反例测试通过；无真实正观测 ledger，不接入控制。 |
| 保护 | 最新 Edit 基线对比：116 参数、3 个刚体、175 个组件、相机、spawn、全局设置一致；343 源文件仅有声明的采集变化。 |

历史失败及误差修正均保留：原 B 日志的相对/绝对时钟混用不能构造严格 capture pose；v1 因 Body/render 插值差异拒绝位姿是正确行为；v2 明确记录图像位姿，仍保留 `body_image_coincident=false`。第一个测试 driver 的模块名错误和第一个 readback 的 epoch 诊断失败也保留，不回标成功。

当前 blocker 和最小修复方向见 [问题报告](reports/OFFICIAL_AUTO_MATCH_PROBLEM_AND_PLAN.md)、[视觉目标所有权报告](reports/VISUAL_DETOUR_OWNERSHIP_PROBLEM_AND_PLAN.md)、[agent 交接](AGENT_HANDOFF.md) 与 [部署](DEPLOY.md)。官方图只提供连接意图；内部节点 chord 不是实测道路轨迹。新 semantic 行为若确实必要，应先按问题报告实现独立 0.3.0 变更，再取得真实 MCP 内外圈左入、内部通过、退出和红/绿回归证据。

原始 RGB/mask、视频、模型权重与大型日志保留在本机诊断目录，不上传 Git。`evidence` 内记录当前小型结果与 SHA；新视频生成后另加媒体清单。Unity 验证不等于 ROS/DDS 或实车验收。
