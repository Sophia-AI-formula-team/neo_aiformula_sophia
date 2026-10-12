# Agent handoff

当前目标：保留官方 OSM，修复自动匹配，并验证红灯触发路口左转绕行。用户拒绝仿真路口标定；不要使用 PR #18 的手工节点、portal 或固定半径路径。用户要求 Unity MCP，禁止 Computer Use。

此版本为 **0.1.0 诊断归档**，semantic core/local planner 仍为 **0.2.0**；原生 bridge 仍为 **0.1.0**。分支从 PR #17 的 `434fdc43d00399808adf7e548d491a5dfdc5bffa` 派生。核心 frozen pin 是 `63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96`。不要修改已有 `codexws/unity_semantic_integration` 四场红/绿成功证据，也不要以离线 tests 或采集成功宣称左转通过。

先读 README、manifest、DEPLOY 和 reports。检查当前 cwd、Git branch/dirty state、共享 Unity/CPU 所有者。另一对话使用 Lane 模块；只按最新 handback 拍新基线，不能用旧快照覆盖后来的合法 UI/源码。

本次结果：v2 transport 24 PASS；实际 512 外圈 + 212 内圈包通过 capture binding/原 mask SHA 检查，各有一个旧 epoch 包在当前上下文被拒。外圈绿首圈完成 286.089m，但原融合 RMSE 3.410266m 失败。v2 内圈 64.875m 失去车道支持。曲线候选 18 synthetic PASS / 69 actual masks / 0 pairs；节点关联 31 synthetic PASS / 0 real association。详细原件在 evidence、experiments 和 reports。所有诊断开关默认关闭，无候选进入控制。

已确认的独立问题：

后续候选的具体失败与准备状态见 `reports/UNCONFIRMED_CANDIDATE_AUDIT_02.md`：量化曲线首次 26/27 PASS，真实 703-mask audit 未开始；video02 在编码前因 post-reset 身份混用失败；原 RGB/完整 plane API 正例仍待独立运行。不要把这些准备态升级为通过。

1. 稀疏官方 junction chord 不等于圆角实测外轮廓；官方 junction 不一定有车体转角。旧整体 fit 已失败，删转角门控不能解决它。
2. 现 LLP 经过 ROI/组件选择/直线化，不能提供完整横向支路；`left_only` 是当前车道边界模式。
3. 远处一行像素的投影阶梯会被固定 0.15m 分片误认成横线；拟合零残差不等于低量化误差。
4. 现 v2 曝光 pose 来源是 render root，不是精确同刻 Rigidbody。原 RGB UI 副本是重编码，不等于 input PNG 原字节；多帧原图算法需要新增严格原 packet 存档，不能放宽 hash。
5. 现局部地面假设和安装高度不证明世界路面。由原像素/真实曝光运动估计平面是后续 shadow 候选，禁止用预设平面回投后再 fit 验证自己。
6. 现 semantic detour 忽略新的视觉 ABC，并使用图节点直线。图匹配成功也不能证明内部道路可行。
7. 原逐段 tracker 无跨节点提前预瞄；红灯替代路线仅在 wait 选择，approach 可能错过入口。这是源代码条件性问题，未实跑证明。

下一步按依赖执行：

1. 在独占窗口测试新的量化曲线与 observation-derived plane shadow 候选；暂不改核心或发送 steering。补原始 RGB PNG packet 载体，并用 MCP 新 case 验证 SHA，保留旧 UI 副本及所有失败。
2. 从真实新帧建立匿名双边界/路口 station 证据，保留 UV、量化格、可见范围、epoch、capture 时钟/pose、竞争解释和误差；单侧、缺线、重复条纹或截断保持未知。
3. 只有足够真实观测才运行所有 cyclic assignment × 双方向、真实米制残差和 runner-up 歧义检验。保留原 whole-lap 失败，reference 长度/trust/真实点不改。候选 transform 是 official→odom，原 `OfficialToOdom` 字段方向相反，接入前必须显式转换并往返测试。
4. 必须修改 semantic 时先遵循已有问题报告，以单独 0.3.0 实现“官方意图 + 新鲜实测局部几何”的目标所有权；不允许旧图 chord 冒充道路。未知/旧帧/epoch reset 时停车，不刷新旧 ABC。
5. MCP 内外圈分别完成真实左入、内部通过、退出恢复；再重跑正常红灯约8–9m停车/绿灯继续。视频展示真实识别 overlay、LLP/ABC/LYA 与 semantic 状态，主画面至少75%；失败也保留独立视频。
6. 每场检查 worker/PID、cleanup、速度和最新完整 Edit 基线；更新版本、manifest、SHA、agent 交接、部署文档并上传 scoped branch/draft PR。不要 merge、force-push、覆写旧 evidence 或自动 tag。

使用新 case 目录与 owned runner；模型、测试、图像诊断和编码串行，每轮有时限与唯一停止动作。当前原始数据目录与绝对路径可从 evidence 中恢复。原生 ROS shadow 接线不连接 motor；真实 ROS/DDS/Orin/车辆验证未建立。
