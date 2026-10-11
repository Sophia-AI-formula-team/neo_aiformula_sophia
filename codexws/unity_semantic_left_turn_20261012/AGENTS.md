# Agent 交接指令

先读 README.md、PROBLEM_ANALYSIS.md、DEPLOYMENT.md、VALIDATION.md 和 manifest.json，检查 cwd、Git branch/dirty state、PR head/base。此归档版本 0.1.0-candidate.1；尚未安装、尚未 Unity 编译、标定闭环未验收，不能宣称左转已修复。

## 边界

1. 用户要求 Unity 操作用 MCP，禁止 Computer Use。Refresh、Play、scene change 和重模型/编码前，先与另一 Unity 对话取得 editor/CPU 独占；有限 CPU grant 到期不得复用。保存并恢复 editor scene、Play state、车体、preset 与 worker 所有权，结束后明确归还。
2. 地图模式问题仍等待用户回答。可以准备默认关闭的可审查结果；启用标定实跑前须取得明确选择，不把时间经过当作同意。标定是新的控制先验，不是自动 junction 识别。
3. 保留 semantic core63eec2d、LLP/LYA upstream05f1651c、相机/灯位/物理/scene、用户原包及其他 agent 工作。必要核心修改先写问题分析再修复。不得放宽融合、车道支持或时效门槛来制造成功。
4. `codexws/unity_semantic_integration` 是冻结历史，不能改写。候选包含完整 final63-before/after，不可只套用基于中间采集版本的小 patch。保留 before/overlay/evidence 原字节和 GUID。
5. `COLCON_IGNORE` 保持。本候选是 Unity stdio ROS shim，不是原生 ROS/DDS 接线；旧 native shadow 入口和版本不变。不得接入实车电机、自动 arm 或转移仿真参数。

## 继续验证

取得地图模式选择后，先验证候选完整哈希及 416 个前置文件。当前本机已经有采集接线，属于 final63 与候选之间的中间状态；installer 应拒绝强制覆盖。逐路径对照保留的 install receipts，不能 reset 项目。关闭 Unity的安装流程按 DEPLOYMENT；如果由 MCP受控安装，必须另外记录同等前置检查、精确文件写入和编译，不伪称使用了关闭Unity installer。

先完成 Unity 编译和默认关闭回归；标定首跑明确 `projectedRecovery=true,currentCorridor=false`。外圈可使用 `fixtures/real_outer_fusion_capture.json` 的严格保存 reference，标 `warm_start` 和 `fresh_learning_proof=false`。内圈必须先采到新的真实闭合首圈及完整 fusion-call snapshot，再考虑加载。不能暗用外圈轨迹，不能用 evaluator route 冒充自动学习。

实际两圈需证明入口物理左转、内部右转、出口左转、显式完成及道路轨迹，记录 fresh RGB、命令/odom时效、控制所有权、worker/epoch连续性与清理。重点审查内圈1.296 m入口偏移与2 m门槛；不要盲调 LYA 或放大门槛。每个可运行失败也保留独立视频和诊断。

未实现自动 junction learning、反向/其他 detour、新 native DDS 或实车验收。证据须分列 unit、保存reference重算、供应protocol tick、真实Unity运动和视频审查；不合并测试数量来证明闭环。

## GitHub

此分支堆叠于 integration434fdc4，保留 PR#15/#16/#17。用户已授权 scoped GitHub upload；使用 feature branch/draft PR，不 merge、force push、移动标签或创建 release。新 PR 创建后调用 attach_artifact。不提交权重、原始 RGB、视频、cache、secrets或其他工作树更改；更新 manifest 时保留历史来源与失败结果。
