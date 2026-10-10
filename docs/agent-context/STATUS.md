# 当前异步交接状态

最后整理：2026-10-11。这里是索引，不是代替不可覆盖的原始记录。

## 2026-10-11 仿真前修复检查点（尚非现场放行）

用户已授权修复建图核心，禁止修改车辆/仿真参数。修复在
`repair/lane-keyframe-transaction-20261011`，版本检查点
`lane-mapping-20261011-r1`；修改前标签为`lane-mapping-20261011-baseline`。
候选参考必须验证、提交后才能取代 active，刷新不再重开首次定位门限。
[修复记录及可重放证据](sessions/2026-10-11-planner-keyframe-005/SESSION.md)：
657项离线测试通过，1项Linux限定测试跳过；已记录故障前缀由31/32变为32/32。
**完整第一圈建图、第二圈定轨迹和实车仍未通过**。该代码位于明确的修复分支/tag，
不能默认 clone main 就已包含这些包；现场任务基线暂不自动更新。
R1代码`422cb4b`已发布，Foxy构建/通信与跨平台档案检查均通过；
精确运行链接和计数见[发布回执](receipts/2026-10-11-planner-keyframe-r1.md)。

## 第二个修复检查点：同一会话的 repeat 定位交接

[session006](sessions/2026-10-11-planner-repeat-handoff-006/SESSION.md)修复了切换时
清零已提交定位修正的问题。小型不可变状态在冻结点捕获，全图索引留给后台；
不继承 pending/refinement，不重开首次门限，不把旧 teach 帧算成三条新定位确认。
已发布版本`lane-mapping-20261011-r2`（`751edef`），沿用修复分支，不移动R1。
本地691项通过、1项Windows限定跳过；仿真适配160项通过。
Foxy构建/通信和跨平台档案CI已通过，精确运行和计数见
[R2发布回执](receipts/2026-10-11-planner-repeat-r2.md)。这不是完整两圈成功声明。
实际R1 trial14建图101/101帧通过，约26.628m后旧教师视觉前端失败；
尚无冻结路线/第二圈。新版完整LYA前端旧轨迹离线支持200/200，但全部为
沿用2.6m宽度假设的右侧单线支持，不等于两侧识别或新策略闭环通过。
实际R2 trial15在启动前因批次最旧gyro超时停止；trial16通过启动并建图105帧后，
在斑马线场景的匹配创新/最终内点检查处停止。无冻结路线、无第二圈。
失败输入和停车快照保留，继续修复，不能以重试启动成功宣称时序架构已解决。

## 原现场交接索引（保留历史基线）

| 项目 | 状态 |
| --- | --- |
| 规划 agent → 实验 agent | 已发出任务单，等待现场接手 ACK |
| 唯一发布仓库 | `Sophia-AI-formula-team/neo_aiformula_sophia`，ID `1303517209` |
| 当前任务 | [2026-09-28-planner-neo-004](handoffs/2026-09-28-planner-neo-004.md)，取代迁移任务 neo-003 |
| 指定功能代码基线 | `bab577b9cb71e5459301c785843ef54ba4a2cefa`（neo 本仓库） |
| 整合分支 | `feat/causal-lane-teach-repeat` |
| 现场实验分支 | 接手时创建并写入 RECEIVED 回执；尚未分配 |
| 旧规划档案（非当前部署指令） | [SESSION](sessions/2026-09-27-planner-001/SESSION.md) / [ZIP](bundles/2026-09-27-planner-001.zip) |
| 旧发布档案（原始哈希保留） | [回执](receipts/2026-09-27-planner-publication-002.md) / [ZIP](bundles/2026-09-27-planner-validation-002.zip) |
| 迁移和清理记录 | [迁移说明](migrations/2026-09-27-neo/README.md) / [SESSION](sessions/2026-09-27-planner-migration-003/SESSION.md) / [ZIP](bundles/2026-09-27-planner-migration-003.zip) |
| 当前发布回执 | [2026-09-28-planner-neo-integration-004](receipts/2026-09-28-planner-neo-integration-004.md)，固定证据提交f039a69 |
| 旧迁移发布回执 | [2026-09-27-planner-neo-publication-003](receipts/2026-09-27-planner-neo-publication-003.md)，固定证据提交44a5005 |
| 旧目标清理 | 本任务旧分支、11次CI、8个artifact已删除；main/其它分支/PR保留；原件备份可恢复 |
| 新仓库 CI | [Foxy参考/固定](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/36427591132)、[Foxy GNSS](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/36427591136) 成功：三包616单测+4 Header单测、63/151 DDS项、24项真实LYA/完整Header检查 |
| 本轮修复与交接证据 | [SESSION](sessions/2026-09-28-planner-neo-integration-004/SESSION.md) / [ZIP](bundles/2026-09-28-planner-neo-integration-004.zip)，含首次CI失败与修复 |
| 实验 agent 回传 | 尚无；不能推断已读、已部署或已实验 |
| 实车许可 | 未取得；默认私有输出和关闭实车开关 |

迁移保留了两份封存包和旧任务/回执的原始内容，不篡改其历史仓库名/旧 SHA。
那些文件不再提供当前发布指令；原始 Actions 链接会随用户要求的旧仓库清理失效。
只使用本页所指的 neo 任务、基线和新 CI。Header、默认话题、教师程序与单源速度继承已修复；
现场标定/时序、命令所有权、物理急停与真实第二圈定位仍未验收。默认源码速度不是驾驶许可，
参数继承也不自动放宽轮速/曲率等门限；详见新任务。

任务单里的 `base_code_sha` 是功能代码基线，后续单纯文档/打包提交可以在其后。
接手 agent 仍须记录实际 checkout SHA，并核对该 SHA 到基线之间的运行代码差异。
如运行代码变了，需要规划 agent 重新指定已验证基线，不得自动接受。

## 如何更新

接手/结果回执存 [receipts](receipts/README.md)，每次新建，链接指定任务和 session。
新任务单存 `handoffs/<唯一ID>.md`，旧任务不改写。规划 agent 在整合分支更新本索引；
实验 agent 在自己的分支提交回执和实验记录，在交接消息中提供精确 commit 链接。
实验分支尚未整合时，本索引可能落后，接手者必须同时查看授权分支和最新回执，不能只读此表。

仓库实现的是**异步存档/接力**，不是一个常驻的自动通信服务。
下次任一 agent 被用户启动时，先拉取并检查新回执再行动；未启动的 agent 不会自动醒来。
