# 当前异步交接状态

最后整理：2026-09-28。这里是索引，不是代替不可覆盖的原始记录。

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
| neo发布回执 | [2026-09-27-planner-neo-publication-003](receipts/2026-09-27-planner-neo-publication-003.md)，固定证据提交44a5005 |
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
