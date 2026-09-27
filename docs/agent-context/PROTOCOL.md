# 两个 agent 的异步交接规则

目的：换电脑、隔几天、换 agent 后仍知道“基于哪版、做了什么、哪些没过、谁接下来做什么”。
不要求同时在线，也不以聊天内容作为唯一证据。

当前唯一远端：`Sophia-AI-formula-team/neo_aiformula_sophia`，仓库 ID `1303517209`。
每次 push 前先核对 `git remote get-url --push origin` 与 `gh api repos/Sophia-AI-formula-team/neo_aiformula_sophia --jq .id`。
任何不匹配都停止发布，不自动改成旧仓库。旧任务、封存 session/ZIP/回执只用于审计，
它们的旧路径和已清理的 Actions URL 不再是当前指令；必须使用 STATUS 的新任务与新基线。

## 分工与写入归属

| 角色 | 可以做 | 要先返回讨论的事 |
| --- | --- | --- |
| planner | 任务设计、主要实现、回归、审阅、整合、更新 STATUS/新基线 | 超出用户授权的仓库/硬件操作 |
| experiment | 指定分支上的话题/路径适配；有测量依据的标定、轮径、gyro bias；兼容修复、诊断与记录 | 安全门限、运动源、控制律、状态机、地图融合/坐标语义、电机/急停逻辑、提速 |
| 现场操作者 | 本次通电/动车范围、物理停车链验证 | agent 不能替人确认 |

实验 agent 不修改规划 agent 正在使用的整合分支；不 push main、不强推、不自动合并。
任务授权上传本仓库的交接/实验分支，不等于授权向其他组织、Issue、聊天或公开原始数据。
没有 GitHub 凭据时保留本地包，报告 BLOCKED_UPLOAD；不能声称已交接。

## 文件约定

- `AGENT_CONTEXT.md`：稳定背景/不可变约束；planner 更新。
- `STATUS.md`：当前任务和最新回执索引；planner 在整合分支维护。
- `handoffs/<id>.md`：不可覆盖的任务单。包括输入任务、角色、base SHA、允许路径、验收、上传目的地。
- `receipts/<id>.md`：不可覆盖的收到/结果/审阅回执；使用 [模板](templates/RECEIPT.md)。
- `sessions/<id>/`：一项工作/实验的可读记录、元数据、脱敏证据；打包后视为封存。
- `bundles/<id>.zip` 和 `.sha256`：上述 session 的可校验副本；小包随 Git 长期保存。
  CI 临时 artifact 不是唯一档案。

ID 用日期、角色/台架和短名，例如 `2026-10-01-experiment-sensors-01`。
每次重试新 run ID；失败、被打断、零帧、未动车也要记录，不只上传成功样本。
同一 session 只有一个写入者；打包后更正用新 session + `supersedes` 文字链接，不覆盖旧证据。
并行做别的工作前核对 dirty state，不能把别人的修改归入自己。

## 一次完整往返

1. **planner 发出任务**：任务单固定代码基线、输入、允许范围、验收和期望回传。
   发布 commit 后给出 GitHub commit/任务链接。任务单不写自己尚未生成的 commit SHA。
2. **experiment 接收**：read context/status/task；记录实际 SHA、运行代码差异、现场环境和权限。
   新建 `RECEIVED` 回执，注明收到哪个任务的哪个提交。若基线不符，先只做只读检查，
   发 `BLOCKED_BASELINE`，不能假装在同一版本上继续。
3. **experiment 执行**：创建独立实验分支和 session，按 runbook 分阶段验证。
   每次命令、退出码、参数变化、观察、失败原因随做随记。每个能改变结论的试验单独 run ID。
4. **experiment 返回**：完成或卡住都上传可读记录、代码/patch、脱敏包和 SHA256。
   新建 `RESULT_READY` / `BLOCKED` / `INTERRUPTED` 回执，列出本地保留但未上传的证据，
   在交接消息中给精确 commit/包链接、通过/失败/未执行项和希望 planner 决定的问题。
5. **planner 接收**：取回指定 commit，验证哈希和实际代码差异；先回
   `ACCEPTED_FOR_ANALYSIS`（只是证据可用）或 `NEEDS_DATA`（具体缺什么）。
6. **planner 整合**：审查并回归后才发 `ACCEPTED_CHANGE`（逐个列出接收 commit），
   更新 STATUS，并发下一任务的新基线。拒绝/暂缓也要明确理由，不默默丢弃现场改动。

收到、证据可信、代码接收、实车放行是四件不同的事。任务结束不自动启动下一圈。
必须通过下一轮的实际用户启动/消息接力；文档不会创建后台 agent 或自动通知服务。

## 打包命令（仓库根目录，Python 3.8+，无第三方依赖）

```sh
python3 tools/agent_handoff.py init --root docs/agent-context/sessions \
  --session-id 2026-10-01-experiment-sensors-01 --role experiment \
  --base-code-sha 93761b5f4df2cbcf87ac01f533e992fdbd33c456 \
  --handoff-id 2026-09-27-planner-neo-003
```

日期/ID 只是例子，必须换成本次实际值；基线 SHA 如已被新任务取代，也应更新。Windows 使用 `python` 并将命令写为一行。
填写生成的 `SESSION.md` 和 `session.json`，将审查后的证据放 `evidence/`。
`input_handoff_id` 与 `consumed_handoff_id` 要指向实际读过的任务；它们不能代替含 commit 的回执。
结束时 `status=handed_off`，`result` 选择 `pass/partial/blocked/not_run`，不能把没做写成 pass。
只有人工/agent 逐文件检查完才能将两个 `publication_review` 布尔值设 true。

```sh
python3 tools/agent_handoff.py validate --session docs/agent-context/sessions/2026-10-01-experiment-sensors-01
python3 tools/agent_handoff.py pack --session docs/agent-context/sessions/2026-10-01-experiment-sensors-01 --output docs/agent-context/bundles/2026-10-01-experiment-sensors-01.zip --confirm-reviewed
python3 -m unittest discover -s tools/tests -v
```

工具**不执行 ROS/车辆命令、不提交、不上传、不代替真实内容审阅**。
检查通过只是结构/部分泄漏检查通过，不保证科学结论正确或已脱敏。
约束：每文件 ≤5 MiB，总量 ≤20 MiB、≤512 文件；拒绝原始 bag、隐藏文件、越界链接等。
压缩包内 `hashmanifest.json` 保存逐文件 SHA256，旁边 `.sha256` 校验整个 ZIP。
ZIP/校验文件分别原子发布，不是跨两文件断电事务；接收者必须确认两者齐全并核验才能接收。
输出已存在就报错，不能覆盖历史；新证据用新 ID。

小包、可读记录、代码和回执一起明确路径 staging；不要 `git add .` 带入现场其它文件。
提交前 `git diff --cached --stat` + 检查完整 staged diff + 再次逐文件隐私检查；
push 自己的授权实验分支后通过 GitHub 读取确认存在，记录远端 commit。
先上传 session/包，再用一个新回执提交引用它们的 commit，避免回执要包含自身 SHA 的循环。

## 大文件与保密

原始 bag/视频/精确 GNSS/网络地址/个人路径不默认进入普通 Git或公开附件。
保留本地受控原件，生成脱敏的小报告/必要片段；记录脱敏方式、原件是否可提供、片段是否缺初始化。
原件哈希/文件名也应审查是否泄露身份。不要为了好看的结果修补源时间或删掉失败段。
超过本项目小包限制：先确认目标存储的访问权限和人类授权，再放 GitHub Release 附件或
批准的受控存储。仅将 `external_artifacts` 清单（URL、SHA256、bytes、access note）放包内。
工具拒绝带 query/fragment 的附件 URL（包括预签名下载凭据），只存稳定的 HTTPS 入口。
工具不会抓取/验证链接内容；接手者必须另行验证权限、下载哈希和可复现性。
不要擅自启用 LFS、改仓库可见性、移动历史或安装上传工具。
