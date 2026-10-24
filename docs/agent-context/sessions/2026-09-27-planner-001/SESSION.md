# Planner session: 2026-09-27-planner-001

## Objective and scope

用户要求把 agent context 写到 GitHub，规划 agent 负责 planning 和主要修改，实验日 agent
负责适配/调试、详细记录打包，双方异步交接。本次只改文档、离线打包/验证工具和其 CI；
不改 ROS 运行逻辑、不接车辆、不安装本地依赖、不动 main、不上传原始 bag。
本 session 封存的是规划准备及已有功能基线证据；发布 commit/新 CI 状态由后续回执补充，
不在包内预言尚未发生的上传/实验结果。

## Input handoff

接收 `user-request-2026-09-27-agent-context`（当前用户直接请求，首份 planner session 无前序仓库任务）。
已读三包说明、共享 VALIDATION、当前 GNSS 源码/工作流及 prior CI 证据。
特别确认用户纠正：GNSS 仅在开始/结束停稳窗口判定固定路线起点，不许建图使用。
新的输出任务 ID 为 `2026-09-27-planner-001`；现场 agent 尚未接收/ACK。

## Source and working tree

- 发布仓库 Sophia-AI-formula-team/aiformula_sophia，branch feat/causal-lane-teach-repeat。
- 实读 HEAD 和远端分支为 `7662f5a5174b7579afadd23291c8e023288ecdfc`；编辑前发布 checkout 干净。
- 另一份本地历史工作目录有大量与本任务无关的迁移/dirty 文件，未暂存、未提交、未清理。
- 本次 source delta：根入口/README、docs/agent-context、tools/agent_handoff.py、两个 test 文件、
  .gitattributes 和 agent-handoff workflow；没有修改三包功能源码。
- 同提交保存可读文件和本包；最终发布 SHA 由回执引用，避免记录自身 SHA 的循环依赖。

## Work performed

1. 只读 `git status --short`、`git rev-parse HEAD`、`git ls-remote origin refs/heads/feat/causal-lane-teach-repeat`，
   确認真实基线；读 README、源码、旧验证记录、CI JSON。发布 checkout 不承接原目录的无关 dirty changes。
2. `gh run view 36257132332 --repo Sophia-AI-formula-team/aiformula_sophia --json headSha,status,conclusion,url`
   查询已完成的原生 CI；只读核对其 artifact 中 runtime.json 的场景和断言，摘要见 evidence。
3. 创建分工、任务单、回执规则、runbook 和“不等于实车许可”的四层状态。
4. 并行只读审阅指出影子阶段不开 arm 无法产出 consensus、bootstrap/因果时间描述不严；已修正文档。
   工具审阅发现预签名外部 URL 可能泄漏下载凭据；现已拒绝 URL query/fragment，并加回归。
5. `python -B -m unittest discover -s tools/tests -p test_agent_handoff.py -v`：退出码0，
   27项中26通过、1因本机无 symlink 创建权限跳过；见 evidence/tool_validation.json。
6. 全仓库文档链接/封存包逐文件校验在本包生成后执行，由发布回执及 GitHub handoff workflow 留证。
   本包封存时未把这些后续结果写成已通过。

## Evidence and result

- [已有 Foxy/DDS 证据摘要](evidence/functional_baseline.json)：520 Linux 单测，9个静止DDS场景151断言；
  原始 runtime JSON 的 SHA256 保留用于来源核对，不含真实 GPS/个人路径。
- [真实 bag 既有阻塞摘要](evidence/known_bag_limits.json)：输入断流和缺GPS字段；不是完整地图/第二圈成功。
- [本次离线工具验证](evidence/tool_validation.json)：本机测试数、环境、跳过原因与未验证项。

结果为 partial：完成本地规划/工具验证，现场适配、完整移动建图/跨圈定位、物理急停均未执行；
没有现场接收 ACK，发布/新 CI 留待新回执，不把静止 synthetic DDS 当实车精度。
原始 CI artifact 有保留期限，已将审查过的必要数字/限制作为可读摘要随本包保存；
本包不含完整 CI 原始日志，也不冒充完整实验复现包。

## Risks and blockers

当前无本机 ROS 和车辆；已下载的原生 CI 不是 Windows 上新跑的测试。
RunJournal 默认轮转可能丢启动事件，GNSS 日志含坐标；现场要求额外参数/版本/阶段记录及逐文件脱敏。
现有真 bag 轮速超时，原始 GPS 缺 POSLLA；不得松门限或用 INS 代替来得到成功。
SHA256 是完整性校验，不是签名或来源认证；工具扫描不保证隐私，二进制图像仍需人工审阅。
没有启动/连接车辆，未发送任何 ROS 控制命令；现场最终安全确认由操作者负责。

## Next agent actions

现场 agent：读 AGENT_CONTEXT/STATUS 和 2026-09-27-planner-001，核对代码并发 RECEIVED；
创建 experiment 分支，先不接电机做真实输入/部署审计，保存全部结果及阻塞，打包回传。
规划 agent：收到后核验代码和包，发 ACCEPTED_FOR_ANALYSIS 或 NEEDS_DATA；
主要修改经过回归再发布下一基线，不能以 ACK 或 CI 替代实车授权。
异步只靠仓库任务/记录/回执接力；没有创建会自动醒来的第二 agent 或后台监控。
