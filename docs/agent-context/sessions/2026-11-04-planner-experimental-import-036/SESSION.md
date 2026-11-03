# 2026-11-04-planner-experimental-import-036

## Objective and scope

用户在看到 diff 和已知问题后明确要求：“因为这是实验使用的最新版本，所以先merge进去，有问题再回退”。
本 session 作为 planner 执行其仓库合并授权；不改变实验算法来修复已知问题，不操作车辆。

## Input handoff

已阅读 root AGENTS、现有 STATUS/PROTOCOL、semantic 源码说明及上一轮审阅。
承接 2026-09-28-planner-neo-004 的保留历史要求，本次用户选择覆盖旧的“暂不提交主线”建议。

## Source and working tree

使用独立 experiment/import-1027simtest-20261104 分支和工作树；main 基准 bef4e78415ce04e87351000dfed6a4a08fe9ea78。
原 neo 工作树和未跟踪 biased_mppi 保持原样。来源的 442+24 个文件及 SHA256 见 evidence/source_manifest.json。
没有删除 main-only 文件；motor、launchers 与 PR18 的五个受保护源码树均未改动。

## Work performed

按交付 tar 的 source/ 映射导入 perception，另导入四个 simtest 包。466 份暂存源码与输入字节一致。
Windows 首次按逐路径参数执行 git add 因命令行长度失败，改用 NUL pathspec 文件。
泛化的 core 忽略规则及 vendor JSON 忽略规则阻止了 8 份清单内源码，随后仅对明确清单使用 -f；未强制加入缓存或权重。
现有 Header/持久化/门控/脚本结束问题按用户要求保留，在 README 中显式记录。

首次归档校验发现 session 根目录不允许 README.md，已移至 evidence/selection_and_rollback.md 后重新校验；来源文件不变。

## Evidence and result

重新运行导入后的 semantic subset：228 项通过。
保留的 main Header contract：1 项通过、3 项失败，与用户已审阅的问题一致，未删改测试。
先前本包 254 个 Python 文件 Python3.8 AST 通过；此次所有导入 Python 文件亦已解析。
源哈希/受保护树检查通过，未运行 C++、原生 ROS、模型推理或实车。

## Risks and blockers

已知接口回退是用户明确选择的实验版本，不宣称全部测试或整车验收通过。完整 CI 不作伪绿处理。
版本回退和 race policy 不兼容、参数/文档矛盾、simtest 生命周期问题均见 evidence/selection_and_rollback.md。
无新增依赖安装、硬件控制、tag/release、强推或旧记录覆盖。

## Next agent actions

RESULT_READY：经普通 PR merge 将本候选发布到 main，核对远端全部来源哈希、motor备份和PR18归档。
若需回退，用该 PR 的 merge commit 做 mainline revert；备份引用 backup/pre-1027simtest-20261104。
后续修复或实际实验另起任务，不因 merge 自动启动车辆。
