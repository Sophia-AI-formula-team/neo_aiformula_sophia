# Planner publication validation: 2026-09-27-planner-validation-002

## Objective and scope

封存第一份交接的发布验证及 CI 修复。此处 pass 仅表示仓库交接文档/离线工具发布检查通过，
不是车辆实验或 ROS 新一轮验收。保留原始 session/ZIP，不覆盖其中“发布待验证”的历史状态。

## Input handoff

接收 `2026-09-27-planner-001`；实际已读任务/规划 session、协议、打包工具及 CI 日志。
第一份发布提交为 `38ad8699f786b61926bc708d9564374e17519b6c`。
现场 agent 尚未接收，不在此代其 ACK。

## Source and working tree

组织仓库 Sophia-AI-formula-team/aiformula_sophia，分支 feat/causal-lane-teach-repeat。
功能基线 `7662f5a5174b7579afadd23291c8e023288ecdfc` 不变。
当前已验证工具/文档提交 `dc2cacfa1918851fb08d937629b1bd77f48f2daf`；
`git diff 7662f5a5174b7579afadd23291c8e023288ecdfc HEAD --exit-code -- pid_ws/src` 退出0。
修复只改测试夹具及回执，没有更改打包生产代码、原始证据、ROS 算法/参数。
封存前该 checkout 干净；本次新增的是本验证记录/包和最终回执/索引。

## Work performed

1. 首次 `git add` 因 sparse checkout 未暂存 docs/tools；核对后只对新路径加 `--sparse`，
   staged diff/check 通过，再 commit/push 38ad869，均成功。
2. GitHub contents API 读取 AGENT_CONTEXT 和第一份 ZIP；远端包大小12044 bytes，
   blob SHA `40df9b3035da5633b790eb0b28635411e9a675cc` 与本地 git hash-object 相同。
3. 首轮 CI 36258746524：Ubuntu 两组成功，Windows 暴露四个模拟故障未触发。
   失败记录保留在原 run 和 publication-001 回执中。
4. 根因：fixture 保留临时目录词法别名，helper 解析规范路径，四个 Path 比较不相等，未注入故障。
   改为 fixture root.resolve(strict=True)；保留所有原异常/内容断言，新增 intermediate/.. 别名回归，
   复用四类故障断言。没有禁用 Windows 测试、删除断言或放宽安全逻辑。
5. `python -B -m unittest discover -s tools/tests -v` 退出0：32项，31通过、1本机权限跳过，0.764秒。
   `git diff --cached --check` 退出0；提交并 push dc2cacf 成功。
6. `gh run view 36258939592 --repo Sophia-AI-formula-team/aiformula_sophia --json status,conclusion,jobs`
   确認 completed/success，Windows/Ubuntu × Python3.8/3.13 四组全部成功。

## Evidence and result

见 [CI evidence](evidence/publication_checks.json)。本地与远端都验证了打包、文档链接、
可读记录和 ZIP 字节/哈希一致性；4组远端环境通过。修复提交的 GitHub CI 是实跑，不是语法检查。
本份封存包及回执随后入仓库，不能把自身未来提交 SHA 写入自身内容；以发布它的 Git commit 追溯。
原包状态 partial 保留，发布检查在本份结果中补完。

## Risks and blockers

无阻塞的交接工具问题仍在等待修复；但没有现场 agent 的 ACK、没有实车验证。
真实 bag/标定/停车链/完整移动建图和跨圈定位缺口仍按任务单处理，不能因这里 pass 而放行。
两份 ZIP 与 sidecar 均应完整校验；哈希不等于签名。自动扫描不能取代隐私审阅。
没有车辆连接/控制指令；没有上传原始坐标、bag、凭据或机器个人路径。

## Next agent actions

现场 agent 按任务单确认版本并回 RECEIVED，再做不连接电机的现场输入/环境审计。
返回自己的 experiment 分支、每次 session/证据 ZIP/哈希和结果回执。
planner 下次被启动后读取返回证据并答复；未建立自动唤醒/后台通信。
