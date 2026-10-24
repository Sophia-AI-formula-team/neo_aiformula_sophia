# 2026-09-27-planner-publication-001

- 角色：planner；类型：BLOCKED（交接工具 Windows CI，非现场实验）；日期：2026-09-27 JST。
- 输入：用户 agent-context 请求；对应 session：`2026-09-27-planner-001`。
- 任务/证据提交：[38ad8699f786b61926bc708d9564374e17519b6c](https://github.com/Sophia-AI-formula-team/aiformula_sophia/commit/38ad8699f786b61926bc708d9564374e17519b6c)。
- 功能基线仍为 `7662f5a5174b7579afadd23291c8e023288ecdfc`；`git diff` 确认本次 ROS 源码无变化。
- 包：[2026-09-27-planner-001.zip](../bundles/2026-09-27-planner-001.zip)，12044 bytes；
  SHA256：`2f32f685070e1d02d3ec9d60d06ffb75f5f01691fbe7bf66408d18f2b78bc38e`。

## 实际完成

`git push origin HEAD:feat/causal-lane-teach-repeat` 退出0。随后用 GitHub contents API 读取入口和包，
包 blob SHA 与本地 `git hash-object` 一致：`40df9b3035da5633b790eb0b28635411e9a675cc`，大小一致。
本地 `python -B -m unittest discover -s tools/tests -v` 退出0：31项，30通过、1 Windows真实symlink权限跳过。
第一次普通 `git add` 遇 sparse checkout 路径限制；检查后仅对本次 docs/tools 使用 `git add --sparse`，
没有关闭 sparse 或混入其它源文件；staged diff/whitespace 均核对。

## 失败与后续

[第一轮交接 CI 36258746524](https://github.com/Sophia-AI-formula-team/aiformula_sophia/actions/runs/36258746524)：
Ubuntu Python3.8/3.13检查通过，Windows Python3.13中4个模拟注入测试失败：
输出抢占、sidecar抢占、读后修改、模拟reparse；预期异常未抛出。包/文档/哈希检查已通过，
Windows真实symlink测试也通过。

这些模拟都通过 Path 相等判断注入位置；调查 Windows临时目录短路径与已resolve路径不一致，
先修正测试夹具路径再重跑全部矩阵，不删除/放宽断言。完整原始 CI 日志在上述 run，未覆盖或重跑抹掉失败记录。
本回执只记录发现时的证据；最终修复提交/新 CI 用后续回执引用，不修改已封存 session/ZIP。

当前安全状态：没有车辆连接、没有执行 ROS 控制/硬件命令。现场 agent 仍未 ACK。
下一责任人：planner，完成测试根因修复并验证；没有需要现场 agent 绕过的门限或实车授权。
