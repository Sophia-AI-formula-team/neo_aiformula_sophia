# 2026-09-28-planner-neo-integration-004

- 日期：2026-09-28 UTC；角色：planner；类型：RESULT_READY（代码与隔离测试），不是实车放行。
- 输入：用户要求修正已确认接线、以 neo 为准并跑通；最新速度要求是默认继承 LYA 当前参数，
  仅明确 config 覆盖，不录放首圈速度、不固定复制 2.0。起始代码 ef7e5ceb7dbf0b730e34f1f9e5de2a0e33807f23。
- 唯一仓库：Sophia-AI-formula-team/neo_aiformula_sophia，ID 1303517209。
  分支：feat/causal-lane-teach-repeat；**未合并 main**，未新建 PR，未修改旧仓库。
- 新任务：[neo-004](../handoffs/2026-09-28-planner-neo-004.md)，取代 neo-003。
- 指定功能基线：[bab577b](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/bab577b9cb71e5459301c785843ef54ba4a2cefa)。
  接收本轮 020df63、900b555、bab577b 的实现/回归；不代表接受尚不存在的现场改动。
- session/任务/证据已先上传于
  [f039a69](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/f039a6937ad2d7cab215679f2120911c9c157f1e)，
  本回执再引用它，避免自身提交循环。功能基线之后只有文档/证据变更。
- [SESSION](../sessions/2026-09-28-planner-neo-integration-004/SESSION.md) /
  [ZIP](../bundles/2026-09-28-planner-neo-integration-004.zip)：356357 bytes，
  SHA256 `a9f57d370bef3fb745c5453b08df16099c457d25cc7bd7d17c8f07488e701ada`。
  CRC、全部 12 个封存文件与提交 blob 字节一致；validate/pack/diff-check exit 0。
  隐私扫描命中的坐标是 runtime_smoke.py:60 的合成 [35,139,10]，不是实车轨迹或原 bag。

## 实际完成与证据等级

修正完整 mask 话题、误判 ROI 的黑名单和源 Header；实际 lya_0221/运行依赖接入；
LYA 与 repeat 共享当前源码参考，空 launch override 不覆盖配置；保留通过准入的首圈/回退原命令。
修正 GNSS fixed-reference bundle 的 build→load→worker 路径；添加竞争 publisher 检查与明确启动冲突诊断。
更新主动文档、现场 runbook 和逐项回传要求。未放宽物理、时效或 GNSS 使用门限。

- Windows 最终本地回归：619 passed、1 平台 skip，17.29 秒；Python 3.8 AST 24 文件通过。
- [Foxy参考/固定](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/36427591132)：
  构建真实 LYA/原生消息/包；354 单测、4 Header 单测、63 项既有 DDS 检查；
  真实 LYA 默认/全局 YAML 覆盖、进程停机、完整 Header 传输共 24 项检查。
- [Foxy GNSS](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/36427591136)：
  三包共 616 单测（含前述354，不重复相加）、4 Header 单测、9 合成 DDS 场景151项检查。
- 原生真实 LYA 观测：源码默认 2 → 2/2.15；ROS YAML 参考 4 → 4/4.15。
  后者仅验证私有 supervisor 参数传播，不证明 GNSS raw-motion 或车辆获准运行4。
- 合成20m半径闭环：125.643m一圈、64.85s，横向RMSE0.01651m、最大0.10m；
  5m/9m半径按原门限拒绝。理想运动学/精确当前位姿，不是建图或真实定位精度。
- 交接工具本地33项：32通过/1 Windows symlink权限skip；打包前后均验证。
  文档提交与本回执的 CI 以相应 GitHub checks 为准，不把旧run冒充新提交结果。

## 失败、边界和下一责任人

首次真 LYA CI 因缺 transforms3d、child exit255 失败，已按官方 Focal rosdep 补声明，
只在一次性 CI 固定安装0.4.2并重跑通过；失败报告永久保留。Docker本机启动失败，未安装本机依赖。
AST fixture/错误测试路径等开发失败也记录，没有只保留成功。实际模型推理、真实传感器闭环、
合格真 bag 整圈、物理急停/电机、真正第二圈精度均未执行。

安全状态：未连接/启动车辆，实车开关仍false、默认私有输出。非私有输出须经确认的正数教师上限。
GNSS只用于两个停稳起点许可窗口，不参与建图。既有 raw-motion 3m/s 与参考4冲突时启动拒绝，
不得暗降参考或提高门限；repeat 0.35名义横向门限也不是反馈实际 v*omega 的硬安全保证。
neo teleop输出所有权、下游watchdog、标定/时间连续性仍需现场证据。

下一责任人：experiment agent 先 ACK neo-004 与精确代码/安装路径，在独立 experiment 分支
按runbook做只读/隔离输入与停车链核验；任何实车阶段由现场操作者另行许可。
需planner决定的问题：实测运动/标定、控制可行性、命令仲裁及门限适配，用完整日志回传后讨论。
未收到ACK，不能声称对方已部署；不自动唤醒另一个agent或启动下一圈。
