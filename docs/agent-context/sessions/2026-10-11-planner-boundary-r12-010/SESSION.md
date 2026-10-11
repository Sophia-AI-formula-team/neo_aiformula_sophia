# Agent session: 2026-10-11-planner-boundary-r12-010

Input handoff: 2026-09-28-planner-neo-004

**REVIEWED / PARTIAL。R12 是失败的开发候选检查点，不是可部署版本。**
主 planner 已逐文件阅读本目录八个文件并完成数据最小化/隐私/秘密审核，允许仅将此
摘要打包到唯一仓库的开发分支。以下工作步骤和风险部分保留准备时状态，末尾的
审阅记录是本次封存决定；远端提交与CI另行回读。单测通过不构成动车许可。

## Objective and scope

目标仍是完整 mask + 因果 raw wheel/IMU 建图，LYA 开第一圈，随后同一连续会话中
关闭受管 LYA，使用冻结路线和视觉定位跑第二圈。R12 尝试修复填充白带的几何表示：
增加独立真实白黑边界层、已确认拓扑图及有界局部曲线位置估计。完整填充 consensus
继续用于展示，不以当前帧给自身定位作参考。

投票确认、几何准入、40/80 ms 时限、LYA 参考速度、车辆和仿真参数不因失败放宽。
GNSS 不建图；真值仅在估计输出封存后独立评分。此档案只汇总源码/测试及失败证据，
不授权安装、硬件操作、新闭环、修改门限或上车。

## Input handoff

已读根 AGENTS、AGENT_CONTEXT、STATUS、完整 PROTOCOL 和输入任务
2026-09-28-planner-neo-004。历史现场任务提供安全约束，不意味着其旧功能 SHA 等同于
本次基线；parent 明确指定下述 R11 检查点为开发基线。

唯一远端 Sophia-AI-formula-team/neo_aiformula_sophia，仓库 ID 1303517209。
本轮没有 push；发布前仍须重新核验远端 URL 和 ID。

旧 009 档案仍是本地未封存过程工作笔记，不能当已发布交接证据。010 是本次精确检查点
的独立摘要，不覆盖、封存或为旧档案背书。原始历史记录原位保留。

## Source and working tree

- 实际 HEAD / 开发基线：408762fb7d1428c73b6cd3d6bc0946c92f0d9a35，R11 tag
  lane-mapping-20261011-r11-rc1。
- 分支：repair/lane-keyframe-transaction-20261011；R12 是脏工作区增量，不能说 HEAD 已包含它。
- 147 项源码/配置/原生文件在完整核心测试前后相同，见
  [tested_source_hashes.json](evidence/tested_source_hashes.json)。
- 主要变化：共享 paint_geometry.py、boundary_localization.py、mask_localization.py；
  _gnss 的 mapping.py、worker.py、bundle.py 及相关测试。graph v2 在 schema3 bundle 中
  保存13个数组，包括完整入度/出度，subset 不得将分叉伪装为普通内部接点。
- 最新已测 paint SHA：95693e0024e0cc7194c9c6cae7ed6c7bfdc311e2911d84c516fc6df6ec71cb7a；
  mapping SHA：5f6bcb6b3340cd30d337fc967ebcdcb09ddf568150f8e94e4311c74281414b01。
  早期结果绑定各自源码，不能互相替换。

没有安装依赖或编译新二进制；使用既有 Windows CPython3.13 原生模块。其七类 API 实际
调用、C 源 SHA 和实际加载 binary SHA 已核验，详见核心摘要。这不是 Foxy/ROS 构建。

## Work performed

命令以 canonical 仓库根为基准；D 表示操作者指定的本地诊断输出目录，O 是受控历史帧
目录。个人绝对路径不进入此档案。

1. 只读检查严格过去参考、完整前景分母、raw/model 分工、拓扑保存及同会话 handoff。
   发现 TEACH 准备/guard 入口异常可留下 accepted proposal；新增6项真实 matcher 回归，
   原实现6项均暴露延迟 commit 可成功。主 planner 增加 process finally 清理。
   这不是已提交后多存储原子回滚声明。
2. 执行实际原生调用核验，exit0：

   ```text
   python -B workspace/src/aiformula/control/lane_mapping_lya_reference/scripts/verify_native.py --source workspace/src/aiformula/control/lane_mapping_lya_reference/native/lane_graph_native.c --output D/r12_native_identity_03.json
   ```

3. 独占 CPU 执行一次完整核心测试，exit0；PYTHONPATH 为 lane_mapping_lya_reference、
   lane_mapping_fixed、lane_mapping_fixed_gnss、trajectory_follower 四个 package root。
   环境 LANE_REQUIRE_NATIVE=1、LANE_TEST_REPO=仓库根、LANE_REAL_FRAME_DIR=O、
   PYTHONDONTWRITEBYTECODE=1。

   ```text
   python -B -m pytest workspace/src/aiformula/control/lane_mapping_lya_reference/test workspace/src/aiformula/control/lane_mapping_fixed_gnss/test -q -ra -p no:cacheprovider --junitxml=D/r12_full_native_01.xml
   ```

4. 本次整理仅读取已完成的准确度、质量失败、缺口、prefix04/05/06 报告；没有重跑
   估计器、矩阵、adapter、ROS、Unity。原始日志/图像/数组/二进制留在受控本地。
5. 用 tools/agent_handoff.py init 创建此新 session；只编辑本目录，不修改 STATUS、
   AGENT_CONTEXT 或旧档案，不 pack、stage、commit、push。首次编辑补丁因同一目标
   使用两种操作被工具拒绝，未改变文件；随后改为单次更新，未覆盖任何旧证据。

## Evidence and result

**总体 result=partial：算法质量和实时两圈目标仍失败。**

| 证据 | 实际结果 | 限制 |
| --- | --- | --- |
| Windows 原生核心 | 2396 passed、1 skipped，pytest67.22s，exit0 | 非 ROS DDS、时限或驾驶验收 |
| 七类原生 API | 实际调用通过，C 源/加载 binary 身份一致 | 本轮未新编译/安装 |
| 完整 adapter attempt01 | 318 tests，21 failures、7 errors，83.923s，exit1 | 失败分类待审，不能称适配通过 |
| _04 共同已提交帧151–247 | R12 横向RMSE0.006386860m，R10A0.220024830m，raw0.004596479m | 比R10A好，仍比raw差；非全圈 |
| 四个预声明开发案例 | 0/4达标；每案7/509接受，frame7拒绝 | exit0仅表示失败被完整保存 |
| recorded replay05 | 101/102接受；seq252 engine92.04999998ms超80，未提交 | 非新闭环或完整111帧通过 |
| recorded replay06 | 104/105接受；seq255 engine84.19130003ms超80，未提交 | 新源码仍失败 |
| 最新 actual | 仍为R10A trial28失败，R12没有新actual lap | 无首圈完成、冻结路线或第二圈 |

唯一核心 skip 为 Windows 不适用的 Linux process-group 测试。准确度固定首 TEACH
起点、同 capture sequence、只取共同已提交帧，不作后续对齐/缩放，不把拒绝返回的raw
pose当校正成功。R12 yaw RMSE0.000940494rad仍比raw0.000646559rad差；总位置
RMSE0.284631732m也未优于raw0.284596967m。

- [核心与原生核验](evidence/core_native_summary.json)
- [源码身份](evidence/tested_source_hashes.json)
- [失败前缀及分版本源哈希](evidence/recorded_prefix_summary.json)
- [质量/几何缺口/ROS入口限制](evidence/quality_and_gaps.md)
- [完整 adapter 失败摘要](evidence/adapter_failure_summary.json)

Adapter 的原878文件保护检查也为false，14项既有peer差异保留，未重设基线。该完整失败
与2396/1核心测试范围不同，不能互相覆盖。正在独立区分实际接线缺陷和旧legacy夹具不兼容；
本档案不提前把21 failures/7 errors统称为无害夹具。任何修复须新候选、新attempt记录，
不得覆盖此失败。未合入或测试后续QP原型；其未跟踪文件不属于R12检查点。

旧 replay01–03失败、优化前性能诊断、夹具错误和真实 signed-zero 差异均保留本地原件。
010不复制大日志，不声称此前所有尝试成功或其完整证据均已公开。

## Risks and blockers

1. 四开发案例全部早期失败；八个 reused-holdout-family 回归未运行。一般曲率、偏置、
   遮挡和全圈效果未证明，不能靠删分母、造端点、未来投票或降低 overlap 解决。
2. 最新prefix仍超过80ms；projection/extraction在engine stopwatch之外，engine-only
   时间不是完整image-to-command延迟。
3. 普通reference/fixed ROS launch仍走CommonGroup GNSS/INS，未统一到R12；只有_gnss
   worker有本轮边界接线，不能用共享核心测试为三个runtime背书。
4. _gnss默认tilt仍legacy_flat，raw specific-force轴向/单位/时钟没有实车验证；
   TEACH拒绝的ROS控制政策不同于Simulator HOLD，需要独立迁移/验证。
5. 实车急停、电机零输出、命令所有权、完整两圈均未验收。
6. 完整adapter尚未通过，错误分类未完成；必须保留失败，再决定最小接线/夹具修复。
7. 本目录仅初步数据最小化，publication_review仍false；主planner必须逐文件审查。

## Next agent actions

1. 主planner审全文、拟提交代码和精确dirty diff，核对计数与源码身份；若发布，明确为
   失败开发检查点，不是可部署版本。
2. 先解决完整分母下的几何缺口与完整pipeline性能，再以新编号按预声明输入/门限试验。
   不重试挑最好时序，不拼接拒绝后的轨迹，保留本次所有失败。
3. 单独规划三ROS入口无GNSS建图迁移、原始IMU验证和失败关闭；保持reference的LYA安全
   回退、fixed的受管LYA关闭，不直接复制_gnss端点政策。
4. 审核后才更新publication flags、validate、pack、发布至唯一仓库；CI/发布回执另记
   精确SHA。本draft不提供上传或动车授权。

## Planner review and sealing decision

主 planner 已全文审阅SESSION、metadata、四个证据JSON、quality摘要及生成器文本。
只含相对源码/本地逻辑证据名和摘要；不含个人绝对路径、口令、精确GNSS、原始日志、
图像、bag或二进制。publication_review 两项现为true，status=handed_off，result仍partial。
生成器作为审计文本保留，不代表执行其中命令。原始失败完整保留本地。

Adapter失败已按源码进一步区分：内部调用缺少同stamp观测11项；缺core的double2项；
手动装旧matcher2项；原320PNG启动不足12项；peer OnGUI哈希变化1项。
单次320诊断表明178候选中仅74有可靠票而原启动要求100；填充地图116格不等于边界
足够。没有降门槛，后续测试迁移保留此负例。缺失boundary参考的运行模式不一致另有
提前拒绝修复在Simulator待测，未计入本次318测试结果。后续修复必须单独编号。

本次核心R12身份仍与2396/1测试及replay06一致；新的QP和局部快照原型没有接生产、
不属于本检查点。没有新actual跑圈，不把本次打包当完成目标。主planner继续修复。

