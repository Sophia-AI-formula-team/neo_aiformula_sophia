# Agent context — 一圈建图，再跑定轨迹

这是现场 agent 的入口，不依赖旧聊天记录。**先读本文件，再读
[当前状态/任务](docs/agent-context/STATUS.md)、[交接规则](docs/agent-context/PROTOCOL.md)
和[实验操作与记录要求](docs/agent-context/EXPERIMENT_RUNBOOK.md)。**

## 谁负责什么

- **规划 agent（本项目原对话）**：planning、主要实现、算法/状态机/安全契约、审阅现场证据、整合修改、发布下一版任务单。
- **实验日 agent**：核对车端真实部署，做指定范围内的适配、调试与测量，保存每次尝试（包括失败）的参数、日志、代码版本、结果，打包上传并回传问题。
- **人在现场**：决定是否通电/动车，实测手操急停和电机零输出，批准实验范围。agent 之间的交接不能代替这些决定。

唯一发布仓库：`Sophia-AI-formula-team/neo_aiformula_sophia`，不可变仓库 ID **1303517209**；整合分支：
`feat/causal-lane-teach-repeat`。实验改动放 `experiment/<日期>-<短名称>`，不要直接写 `main`。
当前功能代码基线和待办以 STATUS 中的任务单为准，不把“最新提交”自动当成已验证版本。
2026-09-27 已按用户要求纠正发布目标。旧仓库 `aiformula_sophia`（ID 1001834365）不是同一仓库，
禁止继续向它 push。封存的旧任务/记录仍保留原始名字和 URL，只是历史证据，不能作为当前部署指令。

## 要实现的事，及不能偷换的含义

车辆第一圈由 LYA 开车，同时用**完整 mask + 运动观测**因果建图；不是把 LYA
命令积分成地图。建图完成后用固定路线行驶，mask 用于定位和持续降噪。
已有足够历史图后，每帧先对旧图匹配，匹配可信后才融合；初始无图时仅在代码规定的
帧数/时间/距离界限内用 raw motion 建种子图。不能让同一帧刚加入地图的点证明自己定位正确。
冻结的驾驶路线与在线降噪层分离，不能无审批偷偷移动路线。

### 用户最后明确的 GNSS 约束（优先于旧文档/聊天）

**GNSS 是用来判断定轨迹起点的，不许用 GNSS 建图。**

- 只在 LYA 开始、结束的两个停稳窗口各取一条定位；不是整个 LYA 阶段都可用。
- `_gnss` 的建图、原点、地图 bundle、路线和运行中的定位均不吃 GNSS。
  GNSS 缺失/不合格仍能建图，只是不允许 `_gnss` 进入 repeat。
- GNSS 只粗筛是否在起点附近；不是全局重定位，不提供初始 ENU 航向。
  精定位/方向还要通过 mask 和连续运动链验证。
- `_gnss` 运行中使用 CAN 实测轮速 + 安装旋转后的原始 gyro + mask，不能用
  VectorNav INS 位置、融合速度、融合姿态“暗中补回来”。
- 地图坐标 `lane_teach_local` 的 x 是 begin_teach 时前方、y 向左、yaw 逆时针；不是 ENU。
  当前版本只支持**同一连续进程** teach → repeat，不能重启后自动加载旧图驾驶。

因果配对：wheel 只配已到达且源时间不晚于该 wheel 的 gyro；mask 只配在该图到达前已收到、
且源时间不晚于该 mask 的运动数据。不是只检查“比现在早”就够了。不用未来插值、整段 bag
后验平滑冒充 runtime。输入是 road detector 的完整 mask，不能换成 lane publisher 的 ROI。
3 m 硬截断已被否定；投影按不确定度/敏感性筛选。展示固定尺度、当时算出的 consensus，
不要把原始累计噪点说成已确认地图。

## 三个包的差别

都在 `workspace/src/aiformula/control/`，与新仓库的 `trajectory_follower` 同级。`_gnss` 和 fixed 依赖参考包的共享实现，
不能只复制一个目录上车。

| 包 | 运动来源 | 定轨迹阶段 |
| --- | --- | --- |
| [lane_mapping_lya_reference](workspace/src/aiformula/control/lane_mapping_lya_reference/README.md) | 原有 VectorNav CommonGroup + mask | LYA 命令作为受限安全参考/回退；视觉失效仍 HOLD |
| [lane_mapping_fixed](workspace/src/aiformula/control/lane_mapping_fixed/README.md) | 同参考包 | 停止自己管理的 LYA，无 LYA 回退 |
| [lane_mapping_fixed_gnss](workspace/src/aiformula/control/lane_mapping_fixed_gnss/README.md) | CAN 轮速 + raw gyro + mask | GNSS 仅起点许可；停止受管 LYA 后运行 |

旧两个包的 CommonGroup 使用与 `_gnss` 严格无 GNSS 运行不是同一条链，不能混用验收结论。

## 代码入口

- ROS 接口/服务/窗口：[GNSS node.py](workspace/src/aiformula/control/lane_mapping_fixed_gnss/lane_mapping_fixed_gnss/node.py)。
- 运动与独立起点许可：[motion.py](workspace/src/aiformula/control/lane_mapping_fixed_gnss/lane_mapping_fixed_gnss/motion.py)、[anchors.py](workspace/src/aiformula/control/lane_mapping_fixed_gnss/lane_mapping_fixed_gnss/anchors.py)。
- 建图/异步提交/保存：[mapping.py](workspace/src/aiformula/control/lane_mapping_fixed_gnss/lane_mapping_fixed_gnss/mapping.py)、[worker.py](workspace/src/aiformula/control/lane_mapping_fixed_gnss/lane_mapping_fixed_gnss/worker.py)、[bundle.py](workspace/src/aiformula/control/lane_mapping_fixed_gnss/lane_mapping_fixed_gnss/bundle.py)。
- 共享投影与 consensus：[mapping_core.py](workspace/src/aiformula/control/lane_mapping_lya_reference/lane_mapping_lya_reference/mapping_core.py)。
- mask 定位/路线/控制：[mask_localization.py](workspace/src/aiformula/control/lane_mapping_lya_reference/lane_mapping_lya_reference/mask_localization.py)、[route.py](workspace/src/aiformula/control/lane_mapping_lya_reference/lane_mapping_lya_reference/route.py)、[controller.py](workspace/src/aiformula/control/lane_mapping_lya_reference/lane_mapping_lya_reference/controller.py)。
- LYA 进程归属：[supervisor.py](workspace/src/aiformula/control/lane_mapping_lya_reference/lane_mapping_lya_reference/supervisor.py)；只能停止自己启动的进程组。

## 当前证据和缺口

迁移前功能来源 `7662f5a5174b7579afadd23291c8e023288ecdfc` 的
[封存验证摘要](docs/agent-context/sessions/2026-09-27-planner-001/evidence/functional_baseline.json)
记录原生消息/三包构建、520 个 Linux 单测，以及 `_gnss` 9 个静止合成 DDS 场景
（151 项断言）。这**不是**实车整圈/第二圈、RViz GUI、急停或电机测试。
旧两包的 DDS 闭环没有在那次 GNSS workflow 中重跑。迁移后的实际基线、新仓库 CI 和清理回执
均从 [STATUS](docs/agent-context/STATUS.md) 进入；不能把旧仓库的 CI URL 改写后冒充新结果。

现有 January 真 bag 在新链路上仍不合格：raw GPS 的 `0x210` 缺 POSLLA；
轮速有 119 个间隔超过 200 ms、最大 2.894 s。只读回放很早就因断流停止，不能
改时间戳、松看门狗或循环 reset 来拼成一圈。GPS 缺失阻止起点许可，**不是建图失败的原因**。
旧版真实 mask 定位也未形成可信连续第二圈：详见
[既有 VALIDATION](workspace/src/aiformula/control/lane_mapping_lya_reference/VALIDATION.md)，不把其合成指标当实车精度。

所以下一步先核对车端时序、原始运动和标定，拿到合格连续输入，再验证地图和跨圈定位。
当前状态：**尚未实车放行**。详细行动/验收/回传字段在当前任务单中。

## 新仓库迁移时发现的明确实车阻塞

以下来自 neo 基线 `8914b613c7200709f7c8617aed2d408afd911a0d` 的源码审计，不能被合成 CI 替代：

- [road_detector.py](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/8914b613c7200709f7c8617aed2d408afd911a0d/workspace/src/aiformula/perception/road_detector/road_detector/road_detector.py#L114)
  只把输入 stamp 传给输出 mask，丢失 frame_id；当前严格 frame 检查会拒绝。正确方向是保持原始完整
  Header 和真实标定，不能伪造 frame 或取消检查。本次迁移没有改 road_detector。
- neo 根 README 的 `lya_0221` 与三个新包默认管理的 `lya_follower_connected_omegat_global` 不是同一程序。
  两个 entry 都存在，但不能同时运行并假定 supervisor 能关掉另一个；它只能管理自己创建的进程组。
- neo 同名 LYA 的线速度限幅为0.8–4.0m/s，角速度上限0.45rad/s；新包 teacher 准入仍保守地限制到
  2.25m/s和0.4rad/s，超出会拒绝。迁移没有放宽限制或提高固定路线速度。
- VectorNav 的11个消息定义与旧来源一致；驱动默认依然0x210缺POSLLA。repeat起点仍需正确0x230配置，
  但不能用它参与建图。具体部署步骤以包 README 为准。
