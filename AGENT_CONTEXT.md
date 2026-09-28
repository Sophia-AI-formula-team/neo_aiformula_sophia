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

### 当前 LYA 接线与参考速度契约（2026-09-28）

受管教师默认是 neo 实际的 `lya_0221`。教师与三个新包默认继承唯一来源
`trajectory_follower/lya_profile.py` 的 `REFERENCE_SPEED_MPS`（原 `v_t`）；本次源码值为
2.0 m/s，不是在新包各处复制 2.0。修改该源值为 4 时，新包默认也继承 4。
`reference_speed_mps` launch 默认空，表示不覆盖当前源码/配置；显式值统一覆盖相关节点。
`params_file` 可用全局 `/**.ros__parameters.reference_speed_mps` 指定本次参考，GNSS 的
supervisor 也接收同一配置。repeat `maximum_speed_mps=0` 表示使用有效参考。

源码默认在重新构建/启动后读取，运行中不热更新。修改参考后，旧 bundle 的速度 policy/参考
不匹配就拒绝，需要重新生成并验证路线；不能偷偷换 profile 降速继续跑旧图。

第一圈保留通过准入检查的真实 LYA 指令，不缩到旧的 0.8 m/s 或另加新包的 slew。
LYA 原反馈律不变，参考不是每帧实际输出：本次 2.0 参考、前方 1 m 误差时可能输出 2.15。
第二圈固定的是本次有效参考，不回放第一圈速度、不按曲率悄悄减速；误差反馈、启动限制
和停车检查仍保留。原名义横向检查 0.35 m/s² 不放宽，不可行路线拒绝 ready/运行。
这是 `v_ref² × |曲率|` 的参考可行性检查，不是反馈后实际 `v × omega` 的严格物理上限。
repeat 角速度限值仍为 0.4 rad/s；独立的教师角速度准入默认继承 LYA 当前 `MAX_YAW_RATE_RPS`
（本次为 2.0 rad/s），不能把教师范围扩大误写成 repeat 也已提高。

默认 `accepted_teacher_max_speed_mps=0` 只允许私有预览的未指定教师线速度上限；
任何非私有输出必须有三个部署确认和经过审核的正数上限。不能把电机参数 4.0 当实测硬上限：
当前 `STATE_BKUP` 路径绕过该截断。教师 FALLBACK 保留原命令，可能比固定候选快；它不是
“只取较慢者”的仲裁器。没有障碍物/硬件验证，不因参考同源而获得上车许可。

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

## 接线修复、测试状态与剩余现场边界

速度继承不等于传感器门限自动获批：`_gnss` 现存 raw-motion 速度准入是
`motion.DEFAULT_CONFIG.max_speed_mps=3.0`。LYA 参考改为 4 时，配置仍继承 4，
但节点在创建运行链前明确拒绝冲突，不暗降速，也不等车开到 3 以上才令整圈失效。
该门限不是实测车辆极速；需要独立审核/测量后才能改，不能为通过演示自动放宽。

2026-09-27 迁移档案中的缺 Header、错误默认教师和速度不一致是当时事实；封存记录不回写。
2026-09-28 当前改动修复以下确定问题，实际发布基线和 CI 回执仍以 STATUS 为准：

- 默认完整 mask 改为 neo `allnodes.launch.py` / `topic_list.yaml` 的
  `/aiformula_perception/road_detector/mask_image`。road detector 把完整输入 Header 独立复制给
  mask/标注图；严格 frame、尺寸、标定检查不取消，不能用 ROI 或猜测 frame 代替。
- 默认教师改为 `lya_0221`，保持其反馈律，补齐真实 `trajectory_follower` 运行依赖。
  它仍要求原 gyro odometry、filtered lane pose、filtered omega 输入。supervisor 只管理自己的
  子进程组，不能和另一个直发电机的 LYA 并行运行。
- `_gnss` 已补实际 resolved/remapped 输出话题的竞争 publisher / graph 异常检查，服务前和
  控制 tick 都检查，故障发零 HOLD，不自动恢复。这不是遥控仲裁，不能停止其他发布者。
- 当前 motor 源码已有零 RPM 停止旁路，不再声称零指令必然被补偿为非零；实际安装版本、
  电机端断流停车、遥控所有权和物理急停仍未现场验证。
- VectorNav 默认仍 0x210 缺 POSLLA；`_gnss` repeat 起点需经授权配置 0x230，仍禁止 GNSS 建图。
  CAN、原始 GPS 默认名已沿实际 launch 核对，不把 converted GNSS 话题当 raw GPS。

上游 Header 单测与本地算法测试不能代替 ROS。新增 `neo_upstream_smoke.py` 使用真实 supervisor
和已安装 LYA，验证合成输入下的反馈输出与真实进程退出；可追加生产 mask 发布方法 + cv_bridge/DDS。
**本次新增真实上游 DDS 结果待对应提交 CI**，不将脚本存在或历史 CI 当本次通过。
这些测试仍不覆盖模型推理、真实传感器驱动、整圈建图精度、实际遥控/电机/急停。
