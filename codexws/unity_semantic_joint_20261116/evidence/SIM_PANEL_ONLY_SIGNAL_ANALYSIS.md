# 仿真已知发光灯牌独立信号模式：实施前分析

2026-11-16。outer02采用v2身份接线后仍在第一圈同区域由模型red锁停，未完成建图。继续叠加cone类别/置信度特例会扩大模型后处理规则且不能证明交通灯身份。本报告先评估一套明确独立的仿真信号接线：A4检测与overlay完整保留供观察，但不作为traffic-light state输入；只接受现有校准RGB panel helper的当前证据。

候选方法是用原helper调用 `model_state='unknown'`，不启用model identity guard，保留当前detections（仅辅助已有RGB候选关联/绘图）。只有当前像素满足现有颜色/暗边框/背景/相机与已知LED几何条件，且通过原两帧关联确认后，才输出red/green。再进入原semantic 0.5s/min3/严格多数的TrafficLightFilter，unknown仍不能释放已确认red。**这不是手工topo校准；不提供Unity位置/面板state/距离给控制；bbox距离模式仍关闭。** 它明确适用于当前仿真已知安装的发光灯牌，不代表通用实车交通灯检测。

回放先使用已完成inner04/05/06、outer01/02的真实保存RGB和原detections，模型不重跑、不改source/live，单线程总≤60s。检查实际red/green输入与原多数票、模型优先曾掩盖的RGB证据、计划停车处转绿前后仍静止时的可见证据，以及外圈原cone误检区域是否会形成red票。

限制事前写明：保存RGB为JPEG而非原始推理PNG，颜色/边框可能改变；以C#接收记录wall时间重建到达间隔，并按原capture age≤0.5s过滤，只能评估原投票的可形成条件，不能逐纳秒复原Python→Unity→semantic投递。真实面板切色event仅用于评价与按捕获时刻分组，绝不参与候选状态。如果停车处没有足够真实green证据形成原多数票，不建议部署该模式来声称红停绿行已解决。

## 已完成的真实帧回放

`replay_panel_only_signal.py` 在 Unity Edit 窗口内完成，单进程、OpenCV/OpenBLAS/MKL/OMP 线程数均为 1，总计 17.18 秒，3,268 张已保存真实 RGB；没有运行模型、修改 live 或 semantic 源码。使用 pre-v1 原 helper 和原 `TrafficLightFilter`，没有调用 v1/v2 identity filter。完整结果为 `panel_only_signal_replay_summary.json` 与各场 `*_panel_only_replay.json`。计数仅涵盖 `trial_stop` 之前，不包含收尾帧。

| 场次 | 真实 RGB 帧 | panel-only red / green | 可形成 native red / green 票 | 首次真红票 | 停车后原地转绿证据 |
| --- | ---: | ---: | ---: | --- | --- |
| inner04 | 835 | 6 / 73 | 4 / 35 | seq508，t262.170，3/3 red | 保存 JPEG 中没有静止时 green 票；seq597 才为 2/3 green，但当时已在移动 |
| inner05 | 959 | 59 / 53 | 55 / 51 | seq443，t100.459，2/3 red | seq504–506 为当前确认 green，seq505/506 在 actual_v≈0 时为 green 多数 |
| inner06 | 827 | 58 / 58 | 55 / 56 | seq536，t120.491，2/3 red | seq593–595 为当前确认 green，seq594/595 在 actual_v≈0 时为 green 多数 |
| outer01 | 322 | 0 / 35 | 0 / 31 | 全程实体绿，无假红票 | 未到第二圈，不存在切红/切绿停车验证 |
| outer02 | 325 | 0 / 33 | 0 / 31 | 全程实体绿，无假红票 | 未到第二圈，不存在切红/切绿停车验证 |

native 票列包括原 0.5 秒滑动窗的切色暂留。例如 inner05 seq503 已捕获实体绿，但该帧 panel-only 是 unknown，窗口仍为 2 个先前 red + 1 unknown，因此保留 red 一帧；这不是新的假红观测。inner06 还有 1 帧 capture 时间刚跨实体切色 event 的 green 输入，未形成 red 阶段的 green 多数。报告不把这些异步边界算作道路上无灯的模型误检。

inner05 实体转绿 t113.207，seq504/505/506 的当前 green 捕获时刻分别为 t113.606/113.834/114.034；seq505 的窗口为 green=2、unknown=1，actual_v=-2.95e-8。inner06 实体转绿 t132.376，seq593/594/595 当前 green 捕获时刻 t132.899/133.147/133.349；seq594 同样 green=2、unknown=1，actual_v=-2.79e-8。这两场证明真实停车位仍有持续绿灯图像证据，恢复不必依赖车先动。**inner04 不满足这项证明，不能用后续移动帧替代其静止恢复验证。**

上述是沿已记录轨迹的反事实信号回放。尤其 inner06 panel-only 真红票比原模型票晚，换模式后实际停车位置可能变化；这不是新模式的闭环验收，也不能保证外圈第二圈真实红灯可见。若实施，必须另行 MCP 实跑。

## 模型优先确实覆盖了现成 RGB 证据

原 helper 中先判断 `model_state == 'red'`，再判断已确认 RGB red，再模型 green，最后 RGB green。原记录（推理 PNG 对应元数据，不是重新估计）中，source 为 `a3_model` 且同时已有已确认 RGB 候选的帧数：inner04=20、inner05=67、inner06=71、outer01=15、outer02=15；大多数只是同色证据被模型 source 标签覆盖。

真正相反色覆盖的例子：outer01 seq4 已有 green panel `[357,177,360,180]`、4 帧确认；无关 model red `[157.117,179.073,161.050,185.202]` 仍使最终 signal=red。outer02 seq19 已有 green panel `[395,175,400,180]`、12 帧确认，同时也有该位置 model green；另一处 model red `[175.118,179.066,179.149,185.257]` 把最终颜色设为 red。inner04 也有 1 帧 model red 覆盖已确认 green。详见 `panel_only_priority_and_false_red.json`，没有据此重命名或删除原检测。

## outer02 原 v2 失败的确定范围

outer02 原输出 red 的序号为 19、111、155、164、165；第一圈后段连续的 seq164/165 位于无有效 RGB panel 的区域，仍进入 native 红票与 `red_wait_first_lap`。它们的 model class10 与 cone 完全同框，但 light 分数分别 .5524/.5408，高于 cone .5246/.5208，不满足 v1 的 higher-score cone 条件。原 PNG 上 v2 的同色像素半像素上界 ray_up=+0.0025537，不满足“所有同色像素都在 horizon 下”这一必要条件，所以 v2 正确地保留了不确定模型色。这是该保守规则不能提供完整身份的限制，不应靠继续放宽置信度/IoU/horizon 阈值修成能跑过。

panel-only 回放对 outer01/02 均没有 red 输出，也没有 red native 多数；当前 RGB 真实 green 仍保留。两次失败记录完整保留，不把回放说成实际完成两圈。

## 最小建议及实施边界

现有真实 inner05/06 的静止红停→绿证据足以支持**作为独立、明确标记的仿真已知面板接线候选进行实跑**。建议替换试验中的 v2 model identity guard，而不是把新模式叠在更多模型补丁后面：只有 `signal_panel_only=true` 才忽略模型颜色；默认关闭结果/schema 与 pre-v1 helper 一致。原模型 detections、raw model_state 和 overlay 继续保留。

必须拒绝 `signal_panel_only=true` 且 RGB pipeline 被禁用的配置；缺标定时只允许 unknown，不能走原 offline API 的 `calibration=None` 未标定兼容通道。当前同色候选仍需原几何、光度、两帧确认，重复/乱序帧不能累计确认，原 unknown/red latch 和 native TTL/投票保持。无当前 panel 不回退模型、不回放旧色、不读取实体颜色或世界位置。原 `signal_range` 可仅作诊断，仍不传给 semantic。

这项接线不会修复已经证明的官方 topo 融合歧义，也不会修复 inner06 四点侧线导致的 LYA 输入不足。后续必须分别记录红停绿行、odom 闭合、official fusion、实际内部左转，不能用感知假红减少替代 topo 成功。当前状态：完成只读评估，尚未实施/部署新模式。
