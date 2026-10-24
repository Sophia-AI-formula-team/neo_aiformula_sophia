# Semantic 信号灯全链路运行报告

2026-10-12；semantic 提交 `63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96`。
四场 `final63_*` 使用同一冻结版本，真实 Unity Play 记录均为 `accepted=true`。
四场 result.json、独立逐样本审计、恢复后的源码审计及最终媒体 QA 已复核通过；下述采样限制保留。

代码已推送并建立 [draft PR #16](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/pull/16)，
基于 [PR #15](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/pull/15) 分支
`feat/semantic-planning-sync-20261011`。读回状态为 OPEN DRAFT、10 个文件、5 个提交；未 merge 或打 tag。

## 四场结果

| 试验目录 | 实际红灯停车车头间距 | 行驶距离 | 记录时长（含停止尾段） | 红灯越线（harness） | 指定区段 |
| --- | ---: | ---: | ---: | --- | --- |
| final63_inner_red | 8.635939 m | 73.622140 m | 39.355972 s | 无 | 完整通过 |
| final63_inner_green | 不适用 | 73.657520 m | 32.628222 s | 无 | 完整通过 |
| final63_outer_red | 8.602903 m | 79.296070 m | 39.522116 s | 无 | 完整通过 |
| final63_outer_green | 不适用 | 79.186140 m | 34.100755 s | 无 | 完整通过 |

红灯两场均在用户选择的 8–9 m 目标带内停稳，随后由试验灯转绿并恢复前进。
内圈、外圈转绿时刻分别为 15.769111 s、14.203250 s。初始绿灯两场均穿过信号灯并完成区段。
完成条件为穿过灯位后，走完东侧直道、东北弯道和北侧直道，到达 `x <= 38 m, z >= 35 m`。
它覆盖指定红框区段；没有宣称跑完整赛道、完成首圈建图或验证自动绕行。

四场 result.json 均记录 `cleanupErrors=[]`、detectorWorker=0、semanticWorker=0；结束速度最大
`4.11634176e-7 m/s`。主代理另完成现场恢复读取，见下方源码与恢复证据。

## 独立采样证据

正式来源：`staging/final_run_evidence.json` 和 `staging/final_run_metrics.json`。

| 红灯场景 | 全红阶段采样最小车头间距 | 正式 wait 时长 / 样本数 | 放行消费的新绿帧序号 | 红灯车头越线采样 |
| --- | ---: | ---: | ---: | ---: |
| 内圈 | 8.635939 m | 3.023326 s / 44 | 100 | 0 |
| 外圈 | 8.602903 m | 3.004284 s / 39 | 92 | 0 |

两场正式等待期间均满足真实红灯、semantic wait、实际速度绝对值小于 0.05 m/s，当前绿观测新鲜。
四场终点与记录清理检查通过，单侧来源/报告的期限违规均为 0。当前 RIGHT/LEFT 推算帧数依次为：
内红 40/36、内绿 36/30、外红 14/0、外绿 16/0。内红/内绿 LEFT 在施加命令样本中的最大报告先验
年龄分别为 1.417519 s、1.423942 s，未超过 1.5 s；C# 的绝对 request deadline 未逐条记录，另由合同测试验证。

两场红灯审计保留唯一 `first_motion_rearm_edge_not_sampled` 标记。首次恢复运动采样为内圈
16.55735 s（tick 328 / vision 102）、外圈 14.93336 s（tick 296 / vision 94）；这些样本已有变化后的
ABC、与转绿后真实 supported candidate 精确对应的来源及 point age=0。但是触发放行的前一个 tick
没有被录像/运动采样捕获，显式 new ABC/new command 标志直到内圈 16.67489 s、外圈 15.15418 s
才被采到。不能把后来的标志当作首次运动之前完整事件边沿的证明；这也不是旧点重放或不安全放行的证据。
新 ABC/新命令门槛有源码及回归验证，这里将运行采样缺口如实保留。

另一个轨迹度量 `final_metrics.json` 使用与 source_before 哈希一致的 CAD/场景几何评估 2022 个实际
车体中心样本；四场中心越出所在车道、中心越过标线内缘样本均为 0。最小左/右边界中心线间距依次为
1.8271/1.8113、1.8122/1.7625、1.6300/1.7512、1.6248/1.7506 m。这是离散车体中心与保存 CAD 轮廓
的比较，不是轮胎/整车包络或连续时间碰撞证明，也不替代实际录像视觉检查。

## 距离口径与信号输入

控制侧 `distance_m` 来自当帧真实 RGB 灯面、原相机 K/R/t 与已知灯面高度关系，表示采集时目标相对
车头参考点在当时车体前向轴上的投影深度。它不是随赛道固定的信号平面间距；车辆转向时两者会不同。
每帧半像素几何不确定度与实际帧龄位移用于保守下界；这不覆盖所有姿态、目标高度或标定误差。

本次停车验收另使用固定 `z=15 m` 信号平面，到车体实际变换后的最前橡胶前角点的纵向间距。
模型测得前角点为局部横向 ±0.3275 m、前向 +0.216 m。这个评估真值以及已知灯的世界位置不输入控制。
原先希望更近停车，但固定相机在内圈接近灯位时会失去灯面；用户最终选择保持相机不变、验收 8–9 m。
本结果不是 2–3 m 精准停车承诺。

## 实际运行链与改动边界

真实 Unity RGB → 原 A3 模型与明确标注来源的 RGB 面板补充识别 → Python LLP 接线 → 原上游
LaneLineSubscriber 选点及原 LYA → MapAssist 速度门控 → Unity 车辆。semantic 通过 stdio worker
以约 20 Hz 真实时钟 tick 运行原类，ROS 消息和发布订阅由传输 shim 连接；没有启动 native ROS/DDS。
感知请求为 10 Hz；录像目标采集 15 Hz、运动目标采样 20 Hz，记录与视频按实际时间戳使用，不假定
请求频率就是每秒成功返回数量。Unity Rigidbody 提供理想、无噪声模拟 odom。

本次 semantic opt-in 的 Simulator 覆盖值为目标下界 8.0 m、接近速度上限 3.0 m/s、保守减速度
1.5 m/s²。包默认关闭该功能，默认值仍为目标下界 8.5 m、上限 0.8 m/s、减速度 1.0 m/s²。
LYA 的 v/w 同比限幅保持命令比值，不保证实际车辆在不同速度下具有相同轨迹曲率。

LLP 接线 opt-in 使用当前掩码的有效投影组件与当前双边共同中心；合格短中心仍受原生至少 3 点、
1 m 观测跨度及新增双边质量检查约束。默认路径保持，原 C++ LLP、原 LaneLineSubscriber 和 LYA
源码/增益未改。为解决服务口组件遮蔽主线，显式接线允许原地面域内当前像素组件的面积归一与备选，
没有补绘车道、外推轨迹点或把历史点重新标成新视觉。

没有当前可靠双边时，当前右线可使用最近三帧真实双边形成的标量宽度；当前左线须通过有限期几何
相似性检查后才调用原 native LEFT 的 2.6 m 参数及原不等偏移。这些单侧目标明确标记为推算来源，
不能称为实测共同中心。校准期限 1.5 s 从三帧一致窗口完成时起算，窗口自身跨度也不超过 1.5 s；
不表示每个贡献样本在消费时均小于 1.5 s。单侧观测不能给先验续期。worker 双时钟与 C# 绑定来源
序号的绝对 deadline 再检查消费期限；原图像、ABC 和命令 TTL 保留。

LEFT 几何身份筛选比较各自图像时刻的 body 坐标，未作 ego 对齐；比较的是两次实际 LEFT 观测域
的交集，不代表参考全域都有双边支撑，因此不保证物理上始终同一边界。本次通过限于此场景四场实跑。

自动 MCP 试验临时设置不序列化的 `RequireApplicationFocus=false`，避免焦点切换终止后台试验。
默认仍为 true；试验保存并在 finally 恢复原值。急停、UI 阻断、手动输入、所有权、新鲜度和丢线保护
保持，原 driver 失焦后的 neutral 要求也保留。该变化仅在自动 `signal_approach` 试验中显式启用。

## 红灯、左转和修复记录

包内另有 traffic gate + costmap/primitive planner：红灯禁止直行，碰撞可行候选中偏好 LEFT，
不是“见红必左转”，也可能右转或停车。本次运行的是 MapAssist。它原有绕行依赖完整建图、拓扑匹配
与附近可用入口；本次冷启动短区段不满足条件，因此测试红灯停车、绿灯直行，没有新增强制左转。

原版只有颜色、没有距离，红票确认后立刻停车；旧实跑提前约 21–23 m 停车。问题报告在修改之前保存，
原始失败及后续失败视频/数据保留。距离接近默认关闭；开启时仍要求真实灯色、有效距离和 odom。
旧红票过期造成 unknown 平票不单独撤销已确认红灯意图；当前真实 unknown 立即零速暂停。
连续新红帧需重新满足完整 0.5 s 窗口且至少 3 帧，距离/odom/制动余量有效，再经原延迟、全新 ABC
与新 LYA 命令才允许接近。结构故障、过期或到达目标后的等待不能靠更远红框重新开启；故障后绿帧
还必须越过 fault_sequence 屏障。

最后发现首入口缺口：首次完整票窗可能是 3 红/2 unknown，而当前帧已 unknown；旧实现直接永久 wait，
下一帧真实红也不能恢复。`63eec2d` 只对 enabled、normal、已确认红意图的入口按同样分类处理：
新鲜 unknown 进入零速 suspended 并清独立恢复窗，结构故障仍锁存，目标等待不重开。
详见 `CRITICAL_STOP_DISTANCE_REPORT.md` 与 `staging/corridor_v2_initial_red_vote_diagnosis.txt`。

## 版本、测试及交付

相对 PR #15 的基准 `5bc52e0`，semantic 仓库仅修改 10 文件；`semantic_planner_core` 和
`semantic_local_planner_ros` 的 package.xml/setup.py 均为 0.2.0，其余包版本保持。功能提交及修复
作为独立后续提交保留，没有覆盖原始源快照。core PR 不包含 Simulator 接线或媒体；后者通过独立
来源审计和 source delta bundle 交付，peer 的 Tools/LaneTeachRepeat 文件排除。

- 正式完整包 pytest：212 tests、203 subtests 通过；67 tracked Python 文件通过 Python 3.8 AST；
  `git diff --check` 通过。证据在 `staging/first_confirmation_fix/`，不是此前 unittest 小计。
- 当前 `63eec2d` 核心 + installed v2 worker 的独立真实时钟 stdio/transport 10 项一次通过
  （13.778 s、exit 0），104 tracked core 与 installed runtime 共 184 个哈希前后不变。
  正式结果 `staging/final_transport63_receipt.json`，复现 runner 为 `staging/run_final_transport63.py`。
  v2 corridor transport 30 项、focus C# stub 63 项也通过，分别计数，
  不加入 212 项包测试；对应报告明确 ROS/Unity stubs 的范围。
- 当前 corridor v2 保存掩码回归与默认逐值等价证据在 `staging/current_corridor_v2/`；离线回放
  不当作新的闭环轨迹，四场表格只使用 `final63_*`。

最终源码审计 `staging/final_source_audit.json` 为 PASS：干净 HEAD 为 `63eec2d`，104 个 vendored
semantic 文件逐字等于该 Windows checkout；937 项原始清单核对后，受保护场景/相机/物理/权重/
上游/持久设置变更为 0，peer 10 项单独归属排除，无未解释错误。Git blob 换行差异独立记录。
四场各自的 85 项 integration_hashes 文件完全相同，其 SHA256 为
`6f2beb338264a70bb41a9e2b8228107b9a4e59c2efe2fd45db9cc1d64e6b35e9`，与当前安装源码一致。

`staging/editor_restoration_verification.json` 记录 editor_final_restored 与接管时 editor_retake
除 UTC 字段外逐项一致。主代理 MCP 焦点恢复读取返回空组件列表：交接场景中没有 Follower，因此没有
遗留实例的 runtime focus override；不能把这一结果写成现场某个 Follower 属性已读得 true。
默认 true 和 cleanup 保存/恢复路径由 63 项 C# 断言验证。主代理已授权在源码审计与媒体 QA 通过后生成
仅写诊断目录的 source delta bundle，并执行只读 verify-after；包的最终路径、SHA 和校验结果由其独立 receipt 记录。

四项媒体/数据位于本报告同一目录：`final_semantic_four_cases.mp4`、`final_trajectories.png`、
`final_trajectory_samples.csv`（2022 个运动样本）、`final_metrics.json`。
视频为 1280×960、H.264/yuv420p、15 fps、2191 帧、146.066667 s、57,116,592 字节；
SHA256 `1f183b2c70adb2b1e0257310b959a714b8653b60e2b8d105f07084fba2ae62fe`。
`final_semantic_four_cases.decode.json` 记录全片 2191 帧解码完成且 stderr 为空。
主视图占上方 75%，下方为原识别 overlay、LLP/LYA 与 semantic 诊断；保留实际静止等待和停止尾段，
按实际 wall 时间以 1 倍速重采样，不裁掉失败/保护片段来制造连续运动。

视频原始来源与时序在 `final_semantic_four_cases.verification.json`，最终媒体清单为
`final_media_manifest.json`，正式可解析 QA 为 `staging/video/qa/final_four/final_media_qa.json`。
全片解码、四场完整时间轴 contact、实际解码红停 2 帧、通过灯位 4 帧、terminal 4 帧、LEFT 2 帧及
全尺寸轨迹图均已检查通过。CSV 共 2022 行、47 列，逐字段对照原 motion 通过；轨迹图终点仅显示
评估停止标记，未将尾段 E-STOP 错标为另一次信号灯停车。主画面没有被诊断大幅遮挡。
媒体 QA JSON SHA256 为 `faf6172a773e807cdd95c448041c2486adc88ae76a89151ebbbe6df93860bc92`。

这些证据验证 Unity 中单个相关信号、固定相机与本区段的全链路。未验证多灯关联、native ROS/DDS
调度、完整建图绕行、真实车辆制动或硬件安全；采样终点和无红灯越线也不等同于连续碰撞/车道边界证明。
