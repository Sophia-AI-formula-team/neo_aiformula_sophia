# 2026-11-19 实验时间线

时间均为日本标准时间（UTC+9）。逐条索引收录每段对话的提问、答复和现场进度、当日部署时间日志、带时刻的报告条目、ROS 启动事件和录包起止。多处重复记录的事件保留其来源。原始消息时间和本机事后分析时间分别注明。[逐小时逐条浏览](records/timeline/) · [全部事件 JSONL](records/events.jsonl) · [命令及文件更改明细](records/operations.tsv) · [完整对话](conversations/) · [105 个启动会话](logs/launch-sessions.tsv)

| JST | 做了什么、遇到什么、结果如何 |
| --- | --- |
| 09:38–10:01 | 修改、回退后重新应用 Allnodes 的 YOLOP 默认项。安装 A3 V14 / A4 / A4Light 选择器、终端与桌面入口。A4Light 原先仍在 shadow 命名空间，补齐 live 模式和七个车辆话题。[接线原始记录](evidence/) |
| 10:02–10:11 | 停止原车测试进程，用同一台 ZED 运行相机基线及三档识别，每档两轮、每轮约十秒。三档均检测到物体。首轮 GPU 均值分别为 55.5%、59.4%、47.3%，补测为 44.8%、51.2%、51.0%；同时记录 CPU、内存、温度、I/O 与时序。[图文及数据](gpu-load-10s/) |
| 10:13–10:40 | 部署 Concept V6.27.5 与冻结的 V6.26 策略。在下载目录找到 B1.4 权重，完成 CUDA、回放和实拍识别检查，登记相机高 0.55 m，建立自动加载环境的终端和桌面入口。[部署时间线](concept-v6275/timeline.log) |
| 10:51–11:07 | 上午原车录包到 10:58:12。10:59:26–11:03:00 的第一份 Concept 专用包有 288 条决策且未发自动指令。随后 SSH 恢复，备份旧 Semantic 并在独立工作区装入 0.4.5。[原车录包](bag/other/) · [首份 Concept 诊断](concept-v6275/2026-11-19-concept-no-auto-output.md) |
| 11:07–11:40 | 核对 A3、`/flag_stop`、旧 `stop_wait` 与新版状态的来源。11:24 初版非红恢复仍将 unknown 维持等待，11:25 按现场要求改为新鲜 unknown 可参与非红恢复投票。11:28 后装车；11:30 关闭 Concept，11:35 隔离验证 599/600 像素、匹配后可行绕行优先和 OA 切换。11:40 发布第一批分类结果。[Semantic 逐步记录](semantic-045/timeline.log) |
| 12:41–13:42 | 取回上午失败包。定位并处理 Jetson 解码卷积及编码延迟。13:22 登记实车 ZED 推理中位 600 → 60 ms；13:42 SSH 恢复后取回 192 条使用模拟静止反馈的规划决策，影子联测未见图像超时。[剖析、原值和限制](records/workstation/concept-latency-20261119-1310/) |
| 13:49–14:42 | 13:49 录包中 468 条决策受几何有效性限制，手柄心跳也缺失。审查灰色候选、顶层路径与地图 UI，现场风大、未取得同帧量测点。应用实时 CameraInfo、本田安装姿态和 0.55 m 高度；14:37 确认自动指令输出，14:41 核对 CAN 左右轮速、VectorNav 和原车 odom 的不同用途。[标定核对](records/workstation/concept-followup-20261119/calibration/) · [运行证据](records/workstation/concept-followup-20261119/logs/) |
| 14:46–15:19 | 14:46 场已有 164 条自动指令。分解约 281–282 ms 规划前延迟，比较 15/30/60 Hz、同步直传和异步，区分短时清零与真正断流。SSH 多次中断；15:18 回装 `0dc4e9e` 并核对。Semantic 快捷入口更名 6.0、5.5 m，对应绕行最小转弯半径。[延迟实验各轮](records/workstation/concept-followup-20261119/auto-timeout/) · [启动入口](records/workstation/concept-followup-20261119/semantic-shortcut/) |
| 15:21–15:42 | 15:21、15:22、15:26 三场自动指令分别为 53、0、119。对照持续锁停、恢复按钮和旧帧看门狗。15:33 取回红灯未停现场记录并停止当次进程，15:35–15:36 完成 600 像素至旧 motor 的静态联测。15:40 修复门控时序；真实传感器、软件 CAN 接收器两轮各 22 秒完成验证。[Concept 三场](records/workstation/concept-followup-20261119/concept-postdeploy/) · [停车现场](records/workstation/concept-followup-20261119/semantic-red-1533/) |
| 15:42–16:08 | 修复面板继续/重启和双速度默认值。15:50 包的自动目标为 1.0 m/s，13 条倒车请求停在 shadow；15:51 的下一份包为空。15:55 归档实际 MapAssist 启动源码。随后修正低速倒车轮速解码、原电机换算与面板持久化。[16:03 修复与速度曲线](records/workstation/concept-followup-20261119/concept-drive-fix/) |
| 16:08–16:33 | 16:10 的 MapAssist 仅记录开始学习，旧电机订阅原手柄速度；裸 OA 的标志和 ABC 未被 MapAssist 接到。16:22 SSH 恢复并取回终端可见区。隔离复现并验证 OA 的四项 remap，安装独立 OA、LYA 和整组 MapAssist 桌面入口。[首圈证据](records/workstation/concept-followup-20261119/semantic-lap-1620/) · [正式组合入口](records/workstation/concept-followup-20261119/semantic-shortcut/mapassist-formal/) |
| 16:34–16:49 | 16:35 Concept 场捕获突加速、弯中轮速误判和障碍投影偏差。另一段对话同时接入 `lane_0927oa`，16:44 隔离验证车道、避障与切回。16:47 两次 MapAssist 快捷启动只留初始化日志，与 Concept 占用硬件的时序一致。16:48 记录其退出。[Concept 曲线](records/workstation/concept-followup-20261119/concept-1635/) · [接线和占用核对](records/workstation/concept-followup-20261119/semantic-shortcut/) |
| 16:50–17:02 | 正式 MapAssist 开始写入旧状态及电机会话。三份包累计 780,265 条消息。16:57:34–16:58:22 收录到 5 条 `red_wait_first_lap`、15 条旧 motor 对应门控记录。同期 Concept 修复 `5314396` 在隔离回放中验证了 0.20 → 0.40 → 0.50 m/s 的起步请求。[Semantic 三场](bag/semantic/) · [回放记录](records/workstation/concept-followup-20261119/concept-1635/trace/) |
| 17:03–17:09 | 一份辅助包收录底盘与传感器。随后三份 Concept 专用包分别发出 371、504、106 条自动指令。最后一包仍有 54 条 `invalid_vehicle_state`。录包的首末消息、逐话题数量及实际门控状态可逐场复核。[底盘辅助包](bag/other/) · [Concept 录包](bag/concept/) |
| 17:13–17:34 | 车辆电量将尽，优先保存记录和图片，SSH 中断后续传至 Extreme SSD。核对 7,222 个车端文件和 17 个数据库。另将最后生效的软件包、节点和启动代码归档，对比 Neo 四个基线；OrinSSD、工作站和 Extreme SSD 三份活动代码压缩包哈希一致。[文件核验](records/collection/) · [软件包 diff](active-code/) |
| 19:47 起 | 按当天两段对话补全公开归档。逐包核对 SQL 消息、分类、图片和清单，建立 GitHub 当日目录与 Release 下载索引。[归档操作](records/publication.tsv) · [远端核验回执](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/experiment/yolop-selection-20261119/docs/agent-context/receipts/2026-11-19-experiment-field-archive.md) |

具体验收结果和仍缺乏的证据可按现场问题查阅。[问题与修复](ISSUES.md)
