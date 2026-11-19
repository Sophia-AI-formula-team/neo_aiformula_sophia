# 2026-11-19 MapAssist 正式接线桌面入口

| JST | 操作与证据 |
| --- | --- |
| 16:31:12 | 经 SSH 核对当前车端启动文件、旧 motor 实际路径、既有 OA 和 LYA 快捷方式。 |
| 16:31:47 | 安装组合启动文件并在原 commissioning 启动中加入 start_lya 参数。原诊断入口仍默认启动自身 LYA，正式桌面入口显式关闭。 |
| 16:32:18 | Foxy 预检组合参数。Allnodes 的 motor_cmd_topic 为 /semantic_drive/map_assist_045/diagnostic_cmd_vel，MapAssist 为 vehicle_output=true/start_lya=false。通过同名软件节点核对启动冲突检测。隔离域 195 的真实 MapAssist 发布正式 stop_wait 状态和 session，等待 LYA 时保持停车，无 LYA、OA 或物理 CAN 发布者。旧 motor 两个源码哈希与先前一致。 |
| 16:32:27 | 安装并核对「MapAssist（正式接线）」和「LYA（MapAssist）」桌面文件。通过参数列举和文件静态解析验证。没有执行完整实车组合启动。 |

桌面正式入口默认使用 5.5 米最小绕行半径，保留从独立 YOLOP 入口选择识别器的工作方式。通过桌面「OA 避障（oa_2）」接入避障，随后使用「LYA（MapAssist）」接入私有 ABC 处理结果和速度话题。终端入口 mapassist-live 可显式传入 lane:=inner 使用 6.0 米。

[隔离验证](no-lya-result.json) · [启动参数核对](validation.log) · [桌面安装核对](install.log)

16:42:54 JST 正式入口接入 `lane_0927oa` 的原生 ABC，见 [接线和验证记录](../README.md#mapassist-直接接入-lane_0927oa-的-abc)。
