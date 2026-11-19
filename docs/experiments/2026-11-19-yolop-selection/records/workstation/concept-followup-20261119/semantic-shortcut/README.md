# Semantic MapAssist 0.4.5 快捷方式

| 时间（JST） | 记录 |
| --- | --- |
| 2026-11-19 15:14:59 | SSH 连通，确认车机 Semantic 0.4.5 诊断入口及 XDG 桌面目录。 |
| 2026-11-19 15:15:05 | 安装 `~/.local/bin/semantic` 与桌面 `Semantic MapAssist.desktop`。 |
| 2026-11-19 15:15 | 验证交互式 Bash 查找命令、内外圈的 ROS `--show-args`、终端菜单与退出。使用 ROS 参数展示核对入口，无需启动节点。 |

桌面入口打开终端菜单。终端可运行 `semantic` 选择内外圈，也可用 `semantic inner` 或 `semantic outer` 直达。脚本加载车机现有 0.4.5 工作区并使用 commissioning 入口。

2026-11-19 15:18:43 JST，通过 SSH 将菜单改为 6.0 m 与 5.5 m。终端已核对 `semantic 6.0` 和 `semantic 5.5` 的 ROS 启动参数。数字分别映射至既有的 inner 和 outer。

2026-11-19 15:37:40–15:37:46 JST，通过 SSH 核对 ROS 包名和可执行路径后，安装 `~/.local/bin/lya` 与 `~/.local/bin/oa_2`。两者分别执行 `ros2 run trajectory_follower lya_follower_connected_omegat_global` 和 `ros2 run obsticle_avoidence oa_2`，自动加载原车 Foxy 环境并透传参数。已核对 shell 查找和可执行权限，未启动节点。

### 原车桌面入口

| JST | 操作 |
| --- | --- |
| 16:25:45 | 通过 SSH 检查 XDG 桌面、既有 LYA 快捷入口和安装的 allnodes 与 oa_2 可执行路径。 |
| 16:26:21 | 安装并核对三个可执行桌面文件。Allnodes 使用原车默认参数，LYA 指向独立的 connected global follower，OA 用已隔离验证的四项重映射，通过车端既有 runner 启动 oa_2 并在正常退出时尝试恢复车道来源。 |
| 16:26:21 | 桌面文件通过 desktop-file-validate，Allnodes 参数表和 OA 命令预检通过，核对 LYA 可执行权限。本轮验证未启动车辆节点。 |

[桌面文件、入口脚本与验证结果](vehicle-desktop/)

16:32 JST 新增 [MapAssist 正式接线组合入口](mapassist-formal/README.md)，LYA 和 OA 由操作者分别启动。

### Allnodes 切换至 lane_0927oa

| JST | 操作与验证 |
| --- | --- |
| 16:40:00–16:40:34 | 核对 Allnodes 与 MapAssist 的实际启动文件，以及两版节点的点云、ABC 和避障话题。 |
| 16:40:58 | 备份并更新车机 `allnodes.launch.py`。`lane_0927oa` 直接订阅原车三路点云，ABC 发布到 `/processed_point_{a,b,c}`，订阅 `/point_flag` 以跟随 OA 接管状态。两个桌面入口都读取这份文件。 |
| 16:41:18–16:42:00 | 核对两个启动入口及 motor 文件校验值。车机隔离域中以合成点云确认正常输出 ABC、OA 接管时暂停、释放后恢复。隔离进程已退出。本次未执行整车启动。 |

[启动文件补丁](lane0927oa/allnodes-lane0927oa.patch) · [部署日志](lane0927oa/deploy.log) · [启动参数核对](lane0927oa/launch-check.log) · [车机隔离验证](lane0927oa/isolation-check.log)

### MapAssist 直接接入 lane_0927oa 的 ABC

| JST | 操作与验证 |
| --- | --- |
| 16:42:32 | 核对车机实际加载的 Allnodes、MapAssist 正式入口、commissioning 和规划节点启动文件。 |
| 16:42:54 | 备份并部署四份启动文件。正式入口为 Allnodes 和 MapAssist 同时配置 `/original_processed_point_` 前缀。独立 Allnodes 和 Semantic 诊断入口默认使用 `/processed_point_`。 |
| 16:43:22–16:43:37 | 核对三个入口的启动参数和 motor 文件校验值。首次隔离测试在发现完整 ROS 话题图之前达到超时。生成的 MapAssist 参数文件已使用三条原生 ABC 话题。 |
| 16:44:12–16:44:30 | 重跑隔离验证，确认三个原生 ABC 的唯一发布者为 `lane_0927oa`，MapAssist 分别订阅。用合成数据确认 LYA 私有输入依次使用车道点、OA 避障点、恢复后的车道点。已确认隔离进程退出。 |

在车机隔离 ROS 域验证数据链路。正式入口仍由用户单独启动 OA 和 LYA，本次没有整车试跑。

[接线补丁](lane0927oa/mapassist/mapassist-lane0927oa.patch) · [部署日志](lane0927oa/mapassist/deploy.log) · [启动参数核对](lane0927oa/mapassist/shortcut-check.log) · [隔离验证](lane0927oa/mapassist/isolation-check.log)

### MapAssist 桌面入口退出排查

| JST | 检查 |
| --- | --- |
| 16:47:32、16:47:44 | 两份 MapAssist 日志均在启动子进程前结束。同期 Concept 的 ZED 和 motor 进程仍在运行。原日志只记录了 launch 初始化，退出原因依据进程时间线和冲突保护推断。 |
| 16:48:06–16:48:52 | Concept 收到 Ctrl+C，ZED 于 16:48:12 退出，最后的面板于 16:48:52 退出。 |
| 16:48:29–16:49:31 | 单独执行正式入口的冲突检查，并在隔离域以同名占用节点验证拦截。确认车机 `can0` 为 `up` 且 `carrier=1`，Concept 已结束。 |

启动顺序为 **MapAssist（正式接线）**，相机出图后通过 **YOLOP Perception** 选 **1（A3 V14）**，随后开启 **OA 避障（oa_2）**，最后开启 **LYA（MapAssist）**。

[车机时间线](mapassist-startup/timeline.log) · [冲突检查](mapassist-startup/owner-check.log) · [隔离复核](mapassist-startup/guard-isolation.log)
