# Semantic MapAssist 0.4.5 快捷方式

| 时间（JST） | 记录 |
| --- | --- |
| 2026-11-19 15:14:59 | SSH 连通，确认车机 Semantic 0.4.5 诊断入口及 XDG 桌面目录。 |
| 2026-11-19 15:15:05 | 安装 `~/.local/bin/semantic` 与桌面 `Semantic MapAssist.desktop`。 |
| 2026-11-19 15:15 | 验证交互式 Bash 查找命令、内外圈的 ROS `--show-args`、终端菜单与退出。使用 ROS 参数展示核对入口，无需启动节点。 |

桌面入口打开终端菜单。终端可运行 `semantic` 选择内外圈，也可用 `semantic inner` 或 `semantic outer` 直达。脚本加载车机现有 0.4.5 工作区并使用 commissioning 入口。

2026-11-19 15:18:43 JST，通过 SSH 将菜单改为 6.0 m 与 5.5 m。终端已核对 `semantic 6.0` 和 `semantic 5.5` 的 ROS 启动参数。数字分别映射至既有的 inner 和 outer。

2026-11-19 15:37:40–15:37:46 JST，通过 SSH 核对 ROS 包名和可执行路径后，安装 `~/.local/bin/lya` 与 `~/.local/bin/oa_2`。两者分别执行 `ros2 run trajectory_follower lya_follower_connected_omegat_global` 和 `ros2 run obsticle_avoidence oa_2`，自动加载原车 Foxy 环境并透传参数。已核对 shell 查找和可执行权限，未启动节点。
