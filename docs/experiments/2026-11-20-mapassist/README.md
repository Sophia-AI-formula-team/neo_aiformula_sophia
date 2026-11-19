# 11 月 20 日实车实验准备

部署 Agent 在车机 `control` 账号操作。采用 11 月 19 日收车时已安装的 Allnodes、`lane_0927oa`、YOLOP 选择器、`oa_2` 和 LYA。MapAssist 使用本分支包含的绿灯放行修复 `43f3df8`。实验分支为 `experiment/vehicle-prep-20261120`。

## 整理旧快捷方式并安装

车辆静止且上次实验进程已退出时，在新 Bash 终端运行。选择新的检出目录，保留原车工作区。

```bash
git clone --filter=blob:none --sparse --single-branch -b experiment/vehicle-prep-20261120 https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git ~/neo-1120
git -C ~/neo-1120 sparse-checkout set docs/experiments/2026-11-20-mapassist workspace/src/aiformula/launchers workspace/src/aiformula/perception/semantic_planning_system workspace/src/aiformula/control/semantic_vehicle_wiring
cd ~/neo-1120/docs/experiments/2026-11-20-mapassist
python3 shortcuts.py
bash prepare.sh
```

第一条 Python 命令列出迁移清单。`prepare.sh` 在 `~/aiformula_runtime/2026-11-20-mapassist` 构建独立 Foxy overlay，核实车上基本包和原生 ABC 参数后，先移走 19 日的入口，再安装新入口。全程不会启动识别、控制或 CAN 节点。车端已有 `~/.local/bin` 应在 PATH 中。完成后在新的交互终端用 `type -a allnodes yolop mapassist lya` 确认四条命令首先解析至该目录。

迁移清单包含旧的 `allnodes`、`lya`、`oa_2`、`oa-avoidance`、`semantic`、`mapassist-live`、`concept-live` 和两层 YOLOP 命令链接；也检查当前 XDG 桌面、旧隐藏桌面和应用菜单的桌面文件。只归档内容或链接目标与 19 日归档一致的入口，同名但内容不同先报告给负责人核对。模型、包源码、YOLOP 运行目录及 Concept 程序保留。旧入口备份在安装器打印的 `shortcut-backup-*` 目录，保留相对 HOME 的原路径。

## 四个终端依次启动

每一步使用独立终端，前面的进程保持运行。桌面四个 `1120` 编号图标调用相同命令。

| 顺序 | 命令 | 运行内容 |
|---|---|---|
| 1 | `allnodes` | 原车底盘、ZED、里程计、车道点和旧 motor。关闭内嵌识别，车道点发到 `/original_processed_point_{a,b,c}`，旧 motor 监听语义受控速度。 |
| 2 | `yolop` | 等相机出图，在终端选 **1 A3 V14、2 A4、3 A4Light**。三档使用车辆灯色、检测和锥桶话题。 |
| 3 | `mapassist` | 默认外圈 5.5 米；内圈用 `mapassist inner`，6 米。一次启动 MapAssist、停车像素适配、OA 选择器、带四项 remap 的 `oa_2`、正式停车状态和 motor session。等 OA 的真实旗标。 |
| 4 | `lya` | 启动原车安装的 connected global LYA，从语义选择器读取过滤后的位姿、角速度，将命令送回 MapAssist 私有速度门。 |

## 出发前核对

在车机同一 ROS 域查看 `ros2 topic info --verbose`。核对 `/point_flag` 来自 OA、三个 `/oa_point_*` 均接选择器、`/flag_stop` 来自真实检测像素适配器。`/semantic_drive/map_assist_045/lya_cmd_vel` 应由专用 LYA 发布，`/semantic_drive/map_assist_045/diagnostic_cmd_vel` 应由 MapAssist 发布并被旧 motor 订阅；该历史命名在正式模式承担受控速度。核对 `/semantic_drive/map_status` 与 `/semantic_drive/motor_session` 的唯一发布者。若 19 日现场安装与当前机器不符，记录包路径、话题和发布者后反馈；不要用固定 Bool 或手发 ABC 代替。

确认红灯后丢框，即使停车旗标为 False 也要停车。稳定绿灯才可直行；重新确认红灯且已匹配、有可行切入轨迹才可绕行。建图试验应起步前开录，保留同一 MapAssist 进程连续两圈。结束时先人工停稳，再按 LYA、MapAssist、YOLOP、Allnodes 的顺序 Ctrl-C。OA 和 MapAssist 随第三个终端一起退出。

如需恢复快捷方式，停稳退出所有实验进程，移走本次安装的四个 `~/.local/bin` 命令和四个 `1120-*.desktop`（仅对应本实验的文件），将备份目录中的文件按相对路径还原到 HOME。原车 Allnodes 和识别权重沿用 19 日安装。

## 发布前验证记录

在断网的 ROS 2 Foxy 容器中，四个 overlay 包通过 colcon 构建，两个 launch 文件的参数可以展开。用 19 日现场归档还原七个命令文件、两个命令链接、八个桌面入口和一份应用菜单副本，预览后共归档 18 个旧入口，并生成四个命令和四个桌面入口。重复安装没有增加备份；同名自定义命令会中止迁移，无关桌面文件保留。

在隔离环境中执行四个新命令，记录 ROS 命令行参数。已验证 YOLOP 菜单选 1、内外圈参数、OA 随 MapAssist 启动及独立 LYA 的三条 remap。该项用命令记录器代替最终的 ROS 执行命令。真实传感器、OA 点、交通灯和 motor 的闭环留待车上实验记录。
