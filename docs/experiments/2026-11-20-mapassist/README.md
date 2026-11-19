# 11 月 20 日实车实验准备

**Concept V6.27.8 已合入正式部署分支。** 车机用同名 `concept-live`、原 Concept 桌面图标或应用菜单启动，在终端选 Pure Pursuit 或 LYA。首次安装、覆盖旧快捷方式以及回退请按 [Concept 1120 部署说明](concept/DEPLOY.md)。下方四终端是单独的 MapAssist 方案。

部署 Agent 在车机 `control` 账号操作。采用 11 月 19 日收车时已安装的 Allnodes、`lane_0927oa`、YOLOP 选择器和 LYA。`oa_2` 由本次 overlay 提供。MapAssist 使用本分支包含的绿灯放行修复 `43f3df8`。AV314 分组试验已经合入本准备分支。

本轮 AV314 内圈 450 px 的部署入口、OA2 分组接线、实测日志和回退步骤见 [AV314 部署与验证](AV314_DEPLOY.md)。

## 整理旧快捷方式并安装

车辆静止且上次实验进程已退出时，在新 Bash 终端运行。选择新的检出目录，保留原车工作区。

```bash
git clone --filter=blob:none --sparse --single-branch -b experiment/vehicle-prep-20261120 https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git ~/neo-1120
git -C ~/neo-1120 sparse-checkout set docs/experiments/2026-11-20-mapassist workspace/src/aiformula/launchers workspace/src/aiformula/perception/semantic_planning_system workspace/src/aiformula/control/semantic_vehicle_wiring workspace/src/aiformula/perception/obsticle_avoidence
cd ~/neo-1120/docs/experiments/2026-11-20-mapassist
python3 shortcuts.py
bash prepare.sh
```

第一条 Python 命令列出迁移清单。`prepare.sh` 在 `~/aiformula_runtime/2026-11-20-mapassist-av314` 构建独立 Foxy overlay，核实车上基本包和原生 ABC 参数后，先移走 19 日的入口，再安装新入口。全程不会启动识别、控制或 CAN 节点。车端已有 `~/.local/bin` 应在 PATH 中。完成后在新的交互终端用 `type -a allnodes yolop mapassist lya` 确认四条命令首先解析至该目录。随后在新交互 Bash 中 source 本目录的 `verify-shortcuts.bash`，检查旧命令、桌面入口与同名别名。Concept 的旧命令与图标由 [Concept 安装器](concept/DEPLOY.md)单独更新。

迁移清单包含旧的 `allnodes`、`lya`、`oa_2`、`oa-avoidance`、`semantic`、`mapassist-live`和两层 YOLOP 命令链接；也检查当前 XDG 桌面、旧隐藏桌面和应用菜单的桌面文件。只归档内容或链接目标与 19 日归档一致的入口，同名但内容不同先报告给负责人核对。模型、包源码、YOLOP 运行目录及 Concept 程序保留。旧入口备份在安装器打印的 `shortcut-backup-*` 目录，保留相对 HOME 的原路径。

## 四个终端依次启动

### 1120 的 A4 / A4Light 更新

在四个终端启动前，退出所有旧 YOLOP 进程。在可访问新 A4 提交的 neo
检出中获取 `main`，再从本目录运行预览。更新器读取固定提交的三份
runtime 源码，核对两套车端已安装文件和权重的 SHA256；不符合 1119 归档
基线时停止，不能用它覆盖未知本地修改。

```bash
git -C ~/neo-1120 fetch origin main
python3 update_yolop_1120.py --checkout ~/neo-1120
python3 update_yolop_1120.py --checkout ~/neo-1120 --apply
```

只覆盖 1119 选择器下的 A4 / A4_lite runtime 文件，保留 V14、权重、Foxy
适配、选择器、ROS 话题及控制。重复运行应报告 0 项；备份目录由脚本打印，
保留相对于选择器根目录的路径。回退时先退出 YOLOP，再以备份恢复这三份
源码。安装器不启动 ROS。实际停车验收前，必须用失败灯帧复跑，并确认
当前门控不会因 unknown、漏检或旧数据解除红灯等待。

每一步使用独立终端，前面的进程保持运行。桌面四个 `1120` 编号图标调用相同命令。

| 顺序 | 命令 | 运行内容 |
|---|---|---|
| 1 | `allnodes` | 原车底盘、ZED、里程计、车道点和旧 motor。关闭内嵌识别，车道点发到 `/original_processed_point_{a,b,c}`，旧 motor 监听语义受控速度。 |
| 2 | `yolop` | 等相机出图，在终端选 **1 A3 V14、2 A4、3 A4Light**。三档使用车辆灯色、检测和锥桶话题。 |
| 3 | `mapassist` | 默认外圈 5.5 米；内圈用 `mapassist inner`，6 米。一次启动 MapAssist、停车像素适配、OA 选择器、带四项 remap 的 `oa_2`、正式停车状态和 motor session。等 OA 的真实旗标。 |
| 4 | `lya` | 启动原车安装的 connected global LYA，从语义选择器读取过滤后的位姿、角速度，将命令送回 MapAssist 私有速度门。 |

## 出发前核对

接线按 11 月 19 日从车机带回的已安装文件核对。`lane_0927oa` 的 `/original_processed_point_a/b/c` 送给 MapAssist。独立 YOLOP 发布 `/aiformula_perception/yolop_a3/traffic_light/state` 给 MapAssist，并另发检测 JSON 给停车像素适配器。

在车机同一 ROS 域用 `ros2 topic info --verbose` 核对 `/point_flag` 来自 OA，OA2 在 `/semantic_drive/map_assist_045/oa_triplet` 发送同一计算窗口的三个点（`PoseArray`），选择器在 OA 模式按整组接收，选择器以 `/semantic_drive/map_assist_045/oa_ready` 通知 MapAssist 收到了新的一组 OA 三点。核对 `/flag_stop` 来自停车像素适配器。专用 LYA 发布 `/semantic_drive/map_assist_045/lya_cmd_vel`，MapAssist 向旧 motor 发布 `/semantic_drive/map_assist_045/diagnostic_cmd_vel`。同时核对 `/semantic_drive/map_status` 和 `/semantic_drive/motor_session` 的发布者。Allnodes 的全局三点处理器和私有选择器都叫 `data_processing_node`，专用 LYA 读取私有的 `filtered_lane_pose` 和 `filtered_omega_t`。若车机安装与 19 日归档不符，记录实际包路径和话题发布者。

自动速度优先级为 **红灯停车 > OA > 绿灯后正常循迹**。进入红灯停车即发送全零速度，之后由独立的定时器默认以 20 Hz 持续发送。首圈按距离标志停车。确认红灯后丢框也会停车。因到达停车点、距离信号缺失或无可用绕行而锁定停车后，`flag_stop=False`、`unknown` 和 OA 指令都不能解除。绿灯确认后，如果 OA 仍接管，要先收到新的一组 OA 三点和新 LYA 速度。OA 已退出时，使用新的视觉三点和新 LYA 速度。在后续圈因丢失灯色暂时停止时，重新确认红灯且切入轨迹通过检查，可以恢复已验证的绕行流程。

建图试验应起步前开录，保留同一 MapAssist 进程连续两圈。结束时先人工停稳，再按 LYA、MapAssist、YOLOP、Allnodes 的顺序 Ctrl-C。OA 和 MapAssist 随第三个终端一起退出。

如需恢复快捷方式，停稳退出所有实验进程，移走本次安装的四个 `~/.local/bin` 命令和四个 `1120-*.desktop`（仅对应本实验的文件），将备份目录中的文件按相对路径还原到 HOME。原车 Allnodes 和识别权重沿用 19 日安装。

## 发布前验证记录

在断网的 ROS 2 Foxy 容器中，四个 overlay 包通过 colcon 构建，两个 launch 文件的参数可以展开。用 19 日现场归档还原七个命令文件、两个命令链接、八个桌面入口和一份应用菜单副本，预览后共归档 18 个旧入口，并生成四个命令和四个桌面入口。重复安装没有增加备份；同名自定义命令会中止迁移，无关桌面文件保留。

在隔离环境中执行四个新命令，记录 ROS 命令行参数。已验证 YOLOP 菜单选 1、内外圈参数、OA 随 MapAssist 启动及独立 LYA 的三条 remap。该项用命令记录器代替最终的 ROS 执行命令。真实传感器、OA 点、交通灯和 motor 的闭环留待车上实验记录。
