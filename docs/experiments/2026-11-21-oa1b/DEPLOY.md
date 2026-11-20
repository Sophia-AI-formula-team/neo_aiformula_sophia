# 1121 车端实验 · OA 1B 0.1.0

部署对象是原车 ROS 2 Foxy、AV314、MapAssist 0.4.5、OA 2.1 和 connected global LYA。分支为 `experiment/oa1b-1121`，基于 1121 准备分支 `dff29d12e805`。本实验增加一个 ROS 包、一套按 Git 提交隔离的运行目录，以及四个新桌面图标。车上的 `allnodes`、`yolop`、`mapassist`、`lya` 和 Concept 继续使用原入口。只运行其中一套驾驶链。

## 本轮要测什么

1. AV314 首次识别仍走原流程。仅在规划器已确认红灯停车、里程计显示车已停稳、同一灯牌已被连续识别后，对原模型本帧留下的交通灯候选做局部复核。候选与灯牌重叠 IoU 至少 0.50，分数至少 0.30，当前画面仍须有符合原颜色和亮区形状条件的红色，连续两帧满足才补红。原检测器全局阈值保留 0.50。绿灯、推杆解除停车、过期或明显移动会使关联失效。
2. 同一图像内多个灯框覆盖的红像素只数一次。默认停车阈值 **50 个去重红像素**。这是 1121 的起测值。以往 50、450、600 的框内累加结果不可直接换算成这个阈值或停车距离。现场应记录首次触发时到灯距离和停稳后灯牌是否仍可见。
3. 手柄保持最高优先级。电机收到带使能的非零手动输入时发一次带时间戳的事件，规划器据此清除当前红灯停车标志。持续推住不会重复清除。等待时若连续收到新鲜的非红灯帧（含 `unknown`）满一秒，沿用 1121 的恢复策略。期间再次确认红灯会重置等待，之后重新确认红灯仍能停车。这个规则允许真实红灯持续漏检时放行，本次同灯复核仍有漏检。
4. OA 仍使用 2.1 和分组 ABC，沿用官方拓扑图和车上 LYA。内圈最小转弯半径 6.0 m，外圈 5.5 m。此次未改避障轨迹或转向限幅。

## 部署 agent 操作

在车机用户下使用单独的干净工作目录。先留存旧四个名字的解析和原来的桌面入口，再准备新版。全程使用同一个用户和既有 ROS 域。准备和安装快捷方式均不会启动底盘。

```bash
type -a allnodes yolop mapassist lya
git clone --filter=blob:none --sparse --branch experiment/oa1b-1121 \
  https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git ~/neo-1121-oa1b
cd ~/neo-1121-oa1b
git sparse-checkout set docs/experiments/2026-11-21-oa1b workspace/src/aiformula/perception/oa_1b
git log -1 --format='%H %s'
prep="$PWD/docs/experiments/2026-11-21-oa1b"
rev=$(git rev-parse HEAD)
release="$HOME/aiformula_runtime/2026-11-21-oa1b/$rev"
bash "$prep/prepare.sh" && \
  python3 "$prep/shortcuts.py" preview "$release" && \
  python3 "$prep/shortcuts.py" install "$release" && \
  python3 "$prep/shortcuts.py" check "$release"
type -a oa1b-allnodes oa1b-yolop oa1b oa1b-lya oa1b-stop oa1b-verify
type -a allnodes yolop mapassist lya
```

每个新命令应解析到 `~/.local/bin/` 中的同名文件。若该目录不在交互式终端的 PATH，可直接使用这六个绝对路径。桌面图标也使用绝对路径。安装器会保留旧入口，遇到同名的自定义文件或链接会报出具体路径。遇到版本或命名冲突，向负责人反馈实际哈希、前缀或冲突路径后再改。

准备脚本从 `/home/workspace/install` 读取 Allnodes、LYA 和电机，从 `~/aiformula_runtime/2026-11-20-mapassist-av314/install` 读取已上车的规划与 OA，从 `~/aiformula_runtime/yolop_a3_v14_20261012` 读取识别环境和权重。它校验源文件和模型，私有复制规划器和电机 Python 包，再应用当天已经过 Foxy 测试的手柄事件差异。运行时检查 ROS console 实际导入的文件。原安装目录、识别模型、shell 启动文件和系统 ROS 环境均无需改写。保存 `$release/control/control-version.json`、`$release/READY` 和准备输出。

## 操作员四终端

按顺序打开四个标有 **1121 OA 1B** 的图标，或分别在四个前台终端执行下表命令。第二个入口固定使用 AV314，每帧仅推理一次。第三个图标默认外圈，右键 **1121 OA 1B Inner** 可选内圈。

| 终端 | 命令 | 实际启动内容 |
|---|---|---|
| 1 | `oa1b-allnodes` | 原车 Allnodes，启用 ZED、里程计、LLP、车道选点、电机等。关掉内嵌 YOLOP，实验电机订阅最终指令 |
| 2 | `oa1b-yolop` | 一次 AV314 推理，同时提供原识别输出和实验灯色、去重像素 |
| 3 | `oa1b outer` 或 `oa1b inner` | MapAssist、OA 2.1、分组选择器、停车位置适配器、停车状态与电机会话。可选第二个参数为去重像素阈值，如 `oa1b inner 50` |
| 4 | `oa1b-lya` | 原车 `lya_follower_connected_omegat_global` 控制器 |

四个终端都出现对应节点后，在第五个终端运行 `oa1b-verify`。它从当前 DDS 域查询发布者、订阅者及消息类型。车端入口还要求 `/aiformula_control/socket_can_sender` 实际订阅电机的 CAN 话题。可用 `ros2 lifecycle get /aiformula_control/socket_can_sender` 核对状态为 `active`。保存 DDS 图和停车像素状态。实际收发和车身运动要对照现场记录。若试验数据要与本次回放比较，保留原图和源时间戳。

```bash
run="$HOME/aiformula_runtime/2026-11-21-oa1b/runs/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$run"
oa1b-verify > "$run/graph.json"
ros2 bag record -o "$run/signal-control" \
  /aiformula_sensing/zed_node/left_image/undistorted /aiformula_sensing/gyro_odometry_publisher/odom \
  /aiformula_perception/yolop_a3/detections /oa_1b/perception/detections /oa_1b/perception/traffic_light/state \
  /flag_stop /point_flag /semantic_drive/map_assist_045/pixel_stop_status /semantic_drive/map_assist_045/status \
  /semantic_drive/map_assist_045/oa_triplet /semantic_drive/map_assist_045/oa_ready \
  /semantic_drive/map_assist_045/lya_cmd_vel /semantic_drive/map_assist_045/diagnostic_cmd_vel \
  /aiformula_control/joy_node/joy_l2_throttle /aiformula_control/motor_controller/manual_motion_started \
  /semantic_drive/motor_session /semantic_drive/map_status \
  /aiformula_control/motor_controller/gate_status /aiformula_control/motor_controller/reference_signal
```

## 上下游接线

| 输入或输出 | 消息、发布和接收 |
|---|---|
| 左相机与里程计 | 原车 `/aiformula_sensing/zed_node/left_image/undistorted` 和 `/aiformula_sensing/gyro_odometry_publisher/odom`。识别使用原图时间戳，灯牌跟踪另外订阅真实里程计和 `/semantic_drive/map_assist_045/status` |
| 原有感知输出 | 公共检测 JSON、灯色、原红像素、车道图、锥桶 mask 和 overlay 照常发布。车道 mask → LLP → `lane_0927oa` → `/original_processed_point_{a,b,c}` → MapAssist |
| 实验交通灯 | `/oa_1b/perception/detections`（String JSON）→ 像素适配器 → `/flag_stop`（Bool）。`/oa_1b/perception/traffic_light/state`（String）→ MapAssist。像素适配器核对新鲜图像戳和光学坐标系 |
| 避障 | `/cone_mask` → OA 2.1 → `/semantic_drive/map_assist_045/oa_triplet`（PoseArray，含同窗口 A、B、C）→ 选择器。切换参考 `/point_flag` 与 `…/oa_ready` |
| LYA 与电机 | 选择器 `…/filtered_lane_pose`、`…/filtered_omega_t` → LYA → `…/lya_cmd_vel` → MapAssist → `…/diagnostic_cmd_vel` → 电机仲裁 → `/aiformula_control/motor_controller/reference_signal`。其中 `…` 为 `/semantic_drive/map_assist_045` |
| 手柄 | 原车 `/aiformula_control/joy_node/joy_l2_throttle` → 电机 → `/aiformula_control/motor_controller/manual_motion_started`（Time）→ MapAssist。电机继续订阅 `/semantic_drive/motor_session` 和 `/semantic_drive/map_status` |

## 停机和回到原车版本

在任一实验终端按 **Ctrl+C**、关闭其中一个实验终端，或在同一 ROS 域执行 **`oa1b-stop`**。监听四个终端的监督进程会依次发 INT、TERM，仍存活的自有后代才收 KILL。回收全部子孙后释放旧版锁。若额外启动了上述 bag 记录器，在它自己的终端按 Ctrl+C 写完索引。

复核 `ros2 node list`、`ros2 topic info --verbose /semantic_drive/map_assist_045/diagnostic_cmd_vel` 及 `ros2 topic info --verbose /aiformula_control/motor_controller/reference_signal`，确认本次节点和发布者已经退出。然后可用原来的 `allnodes → yolop → mapassist → lya`。新版和旧版共用实际底盘话题，不能同时启动。监督器只回收本次进程树，保留同机其他作业。卸除本次快捷方式时在原检出目录执行 `python3 "$prep/shortcuts.py" remove "$release"`。切换新提交时先退出旧会话并移除旧提交自己的 OA 1B 快捷方式，再安装新提交的入口。

部署回报请列出 Git 完整提交、ROS 域、所选车道和像素阈值、`control-version.json`、`oa1b-verify` 输出、实际手柄事件、停车距离、最终电机来源，以及退出后旧版入口的检查结果。台架证据与现场待测项见[验证记录](VALIDATION.md)。
