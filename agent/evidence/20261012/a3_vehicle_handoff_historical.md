# YOLOP A3 V14 车辆部署交接

2026 年 10 月 12 日，`control@192.168.10.111` 已安装 A3 V14，并完成实时相机、车道掩码和人员识别输出验收。验收启动进程已经停止，用户可以直接启动。完整整车控制和行驶准确率未列入本次输出验收。

## 开启流程

连接车辆，在车辆终端运行：

```bash
ssh control@192.168.10.111
source /opt/ros/foxy/setup.bash
source /home/workspace/install/local_setup.bash
ros2 launch launchers yolop_a3.launch.py
```

最后一条命令启动 ZED 相机、A3 V14、原有 lane_line_publisher 和车体 TF。停止使用 Ctrl+C。旧 YOLOP、A2 和独立 cone_detector 必须保持停止，避免相同话题存在多个发布者。新入口会检查重复发布者并拒绝重复启动。

如需原有整车节点一起启动，使用下面的入口，替代上述最后一条命令：

```bash
ros2 launch launchers allnodes.launch.py
```

allnodes 已默认包含 A3，自行启动相机和车道消费者，所以两种入口不能同时运行。allnodes 保留原有 CAN、手柄和电机控制行为；本次只核对了该入口的加载和启动参数，未启动整车控制验收。

如相机和车道消费者已由其他启动入口运行，只单独启动 A3：

```bash
ros2 launch launchers yolop_a3.launch.py start_camera:=false start_lane_publisher:=false
```

## 发布版本与架构

来源为 [a3-v14 标签](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/a3-v14)，标签指向 `05f1651`，发布提交为 `3d68d51de300c62fe4da74af379d60e374feb484`。选定权重是 local V14 step 16384，架构 `a3_mobilenet_shared_fpn_contract_v14`。

权重为 79763687 字节，SHA256：

```text
a411c983b661d6b26217019a968251c89a6b66c95400e4e7cb78f04a644ca10c
```

A3 使用一次 MobileNetV3 编码器遍历，输出独立的车道分割和 FPN/RPN/ROI 物体检测；只有一个模型权重。没有使用 A2 的两个模型、旧 SSD 或远程 A1/FCOS。保留原生 min_size 640、max_size 1280、ImageNet 归一化、原尺寸恢复、分割阈值 0.5 和检测阈值 0.5；车道形态学处理关闭。

## 接线与标定

| 接口 | 话题与格式 |
| --- | --- |
| 相机输入 | `/aiformula_sensing/zed_node/left_image/undistorted` |
| 车道掩码 | `/aiformula_perception/road_detector/mask_image`，Image mono8，0/255 |
| 车道可视化 | `/aiformula_visualization/road_detector/annotated_mask_image`，Image bgr8 |
| 障碍掩码 | `/cone_mask`，Image mono8，人/车/锥桶的填充框 |
| 物体识别框 | `/cone_bbox_image`，Image bgr8 |
| 识别 JSON | `/aiformula_perception/yolop_a3/detections`，String |
| 灯状态 | `/aiformula_perception/yolop_a3/traffic_light/state`，String |
| 红像素计数 | `/aiformula_perception/yolop_a3/red_pixels_count`，Int32 |

四类输出图像保持输入头信息和尺寸。原有 lane_line_publisher 订阅车道掩码；OA 的输入话题名保留。灯状态保留独立命名，未接管已有语义规划灯输入。OA 和控制器未启动。

实际相机序列号为 SN48797506，原启动配置是 SN48442725。新增当前相机的 `intrinsic_nHD.yaml`，从实时 rectified CameraInfo 读取：640×360，fx=fy=246.07122802734375，cx=328.65667724609375，cy=189.0115509033203。allnodes 和 A3 的 camera_sn 默认值改为实际相机。旧相机标定文件保留，车体外参和车道算法未改动。

## Jetson 安装路径

部署根目录为 `/home/control/aiformula_runtime/yolop_a3_v14_20261012`。

- `upstream/YOLOP_A3_Deploy` 保存完整原版；车辆 road_detector 下的 `YOLOP_A3_Deploy` 是指向该目录的链接。
- `platform` 是兼容副本；`venv` 安装其 wheel，`ros_install` 是独立 ROS wrapper 安装目录。
- `run_a3.py` 启动前核对固定 V14 权重哈希，再进入安装的 ROS adapter。
- 启动源文件在 `/home/workspace/src/aiformula/launchers/launch`；安装目录使用源文件链接。
- 启动前备份在 `/home/control/aiformula_backups/20261012_yolop_a3/launchers_before.tar.gz`，原始启动文件哈希在同目录 `launches_before.sha256`。

车辆为 Jetson AGX Orin 64 GB，L4T 35.4.1，Foxy/Python 3.8.10。上游声明 Python 3.10+，本次明确采用经过实测的 Foxy 兼容副本：仅在 `outputs.py` 增加 postponed annotations，并用独立 setup.py/pyproject 构建 `0.3.0+jetson.foxy`。权重和神经网络文件未改变，`source_hashes.json` 记录逐文件对比。

新 venv 通过 `.pth` 读取现有 A2 私有环境的 NVIDIA Torch 2.0.0+nv23.05、TorchVision 0.15.1 和 NumPy 1.24.4，并私有安装 typing_extensions 4.12.2。系统 OpenCV 为 4.2.0；rclpy/cv_bridge 使用同一 Python 3.8 ABI。没有更换系统 Python/CUDA，也没有卸载 A2。该依赖复用要求保留 `/home/control/aiformula_runtime/yolop_a2_20261005`。

## 验收证据

证据文件在部署根目录，并有本地副本 `integration/a3_deployment_20261012/evidence`。

- 权重哈希、328 张量和 V14 输入契约检查通过；CUDA NMS/ROI Align 执行通过。
- 26 项上游测试通过；独立 wheel 安装和 colcon ROS wrapper 构建通过。
- 原尺寸相机帧 GPU 对比：安装版与不可变上游的车道像素差为 0，每帧均只遍历一次编码器。含目标帧同样通过，人员检测框、分数、类别完全一致，检测数量为 1。远程 `gpu_parity_target.json` 内容已由工具返回，本地保存为 `gpu_parity_target_tool_result.json`。
- `live_observation01.json` 的 45 秒窗口收到 490 帧非空车道掩码、538 条检测 JSON，含 162 个 person 识别结果；障碍掩码有 109 帧非空。所有观察掩码为 640×360 mono8 二值，匹配的相机头信息无尺寸或 frame_id 差异，各输出只有一名 A3 发布者，未出现回调错误。推理总时间 p50 45.08 ms、p95 52.15 ms；接收频率不等于保证的发布吞吐量。
- `live_target03.json` 保存再次呈现目标的人员检测和同头信息的相机、车道掩码、障碍掩码、识别框。代表人员检测分数 0.99828，截图在 `live_target03/detected_object_overlay.png`。该窗口另出现 unknown_traffic_light，未验证其分类准确性。
- `restart_observation.json` 证明默认入口重启后继续输出：127 帧车道掩码、144 帧障碍掩码、146 条 JSON，包含 139 个 person 检测。默认入口的相机/A3 图像观察无错误。
- `duplicate_guard.log` 证明重复启动被拒绝。lane mask 的 ROS graph 确认消费者为原有 lane_line_publisher。

当前场景的车道点云消息为空；仅验证消费者接收和发布链路，未通过道路几何或可行驶路径验收。原有 lane_line_publisher 在两次 SIGINT 退出时均报 FastDDS wait-set 错误并以 -11 退出；A3、相机和 TF 正常退出，全部已结束。该既有节点的退出异常未在本次修改其算法或依赖。启动输出验收不等于所有节点都正常退出，也不等于自主驾驶准确率或上游质量门槛通过。

## 回退

原有 A2 代码、两份权重、私有环境、`yolop_a2.launch.py` 和更早的 legacy 备份保留。停止 A3 后，原有整车入口可以通过 `launch_yolop_a3:=false` 禁用 A3，并在另一终端运行 `ros2 launch launchers yolop_a2.launch.py`。不要同时运行 A2/A3。

完整启动文件回退使用上述 `launchers_before.tar.gz` 中的原始 allnodes，先核对当前文件哈希并备份新版本；保留新增 A3 目录和当前相机标定。回退原始 allnodes 时仍应显式选择 `camera_sn:=SN48797506`。不要删除任何旧环境、权重或备份。

## 最终连接状态

停止后的进程列表确认 A3、相机和车道消费者已结束。随后车辆地址从局域网断开，新 SSH 返回 No route to host；最终逐文件远程校验和新增 vehicle_handoff.md 传输未完成。本地交接和已取回的验收证据完整保留。恢复供电与局域网连接后即可使用上述已安装入口；无需重复安装。上游原始 handoff 已随部署包复制到车辆。

## allnodes 重复启动排查

同日用户启动独立 A3 后再次启动 allnodes，触发 A3 的重复发布保护。独立会话 PID 6763 仍发布车道掩码、cone_mask 和 cone_bbox_image；read-only 调用 allnodes 所用 A3 preflight 复现了 Existing publishers 错误。随后只对该已核对身份的独立 A3 会话发送 SIGINT，确认相机及三类感知输出发布者均为零，preflight 返回正常。未更改模型或放宽重复发布保护，也未启动电机/CAN 整车进程。用户现在只需运行 ros2 launch launchers allnodes.launch.py，不应再同时开启 yolop_a3.launch.py。证据为 allnodes_conflict_20261012.json。网络连接已恢复，车辆交接副本此次同步保存。
