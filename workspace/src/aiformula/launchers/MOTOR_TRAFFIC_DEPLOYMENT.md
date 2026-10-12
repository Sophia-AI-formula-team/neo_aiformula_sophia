# 实车 motor 红绿灯门控 / topo 建图

本次用户授权 PR23 编译部署、motor 内门控、手柄优先、修复零轮速；禁止移动车辆。新包 motor_controller 0.3.0；topo 保留 PR23 MapAssist 0.2.1 / core 0.2.0。旧 shadow 入口保持独立，不解除其输出限制。

## 接线与启动

```text
allnodes: A3 → a3_signal_capture → /semantic_drive/signal_capture → motor_controller
                                    └→ signal_capture_bridge → MapAssist
allnodes: 里程计 + /processed_point_{a,b,c} ─────────────────────→ MapAssist
MapAssist → /semantic_drive/map_status + /semantic_drive/reference_path
单独的一个 LYA → /aiformula_control/game_pad/cmd_vel ───────────→ motor_controller
teleop → /aiformula_control/manual/cmd_vel + 映射后 Joy ────────→ motor_controller
motor 内：手动优先 / 自动红灯门控 / 超时 / 零轮速 → CAN 0x210
```

本次完成安装编译、隔离测试及明确禁用运动的零指令启动检查，没有启动 LYA 驾驶。后续静止检查可用以下入口，默认运动禁止：

```bash
source /opt/ros/foxy/setup.bash
source /home/workspace/install/setup.bash
ros2 launch launchers allnodes.launch.py
# 第二终端，同样 source；只选一个已有 LYA，不叠加多个版本：
ros2 run trajectory_follower lya_follower_connected_omegat_global
# 第三终端，同样 source：
ros2 launch launchers semantic_vehicle.launch.py
```

只有之后明确允许运动时，第一条启动命令才加 `enable_motion:=true`，需重新启动 allnodes。不要在本次静止部署中启用。LYA 算法/增益未改；现有 game_pad 话题现在是自动输入，手柄已独立。standalone motor launch 也默认禁止运动。

`semantic_vehicle` 只启动 A3 捕获、信号桥、MapAssist，不重复启动 LYA/LLP/滤波器。MapAssist 的点和速度输出仅用于诊断，真正红灯门控在 motor 内。它读取同一图像时间戳，拒绝旧帧/重复帧；不依赖仿真灯尺寸或米制灯距。allnodes 仍保留车上 A3 的现有独立运行环境/相机配置。

## 行为

- 手柄 L2/L1 使能按住时优先，包括红灯时；速度为零也占有控制权。松开后不能复用旧自动指令。手柄失联且仍处于使能态时停车，直到明确收到释放状态。
- 自动：首个新鲜红帧先停止，稳定红锁停；稳定绿至少 3 帧、0.5 秒窗严格多数后允许新的 LYA 指令。unknown 不能解除红锁；未遇红且视觉正常时 unknown 允许循线。摄像头/semantic/LYA 输入中断时自动停车。
- motor 发出的零速度为全零载荷。非零转向公式和左右编码不变；角速度零的直行不再带入 sigmoid 偏置，左右轮补偿系数仍各自保持。
- `motion_enabled=false` 会压住手动和自动；诊断话题 `/aiformula_control/motor_controller/gate_status` 的原因是 `deployment_motion_inhibit`。

## 一圈建图

真实 odom 位姿按约 0.2m 采样；首圈达到 40m、至少 30 样本，离开起点后回到 1.5m 范围且航向差不超过 25°，才尝试闭环。订阅现有原始 ABC 评估有效性；红灯等待/OA/失效视觉段标记排除。已有 OSM 拓扑与首圈里程计融合，不是发现新道路的 SLAM，不添加自动左转。

查看 `/semantic_drive/map_status` 的 `phase/recorded_samples/recorded_distance_m/map_fusion` 和 `/semantic_drive/reference_path`。闭环后融合通过为 guarding；融合不通过可为 odom_only，不能声称已成功拓扑对齐。参考轨迹保存在进程内存，退出或 reset 会丢失；本次没有实车跑圈。

## 验证边界

策略单测、原 topo 回归、隔离 ROS_DOMAIN_ID + localhost 下的真实 rclpy/DDS/CAN 消息编码测试分开记录。所有测试 motor 输出重映射到私有测试话题，没有 socket_can_sender，也不写 can0。本次不验证真实红灯视觉准确率、路上刹停距离、底盘失联制动或真实首圈融合。既有无图像源戳的 ABC/LYA 仍只能验证接收新鲜度，不能宣称完整图像到执行器谱系。
