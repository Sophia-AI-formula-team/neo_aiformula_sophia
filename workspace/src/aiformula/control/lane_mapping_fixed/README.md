# 第一圈建图，之后关闭 LYA 跑固定路线

这是 `lane_mapping_lya_reference` 的**固定路线独立运行变体**，不是另一套建图算法。它依赖参考包里的共享建图、路线生成、控制和日志实现；固定包的入口强制 `fixed_only`，不能通过改 YAML 偷换为参考模式。

第一圈仍由受管 LYA 开车；建图始终使用**完整 mask + VectorNav**，不使用 LYA 命令推导地图。停车生成并检查固定路线后，先停止本次 supervisor 启动的 LYA 进程组，再允许沿固定路线运行。第二圈定位与路线异常就停，不回退到 LYA。

## 编译和前提

目标：Ubuntu 20.04、ROS 2 Foxy、Python 3.8。两个包放在 `neo_aiformula_sophia/workspace/src/aiformula/control/`，与已有 `trajectory_follower` 同级。按[参考包 README 的 clone/受限 colcon 命令](../lane_mapping_lya_reference/README.md#clone-和编译)同时安装两个包和 `dependencies/vectornav/vectornav_msgs`；已安装原生 VectorNav 消息包时可省略它的源码构建。

neo 基线的两个集成阻塞同样适用：默认 LYA 的 `4.0 m/s / 0.45 rad/s` 输出范围超过本包教师准入范围，road detector 的 mask 输出缺少原始相机 `frame_id`。本次迁移不修改控制门限、LYA 或 detector；参见[参考包部署阻塞说明](../lane_mapping_lya_reference/README.md#neo-基线的已知部署阻塞)。frame override 只能依据实测标定，不能猜值。编译成功不是实车可运行证明。

```bash
source /opt/ros/foxy/setup.bash
cd ~/lane_learning_ws
source install/setup.bash
ros2 launch lane_mapping_fixed fixed.launch.py --show-args
```

车上的相机、完整 mask、CameraInfo、静态相机 TF、VectorNav 和现有 LYA 输入链路应已工作。本 launch **不启动传感器、CAN 或电机**。不要同时启动参考版、固定版或第二个直发电机的 LYA。

## 正常流程

```bash
ros2 launch lane_mapping_fixed fixed.launch.py
```

默认只发 `/lane_learning/cmd_vel`，与电机隔离，且启动即 `DISARMED`。这适合先观察数据和输出，不代表车会自动跑第一圈。

另一个已 source 终端按顺序执行：

```bash
# 定位与私有 LYA 指令新鲜后，第一圈选择 LYA。
ros2 service call /lane_fixed_follower/arm std_srvs/srv/Trigger '{}'

# 回到起点后先 HOLD，并实际确认车已停稳。
ros2 service call /lane_fixed_follower/stop std_srvs/srv/Trigger '{}'

# 结束地图并生成路线；服务成功返回 bundle.json 路径。
ros2 service call /lane_lap_recorder/finish_lap std_srvs/srv/Trigger '{}'

# 第一次仅加载路线、保持 HOLD，并请求关闭受管 LYA。
ros2 service call /lane_fixed_follower/start_repeat std_srvs/srv/Trigger '{}'
```

检查路线、车辆位置、`/lane_learning/control_state` 与 `/lane_learning/teacher_state`。必须等 supervisor 确认自己启动的 LYA 进程组已退出、新原点下的定位新鲜、车辆持续停稳后，再次调用：

```bash
ros2 service call /lane_fixed_follower/start_repeat std_srvs/srv/Trigger '{}'
```

成功才进入 `REPEAT`。关闭 LYA 并不等于关闭 camera/VectorNav；固定路线跟踪仍必须持续收到定位，且当前完整 mask 必须持续与冻结车道地图可信匹配。LYA 停机确认缺失/过期、路线校验不通过、定位或 mask 断流、匹配拒绝、异常位置/航向跳变、偏差过大、命令竞争或日志故障都不能放行。

原来在别处启动的 LYA 不归这个 supervisor 管；它不会搜索并杀掉其他进程。不要手工声称一个无关进程已关闭。固定交接后不会自动恢复/重启 LYA；需要重新开始第一圈时，停止这次 launch，再明确启动新运行。

已有 bundle 可不重新建图：

```bash
ros2 launch lane_mapping_fixed fixed.launch.py \
  record:=false route_bundle_path:=/绝对路径/finished_xxx/bundle.json
```

仍需两次 `start_repeat`：第一次加载并停受管 LYA，第二次在停机与停稳检查通过后启用。默认 `manage_teacher:=true`；不要为了跳过确认而关闭 supervisor 或修改安全参数。

## 手操急停与真实电机输出

保留手操硬急停，且先实测它能独立切断驱动。软件 `stop`/`estop` 发零命令，不是硬件断电证明。现有电机侧仿射修正可能把零指令变成非零；必须确认/修复零指令旁路，并确认电机端断流停车。本包不会替你改电机控制器。

只有完成上述验证，才可显式连接真实话题：

```bash
ros2 launch lane_mapping_fixed fixed.launch.py \
  command_output_topic:=/aiformula_control/game_pad/cmd_vel \
  enable_vehicle_output:=true \
  motor_zero_passthrough_verified:=true \
  hardware_stop_verified:=true
```

三个 `true` 是操作员确认，不是自动检测。默认仍限速 `0.8 m/s`、限角速度 `0.4 rad/s`；没有理由把它们当作已经通过实车验收。

最终选定的行驶输出统一限制速度/角速度变化率；HOLD/ESTOP 直接输出零，不执行缓慢停车 ramp。零指令能否真正使车停止，仍需电机端验证。

```bash
# 软件锁定急停；物理手操急停仍必须可用。
ros2 service call /lane_fixed_follower/estop std_srvs/srv/Trigger '{}'
```

True 的 `/lane_learning/emergency_stop` Bool 也能锁定软件急停。解除输入后不会自动恢复；需安全确认、`reset_estop` 和显式重新启动。

## 数据、参数、可见结果

- `config/learning.yaml` 是固定版覆盖参数，其余有界地图与几何默认来自共享源码；支持 `params_file:=...`。
- mask 默认 `/aiformula_perception/pub_mask_image`；VectorNav 默认 `/aiformula_sensing/vectornav/raw/common`；base frame 默认 `base_footprint`。话题、相机 frame 和 `yaw_offset_rad` 可在 launch 覆盖，录制/跟踪使用同一偏移。
- 地图保存同一 LLA 原点和标定；第二圈以 VectorNav 为初值，使用 mask 对参考车道地图的匹配修正可观测的局部偏差。直道沿线位置仍依赖 VectorNav，不保证全局重定位；当前零平移杆臂假设及车宽、安全边距必须现场核查。
- 通过匹配的内点进入独立的有界多帧降噪层。参考地图、原 bundle 与固定路线本轮不变，防止地图跟着错误定位一起漂移。定位失效直接 HOLD，不回退到 LYA；恢复也需显式重新启动。
- RViz 用 `lane_map`：查看 `/lane_lap_recorder/consensus`、`/lane_lap_recorder/trajectory`、`/lane_fixed_follower/route` 和 `/lane_fixed_follower/pose`。
- 每次新日志在 `~/.ros/lane_learning/` 的唯一目录中；地图/route/bundle 不覆盖，command 与 teacher 日志按大小轮换。包括原始轨迹、mask 配对时间戳、模型校验、指令选择、定位时效和停机过程。

完整输入约定、日志字段、安全限制、台架验收和编译命令见[参考包说明](../lane_mapping_lya_reference/README.md)。程序可以编译启动，不等于已证明你的急停接线、零轮速、赛道定位精度或实车安全。

车端有 RViz2 时，可直接使用共享固定尺度配置（CI 不测试 GUI）：

```bash
rviz2 -d "$(ros2 pkg prefix lane_mapping_lya_reference)/share/lane_mapping_lya_reference/rviz/learning.rviz"
```
