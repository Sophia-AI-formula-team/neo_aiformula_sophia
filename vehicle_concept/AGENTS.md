# V6.26 驾驶策略与 V6.27.1 实时实现的部署手册

**从这里接手。** 正式版本在团队 neo 的 `feat/concept-v626-live` 分支，代码在 `vehicle_concept/src`。现场先核对车机底层版本，再建独立的 ROS 2 工作区。启动命令是 `ros2 run concept live`。默认打开驾驶面板和 RViz，电机输出初始禁用。本文的离线副本仅用于备份；实验记录请标明实际运行的 Git SHA。[团队分支](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/feat/concept-v626-live) · [本手册线上入口](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/feat/concept-v626-live/vehicle_concept/AGENTS.md)

| 接手时先确认 | 目前证据 |
|---|---|
| 版本起点 | 团队 neo main `d8277bd2b4ce95bcab2883e9b68b011749c0d9ad`。本分支单独安装。 |
| 冻结策略是否接对 | 五场历史 Unity 记录共 1,079 帧。重放时路径、路界、桶、候选选择、分数和控制均与冻结版一致。含内、外圈整圈及三种局部障碍，倒车样本为零。[逐场记录](src/concept_drive/REPLAY.json) |
| 相机识别是否接对 | B1.4 在工作站 GPU 上复核 12 张实车录像图，5,529,600 个道路与标线像素判断、1,200 个检测结果与原模型一致。[对照记录](src/concept_yolop/SOURCES.md) |
| 手柄与面板 | Foxy 软件 CAN 端点实测手柄优先、松手停车、恢复只收新指令和各输入失联。Qt5 和 RViz 的话题连接已核对。[试验脚本和实测数据](checks/foxy_gate_dds.md) · [面板外观示例，使用 Unity 画面和示例数值](src/concept_panel/evidence/panel-windows.png) |
| V6.26 初版从相机到车身图的接线验证 | Foxy 中从原始 RGB 走 B1.4 CPU 推理、轮速/IMU、白盒和 H12，到 RViz 消息。目标/上限为 1.0 m/s。内圈归档的首帧两次均得到 75 条可行候选、选择索引 4、路径 21 点、地图 1,931 点，单帧驾驶计算 507 / 560 ms。从干净 Git 归档构建后，以本仓库自带的移动桶外圈首帧复验，75 条可行、选择索引 29、路径 25 点、地图 2,142 点，用时 782 ms。三次均未标定，自动输出均为零。时钟由测试控制，Jetson 在真实时钟下是否满足 0.5 秒新鲜度需要实测。[复验脚本](checks/replay_camera.py) |
| 下一位 agent 的第一项实车工作 | 查实际设备和旧启动命令，核对当前 ZED 安装与投影。确认输入、左右方向、静止轮速和驱动器断 CAN 后的表现，才进入低速自动。车载 Jetson 的推理耗时、实车制动距离、行驶方向与圈速均待测。 |

**实时性能。** V6.27.1 默认相机 15 Hz，来新图时最高请求 15 Hz 规划，验收下限为每秒 5 条新指令。工作站回放内外圈各两次，另开 Qt 面板和 RViz 重跑一次外圈，每场 60 秒。实际平均 10.08–12.67 Hz，任意连续一秒最少 7 条，最长指令间隔 188 ms。部分图像采集到指令仍超过 200 ms。Jetson 要按第 4 节用车上时钟复测。[新版逐轮报告](checks/timing/v6.27.1/README.md) · [原对照版本](checks/timing/v6.27/README.md)

## 1. 车上的接线与节点

以下是 neo 代码和配置声明的连接。现场确认 USB 设备号、相机、急停和线束实物。软件记录不能证明插头、针脚或制动机构的当前状态。[neo 话题表](../workspace/src/aiformula/launchers/config/topic_list.yaml) · [原整机启动文件](../workspace/src/aiformula/launchers/launch/allnodes.launch.py)

| 设备 | neo 配置与 Concept 的读写位置 | 现场核对 |
|---|---|---|
| ZED X 前相机 | 复用 `zed_wrapper`。HD1080、15 Hz、输出缩小 3 倍。左侧去畸变图 `/aiformula_sensing/zed_node/left_image/undistorted`，配对的 `CameraInfo` 默认 `/aiformula_sensing/zed_node/left/camera_info`。 | 查实际尺寸、时间、镜头光学帧和相机序号。neo 的车身 xacro 引用 SN48442725，而另一组在用的 nHD 内参来自 SN48797506；要用现场画面做第 3 节核验。 |
| VectorNav | 串口缺省 `/dev/ttyUSB0`、115200。原生 `vectornav` 和 `vn_sensor_msgs` 发布 `/aiformula_sensing/vectornav/imu`。使用 z 轴角速度，ROS 左转为正。 | 查设备实际指向、IMU 帧和频率。规划输入取拍摄时已经收到的轮速与陀螺。 |
| Kvaser USB-CAN | `kvaser_usb`，经典 CAN `can0`，500000 bit/s，原 socketcan receiver 发布 `/aiformula_sensing/vehicle_info`。0x711 低字节为右轮 data[0]、左轮 data[4]。轮径 0.254 m，轮距 0.60 m，传动比 1.1。 | 检查 `ip -details -statistics link show can0`。记录 CAN 错误计数、左右方向及单位。只验证过 8 位正向反馈，高位非零时状态无效。 |
| 后轮电位计、里程计、TF | 原电位计节点读取同一 CAN 的 0x11，输出 `/aiformula_sensing/rear_potentiometer/yaw`。原 gyro 里程计和车辆 TF 同时启动，便于独立检查。 | 规划和记忆用轮速与陀螺，首圈从空地图开始。 |
| DualShock 4 | 原 `joy_node`，device_id=0，20 Hz 自动重发。原 L2/R2 脚本把 `/aiformula_control/joy_node/joy` 转成 `/aiformula_control/joy_node/joy_l2_throttle`。默认模式 L2 前进、R2 后退、右摇杆转向，L1 与十字键模式切换遵照 neo。 | 确认已配对、空闲时仍有心跳。按原映射产生手动控制即切走自动；无需新增接管键。 |
| 电机命令 | `/concept/auto_cmd` → `/concept/gate` → 私有速度 → 原 `motor_controller` 0.4.1 左右轮换算 → 私有 CAN → 门控复核 → `/aiformula_control/motor_controller/reference_signal` → 原 sender → 0x210。 | 物理输出话题应只有 1 个发布者和 1 个订阅者。读取门控状态或录内部话题诊断；向物理输出话题添加 echo/录包订阅会触发所有权检查。 |
| 急停 | 独立硬件遥控保持原接线。GUI 禁用输出以及软件 `/concept/gate/estop` 是另外的操作。 | 现场确认断 CAN、断电和零轮速分别会怎样减速。软件端收到零帧并不证明机械锁轮或车已停稳。 |

## 2. 在 Jetson 建独立工作区

neo 记录的平台为 Ubuntu 20.04 aarch64、ROS 2 Foxy、L4T 35.4.1、CUDA 11.4、cuDNN 8.6、ZED SDK 4.1.4。以下命令在实际能访问 ZED/CAN 的原车 ROS 环境内执行。如果原节点在车载容器中运行，使用同一容器的设备映射和显示环境。[平台清单](../dependencies/README.md)

```bash
source /opt/ros/foxy/setup.bash
source /home/workspace/install/setup.bash
export REPO="$HOME/src/neo-concept-v626"
export WS="$HOME/concept_v626_ws"
mkdir -p "$HOME/src" "$WS"
git clone --single-branch --branch feat/concept-v626-live \
  https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git "$REPO"
git -C "$REPO" rev-parse HEAD
python3 "$REPO/vehicle_concept/checks/check_underlay.py"
```

已有同名检出时，在该目录用 `git status -sb` 查看分支和改动，用 `git fetch origin` 对照远端。核对脚本逐项比较当前安装的手柄映射、电机换算源码及接线配置。出现 `DIFF/MISSING`，先看具体差异。若车机仍安装较早的 neo 电机或启动器，可在新工作区覆盖安装本分支对应的底层包，保留原安装目录供回退。下面仅在需要覆盖时执行。

```bash
cd "$WS"
colcon build --base-paths "$REPO/workspace/src/aiformula/common/common_python"
source install/local_setup.bash
colcon build --base-paths "$REPO/workspace/src/aiformula/vehicle" \
  "$REPO/workspace/src/aiformula/launchers" "$REPO/workspace/src/aiformula/control/motor_controller"
source install/local_setup.bash
python3 "$REPO/vehicle_concept/checks/check_underlay.py"
```

安装新包前，核对车机的 SDK、CAN/IMU 驱动和显示环境。Python 需要 NumPy、SciPy、OpenCV、cv_bridge、python_qt_binding（Qt5），以及适配此 JetPack 的 PyTorch/TorchVision。TorchVision 要能在 CUDA 上执行 NMS。Foxy 系统 SciPy 1.3.3 与 NumPy 1.24 不能直接混用；Linux x86 测试组合为 NumPy 1.24.4 / SciPy 1.10.1。Jetson 需选配套的 aarch64 二进制并复测。可用 `rosdep check --from-paths "$REPO/vehicle_concept/src" --ignore-src --rosdistro foxy` 作清单核对；自建包通过 neo 底层工作区提供。不要对工作中的车机环境执行无差别升级。

```bash
cd "$WS"
# 这三个包编译 C 扩展。需要 C 编译器（含 OpenMP）、Python 开发头文件和 Cython。
# 已验证 Cython 3.0.12、SciPy 1.10.1（Python 3.8）。在当前 ROS Python 环境核对。
python3 -c 'import Cython,scipy; print(Cython.__version__, scipy.__version__)'
colcon build --base-paths "$REPO/vehicle_concept/src" --symlink-install
source install/local_setup.bash
ros2 pkg executables concept
ros2 run concept live --show-args
python3 -c 'from concept_road import _markings, _memory; from concept_candidates import _progress; from concept_feasibility import _motion, _risk, _tread; from concept_drive.core import Drive; Drive(); print("V6.26 H12 and native kernels loaded")'
python3 -c 'import torch,torchvision; from torchvision.ops import nms; b=torch.tensor([[0.,0.,2.,2.]],device="cuda"); print(torch.__version__, torchvision.__version__, nms(b,b[:,2],.5))'
```

B1.4 视觉权重大小 15,293,077 字节，SHA-256 为 `19d6eeed99fee90a46f09761746845afcf330974a8e1aff057055697a02706a6`。取自团队已授权的研究原件 `B1.4-road-aligned-research.pth`，或从现有研究工程 `workspace/concept_modules_v4/evidence/v626-frozen/V6.26.zip` 直接导入。公开 Git 分支提供导入器和完整推理源码。没有上述文件的新机器，需要向项目持有人取一份。H12 选路模型已随包安装，SHA-256 为 `e87f84dd1a1e3ef4a7df21ca9b879f9e7b142573501937e1985e74632aec0df6`。

```bash
ros2 run concept_yolop concept_yolop_weights /media/usb/V6.26.zip
ros2 run concept_yolop concept_yolop_check \
  "$REPO/vehicle_concept/src/concept_drive/test/data/moving-lap-start/00000_rgb.png" --device cuda
```

## 3. 在停车位标定前视相机

1. 在车前平地布置已量过位置的标记。坐标以车辆规划原点（`base_footprint` 在地面的原点）为基准，记录向右、向前的米数。包括 3 m 内及 7 m 外、车辆左右各 0.5 m 以外。先在原相机运行时取样，或按第 4 节仅开 Concept 观察，保持电机禁用。
2. 执行下方的 `--capture` 命令。它等待同一时间戳、同一光学帧的相机图和 CameraInfo，保存 `camera.png` 和包含真实尺寸、投影矩阵、帧名的 `camera.json`。目录名每次选新的。相机话题不同可传 `--camera-info-topic` 并在启动时传相同的 `camera_info_topic:=...`。
3. 在原图上量每个标记的落地点像素。JSON 每行填 `[像素u, 像素v, 向右米, 向前米]`。用至少 4 个 `fit_marks` 将实际镜头的观察方向对齐冻结相机；另用至少 4 个不同的 `ground_marks` 验证。两组都覆盖远近、左右。若安装本就匹配冻结视角，可移除 `fit_marks` 字段，直接用独立地面点核对。未知图像边缘会从校正后的分割与桶落点中剔除。
4. 所有验证点的地面位置误差须在 0.10 m 内。若无法达标，复核测量原点、时间、相机固定位置和镜头高度；旋转校正不能补偿显著的安装位置偏移。重新定位安装或补做经过验证的几何适配后再上车。记录版本、标记原图与误差。没有合格登记时仍可看识别和规划，驾驶面板显示影子模式。

```bash
ros2 run concept_yolop concept_yolop_calibrate \
  --capture "$HOME/concept_calibration/run01" --mount-id run01
# 填写测量值后运行
ros2 run concept_yolop concept_yolop_calibrate --check "$HOME/concept_calibration/run01/camera.json"
```

## 4. 启动、查看决策、接管和恢复

先记录原方案的启动命令、当前 `ros2 node list`、ROS_DOMAIN_ID，以及第 2 节的 SHA。使用原系统的 `aiformula-init-sensors.service` 或原传感器初始化流程配置 can0 和串口。查看 `systemctl cat aiformula-init-sensors.service`、`ip -details link show can0` 和 `ls -l /dev/ttyUSB0`。停止已经在运行的旧 allnodes、LYA 和其它向电机发命令的进程，让出传感器、手柄和 CAN。确认同一块 CAN 网卡没有被另一容器、ROS 域或非 ROS 程序写入。启动前的图检查只能看到当前 DDS 域。

```bash
source /opt/ros/foxy/setup.bash
source /home/workspace/install/setup.bash
source "$HOME/concept_v626_ws/install/local_setup.bash"
ros2 run concept live calibration:="$HOME/concept_calibration/run01/camera.json"
# 等价长命令是 ros2 launch concept concept.launch.py calibration:=...
```

首次仅观察可省略 `calibration:=...`。默认目标速度和自动速度上限均为 1.0 m/s。RViz 与 Qt 面板各自有窗口；车载桌面使用当前会话的 DISPLAY。无显示时传 `panel:=false rviz:=false`，在已接入同一 ROS 域的显示终端另开 `ros2 run concept_panel panel` 或 `rviz2 -d "$(ros2 pkg prefix concept_visualization)/share/concept_visualization/rviz/concept.rviz"`。要单测识别，可单独 `ros2 launch concept_yolop concept_yolop.launch.py`，整机启动已含此节点。

Foxy 的 Fast DDS 默认共享内存段在本次环境约为 545 kB，一张 640×360 BGR 图为 691,200 字节。发现节点却收不到图时，先查传输设置和容器的 `df -h /dev/shm`。在环境未指定 `FASTRTPS_DEFAULT_PROFILES_FILE` 时，整机 launch 为子进程选择随包 XML。车端 `vehicle-fastdds.xml` 使用 2 MiB 共享内存段及 UDPv4；同机隔离回放 `foxy-fastdds.xml` 仅用共享内存。单独开的面板、RViz、图像发布器或测试终端应加载对应模式的设置，例如 `export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix concept)/share/concept/config/vehicle-fastdds.xml"`。若现场已有 DDS 配置，核对里面的传输和图像容量。本次没有测场地无线网络的延迟。

| 观察项 | 读数和图怎样理解 |
|---|---|
| 面板 | 显示自动/手柄/暂停、当前输出、决策、可行数、选中的轨迹和相机叠图。相同事件按来源合并成一行，记录首次、最近时间和次数。若门控消息过期，面板显示失联。 |
| RViz 相机 | `/concept/view/camera`。蓝色为当前车道，橙色为标线或边缘，粉色为这张图对应的已选轨迹。尚未得到同时间戳决策的画面可以先显示识别。 |
| RViz 车前与记忆 | 固定帧 `base_footprint`。局部图 `/concept/view/local` 用浅绿色显示通过筛查的候选、灰色显示淘汰的候选、粉色标出最终一条。记忆图 `/concept/view/map` 分色表示本车道与邻道，实虚线均为橙色；已确认静止桶标点，移动桶显示未来中心位置的不确定范围。图随车身坐标更新。 |
| 调速 | 面板把「规划目标」发给 `/concept/drive`，把「自动上限」发给 `/concept/gate`，有效目标取较小值。改动上限会暂停自动，确认后点「继续」。目标值是驾驶请求，实际速度看轮速。 |
| 动作 | 开启输出后仍处于暂停。手柄居中、有心跳、状态有效时点击「继续运行」。手柄一旦接管，自动命令被切断；视觉和选路继续。松手立即要求零轮速，保持到下一次点击「继续」，恢复前产生的指令全部作废。禁用输出覆盖手动和自动。 |
| 停滞重启 | 先读原因。在面板点重启，面板先等门控确认暂停，再请求驾驶节点清空路界、桶和时序历史，重新接收传感器。等待新输入后手动继续。若驾驶进程退出，launch 尝试重启进程；门控超时后仍需手动恢复。服务未应答时按第 6 节处理。 |

静止检查时可运行下列只读命令。控制门物理所有权检查针对发布者、订阅者和节点名，重复的电机节点会阻止启动或启用。

```bash
ros2 node list
ros2 lifecycle get /aiformula_control/socket_can_sender
ros2 lifecycle get /aiformula_sensing/socket_can_receiver
ros2 topic info -v /aiformula_control/motor_controller/reference_signal
ros2 topic hz /aiformula_sensing/zed_node/left_image/undistorted
ros2 topic hz /aiformula_control/joy_node/joy_l2_throttle
ros2 topic echo /concept/vehicle_state
ros2 topic echo /concept/gate/status
python3 "$REPO/vehicle_concept/checks/measure_timing.py" --seconds 30
```

默认 ZED 采集 15 Hz，规划对新图最高请求 15 Hz。要复现原来的抽帧档，启动时显式传 `planning_hz:=5.0`。测速工具继续按 5 Hz 的最低服务要求统计 200 ms 时间窗。用于调参时先禁用电机输出，核对相机时钟，然后测 60 秒。指标看实际收到的新 `/concept/auto_cmd`、两条指令的间隔、采集至指令的 P95 和最大值，同时检查 200 ms 时间窗中是否有空窗。界面或 CAN 重发同一个指令不能代表有了新决策。相机未标定时驾驶节点输出 shadow，可以定位耗时，此时真实新指令数为零。[测速字段与车端命令](checks/measure_timing.md)

V6.26 初版白盒单帧中位约 563 ms，H12 约 0.9 ms。V6.27 已把 75 条候选轨迹的车轮边界、停车距离及障碍风险检查成批计算，保留相同筛选规则。五场 1,079 帧回放的白盒和 H12 选择、可行标记、选中路径和输出指令均与冻结结果一致。V6.27.1 进一步合并完全相同的预测轨迹，并对障碍物风险做有误差上界的快速筛选。五场共 1,500 个 200 ms 窗口均收到新指令。车端先做观测、标定与台架验收。[最新冻结回归](checks/timing/v6.27.1/frozen-regression.json) · [最新工作站时钟回放](checks/timing/v6.27.1/README.md)

在现场负责人安排下，架空车轮确认零输出、左右转向、手柄松开保持和硬件急停；按原接线核对实际车轮方向。随后可直接进行短距离低速自动测试，记录距离、用时、最低帧率、拍摄到控制的延迟和停车距离。无需先跑一圈手操。5 m/s 属于仿真压力测试条件；目前已解码的轮速只有 0–255 rpm，轮径 0.254 m 时约到 3.39 m/s。验证方向位和高位之前，不用这路反馈做更高速度或自主倒车的实车验收。

## 5. 从图像到轮速，修改时找哪个包

线上交付含 12 个 ROS 包。部署目录约 1.4 MB（含 5 张 Unity 回归图、预期结果和面板示例），另加约 15.3 MB 的 B1.4 私有权重。ROS、CUDA、Torch、ZED SDK 和原 neo 底层依赖没有计入。

| 功能 | 包和接口 |
|---|---|
| 编排、设备启动 | `concept`。实际设备模式使用 neo 已安装的 ZED、CAN 生命周期、VectorNav、手柄和车身 TF。 |
| 识别 | `concept_yolop`。B1.4 一次给出本车道路面、标线和物体。发布同时间戳的 `/concept/perception/frame`。 |
| 车辆反馈 | `concept_vehicle_state`。从 CAN 与 IMU 配对出左右轮速和角速度；无效、未来或过期样本给出原因。 |
| 道路及记忆、桶 | `concept_road` 结合路面与标线，靠轮速和陀螺更新车身附近的地图；`concept_obstacles` 合并重复检测，区分静止和可能运动的桶。 |
| 预测与筛选 | `concept_candidates` 生成动作；`concept_feasibility` 根据车辆响应预测占地、边界与避障风险。车道之间允许跨线，支路不可驶入，路界按一侧轮胎完全越出判断。具体不确定度仍会影响可选方案。 |
| 模型选择 | `concept_ranker` 的 4 个宽度为 12 的网络对允许的前进候选给分数修正。正常驾驶按最终得分选择，特定恢复阶段走冻结的白盒逻辑。 |
| 控制 | `concept_drive` 按 5 Hz 的冻结节奏推进，选中的速度、转向直接形成 `TwistStamped`。车上复用 neo 电机换算和补偿。此链路没有 LYA 或独立 P follower 节点。[同一张图的真实评分例子](src/concept_drive/README.md) |
| 输出与显示 | `concept_control_gate` 仲裁唯一最终输出，`concept_visualization` 发布路径和地图，`concept_panel` 负责 GUI。ROS 中左转为正，冻结规划内部向右为正，边界只转换一次。 |

## 6. 复验、排错和回退

在开发机复现原图到 RViz 的整链，两边先 source 同一个底层与 Concept 安装环境，在两个终端均设置 `export ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1` 和 `export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix concept)/share/concept/config/foxy-fastdds.xml"`。第一个终端运行 `ros2 run concept live mode:=replay device:=cpu use_sim_time:=true panel:=false rviz:=false`。第二个终端运行：

```bash
python3 "$REPO/vehicle_concept/checks/replay_camera.py" \
  "$REPO/vehicle_concept/src/concept_drive/test/data/moving-lap-start/00000_rgb.png"
```

这个脚本发布原始图像和模拟轮速/IMU，检查同一采集时间戳的摄像头叠图、实际选择、局部路径、地图和隔离状态。软件端跨机模式另用两个独立网络地址和 IPC 空间的 Foxy 容器验证，车端 XML 经 UDP 连续传过三张 691,200 字节原图，接收端逐张核对 SHA-256 一致。

- **有相机图，没有路径。** 查 `/concept/vehicle_state` 中的 `valid/reason`，确认真实 IMU 与轮速时间均在图像之前。查 `/concept/decision` 的 `reason`、`capture_age_s`、`processing_ms`。大消息只在需要时短暂读取。没有有效车况、图像年龄大于 0.5 s、光学帧切换或计算异常时会抑制该帧自动输出。先恢复输入；面板若提示重新开始，暂停后重启驾驶状态。
- **能看到路线却不动。** 查几何状态、门控使能、暂停、空闲手柄心跳和目标上限。有路线的 `shadow` 只用于观察。若出现 `graph_conflict`，核对最终 CAN 话题的收发者，以及是否有额外录包/echo 订阅。停止冲突源后重新启用输出。自主倒车还缺有符号实车轮速验证。
- **识别节点报错。** 查权重 SHA、GPU NMS、相机宽高比与节点解释器。开始时用第 2 节的冻结真图运行一次，再测实际采集时刻到决策的耗时。模型版本、安装位置、推理设备、源图变化都应单独记录。
- **离线复验。** 在开发机独立容器中使用 `ROS_LOCALHOST_ONLY=1` 和空闲的 `ROS_DOMAIN_ID`。启动 `ros2 run concept live mode:=replay device:=cpu` 可用软件终端接收内部命令，跳过相机/串口/真实 CAN 驱动。开发机也须有相应的模型和兼容依赖。重放旧录像时所有节点要用同一时钟；Foxy 的 `ros2 bag play` 没有生成时钟的 `--clock` 选项。使用带 `/clock` 的录制或独立提供同步时钟；只回放原始相机、CameraInfo、IMU、反馈 CAN 和需要的手柄输入。不要整袋回放历史输出。参考 [Foxy 门控实验](checks/foxy_gate_dds.md)、[驾驶 DDS 实验](src/concept_drive/tools/ros_roundtrip.py)。
- **回退。** 在面板禁用输出，确认停稳后在启动终端 Ctrl+C 停 Concept。必要时使用硬件遥控急停。确认 `/concept/gate` 和 Concept 启动的 CAN 进程退出。打开干净的新终端，只加载 `/opt/ros/foxy/setup.bash` 与原 `/home/workspace/install/setup.bash`，运行事先记录的原启动命令。原基础入口为 `ros2 launch launchers allnodes.launch.py`；若原本另开 LYA，按该车记录只开一个。仓库有不同年代的 LYA 入口，切回时应使用现场记录的那个。原 init/systemd 保持原样。[原车运行说明](../workspace/src/aiformula/control/semantic_vehicle_wiring/README.md)
- **实验交回。** 记录分支 SHA、软件/模型版本、标定文件 SHA、启动参数、`/concept/gate/status`、`/concept/decision`、输入频率、丢帧与停车表现。区分桌面、ROS 软件链、架空台架和实际落地结果。现场日志与原始图像按团队原有权限保管。

离线备份按线上提交号取同一份文件即可。离线打开时，前文安装、接线、核验和回退步骤仍可直接阅读。
