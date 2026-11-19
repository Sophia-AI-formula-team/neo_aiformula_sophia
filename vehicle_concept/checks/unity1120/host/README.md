# Unity 与 Foxy 的车辆闭环

本工具从一台独立的 Unity 编辑器发出前视图、双轮 RPM 和车身角速度。Foxy 对这一帧推理，返回带帧号的速度、角速度及左右轮 CAN 原始整数。Unity 把两个整数交给电机模型，推进 20 个 0.01 秒物理步，再拍下一帧。圈形、车身世界位置和车轮是否出界只供 Unity 内部布置起点和裁判计分。

## 准备预览实例

本机使用 [aiformula_concept](https://github.com/Tsubashimo-Nanato/aiformula_concept) 的 `workspace/concept_modules_v4` 工作区及其已安装模拟工程。编辑器版本为 Unity 2022.3.22f1。仿真工作区的 Git 基线为 `6db0146c07c87a6775d14c37176ceda3555404ed`，车端宿主补丁基于 neo `70209be`。目前需要从团队已有的私有仿真工作区单独迁移 V6.26 实验源目录 `.runtime/v626_time/eval-inner-check/solver`，在新机器用 `--referee-source 路径` 指向该副本。按文件名顺序拼接九份 C# 规范化文本的 SHA256 为 `46a827df6272e4b6e2806d155c81149c6cd5ba4293d9860bba1b8639281bb934`。运行器核对这一哈希，在本地输出目录内组装车端输入；每次运行保留构建用源件、DLL 和场景哈希。这些私有构建产物留在本机。Unity 自身和加速设备依赖由安装好的模拟工程提供。

在新电脑准备好上述工程、Unity 编辑器及 Concept 实验所需的 Python 依赖后，从 **Concept 仿真仓库根目录**新建专属实例。下面示例中的 `simulation` 目录内需要有 `Simulator/Assets`，如果源项目在别处，将参数换为其上一级目录。该命令需要可用的图形后端；Linux 请按仿真仓库的迁移手册配置 Unity 和显示环境。

```sh
python research/learned_racing/unity_laps/isolated.py --sim simulation --copy .runtime/UnityLab-v623-02
```

已有私有实例时，核对 `.runtime/UnityLab-v623-02/LabQueue/owner.json` 中的项目、源工程、PID 及该 PID 的启动参数。等待 `ready`，确认没有别的实验使用该实例。程序拒绝使用其他名称、其他工程或已有申请的队列。Windows 实测使用 D3D11 batchmode。Linux 原生编辑器的闭环仍待验证。

## 连接容器和运行

先启动同目录上层的 Foxy 端，让主机回环地址 `127.0.0.1:16826` 可以连到容器。不要把端口映射到车上的 ROS、CAN 或局域网。切到 **neo 仓库根目录**。以下路径按本机两个相邻的工作树举例。

```sh
python -B vehicle_concept/checks/unity1120/host/run.py --concept ../concept_modules_v4 --out work/straight01 --start 0.87 --offset 0 --heading 0 --seconds 5
python -B vehicle_concept/checks/unity1120/host/run.py --concept ../concept_modules_v4 --out work/outer_edge01 --start 0.87 --offset 0.65 --heading 12 --seconds 5
```

输出目录必须是新目录。`--start` 是仅用于布置的赛道里程比例，`--offset` 是相对外车道中心向右的米数，`--heading` 是相对道路切线向右的角度。可用 `--compile-only` 只构建实验 DLL。

## 图像、物理和时钟

- Unity 在每次运行的预览场景中重新放置左相机，用实际地面碰撞测量 0.55 米离地高度。安装位置在车基前 0.055 米、左 0.0600025 米，视轴上仰 1 度。640 × 360 的整流图采用 11 月 19 日保存的内参 `fx=fy=246.0857544, cx=328.6564941, cy=189.0118256`。原始图由 Unity 编为 JPEG（质量 95），相同字节保存并送 Foxy。Honda 地面投影变换由 Foxy 的相机节点执行一次。
- Unity 直接使用返回的 `right_can_rpm, left_can_rpm`。场景自带的 `Archived1030` 速度换算器在此路径旁路。电机和轮胎响应仍来自 `JanuaryLapIdentified` 仿真模型。双轮 RPM 是模拟电机和轮胎模型的转速代理，gyro 是刚体实时角速度；ROS 左转为正，故其值等于负的 Unity 绕 Y 轴角速度。
- 每 0.2 模拟秒拍一帧。仿真等 Foxy 回复再推进物理，适于检验指令能否让下一帧画面、轮速和转向实际变化。实际 5 Hz 或端到端截止时间需另做自由运行测量。每一帧另存 Windows 宿主 UTC 微秒时间以定位编码、往返和写入 CAN 的耗时。容器端使用自己的单调时钟，避免跨机器时钟相减。
- 没有连接、命令缺字段、错帧、收包超时或命令断流时，回合记录错误，左右模拟 CAN 清零，预览场景被关闭。Unity 在等待回复时不推进物理。结束时只读回此预览场景的零指令，原工程和其他 Unity 实例保持独立。

## 如何核对

每次运行看 `mount.json` 核实相机离地和电机，`camera_*.jpg` 为所有真实渲染原图，`exchange.jsonl` 每行保存图像信息、Foxy 回包、上帧 CAN 读回、写入 CAN 读回、轮速、实际 gyro 与车身前速。`result.json` 是独立裁判的逐步位置、进度、轮胎和终止原因。`release.json` 记录结束时左右 CAN 的实际读回。`host.json` 记录源项目、Unity 版本所在路径、场景及实际程序集的哈希。各场次的状态 `duration` 仅表示达到设定测试时长，整圈需单独验证。
