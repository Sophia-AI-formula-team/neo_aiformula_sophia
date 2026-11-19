# Concept V6.27.5 Jetson 安装记录

日期为 2026-11-19 JST。来源为 Neo `feat/concept-v626-live` 的 `a9d7d7a94d062ad5ff61fedfe21d3dbe94fa1bd1`。部署包为 V6.27.5，自带 V6.26 驾驶策略。按该提交的 [Concept 车端部署手册](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/a9d7d7a94d062ad5ff61fedfe21d3dbe94fa1bd1/vehicle_concept/AGENTS.md) 第 2 节安装。

- Jetson 源码为 `~/src/neo-concept-v6275`，独立工作区为 `~/concept_v6275_ws`。
- 原车安装用 `/home/workspace/install`。底层核对唯一发现的差异是此前实验将原 `allnodes.launch.py` 的 A3 默认值置为 false。分支原版 `launchers` 另装在 Concept 覆盖层。原车安装及当天 YOLOP 选择器继续使用其原路径。
- 独立 Foxy Python 环境使用 NumPy 1.24.4、SciPy 1.10.1、Cython 3.0.12，并复用原车 NVIDIA Torch 2.0.0+nv23.05 与 TorchVision 0.15.1 的二进制。仅在 Concept 环境预加载 libgomp 和 libGLdispatch，修复 aarch64 Qt/Torch 的静态 TLS 初始化。
- 底层文件逐项比对、CUDA NMS、cv_bridge 和离屏 Qt 面板检查已通过。软件验证未发送车辆控制指令。

10:16:43 JST 完成 12 个 ROS 包的编译安装。10:16:48 通过路径和版本检查，验证了原生道路扫描、精确候选合并、六个编译扩展、冻结的 H12 模型、默认 15 Hz 规划参数和 ROS bag 工具。10:16:56 车辆传感器启动图静态检查通过，并通过 7 项控制门与冻结驾驶策略的离线回归。原车 `allnodes.launch.py` SHA-256 与今天此前部署的版本一致。

检查命令可在新的 Bash 终端中执行。

```bash
source ~/concept_v6275_ws/activate_concept.bash
python3 ~/src/neo-concept-v6275/vehicle_concept/checks/check_overlay.py ~/concept_v6275_ws
ros2 run concept live --show-args
```

B1.4 私有权重由项目单独分发。预期文件名为 `B1.4-road-aligned-research.pth`，大小 15,293,077 字节，SHA-256 为 `19d6eeed99fee90a46f09761746845afcf330974a8e1aff057055697a02706a6`。官方导入器也可读取 `V6.26.zip` 内的该文件。

10:18:47 JST，已从本机下载目录传输官方 B1.4 权重，经传输前后及导入后 SHA-256 验证，使用项目导入器安装到车端默认私有模型目录。10:18:57，随包测试图在 Jetson 上通过 CUDA 推理，同时输出道路、车道与 100 个检测条目。

10:19:23，项目自带的隔离摄影机回放在车端通过。冻结策略生成 75 条可行候选，选择索引 29 并输出 25 点路径和 2,142 个地图点。未标定时状态为 `shadow`，自动指令为 0；CAN 使用软件接收端。10:19:43 确认回放进程退出后，开始真实 ZED 的识别验证。

10:20:03，在独立 localhost ROS 域中验证真实 ZED 至 B1.4 CUDA 识别，确认 640×360 BGRA 输入及尺寸匹配的 CameraInfo。收到 12 帧感知结果与 12 帧 JPEG 叠图，12 帧感知均匹配源相机时间戳和帧名；只保存计数日志。10:20:15 确认测试相机与识别进程退出。

## 现有 ZED 几何复核

Neo 和车端都已有实际相机 SN48797506 的 640×360 nHD 内参，两个文件 SHA-256 一致。车端 SDK 的 SN48797506 参数文件也在。车身 xacro 仍引用 SN48442725 的安装外参；历史 A3 交接明确记录更换内参时保留了原车身外参。

10:21 JST 使用 Concept 原生工具采集了同时间戳的一张图像和 CameraInfo。实时焦距为 245.857 px，Neo 现有内参为 246.071 px，相差约 0.214 px。Concept 按实际投影矩阵比对，每项绝对容差为 0.001；草稿保留本次采集的矩阵。比较值见 [内参核对](calibration-intrinsics-check.json)。

已有内参可用于确认相机身份和图像规格。公开分支和车端均未找到这台车在当前安装状态下的独立地面测点回执。已在车端准备单帧及配套草稿，地面测点留空。通过 Concept 手册的 0.10 m 独立地面误差检查后才可启用该回执。实车自动驾驶输出仍未验收。

现场反馈当前相机高度为 **0.55 m**。暂按镜头光心离地高度记录，其测量基准待确认。Concept V6.26 的参考投影高度为 **0.655 m**。若这两个值均以光心到地面计，且视线角度已经相同，直接套参考高度会将同一条地面视线的距离放大约 19.1%。仍需用独立地面点检验当前安装的投影。车端原标定、软件及驾驶授权状态保持原样。

10:39:34 JST 已安装 `~/.local/bin/concept-live`。车载桌面终端输入 `concept-live` 即可先加载专用环境再启动 Concept。SSH 无界面用 `concept-live panel:=false rviz:=false`。快捷方式的环境加载、参数传递和退出状态用替身命令确认，本次未启动 Concept。回退时可删除该快捷命令。源码见 [concept-live](concept-live)。

10:40:30 JST 已在车载桌面安装 [Concept 快捷方式](Concept.desktop)。双击会在终端执行 `concept-live`，打开驾驶面板与 RViz。已验证桌面条目语法、执行权限和目标路径，未启动 Concept。

详细时间戳见 [部署时间线](timeline.log)。随今日实验材料汇总发布。

11:30:14 JST 按现场要求调用输出禁用服务并收到 `output_disabled`，随后中断 Concept。11:30:34 确认驾驶面板、相机、识别、电机和 CAN 进程全部退出。

12:38 JST 重连车端后，对 10:59–11:03 JST 的实车录包做了只读排查。规划入口因图像超时拒绝了全部 288 帧，未发布自动指令。[实车自动输出排查](2026-11-19-concept-no-auto-output.md)
