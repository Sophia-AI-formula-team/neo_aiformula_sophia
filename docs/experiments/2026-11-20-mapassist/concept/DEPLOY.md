# 11 月 20 日 · Concept V6.27.6 实验准备

车端源码、独立 Unity/Foxy 仿真入口和复核脚本都在团队 neo。V6.27.6 完成了 11/19 原车录包重放和四场 Unity 短闭环。朝外偏 23° 的场次停在道内。实车整圈、边缘恢复、Jetson 实际发令频率尚待验收。[四场实测与实车回正图](README.md) · [车端源码](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/5511be2fc48299180a4c8287c179588f81730449/vehicle_concept) · [路界修复记录](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/5511be2fc48299180a4c8287c179588f81730449/vehicle_concept/checks/field_20261119.md)

## 车机上建立独立工作区

在原车的 Ubuntu 20.04 / ROS 2 Foxy 环境中打开新终端。以下命令用于新目录。先加载现场既有的 ROS 底层，保留旧工作区、init 和快捷方式。核对脚本会比较手柄、电机换算和底层接线的实际安装文件；遇到差异请先记下包路径和文件，再与 11/19 车机安装记录核对。

```bash
set -e
source /opt/ros/foxy/setup.bash
source /home/workspace/install/setup.bash
concept_repo="$HOME/src/neo-concept-1120"
concept_ws="$HOME/concept_1120_ws"
git clone --filter=blob:none --sparse --single-branch -b experiment/concept-foxy-unity-20261120 \
  https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git "$concept_repo"
git -C "$concept_repo" sparse-checkout set vehicle_concept workspace/src/aiformula
git -C "$concept_repo" checkout --detach 5511be2fc48299180a4c8287c179588f81730449
python3 "$concept_repo/vehicle_concept/checks/check_underlay.py"
mkdir -p "$concept_ws"
cd "$concept_ws"
python3 -c 'import Cython, scipy; print(Cython.__version__, scipy.__version__)'
colcon build --base-paths "$concept_repo/vehicle_concept/src"
source install/local_setup.bash
python3 "$concept_repo/vehicle_concept/checks/check_overlay.py" "$concept_ws"
```

核对输出应列出 12 个包。启动、驾驶、路界、候选、障碍、排序、面板七个包显示 `6.27.6`，六个原生扩展从当前工作区加载。Cython 3.0.12、SciPy 1.10.1 是此次 Foxy 工作站使用的版本。Jetson 使用本机兼容的 Torch 和 TorchVision；桌面 CUDA 11.8 的 x86 wheel 只用于仿真。

## 核对 B1.4、GPU 和传感器

若车上已有 B1.4，可复用同一文件。默认路径为 `${XDG_DATA_HOME:-$HOME/.local/share}/concept_yolop/B1.4-road-aligned-research.pth`。若目录中没有它，先从团队模型归档或实车备份取回。权重大小 **15,293,077 字节**，SHA-256 为 `19d6eeed99fee90a46f09761746845afcf330974a8e1aff057055697a02706a6`。H12 随源码安装。

```bash
# 首次导入时，将路径改成已有的 B1.4 权重或 V6.26.zip
ros2 run concept_yolop concept_yolop_weights /path/to/B1.4-road-aligned-research.pth
ros2 run concept_yolop concept_yolop_check \
  "$concept_repo/vehicle_concept/src/concept_drive/test/data/moving-lap-start/00000_rgb.png" --device cuda
ros2 run concept live --show-args
```

现场软件启动入口是 `ros2 run concept live`。在车静止、旧控制进程已退出时检查相机图像与 CameraInfo、双轮转速、陀螺和手柄心跳。启动后电机输出默认禁用。沿用现场匹配的相机配置。光心距路面约 0.54–0.55 m；还需匹配图像尺寸、内参、光轴方向及帧名。手柄有操作即接管，松开停住，面板点“继续”才恢复自动。结束时先禁用输出、退出本次 launch，再用干净终端启动原方案。[传感器接线、话题和故障处理](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/5511be2fc48299180a4c8287c179588f81730449/vehicle_concept/AGENTS.md)

## Unity 对照环境

桌面选 Concept 自己的 UnityLab 实例。四场闭环以 semantic 共用的 `vehicle-semantic:1120` 为基础镜像（ID `sha256:3fb88ac063248c0c78f21f115544d86994b754c5a286ca954bb1c872ebca8997`）。其上使用 `/vision` 中的 Torch 2.0.0+cu118、TorchVision 0.15.1+cu118、NumPy 1.24.4，另加 SciPy 1.10.1，构建镜像内 Cython 为 0.29.14。用同一基础上的 `vehicle-semantic:pr47-grouped`（ID `sha256:1a9afe0c25145f43291f25ffaa3395bfb24e61de0b069b2051847724bddd950d`）核对了 100 条指令经过真实 ROS 门控后的软件 CAN 输出。Jetson 的 Torch 为 2.0.0+nv23.05。桌面 B1.4 在冻结样本上的路面及车线掩膜差异均为零。

新工作站需从已有团队开发机取两个本地镜像或等价的已核实环境、B1.4 权重，以及私有仿真工程和 V6.26 裁判源码快照。车机部署上述 ROS 包时不需要 Unity 或桌面镜像。[仿真容器准备及镜像迁移](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/experiment/concept-foxy-unity-20261120/vehicle_concept/checks/unity1120/foxy) · [Unity 工程和裁判源码哈希](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/experiment/concept-foxy-unity-20261120/vehicle_concept/checks/unity1120/host)

## 怎么核对“打方向晚”

用同一图像时间戳串起拍摄、选中路线、自动速度、门控后指令及随后轮速/陀螺。分别记下首次能看清外缘、所选路线首次指向外缘、减小向外转向、首次向内发令和车身实际向内转的时间。判罚按一侧轮胎完全出界；压线允许。看不到实际轮胎过线时，将该时刻记为未知。出界后停车要保留；成功恢复另需观察到车身重新沿路前进。
