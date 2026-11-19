# B1.4 来源和对照

本包保留单帧 RGB 推理所需的 MobileNet V3 特征层、两支独立分割解码器、FCOS 检测头及其冻结参数映射。类别顺序为 1、3、10、91。对研究模型做了严格的 state_dict 加载和独立输出对照。训练损失、训练数据工具、CSP 候选分支、实验字体/绝对路径以及读取旧检查点的初始化程序没有进入 ROS 节点。网络模块由本机的 TorchVision 提供。上游研究源和 SHA256 如下。

| 原研究路径 | SHA256 |
|---|---|
| YOLOP_B1/model.py | b7374fa180087ecde5b9849f07b3ab4d7b1460a8be2cfa0cff4ffa2ace706f69 |
| YOLOP_B1/phase2/models.py | 54c5922b569494f39cff0381ae151cfd9b1a68b61f7aa06fcc833879fcf47794 |
| YOLOP_B1/common.py | 79a2d33f19cf5157bd6eb0a65c59b2de852877ddeab8e1082fa9d894631bedfc |
| YOLOP_A3/experiments/efficient_v22_20261025/model.py | a022cb227ab1f1fdff9a587f9b87245b9a0e68943becca5e6b1f095ebeb9b89e |
| YOLOP_A3/experiments/class_scope_v30_20261025/objectives.py | 8c1fcff5073366623544288a44b539268716e8b08b89f14d28bfc548346b044c |

权重是本机 B1 实验生成的 `B1.4-road-aligned-research.pth`，没有随源码分发。FCOS 的训练子类只覆盖损失函数，推理沿用 TorchVision 的 FCOSHead，载入完全相同的权重。B1.4 的 backend 是 mobile，初始化以后运行只需本版检查点。

2026-11-19 在 NVIDIA RTX 4070 Ti、torch 2.11.0+cu130、torchvision 0.26.0+cu130 上运行了两个独立入口。原入口为 `YOLOP_B1.phase2.models.load` 和 `YOLOP_B1.common.image_tensor`。新入口为 `concept_yolop.inference.Predictor`。原模型和新模型均以 FP32 推理。用 2026-06-29 17:48:44 的实车 bag，按原检测项目 `reports/real_bag/source.json` 核验 12 张原图 SHA256。涵盖序号 0、140、281、421、561、702、842、983、1123、1263、1404、1544。道路和标线共 5,529,600 个逐像素判断一致，1,200 个检测的坐标、类别与分数逐项完全相同。每帧打包再解码后，图像、两张掩膜、全部检测及原始纳秒时间戳都相同。

另外，V6.24 移动桶外圈的第 201 帧重现了此前保留的 Unity 输出：道路 83,984 像素、车线 892 像素、检测 100 项。这里验证的是与原感知模型输出一致；车机的安装、实时延迟和物理相机标定仍要在目标设备测量。

独立 ROS 2 Foxy 基础容器（Linux x86_64、Ubuntu Focal、Python 3.8.10）断网完成 colcon 构建、ament 发现、三个可执行入口枚举、单独 launch 参数解析和冻结归档权重导入。该基础镜像未装 TorchVision、PyTorch、cv_bridge 和 OpenCV；上述 CUDA 推理来自 Windows 工作站。Jetson 上需另测本地 TorchVision NMS 和整条图像消息链。

后续在独立的 ROS Foxy CUDA 容器中重放了 10 月 27 日实车原始 BGRA8 画面，从相机消息到 B1.4 输出收到 120/120 条，15.00 Hz，延迟中位数 33.48 ms。完整设置、对照和原始统计见[相机墙钟实测](bench/README.md)。

2026-11-20 针对团队 Foxy 开发镜像比较了两种 FP32 卷积实现。117 张图的路面、车线与全部检测逐项相同。两次新版的图像发布到 ROS Twist 中位延迟为 89.25 和 90.90 毫秒，沿用车端当前的 Orin 默认设置。[各轮墙钟、GPU 显存与复测步骤](bench/fusion.md)
