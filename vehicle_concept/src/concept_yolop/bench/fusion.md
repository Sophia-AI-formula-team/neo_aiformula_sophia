## 两个 FP32 卷积与实测延迟（2026-11-20）

在 Foxy 开发镜像的 RTX 4070 Ti（Torch 2.0.0、TorchVision 0.15.1、CUDA 11.8、cuDNN 8.7）上，两支分割头各有两个相同形状的卷积耗时很长，每帧还需要约 2.6 GiB 额外工作空间。保留原分辨率、权重、归一化、每个卷积、插值和检测，只把 112→48 和 160→64 通道的卷积交给 PyTorch 原生 FP32 CUDA 实现。GPU 的其他卷积继续走原路径。

逐图比较用了 12 张 6 月 29 日实车原图、5 张冻结的移动桶样例和 4 个 Unity 场景各 25 张镜头。117 帧的两张掩膜共 53,913,600 个二值判断；全部相同。11,521 个检测输出的类别、框坐标和分数也全部精确相同，其中 247 个是视觉模型分数至少 0.5 的桶检测，同一只桶可以出现在多帧。对有同步轮速和陀螺的 100 个 Unity 画面，以当拍原始输入推进两套独立的驾驶记忆，路界行、每个候选的可行性、选中的轨迹索引、左右转向和速度指令逐拍一致。实车照片用于视觉对比。

相同图像按旧、新、新、旧顺序测量，表中保留四遍。每次仅驻留一份网络。视觉包含已解码 BGR 到 NumPy 掩膜和检测，按 GPU 输出拷回 CPU 的实际完成时间计时。系统墙钟另用 `wall_probe.py`，每秒向正式 ROS 节点发布 5 张直线片段输入，各次均为 25 张。两组实验分别计时。详见[每帧结果](results/fusion_fp32_20261120.json)。

| 次序 | 视觉 p50 / p95（毫秒） | PyTorch 峰值已分配（MiB） | 图像发布至 ROS Twist p50 / p95 / 最大（毫秒） | 收到指令 |
|---|---:|---:|---:|---:|
| 旧，第一次 | 61.55 / 90.28 | 2847 | 137.86 / 193.90 / 220.55 | 25/25 |
| 新，第一次 | 32.82 / 66.02 | 183 | 89.25 / 159.27 / 211.70 | 25/25 |
| 新，第二次 | 17.81 / 47.79 | 184 | 90.90 / 144.73 / 198.46 | 25/25 |
| 旧，第二次 | 62.12 / 91.16 | 2856 | 140.28 / 186.56 / 232.82 | 25/25 |

ROS 参数 `cuda_fusion` 的默认值为 `auto`。cuDNN 8.6 的 Orin 保留此前在车上使用的 112→48 设置。cuDNN 8.7 的 RTX 4070 Ti 在最终弯道对照后，默认只替换 112→48。本文前表归档两类卷积都替换的测值；该版本可显式设为 native112_160。每帧驱动的比较见[最终驾驶报告](../../../checks/edge1120b/README.md)。其他 GPU 和 CPU 保留原来的默认库设置。没有本轮 Jetson 测值。可通过 `stock`、`native112`、`native112_160` 明确选择 CUDA 测试设置。程序启动时会打印实际命中的通道；CPU 始终采用 CPU 后端。

同一台新设备可用下列命令成对测量。Orin 的对照设为 `--before native112 --after native112_160`，与它现有车端路径比。若只验证视觉，可以省去场景与标定文件。ROS 墙钟测试的隔离与启动方式见 [Foxy 执行说明](../../../checks/unity1120/foxy/README.md)。

```sh
python3 /opt/concept/src/concept_yolop/bench/cuda_fusion.py \
  --real /data/real_bag --frozen /opt/concept/src/concept_drive/test/data/moving-lap-start \
  --weights /weights/B1.4.pth --before stock --after native112_160 \
  --scene /unity/straight01 --scene /unity/outer_edge01 --scene /unity/inner_edge01 --scene /unity/outer_hard01 \
  --calibration /opt/concept/unity/honda.json --output /evidence/fusion.json
```
