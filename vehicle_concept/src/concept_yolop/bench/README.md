# B1.4 相机到感知的墙钟实测

相机发布进程和原版 B1.4 节点分别运行。程序每秒发送 15 张不同的原始画面，记录每张发送和收到同拍感知结果的单调时钟，并保留模型的全部三路输出。测试没有启动规划器。GPU、操作系统和 ROS 消息链的结果与实车 Jetson 分开统计。

## 测到了什么

环境为 RTX 4070 Ti、Linux x86_64 Docker、Ubuntu 20.04、ROS 2 Foxy、Fast DDS 2.1.4、Python 3.8.10、Torch 2.4.1+cu118、TorchVision 0.19.1+cu118 和 OpenCV 4.12。GPU 使用 FP32、冻结的 B1.4 权重和原来的 640×360 模型输入。各次单独测试，不开启 RViz 叠图订阅。正式段开始前先等 15 条真实视觉结果。

| 输入和版本 | 相机发出 / 模型收到 / 下游收到 | 输出频率 | 相机发布到收到感知，p50 / p95 / 最大 |
|---|---:|---:|---:|
| Unity 外圈 120 张 BGR8，原编码与默认 DDS | 120 / 86 / 86 | 10.78 Hz | 36.29 / 55.57 / 66.49 ms |
| 同一组 120 张，固定 PNG 行滤波与默认 DDS | 120 / 93 / 93 | 11.59 Hz | 29.27 / 58.85 / 91.30 ms |
| 10 月 27 日实车 bag 的 120 张 BGRA8，固定 PNG、CUDA 预热、仅相机发布者使用下述 DDS 配置 | 120 / 120 / 120 | **15.00 Hz** | **33.48 / 66.05 / 79.73 ms** |

Unity 同场编码中位耗时从 12.03 ms 降到 6.12 ms。两次都有部分图像未到达模型订阅回调，回调处理过的每一张都送到了感知订阅者。默认配置另跑 180 张时收到 146 张。不同次的 GPU 耗时会波动，编码改善和相机丢帧分别统计。每轮的原始统计见 [results](results)。

实车组来自 `rosbag2_2026_10_27llp`。原 bag 的左图话题共有 1,847 张，全部为 640×360 BGRA8，按采集戳计算为 14.985 Hz。回放测试取开头连续的 120 条原始 CDR 消息，保留四通道像素、尺寸和光学帧名，以本次回放的拍摄时刻计算延迟。启动预热后首张真实图像用了 35.69 ms。节点处理一次画面的中位耗时 31.71 ms，其中推理 22.42 ms、消息编码 8.77 ms。测试未装入车上实测的投影校准文件。

Neo 默认让 ZED 以 15 Hz 采 HD1080，由 ZED 节点按三倍下采样后发布 640×360。10 月 27 日的实车图每条含 921,600 字节像素，Unity 的 BGR 图每条为 691,200 字节。用真实相机消息与 CameraInfo，CPU 抽 12 条各测 10 次，BGRA 转 BGR 的中位数为 0.15 ms，p95 为 0.48 ms。已有输入尺寸时尺寸检查中位数 0.004 ms。采用实测内参、假设相机无额外安装旋转，仅测 640×360 投影变换的成本，中位数 1.35 ms、p95 3.40 ms。物理安装旋转、ZED SDK 的采集和校正时延、Jetson 的 GPU 耗时需要上车测量。

## 复现

在有 CUDA GPU 且已装好 Foxy、`cv_bridge`、`concept_yolop` 的隔离环境运行。该测试使用官方 Python 3.8 Linux x86_64 的 CUDA 11.8 torch 2.4.1 与 torchvision 0.19.1 wheel。两者和各 CUDA 依赖独立安装，已有 CPU 环境保留。可以在新 Python 环境中按官方版本表选择对应 wheel。[PyTorch 版本表](https://pytorch.org/get-started/previous-versions/)

```bash
source /opt/ros/foxy/setup.bash
source /path/to/colcon/install/setup.bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
BENCH=/path/to/vehicle_concept/src/concept_yolop/bench
python3 "$BENCH/foxy_camera_latency.py" /data/outer-lap/frames \
  --count 120 --weights /data/B1.4.pth --profile --output /tmp/vision/unity.json

python3 "$BENCH/foxy_camera_latency.py" /data/1027-first120-cdr \
  --format zed-cdr --count 120 --weights /data/B1.4.pth --profile \
  --camera-dds-profile "$BENCH/fastdds_camera_local.xml" --output /tmp/vision/real.json
```

PNG 目录按 `*_rgb.png` 文件名排序。CDR 目录按 `*.cdr` 文件名排序，每个文件保存该拍完整的序列化 `sensor_msgs/Image`，不应混入 CameraInfo。SQLite3 bag 的内容来自 `topics.name='/aiformula_sensing/zed_node/left_image/undistorted'` 对应的 `messages.data`，按 `messages.timestamp` 顺序单独导出并用补零序号命名。测试先把原消息读入内存，文件读取不计入相机延迟。`--profile` 另记进程内推理、输出编码与整个回调耗时，时间戳逐拍对齐。未加此参数时原版视觉节点直接运行。

## 相机消息为何会少

在隔离的 CPU 容器重复发布同一张实车 BGRA8 原图，发 120 个不同序号，用独立订阅进程逐条确认。发送频率为 15 Hz，QoS 为 best effort、depth 1，每次接收后停留 28 ms，模拟订阅处理占用。默认 Foxy 传输收到 84/120。只给接收者增大共享内存收到 93/120，只给发送者设置则收到 120/120，两边都设置时也是 120/120。

随附 XML 为可选实验参数，让 Fast DDS 使用 16 MiB 的共享内存区和最多 1 MiB 的共享内存消息，同时保留 UDPv4。可仅给相机发布进程设置 `FASTRTPS_DEFAULT_PROFILES_FILE`。使用本机仅支持 UDP 的订阅进程也收齐了 120 张。另一次跨 Docker 内部网桥测试中，原配置与新配置都没有发现对端，因此远程 RViz 和车载网络发现仍需现场验证。车载配置和依赖均未因这项测试而改变。
