# concept_yolop

ROS 2 Foxy 感知节点。B1.4 用一张 ZED 左相机图同时输出当前车道的道路掩膜、可见标线掩膜与四类物体检测。原尺寸比例为 16:9，交给 V6.26 的图像和像素坐标统一为 640×360。改变相机高度、俯仰或光学参数会改变像素到路面距离的关系。

## 离线安装和权重

在 ROS 环境可用的终端，从仓库的 `vehicle_concept` 目录构建。Foxy 使用节点解释器中的 NumPy、OpenCV、cv_bridge、PyTorch 和包含原生 NMS 算子的 TorchVision。Jetson 使用与机器的 JetPack/CUDA、Python 和 torch 对应的 aarch64 wheel；可将官方对应 wheel 和系统包事先带到机器。此包安装时不会请求网络或替换设备上的 torch。[NVIDIA Jetson 安装说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/) · [TorchVision 版本对照](https://github.com/pytorch/vision/blob/main/README.md#installation)

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select concept_yolop
source install/setup.bash
ros2 run concept_yolop concept_yolop_weights /media/usb/B1.4-road-aligned-research.pth
ros2 run concept_yolop concept_yolop_check /media/usb/front.png --device cuda
ros2 launch concept_yolop concept_yolop.launch.py
```

导入程序将校验通过的权重原子写入 `$XDG_DATA_HOME/concept_yolop/`，默认是 `~/.local/share/concept_yolop/`。也可以把已有的本地冻结归档 V6.26.zip 作为输入，程序只读取其中的 B1.4。文件 15,293,077 字节，SHA256 为 `19d6eeed99fee90a46f09761746845afcf330974a8e1aff057055697a02706a6`。使用有权访问的原件；Git 只包含源码和校验值。启动时传 `weights:=/absolute/path/to/model.pth` 可直接使用本机文件。每次加载前都会验证实际读取的字节。初次带到另一台机器时，使用上面的真实图检查核对 GPU 和 NMS。


## 话题与几何

| 用途 | 默认 ROS 话题 | 类型 |
|---|---|---|
| 已去畸变的 ZED 左图 | `/aiformula_sensing/zed_node/left_image/undistorted` | sensor_msgs/Image |
| 相机标定信息 | `/aiformula_sensing/zed_node/left/camera_info` | sensor_msgs/CameraInfo |
| 同一帧的图像、预测和时间 | `/concept/perception/frame` | std_msgs/String |
| RViz 叠图 | `/concept/perception/overlay` | sensor_msgs/Image |

叠图中蓝色是本车道，橙色是标线，有框的桶标注类别 91、分数至少 0.5。规划消息保留全部模型输出。节点使用深度 1 的相机订阅，RViz 有订阅者时才生成叠图。消息中的时间戳来自输入图像。

`concept_yolop.frame_codec.decode_frame(msg.data)` 检查协议版本和模型摘要，返回 `stamp_ns`、`frame_id`、`geometry_profile`、`bgr`、`road`、`lane`、`boxes`、`scores`、`labels`。RGB 通道以 BGR8 内存排列交付。两张 bool 掩膜及 BGR 使用无损 PNG；选择监听规划话题的远端设备时应考虑图像负载。6 月实车 bag 的 12 帧每条消息约 443–493 kB。

实车默认标为 `unverified-resize`。可用一张实测几何登记表，通过 `calibration:=/absolute/path/to/camera.json` 检查。JSON 字段为实际相机的话题 `image_topic`、光学坐标系 `frame_id`、挂载记录编号 `mount_id`、图像 `width` 和 `height`、去畸变后的 3×4 矩阵 `projection`（按行，共 12 个数），以及 `ground_marks`。地面点每行是 `[原图像素u, 原图像素v, 距车中心向右米数, 向前米数]`。这些位置应独立实测。程序验证至少 4 点，覆盖前方 3 米以内和 7 米以外、左右各 0.5 米之外。基于 V6.26 相机模型重建，每点平面误差最多 0.10 米。本版用这个范围筛查明显的投影不匹配；它只说明这批标记的测量结果。登记表与实时话题、光学帧、图像尺寸及 CameraInfo.P 对上后，发布 `calibrated-v626`。更换安装位置后应重新测量。节点仅输出感知，两种状态都能在 RViz 查看。

## 复核

现场可用 `ros2 run concept_yolop concept_yolop_calibrate --capture 新目录 --mount-id 安装编号` 取配对原图及参数。若真实镜头与冻结视角有角度或内参差异，在登记表添加另一组实测的 `fit_marks`。程序利用 CameraInfo 和这组点校正观察方向，再用独立的 `ground_marks` 核对米制误差。登记文件可用 `concept_yolop_calibrate --check camera.json` 检查。完整测量步骤见[车辆交接手册](../../AGENTS.md)。

`concept_yolop_check image.png --weights model.pth --compare same_frame_perception.npz` 在本机检查道路和标线逐像素相同、检测坐标和分数绝对误差不超过 1e-5，并把规划消息编码、解码与原数组对照。没有比对文件时仍检查推理与消息回读。图像须与比对记录来自同一拍。模型与实测证据见 [来源和实测记录](SOURCES.md)。
