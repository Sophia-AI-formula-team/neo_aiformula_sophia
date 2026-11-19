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

实车整机启动会依次查找已安装的 `camera.json` 和 `honda-ground.json`。后者结合实时左相机 CameraInfo.P 与本田安装位姿，输出 `honda-v1.4.0-ground`。没有配置时使用 `unverified-resize`。可用一张实测几何登记表，通过 `calibration:=/absolute/path/to/camera.json` 检查。JSON 字段为实际相机的话题 `image_topic`、光学坐标系 `frame_id`、挂载记录编号 `mount_id`、图像 `width` 和 `height`、去畸变后的 3×4 矩阵 `projection`（按行，共 12 个数），以及 `ground_marks`。地面点每行是 `[原图像素u, 原图像素v, 距车中心向右米数, 向前米数]`。这些位置应独立实测。程序验证至少 4 点，覆盖前方 3 米以内和 7 米以外、左右各 0.5 米之外。基于 V6.26 相机模型重建，每点平面误差最多 0.10 米。本版用这个范围筛查明显的投影不匹配；它只说明这批标记的测量结果。登记表与实时话题、光学帧、图像尺寸及 CameraInfo.P 对上后，发布 `calibrated-v626`。更换安装位置后应重新测量。节点仅输出感知，两种状态都能在 RViz 查看。

## 复核

现场可用 `ros2 run concept_yolop concept_yolop_calibrate --capture 新目录 --mount-id 安装编号 --optical-height-m 0.55` 取配对原图和 CameraInfo。将实际左相机光心到水平地面的米数填入 `--optical-height-m`。该值随草稿保存为 `optical_height_m`。

用实拍图中至少四个已测地面标记填写 `fit_marks`，再用另外至少四个已测标记填写 `ground_marks`。每组都要覆盖左右和远近。以车体 `base_footprint` 为参照记录向右、向前的距离，单位为米。程序按实测光心高度和镜头内参拟合朝向，把地面投影变换到冻结策略的 0.655 m 参考视图，随后用独立标记检验所有点的误差。每点均须在 0.10 m 以内。[车辆交接手册](../../AGENTS.md)

填完后执行 `ros2 run concept_yolop concept_yolop_calibrate --install 新目录/camera.json`。校验通过后，命令会原子写入 `$XDG_CONFIG_HOME/concept/camera.json`（缺省为 `~/.config/concept/camera.json`）。`--check 新目录/camera.json` 可以单独查看验证误差。

`concept_yolop_check image.png --weights model.pth --compare same_frame_perception.npz` 在本机检查道路和标线逐像素相同、检测坐标和分数绝对误差不超过 1e-5，并把规划消息编码、解码与原数组对照。没有比对文件时仍检查推理与消息回读。图像须与比对记录来自同一拍。模型与实测证据见 [来源和实测记录](SOURCES.md)。
