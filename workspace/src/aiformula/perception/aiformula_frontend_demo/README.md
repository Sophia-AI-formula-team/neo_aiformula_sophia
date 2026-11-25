# 单帧相机语义与 Opti OA 实验

三个独立的 ROS 2 可执行入口都位于 perception 下的 aiformula_frontend_demo 包。

| 入口 | 输入 | 输出 |
| --- | --- | --- |
| semantic | 前视图像、CameraInfo、车身到相机的 TF | 当帧车身坐标下的路面、标线、障碍与通行掩码 |
| geometric_hash | 语义观测、里程计 | 局部几何哈希与点到曲线配准的诊断结果 |
| opti_oa | 当帧语义观测 | 通过整带检查的局部路径、通行带和状态 |

默认 replay.launch.py 运行 semantic 与 opti_oa。通行带和中心线每帧重新计算。geometric_hash 可独立启动，用于诊断局部配准。独立节点的结果发布在 /front_demo 下。

横线和斑马线按经验证的地面漆处理。能观察到两段白漆及中间路面时才确认纵向虚线。双白实线、路外与障碍作为通行硬约束。规划按车宽检查完整通行带，并按路径长度、曲率和单线接触代价选中心线。

## 回放

准备 ROS 2、NumPy、OpenCV、ONNX Runtime、相机标定和匹配的 YOLOP 640×640 ONNX 权重。依赖的 optimization_oa 包来自 PR #52。以下命令从仓库根目录运行。

```
colcon build --base-paths workspace/src --symlink-install --packages-up-to aiformula_frontend_demo
source install/setup.bash
ros2 launch aiformula_frontend_demo replay.launch.py model:=/absolute/path/yolop-640-640.onnx
```

根据录包调整 image_topic、image_kind、info_topic、body_frame 和车宽等参数。输入 Bag 需要相机图像、标定和相机 TF。inference_provider 默认 auto，可选 cpu 或 dml。dml 需要安装支持 DirectML 的 ONNX Runtime。独立查看几何配准时可运行以下命令，里程计话题和车身坐标系按实际录包设置。

```
ros2 run aiformula_frontend_demo geometric_hash --ros-args -p odom_topic:=/odom -p body_frame:=base_link
```

## 当前验证和问题

- Neo 16:04 Bag 的 02:35–02:43 片段使用 108 张原始相机图像，50 帧通过整带检查。同一片段在 Windows 的语义加规划单帧中位耗时由 CPU 旧版的 278 ms 降到 DirectML 新版的 80 ms。
- Unity 双锥桶场景的实际闭环图像处理速率由 2.31 提高到 7.97 张每秒。最新试验通过第一只锥桶，在 14.93 m 处报 start_footprint_blocked 后停车。整圈和实车 ROS 2 尚待验收。
- **虚线语义**仍不稳定。近端、磨损、低对比度或截断的漆段可能漏标，局部识别覆盖有改善，实拍片段中识别到虚线的帧数仅从 47 增至 48。
- **语义地图边界**仍会限制连续通行带。需要核对道路分割边界与真实铺装边界，区分边界误判、车头视野缺失和真实障碍遮挡。
