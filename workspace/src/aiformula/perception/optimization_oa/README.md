# 相机视角带状通行试验

独立 `ament_python` 试验包，包名为 `optimization_oa`，节点版本为 `opti_oa_0_0`。在相机原图中规划完整通行带。选定后，只将中心线转换到 `base_footprint` 供诊断或后续控制接口使用。所有发布话题在 `/opti_oa_0_0/` 下。

## 车辆输入

| 话题 | 用途 |
| --- | --- |
| `/cone_mask` | 原始相机坐标中的障碍 mask，使用 `mono8` |
| `/aiformula_perception/yolop_a3/detections` | 同一帧 `cone_mask` 缺失时，原地重建 person、car、cone 实心检测框 mask |
| `/cone_bbox_image` | 同源时间戳的叠图背景，也可用 `overlay_base_topic` 指向相机原图 |
| `/aiformula_perception/lane_line_publisher/lane_lines/{center,left,right}` | 中心和边界点云，按标定投到相机原图 |
| `/aiformula_sensing/zed_node/left/camera_info` | 左侧去畸变图像的投影矩阵和尺寸 |
| `/aiformula_sensing/gyro_odometry_publisher/odom` | 将点云对齐到图像源时间戳 |
| `/tf`、`/tf_static` | 相机光学坐标系与车身坐标系之间的变换 |

## 启动

在装有 ROS 2、colcon 和 NumPy 的 Linux shell 中构建。先将环境变量 `VEHICLE_WIDTH_M` 设为以米为单位的实测车宽。

```bash
cd workspace/src/aiformula/perception/optimization_oa
colcon --log-base trial_log build --base-paths . --build-base trial_build --install-base trial_install --packages-select optimization_oa
source trial_install/setup.bash
ros2 run optimization_oa opti_oa_0_0 --ros-args -p vehicle_width_m:="$VEHICLE_WIDTH_M"
```

可以设置 `clearance_m` 调整单侧余量，默认为 0.08 m。回放 rosbag 时用 `/clock` 配合 `-p use_sim_time:=true`。在 `rqt_image_view` 中打开 `/opti_oa_0_0/overlay` 查看原图叠加效果。

| 输出 | 内容 |
| --- | --- |
| `/opti_oa_0_0/path_band` | 原图分辨率的整条通行带 mask。白色像素为整带 |
| `/opti_oa_0_0/overlay` | 相机视角的障碍、原参考线、选中通行带和中心线 |
| `/opti_oa_0_0/path` | 仅选中通行带的米制中心线，帧为 `base_footprint` |
| `/opti_oa_0_0/status` | 选路结果、近远端像素带宽、长度、平滑度与各类淘汰数 |

## 选路

保留原图 `roi_top_ratio=0.5`。最终可规划到哪一行，还取决于前视距离和点云覆盖。用车宽和两侧余量计算通行带。物理宽度固定，透视使得近端占用更多像素。每条候选都从近到远逐行检查，要求整条带连续、完整落在左右边界以内，并与障碍 mask 零像素重叠。

原参考带可行时优先沿用。其余情况在图像上生成不同横向偏移的平滑候选，在合格候选内最小化 `总长 × length_weight + 曲率平方积分 × smoothness_weight + 偏离参考线 × reference_weight`。搜索用逐行标定尺度估计长度和曲率，选中后再转换中心线并计算输出路径的米制长度。`no_feasible_candidate` 表示本轮采样候选均未通过。

路径起点是相机可见近端，车前盲区没有经过本节点的障碍检查。靠近车辆的有限车道点缺口可外推，用 `near_lane_gap_m` 查看。节点按单帧障碍 mask 验证几何净空，缺输入或无可行候选时输出空路径。

## 离线验证

```bash
python -m unittest -v test_trial
python replay_export.py "$BAG_EXPORT_DIR" --vehicle-width-m 0.55
```

`BAG_EXPORT_DIR` 指向本机导出的 bag 目录。两份验证 bag 使用检测框重建原图 mask。共输出 480 条通行带，全部通过逐像素零重叠和边界检查。下午 bag 有 444 帧保持原参考线、3 帧选择绕行、2 帧的所有候选均被淘汰。示例中的 0.55 m 车宽用于离线验证，实车尺寸、投影精度和 ROS 2 通信仍需实测。
