# 调试与部署报告 — 2026-07-17 JST

## 今日完成的工作

1. 阅读 `/home/nvidia/Desktop/context.md`，审查本地 ROS 源代码，并连接 `control@192.168.0.197`。
2. 在物理相机拔除的情况下，把 `/home/nvidia/Downloads/000750.jpg` 发布到 `/aiformula_sensing/zed_node/left_image/undistorted`。
3. 确认道路检测器输出有效的 640 x 360 掩码，但车道发布器输出的左、中、右 `PointCloud2` 全为空。
4. 检查实际标定和 TF：
   - 内参文件对应 640 x 360 nHD 输出；
   - 车体坐标系为 `base_footprint`；
   - 相机坐标系为 `zed_left_camera_optical_frame`；
   - TF 有效，能把数千个有限掩码点投影到车辆 ROI 内。
5. 找到真正故障：`min_component_pixels = 180` 是按 1920 像素宽度设计的阈值，却直接用于 640 像素图像，导致有效的 108 像素和 74 像素车道分量被拒绝。
6. 把该阈值改为随图像宽度缩放。640 像素时阈值为 60。重新构建并部署远程软件包。
7. 使用同一图像验证：左、中、右点云各包含 9 个有限、非零点。
8. 删除远程机器上的五个重复 `lane_line_publisher` 源代码软件包。没有删除本地软件包。
9. 对比远程整合启动文件和本地启动文件。补回缺少的 `lane_points/lane_0215` 与 `kalman_filter/withoutkalman_0312` 节点。没有把 LYA 加入 allnodes，也没有修改 LYA。
10. 修正陀螺仪里程计，通过可配置的 `imu_topic` 参数订阅 `/aiformula_sensing/vectornav/imu`。
11. 仅回放 `/media/control/R/OWEN/rosbag2_2026_06_29-17_48_44` 中的图像话题，并单独启动未修改的 LYA。LYA 收到 `/filtered_lane_pose`，输出非零速度和转向命令，CAN 里程计增加约 97 m。根据用户指令停止回放。
12. 诊断 RViz 故障。配置中有三个 Ogre Image 显示项，并保留 1600 x 900 的旧窗口布局，而 Jetson 桌面为 1024 x 768，因此出现 `GLXBadDrawable`。
13. 把远程 RViz 配置改为只显示 PointCloud2 和 Grid。`vehicle_fit_image` 改由独立 `rqt_image_view` 显示。为两个 GUI 进程明确设置 `DISPLAY=:0` 和 `XAUTHORITY=/home/control/.Xauthority`。
14. 在不启动车辆控制的情况下验证 RViz 和 `rqt_image_view`。两者都能正常打开；RViz 报告 OpenGL 4.6，没有 GLX、Ogre 或堆错误。
15. 调查约 13:44 JST 的远程重启。新启动报告 `PMC reset source: SYS_RESET_N`。没有保留上一次启动日志或 pstore 记录，也没有发现 OOM、内核 panic、NVIDIA Xid、过热或看门狗重置证据。无法证明精确原因；现有证据指向硬复位、供电中断或复位线事件。

## 当前远程状态

- ROS 节点：已停止
- rosbag 回放：已停止
- LYA：已停止
- RViz 和 `rqt_image_view`：验证后已停止
- 电机与 SocketCAN ROS 进程：已停止
- 物理相机：已拔除

## 当前远程哈希

- 修正后的 `lane_line_publisher.cpp`：`87a7748b3c363ec144f644dccfabd0f95a7befc599a45efeb3a2a0eaa574b93b`
- 已部署车道可执行文件：`7e3cfd2adaf8a8339b347d5948136a8db033df01e722c2b7e8745afc401bfd31`
- `allnodes.launch.py`：`f97159b6d6d0cb7a9dea731bced6f9c6495aa53a3e9e937da9521cfbdb33a6b4`
- 仅数据 `2x2.rviz`：`ec0e1880b8870e33231bc848076f8a91854ae4dfcb0c85efb52209bfb207a39c`
- 陀螺仪里程计启动文件：`276d35667c18d0b7fc50287d51004e17bf9e42042491318a7c1475129bfe7f88`

## 本地范围

没有修改本地 ROS 工作空间。所有本地补丁、证据与回滚文件都保存在 `/home/nvidia/Desktop/codexws`。
