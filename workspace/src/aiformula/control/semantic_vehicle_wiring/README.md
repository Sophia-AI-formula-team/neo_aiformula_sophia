# 车辆 A3 信号接线

`a3_signal_capture` 将 `/aiformula_perception/yolop_a3/detections` 中真实 red/green/unknown 与原始相机 Header 转为 `/semantic_drive/signal_capture`，sequence 等于采集纳秒时间戳；重复/回退帧不发布，不定时重复旧颜色。

本包没有模型、距离估计或电机输出。距离和不确定度为 null；本次稳定红灯停/绿灯走不需要米制距离。

当前已编译部署。`ros2 launch launchers semantic_vehicle.launch.py` 使用本节点和信号桥给 topo 提供观测，motor_controller 同时直接读取原始带时间戳捕获，在发 CAN 前门控。运动默认禁止，详见 launchers/MOTOR_TRAFFIC_DEPLOYMENT.md。旧 semantic_vehicle_staged / semantic_shadow 仍是独立观察链，不与新入口叠加。

相机帧为 zed_left_camera_optical_frame。本次不更改相机标定、TF 或 LYA；里程计与车道点由现有 allnodes 提供。静止隔离通信测试不等于实车建图/刹停验收。
