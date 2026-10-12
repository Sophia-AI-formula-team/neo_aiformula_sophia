# 车辆启动（motor 0.4.0）

每个终端先加载 `source /home/workspace/install/setup.bash`。

1. 原基础节点：`ros2 launch launchers allnodes.launch.py`
2. 原 LYA：`ros2 run trajectory_follower lya_follower_connected_omegat_global`（仅运行一个 LYA）。
3. 需要红停绿行和 topo 时另开：`ros2 run semantic_vehicle_wiring semantic`。

不加 enable_motion 或 use_semantic 参数。不运行第 3 条时普通 LYA 不依赖红绿灯。手柄保持原操作，L2/L1 使能优先接管。

正常 Ctrl-C 关闭 semantic 后恢复普通 LYA；异常退出/失联保持自动停车，重新启动 semantic 恢复。手柄仍能接管。停止 LYA 后 0.5 秒命令超时归零。

semantic 使用 PR23 stop_wait，去掉按距停车，稳定红灯等待、稳定绿灯放行；拓扑按真实里程计记录一圈，不执行自动左转。map_assist 速度输出仅诊断，motor 使用其决策门控原 LYA 输入。旧 shadow 入口保持不接车。

部署和隔离验证不会启动真实电机输出；静止模拟消息不代表实车跑圈验收。
