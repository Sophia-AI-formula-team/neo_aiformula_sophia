# motor_controller 0.3.0

- 基于用户确认可转向的 USB sigmoid 版本，保留非零转向拟合和 CAN 0x210 右/左轮 int32 小端编码；零 Twist 输出八个零字节，直行绕过 sigmoid 零点偏差。
- 门控位于 motor controller 发 CAN 前：自动 LYA 使用原 `/aiformula_control/game_pad/cmd_vel`；手柄改用 `/aiformula_control/manual/cmd_vel`，按住映射后 L2/L1 使能时手动优先（包括红灯时），松开后要求新自动指令。
- 真正新鲜的 A3 图片观测：首次红灯先停止，0.5 秒窗内至少 3 帧且严格多数红灯锁停；未知/丢帧不解除，稳定绿灯且收到新自动指令后放行。未遇红灯时，新鲜 unknown 表示继续普通循线。无按距停车、无自动左转。
- 命令要求 |v|≤4m/s、|ω|≤3rad/s 且数值有限；自动输入 0.25 秒失效停车；信号源图像年龄 0.5 秒，topo 状态 0.6 秒，里程计/车道点需新鲜。手柄使能或速度超时保持停车，不掉回自动。多个速度发布者也停车。软件进程完全死亡仍依赖底盘自身 CAN 超时，未作物理制动验收。
- 部署默认 `motion_enabled=false`，所有模式输出零；未来允许运动才使用 `ros2 launch launchers allnodes.launch.py enable_motion:=true`。另开一个 LYA 和 `ros2 launch launchers semantic_vehicle.launch.py`；不要叠加 shadow/map_drive。详见 `launchers/MOTOR_TRAFFIC_DEPLOYMENT.md`。
- 历史来源和原始 SHA 保留在 UPSTREAM.json；本版测试和部署回执在日期交接记录，静止测试不等于实车跑圈验收。
