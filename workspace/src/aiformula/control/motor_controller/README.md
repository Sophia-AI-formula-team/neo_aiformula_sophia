# motor_controller 0.4.1

- 保留已验证 USB sigmoid 非零转向和 CAN 0x210 右/左轮 int32 小端编码；零速度输出全零，直行绕过 sigmoid 零点。
- 原 allnodes、LYA、手柄入口不变，不需要额外参数。未启动 semantic 时直接使用原速度话题；命令超过 0.5 秒未更新输出零。
- `ros2 run semantic_vehicle_wiring semantic` 启动红绿灯与 topo。红绿灯投票由 semantic 执行，motor 只执行其 stop/go。正常 Ctrl-C 退出后恢复普通 LYA；异常失联暂停自动行驶，重启 semantic 可恢复。
- 手柄 L2/L1 使能优先，沿用原映射和速度话题；松开后需要新自动指令。开启 semantic 也不影响手柄接管。
- 不按距离停车、不自动左转。未完成实车跑圈验收。历史来源见 UPSTREAM.json。

0.4.1 修复绿灯后被 topo 的车道点/诊断速度状态持续拦住：motor 只读取 semantic 的 `traffic_stop_required`。需与 MapAssist 0.2.2 配套更新；LYA 仍须提供新鲜非零指令，绿灯不会重放停车前速度。
