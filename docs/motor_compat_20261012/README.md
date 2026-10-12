# 2026-10-12 motor 0.4.0 兼容版交接

本次替换此前已回滚的 motor；保留用户 USB sigmoid 非零转向拟合，修复零速度 CAN 非零。不再要求 enable_motion/use_semantic。车上原 A3 allnodes 仅给 motor 增加与 teleop 相同的手柄配置，摄像头、A3、LYA 和标定不改。

- motor 纯逻辑 25 项测试本地及车上 Python 3.8 均通过；安装后的真实 DDS 隔离测试 27 场景全部通过，见 dds_results.json。
- motor 0.4.0、semantic_vehicle_wiring 0.2.0、launchers 三包在车辆 Foxy 编译通过；PR23 MapAssist 0.2.1 不改。
- 原 allnodes + 原 LYA 可独立运行。启动 `ros2 run semantic_vehicle_wiring semantic` 后启用 semantic stop/go 和 topo；正常 Ctrl-C 退出解除门控。异常失联保持自动停车，重新启动 semantic 可恢复；手柄始终优先。
- 灯色判定完全归 semantic；motor 不复制红绿灯投票。PR23 stop_wait 未遇红灯/绿灯放行后无新灯色仍允许行驶；已锁定红灯后 unknown 不解除。没有新增“相机丢帧一律停车”策略。
- motor 收到有效手柄使能直接用映射后 Joy 计算手动速度，保持旧 teleop 和 LYA 的共享速度话题。松开/门控释放后只接受新自动指令；命令和手柄 0.5 秒超时归零。
- 本次验证没有启动真实电机、CAN socket 或实际驾驶节点；测试域 93、localhost，CAN 仅发隔离诊断话题。没有移动车辆，未验收实际一圈或物理制动。

启动步骤见 [部署说明](../../workspace/src/aiformula/launchers/MOTOR_TRAFFIC_DEPLOYMENT.md)。备份 `/home/control/aiformula_backups/20261012_motor_compat_0_4_0`；运行记录 `/home/control/aiformula_runtime/motor_compat_20261012`。

回滚时先结束驾驶节点、保存当前修改，再按 runtime/before.json 和本目录 manifest 恢复 backup/source 对应文件及三个包 build/install；仅删除本次新增且哈希匹配的文件，不能覆盖其他包。旧备份保持不动。回滚将恢复旧零速问题。

收尾只读检查发现外部终端已启动真实 motor/CAN 栈；本次未启动、停止或向其发送测试消息，实际车辆是否移动不由隔离验收推断。
