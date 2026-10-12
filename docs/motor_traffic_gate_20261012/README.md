# 2026-10-12 静止部署回执

**已按用户要求从车辆回滚 motor 和 allnodes 接线。** 原 USB motor 源码、build、install 已恢复，SHA256 为 `022a4c6210a0fb9f8d2543ab9337a90ce117dae0f2bf230ff5904c88bec42742`；原 allnodes SHA256 为 `0957d8fa41183d80cbcc25565de414aeccfd7657a76b17e42d2e5b9f60c318df`。不再强制 semantic 门控、手柄独立话题或 enable_motion 开关；原版零速问题也随之恢复。其他包未回滚，未启动任何节点。新版本存档于 `/home/control/aiformula_backups/20261012_motor_gate_rollback`，车辆回执为 runtime 下的 `rollback_result.json`。以下是已回滚版本的历史验证记录，不代表当前车上 motor。

基于 PR23 `6caa6a8`，motor 来源为 PR20 `b9676e4` 的用户确认 USB sigmoid 版本。
本次用户授权编译、motor 内红灯门控、手柄优先和零轮速修复；没有发送运动指令。

- 6 包编译通过：semantic_planner_core / semantic_local_planner_ros / semantic_drive_bridge / semantic_vehicle_wiring / motor_controller / launchers；最终 motor/launchers 增量编译和启动参数检查通过。
- motor 策略 23 项（本地 + 车上 Python3.8 各通过）；topo 节点 108 项、core 100 项本地回归通过。首圈闭环/融合是回调单测，不是实车跑圈。
- 车上真实 rclpy/DDS 隔离测试 20 场景通过；域93、localhost、CAN 只到 `/gate_acceptance/can`，无 SocketCAN 节点/硬件发送。红/绿、手动抢占/释放/失联、摄像头/LYA/semantic 失联、重复自动发布者、零载荷、非零 sigmoid 转向回归均通过。
- 初次编译发现新增 launcher→motor 声明与现有 common_python/vehicle/launcher 形成依赖环，已移除冗余声明；首次 DDS 诊断 JSON 遇 Foxy uint8 数组序列化错误，已转原生 int 后复测全部通过。失败日志保留。
- 随后用 `enable_motion:=false` 实际启动 allnodes：767 帧零 CAN、73 条禁用状态，145 个 odom 样本的前进速度全为零；无 LYA，检查结束所有自启节点已退出。停止时既有 LLP/CAN receiver 报异常退出，发生在 SIGINT 后，未改这两个包。
- 收尾期间车辆终端另起 allnodes 出现一次 motor 启动退出，日志没有具体 Python 异常；相同安装入口隔离复核与后续实际禁用运动启动均正常，未复现，不推断该次根因。
- 50 个部署源文件和关键安装模块哈希核对通过。motor 0.3.0 / local planner 0.2.1；`motion_enabled=false`。最终没有真实驾驶或测试节点运行。
- 既有车上 A3 allnodes 覆盖层完整保留，只改手柄独立输入、motor 接线和运动开关；实际 allnodes 全文单独归档，哈希见 vehicle_source_manifest.json。未改 A3/相机/LYA 算法、参数或 TF。

启动与行为见 [MOTOR_TRAFFIC_DEPLOYMENT.md](../../workspace/src/aiformula/launchers/MOTOR_TRAFFIC_DEPLOYMENT.md)。核心证据见 receipt.json；完整车辆日志/初次失败/安装回执位于 `/home/control/aiformula_runtime/motor_traffic_gate_20261012`。真实首圈、视觉灯色准确率和物理制动验收未执行。

## 回滚（不要在驾驶节点运行时执行）

备份根 `/home/control/aiformula_backups/20261012_motor_traffic_gate`：`source/` 保存被覆盖源文件，`build/` 和 `install/` 保存这 6 个包的原目录（含原 symlink）。`runtime/before.json` 记录原本不存在的新文件。

1. 保持车辆静止，确认 allnodes/LYA/semantic/motor/CAN sender 均已退出；停止进程不等于物理刹停。
2. 对照 vehicle_source_manifest.json 检查当前文件没有后续改动；若有漂移先保留并审查。把当前 6 包 build/install 和当前部署文件移入新的回滚存档目录。
3. 按 before.json 恢复 `source/` 原文件，只删除本次新增且哈希仍匹配的文件；不要覆盖整个 src 树。
4. 将备份的 6 包 build/install 移回原位，重新 source 原 install，检查旧源码/安装哈希和入口；不要自动启动驾驶。

注意：回滚会恢复旧 motor 的零速 sigmoid 问题及旧手柄共用话题。原 PR19/20/23 与所有旧备份均保留，本次不合并或改写它们。
