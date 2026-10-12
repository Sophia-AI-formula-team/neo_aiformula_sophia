# 绿灯恢复修复：motor 0.4.1 / MapAssist 0.2.2

状态：离线修复和部署包已准备，尚未连接或部署到车辆。

## 问题和证据

0.4.0 把 motor 放行同时绑定到 MapAssist 的 `traffic_mode=normal`、`velocity_gate_reason=forwarding_controller` 和 `path_output_allowed=true`。然而车辆 LYA 使用原有车道链，MapAssist 的点/速度输出仅为诊断；绿灯后 MapAssist 等待新的完整 ABC，不能代表原 LYA 是否可用。

本地运行真实 MapAssist 与 MotorGate 回调（ROS stub）复现：`traffic_reason=green_resume`、`traffic_mode=normal`，但 `velocity_gate_reason=waiting_fresh_visual_triplet`，motor 仍输出零。修复前记录见 before_reproduction.json。车辆当时的具体阻塞状态尚无日志，联网后须读取确认，不能把该复现等同于实车根因已完全确定。

## 修复

MapAssist 状态新增 `traffic_stop_required`，明确表示 stop_wait 红灯锁停：稳定红灯 true，当前新鲜且确认的绿灯 false；unknown/单帧绿灯/断信号不能解除已锁定红灯。motor 只消费此交通决定，不再要求 topo 诊断车道/速度就绪。

门控之外，motor 仍要求新鲜 LYA 指令并保留手柄优先。绿灯不会生成速度、重放旧命令或把 LYA 的零速度变成非零。正常退出 semantic 恢复普通控制、异常失联保持自动停车的规则不变。拓扑学习、私有 ABC 输出、距离策略、LYA 参数和轮速算法不改；没有新启动参数。

两包必须一起更新；motor 0.4.1 遇到没有新字段的旧 semantic 会保持停车。其他 semantic/shadow 消费者可以忽略新增字段。

## 验证

Python 3.8.20：motor 27 项、生产回调联动 6 项、MapAssist/简单红绿灯/启动接线 108 项、部署事务 4 项全部通过。联动覆盖三次红绿循环且完全没有 topo ABC、topo 延时未结束、绿灯无新 LYA、LYA 零命令、单帧绿灯/unknown、手柄优先。

部署事务使用临时目录和模拟编译器，检查源码漂移/进程占用拒绝部署、备份、编译失败恢复及安装文件校验。车辆 ROS/Foxy 编译、真实 DDS 和实车恢复尚未执行。dds_acceptance_pending.py 已增加“无 topo ABC 的红停绿行”隔离场景，等待联网后执行；它不是已通过的实车证据。

## 包与部署

部署包只覆盖 manifest 内的 motor / semantic_local_planner_ros 修改文件，不改车上 allnodes、A3、LYA 或 wiring。包中 install.py 先校验包和车辆基线 SHA、拒绝运行中的驾驶/semantic 栈，保存源码及两包 build/install，再编译并校验安装模块；失败自动恢复。不会启动任何 ROS 驾驶节点。

车辆恢复连接后先读取状态/日志、确认实际源码和节点状态，再安排停止相关节点并部署。安装器位于解压根目录，可先 `python3 install.py --check-only`，实际安装使用 `python3 install.py`。这是维护命令，不是驾驶启动参数。已有 allnodes/LYA/semantic 启动命令保持不变。

备份目录 `/home/control/aiformula_backups/motor_green_resume_<UTC时间>_<随机后缀>` 保存 manifest、build.log、receipt.json 和旧文件。手动回滚前应退出驾驶/semantic，另存当前修改，再仅按 manifest 恢复对应 source/build/install；不要覆盖其他包。
