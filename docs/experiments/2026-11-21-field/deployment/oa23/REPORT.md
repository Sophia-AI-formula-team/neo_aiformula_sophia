# 2026-11-21 MapAssist OA 2.3 车端切换

均为 JST。上游是 Neo 的 `experiment/oa-2-3-nurbs`，提交 `8839d5bdcd78a4a04227b193b778510ee9524765`。OA 2.3 继承 OA 2.2 的动态 ROI 和手柄交接，点位使用上游 NURBS 点窗。实装的 OA 两份脚本 SHA-256 与提交逐字节一致。

## 操作记录

- 10:48:59–10:49:21 回读车端状态，确认本轮 139 份有内容的日志发布到 Neo 当天实验分支。远端返回提交 `814157df8c0f701d77ebffa795031cdcd459ce05`。
- 10:49:28–10:49:37 在车端独立 overlay 构建 `launchers`、`semantic_local_planner_ros`、`semantic_vehicle_wiring`、`motor_controller` 四个上游包。新 Allnodes 入口加载同提交的 motor gate 以提供手柄时间戳，车辆底盘参数与现用版本逐字节相同。全局安装的 motor_controller 不覆盖。
- 10:49:35–10:50:08 盘点并在本机存档 B 专用 Jetson 补丁和部署回执，试读两套新入口的 ROS 启动参数。全量测试第一次和第二次缺少上游的测试夹具，随后按固定提交补齐。
- 10:50:00–10:50:46 两次隔离测试不足以判定接管。第一轮只有距离 Bool、没有红灯颜色，第二轮初始化窗口太短。第三轮补齐姿态和红灯消息，并延长发现时间。
- 10:51:08–10:51:23 隔离 ROS 域观察到红灯和距离标志触发停车、手柄按下时 MapAssist 清掉停车状态和 OA 进入 `manual_takeover`、松手后 OA 等待新鲜障碍物帧。同步运行实装 MotorGate 的独立测试，红灯挡自动命令，手柄命令取得优先，松手仍红则停止，变绿后仅放行新命令。
- 10:51:24–10:51:30 切换普通 MapAssist 和 Allnodes 的终端及桌面入口。删除 B 专用运行目录、checkout、启动脚本、字节码及桌面／终端入口，移除 NeoBag 中 3 个 `/oa_1b/` 话题，默认配置剩 42 个话题。B 历史日志作为已完成实验保留。
- 10:51:34 回读确认普通 MapAssist 固定 `oa_algorithm:=oa_2_3 manual_handoff:=true`，读到的实装 OA 脚本与上游一致。旧 Allnodes 和旧 motor gate 进程仍在运行。实际使用时先从新的 Allnodes 入口重新启动，再启动普通 MapAssist。此次验证没有启动实际车辆输出。
- 10:51:38 起确认隔离启动经父进程 SIGINT 收尾时 OA 正常退出。10:51:49 开始的手柄、停车信号和首圈定向测试共 8 项通过。

车端新部署在 `~/aiformula_runtime/2026-11-21-mapassist-oa23`。普通入口旧版链接和桌面文件放在新部署的 `previous-entries`，可回切普通 OA 2.2。MapAssist B 已卸载。

[上游提交](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/8839d5bdcd78a4a04227b193b778510ee9524765) · [构建](build.log) · [隔离交接](isolated-handoff-final.log) · [实装门控验证](motor-gate-red-override.log) · [切换与清理](cutover.log) · [实装回读](readback.log)

10:52:12 全量测试统计 80 通过、1 失败。失败项是已建图后的红灯绕行测试，夹具中的拓扑融合报 `solver_budget_exhausted`。10:52:16–10:52:25 在同一车端安装环境单独重跑该项，通过用时 5.93 秒。隔离 ROS 测试退出时 OA 正常结束，MapAssist 的 Foxy 节点记录 SIGINT 的 `KeyboardInterrupt`。

[定向测试](targeted-tests.log) · [全量测试](upstream-tests-final.log) · [绕行测试重跑](detour-test-retry.log)
