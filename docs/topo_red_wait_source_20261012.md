# Topo / MapAssist 简易红灯等待源码更新

车辆已同步 12 个源码、配置、测试和说明文件，MapAssist 源码版本 0.2.1；默认 `traffic_policy=stop_wait`，按距接近关闭，红灯不触发拓扑绕行。Topo 记录、闭环和融合算法未修改。

验证：本地 167 项单元/回调/启动描述桩测试通过；车上 Python 3.8 的 11 项新策略回调测试通过；Python 3.8 语法和 diff 检查通过。这些不是实时 DDS 或车辆行驶验收。

本次未编译、未启动节点，未改 allnodes 或 motor_controller。车上安装的 semantic 仍是旧版。两个 launchers 安装文件原来指向源码，现保存为内容完全相同的普通文件，避免旧安装空间提前加载新入口；原链接保存在备份中。build 中 package.xml 的源码链接随源文件显示新版本，但构建目录普通文件未改变。

- 备份：`/home/control/aiformula_backups/20261012_topo_red_wait_source`。
- 清单和结果：`/home/control/aiformula_runtime/topo_red_wait_source_20261012/{manifest.json,result.json,pinned_links.json,python38_tests.log}`。
- 回退：先核对后续修改，恢复备份中的 `workspace/src/...` 文件，删除清单里原本不存在的新增文件，再恢复 `installed_links` 中两条旧链接；保留备份。
- 后续若用户要求编译，需同时更新 `semantic_local_planner_ros` 与 launchers，避免策略参数与旧入口混用。不要仅重启旧安装版并声称新逻辑已生效。

当前 Sigmoid motor_controller 的零 Twist 会产生非零轮速，新策略保持 shadow 输出；此次没有修复或验收实车停车。
