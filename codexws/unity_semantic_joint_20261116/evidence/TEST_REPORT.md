# 1115 joint-map 原包与 Unity 接线验证记录

日期：2026-11-16。源码分支 `feat/semantic-joint-20261115`，冻结提交 `6b71267`。本记录来自新包及原 LYA 的本机执行；未把 callback 测试当作 Unity、DDS 或实车验收。原算法和原测试均未改动。

## 版本与接线

- 两个导入包版本 0.4.0；源 ZIP SHA256 `4126492d69cca6040e7617f06c0010c78308d4bc4a41bf2516df22b264dcc527`。
- MapAssist 文件 SHA256 `9b8590588e707173d6e623f144c41e6c1dd97531f92ce4282415eb4dc7eedb5b`。
- 接线读取的 source tree SHA256 `33190441692f25cd451d9020e6f3afb6b25766c67b8011cfe7ba8dea6b21480d`。
- worker SHA256 `74940f8eb67a522150e2c1fdb2b6d7e801374007f2f4d690e8a58805da7559e9`，文件 `semantic_joint_worker.py`。
- 原视觉/LYA固定来源 commit `05f1651c8a14d56ab9a86a9767c52793403dcfdc`。MapAssist ABC 实际触发 `withoutkalman_0312` 和 `lya_0221` 原类，再由新包 `_on_controller_cmd` / `cmd_pub` 门控；输入旧 controller 数值不会被采用。

## 已完成的短接线 smoke

`smoke_handshake_01.stdout.jsonl` 与 `smoke_handshake_01.stderr.log`：启动、真实 import、YAML 参数、官方图和干净关闭通过；实际采用 `alignment_max_rmse_m=3.2`。

`smoke_callbacks_01/smoke_receipt.json`：2.30 秒，11 项通过，未触发 fusion、未运行模型/Unity。

1. 无 ABC 冷启动保持零命令。
2. `[2,0], [3,0], [4,0]` 真实 ABC 经原 LYA 得 `v=2.3 m/s`，同时测试请求中的 `controller.v=999` 被忽略。
3. 重复视觉 sequence 不增加投票、不生成新 ABC/LYA 命令。
4. 捕获过期视觉被拒绝。
5. 无新点时原包 watchdog 停止。
6. 按真实 monotonic 时间成熟的红灯投票进入 `red_wait_first_lap` 并输出零命令。
7. 绿灯投票及新 ABC 恢复原 LYA 行驶。
8. Pose2D 无 stamp 不会使旧捕获帧重新变新：运输层单独执行捕获 TTL，原 semantic 命令仍单列记录。
9. 错实验 mode 被拒绝。
10. bbox/range 协议字段被拒绝，避免混用模式。
11. 干净关闭，不声明 checkpoint 恢复。

## 原 ZIP core 测试

仅运行 ZIP manifest 实际包含的 14 个 core 测试模块，排除主分支多出的 `test_signal_approach.py`。命令及日志：`tests/archive_core_01.json`、`tests/archive_core_01.stderr.log`。数值库线程环境均为 1；测试进程串行执行；每批 subprocess 硬限 60 秒。

结果：**103 项，96 通过，7 失败；39.681 秒；未超时。** 7 个失败全部在原包自带旧 `test_route_fusion.py`，与 ZIP 自带已知结果相符：

| 失败测试 | 真实原因 | 判断 |
|---|---|---|
| `test_actual_official_map_nodes_are_observable_in_both_directions` | 新算法返回 `ambiguous_correspondence` | 原成功断言不再满足；真实官方图对应不确定性仍是实跑重点 |
| `test_actual_official_uniform_scale_is_not_a_quality_gate_or_route_scale` | `ambiguous_correspondence` | 同上 |
| `test_individual_segment_length_mismatch_uses_measured_corners` | `ambiguous_correspondence` | 同上 |
| `test_node_progress_matches_final_smoothed_curve` | `ambiguous_correspondence` | 同上 |
| `test_official_shape_changes_final_reference` | `ambiguous_correspondence` | 同上 |
| `test_excluded_turn_cannot_be_used_as_anchor` | 实际 `final_node_projection_unavailable`，预期 `node_turn_unobserved_or_excluded` | 仍拒绝不可用融合，失败是原因字符串契约变化 |
| `test_long_final_smoothing_keeps_each_node_at_its_own_corner` | 节点误差 `0.4636301568 m`，旧断言要求 `<0.3 m` | 有实际几何偏差，不能仅当作字符串或测试过时问题 |

新 `test_joint_alignment` 与 `test_route_fusion_regressions_20261115` 均通过，其中“官方图 + 合成圆角外圈保持歧义”是新测试明示预期。没有放宽判断、改写官方图或修改测试以消除失败。

## 原 ZIP ROS shim / callback 测试

仅运行 ZIP 所含 `test_launch_standalone`、`test_map_assist`、`test_map_drive_launch`、`test_map_joint`。没有把主分支多出的 `test_simple_traffic`、`test_topo_checkpoint`、`test_race_map_contract` 混进新包功能验收。

第一批证据 `tests/archive_ros_callbacks_01.json` / `.stderr.log`：unittest 在 **23.360 秒打印 70 项完成、18 个 failure 事件、1 个 error**（failure 包括 subtest），但进程仍有未结束的融合工作，达到 **60 秒硬限后被终止**。因此此批不能报通过，也不能把全部耗时说成 23 秒。

原因分析：

- `test_map_assist.py::learned_official_loop` 在 `_try_complete_lap` 后立即断言 `fusion_status.available`，新包此时只提交异步 fusion，所以大量失败实际状态是 `alignment_pending`。这份测试是 ZIP 自带旧 fixture，本次没有从主分支新添，也没有为通过而改写。
- 该旧 fixture 的 `setUp` 创建 MapAssist，却没有把 `close_fusion_worker` 加入 cleanup；连续用例产生多个未关闭的工作线程。数值库环境仍限制为 1，但原测试自身累计 async worker，不能把全批称作严格单工作线程。
- 新 `test_map_joint::test_real_fusion_attaches_graph_and_red_uses_internal_route` 在这批中等待 future 20 秒而超时。为区分真实新功能失败与旧 fixture 的后台任务干扰，下一批独立运行新 joint 与 launch 模块，没有修改源码/测试。
- `test_rejected_map_correspondence_is_odom_only_and_cannot_correct_or_detour` 期望 guard 计数 0，实际 1。新设计允许已闭合 odom reference 在官方融合未完成/被拒时继续辅助视觉；这与旧用例期待不同。仍不能据此宣布真实道路上的修正正确。

隔离批 `tests/archive_new_joint_01.json` / `.stderr.log`：**19/19 通过，2.287 秒，进程正常退出**。包含 10 个新 joint 用例与 9 个 launch 用例；真实 fusion 后红灯生成含内部节点的 route、异步不阻塞控制、待接受结果被 OA/定位状态约束、重置丢弃旧结果等通过。这解决了该新用例的隔离判定，没有抹去上一批失败记录。

所有本审计测试 CPU 回合已经结束，未再运行模型/GPU或 Unity。总体不能表述为“原包所有测试通过”。

## 供主管部署的运输接口

```text
python -B semantic_joint_worker.py --source-root <repo>/workspace/src/aiformula/perception/semantic_planning_system --vision-root <Simulator>/Assets/StreamingAssets/VisionLaneFollower --output-root <unique-run-directory>
```

`ready`、`tick`、`shutdown` 都带 `experiment_mode=semantic_joint_1115` 和随机 `worker_boot_id`。`ready` 记录源码/配置/视觉 adapter 哈希和实际参数；`shutdown.clean=true` 仅表示接线清理完成，`fusion_pending_on_close` 另行记录，没有 checkpoint 成功含义。

启动 gate：文件及固定原 LYA provenance 哈希相符，完整 YAML 成功读取，真实 MapAssist/原 LYA 类可构造，output-root 不覆盖已有 run。实际运行 gate：tick/视觉 sequence 单调、epoch 固定、sim time 不倒退、输入数字有限、视觉捕获 TTL、原生光色投票/ABC/odom/controller watchdog、原视觉 corridor 来源/期限。门控原因不冒充新包状态：`status` 保留新包原值，额外运输限制写 `transport_gate_reason`。

控制输出：`selected_points` 是 MapAssist 的真实输出 ABC；`target` / `controller_v,w` 来自原 LYA；`semantic_v,w` 是原包真实 `cmd_pub`；`v,w` 仅可能由捕获 TTL/corridor 运输约束进一步置零。`command_new` 是原包真实 publish，`command_gate.raw_command_new` 是本 tick 真正生成的新 LYA 命令；轮询缓存不能被当成新命令。

`--output-root` 生成 `ready.json`、逐 tick 的 `semantic_joint_ticks.jsonl`、`final_state.json`。每个 tick 有 UNIX 秒 `wall`、Unity `sim_time`、实际 `odom`、完整原 `status` 和只读 `map_data`。地图含首次闭合 odom reference、当前 reference、可信掩码、学习样本、图节点边、detour 路径/进度；未对齐官方图明确标 `official_unaligned`，已映射内部节点明确标 `internal_estimate`。

## 仍需单独观察的原包问题

详见 `WIRING_ANALYSIS.md`。最关键的是分开报告 odom 闭合、官方 topo 融合、实际内部路线运动。原包红灯确认后立即停止，而绕行只能在路口 3 m 范围内启动；远处先见红灯可能形成 `red_wait_near_junction` 等待。新包也没有 checkpoint 恢复功能；本次不补旧 stop_wait/bbox/checkpoint 模式。

本报告不意味着这些问题已在 Unity 复现或已修复。Unity 完整两圈结果和视频由主管另行附加。

## inner_joint_01 实跑后的接线回放与诊断修补

首场实际 Unity 尚未完成起步：行程 0.00168 m，0 圈，未红灯/绕行。第二个新鲜真实视觉帧触发 exit 1，根因是 C# Joint 模式遗漏 `vision_commit`；见 `CRITICAL_ISSUES.md` 第 4 项。第一帧因冷启动年龄 12.03 秒被拒绝，第二帧实际推理 23.83 ms，不能将该失败归因于 topo 算法或长期模型超时。

真实第二帧 candidate 回放到原冻结 worker：缺 commit 精确复现来源异常；仅补真实 commit 则接受，原 LYA/semantic 输出 `v=2.2248456404,w=0.6740196235`。证据 `inner01_replay/receipt.json`。C# 实际修复由接线负责人完成，本审计未更改 C# 或 semantic source。

主管另授权为 worker 加异常落盘，补 `error.json` 和白名单 `failed_request.json`，避免 follower 状态覆盖掩盖异常。旧 worker 已备份，新 SHA256 **`cff1e44ffd6fe5fc2184c6659e933277a6db79454c1f7c04afe23f5ac8591674`**；以上第一版 SHA 及原 smoke 证据仍保留，不覆盖历史。`inner01_replay_diagnostics/receipt.json` 再次证明缺 commit 失败且两个诊断文件存在、补准确 commit 通过且命令完全相同。原 semantic source/source tree 哈希不变；此处是接线诊断修补，不是算法修复。
