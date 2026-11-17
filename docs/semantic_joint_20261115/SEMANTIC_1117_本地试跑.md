# 1117 新版语义地图：本地匹配试跑

## 结果

旧版0.4.0已在 GitHub draft PR #38，远端 HEAD `3e2cf683fd1d649c8362282500c445aa50106023`；尚未合并。新包本次只在本地独立分支 `feat/semantic-1117-local` 整理和试跑，没有上传。

新版本0.4.1原生匹配逻辑在 Unity MCP 实跑中对内圈、外圈均报告 `accepted`，均输出5个外圈节点。首次内圈试跑因视觉选点失效在42.107 m停车，未完成首圈，不能判断地图匹配；同配置重跑后首圈完成并匹配。

| 运行 | 实际行驶与结束 | 匹配状态 | 节点到参考线RMSE | 官方节点数 |
| --- | --- | --- | ---: | ---: |
| 内圈第一次 | 42.107 m，0圈；视觉选点持续不可用 | 未产生匹配结果 | — | 0 |
| 内圈第二次 | 322.846 m，首圈闭合后按匹配任务要求停止 | accepted | 0.175 m | 5 |
| 外圈 | 334.427 m，首圈闭合后按匹配任务要求停止 | accepted | 0.282 m | 5 |

RMSE是包内候选节点对里程计参考曲线的拟合误差，**不是独立测量的实际路口位置误差**。两次 successful matcher 的 `field_validation_required=true`；内部节点仍由官方图和外圈锚点推算，未独立确认内路位置或测试红灯左转。由于沿用旧的自动试验工具，匹配得到后才停止，期间其自动灯光事件仍发生；本次没有把灯光行为当验收目标。

## 新旧规则差异

供应包将 `fusion_candidate_tolerance_m` 标明为诊断阈值，候选位置/评分分歧不再直接阻止选择；增加 `geometric_seam_v3` 和分支角度约束。外圈本次最佳与替代评分差为0.25014，小于旧版0.3门槛，但新版仍接受；这符合新版不同规则，不能单凭结果证明原来的歧义已经被独立真实路口观测消除。当前分支角度使用官方内外连接与里程计切线计算，不是额外采集的真实路口分支识别。

## 来源与变更

- 用户原包：`C:/Users/PC/Desktop/1117(1).zip`，SHA256 `0ef3263db5a4997ff6d9b2a1eefbb7874cd9d5c667ca0325f6e9017fa0c48aa1`；原包及全部解压文件保留在下方试验目录。
- 57个供应方文件放入仓库标准位置 `workspace/src/aiformula/perception/semantic_planning_system/`。其中4个版本字段从0.1.0升为0.4.1，其余供应文件与原ZIP逐字节一致；未修补新包算法。
- 仓库原有但ZIP未提供的历史模块、测试和文档继续保留，不当作供应包新功能。可选的 `lane_points_addition` 保留于原包解压目录，没有覆盖现有视觉源。
- 保留既有Unity接线、摄像机、灯位、物理参数、LYA、官方图与感知权重。独立本地worker从本工作树加载新包；两次成功运行的全部冻结输入运行前后哈希一致。

## 检查结果和问题

- 新包 `test_branch_angle.py`：8项通过。
- 新包 `test_geometric_alignment.py`：7个测试方法，报告6个failure事件（包括参数化子测试）；实际返回 `branch_connections_missing`，与测试期待的匹配、预算耗尽或歧义结果不一致。原因是这些测试输入没有满足新版需要的分支连接；尚未判定是测试夹具过时还是接口设计缺陷。完整堆栈保留，未改测试或源码来消除失败。
- 内圈第一次：感知选点大部分为 `default_straight` / `no_observed_triplet`，既有超时机制停车；根因尚未确认。不得因为第二次通过就抹去此失败。
- 三场 worker 均干净退出；Unity回到干净 Edit 状态，渲染设置恢复 `targetFrameRate=-1`、`vSync=1`，物理步长恢复0.02 s。
- 本次是Unity仿真、理想仿真里程计与ROS消息shim，不是原生ROS/DDS、实车、重启恢复或真实内部道路精度验收。

## 日志与实际输出

试验目录：`E:/Mess/Projects/Programming/aiformula/workspace/semantic_1117_trial_20261117/`。

- `final_match_summary.jsonl`：三场结果、实际节点坐标、匹配原因与指标。
- `runs/inner_1117_01/`、`runs/inner_1117_02/`、`runs/outer_1117_01/`：每场原始事件、状态、第一视角、输入哈希、结果及 `worker/semantic_joint_ticks.jsonl` 完整语义输出。
- `inner_1117_01.runner.log`、`inner_1117_02.runner.log`、`outer_1117_01.runner.log`：运行日志。
- `geometric_alignment_tests.log`、`branch_angle_tests.log`：原测试完整输出。
- `import_manifest.json`、`source_verify.json`、`version_changes.json`：原包来源、逐文件哈希和版本字段说明。
- `editor_initial.json`、`editor_restored.json`、`editor_console.json`：MCP状态和恢复证据。

建议后续先给测试夹具补充分支连接并核对原期望是否符合供应商新版契约，再针对竞争候选/真实路口位置做独立地面几何验证。不要通过修改门槛或手工地图位置把accept当成实际路口精度通过。
