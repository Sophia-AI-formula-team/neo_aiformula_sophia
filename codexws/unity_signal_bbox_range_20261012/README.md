# 灯牌框高估距试验 0.1.0

仅替换 Unity RoadDetector 的 distance 来源，semantic 包不改。默认关闭；启用后使用当前已确认的发光面框，按 `Z = 72.756003892 / height_px` 计算相机光轴深度，再减现有车头偏移0.216m。原框检测、颜色确认、LLP/LYA、制动、避障、急停、时序、地图和左转功能不改。

| 配置 | 初值 | 作用 |
| --- | ---: | --- |
| 图像 | 640×360 | 与 neo nHD 内参匹配 |
| 最小宽/高 | 4px / 4px | 任一不足就无 distance，绝不保留上一帧 |
| 到达框高 | 8px | `height >= 8` 输出 `target_reached=true` |
| 对应车头距离 | 8.8785m | 同一尺寸公式的预测值 |
| 框高量化误差 | ±1px | 仅量化区间，不是完整误差界 |

数据来源：neo `workspace/src/aiformula/vehicle/config/zedx/camera_params/SN48442725/intrinsic_nHD.yaml` 的 `fy=254.391622`，Unity `DisplayFace.asset` 的高0.286m。根、显示面prefab缩放为1，保存的Demo信号实例没有缩放覆盖。Unity传输内参经过float32，匹配容差仅2e-5px。外参保留本仿真已有配置；neo实车 `extrinsic.yaml` 与仿真外参不同，此模块不可未经标定直接部署实车。

**实际控制仍消费 distance/uncertainty。** `target_reached` 是本帧距离结果的附加标志，未替换 semantic 的制动状态机。红灯但距离缺失仍沿用原故障停车行为。因此此版本不承诺小框红灯继续行驶，也不以静态像素阈值宣称停车/左转已通过。完整原因见 [PROBLEM_ANALYSIS.md](PROBLEM_ANALYSIS.md)。

当前状态：2026-10-12 14:45 JST，18项标准库单测全部PASS（0失败/0skip）；单次primary 0.291s，Job清空且父/子独立确认退出，CPU已归还。完整具名ledger、源码前后SHA及进程回执见evidence/tests_01。未安装到共享Simulator，安装往返和MCP实跑仍待独立窗口；Unity仍由另一对话持有。历史PR21、final63及其证据保持原样。测试后仅文档/manifest状态与证据归档更新，运行时源及参数不变。

本目录为Unity验证接线归档，有COLCON_IGNORE，不属于原生ROS包。原生ROS capture relay仍使用其原有distance输入契约，未注入此试验。
