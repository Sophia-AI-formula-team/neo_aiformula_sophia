# 冻结包和现场试验评价交接

## Objective and scope

将当日车机原始冻结快照作为 ZIP 发布，并用既有采集数据评定当天试验。改动限定在交接和试验归档目录。

## Input handoff

已接收 `2026-11-19-user-frozen-runtime`。输入包括 Extreme SSD 的活动代码原包、GitHub 先前回读的同一文件、四场 ROS 录包、车端部署说明及上一轮验收回执。

## Source and working tree

仓库 ID `1303517209`，实验分支 `experiment/yolop-selection-20261119`，起始提交 `d47dd04095bee74d2a90cf8e8600ee4b981247fa`。开始时独立归档工作树干净。感知、规划、电机各包的运行代码直接取自 17:34 核验的车机 tar。

## Work performed

20:30 JST 核对 SSD、先前下载原件及团队单独分发的 Concept B1.4 文件 SHA-256，生成 Unix ZIP，逐成员验证文件及链接。20:31 上传到日期 Release，20:32 取得 GitHub 附件摘要并开始公开地址回读。20:39 离线重算四场实车录包并绘制门控和轮速曲线。复算命令在日期目录 `assessment/` 下运行。

```sh
python3 measure_runs.py --bags /path/to/bag --output field-measurements.json
Rscript plot_events.R
```

## Evidence and result

[文件核对](evidence/frozen-runtime.json)覆盖 8,486 个条目及 7 个原包的硬链接目标。ZIP 大小 259,393,710 字节。复核的轮速反馈显示，首圈红灯曾使两轮归零后恢复。末场 Concept 的 54 条无效车况门控消息覆盖一段持续 5.317 秒的暂停。[四场实测](evidence/field-measurements.json)

## Risks and blockers

冻结范围依赖 Jetson 既有平台和单独分发的 B1.4 权重。常规 ZIP 解包保留 7 个硬链接别名的文件内容，原关系保存在冻结清单。整圈匹配绕行、机械起动阈值、最终版 4 m/s 实车跟踪和绝对距离精度仍待现场验证。本次分析仅离线进行。

## Next agent actions

复用 17:08 的录包检查车况节点的轮速与 IMU 时间配对。下一次实车应记录同场首圈闭合、红灯、绕行来源和轮速，低速起动另作定长测量。回读结果、报告提交和下载地址见日期目录的 `records/frozen-runtime-verification.json`。
