# 原始运行日志

[在 GitHub 浏览 926 个车机 ROS 日志文件](ros/) · [下载同一批原文件](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/releases/download/field-archive-2026-11-19/vehicle-ros-logs.tar.gz) · [105 个 launch 会话及其启动进程](launch-sessions.tsv)

其中 105 个文件是各启动会话的 `launch.log`，另有 821 个节点或进程日志。大小为零的启动记录也保留；单凭空日志无法判断退出原因。检查某次运行时，先按启动目录的日期时间选会话，再在表格中读取进程名称和 PID，并与同时间的节点日志及 bag 核对。

| 现场问题 | 对应记录 |
| --- | --- |
| 15:33 红灯未停、A3 像素和门控交接 | [终端摘录、现场记录、隔离复核](../records/workstation/concept-followup-20261119/semantic-red-1533/) |
| 16:10 首圈没有闭合证据，裸 OA 未进入 MapAssist 选择器 | [终端可见区、节点统计和 OA 对照](../records/workstation/concept-followup-20261119/semantic-lap-1620/) |
| 16:47 两次点击快捷方式只写出初始化日志 | [启动时序、占用检查和后续进程退出](../records/workstation/concept-followup-20261119/semantic-shortcut/mapassist-startup/) |
| 16:50–17:02 后续正式接线 | [三份 Semantic 录包](../bag/semantic/) |
| 16:35 起步、转向、失锁与同机负载 | [现场 trace、软件修复和回放](../records/workstation/concept-followup-20261119/concept-1635/) |
| 17:04–17:08 后续 Concept 门控和决策 | [各场逐话题计数与状态分布](../bag/concept/) |

车机部署目录、终端可见区、备份、构建输出、源码及其他收集文件按原路径另行归档。本机保留的日志、图表、源码差异和临时工作包亦可整包下载。[车机记录](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/releases/download/field-archive-2026-11-19/vehicle-records-and-changes.tar.gz) · [工作站记录](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/releases/download/field-archive-2026-11-19/workstation-records.tar.gz) · [逐文件清单](../records/files.tsv)
