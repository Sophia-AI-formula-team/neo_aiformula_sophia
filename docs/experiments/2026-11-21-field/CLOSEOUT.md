# 2026 年 11 月 21 日实车实验收尾

实验日期 2026-11-21。MapAssist 和 Concept 于 2026-11-22 汇总归档。以下时间均为 JST。

| 记录 | 归档内容 | 实验结论或范围 |
| --- | --- | --- |
| 上午四份原包 | [包目录、恢复说明与 SHA-256](closeout/morning-bags/REPORT.md) | 11 月 21 日已校验 Drive 全部 30 个分卷，涉及前视相机、外道转向和两份 MapAssist 场次。 |
| 12:10–12:33 MapAssist、Allnodes、YOLOP、LYA | [17 份车端 ROS 日志](closeout/mapassist-ros/README.md)和[手柄重启诊断](closeout/manual-reboot/REPORT.md) | 找到了启动和节点输出，未取得这段相对应的 MapAssist 原始 rosbag。 |
| 12:40 OA 接管计时 | [合成掩码对照及逐次结果](closeout/oa23-takeover/REPORT.md) | 动态 ROI 在合成对照中把跨门槛推迟约 210 ms。首组 ABC 被拒额外等待约 110 ms。 |
| 12:49 OA 2.3 离线回放 | [真实 bag 的重建及计时](closeout/oa23-bag-replay/REPORT.md) | 真实 bag 回放复现首组拒收约 110 ms。另一段先等约 0.72 s 才产生有效路径。重建 ROI 片段未复现 210 ms 跨门槛差。 |
| 12:58–13:15 统一车端工作区 | [构建、启动检查、清理回执和源码布局补丁](closeout/single-workspace/REPORT.md) | OA 源码归入 `perception/obsticle_avoidence`，依赖留在 `/home/dependencies`，Concept 按要求保留独立。车保持静止。 |
| 下午 Concept | [13:36、13:52、13:58 日志与逐场状态](../2026-11-21-concept/field-closeout/README.md) | Drive 有 13:52 和 13:58 完整包，以及 13:36 的 158,805 条诊断。四个数据对象均已重新下载核对 SHA-256。 |

## 目前可核对的边界

13:36 Concept 的完整 SQLite 原包还在昨天的车端清单中，大小为 518,963,200 字节。11 月 22 日的本机、SSD 与 Drive 检查均未找到它，GitHub 有元数据与 ROS 日志，Drive 有诊断导出。12:33 Concept 有诊断统计和启动日志，完整 bag 待回读。11 月 21 日 14:02:03 的最后车端快照没有活跃录包器或新增完成包，当时仍显示 Concept 主进程。11 月 22 日 SSH 超时，未能核对该快照之后的车端文件。

[总录包索引](BAGS.md) · [时间线](TIMELINE.md) · [本次发布的文件清单与摘要](closeout/evidence-manifest.json)
