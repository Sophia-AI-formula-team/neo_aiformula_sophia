# 2026-11-20 联合收尾归档

时间均为 JST。Concept 和 MapAssist 的场次、报告和归档位置在同一实验目录。

| 内容 | 归档 |
| --- | --- |
| 7 个 Concept 原包和 1 个 MapAssist 原包 | 372,234 条 Concept 消息和 164,563 条 MapAssist 消息。Drive 上的 27 个数据文件与分卷已逐个读回 SHA-256。 [录包索引](BAGS.md) · [Drive 原包与说明](https://drive.google.com/drive/folders/1PLbIx1ZWQuzODyqZWWDAfGdixOLRV-lr) |
| 当天实装代码 | Concept V6.27.8 源码及安装、MapAssist 0.4.5 实装运行树、当天底层增量、A3/A4/A4 Lite 识别运行代码与 B1.4 权重。 [Drive 车机代码与日志](https://drive.google.com/drive/folders/1vy8gy2CsctWa9RUD19sabfESZA4Y7ACJ) |
| 日志、部署回执与实验材料 | ROS 和构建日志、部署与回退记录、末两场 Concept 参数及 MapAssist 15:06/16:04 的补充诊断。 [Drive 车机代码与日志](https://drive.google.com/drive/folders/1vy8gy2CsctWa9RUD19sabfESZA4Y7ACJ) |
| 11 月 19 日冻结底座 | 配合 11 月 20 日实装增量恢复。Drive 底座包为 00–02 三卷，拼接后 143,445,917 字节。 [Drive 分卷与合并说明](https://drive.google.com/drive/folders/1vy8gy2CsctWa9RUD19sabfESZA4Y7ACJ) |

## 回放与恢复

- 17:08 LYA 和 17:13 Pure Pursuit 只有初步统计，完整原件保留于 SSD 和本机。17:08 的数据库、75,124,112 字节的 WAL、SHM 与 metadata 同目录保存。 [Concept 场次及原件校验](concept-wrap-up/CONCEPT-RUNS.md)
- MapAssist 16:04 的实录早于当日 OA 2.1 部署。16:57 是后来接线的隔离验证。 [MapAssist 实录与复盘](mapassist-wrap-up/)
- 14:09 Concept 与 15:02:47 MapAssist 的完整 ROS bag 仍缺。Drive 保存 14:09 的诊断归档和本次补传的 MapAssist 控制证据。 [14:09 诊断归档](https://drive.google.com/file/d/1oqAnv4tuIcW4ZkbB9UdLEfY-ZCksPLwy/view)

20:42 核对 MapAssist 实装与部署回执。20:43:20 完成代码与日志目录 14 个文件的逐项 SHA-256 回读，共 215,592,811 字节。 [MapAssist 实装回读](mapassist-wrap-up/DEPLOYMENT.md) · [16:57 与 17:05 证据](https://drive.google.com/file/d/17GA2htzSqv6ioqT3v_goTjdAkm-HWgzD/view)
