# ROS 2 录包

17 份原始场次，共 1,993,511 条消息。每个场次的目录中有原始 `metadata.yaml`、逐话题计数、起止时刻、SHA-256 和独立下载链接。附件解压后直接得到 `bag/类别/原场次名/`，其中保留了原始 `.db3`、`-wal`、`-shm` 和元数据。

| 类别 | 场次 | 判定依据 |
| --- | ---: | --- |
| [Semantic / MapAssist](semantic/) | 3 | 实际收到旧接口 map_status、motor_session，且同期有正式 MapAssist 启动日志。 |
| [Concept](concept/) | 12 | Concept 面板专用目录及会话编号。其中 15:51:42 的原包为零条消息。 |
| [其他](other/) | 2 | 上午原车 Allnodes/A3/跟踪器录包，以及 17:03 开始收录的底盘辅助包。后者与 Concept 启动重叠。 |

目录时间取自录包程序命名。下表和每场的起止时刻取自 SQLite 的第一条和最后一条消息。话题出现在 metadata 里与实际收到该话题的数据分别统计。原始车载录包保留了当时采集的传感器话题，部分包含 VectorNav/GNSS、相机识别叠图和操作状态。

在检出的当日实验目录运行以下命令可取回指定组并校验。可将 `semantic` 改为 `concept`、`other`、`all` 或完整场次名。再次执行会校验已有文件。

```bash
python3 bag/fetch.py semantic --output .
```

也可从分类目录直接下载某一份压缩包并解压。恢复后用 `ros2 bag info bag/<类别>/<场次名>` 查看元数据。[机器索引](index.json) · [原始逐包核验](../records/collection/bag-verification.tsv)
