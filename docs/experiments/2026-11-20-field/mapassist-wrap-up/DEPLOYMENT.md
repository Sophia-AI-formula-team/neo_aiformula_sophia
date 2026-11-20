# MapAssist 0.4.5 收车实装回读

2026-11-20 JST。以下为 17:35 从车机回收的 MapAssist 运行树及底层安装件。回读文件与 14:55、16:42 和 16:44 的部署回执相符。

| 项目 | 实装回读 |
| --- | --- |
| 启动 | `allnodes`、`mapassist`、`lya` 各有入口。`allnodes` 使用 `lane_0927oa` 并禁用内置 YOLOP A3，感知识别器有独立选择入口。`mapassist` 默认外圈，启动 OA，LYA 由独立入口启动。 |
| 交通 | MapAssist 默认 50 个红像素发布停车位置标志 `/flag_stop`，守护转接把交通停等状态发到旧 `/semantic_drive/map_status`，供 `stop_wait` 使用。连续非红 1.0 s 放行。A3 与 A4 Lite 的红框置信度分别为 0.30 和 0.38，绿框均为 0.23。 |
| OA | MapAssist 默认 `oa_2_1`，来自 Neo `d1ea557426f3d7ea0d8ddb7390ad47a867cbf17e`，ROI 上沿比例 0.55。16:57 隔离测试验到 OA 三点经 MapAssist 进入 LYA，退出后 `/point_flag=false`。 |
| LYA | 16:42 安装版参考速度 2.0 m/s，角速度限幅 ±0.40 rad/s，与源码回读一致。 |

16:04 是 OA 2.1 装车前的道路实录，16:57 是禁用电机输出的隔离测试。17:05 的启动日志保留交通判定，但没有同期逐帧电机门控录包。16:04 完整实录确认该场首圈未闭合、一次红灯期间仍有手柄优先输出，以及一次红停后里程计确认车辆静止。[16:04 场次证据](run-1604.md)

## 实装恢复资料

当日归档目录为 Extreme SSD 的 `2026-11-20 Experiment Logs/wrap-up-1717`。恢复实装需要 11/19 冻结底座、11/20 运行树和底层实装增量。17:35 的安装文件与当时实验分支源码存在差异，例如 `semantic_045_commissioning.launch.py` 和 LYA 安装件。现场恢复核对以下压缩包与文件指纹。

| 归档相对路径 | SHA-256 |
| --- | --- |
| `artifacts/vehicle-code-final/vehicle-current-code-20261120.tar.gz` | `929d615ca48b762f528b5062a6409b8f976065dd7b01f64e4042336e29b42ca2` |
| `artifacts/vehicle-code-final/vehicle-underlay-and-perception-20261120.tar.gz` | `24b034a2492ebce523388281b30b47a2933ccea2f21de3a0375776539d80808a` |
| `artifacts/frozen-active-code/2026-11-19-vehicle-active-code.tar.gz` | `3ee71ae49b45ea78d676c5ada5b2bc66111051112e32025234d5360fca3a182e` |

| 实装文件 | SHA-256 |
| --- | --- |
| `semantic_045_commissioning.launch.py` | `bf2246348605be59552e0f71a34125f0a8f2bccca6d3cbb0edde1f4277d6fcc1` |
| MapAssist 安装树中的 `oa_2_1.py` | `5f0cde0f0ddc7fee7dd87542c9d6a85783875241ddaf6df800cfe1d36286111d` |
| `lya_follower_connected_omegat_global.py` | `b4f133d7f57e2b292b005a86133961db373948017c0440895bd9b336d4e42576` |

16:04 的完整录包也已在 Drive。按说明使用 16 个分卷恢复。[Drive 收尾目录](https://drive.google.com/drive/folders/1PLbIx1ZWQuzODyqZWWDAfGdixOLRV-lr) · [分卷与校验](../concept-wrap-up/DRIVE-ARCHIVE.md)
