# 2026-11-20 MapAssist 收尾

时间均为 JST。完整录包、原始日志和冻结代码留在实验存储，以下是场次证据和可核对的参数记录。

| 时间 | 结果 |
| --- | --- |
| 13:15 | 独立 LYA 参考速度设为 2.5 m/s。 |
| 13:52–13:58 | OA ROI 上沿设为 0.55，关闭转发器时等待 OA 退出，再发布 `/point_flag=false`。两个启动入口在隔离环境验证退出。 |
| 14:05–14:08 | A4 Lite 两次把罐车红字识别成红灯，车身静止约 11.7、2.2 秒。另一次短停缺触发帧。OA 切回视觉时有一次电机参考帧归零，车仍前进。回到学习起点附近时未闭合，状态整段首圈，未进入 detour。 [轨迹](trajectory-140401.png) |
| 14:55 | 同场原图回放后，A3 红框置信度 0.30、A4 Lite 红框置信度 0.38 装车，绿框置信度 0.23，停车点门槛 50 个红像素。两档均滤掉已标注罐车误红，A4 Lite 在一段旧包真灯的首次像素达标推迟 0.266 秒。 |
| 15:06–15:08 | 视觉 B 点在 76 ms 内横跳 1.66 m。OA 接管期间 LYA 共算出 64 条指令，其中 60 条触及转向限幅，最短前视约 4 cm。相机录包 `15_02_47` 缺失，控制定位依据车端日志。 [时序图](control-15-06.png) |
| 16:04–16:09 | MapAssist 学习约 279.11 m。最近回到学习起点约 2.70 m，状态始终首圈。16:08:14 期间红停条件已触发，电机记录 21 条手柄优先的非零 CAN。16:09:09 的红灯门控和里程计均确认停车。 [详细复盘](run-1604.md) |
| 16:42–16:44 | 将 LYA 参考速度改回 2.0 m/s、角速度限幅设为 ±0.40 rad/s。装 Neo `d1ea557` 的 OA 2.1，MapAssist 默认使用这一版本，ROI 上沿仍为 0.55。 |
| 16:57 | 隔离域以模拟障碍和里程计核对 OA 2.1 → MapAssist → LYA。LYA 输出约 1.149 m/s、+0.400 rad/s，退出后 `/point_flag=false`。 |
| 17:02、17:05 | 17:02 时车上未见 MapAssist 私有视觉点发布者，独立 LYA 收不到目标。17:05 再启动的 MapAssist 场次有 27 帧达到停车像素门槛。该段缺对应的逐帧电机门控录包。 |
| 17:21 | Extreme SSD 回读通过。16:04 MapAssist 原包和冻结运行代码已刷盘，原始转存位置及哈希见下方。 |

视觉失稳与 OA 前视过近在同一天的日志中并存。回程两次进入起点区域而保持首圈，实车没有验证 detour。OA 2.1 的接线和退出通过隔离测试，后续仍需实车验证绕障和轨迹连续性。

[17:35 实装与恢复所需归档](DEPLOYMENT.md)

## 归档坐标

- 16:04 场原包位于 Extreme SSD 的 `2026-11-20 Experiment Logs/wrap-up-1717/bags/MapAssist/rosbag2_2026_11_20-16_03_08`，包含原 SQLite、metadata、SHM 和 WAL。消息数 164,563 与 metadata 相符。
- 14:05 场完整压缩归档已于 14:33 完成 Drive 回读，文件为 [1120-1405 MapAssist](https://drive.google.com/file/d/1KUYO5ucsqz-MujwCX0HIFFCwxWc-lsO3/view)。
- OrinSSD 的 11/19 冻结运行代码及当日实装补充在同一 Extreme SSD 目录的 `artifacts/`。SHA-256 和文件名见 [SSD MapAssist 归档清单](ssd-sha256.txt)。

16:04 场的 Drive 归档名为 1120-MapAssist-160308，压缩包共 16 卷。 [Drive 收尾原包与说明](https://drive.google.com/drive/folders/1PLbIx1ZWQuzODyqZWWDAfGdixOLRV-lr) · [合并与校验](../concept-wrap-up/DRIVE-ARCHIVE.md)
