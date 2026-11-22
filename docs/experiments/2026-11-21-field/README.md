# 2026 年 11 月 21 日实车实验记录

时间均为 JST。首批车端日志于 08:39:50 采集完成。准备分支为 `experiment/vehicle-prep-20261121`，基线 `dff29d12e805f38f6fb34027b5f93789f4a59e3f`。

| 方案 | 装车版本 | 已验证 | 待实车确认 |
| --- | --- | --- | --- |
| Concept 1121 | `6793a13c0a1a22c127accb452ef65a206d6f415d` | 终端、桌面、应用菜单入口切换成功。12 个包和专用启动器构建通过，B1.4/H12 指纹和 Jetson CUDA 单图检查通过。保留两套旧工作区用于回退。 | 实际传感器输入下的规划、门控与控制输出。 |
| MapAssist B / OA 1B 0.1.2 | 上游 `dc8d540278137ddc608f9b87bcfb1149913a7e12`，实装 `c8a8a4d8be47aff06f252c7a3dd612d9c7ef9ee1` | Jetson 导入修复通过。隔离域内合成红灯置位、绿灯清除；停车信号走 B 私有话题。四个原快捷方式和两份原 MapAssist launch 指纹一致。 | 赛道上的停车位置、红转绿行为和建图匹配后的接管。 |
| OA 2.2 | `e4866962ac24ba2ed8a7ee7d99523e7c239ba76a` | 独立工作区，收到 14 帧保留输入时戳与坐标系的 `mono8` ROI，空车道输入进入静态回退。MapAssist B 验证仍走 OA 2.1。 | 真实相机与车道条件下的动态 ROI 和绕行表现。 |

## 记录与证据

- [时间线](TIMELINE.md)包含预检、版本选择、变更、失败重试与车端采集时间。
- [装车前检查](preflight/REPORT.md)及[装车与隔离验证](deployment/REPORT.md)包含详细结果、验证日志和两处现场安装补丁。
- [ROS 启动与节点日志](ros-logs/)含 08:30、08:32 的早间启动，以及安装后的隔离检查。08:32 日志属于本轮切换前的启动，不能作为 1121 新版本的路测证据。
- [录包索引](BAGS.md)供现场包编号、转移与 Drive 归档使用。

车端首个本机原始快照为 `20261121T083948+0900/vehicle-logs.tar.gz`，494,783 字节，62 个文件。SHA-256 为 `c5e373f6f51a2d2ae91a4320a6238951fa6db26506f197f68b739c48044062ca`。ROS 文件中有 18 份有内容，另有 27 份空文件由本机原始快照保留。GitHub 中发布的是脱敏文本。08:39:24 的收尾检查显示实验启动组、相机进程与录包器均已退出。

## 第一组实验回读

10:47–10:48 回读车端日志并交接四份完整 bag。详见[第一组回读报告](runs/experiment-01/REPORT.md)、[车端日志](runs/experiment-01/vehicle-logs/)与[录包索引](runs/experiment-01/SSD-README.md)。

## OA 2.3 切换

普通 MapAssist 已装车 OA 2.3，开启手柄交接。MapAssist B 的车端部署已卸载。[部署和隔离验证](deployment/oa23/REPORT.md)。

## 第一组复盘及后续改动

- [Run 3 避障与停车复盘](runs/avoidance-stop-review/REPORT.md)给出 10:30:57.378 的门控、轮速和选点状态。
- [OA 输出 11:02 实装](deployment/oa-output-1102/REPORT.md)记录固定 x 偏移移除和 A.y 半幅，以及车端哈希回读。
- [A3 V14 与 A4 Lite 离线灯牌复核](runs/light-a3-a4l-replay/REPORT.md)记录原帧、灯态丢失恢复和阈值数据。
- [OA 2.3 清障交还与手柄重置](deployment/oa23-clear-release/REPORT.md)通过本机隔离 ROS 验证，待装车。
- [LanePoints 3 点门槛与 LYA 直线试算](deployment/lane0927oa-min3/REPORT.md)包含候选源码、补丁和 ROS 验证。11:22 JST 车机仍不可达，待装车。

## 车端冻结

12:04 JST，OA 2.3 故障交还、LanePoints 放宽版和 A3 V14 饱和度 45 已在 Jetson 回读。五个现场快捷方式、安装哈希、手柄与故障隔离验证随日期标签冻结。[field-2026-11-21-oa23 冻结目录](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/field-2026-11-21-oa23/docs/experiments/2026-11-21-field/freeze) · [OA 2.3 合并 PR](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/pull/56)

## 11 月 22 日收尾归档

[共同收尾索引](CLOSEOUT.md)汇总 MapAssist 工作区与 OA 回放、下午 Concept 日志，以及已校验的录包和待取回的文件。
