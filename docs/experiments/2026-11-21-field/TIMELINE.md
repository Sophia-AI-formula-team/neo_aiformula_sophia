# 2026 年 11 月 21 日操作时间线

时间均为 JST。带秒的条目取自日志或带时间的回执。只精确到分钟的记录沿用原记录精度。

| 时间 | 操作与结果 | 回执 |
| --- | --- | --- |
| 08:30:28、08:32:43 | 车上留有两次切换前的启动日志。后一份包含 ZED、手柄、VectorNav 与车道发布节点启动信息。 | [08:30](ros-logs/2026-11-21-08-30-28-471534-jetson-2352/launch.log) · [08:32](ros-logs/2026-11-21-08-32-43-718505-jetson-3300/launch.log) |
| 08:32:49–08:32:57 | 核对 Neo 仓库、准备分支和三个固定提交，确认 `oa1b-1121` 指向兼容回滚，选择 `mapassist-b-1121`。核对车机 ROS 2 Foxy、aarch64、可用磁盘和运行进程。 | [装车前检查](preflight/REPORT.md) |
| 08:33:35–08:34:03 | 核对 B1.4 权重哈希、本田投影文件和现场保存速度。复现旧 Concept 清理预检发现未追踪本地文件，改走独立工作区。MapAssist B 单测第一次缺 NumPy，换已有 Python 3.12 环境通过。OA 2.2 离线 ROI 检查通过。 | [旧版预检](preflight/legacy-preflight-on-vehicle.log) · [单测](preflight/test-mapassist-b-python312.log) · [ROI](preflight/test-oa22-roi.log) |
| 08:35:17 起 | 确认无驾驶组和录包进程，记录入口指纹并开始独立安装三个候选。 | [安装前回执](deployment/preflight-now.txt) · [部署报告](deployment/REPORT.md) |
| 08:36:16–08:36:48 | MapAssist B 第一次构建后遇到 Jetson `cv_bridge_boost` 初始化失败。定位为导入时序，应用 B 专用补丁并安装。 | [诊断](deployment/ma-b-cvbridge-diagnosis.log) · [补丁](deployment/vehicle-logs/mapassist-b-0835/jetson-cv-bridge.patch) · [安装](deployment/ma-b-jetson-install.log) |
| 08:37:03–08:37:23 | MapAssist B 在隔离域验证合成红绿灯、B 私有停车话题、原车道输入和 OA 2.1 发布者。检查同域入口互斥。 | [话题与灯态](deployment/ma-b-jetson-smoke.jsonl) · [互斥](deployment/ma-b-mutual-exclusion.log) |
| 08:37:22–08:37:43 | OA 2.2 首轮测试缺相机到车身 TF，lane publisher 退出。隔离域补合成 TF 后重试，收到 14 帧 ROI 并核对静态回退。 | [首次测试](deployment/oa22-smoke-without-tf.jsonl) · [补 TF 后](deployment/oa22-smoke-tf.jsonl) |
| 08:38:55 | Concept 首次切换发现旧快捷方式为 11 月 20 日现场修订版。以旧回执确认两种文件摘要，再扩充安装器允许迁移的摘要。 | [现场补丁](deployment/concept-shortcuts-field-1120.patch) |
| 08:39:10–08:39:24 | Concept 三入口切换并验收。再次核对 MapAssist B 桌面入口及内圈动作，完成进程待机检查。 | [Concept 切换](deployment/concept-switch-retry.log) · [待机](deployment/desktop-and-standby.log) |
| 08:39:41 | 盘点 ROS 日志、装车回执及车端已知 bag 根目录，当日录包目录未发现条目。 | [录包索引](BAGS.md) |
| 08:39:48–08:39:50 | 采集车端第一份原始快照到本机，验证归档可读并记录 SHA-256。 | [快照摘要](README.md#记录与证据) |
| 08:40 | 整理构建、变更和运行结果，审核可公开的日志副本，保留原始归档。 | [预检](preflight/) · [部署](deployment/) · [ROS 日志](ros-logs/) |
| 08:40:50 | 尝试重新核对两版 MapAssist 的 XDG 桌面入口和当日录包清单。SSH 返回不可达，记录保留 08:39 的确认范围。 |
| 08:41:06 | 第二次尝试核对桌面两个 MapAssist 入口，SSH 仍不可达。11 月 20 日车端回执确认过旧版桌面图标，当前是否同时显示两版需在线复核。 |
