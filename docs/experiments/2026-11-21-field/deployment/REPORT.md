# 2026-11-21 装车记录

时间为 JST。车机 ROS 2 Foxy / aarch64，三个候选均按 [08:32 装车前检查](../preflight/REPORT.md) 固定版本部署。静态验证使用隔离 DDS 域，动力链未启动。

## MapAssist B / OA 1B 0.1.2

- 08:35:17 确认无驾驶启动组和录包器，记录原 `allnodes`、`yolop`、`mapassist`、`lya`、`concept-live` 的 SHA-256。MapAssist B 用独立源码与工作区，两份原版 MapAssist launch 指纹均匹配。
- 08:36:16–08:36:37 原版 `dc8d540` 构建后，Jetson 系统 Python 的 `cv_bridge_boost` 初始化失败。实测预先导入 OpenCV、NumPy 或预载旧环境的系统库均能解决。采用本地提交 `c8a8a4d8be47aff06f252c7a3dd612d9c7ef9ee1`，仅调整 B 包里的导入顺序。[最小改动](vehicle-logs/mapassist-b-0835/jetson-cv-bridge.patch)
- 08:36:48 新 `mapassist_b` 针对 Jetson 的独立发行安装通过。标签和终端命令为 **1121 MapAssist B** / `mapassist_b outer`，内圈用 `mapassist_b inner`。四个原命令和两份 MapAssist 旧启动文件哈希一致。
- 08:37:03 在 `ROS_DOMAIN_ID=74`、`ROS_LOCALHOST_ONLY=1`、`vehicle_output:=false` 试跑。`/map_assist` 订阅 `/oa_1b/traffic_light/state`、`/oa_1b/flag_stop` 和原 `/original_processed_point_a`。现有 OA 2.1 的 `/semantic_drive/map_assist_045/oa_triplet` 发布者正常。用同时间戳的合成图像与识别消息确认红灯布尔值置位、绿灯清除。实验节点未向公共 `/flag_stop` 或物理电机话题发布。08:37:23 已确认两个真实入口会拒绝占用同一域的同一把锁。

[安装和指纹](ma-b-jetson-install.log) · [隔离接线结果](ma-b-jetson-smoke.jsonl) · [互斥回执](ma-b-mutual-exclusion.log)

## OA 2.2

- 08:35–08:37 在 `~/aiformula_runtime/2026-11-21-oa22/e4866962ac24ba2ed8a7ee7d99523e7c239ba76a` 独立构建 `launchers` 和 `lane_line_publisher`。aarch64 二进制包含 `dynamic_roi_mask`，`obstacle_avoidance.launch.py` 可选 `oa_2_2`。MapAssist B 仍选择其原 overlay 的 OA 2.1。
- 08:37:22 第一轮隔离测试缺相机到车身的 TF，lane publisher 因缺变换退出。08:37:43 补合成 TF 后，实装 `lane_line_publisher` 与 OA 2.2 成功连接，核对 14 条与输入源时间戳和坐标系一致的 `mono8` 消息。空车道输入触发静态 ROI 回退，组合避障点话题存在，未启动电机节点。实际动态车道与障碍绕行仍待现场验证。

单独选择 OA 2.2 时，使用其 `install/local_setup.bash`，并在同一套 overlay 启动车道发布器与 OA 2.2。独立避障入口为 `ros2 launch launchers obstacle_avoidance.launch.py algorithm:=oa_2_2`。

[构建和安装](oa22-install.log) · [隔离接线结果](oa22-smoke-tf.jsonl)

## Concept 1121

- 08:35–08:38 以 `6793a13c0a1a22c127accb452ef65a206d6f415d` 构建 `~/concept_1121_ws`。迁移私有 Python 依赖，保留两套旧工作区和源码。12 个 Concept 包和专用 `launchers` 编完。原底层、电机文件、实装 overlay、H12 与 B1.4 指纹、Qt/Foxy 导入及 Jetson CUDA 单图测试通过。
- 08:38:55 首次替换时，准备分支未识别 11/20 16:17 修过的终端和桌面入口。核对车端与 11/20 回执两种字节摘要完全一致，只把这两种已有现场摘要加入迁移工具。08:39:10–08:39:15 完成终端、应用菜单和 XDG 桌面三入口切换，`concept-live --verify-install` 通过。旧入口的按文件备份位于 `~/.local/state/aiformula/concept-shortcuts-1121/backup-n9ac_kvk`。保留 `~/concept_v6275_ws` 和 `~/concept_1120_v6278_ws` 以便回退。退回时用车端 `1121-deploy-records/concept-preserve-0835/experiments/2026-11-21-concept/shortcuts.py --rollback`。
- 08:39:15 复核固定提交、备用工作区和原 MapAssist 的 6 项指纹。MapAssist B 的入口只需在 XDG 桌面核对，第一次查应用菜单位置返回缺文件。08:39:24 在实际桌面确认 **1121 MapAssist B** 和 **Inner** 动作，同时确认 **Concept** 桌面入口。最终进程清单无实验启动组、相机或录包器。

[旧版保留改动](concept-preserve-legacy.patch) · [构建与 CUDA 核对](concept-install.log) · [快捷方式切换](concept-switch-retry.log) · [桌面与待机复核](desktop-and-standby.log)
