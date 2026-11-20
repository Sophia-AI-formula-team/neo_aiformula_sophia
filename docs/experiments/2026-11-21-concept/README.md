# 1121 实验准备 · Concept

本次只涉及 Concept。启动入口仍叫 **Concept** 和 `concept-live`。安装器从固定源码构建独立 Foxy 工作区，校验通过后替换入口，并备份车机原有的命令和图标。现场准备从[部署说明](DEPLOY.md)开始。昨天的整车准备、AV314 和 MapAssist 入口继续按[1120 交接文档](../2026-11-20-mapassist/AGENT_DEPLOY_1120.md)操作。

| 项目 | 11/21 结果 |
|---|---|
| 最低非零速度 | 默认 0.5 m/s，自动规划选零时仍发零。默认下限会排除仅验证过 −0.2 m/s 的主动倒车。 |
| LYA | 从候选路径的目标位姿算出 `v`、`w`，沿用和 Pure Pursuit 相同的约束和门控。 |
| 停车与出界 | 两种控制器在 Unity 各跑五个短场景。已有 1.82 m/s 又贴近外缘时，首帧发零仍在 0.30 秒越线。另有一道内起点 H12 选停，五秒未恢复。 |
| 延迟 | 桌面 RTX 4070 Ti，15 Hz 图像输入时，两种均收到 129/150 条新指令，约 12.9 Hz，P95 约 202 ms。5 Hz 时均收到 50/50，最长相邻指令间隔分别约 315 和 255 ms。 |
| 验证范围 | 140 项 Python 测试通过。两段 Unity 指令经 Foxy 门控和原车换算重放，各有 25/25 条双轮指令吻合。现场 Jetson、相机、物理 CAN 和整圈表现待实测。 |

[离线完整报告、刹停图和复现脚本](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/6793a13c0a1a22c127accb452ef65a206d6f415d/vehicle_concept/checks/offline1121/README.md) · [逐场指标与分段延迟](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/6793a13c0a1a22c127accb452ef65a206d6f415d/vehicle_concept/checks/offline1121/measurements.json)

安装器在隔离的 Ubuntu 20.04 / Foxy 工作站从 GitHub 检出固定 SHA，构建了 12 个 Concept 包和专用的 `launchers`。底层校验、H12 和 B1.4 哈希、单图 CUDA 推理、启动参数、`concept-live --verify-install` 及回退均已通过。临时家目录模拟了桌面环境，独立 Linux 测试还覆盖了从已装 1120 升级及回退，合计 12 个快捷方式测试。车机安装由现场执行。
