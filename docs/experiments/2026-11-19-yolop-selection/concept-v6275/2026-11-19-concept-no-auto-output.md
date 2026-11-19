# Concept 实车无自动输出排查

以下时间均为 2026-11-19 JST。

| 时间 | 操作与结果 |
| --- | --- |
| 12:38:21 | SSH 重试成功。定位 10:58:43 的 Concept launch 和唯一对应的实车录包。 |
| 12:40–12:41 | 用 SQLite 只读分析录包，核对规划拒绝原因、控制门与轮速状态，并对照部署版本的代码。 |
| 12:41 | 检查车端进程，未发现 Concept launch 或其已安装节点进程。 |
| 12:42:47 | 开始将排查记录补入当天实验索引和 ZIP。 |

车端 launch 日志为 `~/.ros/log/2026-11-19-10-58-43-654258-control-desktop-24328/launch.log`，记录覆盖 10:58:43–11:30:34。车端录包目录为 `~/rosbags/concept/concept_20261119_105925_48d1725036ca4176a2d8e6d3c1e16cf1`，消息覆盖 10:59:26.371–11:03:00.188。

| 环节 | 录包证据 |
| --- | --- |
| 识别 | `/concept/perception/overlay/compressed` 有 288 帧。帧到达录包器时距相机时间戳 0.810–0.974 秒，中位数 0.882 秒。 |
| 规划 | `/concept/decision` 共 288 条，全部为 `invalid`，原因为 `image is stale or its stamp is in the future`。图像年龄全为正，范围 0.804–0.953 秒，中位数 0.876 秒，P95 为 0.928 秒。当前上限为 0.500 秒。 |
| 相机几何 | 288 条决策均为 `unverified-resize`。即使图像足够新，该实车配置下规划也只会进入 `shadow`，仍需验证后的 `calibrated-v626` 才能发布自动指令。 |
| 自动控制 | `/concept/auto_cmd` 0 条。控制门有 808 条 `auto / waiting_new_auto` 状态，速度和转向都是零。首次记录到该状态的时间为 11:01:22.175。 |
| 手动控制与反馈 | 控制门有 132 条非零手动指令状态。录包有 640 条非零内部电机请求、1,245 条非全零内部 CAN 帧及 349 条至少一侧非零的有效轮速反馈。非零轮速覆盖 10:59:42.059–11:00:45.807。 |

现有录包可确认自动指令断在规划输入的图像时效检查。它未包含最终电机 CAN 话题，也没有采集到足以拆分相机、排队、预处理和推理耗时的分段时间戳。录包窗口以外的自动输出无法由此验证。本次排查仅在车端读取记录和进程状态。

部署代码固定于 Neo 提交 `a9d7d7a94d062ad5ff61fedfe21d3dbe94fa1bd1`。[规划节点源码](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/a9d7d7a94d062ad5ff61fedfe21d3dbe94fa1bd1/vehicle_concept/src/concept_drive/concept_drive/node.py)
