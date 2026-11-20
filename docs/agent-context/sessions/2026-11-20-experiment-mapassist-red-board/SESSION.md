# 两场 MapAssist 正面灯牌归档交接

## Objective and scope

同步两场 ROS 日志、相机叠图和正面漏检复核。当天录包原件保管在本地。本次验收针对公开的证据包和可读报告。

## Input handoff

已接收 `2026-11-20-user-mapassist-red-board`。其结论口径为正面红牌漏检需要展示，逆光 `unknown` 符合操作员预期。原始录包可按本地试验登记对照。

## Source and working tree

Neo ID `1303517209`，准备基线 `1a663bac77dde8084699dba9e0d77bca1d766ae4`，用独立干净 worktree。公共目录 `docs/experiments/2026-11-20-field/`。车端 MapAssist 1120-av314.1，早间试跑识别 A3 V14，后验对照使用 A4/A4Light。共享工作树的另一项 allnodes 修改 `008f33c` 留待独立发布。

## Work performed

09:20–09:21 JST 经 SSH 读取两场日志和关键话题。09:26 回读终端和桌面像素阈值 50。09:28–09:35 回收、核对 475 张车载叠图，按相机时标配对取出正面原图。09:36 装车回读 A4/A4Light 候选分数 0.23。09:40 后审阅并准备公开版日志、图片、时间线与实验报告。

## Evidence and result

[现场专题](../../../experiments/2026-11-20-field/mapassist-red-light/)收录 225 份 ROS 日志、475 张原车叠图、正面漏检图及回放参数。小包附 [实拍 overlay](evidence/130-vehicle-overlay.jpg)、[同相机时标原图](evidence/130-camera.jpg)及[发布索引](evidence/publication-index.json)。正面样本在新门槛的离线回放为 3/3，另外 11 张对照输出一致。`unknown` 逆光片段按预期处理。

## Risks and blockers

调参结果来自三张正面样本和十一张对照。09:36:49 仍观察到旧进程，下次现场运行的生效状态待核验。公开全量中 18 张无关近景面部已遮挡，正面关键样本保留原导出内容。

## Next agent actions

后续场次使用单独会话记录，实测前核对生效参数并记录灯牌经过的源时间、车载 overlay、`/flag_stop` 与轮速反馈。此批次的公开证据、附件散列及图像来源见日期目录。
