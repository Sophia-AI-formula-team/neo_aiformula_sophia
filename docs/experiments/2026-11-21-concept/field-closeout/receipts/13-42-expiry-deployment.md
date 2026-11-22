# 2026-11-21 Concept 短时到期修复

时间均为 JST。

- 13:41:43 回读正在运行的 Concept `afe3227b`，找到 13:36 录包。13:41:48 向本次 Concept launch 发送 SIGINT。面板收尾耗时，launch 在 13:42:34 完成关闭。
- 13:41–13:42 回读 158,805 条诊断消息。门控有 519 条 `waiting_fresh_auto` 和 277 条挂起的 `auto_timeout` 状态。录到 969 条自动指令，其中有 395 条由规划器发出的零速指令。门控改动只处理短时采集时戳到期。
- 13:42:32 因上一次 launch 尚在收尾，首次部署检查退出。13:42:40–13:42:43 重新构建 `concept_control_gate` 并更新 `concept-live`。车端提交 `9304c85d4f8842752300d670d8dab34d129f13c2`，构建成功。原入口和门控文件备份在 `/home/control/aiformula_runtime/concept-expiry-fix.07W1m49V`。
- 新入站指令的采集时限为 0.50 s。已接纳的上一条指令允许在短时采集时戳到期后继续输出，最迟至采集后 0.70 s，并仍受最近一次接收后 0.50 s 的看门狗限制。明确零速、急停、手柄接管和其他门控故障按原路径处理。
- 按要求跳过构建后的验证和启动。

[构建回执](build-and-install.log) · [代码补丁](concept-gate-expiry.patch)
