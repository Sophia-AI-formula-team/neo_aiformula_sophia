# RESULT_READY · 当日冻结包及实车评价

2026-11-19T20:55:18+09:00（日本时间），已对 Neo 实验分支完成第二次归档验收，仓库 ID `1303517209`。[完整冻结运行 ZIP](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/releases/download/field-archive-2026-11-19/2026-11-19-vehicle-frozen-runtime.zip) · [当天试验总评](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/ccb72d5096bb771db5ab77e05d90117720dfd9ad/docs/experiments/2026-11-19-yolop-selection/ASSESSMENT.md) · [新版离线图文报告](https://raw.githubusercontent.com/Sophia-AI-formula-team/neo_aiformula_sophia/ccb72d5096bb771db5ab77e05d90117720dfd9ad/docs/experiments/2026-11-19-yolop-selection/2026-11-19-field-report.zip)

| 核验项目 | 可复核结果 |
| --- | --- |
| 车机原件到运行 ZIP | 17:34 冻结快照的 8,486 个成员均与 ZIP 核对，涵盖 60 个工作区安装版本及 152 个已安装可执行项。7 个硬链接别名在 ZIP 中有相同文件内容。 |
| 运行 ZIP 公开下载 | 2026-11-19T20:45:21+09:00 完成公开地址整包回读。共 259,393,710 字节，SHA-256 `93fb34a5f07eecbbd53a1f90d6ace9e399210ae591edb943f77c2c501c8bfc9e`，与本地及 GitHub 服务端一致。 |
| 离线图文报告 | 1,786 个归档文件逐一对照源提交的 Git blob。公开下载 13,534,263 字节，SHA-256 `6782b955438645ae31fc0b7c833ad471858f93d8897136e5dbf91e4d5e210361`。两份 ZIP 和清单已复制到本机 Extreme SSD，单独分发的 B1.4 也已在该盘独立备份。 |
| GitHub | 实验资料发布提交 `ccb72d5096bb771db5ab77e05d90117720dfd9ad` 的日期子树共有 1,788 个 blob，路径、文件模式与对象 ID 完全相符。原日期 Release 含 24 个附件，大小及 GitHub SHA-256 均通过，原标签仍指向 `18d2993784823080ec93594418b7ba48a8e39dcc`。 |
| 总评复算 | 离线打开一场 Semantic 与三场 Concept 录包，确认一次首圈红灯停车和恢复、末场 5.317 秒连续暂停、约 235 毫秒倒车请求及其下游 CAN 计数。车机平台恢复、持续低速起动和整圈匹配绕行仍需实车验证。 |

[机器可读验收数据](../../experiments/2026-11-19-yolop-selection/records/frozen-runtime-verification.json) · [逐步时间戳](../../experiments/2026-11-19-yolop-selection/records/frozen-runtime-publication.tsv) · [先前原始数据验收](2026-11-19-experiment-field-archive.md) · [实验交接](../sessions/2026-11-19-experiment-frozen-runtime/)
