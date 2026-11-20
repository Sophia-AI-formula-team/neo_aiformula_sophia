# 2026-11-20 实车实验记录

时间均为日本标准时间（UTC+09:00）。Concept 和 MapAssist 在同一天的车端部署、场次与问题按同一时间线整理。版本以当场车端已安装的记录为准。

| 项目 | 车机部署基线 | 启动 |
| --- | --- | --- |
| MapAssist | 1120-av314.1，准备提交 `61904bf`，分组 OA2、像素停车与原车 LYA | `allnodes` → `yolop` → `mapassist` → `lya` |
| Concept | V6.27.8，固定提交 `f305d5f7`，B1.4、H12 | `concept-live`，1 为 Pure Pursuit，2 为内置 LYA |
| A4/A4Light | 运行更新 `220f8cba`，两档共 6 文件 | `yolop` 选择 2 或 3 |

- [共同操作与部署时间线](TIMELINE.md)
- [Concept 场次](CONCEPT.md)
- [MapAssist 场次](MAPASSIST.md)
- [录包索引和证据状态](BAGS.md)
- [11 月 19 日 LYA 调参回滚](LYA_ROLLBACK.md)

[09:03 与 09:09 两场 MapAssist 红灯检查，含正面灯牌实录、日志和参数回放](mapassist-red-light/)

本次批次覆盖截至 11:37 的已关闭录包、Concept 界面录制装车和独立 LYA 回滚。场次和实际时标见录包索引。
