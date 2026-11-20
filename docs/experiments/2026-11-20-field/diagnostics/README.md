# 2026-11-20 诊断图文与回放

时间为 JST。下表编号对应 [录包索引](../BAGS.md)。各报告注明现场结果与后续回放。

| 诊断 | 报告及重点图 |
| --- | --- |
| LYA 目标偏出车道 | [未接管路段分析](lya-tracking/README.md) · [实际车道及目标轨迹鸟瞰](lya-tracking/lya-lane-exit-birdseye.png) |
| 1120-04 第一圈闭合与第二圈失配 | [诊断报告](mapassist-lap-review/REPORT.md) · [闭合路径与失配位置](mapassist-lap-review/lap-match-1025.png) |
| 1120-04 红转绿 | [SSD 原包复算](mapassist-start-speed/ssd-replay/REPORT.md) · [红绿灯牌实拍局部](mapassist-start-speed/panel-red-green-detail.png) |
| A3、A4 和 A4 Lite 灯牌识别 | [早场检测对照](mapassist-red-light/stills/a3-a4-detection-comparison.png) · [A4 Lite 转绿报告](mapassist-green-resume/REPORT.md) · [A3 回放](mapassist-a3-tuning/REPLAY.md) |
| 1120-05 起步外拐、近场 OA 和红停不解锁 | [逐事件复盘](mapassist-latest/REPORT.md) · [OA 原生 HUD](mapassist-latest/work/oa-hud-2.jpg) · [绿牌实拍局部](mapassist-latest/work/held-green-panel-crop.png) |
| Concept 门控与停车 | [两次实车场次](concept-1037/REPORT.md) · [新指令断档定位](concept-1037/command-fix/REPORT.md) · [15 FPS 界面样帧](concept-1037/panel-15fps-frame.jpg) |

## 车载回放

| 录包 | 视频 | 时间与内容 |
| --- | --- | --- |
| 1120-04 | [MapAssist 完整首圈 POV](videos/1120-04-mapassist-pov-public.mp4) | 6 分 18.625 秒。含红转绿、短中线断供、第一圈闭合与第二圈失配。 |
| 1120-05 | [MapAssist 11:17 场 POV](videos/1120-05-mapassist-pov-public.mp4) | 3 分 13.875 秒。含首次 LYA 目标、近锥桶、红灯锁与漏绿，截止持续手柄接管前。 |

本页视频为 8 FPS 公开版。为遮挡可辨识面部，对相机画面上部做了像素化，保留车道地面、LYA 目标与门控读数。含完整相机画面的回放位于 Drive 日期目录的「诊断图文与回放」；报告中的编码与覆盖审核以原始回放为准。放大的灯牌证据使用原图局部裁切。
