# 2026-09-27：纠正仓库并清理误投内容

用户明确要求：旧的删除，放到正确的 `Sophia-AI-formula-team/neo_aiformula_sophia`。
旧仓库本身、原有main/其它分支/PR不在删除范围。

## 身份与来源

- 正确仓库：neo_aiformula_sophia，ID **1303517209**；迁移基底main：`8914b613c7200709f7c8617aed2d408afd911a0d`。
- 错误旧目标：aiformula_sophia，ID **1001834365**；迁出分支HEAD：`328c2283347499161cec22775ce13e0834d29c60`。
- 迁移后功能提交：[93761b5](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/93761b5f4df2cbcf87ac01f533e992fdbd33c456)。
- 只迁本任务文件，不合并旧车辆代码；neo原有main、control/perception等源文件不变。

## 目录与完整性

| 旧位置 | neo位置 |
| --- | --- |
| pid_ws/src/lane_mapping_lya_reference | workspace/src/aiformula/control/lane_mapping_lya_reference |
| pid_ws/src/lane_mapping_fixed | workspace/src/aiformula/control/lane_mapping_fixed |
| pid_ws/src/lane_mapping_fixed_gnss | workspace/src/aiformula/control/lane_mapping_fixed_gnss |
| aiformula/sensing/vectornav/vectornav_msgs（仅依赖引用） | dependencies/vectornav/vectornav_msgs（使用neo已有源码） |

三包72个文件全部迁入：67个逐字节不变，改动仅3个README和2个CI脚本。
全部46个Python源文件逐SHA256一致；2份ROSworkflow另外更新路径、触发范围并验证正确仓库ID。
VectorNav 11个消息定义与旧来源逐SHA256一致。没有修改算法、安全参数、上游LYA、road_detector或电机。
两份已封存session/ZIP与原始任务/回执均按原字节保留；当前入口和新任务明确覆盖旧发布目的地。
不能用全文替换把旧CI地址改成neo地址，那会伪造历史。

## 清理前备份与验收门槛

[旧CI校验清单](old-ci-inventory.json) 已审查，只含公开元数据、相对文件名、大小和SHA256；
清单本身SHA256为 `e037129470d8b861782707b8d23411b6de5a8ab688a77bbce133aba9549bd7e4`。
11份原始日志和8个artifact ZIP共16,161,491 bytes已在本地受控备份，逐hash/ZIP CRC复核通过。
原始内容未作公开隐私审阅，**不在本Git目录中**；需要原件时向planner请求授权传递。
已经公开的历史摘要和原始封存包保留在neo，未来使用neo本仓库新CI证据。

仅在新仓库文件/证据包完整、Foxy和交接CI成功后，才删除旧功能分支和它独占的11次运行/8个artifact。
删除前再次核对旧仓库ID、分支HEAD及精确运行名单。分支有新提交或出现非本任务运行就停止核查。
不得删除旧main、其它3分支、PR#1/#2、仓库本身或改写公共历史。

2026-09-27 05:02 UTC 已完成删除后核验：旧 feature 分支返回404，旧 Actions 运行、artifact、workflow列表均为0。
删除精确的11次运行和8个产物；旧main、另外3个分支SHA和PR#1/#2状态均保留。
旧本地来源分支/19个原件备份仍可恢复；不宣称隐藏Git对象或GitHub缓存立即消失。
详细过程与精确ID见 [迁移session](../../sessions/2026-09-27-planner-migration-003/SESSION.md) /
[清理证据](../../sessions/2026-09-27-planner-migration-003/evidence/cleanup-result.json) /
[封存ZIP](../../bundles/2026-09-27-planner-migration-003.zip)。
备份清单里的 `remote_mutations_performed=false` 只描述备份生成时刻，不代表之后没有执行清理。

## neo 新证据（不是改写旧链接）

| 检查 | 实际neo运行 | 结果与边界 |
| --- | --- | --- |
| Teach-repeat Foxy | [36295295919](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/36295295919) | 293单测、63项真实DDS检查通过；合成bundle，不是相机整圈 |
| GNSS Foxy | [36295295920](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/36295295920) | 520单测（含前293项）、9个静止DDS场景151项检查通过；不是完整圈到repeat |
| Agent交接 | [36295592296](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/36295592296) | 同33项测试在Ubuntu/Windows × Python3.8/3.13四组通过；早于本迁移session的发布 |

两组Foxy都对应功能提交93761b5；交接CI对应b3e5767。
交接的[首次失败](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/36295441564)保留：
sparse checkout少取了workflow源文件，已在b3e5767补齐并加回归检查，未放宽断言。
实际日志和artifact核验摘要见 [neo-ci-summary](../../sessions/2026-09-27-planner-migration-003/evidence/neo-ci-summary.json)。
单测计数来自CI日志，artifact没有pytest XML。4组理想运动模型demo通过，但没有新MP4/GIF、实车动力学或急停证明。

## 尚不代表实车可用

neo mask丢frame_id和LYA限幅差异已写入 [新任务](../../handoffs/2026-09-27-planner-neo-003.md)。
本次迁移不放宽准入、不猜标定，不把本机单测或合成DDS说成实车整圈/急停通过。
