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

当前阶段：新代码已推送；新CI、交接文件发布核验以及旧目标清理尚在进行。
最终完成状态由新的清理回执记录；不能从这份备份清单里的 `remote_mutations_performed=false`
推断之后没有清理，它只描述备份生成时刻。

## 尚不代表实车可用

neo mask丢frame_id和LYA限幅差异已写入 [新任务](../../handoffs/2026-09-27-planner-neo-003.md)。
本次迁移不放宽准入、不猜标定，不把本机单测或合成DDS说成实车整圈/急停通过。
