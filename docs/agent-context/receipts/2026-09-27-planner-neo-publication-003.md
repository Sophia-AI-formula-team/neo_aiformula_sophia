# 2026-09-27-planner-neo-publication-003

- UTC：2026-09-27 05:05；角色：planner；类型：RESULT_READY（仓库迁移和清理，不是实车返回）。
- 输入：用户明确要求删除旧仓库误投内容、放入正确的neo仓库；session输入ID为user-request-20260927-correct-repository。
- 当前交接任务：[neo-003](../handoffs/2026-09-27-planner-neo-003.md)，最初发布于
  [50d63b9](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/50d63b9a14d777e3112616426985b4555887e2bc)。
- 唯一发布仓库：Sophia-AI-formula-team/neo_aiformula_sophia，ID1303517209；分支：feat/causal-lane-teach-repeat。
- 功能基线：[93761b5](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/93761b5f4df2cbcf87ac01f533e992fdbd33c456)。
  本回执前实际checkout/证据提交：[44a5005](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/44a5005e4ab12c5d784df98c535b07053313bd58)。
  93761b5之后未改三包运行代码/参数/ROS测试；仅交接工具回归、CI checkout和文档证据。

## 完成与核验

- 三包位于workspace/src/aiformula/control，与trajectory_follower同级；CI使用neo已有VectorNav消息目录。
  不合入旧车辆代码，用户原有两个checkout的改动未触碰。
- [迁移记录](../sessions/2026-09-27-planner-migration-003/SESSION.md)和
  [封存包](../bundles/2026-09-27-planner-migration-003.zip)已经发布在44a5005，后写本回执引用它。
  ZIP为24120 bytes，SHA256：`d4ddb85c9258ad707f372d8adc899502ad15618f17a8531a7bc018e3fc8bc840`。
- [清理证据](../sessions/2026-09-27-planner-migration-003/evidence/cleanup-result.json)：
  旧feature分支、精确11个run和8个artifact全部删除，命令exit0；随后分支HTTP404，run/artifact/workflow列表均0。
  旧main和其它3分支SHA、PR#1/#2状态保留，未删除旧仓库。
- 删除前备份11份日志、8个ZIP共16,161,491 bytes，hash/大小/ZIP CRC均通过，本地仍可恢复。
  原件未公开审阅，不上传；旧封存摘要包原字节保留在neo。旧Actions URL失效属于预期，不改写历史URL伪造neo证据。
- neo [teach-repeat CI](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/36295295919)：293单测、63项DDS检查；
  [GNSS CI](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/36295295920)：520单测（含前293项）、9场景151项DDS检查。两组成功。
- neo [交接CI](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/36295592296)：33项在四个OS/Python环境均成功。
  本次新session/ZIP随后又在本地用`python -B -m unittest discover -s tools/tests -v`验证：33项，32通过/1真实symlink权限跳过，exit0。
  validate、pack和git diff --check均exit0；44a5005及本回执的远端检查由各提交关联的新CI显示，不把之前run冒充覆盖新记录。

## 失败与未执行

首次neo交接CI因sparse checkout缺workflow文件失败，修复于b3e5767并加回归；失败run36295441564保留。
新CI摘要使用v2，测试数来自workflow日志，artifact未归档pytest XML。没有新视频或实车整圈验证。
实际road_detector丢frame_id、LYA程序/限幅不匹配、旧bag断流等阻塞未因迁移消失，详见当前任务。
没有连接/启动车辆、没有实车输出许可；GNSS仍只用于两个停止窗口的起点许可，绝不用于建图。
物理停车状态和急停链须由现场操作者确认，不能从CI推断。

## 下一责任人

experiment agent先核对neo身份和实际checkout、回RECEIVED；再按neo-003进行只读/隔离输入核对，记录失败和差异并打包回传。
planner负责上游Header/LYA集成决策和主要修复，不授权现场agent放宽gate。
当前无人回ACK，未部署、未跑车；本回执不表示接受尚不存在的实验改动，也不自动启动任何agent或车辆。
