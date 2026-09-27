# Agent session: 2026-09-27-planner-migration-003

Input handoff: user-request-20260927-correct-repository

Role: planner. Migration and evidence housekeeping only; no vehicle operation.

## Objective and scope

用户要求把误投旧仓库的本任务内容放入正确的 neo 仓库，并删除旧的误投内容。
验收范围是三包、构建/测试、agent context、封存包和回执；不是删除整个旧仓库，
也不是合并旧车辆代码、修改控制算法、放宽安全限制或驾驶车辆。

## Input handoff

输入为用户明确的仓库纠正/清理请求，不是历史任务中的旧发布 URL。
已读 AGENTS.md、AGENT_CONTEXT.md、STATUS.md、PROTOCOL.md、新任务 neo-003，
核对旧封存 session/回执与包 README。唯一发布目标固定为
Sophia-AI-formula-team/neo_aiformula_sophia，GitHub ID 1303517209。
旧目标 ID 1001834365，只允许本次清理，不再发布。

## Source and working tree

旧来源 feature HEAD：328c2283347499161cec22775ce13e0834d29c60。
neo main 基底：8914b613c7200709f7c8617aed2d408afd911a0d。
neo 功能提交：93761b5f4df2cbcf87ac01f533e992fdbd33c456。
neo 文档迁移：50d63b9a14d777e3112616426985b4555887e2bc；交接 CI 修复：
b3e5767632edc7caede9c2461792f368d1711059。
分支为 feat/causal-lane-teach-repeat；使用独立 sparse clone，发布前逐次确认 push URL 和不可变仓库 ID。
旧发布 clone 及本地备份保留用于恢复。用户原本旧 checkout 的大量未提交改动、
neo checkout 的未跟踪 biased_mppi 均未触碰。本 session 是随后独立的文档/证据提交。
依赖使用 neo 已有 dependencies/vectornav/vectornav_msgs；11个 msg 与旧来源逐 SHA256 相同，未安装新依赖。

## Work performed

1. 在 Windows/PowerShell 核对两个仓库 ID、refs、旧 feature 的11个专属提交、所有分支树、PR和Actions所有权。
   `gh api repos/<owner>/<repo>`、`gh api repos/<owner>/<repo>/git/trees/<sha>?recursive=1`、
   `git status --short`、`git remote get-url --push origin` 均为只读核验；GitHub 树未截断。
2. 基于 neo main 建立 feature 分支；只复制本任务文件并调整目录。
   三包放 workspace/src/aiformula/control，与 trajectory_follower 同级，不带入旧上游源码。
   72个包文件中67个原字节不变，仅3个README和2个CI脚本调整；46个Python文件全部原字节不变。
   另迁2个ROS workflow，加入正确仓库名/ID保护。当前入口更新，旧封存历史不做伪造性替换。
3. 本地包 pytest 回归519通过、1个Windows平台跳过；46个Python文件按3.8语法解析、Bash语法和YAML检查通过。
   远端 neo 两组 Foxy CI 在93761b5成功，已下载并审查公开摘要。具体计数/哈希见 evidence/neo-ci-summary.json。
4. 交接 CI 在50d63b9首次失败：新增身份检查读取 workflow 文件，但 sparse checkout 没有取 .github/workflows。
   b3e5767补齐该目录并增加回归断言，没有减弱检查；`python -B -m unittest discover -s tools/tests -v`
   本地33项中32通过、1真实symlink权限跳过。远端 Ubuntu/Windows × Python3.8/3.13 四组成功。
5. 04:43 UTC 起备份旧11份日志和8个artifact；19个原件共16,161,491 bytes，全部校验大小/SHA256，8个ZIP CRC无错。
   原件仅本地保存，未公开审阅，不上传。公开的 sanitized_inventory.json SHA256为
   e037129470d8b861782707b8d23411b6de5a8ab688a77bbce133aba9549bd7e4。
6. 先对新repo105个交付文件逐Git blob比对，0不匹配；确认三组新CI正确HEAD且成功后才清理旧目标。
   删除使用完整旧URL和精确branch HEAD lease：
   `git push --force-with-lease=refs/heads/feat/causal-lane-teach-repeat:328c2283347499161cec22775ce13e0834d29c60 https://github.com/Sophia-AI-formula-team/aiformula_sophia.git :refs/heads/feat/causal-lane-teach-repeat`。
   Artifact/run 删除使用 `gh api --method DELETE repos/Sophia-AI-formula-team/aiformula_sophia/actions/artifacts/<id>`
   和 `.../actions/runs/<id>`，只允许已备份名单，逐个核对owner/branch/status。
   清理最终状态、精确ID、保留refs和退出结果见 evidence/cleanup-result.json。
7. 封存前按协议运行 validate、pack --confirm-reviewed、33项交接测试，提交 session/ZIP后用独立回执引用其提交，避免自引用SHA。
   没有本地ROS、相机、硬件、LYA或电机运行；没有原始bag/坐标/凭据上传。

## Evidence and result

功能基线不变。neo teach-repeat run36295295919：293单测、63项真实DDS检查通过；
neo GNSS run36295295920：520单测（包含前293项）、9场景151项DDS检查通过，不能把293+520相加当独立覆盖数。
两者均使用原生VectorNav消息。测试数量来源是已保存workflow日志，artifact没有pytest XML，不能声称XML已打包。
前者用合成ready bundle测试两阶段REPEAT、失效停车，STOPPED输入为模拟信号；没有测supervisor真实停进程。
GNSS测试限于静止窗口/隔离/异常/启动断流，没有完整相机行驶圈到repeat。
4组ideal-unicycle demo（0.4/0.8m/s × 0/0.25m偏置）全部通过，每组77.63m路线跑2圈；不是车辆动力学、实车精度或视频证据。
交接修复 run36295592296 在b3e5767四组成功。新仓库首次失败run36295441564保留，不删除或改写失败记录。
公开证据是逐文件审查后的摘要、哈希和原有封存包；原始旧CI可由planner本地恢复，不公开原件。

## Risks and blockers

迁移/清理不表示实车可用。neo road_detector只传stamp，丢frame_id，会被新包严格拒绝；
neo README推荐lya_0221，与默认受管程序不同，且同名LYA限幅可达4.0m/s/0.45rad/s，
teacher准入仍为2.25m/s/0.4rad/s，超限拒绝。上述是明确待解决的问题，没有借迁移修改上游或放宽安全门限。
旧bag缺POSLLA且轮速有119次>200ms断流，不能通过改时间/重置/未来数据拼成合格整圈。
GNSS仍仅停止时起点许可，绝不建图；运行运动链和mask定位没有改动。
备份校验不等于隐私审查；原始日志/ZIP不能直接公开。旧Actions链接清理后失效属于预期；
历史封存记录保留原URL和SHA，当前部署只能从neo STATUS进入。
删除分支/运行/产物不是承诺GitHub隐藏commit对象或缓存立即消失，也不删除旧main/其他分支/PR。
只读排查中两次按错测试文件路径的Get-Content未找到文件；已用rg --files确定实际tools/tests/test_agent_context.py，无源码影响。
新CI摘要初稿把缺XML误推为0测试；最终证据使用v2，从workflow日志恢复真实计数，错误初稿仅本地保留。

## Next agent actions

planner发布独立RESULT_READY回执并确认远端完整性；后续发布只用neo ID1303517209。
experiment agent从STATUS读取2026-09-27-planner-neo-003，先ACK实际checkout和功能差异，
再做只读输入/部署核对。上游Header和LYA命令/归属问题回传planner决定主要修改，不擅自关闭gate。
现场未获动车许可，不启动第二圈；物理停车链仍须现场操作者实测。
接手、证据接收、代码接收、实车许可四者分开；异步交接不是后台自动唤醒服务。
