# 交接

用户最新要求先尝试框尺寸distance，不动其他功能；补充明确按Unity模型和neo内参计算，无需另问用户标定值。旧红灯左转任务仍未完成，但本次不扩大到地图/路口/控制修复。

分支feat/semantic-bbox-distance-20261012，基于PR21的7ef9af132811d5e5f1b19f6d3918f87d5801f372。本目录0.1.0，semantic/ROS bridge版本不变，历史归档不改。新worker导入转至signal_bbox_range；模块启用时只提供框高distance，禁用时原函数原参数原返回。JSON默认false，试验安装显式enable。

2026-10-12 14:45 JST唯一授权窗口：18项标准库单测全部PASS（unittest 0.009s，primary0.2907743s，cleanup0.0000946s）。父PID22752、birth="134362575487531434"，子PID23484、birth="134362575488605360"；Job active=0，独立Get-Process两者ABSENT，7份源码/配置/manifest的literal SHA在测试前后相同。CPU已明确归还，未使用剩余窗口。原始回执、18 ledger、pins及测试时manifest见evidence/tests_01。测试后只更新文档、manifest状态和证据，运行时源不变。

MCP未运行，新模块未安装；安装往返未验证。Lane线程019fff52-58c3-75d3-b880-f1a5c9dbab13仍持有Unity；原窗口只允许18单测，不允许再跑测试、模型、图像、45/703、视频或MCP。继续部署必须取得新的明确窗口。

所有写入目前只在本Git工作树。本次已核对live源worker=bf50643792b2c998a179bdab47ddc9e9400ee3d009de704a1e14d1e3cd328469，range=baf09f79cd0649b05e8d1acd90e6a9395bd7361d00e2b8ff51f6f20dd54d4ce7，RGB=aeb7b9c23d019b67fcf795a3a5067d9de55eacb33af9e0220f7095cb7dd61326。新的部署必须再次预检；不碰另一对话Lane源码，不用旧全局基线覆盖它。

先测试并保存实际结果，再独占MCP现场校验并部署；若存在问题先更新PROBLEM_ANALYSIS，再作必要最小修复。保留原失败、不伪造功能验收。GitHub使用scoped draft PR，不merge/force-push/tag。原native ROS shadow约束不变。
