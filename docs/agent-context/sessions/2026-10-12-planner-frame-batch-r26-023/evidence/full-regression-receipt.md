# R26 全量native回归结果回执

2026-10-12，只读核验root已执行的单次输出；本回执整理未重新运行测试、估计器、profile或Unity，未修改R/T。

## 已观察结果

- Windows / Python3.13.13，pytest exit0：4242 passed、1 skipped、0 failure、0 error；JUnit共4243项。
- stdout记93.38s，JUnit suite93.361s，pytest子进程94.62946100003319s；不同计时口径分开记录，不混作runtime预算。
- 开始UTC2026-10-11T18:25:00.966824+00:00，结束UTC2026-10-11T18:26:36.233598+00:00（本地为10月12日）。
- 唯一skip：test.test_supervisor::test_owned_process_shutdown_leaves_unrelated_process_alive，test_supervisor.py:62，原因Target Linux process-group test。不是native缺失skip。
- 原测试范围reference/test、fixed package根、gnss/test完整保留；fixed/test子目录不存在，runner记录该事实而未伪造测试路径。
- 唯一显式排除为原未集成test_quantized_curve_fit.py。其源文件与测试不读取、不导入或纳入本回归证明，未删除/修改。

## 源冻结与native证据

HEAD def21001a53e1b7a5e097f5e7bca7bbcc9676f02，加明确的R26两处dirty源与一个新测试；不是将dirty候选称作已提交HEAD内容。

193项pins，before/after完整JSON内容及文件字节相同，且等于事先pending manifest snapshot。summary source_changes={}，summary列出的9个输出hash全部由本审计读盘复核一致。manifest SHA377652f6e0e8a07416d97df1b12ef715d3820ab1b0f7a31483d677f121740d8a。

- paint_geometry.py：b8ece7fb614423277ad4a5e27fa5463d1d8c1198357156390b6cfa03d3d75a77。
- continuation_support.py：178cb6dbaf5af39ae285bd00eb90a7ee2c95b93205c1f78971dc41f15e9b19e4。
- test_continuation_batch.py：0735c09770f946d1c142cf1fd9bf4dc0388d9fe4fd246483a650dacb1bf97b82。
- C源：9dae4a7ab517d872af29d3c2d0a27d1cdfa7e8446c8f138be206df5ba4c4438f。
- pyd：f2f9712754f8cecaae379af5355c8aa82b3a28855fa2da2d0ce949ddf339ef45。

native verifier exit0，0.1609732000506483s；backend=native，全部9个API版本字段=1，real_calls_passed=true，finite_current_union_real_call_passed=true，C/binary身份匹配。LANE_REQUIRE_NATIVE=1，四root PYTHONPATH及原LANE_REAL_FRAME_DIR保留。

重要限定：本次native_actual_01.json的installed_prefix_verified=false，不能改写为true；这是本次源码工作区加载/真实调用验证，不是新的安装前缀验证。既有安装证据由strict replay另行锁定，本次回执不混合两次验证的语义。

## 证据SHA256

|文件|SHA256|
|---|---|
|summary_01.json|d5aabad15a1f857c262085a2de534940e65c1b4f2918aaf83e85bd90cb30a47d|
|pytest_01.xml|35144f5be20ec1a8a4c1ea3e29613ba0adfb80eedb1e5a57c2c403156fc72f19|
|native_actual_01.json|286231e8f872fd271b9fbe15b18e1e616814d19c72bf5c9bf27c735d5368bd1c|
|sources_before_01.json|ddff6aa439f3b18cb2a3ffdec26f07a66d0c2fc74fb4de086859516a78ef387d|
|sources_after_01.json|ddff6aa439f3b18cb2a3ffdec26f07a66d0c2fc74fb4de086859516a78ef387d|
|pytest_stdout_01.txt|f3c6afc405f24a0e2a1e05b581f2b4fa18bf345dd56879a09a8b2cc13f94aef0|
|pytest_stderr_01.txt|e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855|

## 不成立的成功声明

单测全通过不证明DDS、原40/80/200ms端到端、live Unity或整圈成功。随后原111帧strict replay仅15 attempts/14 accepted，seq165 processing_deadline，all_prefix_frames_accepted=false；详见D/r26_observed_01/COMPARE_01.md及json。seq164的最终80.5944ms接受值也保留为已知提交前预算而非返回时限保证。

本地测试和固定失败前缀都不是完成实验或whole-lap证据；无重试、无新增参数调整。
