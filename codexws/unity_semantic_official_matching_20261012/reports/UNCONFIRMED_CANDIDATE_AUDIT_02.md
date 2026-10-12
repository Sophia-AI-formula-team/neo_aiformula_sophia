# 后续候选诊断状态 — 2026-10-12

本记录补充 0.1.0 诊断归档。既有安装 payload、semantic core/local 0.2.0、native bridge 0.1.0 和官方 OSM 未因本记录改变。红灯触发路口左转绕行仍未通过；用户选择保留官方图，禁止启用 PR #18 的仿真节点/portal 标定。

## 量化曲线候选

独立 review2 候选的首次有界测试实际完成 27 项：26 PASS、1 FAIL、0 ERROR。失败为 `test_clean_two_bands_can_support_only_unconfirmed_geometry`：没有得到支持的双边界假设。原断言、门槛、失败日志和源码保留。真实 703 张掩膜批次没有开始，不能声称该批验证通过。

冻结 observer SHA256 为 `9ba0a4c5ae9c352b0d0185ab017387e4cbf61d30f75fa3ffa594489d5162cea8`，test SHA256 为 `6257487526ce53f9188e0a97f510b185a122ec5259ba07b96af1f91c24497555`，review2 manifest 为 `39af7fb3e8750013ad360c5ae9209a2c83c9042880800ad789851b6a29183253`。候选不在安装 payload 中，不进入控制。

单用例诊断的 runner01 在 Windows venv launcher 启动真正解释器之前因 Job 单进程限制失败（exit101）；未完成 observer。这不是算法测试结果。runner02 未执行；静态复审发现普通 Popen 后加入 Job 可能漏掉已派生的解释器。runner03 已改为 suspended→Job→核验初始线程→resume→event，并准备实际解释器 PID/出生时间 sidecar。其准备态 SHA256 为 `c70c04b7d67b417cf3854619a945b5aaf3ca785953c401c79e35737100a61722`；此时尚未执行。各版源码、失败回执和独立锁保留。

静态检查表明完整闭合 contour 已拆臂，不能直接认定 out-and-back 是失败原因；仍需读取端点角锥、实际匹配跨度及具体失败门控。不要改断言为 unknown 或降低门槛来获得绿色测试。

## 原图载体与多视图平面

原始 RGB PNG 存档候选仅修改 detector/harness 接线，原 worker、相机、模型、LP/LYA 和 semantic 核心保持原版本。静态审查未发现缓存错序或原 PNG 丢失；尚未编译、测试或通过 MCP 真实采集。

独立 observed-plane 候选的旧 public API 测试仅覆盖 disabled/unknown，内部 helper 正例不足以证明完整链路能成功。已准备六项额外测试，包括固定倾斜平面/非重复纹理/原 KRT 的两视图与三视图 public API 正例及错 pose/错第三视图/错 SHA 拒绝。新增测试 SHA256 为 `b24231a6071c60b5aece77347e2056a5f541daf5c737b31cf635a425b49af64c`，尚未执行，不降低门槛。即使平面得到 shadow candidate，也不自动证明路口、地图匹配或可行驶路径。

## 视频

video02 单次在编码前失败：旧 capture `(epoch1, sequence503)` 被错误地与停机后 UI epoch2 比较。原响应/PNG 可以保留为诊断证据，不能升级为当前控制输入。原02失败回执与原始采集未覆写。

已准备 video03：以 capsule 身份核对原 lane PNG，单独显示 UI epoch，旧上下文标 STALE/current-control excluded；第三、第四栏使用相同 semantic elapsed。主画面保留完整 1280×720，占 1280×960 输出的 75%。输入仅两场 v2 **绿灯元数据采集**；不标为红灯或左转验证。准备态 builder SHA256 为 `dbab8588882e90cd6b09aa31add9550e7e30964a4f9c2e605752f79fea713a7a`，manifest 为 `a000644fdaf8d7685f8d62da8609e33c4791656d6bae800f4615ca73284c27dd`。尚未编码/全解码/视觉验收。以前的真实红灯左转失败视频仍是原证据。

## 执行交接

Unity/CPU 当前由另一对话持有；仅使用最新明确窗口，禁止沿用过期 lease。先执行具名单用例诊断，读实际失败门控；视频及 RGB/plane suites 分别使用独立窗口。新 RGB 接线需 fresh source SHA/backup、MCP 编译和同响应原 PNG SHA 采集。之后才能用真实图像评估路口与官方关联，再按既有问题报告修必要核心边界并 MCP 重跑内外圈。所有候选结果、旧失败和真实场地功能验收分别记录。
