# 2026-11-21 装车前检查

检查时间 08:32–08:34 JST。实验角色为 `experiment`。准备分支为 Neo `experiment/vehicle-prep-20261121`，审核提交 `dff29d12e805f38f6fb34027b5f93789f4a59e3f`，仓库 ID `1303517209`。本轮执行源码和车端只读预检、本机隔离测试。

| 候选包 | 固定提交 | 结论 |
| --- | --- | --- |
| Concept 1121 | `6793a13c0a1a22c127accb452ef65a206d6f415d` | 权重和现场依赖路径齐备。原安装器的旧版清理预检在车机返回 1，需先处理保留旧版的安装路径。 |
| MapAssist B / OA 1B 0.1.2 | `dc8d540278137ddc608f9b87bcfb1149913a7e12` | 两份依赖的实装 MapAssist launch 哈希与脚本一致，系统 Python 可导入 ROS、OpenCV 和 NumPy。独立构建与车端静态接线验证待执行。 |
| OA 2.2 | `e4866962ac24ba2ed8a7ee7d99523e7c239ba76a` | 新 C++ lane publisher、launchers 和 OA 节点需要配套。现场已有 C++ 依赖包，旧安装二进制未发现 `dynamic_roi_mask`。Foxy aarch64 构建和同戳话题验证待执行。 |

## 时序和差异

- 08:32:49–08:32:57 从远端确认 1121 各分支。兼容分支 `experiment/oa1b-1121` 的 tip 是旧独立 OA 1B 方案的回滚，当前 MapAssist B 以另一个 `mapassist-b-1121` 分支为准。SSH 确认 Jetson 为 aarch64、ROS 2 Foxy / Python 3.8，约 29 GB 可用。首次 SSH 有 11/20 的 MapAssist 启动 PID 3300。08:34 复查时进程清单未见该驾驶启动组或 ZED 进程。
- 08:33:35 车端 B1.4 文件 SHA-256 为 `19d6eeed99fee90a46f09761746845afcf330974a8e1aff057055697a02706a6`，与准备要求及本机副本一致。本田投影配置存在，旧 Python venv 约 184 MB。保存的目标速度是 5.0 m/s，自动门控上限 3.5 m/s，1121 省略显式参数时会读取这些值。新版最低非零速度默认 0.5 m/s，LYA 参考默认 2.0 m/s，保留明确停车、面板 15 FPS 录制、关窗退出和同机推理／规划分进程。按上游离线报告，默认下限排除旧的 −0.2 m/s 倒车；一次 Unity 滚动外缘场景首帧发零仍越线，另一次 H12 选择停车后五秒未恢复。
- 08:33:35–08:34:03 车端 6.27.5 源目录含 6 个未追踪 aarch64 `.so`，旧工作区还有 `dependency-check.log` 和 `python-dependencies.log`，均触及旧版清理预检。只调用 `legacy_install.py preflight` 原样复现报错。调整安装方案时宜保留旧环境，先在独立 1121 目录完成新版验证，再确定清理范围。4 项本机安装、快捷方式和回退测试通过。
- 08:33–08:34 MapAssist B 的预检哈希为 `bf224634…fcc1` 和 `409e91fb…106b`，现场匹配。MapAssist B 继续用原 YOLOP 做模型推理，CPU 以同戳同帧的检测与相机图做灯态补判，默认 50 个去重红像素，只把该实验实例重映射到私有停车话题。脚本所带 5 项灯态测试在已有 Python 3.12 环境通过；第一次默认主机 Python 缺 NumPy。台架中有 20 帧从原始达阈值变为去重后低于阈值，实际停车位置仍需验证。`mapassist_b` 入口显式打开 `vehicle_output`，静态验收须走禁用动力的启动参数。
- 08:33–08:34 OA 2.2 的 Python 和 package.xml 静态检查通过。6 个离线输入验证时戳、相机坐标系、形状、空掩膜和过期回退。新 `semantic_045_commissioning.launch.py` SHA-256 为 `9509aed3…7638`，与 MapAssist B 强制要求的 `bf224634…fcc1` 不同。OA 2.2 应使用独立 overlay。MapAssist B 当前 launch 显式选 `oa_2_1`，保留其现有 overlay 可避免破坏安装检查。整条动态 ROI 链须在隔离域验证 `mono8`、输入源时间戳和 `/semantic_drive/map_assist_045/oa_triplet`；旧 lane publisher 时 OA 2.2 会退回静态 ROI。

[车端只读清单](vehicle-readonly.txt) · [车端运行库和 ROI 检查](vehicle-finish.log) · [Concept 车机预检返回](legacy-preflight-on-vehicle.log) · [Concept 本机回归](test-concept-prep.log) · [MapAssist B 单测](test-mapassist-b-python312.log) · [OA 2.2 ROI 离线检查](test-oa22-roi.log)
