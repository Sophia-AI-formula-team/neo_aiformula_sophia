# OA 1B 接线验证和实验边界

本目录留存 Linux/ROS 2 Foxy 实际进程、DDS 消息和 GPU 回放结果。四段联通台架使用 11/20 17:35 收车安装件、AV314 权重、原车电机 console 入口和 1121 私有覆盖。输入为一张重复播放的原始相机帧和静止测试里程计。LLP、车道选点、OA 2.1、MapAssist、LYA、电机仲裁均运行真实节点。CAN 只在容器内的 ROS 话题发布。

台架基础镜像为 `vehicle-semantic:field1120-oa21`，镜像 ID 为 `sha256:57eb15811c61bf0f6faa1b5233ed0ba54286f40b890a9ad94274f80e5bf97295`。安装日志中的 Git 前缀来自隔离测试检出，实车发行目录以实际检出的完整提交为准。最终业务链采集了 129 组车道 ABC、94 条 LYA 指令、146 条电机自动放行状态和 2,523 帧 ROS CAN 数据。另用同名 DDS 观察节点接入再断开，车端检查按预期先拒绝、接入后通过、断开后再次拒绝。[车端 CAN 订阅检查](evidence/socketcan-contract.log)

| 检查 | 结果及原始记录 |
|---|---|
| 四段上下游 | 依次通过 Allnodes 前置检查、识别前置检查、OA 前置检查、LYA 前置检查和最终 DDS 检查。能看到实发 ABC、车道选择、LYA、最终控制、手柄事件和 CAN 的节点关系。[进程与命令](evidence/bench.log) · [实际 DDS 图](evidence/live.json) |
| 控制是否到达电机 | 查看 `samples.json` 中的 `matched`。同一时间窗口里，LYA 与 MapAssist 发出的线、角速度和电机选中的两项数值一致，电机状态为 `semantic_go`、`manual_held=false`。DDS CAN 帧 ID 为 `0x210`，8 个数据字节与电机记录相符。关停后的末条 `command_timeout` 则为零速。[时间对齐样本、消息数和末帧](evidence/samples.json) |
| 新旧感知能否区分 | 一次实际 GPU 推理，同一图像时间戳，原检测 JSON 记录 50 红像素，私有 JSON 记录 25 个独立红像素。这一帧实际 `/flag_stop=false`，规划仍在 `prepare`。在单独识别探针中，相机只出现一个识别订阅者。原车道 mask 与锥桶 mask 均有输出。[识别 DDS](evidence/dds.log) · [端到端记录](evidence/samples.json) |
| 内外圈配置 | 分别启动两套车道配置。原公共检测发 500 px 不进入实验停车适配器。实验私有输入 49、50 依次得到 `false`、`true`。实发状态中的最小转弯半径依次为内圈 6.0 m、外圈 5.5 m，第三终端没有重复启动 LYA。[内外圈原始日志](evidence/control-graph.log) |
| 手动优先 | 使用原车电机 console 与原参数，向真实 Joy 订阅口输入两次带使能的推杆，私有版发出 2 次边沿，原版发出 0 次。两者手动速度均为 1.05 m/s。[电机对照](evidence/motor-console.log) |
| 识别回放 | 三次外圈原始 PNG，共 420 个识别帧，原灯色及像素逐帧复现。再次亮红的 50 帧从 41 帧识别为红提高至 48 帧。82 个熄灯帧中没有新增红判。3 个过期帧未进入私有输出。按 50 px 阈值，20 个原本过线的帧在去重后低于阈值。各场原始计数和 5 个状态机测试结果可复核。[三场计数](evidence/replay_420.json) · [单测](evidence/unit.log) |
| 共存与退出 | 用原部署路径布局连续执行两次准备，旧文件哈希集合相同。干净 shell 仍解析到原车电机和规划器。六项快捷方式测试覆盖重复安装、移除、保留原桌面/命令和路径冲突。九项监督器测试含首终端或后续终端被强制关闭、孙进程脱离进程组及保留同机其他进程。四个独立 Foxy launch 实测退出后原启动锁可重新获取。完整业务链结束后 DDS 剩余节点为空。[安装对照](evidence/prepare.log) · [快捷方式](evidence/shortcuts-tests.log) · [进程树](evidence/linux-tests.log) · [四终端退出](evidence/ros-smoke.log) · [业务链退出](evidence/after_stop.json) |

## 发现的问题与现场复核

- **首终端启动问题，已修复。** 将识别和监督器合到最终 Python 包后，从 `entry` 实际使用的路径直接执行 `supervise.py`，曾报 `AttributeError: module 'signal' has no attribute 'SIGINT'`。原因是同目录的识别文件 `signal.py` 遮住了 Python 标准库。各组件在独立目录测试时未触发。实验模块已改名为 `traffic_signal.py`，监督器以 Python 隔离模式运行。完整包已通过 9 个进程树测试。安装后的真实首终端到达 Allnodes 预检，越序启动 LYA 得到拒绝，同域停止命令得到回执，记录的 PID 和套接字均已消失。四个专用快捷方式也在独立终端通过了安装版入口的启动和停止测试，硬件负载由 Foxy 测试节点替代。[安装入口复查](evidence/prepare.log) · [四个快捷方式实跑](evidence/entry-lifecycle.log) · [进程退出日志](evidence/linux-tests.log)
- **旧 LLP 在退出阶段可能报段错误。** 台架中节点工作期间有车道输出。SIGINT 后，原 C++ LLP 直接执行时返回 `-11`。用相同旧二进制单独复现，无需启动 OA 1B。该进程已经退出，结束后的 DDS 检查为空。保留原包并记录退出码。现场若再次出现，单独采集 LLP 调用栈。[旧 LLP 单独复现](evidence/stock_llp_standalone.log)
- **50 个独立红像素的停车距离仍需在车上量。** 真实灯面积、距离和遮挡影响识别，去重还会改变越过阈值的时刻。3 次同场景回放改善了特定灯牌的复检，仍有 2 个再次亮红帧漏检。该数据没有覆盖车辆、锥桶等全部干扰，也未验证连续漏检一秒时的车身位移。建议现场将同一源帧的原/私有识别、停车位置标志、最终仲裁和车身运动对齐。
- **版本差异须在车上核实。** 已从 11/20 归档校验 AV314 活跃运行库和权重。已测试的 ROS 识别封装来自 11/19 车机包的 A3 源目录。11/19 的现场审计记录了源码与安装件的关键行。部署时还会比对从实际 venv、`ros_install` 解析的文件和哈希。若不一致，保存实际路径、SHA-256 并反馈。Jetson CUDA、ZED 设备、实体 CAN、带轮行驶、避障净距及信号灯前实际停车距离需由部署方另行核验。[识别封装归档核验](evidence/wrapper.log) · [原车识别审计](../2026-11-19-yolop-selection/semantic-045/reconnect-wiring-audit.log)
