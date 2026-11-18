# Foxy 控制交接实测

这个测试启动仓库里的 `concept_control_gate`、`concept_vehicle_state` 和 neo `motor_controller`，每个节点使用独立的 Python 进程。另一个进程经 DDS 发布合成的手柄、0x711 轮速、IMU 和带时间戳的自动指令，并以软件 CAN 终端的身份监听车辆模式下的最终 0x210 话题。转速取自收到的两个有符号 CAN 字，不使用控制节点自报的目标转速。

## 复现

在装有 ROS 2 Foxy、`rclpy`、`can_msgs`、`geometry_msgs`、`sensor_msgs`、`std_msgs`、`std_srvs` 和 NumPy 的 Linux Docker 容器中运行。将包含上述三个包的完整 neo 工作树只读挂在 `/neo`，使用容器自己的网络命名空间。给测试留一个独立 ROS 域，容器中无需 CAN 设备。

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=69 ROS_LOCALHOST_ONLY=1
export ROS_HOME=/tmp/qa_gate/ros ROS_LOG_DIR=/tmp/qa_gate/log
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="/neo/vehicle_concept/src/concept_control_gate:/neo/vehicle_concept/src/concept_vehicle_state:/neo/workspace/src/aiformula/control/motor_controller:${PYTHONPATH}"
mkdir -p /tmp/qa_gate
set -o pipefail
python3 -u /neo/vehicle_concept/checks/foxy_gate_dds.py | tee /tmp/qa_gate/result.jsonl
```

程序使用实车 wheel、motor 和 DualShock 配置。每行 JSON 记录一个阶段。时间从测试切换输入到软件终端第一次收到对应 CAN 指令，包含最长约 25 ms 的下一次合成输入等待时间。末行 `PASS` 表示所有阶段通过。子节点日志位于 `/tmp/qa_gate/log`。脚本结束时关闭它启动的进程。

## 三次重复的实际结果

环境为 Ubuntu 20.04、ROS 2 Foxy、Python 3.8。控制门节点 `node.py` 和状态机 `core.py` 的 Git blob 前缀分别为 `af3eeb1` 和 `eaafe5e`。每次 12 个阶段全部通过。

| 动作 | 软件 CAN 端点的结果 | 切换输入至第一条对应指令 |
|---|---|---:|
| 自动 0.4 m/s | 双轮均为 +69 rpm | 43.4–50.0 ms |
| 反向手柄接管，自动仍持续请求前进 | 切换时先收到双零，随后双轮均为 −98 rpm | 46.3–63.9 ms |
| 松开手柄，自动指令持续更新 | 首条双零后收到连续 109–110 条双零，暂停持续超过 0.65 s | 21.1–25.6 ms |
| 调用恢复，持续重放恢复之前的时间戳 | 状态显示等待新指令，34–35 条 CAN 指令全部为双零 | 全程零 |
| 收到恢复之后的新指令 0.6 m/s | 状态显示已收到新指令，双轮均为 +88 rpm | 22.0–40.4 ms |
| 自动指令流中断 | 暂停，双零 | 475.8–496.1 ms |
| 手柄心跳中断，自动指令仍持续 | 暂停，双零，恢复请求被拒绝 | 480.2–492.1 ms |
| 轮速 CAN 中断，IMU 和自动指令仍持续 | 暂停，双零，恢复请求被拒绝 | 150.2–158.1 ms |
| 暂停电机换算进程，DDS 节点仍存在 | 禁用输出，三次都收到 11 条双零，从首条到末条间隔 199.8–200.0 ms，恢复请求被拒绝 | 109.6–120.1 ms |

测试同时核对软件终端和控制门的状态。每段持续自动行驶都收到预期的非零转速，控制门的 `auto_fresh` 为真，`reason` 为 `auto_command_received`。三轮均收到 232 条合成识别序号，接管、暂停、恢复过程中序号连续，最大间隔 29.5 ms。这项结果覆盖容器内 DDS 和软件 CAN 终端。ZED 采集、YOLO 推理、物理 CAN 和电机停车距离需要分别验证。
