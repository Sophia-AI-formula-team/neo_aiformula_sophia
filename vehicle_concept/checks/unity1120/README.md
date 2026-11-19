# 用 Concept 自己的 Unity 跑 V6.27.6

Unity 拍前视图并读模拟轮速、陀螺。Foxy 里的车端相机节点运行 B1.4，驾驶节点用路界筛候选、用 H12 排序，返回本帧指令。Unity 收到双轮转速后推进物理，重新拍下一帧。下图及数据取自 11 月 20 日四个不同初态的短跑。[四场结果、失败样例和实车对照](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/experiment/vehicle-prep-20261120/docs/experiments/2026-11-20-mapassist/concept)

1. 使用已有的 semantic Foxy/GPU 镜像，在独立容器里启动 Concept 服务。[容器及权重准备](foxy/)
2. 准备专属 Unity 预览工程，按下表在同一位置各跑 5 秒。[Unity 操作和输出文件](host/)
3. 原地核对门控和电机换算。闭环中的门控状态由仿真提供；下面的第二次检查通过车辆使用的真实 ROS 节点，在单独的 DDS 域重放同一段速度、转向。软件接收端逐条检查最终 0x210 双轮转速，并检查自动命令断流后持续发零。给测试的手柄心跳、轮速和陀螺为合成输入。这项核对能发现换算、转向符号和门控的不一致，不能测量 ZED、物理总线和电机的延迟。

| 输出目录名 | 相对外车道中心偏移（右为正） | 相对道路方向（右为正） |
|---|---:|---:|
| straight01 | 0 m | 0° |
| outer_edge01 | +0.65 m | +12° |
| inner_edge01 | −4.08 m | −12° |
| outer_hard01 | +0.80 m | +23° |

Unity 入口的四场 `--start` 都为 `0.87`、`--seconds` 都为 `5`。布置初态和赛道裁判在 Unity 内部运行。ROS 侧收到当前图像、轮速、陀螺及上一拍指令反馈。

### 核对侧向响应

在有 Docker 的工作站将 `NEO` 指向本分支仓库的绝对路径，`HOST_RUNS` 指向上述四个输出目录的上一级，`ROS_EVIDENCE` 指向 Foxy 服务保存逐帧 JSON 的目录。Linux shell 示例为每场运行一个全新的离线容器。镜像中应可导入 Foxy 的 `can_msgs`。重放使用原始指令间隔和新的墙钟时间戳。

```bash
set -e
for name in straight01 outer_edge01 inner_edge01 outer_hard01; do
  docker run --rm --network none --entrypoint bash \
    -v "$NEO:/neo:ro" -v "$HOST_RUNS/$name:/trace:ro" -v "$ROS_EVIDENCE:/evidence" \
    vehicle-semantic:pr47-grouped /neo/vehicle_concept/checks/unity1120/replay_gate.sh \
    /trace/exchange.jsonl "/evidence/$name-gate.json"
done
python vehicle_concept/checks/unity1120/report_runs.py --host "$HOST_RUNS" \
  --ros "$ROS_EVIDENCE" --out comparison --font /path/to/CJK-font.ttc
```

作图脚本使用 Python 和 Pillow，按完整回包逐帧配对 Unity 与 Foxy，核对写入及下一拍读回的双轮 CAN 字，再生成 `unity-comparison.png` 与 `unity-runs.json`。模拟每 0.2 秒计算一次驾驶决策。Unity 等待回复时暂停物理，因此真实耗时以宿主的拍照到写入记录、另行定时的墙钟测试为准。车端接线、初始化和软件版本见 [1120 实验准备](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/experiment/vehicle-prep-20261120/docs/experiments/2026-11-20-mapassist/concept/DEPLOY.md)。
