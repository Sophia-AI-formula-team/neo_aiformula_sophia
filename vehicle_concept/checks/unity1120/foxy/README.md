# Foxy GPU 仿真车端

这里运行车上的 `CameraNode`、Honda 地面投影、B1.4 和 `DriverNode`，包括可行性筛选、H12 排序和反馈记忆。Unity 根据返回的整数左右轮 CAN 目标推进物理，并在下一帧回读实际设置值。容器仅从图像、轮速、陀螺和上一条执行反馈规划。场地坐标和裁判标签留在 Unity。

## 准备镜像和权重

基线 neo 提交为 `70209be`。需要 x86-64 Linux 容器、Docker Compose、支持 CUDA 11.8 的 NVIDIA 驱动及 NVIDIA 容器支持。Windows 用 Docker Desktop 的 Linux/WSL2 引擎，Unity 编辑器可以继续在 Windows 运行。私有基础镜像 `vehicle-semantic:1120` 在构建时应显示 `sha256:3fb88ac063248c0c78f21f115544d86994b754c5a286ca954bb1c872ebca8997`。基础镜像提供 Foxy、CUDA Torch/TorchVision 和已缓存的 SciPy 1.10.1 wheel。所有容器部署命令都从本文件所在目录执行。

已有镜像的授权开发机可以用下列命令从本地导出。在新机器上读取同一个 tar，并核对 `docker image inspect vehicle-semantic:1120 --format '{{.Id}}'`。也可以只搬最终成品 `aiformula-concept:foxy-gpu-1120`，在新机器上 `docker load` 后以 `docker compose up -d --no-build --pull never` 启动。最终镜像采用该团队开发环境，按团队内部镜像分发。

```sh
docker save -o vehicle-semantic-1120.tar vehicle-semantic:1120
docker load -i vehicle-semantic-1120.tar
```

从团队研究工程取 `YOLOP/YOLOP_B1/weights/B1.4-road-aligned-research.pth`。也可以使用部署手册提供的冻结 V6.26.zip 和 `concept_yolop.weights` 导入器。B1.4 长度 15,293,077 字节，SHA-256 为 `19d6eeed99fee90a46f09761746845afcf330974a8e1aff057055697a02706a6`。H12 随源码提供，SHA-256 为 `e87f84dd1a1e3ef4a7df21ca9b879f9e7b142573501937e1985e74632aec0df6`。

在当前目录创建本机 `.env`，下面两项换成本机的绝对路径。输出目录先建好。Windows 路径可以写成 `E:/.../文件`。

```ini
CONCEPT_WEIGHTS_FILE=/data/weights/B1.4-road-aligned-research.pth
CONCEPT_EVIDENCE_DIR=/data/foxy-runs
CONCEPT_TARGET_MPS=2.0
```

```sh
docker compose build driver
docker compose run --rm --no-deps driver python3 /opt/concept/unity/gpu_check.py
docker compose up -d --no-build --pull never
docker compose logs driver
```

出现 `READY concept-unity/1` 后使用相邻 `host` 文件夹里的 Unity 程序。每次 Unity 建立连接，桥会走正式 ROS restart 服务，清除上一场道路记忆。每次改车端源代码均重新执行 `docker compose build driver` 和 `docker compose up -d --no-build --pull never`。构建时会重新编译该源码的六个 Cython 模块。运行结束用 `docker compose down`。证据写入每个会话独立的 `<CONCEPT_EVIDENCE_DIR>/<session>/frames.jsonl`，每行包含接收的传感器、不含图片的回复、整个车端决策及候选。原始合成 JPEG 和实际物理/CAN 回读保存在 Unity 输出目录。

## 驾驶链路与时间

Unity 渲染 11/19 的左目去畸变视角，尺寸 640×360、焦距 246.085754 像素、光心 (328.656494, 189.011826)，相机离地 0.55 米、向左 0.0600025 米、向前 0.055 米、上仰 1°。车端按 `honda.json` 投影一次。输入话题依次为 `/concept_sim/camera_info`、`/concept_sim/camera`、`/concept/vehicle_state` 和 `/concept/gate/status`。标准节点向 `/concept/decision` 和 `/concept/auto_cmd` 发布，桥等待与当前图相同的时间戳。桥只为仿真提供执行状态，用部署版物理换算把 Twist 变为 CAN 整数。手柄接管、正式 GateNode、stock MotorNode 及 SocketCAN 驱动属于单独的门控验证。

Foxy 与所有 DDS 话题只在私有容器内运行，`ROS_LOCALHOST_ONLY=1`、独立 domain 117。只有一次单向发起的 Windows/Linux 主机 TCP 连接进入 `127.0.0.1:16826`。负责端口转发的 sidecar 不运行 ROS。驱动容器单独挂在 Docker 内部网络上，使用 GPU，不挂载物理 CAN、相机或车载设备。

Unity 每拍发带 `concept-unity/1`、`kind=frame`、`seq`、`simulation_time_s`、图像大小、左右轮 RPM、ROS 正方向陀螺和上拍执行值的 JSON 行，紧随 `image_bytes` 个 JPEG 字节。首拍从零开始，之后每拍带 `applied_seq`、`applied_right_can_rpm`、`applied_left_can_rpm`，桥会检查它们是否匹配上一条命令。回复为带同一 `seq` 的 JSON 行。Unity 每次收到一条有效命令才推进下一帧物理，断线由 Unity 将左右 RPM 归零。

闭环使用仿真时钟。输入的轮速、陀螺和图像在同一仿真拍采样，推理期间仿真暂停。因而 ROS `capture_age_s=0` 和 `sensor_age_s=0` 是时钟定义，不是车端速度指标。实际墙钟包括两种可复核测量：

- 容器回复 `wall_ms` 从 JPEG 解码前计到收到匹配的 ROS Twist，包含 DDS、推理、规划。Unity 的 `exchange.jsonl` 用 `(can_set_utc_us-capture_start_utc_us)/1000` 记录拍照到实际设入 CAN 的毫秒数，包含传输、JPEG 编码和文件写入。
- `wall_probe.py` 用同段原始 Unity JPEG、轮速、陀螺和已执行指令，以指定频率独立推送给正式节点，使用系统时钟及车端现有 0.5 秒图像时限和 0.15 秒轮速时限。它分别列出每帧返回、超时、缺失与命令延迟。已解码图像在发往 ROS 时打时间戳，因此不含场景渲染、JPEG 编码和传输。该实验为固定输入开环重放，不反映替换命令后的运动。开发机 GPU/CPU 墙钟不能作为 Jetson 实车实测。

将 `/path/to/straight01` 换成含 `exchange.jsonl` 和 `camera_*.jpg` 的 Unity 输出。先结束正在运行的 Unity 场景，随后暂停本项目的常驻车端。第二条命令应无输出，这表示没有本项目常驻模型占用 GPU，接着一次性回放只加载一份 B1.4。

```sh
docker compose stop driver ingress
docker compose ps --status running -q driver
docker compose run --rm --no-deps -v /path/to/straight01:/input:ro driver python3 /opt/concept/unity/wall_probe.py /input /evidence/wall-straight-5hz.json --hz 5
docker compose up -d --no-build --pull never
docker compose logs --tail 4 driver
```

宿主有 Python 时，在上面的 `docker compose run` 命令前加 `python monitor_gpu.py gpu.csv --` 即可逐条落盘记录整机 GPU 利用率和显存。最后两条命令恢复常驻服务，并可查看 `READY` 日志。此检查只统计本项目这套 compose，其他项目也在同时使用 GPU 时请单独记录。

对照实际保存的两端记录可运行 `python3 /opt/concept/unity/audit_wire.py /input/straight01 /evidence/<session>`。把 Unity 输出只读挂载到 `/input`。检查逐帧候选所选 Twist、真正写到 Unity 的 CAN 整数，以及下一帧读回的左右整数。最后一帧没有下一张图，其设入值仍写在 Unity 日志里。
