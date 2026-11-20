# 1121 车端部署 · OA 1B 0.1.2

`mapassist_b` 是第三个终端的可选入口，一次启动原 MapAssist、OA 2.1 和独立的 `oa_1b` 灯态复核节点。实验包的 Git 分支为 `experiment/mapassist-b-1121`，基于 1121 准备分支 `dff29d12`。安装范围是 OA 1B 专用发行目录、一个用户命令和一个桌面图标。手柄仲裁、停车等待状态机、车道点、避障轨迹及转向约束都沿用车端版本。

## 四个终端

| 顺序 | 命令 |
|---|---|
| 1 | `allnodes` |
| 2 | `yolop`，原菜单可选 `1`（A3 V14）、`2`（A4）或 `3`（A4L） |
| 3 | 旧版用 `mapassist`。实验外圈用 `mapassist_b outer`，内圈用 `mapassist_b inner` |
| 4 | `lya` |

等第三个终端显示 `OA1B_CAMERA_READY` 以及 MapAssist 启动信息，再开 LYA。新增桌面图标为 **1121 MapAssist B**，默认外圈，右键 **Inner** 选内圈。实验默认以 **50 个去重红像素**作为停车位置起测阈值，例如 `mapassist_b inner 50`。内、外圈最小转弯半径继续用 6.0 m、5.5 m。旧版参数用法见 [1120 部署说明](../../../../../docs/experiments/2026-11-20-mapassist/AV314_DEPLOY.md)。

在第三个终端按 Ctrl+C 或关闭窗口，会结束本次 MapAssist、OA、选择器、停车仲裁和 OA 1B 节点。Allnodes、YOLOP、LYA 继续由各自终端管理，整车结束时在各自终端退出。在相同车机用户、ROS 域和 `XDG_RUNTIME_DIR` 下，两个第三步入口共用一把原启动锁。切回旧版时先退出 `mapassist_b`，再开 `mapassist`。

## 部署 agent 安装

用原车用户从独立检出目录安装。OA 1B 使用 ROS Foxy、系统 Python、OpenCV 和 NumPy，由原 `yolop` 入口决定模型与运行环境。安装脚本比对两份原 MapAssist launch 的 SHA-256，并检查系统 Python 能导入 OA 1B。现场版本不一致时，记录路径和实际摘要并先回报。

```bash
git clone --filter=blob:none --sparse --branch experiment/mapassist-b-1121 \
  https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git ~/neo-1121-mapassist-b
cd ~/neo-1121-mapassist-b
git sparse-checkout set workspace/src/aiformula/perception/oa_1b
git log -1 --format='%H %s'
bash workspace/src/aiformula/perception/oa_1b/scripts/install.sh
type -a allnodes yolop mapassist mapassist_b lya
```

安装器只构建 `oa_1b` 到 `~/aiformula_runtime/2026-11-21-mapassist-b/<完整 Git 提交>`。若 `~/.local/bin` 不在 PATH，用 `~/.local/bin/mapassist_b`。如果装过 0.1.1 或要撤下新版，先退出第三步。核对 `readlink ~/.local/bin/mapassist_b` 指向本实验发行目录，并核对 `1121-mapassist_b.desktop` 的 `Exec`，再移除这两个实验入口。升级时随后运行新版安装脚本。安装脚本遇到同名自定义入口会停止。

## 接线与现场核对

| 方向 | 话题 |
|---|---|
| 原版供给 OA 1B | `/aiformula_sensing/zed_node/left_image/undistorted` 和 `/aiformula_perception/yolop_a3/detections`，按相同源时间戳和相机坐标系配对。里程计及 MapAssist 状态用于确认车停稳 |
| OA 1B 到原 MapAssist | `/oa_1b/traffic_light/state`（String）与 `/oa_1b/flag_stop`（Bool）。仅本实验启动实例改订阅路径。逐帧来源、原灯态和原／去重像素见 `/oa_1b/detections` |
| 原选点、避障和 LYA | 原 `/original_processed_point_{a,b,c}` → MapAssist，原 OA 2.1 带同组 A/B/C 的 `/semantic_drive/map_assist_045/oa_triplet` → 选择器 → LYA |
| 到电机 | LYA 的 `/semantic_drive/map_assist_045/lya_cmd_vel` → MapAssist 的 `/semantic_drive/map_assist_045/diagnostic_cmd_vel` → 原电机仲裁和 CAN |

整条链只有原 `yolop` 执行模型推理。OA 1B 在 CPU 上对检测框内的红像素去重。红灯已确认且车停稳时，记录原检测框中的红色亮区。原检测暂报 `unknown` 时，只在同一区域检查当前红色连通区域的面积、宽高比、填充率和重叠。连续两帧满足才补红，检测到绿灯会立即取消关联。复核输出标明 `oa_1b_red_region`。现场应记录同戳原输出、实验输出、首次达阈值的距离以及停稳后灯牌可见性。台架数据见 [验证记录](VALIDATION.md)。
