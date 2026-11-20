# 1121 车端部署 · OA 1B 0.1.1

`mapassist_b` 是第三个终端的可选入口，一次启动原 MapAssist、OA 2.1 和独立的 `oa_1b` 灯态复核节点。实验包的 Git 分支为 `experiment/mapassist-b-1121`，基于 1121 准备分支 `dff29d12`。安装范围是 OA 1B 专用发行目录、一个用户命令和一个桌面图标。手柄仲裁、停车等待状态机、车道点、避障轨迹及转向约束都沿用车端版本。

## 四个终端

| 顺序 | 命令 |
|---|---|
| 1 | `allnodes` |
| 2 | `yolop`，实验模式在原菜单选 `1`（A3 V14 / AV314） |
| 3 | 旧版用 `mapassist`。实验外圈用 `mapassist_b outer`，内圈用 `mapassist_b inner` |
| 4 | `lya` |

等第三个终端显示 `OA1B_CAMERA_READY` 以及 MapAssist 启动信息，再开 LYA。新增桌面图标为 **1121 MapAssist B**，默认外圈，右键 **Inner** 选内圈。实验默认以 **50 个去重红像素**作为停车位置起测阈值，例如 `mapassist_b inner 50`。内、外圈最小转弯半径继续用 6.0 m、5.5 m。旧版参数用法见 [1120 部署说明](../../../../../docs/experiments/2026-11-20-mapassist/AV314_DEPLOY.md)。

在第三个终端按 Ctrl+C 或关闭窗口，会结束本次 MapAssist、OA、选择器、停车仲裁和 OA 1B 节点。Allnodes、YOLOP、LYA 继续由各自终端管理，整车结束时在各自终端退出。在相同车机用户、ROS 域和 `XDG_RUNTIME_DIR` 下，两个第三步入口共用一把原启动锁。切回旧版时先退出 `mapassist_b`，再开 `mapassist`。

## 部署 agent 安装

用原车用户从独立检出目录安装。保留现有 ROS Foxy underlay、1120 MapAssist/OA 2.1 安装、AV314 环境和旧快捷方式。安装脚本比对两份原 MapAssist launch、AV314 权重及运行模块的 SHA-256。现场版本不一致时，记录路径和实际摘要并先回报。

```bash
git clone --filter=blob:none --sparse --branch experiment/mapassist-b-1121 \
  https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git ~/neo-1121-mapassist-b
cd ~/neo-1121-mapassist-b
git sparse-checkout set workspace/src/aiformula/perception/oa_1b
git log -1 --format='%H %s'
bash workspace/src/aiformula/perception/oa_1b/scripts/install.sh
type -a allnodes yolop mapassist mapassist_b lya
```

安装器只构建 `oa_1b` 到 `~/aiformula_runtime/2026-11-21-mapassist-b/<完整 Git 提交>`。若 `~/.local/bin` 不在 PATH，用 `~/.local/bin/mapassist_b`。同名自定义入口会使安装失败。要撤下新版快捷方式，先退出第三步。核对 `readlink ~/.local/bin/mapassist_b` 指向本次发行目录，并看桌面图标的 `Exec` 指向上述用户命令，然后删掉这两个本版入口。原客户端、桌面、Bash 配置和已有 ROS 安装均保留。

## 接线与现场核对

| 方向 | 话题 |
|---|---|
| 原版供给 OA 1B | `/aiformula_sensing/zed_node/left_image/undistorted` 和 `/aiformula_perception/yolop_a3/detections`，按相同源时间戳和相机坐标系配对。里程计及 MapAssist 状态用于确认车停稳 |
| OA 1B 到原 MapAssist | `/oa_1b/traffic_light/state`（String）与 `/oa_1b/flag_stop`（Bool）。仅本实验启动实例改订阅路径。逐帧来源、原灯态和原／去重像素见 `/oa_1b/detections` |
| 原选点、避障和 LYA | 原 `/original_processed_point_{a,b,c}` → MapAssist，原 OA 2.1 带同组 A/B/C 的 `/semantic_drive/map_assist_045/oa_triplet` → 选择器 → LYA |
| 到电机 | LYA 的 `/semantic_drive/map_assist_045/lya_cmd_vel` → MapAssist 的 `/semantic_drive/map_assist_045/diagnostic_cmd_vel` → 原电机仲裁和 CAN |

已确认红灯且停稳后，只有原检测暂报 `unknown`、同一灯牌位置仍有合格红色亮区时，OA 1B 才对这一帧另做一次 AV314 推理。仍需两帧同一位置的候选通过分数、重叠和颜色检查。原 YOLOP 的发布、置信度和起停均独立。新模式会增加一个 AV314 模型的显存和少量触发帧的推理负载。现场应记录两进程 GPU、同戳原/实验检测、首次达阈值的距离及停稳时灯牌是否持续可见。Foxy 台架的数据和现场验证范围见 [验证记录](VALIDATION.md)。
