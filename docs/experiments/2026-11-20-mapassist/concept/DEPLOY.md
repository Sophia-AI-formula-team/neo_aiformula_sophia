# 1120 车机部署 · Concept V6.27.8

**现场入口沿用原名。** 安装完成后，双击 **Concept** 或运行 `concept-live`。在弹出的终端输入 **1** 使用 Pure Pursuit，**2** 使用 LYA，回车使用 Pure Pursuit。LYA 的 A、B、C 叠在同一拍的摄像头画面上。操作员使用 Concept 时，先停稳并退出另一套驾驶方案。Concept 自己启动相机、识别、手柄、门控、面板和 RViz。

| 用途 | 来源 |
|---|---|
| 安装器、快捷方式、本文 | 团队 neo 的 `experiment/vehicle-prep-20261120` 分支，本目录 |
| 正式部署分支 | `feat/concept-v626-live` |
| 此次固定运行源码 | [f305d5f7acea5c87af0e2003bec13bdc06199d8f](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/f305d5f7acea5c87af0e2003bec13bdc06199d8f/vehicle_concept) |
| 新源码与独立工作区 | `~/src/neo-concept-1120-v6278`、`~/concept_1120_v6278_ws` |
| 现场已知旧入口 | `~/.local/bin/concept-live` 以前加载 `~/concept_v6275_ws/activate_concept.bash`。原 `Concept.desktop` 调用同名命令。 |

## 在车机安装或升级

在车机的 **`control` 用户、干净的 Bash** 中操作。第一次取准备工具时用新的检出目录。下列命令带有错误检查，某一步失败会结束这个子 shell。

```bash
(
  set -e
  prep_repo="$HOME/neo-1120-concept-prep"
  branch=experiment/vehicle-prep-20261120
  url=https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git
  if [[ ! -e "$prep_repo" ]]; then
    git clone --filter=blob:none --sparse --single-branch --branch "$branch" "$url" "$prep_repo"
    git -C "$prep_repo" sparse-checkout set docs/experiments/2026-11-20-mapassist
  else
    test "$(git -C "$prep_repo" remote get-url origin)" = "$url"
    test "$(git -C "$prep_repo" branch --show-current)" = "$branch"
    test -z "$(git -C "$prep_repo" status --porcelain)"
    git -C "$prep_repo" pull --ff-only origin "$branch"
  fi
  cd "$prep_repo/docs/experiments/2026-11-20-mapassist/concept"
  git rev-parse HEAD
  python3 shortcuts.py
  bash prepare-concept.sh
)
```

安装器先在独立工作区构建 12 个 Concept 包和专用的 `launchers`，验证依赖、识别、启动菜单，再替换快捷方式。重复执行使用同一份固定代码。车上已装的 B1.4 默认位于 `${XDG_DATA_HOME:-$HOME/.local/share}/concept_yolop/B1.4-road-aligned-research.pth`。缺少文件时，在上面命令最后一行使用 `bash prepare-concept.sh --weights /实际路径/B1.4-road-aligned-research.pth`；导入器也支持团队的 `V6.26.zip`。[安装脚本](prepare-concept.sh) · [旧车安装记录](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/b866165366d53875402d61f2d8bab258ea0f1c4e/docs/experiments/2026-11-19-yolop-selection/concept-v6275)

**为什么单独编译 launchers。** 核对了 11/19 终场的车载安装归档。只加载 `/home/workspace/install` 时 12 个被检查的文件中有 11 个吻合，差异是 `allnodes.launch.py`。当天实际运行的 Concept 用了独立的 `launchers`，按此顺序逐项核对是 12/12。新的工作区复用这一接线方式，以及已有的车辆 Python 依赖和 Jetson 的 Qt/Torch 预加载库。原车的 init、已装软件和独立急停继续使用现场接线。[归档来源](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/releases/tag/field-archive-2026-11-19) · [新版接线与话题](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/feat/concept-v626-live/vehicle_concept/AGENTS.md)

## 确认旧入口已经换成新版

安装器按归档指纹识别旧命令和图标。它先备份，再在同一位置写入新入口。命令仍叫 `concept-live`，桌面和应用菜单仍叫 **Concept**。现场隐藏桌面上的旧入口也在迁移范围内。新的桌面项使用 `Terminal=true`，选择控制器时可以直接输入数字。MapAssist 的 `allnodes`、`yolop`、`mapassist`、`lya` 保留原来的用途。单独执行的 `lya` 是 MapAssist 的 LYA 命令。

在操作员新开的交互式 Bash 中运行：

```bash
prep="$HOME/neo-1120-concept-prep/docs/experiments/2026-11-20-mapassist/concept"
source "$prep/verify-shortcuts.bash"
concept-live --show-args
```

第一项检查磁盘上的入口和当前 shell 的命令解析。若 `type -a concept-live` 显示旧 alias、函数或其它 PATH 副本，按输出定位它的定义，再打开新终端复查。第二项列出的 `tracking_controller` 默认值为 `auto`。另可运行 `concept-live --verify-install` 核对已安装的包和原生模块。输出应指向 `~/concept_1120_v6278_ws/install`；`concept`、`concept_candidates`、`concept_drive`、`concept_panel` 都是 6.27.8。

## 打开面板

直接运行 `concept-live` 后在菜单选择；脚本也可显式运行 `concept-live tracking_controller:=lya`。车静止、电机输出禁用时，在已加载新工作区的另一个终端执行：

```bash
source "$HOME/concept_1120_v6278_ws/activate_concept.bash"
ros2 param get /concept/drive tracking_controller
ros2 param get /concept/drive target_speed_mps
ros2 param get /concept/gate max_auto_speed_mps
```

核对摄像头和同戳 CameraInfo、左右轮速、陀螺以及手柄心跳。11/19 固定安装使用 `~/.config/concept/honda-ground.json` 和 0.55 m 的实测高度。启动会先查同目录的 `camera.json`，若有两份文件，核对现场实际生效的投影。面板可见本车实际目标速度和上限；没有保存值时两项缺省为 **4.0 m/s**，Unity 短测使用 **2.0 m/s**。手柄操作立即接管，松手后停车，手动点「继续」才恢复自动。[现场相机登记](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/b866165366d53875402d61f2d8bab258ea0f1c4e/docs/experiments/2026-11-19-yolop-selection/concept-v6275/field-followup/calibration/2026-11-19-calibration.md)

## 遇到问题与回退

* **模型缺失或校验失败**，B1.4 应为 15,293,077 字节，SHA-256 为 `19d6eeed99fee90a46f09761746845afcf330974a8e1aff057055697a02706a6`。H12 随代码安装，SHA-256 为 `e87f84dd1a1e3ef4a7df21ca9b879f9e7b142573501937e1985e74632aec0df6`。
* **同名快捷方式被人改过**，安装器会打印具体路径。用 `type -a concept-live`、`readlink -f 路径` 和 `sha256sum 路径` 核实来源，保留现场改动。旧工作区及其私有 venv 仍可能提供依赖。
* **需要退回原入口**，先禁用输出、待车停稳并退出本次 launch。在上述 `prep` 目录运行 `python3 shortcuts.py --rollback`。它依据 `~/.local/state/aiformula/concept-shortcuts-v6278/` 的清单恢复原内容、权限和原先没有入口的位置。若早期 MapAssist 安装已取走了旧图标，它原本的备份保存在 `~/aiformula_runtime/2026-11-20-mapassist-av314/shortcut-backup-*`。在当时就没有旧入口的机器上，可按 11/19 冻结手册从旧工作区启动。[V6.27.5 手册](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/a9d7d7a94d062ad5ff61fedfe21d3dbe94fa1bd1/vehicle_concept/AGENTS.md)

## 目前验证到哪里

V6.27.8 的四次短程 Unity 对照都通过越线检查。长弯 Pure Pursuit 走了 **51.05 m**，LYA 走了 **46.57 m**，两者都出了弯，LYA 末段停稳。默认保留 Pure Pursuit。实车转向、Jetson 拍照到发令的延迟和实车整圈需要现场复测。[同帧面板、驾驶对比视频和逐帧数据](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/f305d5f7acea5c87af0e2003bec13bdc06199d8f/vehicle_concept/checks/tracking_v6278)
