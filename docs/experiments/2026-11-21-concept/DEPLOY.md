# 1121 Concept 车机部署

| 用途 | 固定位置 |
|---|---|
| 准备分支 | `experiment/vehicle-prep-20261121`，本目录 |
| 运行源码 | `experiment/concept-1121-offline` 的 [6793a13c0a1a22c127accb452ef65a206d6f415d](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/6793a13c0a1a22c127accb452ef65a206d6f415d) |
| 车机独立源码和工作区 | `~/src/neo-concept-1121` 和 `~/concept_1121_ws` |
| 车机底层 | ROS 2 Foxy、`/home/workspace/install`、同机 B1.4 权重及原有 Qt/Torch 运行环境 |

## 构建并替换快捷方式

先停稳并退出正在运行的驾驶方案。在车机 `control` 用户的干净 Bash 中运行。准备目录同时检出昨天的文件，以复用同一套快捷方式迁移与备份逻辑。

```bash
(
  set -e
  url=https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git
  branch=experiment/vehicle-prep-20261121
  prep="$HOME/neo-1121-concept-prep"
  if [[ ! -e "$prep" ]]; then
    git clone --depth 1 --filter=blob:none --no-checkout --single-branch --branch "$branch" "$url" "$prep"
    git -C "$prep" sparse-checkout set docs/experiments/2026-11-21-concept docs/experiments/2026-11-20-mapassist
    git -C "$prep" checkout "$branch"
  else
    test "$(git -C "$prep" remote get-url origin)" = "$url"
    test "$(git -C "$prep" branch --show-current)" = "$branch"
    test -z "$(git -C "$prep" status --porcelain)"
    git -C "$prep" pull --ff-only origin "$branch"
    git -C "$prep" sparse-checkout set docs/experiments/2026-11-21-concept docs/experiments/2026-11-20-mapassist
  fi
  cd "$prep/docs/experiments/2026-11-21-concept"
  python3 shortcuts.py
  bash prepare-concept.sh
)
```

安装器校验固定源码、底层包、12 个 Concept 模块、H12、B1.4 的单图 CUDA 推理，以及启动参数。缺少 B1.4 时使用 `bash prepare-concept.sh --weights /实际路径/B1.4-road-aligned-research.pth`，或传旧部署包 `V6.26.zip`。11/21 只在新工作区构建，已有安装用于现场回退。

## 核对后启动

在新开的交互式终端运行以下命令，确认 PATH 实际选中的入口和启动参数。准备脚本不会启动车辆节点。

```bash
prep="$HOME/neo-1121-concept-prep/docs/experiments/2026-11-21-concept"
source "$prep/verify-shortcuts.bash"
concept-live --verify-install
concept-live --show-args
```

直接运行 `concept-live` 会提示选择 **1 Pure Pursuit** 或 **2 LYA**。正式动车前，按原接线核对手柄、物理急停、左右轮速、陀螺和相机，并在面板确认电机输出仍禁用。复现离线短测的速度设置可显式传入：

```bash
concept-live tracking_controller:=lya target_speed_mps:=2.0 max_auto_speed_mps:=2.0 \
  min_nonzero_speed_mps:=0.5 lya_v_t_mps:=2.0
```

启动后在另一个终端加载 `~/concept_1121_ws/activate_concept.bash`。核对 `ros2 param get /concept/drive min_nonzero_speed_mps`、`ros2 param get /concept/drive lya_v_t_mps` 和面板上的实际速度上限。显式传入的车速和上限优先，省略时读取已有保存值。最低非零速度与 LYA 参考速度按启动参数读取。目标车速低于有效非零下限时，前进候选无法执行。[车端接线和话题](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/6793a13c0a1a22c127accb452ef65a206d6f415d/vehicle_concept/AGENTS.md)

## 回退

禁用电机输出，停稳后退出本次 launch。在上述 `prep` 中执行 `python3 shortcuts.py --rollback`。1121 的回退清单位于 `~/.local/state/aiformula/concept-shortcuts-1121/`，用来恢复升级前的文件内容、权限和链接。若升级前使用 1120，恢复后在新终端执行 `concept-live --verify-install`，检查其是否加载昨天的工作区。[1120 Concept 部署](../2026-11-20-mapassist/concept/DEPLOY.md)
