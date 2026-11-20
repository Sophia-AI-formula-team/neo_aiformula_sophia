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

安装器校验固定源码、底层包、12 个 Concept 模块、H12、B1.4 的单图 CUDA 推理，以及启动参数。缺少 B1.4 时使用 `bash prepare-concept.sh --weights /实际路径/B1.4-road-aligned-research.pth`，或传旧部署包 `V6.26.zip`。清理旧安装前，会把 11/19 Python 环境的依赖迁入 `~/concept_1121_ws/venv`，再用新位置编译和验证。

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

## 删除旧安装

新版编译通过并更新快捷方式后，安装器会暂时移走下列旧目录，再验证 `concept-live --verify-install` 和 B1.4 的 CUDA 推理。验证通过才删除目录，失败则恢复路径并报错。

- V6.27.5 的 `~/src/neo-concept-v6275` 和 `~/concept_v6275_ws`
- V6.27.8 的 `~/src/neo-concept-1120-v6278` 和 `~/concept_1120_v6278_ws`

旧源码有未保存改动、旧工作区包含布局之外的文件，或该路径是链接或挂载点时，安装器会列出对应路径并停止。共享底层、B1.4 权重、相机配置和单独存放的录包在清理范围外。最后显示 `Ready` 才表示安装与清理均完成。此后如需回到 1120，须从 GitHub 的固定版本重新安装。[1120 Concept 部署](../2026-11-20-mapassist/concept/DEPLOY.md)
