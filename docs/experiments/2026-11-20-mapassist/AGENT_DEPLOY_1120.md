# 11 月 20 日车机部署交接 · Concept 和 MapAssist

**给车端部署 agent：用车上的 `control` 用户，在 11/19 能访问 ZED、CAN、GPU 和桌面的 Foxy 环境执行。** 本文覆盖取代码、识别器迁移、两个工作区清理重编、快捷方式、接线、验收与回退。第 1 节可整段执行，到“软件安装完成”就会退出。第 2 节在操作员终端核对入口，第 3 节留给车辆停稳、现场人员到位后的分套通电检查。交接前记录第 4 节的结果。

| 要部署的部分 | 取哪里的代码 | 完成后从哪里启动 |
|---|---|---|
| MapAssist `1120-av314.1`、分组 OA2 和红灯停车 | 团队 neo 的 [1120 准备分支](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/experiment/vehicle-prep-20261120/docs/experiments/2026-11-20-mapassist)，本次核查起点 `779593e`；建 5 包 | 四个独立终端依次运行 `allnodes`、`yolop`、`mapassist`、`lya` |
| MapAssist 的识别选择器 | 保留 11/19 的 A3 V14；将 A4 和 A4Light 的运行代码更新至 [220f8cba](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/220f8cba0b96c9bb9ac2873b2685c91da2277e03) | 第二个终端的 `yolop` 菜单。复现 AV314 选 **1 · A3 V14** |
| Concept V6.27.8、B1.4、H12 和控制器选择 | [部署分支](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/feat/concept-v626-live)的固定提交 [f305d5f7](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/f305d5f7acea5c87af0e2003bec13bdc06199d8f/vehicle_concept)；12 个 Concept 包加专用 `launchers` | 一个 `concept-live` 或原 **Concept** 图标。回车或选 **1** 为 Pure Pursuit，**2** 为内置 LYA |

### 1　一次完成安装与干净重编

先让车停稳、退出旧的 Concept 或四终端方案，停止录包，确认其他终端、容器和 ROS 域均已释放相机与 CAN。由一个部署终端执行整段，等待先前的独立 `prepare.sh`、`prepare-concept.sh` 或手工 colcon 编译退出。用没有加载旧实验 overlay 的新 Bash。保留车辆实际的 `ROS_DOMAIN_ID`、设备映射和显示环境。安装会使用两个相互独立的编译目录，分别记录退出码；车机内存吃紧时可先运行 `export SERIAL_1120=1`，依次编译。

车上原有 B1.4 会直接复用。如果缺少，请先运行 `export B14_INPUT=/实际路径/B1.4-road-aligned-research.pth`，也可指向团队的 `V6.26.zip`。11/19 的车载代码归档另行提供 A3/A4 权重和已安装节点。需要恢复底层文件时，先解到空目录，与车上逐项核对；归档含有已安装版和源码有差异的 LYA。整棵覆盖或从 main 重建会改变这些实车参数。[11/19 车辆运行包与校验值](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/b866165366d53875402d61f2d8bab258ea0f1c4e/docs/experiments/2026-11-19-yolop-selection/active-code/README.md)

```bash
bash --noprofile --norc <<'DEPLOY1120'
set -Ee -o pipefail
umask 022
trap 'echo "1120 部署失败（行 $LINENO）；记录目录：${R:-未创建}" >&2' ERR
[[ $(id -un) == control ]] || { echo '请在车载 control 账号执行'; exit 1; }
cd "$HOME"
url=https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git
branch=experiment/vehicle-prep-20261120
toolsrepo="$HOME/src/neo-1120-field-prep"
P="$toolsrepo/docs/experiments/2026-11-20-mapassist"
M="$HOME/aiformula_runtime/2026-11-20-mapassist-av314"
C="$HOME/concept_1120_v6278_ws"
selector="$HOME/aiformula_runtime/2026-11-19-yolop-selection"
A4=220f8cba0b96c9bb9ac2873b2685c91da2277e03
pin=f305d5f7acea5c87af0e2003bec13bdc06199d8f
src="$HOME/src/neo-concept-1120-v6278"
mkdir -p "$HOME/src" "$HOME/.cache" "$HOME/aiformula_runtime/1120-deploy-records"
exec 8>"$HOME/.cache/aiformula-1120-deploy.lock"
flock -n 8 || { echo '另一份整套部署正在运行'; exit 1; }
R=$(mktemp -d "$HOME/aiformula_runtime/1120-deploy-records/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")
echo "本次记录 $R"
exec > >(tee "$R/install.log") 2>&1

for c in git flock colcon python3 xdg-user-dir tar; do command -v "$c"; done
for f in /opt/ros/foxy/setup.bash /home/workspace/install/setup.bash "$selector/yolop.launch"; do test -r "$f"; done
case "${AMENT_PREFIX_PATH:-}:${PYTHONPATH:-}:${CMAKE_PREFIX_PATH:-}:${PATH:-}" in
  *"$HOME/concept_"*|*"$HOME/aiformula_runtime/"*) echo '此终端已加载旧实验环境；清掉自动 source 的实验 overlay，再从车辆基础环境启动'; exit 1;;
esac
desk=$(xdg-user-dir DESKTOP)
[[ -n "$desk" && $(realpath -m -- "$desk") != "$(realpath "$HOME")" ]] || { echo 'MapAssist 需要有效的 XDG 桌面，见第 4 节'; exit 1; }
base_launch=/home/workspace/install/launchers/share/launchers/launch/allnodes.launch.py
grep -q lane_0927oa "$base_launch"
df -h "$HOME"; free -h
prep_paths=(docs/experiments/2026-11-20-mapassist workspace/src/aiformula/launchers
           workspace/src/aiformula/perception/semantic_planning_system workspace/src/aiformula/control/semantic_vehicle_wiring
           workspace/src/aiformula/perception/obsticle_avoidence)
if [[ ! -e "$toolsrepo" ]]; then
  staging=$(mktemp -d "$HOME/src/.neo-1120-field-prep-XXXXXX")
  git clone --depth 1 --no-checkout --filter=blob:none --single-branch -b "$branch" "$url" "$staging"
  git -C "$staging" sparse-checkout init --cone
  git -C "$staging" sparse-checkout set "${prep_paths[@]}"
  git -C "$staging" checkout "$branch"
  mv -T -- "$staging" "$toolsrepo"
fi
test "$(git -C "$toolsrepo" rev-parse --show-toplevel)" = "$(realpath "$toolsrepo")"
test "$(git -C "$toolsrepo" config --get remote.origin.url)" = "$url"
test "$(git -C "$toolsrepo" branch --show-current)" = "$branch"
test -z "$(git -C "$toolsrepo" status --porcelain)"
git -C "$toolsrepo" pull --ff-only origin "$branch"
test "$(git -C "$toolsrepo" rev-parse HEAD)" = "$(git -C "$toolsrepo" rev-parse FETCH_HEAD)"
git -C "$toolsrepo" sparse-checkout init --cone
git -C "$toolsrepo" sparse-checkout set "${prep_paths[@]}"
git -C "$toolsrepo" rev-parse HEAD | tee "$R/prep.commit"

# 两份 Git 检出相互独立。先把两边源码取齐，之后才改车上的文件。
stage_concept() (
  set -e
  if [[ ! -e "$src" ]]; then
    tmp=$(mktemp -d "$HOME/src/.concept-1120-XXXXXX")
    git clone --no-checkout --depth 16 --filter=blob:none --single-branch -b feat/concept-v626-live "$url" "$tmp"
    if ! git -C "$tmp" merge-base --is-ancestor "$pin" origin/feat/concept-v626-live; then
      git -C "$tmp" fetch --unshallow --filter=blob:none origin feat/concept-v626-live
    fi
    git -C "$tmp" merge-base --is-ancestor "$pin" origin/feat/concept-v626-live
    git -C "$tmp" sparse-checkout init --cone
    git -C "$tmp" sparse-checkout set vehicle_concept workspace/src/aiformula/launchers \
      workspace/src/aiformula/vehicle workspace/src/aiformula/control/motor_controller
    git -C "$tmp" checkout --detach "$pin"
    mv -T -- "$tmp" "$src"
  fi
  test "$(git -C "$src" rev-parse --show-toplevel)" = "$(realpath "$src")"
  case "$(git -C "$src" config --get remote.origin.url)" in "$url"|"${url%.git}"|git@github.com:Sophia-AI-formula-team/neo_aiformula_sophia.git) ;; *) exit 1;; esac
  test "$(git -C "$src" rev-parse HEAD)" = "$pin"
  test -z "$(git -C "$src" status --porcelain)"
  test -f "$src/workspace/src/aiformula/launchers/CMakeLists.txt"
  test -f "$src/vehicle_concept/checks/check_overlay.py"
)
stage_concept >"$R/concept-source.log" 2>&1 & sc=$!
( set -e
  git -C "$toolsrepo" fetch --depth 1 --filter=blob:none origin "$A4"
  python3 "$P/update_yolop_1120.py" --checkout "$toolsrepo"
) >"$R/a4-preview.log" 2>&1 & sy=$!
rc=0; wait "$sc" || rc=1; wait "$sy" || rc=1
cat "$R/concept-source.log" "$R/a4-preview.log"; test "$rc" -eq 0
python3 "$P/shortcuts.py" | tee "$R/mapassist-shortcut-plan.txt"
python3 "$P/concept/shortcuts.py" | tee "$R/concept-shortcut-plan.txt"
( source /opt/ros/foxy/setup.bash; source /home/workspace/install/setup.bash
  /usr/bin/python3 "$P/check_vehicle.py"
  ros2 launch "$base_launch" --show-args >"$R/allnodes-args.txt"
  for k in motor_cmd_topic lane_point_topic_prefix launch_yolop_a3; do grep -Fq "'$k'" "$R/allnodes-args.txt"; done
  ros2 pkg executables trajectory_follower | grep -Fx 'trajectory_follower lya_follower_connected_omegat_global'
)
( source "$P/concept/activate_concept.bash" --base-for-build
  python3 -c 'from torchvision.ops import nms; import torch, scipy, Cython; b=torch.tensor([[0.,0.,1.,1.]], device="cuda"); print(nms(b,b[:,2],.5))'
)
model="${XDG_DATA_HOME:-$HOME/.local/share}/concept_yolop/B1.4-road-aligned-research.pth"
args=()
if [[ -n ${B14_INPUT:-} ]]; then
  test -r "$B14_INPUT"; args=(--weights "$(realpath -- "$B14_INPUT")")
  python3 - "$src" "$B14_INPUT" <<'MODEL1120'
import runpy, sys
from pathlib import Path
w = runpy.run_path(str(Path(sys.argv[1]) / "vehicle_concept/src/concept_yolop/concept_yolop/weights.py"))
with w["_source_stream"](sys.argv[2]) as stream:
    w["verify"](stream)
MODEL1120
else
  printf '%s  %s\n' 19d6eeed99fee90a46f09761746845afcf330974a8e1aff057055697a02706a6 "$model" | sha256sum -c -
fi
python3 "$P/update_yolop_1120.py" --checkout "$toolsrepo" --apply | tee "$R/a4-apply.log"
python3 "$P/update_yolop_1120.py" --checkout "$toolsrepo" | tee "$R/a4-after.log"
grep -Fxq 'Files requiring update: 0' "$R/a4-after.log"

# 只清两个新工作区。先验证全部四个路径，再把原有产物存到本次记录，最后删、重编。
python3 - "$R" <<'CLEAN1120'
import os, re, shutil, subprocess, sys
from pathlib import Path
home = Path.home().resolve()
record = Path(sys.argv[1]).resolve(strict=True)
if record.parent != home / "aiformula_runtime/1120-deploy-records":
    raise SystemExit("备份目录必须在部署记录目录内")
roots = [home / "aiformula_runtime/2026-11-20-mapassist-av314", home / "concept_1120_v6278_ws"]
targets = [root / name for root in roots for name in ("build", "install")]
mounts = []
for line in Path("/proc/self/mountinfo").read_text().splitlines():
    mounts.append(Path(re.sub(r"\\([0-7]{3})", lambda m: chr(int(m[1], 8)), line.split()[4])))
for p in targets:
    if p.resolve() != p or (p.exists() and not p.is_dir()) or any(p == m or p in m.parents for m in mounts):
        raise SystemExit("路径经过链接、挂载点或不是目录，保留原样：" + str(p))
active = [p for p in targets if p.exists()]
if active:
    archive = record / "previous-build-install.tar"
    helpers = [roots[0] / "check_vehicle.py", roots[1] / "activate_concept.bash"]
    subprocess.run(["tar", "-C", str(home), "-cf", str(archive), "--",
                    *(str(p.relative_to(home)) for p in active + [p for p in helpers if p.is_file()])], check=True)
    subprocess.run(["tar", "-tf", str(archive)], stdout=subprocess.DEVNULL, check=True)
    for p in active:
        print("清理", p, flush=True)
        shutil.rmtree(p)
if any(os.path.lexists(p) for p in targets):
    raise SystemExit("旧编译目录仍存在")
print("四个旧 build/install 均已确认不存在", flush=True)
CLEAN1120
map_rc=0; concept_rc=0
if [[ ${SERIAL_1120:-0} == 1 ]]; then
  bash "$P/prepare.sh" >"$R/mapassist-build.log" 2>&1 || map_rc=$?
  bash "$P/concept/prepare-concept.sh" "${args[@]}" >"$R/concept-build.log" 2>&1 || concept_rc=$?
else
  bash "$P/prepare.sh" >"$R/mapassist-build.log" 2>&1 & mp=$!
  bash "$P/concept/prepare-concept.sh" "${args[@]}" >"$R/concept-build.log" 2>&1 & cp=$!
  wait "$mp" || map_rc=$?
  wait "$cp" || concept_rc=$?
fi
printf 'MapAssist=%s  Concept=%s\n' "$map_rc" "$concept_rc" | tee "$R/build-status.txt"
tail -n 6 "$R/mapassist-build.log" "$R/concept-build.log"
test "$map_rc" -eq 0 && test "$concept_rc" -eq 0 || exit 1
python3 - "$M" "$R" <<'RECEIPT1120'
import json, shutil, sys
from pathlib import Path
p = max(Path(sys.argv[1]).glob("deployment-*.json"))
d = json.loads(p.read_text())
shutil.copy2(p, Path(sys.argv[2]) / p.name)
bad = [k for k,v in d["matches_tested"].items() if not v]
missing = [k for k,v in d["packages"].items() if not v.get("prefix")]
print("AV314 指纹", d["matches_tested"], "缺少的包", missing)
if bad or missing or d["worktree_status"] or d["commit"] != (Path(sys.argv[2]) / "prep.commit").read_text().strip():
    raise SystemExit("请逐项核对 AV314 收据和查询到的包；安装结果待核实。")
RECEIPT1120
"$HOME/.local/bin/concept-live" --verify-install >"$R/concept-packages.txt"
"$HOME/.local/bin/concept-live" --show-args >"$R/concept-launch-args.txt"
echo "软件安装完成。现场核对从第 2 节开始。记录：$R"
DEPLOY1120
```

编译时可在另一终端对脚本第一行打印的记录目录运行 `tail -F 路径/mapassist-build.log 路径/concept-build.log`。MapAssist 和 Concept 的构建进程各自继承同一份车载基础环境。它们的快捷方式安装器会在写桌面时共用一把锁。两个返回码都为零、AV314 收据内 8 个指纹一致且 13 个依赖包可解析，才会打印完成。备份、录包、B1.4、旧 venv、相机配置和 `/home/workspace` 都在四个清理目标之外。

### 2　操作员看到的入口

在桌面正常启动的**新交互式 Bash**（会读取用户真实的 alias 和 PATH）中粘贴。两次检查都返回 0，且五个命令都指向 `~/.local/bin/`。

```bash
P="$HOME/src/neo-1120-field-prep/docs/experiments/2026-11-20-mapassist"
source "$P/verify-shortcuts.bash" && source "$P/concept/verify-shortcuts.bash" &&
  concept-live --verify-install && concept-live --show-args
```

桌面应有四个编号的 MapAssist 图标，以及沿用原名的 **Concept**。如果部署时 XDG 桌面指向隐藏目录，运行 `xdg-user-dir DESKTOP` 查实际位置。Concept 图标的 `Terminal=true` 让使用者能键入 1 或 2。终端出现旧 alias、function、第二份同名命令时，用 `type -a 名称` 找出具体路径并核对用户启动文件。原车 init 沿用原配置。

### 3　接线和分套验收

**两套软件可以同时装，车辆一次只启动一套。** 在相同 ROS 域开观察终端。MapAssist 的第三个入口直接以 `vehicle_output:=true` 启动，完整四终端属于实车联调；现场人员按车队流程隔离牵引输出、确认遥控急停和停稳状态后再启动。仅做软件移植时做到第 2 节即可，将本节记作待车验。跨方案切换先退出前一整套。各入口的锁只覆盖各自的命令，孤立的旧 YOLOP 或 LYA 可能仍在运行。查 `ros2 node list`、相关进程与物理设备占用；仅切换 ROS 域不能隔离同一个 CAN 或相机。

| 实车接口 | 使用哪条连接 | 静止时核对什么 |
|---|---|---|
| ZED X 左目 | 原 ZED 接线。图像 `/aiformula_sensing/zed_node/left_image/undistorted`，Concept 配对 `/aiformula_sensing/zed_node/left/camera_info` | 确认有新图、有对应时间戳、frame 和尺寸。V6.27.8 默认请求采集和发布 **60 Hz**、缩图系数 3，规划请求 15 Hz；实测另记。固定安装约 0.55 m；Concept 优先读 `${XDG_CONFIG_HOME:-~/.config}/concept/camera.json`，其次 `honda-ground.json`。记录实际选中路径。 |
| VectorNav、底盘反馈 | 原串口默认 `/dev/ttyUSB0`、115200；`/aiformula_sensing/vectornav/imu`。Kvaser 接 `can0`，经典 CAN 500000 bit/s；`/aiformula_sensing/vehicle_info` 中 0x711 提供左右轮反馈 | 用 `ls -l /dev/serial/by-id/ /dev/ttyUSB0` 和 `ip -details -statistics link show can0` 查实物。惯导有更新、停车时两轮接近零。CAN 缺口或串口号变化先查原车传感器初始化和设备映射。 |
| DualShock 4 | 原 `joy_node` 的 `/aiformula_control/joy_node/joy` → 原 L2/R2 映射 → `/aiformula_control/joy_node/joy_l2_throttle` | 空闲有心跳。在牵引隔离下观察 Concept：有手柄输入即接管，识别继续；松手停车保持暂停，点面板「继续」才恢复自动资格。 |
| 最终电机 | 两套共用 `/aiformula_control/motor_controller/reference_signal` → socketcan sender → 0x210。独立硬件急停沿用原接线 | 用 `ros2 topic info --verbose` 核对发布者和订阅者身份。Concept 要求 **1 发、1 收**；监视或录制 `/concept/internal/motor_can`。对最终物理话题使用 echo、hz 或录包会额外增加订阅者，触发所有权保护。 |

**Concept**　停掉四终端，在车机桌面运行 `concept-live`。终端选 1 或 2。面板初始电机禁用，确认识别、候选路线、粉色选中路线和最终箭头。选 2 时相机叠图有同帧 A/B/C。车载默认在驾驶进程内执行 B1.4，旧 A3/A4 选择器无需启动。控制路径为 `/concept/auto_cmd` → 门控 → 原电机换算 → 私有 CAN → 门控复核 → 物理发送。下面的诊断在**另一个**终端运行，读取当前值。车辆有保存过的速度设置时，以两项参数实读为准。

```bash
source "$HOME/concept_1120_v6278_ws/activate_concept.bash"
ros2 param get /concept/drive tracking_controller
ros2 param get /concept/drive target_speed_mps
ros2 param get /concept/gate max_auto_speed_mps
ros2 topic info --verbose /aiformula_control/motor_controller/reference_signal
ros2 topic info --verbose /concept/gate/status
ros2 topic info --verbose /concept/decision
for t in /aiformula_sensing/zed_node/{left_image/undistorted,left/camera_info} \
         /aiformula_sensing/{vehicle_info,vectornav/imu} /aiformula_control/joy_node/{joy,joy_l2_throttle}; do
  ros2 topic info --verbose "$t"
done
```

**MapAssist**　先禁用 Concept 输出、停稳并退出整个 Concept launch。关闭加载过 Concept 的诊断终端。为复现 AV314 内圈，在四个新终端依次执行下表，每个保持运行。桌面第三个图标的右键动作 **AV314 inner (450 px)** 等价于第三步。

| 终端 | 命令 | 连接对上才继续 |
|---|---|---|
| 1 | `allnodes` | 从 **/home/workspace/install** 读取 11/19 整机 launch，启动 ZED、底盘、手柄、`lane_0927oa` 和旧 motor。内嵌 YOLOP 关闭，A/B/C 输出 `/original_processed_point_{a,b,c}`。 |
| 2 | `yolop`，选 **1** | A3 V14 发布检测和灯色。也可显式 `yolop a3`。另测 A4/A4Light 时先退出本终端，再用 `yolop a4` 或 `yolop a4l`，记录实际档位；三档共用车上的感知话题。 |
| 3 | `mapassist inner 450` | MapAssist、停车像素适配器、OA2、选择器和 motor session 一起启动。内圈半径 6.0 m，450 是灯框内红像素数。`mapassist inner` 回到 600；`mapassist` 是外圈 5.5 m、600。 |
| 4 | `lya` | 读取私有 `filtered_lane_pose` 和 `filtered_omega_t`，发送 `lya_cmd_vel`；MapAssist 的停车/OA 门控处理后，通过 **`/semantic_drive/map_assist_045/diagnostic_cmd_vel` 送给原车 motor**。直接运行未 remap 的 LYA 会绕开这条接线。 |

观察终端仅加载原车基础环境。前三点、两个旗标、整组 OA 和最终速度都要核对实际发布者。检查完先停稳，再按 **LYA → MapAssist → YOLOP → Allnodes** 退出。

```bash
source /opt/ros/foxy/setup.bash && source /home/workspace/install/setup.bash
ns=/semantic_drive/map_assist_045
for t in /original_processed_point_{a,b,c} /aiformula_perception/yolop_a3/{detections,traffic_light/state} \
         /point_flag /flag_stop "$ns"/{oa_triplet,oa_ready,filtered_lane_pose,filtered_omega_t,lya_cmd_vel,diagnostic_cmd_vel} \
         /semantic_drive/{map_status,motor_session} /aiformula_control/motor_controller/reference_signal; do
  ros2 topic info --verbose "$t"
done
timeout 5 ros2 topic echo "$ns/pixel_stop_status"   # 约 5 秒后主动结束
```

`/point_flag` 来自 OA，`/flag_stop` 来自停车像素节点。OA 启用后应收到新鲜的三点整组 `oa_triplet`（PoseArray，A/B/C 同窗），然后 `oa_ready` 生效。三个旧单点有流量仍需验证这条整组连接。确认红停后，unknown 或旧检测不能放行；AV314 当时内圈两组丢灯都停了，切绿后的恢复尚待重测。450 场可核对 `pixel_stop_status` 中阈值、实际像素及年龄。留意两处同名的 `data_processing_node`，按命名空间及实际连接辨认。

### 4　重试、回退和交班

| 看到的现象 | 处理 |
|---|---|
| 安装器报告旧版 `allnodes` 指纹不同，或出现 `Wrong ROS overlay` | MapAssist 需要 11/19 **车端已安装**的 `allnodes.launch.py`；Concept 需要其**专用** `launchers` 才能使 12 个底层文件全部匹配。新开终端核对 `ros2 pkg prefix launchers` 和收据中的 allnodes/LYA 文件路径。保留现场差异，按 11/19 归档逐文件比较。 |
| MapAssist 预检称 XDG 桌面已关闭 | 此版本的四图标安装器要求 `xdg-user-dir DESKTOP` 返回 HOME 内一个实际子目录。先核对原桌面设置。若现场继续使用归档中的隐藏桌面，可备份 `~/.config/user-dirs.dirs`，再执行 `mkdir -p "$HOME/.xdg-hidden/Desktop"` 和 `xdg-user-dirs-update --set DESKTOP "$HOME/.xdg-hidden/Desktop"`。Concept 单独支持禁用桌面。 |
| 权重或源文件哈希不同 | 改动前核对真实文件。B1.4 由团队的 PTH/ZIP 单独导入；A4 两档共享 checkpoint。A4 导入器会拒绝未识别的文件。若 B1.4 推理未通过，看 `concept-build.log` 的 TorchVision CUDA NMS、Qt、cv_bridge 或固定文件哈希。保留旧 `~/concept_v6275_ws/venv` 及 Jetson 的原配库。 |
| 一个构建成功、另一个失败 | 查看本次 `build-status.txt` 和两份 build.log。先修报错，保持两套停用，再重跑第 1 节。已有的第一份备份保留；再次执行会产生新的记录目录。 |
| 旧命令在取代码时提示找不到目录，目录名看起来是 GitHub URL | Foxy 的 Git 2.25.1 实测在 `clone --sparse` 指定目标目录时出现过这个错误。第 1 节分开执行 clone、设置稀疏路径和 checkout，已从空目录验证。 |
| 恢复入口或识别器 | 停止运行中的两套程序。在准备目录执行 `python3 concept/shortcuts.py --rollback` 可恢复 Concept 迁移前的入口。MapAssist 使用 **mapassist-build.log 中当次** `Backup:` 指向的 `shortcut-backup-*`，按相对 HOME 路径恢复命令、链接和图标；对预览列为 `install` 且原先不存在的入口，核实仍为本次生成内容再移走。A4 使用 **a4-apply.log 中当次** `Backup:` 指向的 `a4-before-1120-*`；在选择器已退出时，将该备份的 `bundles/` 按原相对路径拷回 `~/aiformula_runtime/2026-11-19-yolop-selection/`，再运行预览。完整回退两档应显示 6 项待更新；只回退本次的部分变更时项数与当次相符。更新 0 项的运行没有新备份。 |
| 还要恢复原有编译产物 | 使用**本次清理前**的 `previous-build-install.tar`。把 `R` 设为该次实际记录目录，先用 `tar -tf "$R/previous-build-install.tar"` 查看成员。停用两套后，将当前失败的目录分别移入记录目录、保留 workspace 层级，再执行 `tar -xf "$R/previous-build-install.tar" -C "$HOME"`。归档含原有编译目录及当时存在的激活和检查脚本。首次安装没有旧产物时，回退到保留的 11/19 工作区和快捷方式。 |

交班时附本次记录目录、两个源码提交、两项构建退出码、AV314 收据、识别档位、B1.4 CUDA 校验以及五个快捷方式的解析结果。现场核对另记相机、轮速、IMU、手柄、CAN 发布者/订阅者及人员实际观察。若只完成软件安装，把未接到的设备和未做的运行逐项写明。

### 这份交接已经验证到哪一步

用新的 `control` 账号在 Ubuntu 20.04.6 / Foxy / RTX 4070 Ti 隔离环境中，将第 1 节命令原样连续执行两次。使用 11/19 归档、真实模型和固定 Git 对象，下载通过本地 Git 镜像提供。首轮从空目录安装；第二轮备份并删除两个工作区共四个编译目录，再并发重建。第二轮 MapAssist 编译 3.19 秒、Concept 编译 35.0 秒，两份退出码为 0，A4 待更新 0 项。此处为工作站 colcon 耗时。归档的基础环境另有一条旧 `simtest_ws/local_setup.bash` 缺失提示；这两次后续包路径、指纹和导入均通过，现场同样以实际收据逐项核对。

| 隔离 Foxy / x86 工作站实际做过的检查 | 结果 |
|---|---|
| 分别删除两个已编译工作区的 `build` 和 `install`，再运行仓库原安装器 | MapAssist 五包重建，收据中 13 个 ROS 包均可解析，8/8 项文件指纹一致。Concept 十三个包重建，12/12 项底层文件一致，7 个编译模块从新路径加载。 |
| 还原 11/19 原车识别选择器与权重，应用 A4 更新，再恢复 | 两档共更新 6 个文件，再运行剩余 0 项；从当次备份恢复后，检查的 61 个文件与归档逐字节一致。 |
| 接通 MapAssist 分组 OA 与原车 LYA，注入合成锥桶掩膜 | 加入静止里程计后，34 套完整三点产生 34 条私有 LYA 指令；没有里程计时为 0 条。测试中物理 CAN 发布者为 0。 |
| Concept 用 B1.4 跑真实测试图、在终端分别选两种控制器 | 工作站 GPU 的两次输出一致。两种模式各生成 75 条候选；未登记相机几何的回放按预期保持零自动输出。快捷方式回退后，核对的 40 处都与安装前相同。 |

上述结果覆盖软件、模型和 ROS 连接。现场仍需填写 Jetson 的真实频率、相机与 CAN 连通性、手柄和急停表现，以及红灯场景能否正确停车和恢复。
