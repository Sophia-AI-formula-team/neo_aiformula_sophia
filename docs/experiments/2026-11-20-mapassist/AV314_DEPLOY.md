# 1120 车端部署 · AV314、分组 OA2 和内圈 450 px 试验

**本轮内圈命令为 `mapassist inner 450`，YOLOP 选 1（A3 V14）。** 第二个参数按检测框内的红色像素数判断停车位置，省略时使用 600。内圈和外圈的转弯半径分别为 6.0 m 和 5.5 m。bbox 尺寸估距属于另一个试验。

## 先核对版本并安装

本分支整合 1120 准备分支、PR47 的红灯停车优先级，以及 AV314 实跑所用的 OA2 发布端和选择器。已有准备目录的车机请另开干净 Bash 终端，在新的检出目录执行以下命令。安装前车辆停稳，退出原来的四个终端，保留原来的 overlay 和现场改动。

```bash
git clone --filter=blob:none --sparse -b experiment/1120-av314-oa2-grouped https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git ~/neo-1120-av314
cd ~/neo-1120-av314
git sparse-checkout set docs/experiments/2026-11-20-mapassist workspace/src/aiformula/launchers workspace/src/aiformula/control/semantic_vehicle_wiring workspace/src/aiformula/perception/semantic_planning_system workspace/src/aiformula/perception/obsticle_avoidence
git rev-parse HEAD
cd docs/experiments/2026-11-20-mapassist
python3 shortcuts.py
bash prepare.sh
```

安装器构建五个包到 `~/aiformula_runtime/2026-11-20-mapassist-av314`。它检查包前缀，并比较实际导入的 OA2 模块与检出源码。四个旧 1120 命令通过已知文件哈希识别并备份。自定义的同名命令会中止安装，部署人员应先记录差异并反馈。保留安装日志打印的 `shortcut-backup-*` 绝对路径。

## 版本与安装记录

本部署标识为 `1120-av314.1`，对应 launchers 0.1.1、obsticle_avoidence 0.1.0 和 MapAssist 0.4.5。各包按自己的变更独立编号。版本清单保留 PR47 SHA、11/19 车端归档 SHA、Unity 试跑脚本提交、镜像 ID、模型与关键代码 SHA-256。镜像中的 Python、Torch 等版本仅描述此次仿真。四个接线文件在早先仿真镜像中使用 CRLF，Linux Git 版本使用 LF；已逐字节验证仅换行有差异。清单同时保留镜像原始哈希和车机安装应匹配的 Git 文件哈希。[版本清单](av314-version.json)

`prepare.sh` 在安装快捷方式前写入带 UTC 时间的 `deployment-*.json`。每次记录检出提交、分支、工作区状态、已解析的 ROS 包版本和前缀，以及 OA2、LYA、选择器、红灯适配器、原车 Allnodes 和 V14 权重等文件指纹。请把该文件与同场 bag 一起保留。分组收发端或红停适配器与试跑代码不符会中止安装；其他文件缺失或指纹不同会列在收据中。AV314 试验前核对 `matches_tested` 的每一项，差异先反馈。运行中的识别节点还应核对其实际 `weights` 参数与收据所指文件一致。

## 四终端启动与接线核对

桌面依次打开 **1120 1 allnodes → 1120 2 yolop → 1120 3 mapassist → 1120 4 lya**。四个图标各自开终端。用命令行时也按同一顺序开四个新终端，依次保留运行。默认桌面 MapAssist 使用外圈 600；内圈 450 试验在第三个终端显式运行下表命令。当前 Concept 图标和命令另由 Concept 部署人员维护。

安装后在操作员的新交互式 Bash 运行 `source ~/neo-1120-av314/docs/experiments/2026-11-20-mapassist/verify-shortcuts.bash`。该检查验证两套入口、显示四个名字的实际解析，并查找遮住新命令的 alias、function 和仍能调用的旧命令。1119 受管入口（含已知旧 Concept）、早期 1120 四命令、用户 bin 里的同内容副本以及桌面和应用菜单里的旧图标由安装器归档到不在 PATH 的备份目录。若检查发现现场另设的旧别名或函数，用 `type -a 名称` 核实后删除其用户启动文件中的对应定义，再开终端复查。现场私有的定义不在 1119 归档里，不能从仓库推断其内容。

| 顺序 | 内圈本次试验 | 核对项 |
| --- | --- | --- |
| 1 | `allnodes` | 11/19 实车 Allnodes 启动 ZED、里程计、`lane_0927oa` 和 motor。关闭内嵌识别。原车道点进入 `/original_processed_point_{a,b,c}`。 |
| 2 | `yolop`，选 `1` | 核对菜单身份 A3 V14。试跑权重 SHA-256 为 `a411c983b661d6b26217019a968251c89a6b66c95400e4e7cb78f04a644ca10c`。 |
| 3 | `mapassist inner 450` | 同时启动分组版 `oa_2`、选择器和像素适配器。外圈基线用 `mapassist outer`。同车道恢复 600 可用 `mapassist inner`。 |
| 4 | `lya` | 车上 connected global LYA 读取私有 filtered pose/omega，发往 `/semantic_drive/map_assist_045/lya_cmd_vel`。 |

车辆保持静止时，在同一 ROS 域核对实发话题。用 Ctrl-C 结束每个 echo。

```bash
type -a allnodes yolop mapassist lya
ros2 topic echo /semantic_drive/map_assist_045/pixel_stop_status
ros2 topic info --verbose /aiformula_perception/yolop_a3/detections
ros2 topic info --verbose /flag_stop
ros2 topic info --verbose /point_flag
ros2 topic info --verbose /semantic_drive/map_assist_045/oa_triplet
ros2 topic info --verbose /semantic_drive/map_assist_045/oa_ready
ros2 topic info --verbose /semantic_drive/map_assist_045/diagnostic_cmd_vel
```

- 内圈本次应读到 `threshold=450`。检测话题与 `/flag_stop` 各应有一个发布者。查看 `latest_pixels`、`age_ms` 和 `counts`，区分低于阈值、图像过期、重复时间戳与发布者冲突。新鲜红像素数达到阈值才会给出位置标志。红灯已确认后丢框，另有 `red_wait_signal_lost` 停车路径。
- OA2 的 `/cone_mask_avoider` 在一次规划窗口内按 A/B/C 顺序发布一条 `geometry_msgs/msg/PoseArray`。选择器处于 OA 模式时，接收带 `oa_window` 标记、三个有限坐标、切换后生成且年龄不超过 150 ms 的新组。旧 `/oa_point_{a,b,c}` 仍可观察。单点话题有流量不能证明新组已接通。
- 若启动提示 `Wrong ROS overlay` 或 `Unexpected OA2 Python module`，记录涉及的包、`AMENT_PREFIX_PATH` 和 `PYTHONPATH`。从新终端核对实际包路径。若现场存在其他发布端或自定义接线，先向负责人反馈再改。保持已确认红灯后的停车锁，`unknown` 不触发放行。
- motor 的自动输入是 `/semantic_drive/map_assist_045/diagnostic_cmd_vel`。名称中的 diagnostic 不能据此判断与 motor 隔离。该车端入口使用 `vehicle_output:=true`。

## 试验结果和待验证项

| AV314 单灯内圈 | 600 px | 450 px |
| --- | ---: | ---: |
| 从起点至首次停稳的累计行程 | 14.930 m | 13.188 m |
| 最大红像素数 | 80 | 61 |
| 首次停稳原因 | `red_wait_signal_lost` | `red_wait_signal_lost` |
| 真值转绿 0.5 s 后的检测 | 214 unknown | 204 unknown，4 green |
| 转绿后最大电机线速度 | 0 m/s | 0 m/s |

450 场收到 359 条 `/flag_stop`，均为 false。停车比上一场早 1.74 m，灯牌仍在画内，但绿色未达到确认条件。两次都由丢灯触发，停位差不能归因于阈值。外圈 600 的单灯场正常绿行；单独锥桶场内外圈都恢复循迹；停车后叠加锥桶的两场均维持红停，切绿后未恢复。450 仅试跑内圈单灯，整圈建图、绕行和实车放行仍待验证。[像素节点与 ROS 日志](av314-evidence/ros.log.gz) · [内圈原始话题](av314-evidence/topic-events.jsonl.gz) · [行程与真值](av314-evidence/observations.jsonl) · [结束状态](av314-evidence/result.json) · [600 px 六场记录](av314-evidence/baseline-600.json)

提交前以 Git 中的 Linux 文件在隔离的 Foxy 容器构建全部五包。DDS 回归覆盖单点乱序、整组、缺点、重复、过期、未来时间和 OA 退出后恢复视觉。旧 1120 四条快捷方式可备份升级，重复执行后备份数保持为一。通过实际安装的内圈 450、内圈默认与外圈默认三个入口，像素节点分别报告 450、600、600。容器没有车机的硬件 Allnodes、ZED 和 CAN，安装脚本中的实车 Allnodes 检查需在车机执行。[Foxy 安装与回归实录](av314-evidence/foxy-install.txt)

## 回退

只回退阈值时，停稳并退出本轮 MapAssist，再按相同车道启动省略阈值的命令。若要回退整套接线，停稳后依次退出 LYA、MapAssist、YOLOP、Allnodes。对从旧 1120 升级的车机，将安装时备份的四个 `.local/bin` 文件按原相对路径恢复到 HOME，已有 1120 桌面图标继续调用同名命令；在新终端核对命令内容指向原来的 `2026-11-20-mapassist` 目录。直接从 1119 安装的车机按[准备目录的快捷方式恢复说明](README.md)恢复对应桌面、命令和链接。保留本次部署日志与备份。
