# Semantic MapAssist 0.4.5 车端准备记录

2026-11-19 JST。Neo main 的 `d8277bd2b4ce95bcab2883e9b68b011749c0d9ad`（PR #45）已合并 0.4.5，包括同名的 `semantic_planner_core`、`semantic_local_planner_ros` 和独立入口 `semantic_vehicle_045.launch.py`。GitHub 另有 `backup/pre-1027simtest-20261104`，其中 local planner 为 0.3.1，core 为 0.2.0。11:05 JST 查到车端实际旧版为 local planner 0.2.3、core 0.2.0、motor 0.4.1。Neo 的 `experiment/import-1027simtest-20261104` 是匹配版本的快照。逐文件比对，73 个 Git 文件中 56 个与车端字节一致，17 个未部署到车端，现有文件零差异，详见 [旧版哈希对照](legacy-compare.json)。

10:49 起多次尝试车端 SSH，连接超时或返回 `No route to host`。至 11:04:48 恢复连接，该时点前未写入车端。已在本地从指定提交打包两个包和诊断启动入口。源包 SHA-256 为 `a1922dde66e4cdb3c4cb589c094b34f64b544b9ceb68ce80a3112461d622716a`，65 个 Python 文件通过 Python 3.8 语法解析。

## 已验证的诊断接线

| 来源 | 端口 |
| --- | --- |
| A4 / A4Light 灯色 | `/aiformula_perception/yolop_a3/traffic_light/state` |
| 车道 ABC | `/processed_point_a`、`/processed_point_b`、`/processed_point_c` |
| OA 状态 | `/point_flag` |
| 实际停车距离 | `/flag_stop` 为代码默认名，车端发布者、语义和频率待核对 |
| 里程计 | `/aiformula_sensing/gyro_odometry_publisher/odom` |
| LYA 速度入口 | `/aiformula_control/game_pad/cmd_vel` |
| 诊断速度输出 | `/semantic_drive/map_assist_045/diagnostic_cmd_vel` |

两个 ROS 包在本机 Jazzy 构建完成。35 项测试与 4 项子测试通过。10:50:59 在本机隔离 DDS 域中验证所有上述订阅、诊断输出、红灯加距离信号的等待行为及绿灯恢复。距离 Bool 使用明确标识的合成话题。物理速度和 CAN 话题均无 MapAssist 发布端。详见 [DDS 证据](dds-probe.json) 和 [测试输出](focused-tests.log)。

## 车端安装与 Foxy 验证

11:05:25 在车端实验目录中备份旧源码、两处安装目录和既有启动配置。0.4.5 两包已构建并安装到 `~/semantic_045_ws`。原工作区和正在运行的 Concept 保持工作，测试结束后只停了隔离域中的 MapAssist。需要查看新安装时，在新 Bash 终端执行 `source ~/semantic_045_ws/activate_semantic.bash`。

Foxy 实测发现上游启动中的节点专属默认 YAML 压过了通配符覆盖，导致路由落回旧默认。车端仅修复启动参数合并，将继承值和诊断覆盖作为一份明确绑定 `/map_assist` 的参数交给 ROS。规划包源和算法不变。[Foxy 补丁](foxy-parameter-precedence.patch)

11:06:57 Jetson Foxy 隔离 DDS 测试通过。核对诊断速度、状态、路径、ABC 发布，所有订阅为 reliable／volatile。使用合成距离 Bool，确认红灯缺距离时等待、距离为 True 时首圈等待、转绿恢复。MapAssist 对车辆速度输入和物理 CAN 两个话题的发布者均为零。[车端结果](foxy-dds-fixed-result.json)

当前车辆节点图由 Concept 占用，未发现 `/flag_stop`、`/point_flag`、A4 灯色、LYA 速度或旧 `map_status` 的活动发布者。停车距离端口仍需核实真实来源、类型、频率、QoS 与 True/False 含义。0.4.5 的 `ubs_auto_turn_v1` 状态与旧 motor 0.4.1 的 `stop_wait` 契约不同，当前速度只接诊断输出。

本地材料待今天结束后汇总发布。

## 电机和红灯输入复核

11:09 JST 复核 Neo 0.4.5 的提交范围、原车底层文件对照和 A4/A4Light 的部署路由。motor 0.4.1 未被本次安装修改，早前实车安装的 `gate.py` 和 `motor_controller.py` 已通过文件一致性检查。兼容缺口来自接收方要求的 `traffic_policy: stop_wait` 与 0.4.5 诊断状态的字段契约。

当前 YOLOP 发布车辆灯色 String 和红像素数 Int32，未在部署路由中发布 `/flag_stop`。0.4.5 将灯色和停车距离 Bool 分开；将灯色直接转换为停车距离会改变首圈停车触发时机。本次追加 SSH 核对时车辆再次失联，未新增 ROS 实测或改写配置。

## A3 V14 和旧 stop_wait 契约

A3 V14 官方发布的 ROS 节点输出灯色、红像素数和带相机时间戳的检测 JSON。旧 `semantic_vehicle.launch.py` 使用 `a3_signal_capture` 和 `signal_capture_bridge` 将识别颜色与源时间戳送到 `/semantic_drive/map_signal`，交给旧 MapAssist 的 `stop_wait`。捕获器将 `distance_m` 和 `uncertainty_m` 设为 null，A3 没有提供新版独立的 `/flag_stop`。这次两次 SSH 复核均不可达，结论来自锁定的 A3 V14 发布版和 Neo 的旧车端接线代码。

[stop_wait 预览适配](stop_wait_preview.py) 读取 0.4.5 诊断状态，保留源时间戳并仅向 `/semantic_drive/map_assist_045/stop_wait_preview` 发布旧门控可解析的 JSON。映射红灯等待、未知或输入不齐、`prepare` 和 `detour` 为 `wait`。仅在新版正常、稳定绿灯且视线／OA／分支状态齐备时产生预览 `normal`。该节点不发布电机 session 或正式 `map_status`。

11:17:50 将转换结果喂给 Neo 原版 motor 0.4.1 的纯软件 MotorGate，验证等待清空自动速度、手柄优先、绿灯后要求新速度及状态失联停车。11:18:12 用实际 0.4.5 诊断节点和合成 ROS 话题通过本机隔离 DDS 测试，确认红灯等待和绿灯恢复的预览状态。第一个 DDS 探针没有等待异步适配状态，在修正测试等待后通过。见 [门控测试](stop-wait-contract-test.log) 和 [DDS 记录](stop-wait-dds-result.json)。

车端仍不可达，适配尚未实装或接入正式门控。还需确认真实停车距离信号及其失联语义、实际车端 A3 路由、旧 MapAssist 退出后的单一 status/session 发布者，以及 OA 接管、绿灯恢复和电机停车的静止验收。

## 首圈停车与匹配后绕行

前述 `stop_wait_preview.py` 仅验证旧 motor JSON 接口。它将 `detour` 映射为等待是诊断默认行为，不能作为最终适配，否则会阻断新版已经规划出的绕行。

- 首圈参考线尚未生成时，确认红灯后，0.4.5 将 `traffic_stop_flag=True` 作为停车距离触发。未收到 Bool 时也会进入 `red_wait_flag_unavailable`，已明确收到 `False` 则继续沿现有路线接近。确认绿灯后复位并等待新鲜 ABC。YOLOP 的 `unknown` 同时覆盖无有效识别，当前实现保持已锁住的红灯停车。
- 参考线存在后，0.4.5 先进入 `prepare`。检查地图融合、车辆位置匹配、路径可用且达到切入条件后进入 `detour`，开始发布绕行 ABC。路径或定位不满足条件时仍使用停车距离信号决定降级等待。已提交的绕行遇到绿灯仍完成并交还车道视觉。
- 正式验收需要把新版绕行 ABC 经原 OA 仲裁送到 LYA，并确认 LYA 实际跟随绕行轨迹。旧门控的停车状态转换不能单独替代这条轨迹链。

## 非红灯解除首圈等待

按现场确认的规则，收到并投票确认绿灯、黄灯或明确的熄灯后，可以解除红灯等待。已在本地为 0.4.5 准备 [补丁](confirmed-non-red.patch)。最新观测必须仍为可确认的非红灯。在丢帧、超时或 `unknown` 时，保留已有红灯停车。已提交绕行不会因非红灯被中断。

11:23:57 通过 54 项测试和 44 项子测试。11:24:24 使用隔离 DDS 验证红灯等待、unknown 维持等待、明确熄灯恢复与黄灯恢复，兼容预览接收同样结果。Foxy 车端补丁尚未安装，motor 及其会话没有改动。[测试输出](non-red-tests.log) · [DDS 记录](non-red-dds-result.json)

现用 A3/A4 分类器可在单个灯的检测结果记录 `yellow`，总灯色发布把黄灯与无足够颜色证据都归为 `unknown`，且没有发布明确的 `off`。上车需要绑定能区分已确认熄灯和识别失败的观测，并验证来源、时效和真实距离输入。现有话题保持原样。

## 修正后的非红灯规则

按后续明确要求，**收到新鲜有效检测消息时，`unknown` 也计为非红灯**。红灯仍按现有滑窗投票和首圈停车／匹配后绕行逻辑处理。失联、无消息和无效内容不能构成非红灯投票，已触发的停车保持。旧 motor 未改动。当前 YOLOP 的 `unknown` 可以用于这条规则，依赖的是新消息里的状态，无需额外发布 `off`。

[fresh-non-red.patch](fresh-non-red.patch) 替代上一版仅确认明确非红的候选。11:25:19 通过 55 项测试及 44 项子测试。11:25:30 隔离 DDS 验证红灯停车、无消息保持、新鲜 `unknown` 恢复，以及熄灯和黄灯恢复；相关输出仍仅为诊断。SSH 无路由，补丁尚未装车，未推送 GitHub。

## 11:28 车端复联与接线核对

11:28:31 安装并验证最新 `fresh-non-red.patch`。车端 MapAssist 0.4.5 的真实检测消息 `unknown` 现计作非红；无新消息仍不能解除已有停车。原状态包有备份，motor 两个 Python 文件的 SHA-256 前后一致。

11:29:18 安装独立 [诊断启动](commissioning/semantic_045_commissioning.launch.py)。MapAssist 的 ABC 进入基于原 `withoutkalman_0312` 的 OA 切换器，选中的位姿和角速度进入原 `lya_follower_connected_omegat_global`；LYA 速度回到新版 MapAssist 的速度门。切换源时清除旧点缓存。11:29:45 在 Jetson Foxy 隔离域以合成 ABC 和里程计验证走 MapAssist、OA 接管、切回、拒绝不完整的 OA 三点、红灯诊断速度归零、新鲜 unknown 恢复。实测结果在 [车端证据](commissioning-result.json)。

另有 [新版状态到旧 stop_wait 试验适配](commissioning/guarded_stop_wait.py)。有有效绕行路线和新鲜受控 LYA 速度时，可向旧门控描述为可走；真正的 `wait` 仍停车。使用与车端字节一致的旧 0.4.1 软件门控验证通过。车端隔离测试的状态只发到 `/semantic_drive/map_assist_045/stop_wait_candidate`，未发布正式 `/semantic_drive/map_status`、session 或 CAN。

查到车端 A3 V14 发布的是 `red_pixels_count` Int32 和灯色 String。老代码确实有像素阈值，Semantic 旧颜色适配用 600 将像素数转换成 String；另一条旧 OA 路径用 600 触发轨迹回放。车端代码和当时 ROS 图都未找到 `/flag_stop` 的 Bool 发布者。0.4.5 中它仍是停车距离输入，未替换为像素阈值。正式门控仍需核对实际停车来源和唯一的速度、状态与会话所有者。

11:30:34 Concept 按现场要求关闭，先禁用输出再退出；本轮未启动正式 Semantic 或 motor。

## A3 像素停车位置和绕行优先级

11:34 JST 车端起用 [A3 停车位置适配](commissioning/pixel_stop_flag.py)。专用诊断入口从 YOLOP 带原始相机时间戳的 `detections` JSON 读取 `red_pixels_count`，默认 `>=600` 在 `/flag_stop` 发布 `True`，低于阈值发布 `False`。阈值可在启动参数 `red_pixel_stop_threshold` 配置。重复、过期、无效数据或冲突发布者不推新值；MapAssist 诊断入口将无新 Bool 的 0.5 秒超时显示为未知。

首圈保留红灯、停车位置和非红恢复规则。匹配完成后，若当下定位、确认红灯和可行的绕行轨迹均满足，像素到点可以触发并提交该轨迹；`/flag_stop=True` 不再把可用绕行覆盖成直行停车。定位或绕行不可用仍可按像素到点回退停车。回归测试验证了待到点、到点开始绕行、持续红灯以及不可行回退；见 [增量补丁](pixel-detour-priority.patch) 与 [Foxy 回归](pixel-detour-native-tests.log)。

11:35:47 Jetson Foxy 隔离 DDS 通过，599 像素仍在接近，600 像素停车，像素输入断流后旗标变未知且车端诊断仍保持等待，新鲜非红消息可恢复。OA 选择与 LYA 的私有速度输出同轮复验通过，正式电机和 CAN 发布端为零，motor 两个程序文件的 SHA-256 未变。[车端结果](pixel-commissioning-result.json)

## 归档复现路径

从 Neo d8277bd 取两包和原诊断启动。对仓库源码依次应用 [fresh-non-red.patch](fresh-non-red.patch) 与 [pixel-detour-priority.patch](pixel-detour-priority.patch)。在启动文件所在目录依次应用 [Foxy 参数优先级](foxy-parameter-precedence.patch) 和 [像素旗标超时参数](pixel-planner-launch.patch)。

专用诊断入口与同目录脚本放到工作区的 commissioning/，并将本目录的 [stop_wait_preview.py](stop_wait_preview.py) 一并放入该运行目录供状态转换模块导入。入口从该目录的上一级按 source/workspace/src/aiformula/launchers/launch 查找修补后的启动文件。旧隔离探针 commissioning_probe.py 使用显式合成距离话题，新探针 pixel_commissioning_probe.py 使用默认 /flag_stop 和带源时间戳的合成 A3 检测。两者都要求隔离 localhost ROS 域。


## 午后真车红灯排查

15:33 的现场快照、最新 MapAssist 接线文件、旧 motor 的静态 CAN 验证及快捷方式已归档。[红灯事件与验证](red-stop-field/README.md) · [最终接线与启动](red-stop-field/fix/README.md)
