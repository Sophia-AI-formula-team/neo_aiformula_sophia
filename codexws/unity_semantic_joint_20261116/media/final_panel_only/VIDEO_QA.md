# 最终 60 秒视频验收记录

2026-11-16。视频证据交付通过；**semantic topo 融合与红灯绕行仍失败，整体实验 accepted=false**。这不是实车验收。内圈 inner_joint_07 与外圈 outer_joint_03 均实际完成两物理圈，使用同一套已封存 panel-only 接线、原 semantic 0.4.0、原 LYA。没有在视频中补画未发生的内部建图或转弯。

- 视频：`encode_v1/render/semantic_joint.mp4`，8,343,024 bytes，60.000 秒，1920×1080，15 fps，H.264/yuv420p，900 帧。
- SHA-256：`815285ea9da67903df8fc92d67deafb8845aff9159670c8c8a1c2d49a9994c9b`。
- 0–30 秒为内圈，30–60 秒为外圈。直接顺序渲染到一个编码器；没有中间视频或二次重编码。画面 `[1/2]`、`[2/2]` 明确区分独立实验。
- 第一视角为真实 detector.Input 与既有识别 overlay；原 raw model 标签保留。额外状态和地图位于下方/右方，不盖住主驾驶区域。
- 完整 MP4 已单线程解码全部 900 帧，逐帧验证 PTS=`index/15`、尺寸、格式、计数及 case/边界；`output_frames.jsonl`、`decoded_frames.jsonl`、`case_boundaries.json` 保留逐帧证据。
- Windows owned Job 一次运行 exit0，108.726442 秒，300 秒上限，cleanup_verified=true，Job ActiveProcesses=0，前后源文件 SHA 相同，无自动重试。原机器 receipt 中 visual_QA_pending 保持原样，本文件补充实际视觉检查。

## 真实功能结果与显示边界

| 记录 | 行程 / 两圈 | 红停绿行 | 官方 topo / 红灯绕行 |
| --- | --- | --- | --- |
| inner_joint_07 | 519.832642 m / 2 | 红灯停稳、停止位识别真实转绿、恢复后完成第二圈 | ambiguous_correspondence；official_unaligned；没有 detour |
| outer_joint_03 | 563.1467 m / 2 | 同上 | 同上 |

右侧实际轨迹、首次闭合 odom reference、当前 package reference 只在记录到后出现。Unity surface 是 live MCP 验证过的评价底图，不进入控制。官方 OSM 未对齐时只画在独立 inset，标 `Official / NOT ALIGNED`；统计使用 `OSM nodes`，不暗示内部节点已建成。两场终帧保留 `accepted: false`、`Red detour: FAILED` 和融合原因。

实际控制模式为 `signal_panel_only=true`、`signal_policy=calibrated_rgb_panel_only`。显示 `Signal: calibrated RGB panel; raw model display-only`；有效控制色取实际 RGB aggregate。raw model/effective model unknown 不等于控制没有看到绿灯。此片没有混用 bbox 距离或旧 stop_wait，源包确认红灯即停车的行为保留。场景真值仅评价。

## 抽帧检查与时间索引

下列均来自**完成的 MP4 实际解码**。不是预渲染图片。新增六张绿灯切换 PNG 用单线程 decode-only 提取，耗时 3.18 秒，MP4 前后 SHA 不变；其余复用完整 decode 已生成的 PNG。`qa_decoded/receipt.json` 保存图片 SHA、源帧/语义序号、motion 测量与独立时间戳。

| 视频秒 / 帧 | 现象 | 图片 |
| --- | --- | --- |
| 15.000 / 225 | 内圈真实红停，native/applied/actual 速度为 0；LLP、LYA、map failure 可读 | `encode_v1/render/decoded_0225.png` |
| 23.067 / 346 | 内圈仍静止，当前 RGB 已见绿，原 semantic 尚在 wait | `qa_decoded/decoded_0346.png` |
| 23.400 / 351 | 内圈原 semantic 转 green_resume，原恢复延迟仍保持静止 | `qa_decoded/decoded_0351.png` |
| 23.933 / 359 | 内圈 green_resume，actual_v=2.0833 m/s | `qa_decoded/decoded_0359.png` |
| 29.933 / 449 | 内圈 2 圈终帧，accepted=false；真实 stopped tail 不伪填 LLP | `encode_v1/render/decoded_0449.png` |
| 30.000 / 450 | 外圈独立冷启动；尚无第一视角时明示空缺，没有倒填未来图片 | `encode_v1/render/decoded_0450.png` |
| 45.000 / 675 | 外圈真实红停，native/applied/actual 速度为 0 | `encode_v1/render/decoded_0675.png` |
| 53.600 / 804 | 外圈静止看到 RGB green，semantic 仍 wait | `qa_decoded/decoded_0804.png` |
| 53.867 / 808 | 外圈 green_resume 的原恢复延迟，车辆仍静止 | `qa_decoded/decoded_0808.png` |
| 54.467 / 817 | 外圈 green_resume，actual_v=2.4478 m/s | `qa_decoded/decoded_0817.png` |
| 59.933 / 899 | 外圈 2 圈终帧，accepted=false 与融合失败未掩盖 | `encode_v1/render/decoded_0899.png` |

媒体 agent 目视检查 225、346、359、449、450、804、817、899；root 独立检查 225、675、359、804、450，确认主视角清楚、控制和地图失败标注可读、边界没有伪造连续运动。351/808 的状态由帧日志和对应真实 motion 核对，不把这些两张未另行目视的图声称为额外人工审阅。

`event_time_index.json` 记录每项原事件到视频首次对应帧的映射。关键时刻：内圈 reference closed 9.200 秒、确认红 10.133 秒、绕行失败/实体转绿 22.733 秒；外圈分别 38.933、40.533、53.133 秒。图像、感知与 semantic 是异步输入，保留各自 sequence、捕获时间和 age；不声称每个显示部件是同一瞬间。

## 时间压缩的限制

完整源时间线从 wall=0 延续到各自停止尾部：内圈 215.036 秒、外圈 230.834 秒。源首图之前的启动时段照实显示空缺。内圈约 11.938 秒、外圈约 12.004 秒红灯静止保护窗口保持 1×；临近事件约 1.10× / 1.21×，其余长段约 37.33× / 40.32×，画面顶部持续标明倍速。60 秒版本覆盖两个两圈记录的全时间范围，但抽样压缩不会展示每一张源帧，不能用它评估所有瞬时检测或控制抖动。真实事件、全源日志与完整视频逐帧索引仍单独保留。

## 可复现文件与依赖

`inner07_plan_v1.json`、`outer03_plan_v1.json`、`combined_plan_v1.json` 与 encoder/native library 哈希均已封存。`encoded_sources_v1/` 保存实际编码时的 renderer、owned Job wrapper/helper 和说明文档；`media_manifest.json` 给出这些文件、计划、receipt、索引和媒体的 SHA。

编码入口为 Python313 运行 `run_joint_bounded.py --plan combined_plan_v1.json --python <相同 Python313> --attempt <全新的 task/media 子目录>`。它读取原计划里的绝对源路径，源记录、images、已装 PyAV 17.1.0/Pillow、evaluation_surface.json 都必须存在且哈希相同。归档里的小文件是复现索引，不包含大源记录或视频。不要编辑旧计划来绕过校验；迁移环境需要另行制作有明确来源的新计划。

本次只写媒体制品与 QA，没有修改 semantic、Unity、录制内容、旧 sealed 方案或历史失败视频。
