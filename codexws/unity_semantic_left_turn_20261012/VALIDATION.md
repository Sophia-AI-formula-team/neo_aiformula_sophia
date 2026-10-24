# 验证记录与尚缺验收

这些证据分属不同层，不能合成“闭环已通过”。历史receipts包含本机路径，供归属追溯；可移植测试入口是run_tests.py与install.py。

| 证据 | 已记录结果 | 范围 |
| --- | --- | --- |
| topology unit | 18 PASS | 严格配置/投影/trust/order/缺边拒绝，人工fixture |
| continuous tracker unit | 18 PASS | 真实原core返回类型/局部进度/弯/自交/拒绝/结束，离线几何 |
| transport unit | 16 PASS | 生命周期、source/捕获/闭合校验、reference identity、默认控制体 |
| 独立默认worker三tick对照 | PASS，新helper未导入 | 除明确monotonic age字段，所有输出一致；不是C#回归 |
| 外圈实际快照原builder重算 | 原失败重现、源73files不变 | 1014candidate，4321原输出数值逐项一致；RMSE3.404463>3 |
| 外圈配置投影 | 最大0.073784 m，五portal唯一/可信，方向+1 | 真实reference，人工先验关联；不是自动junction detection |
| 实际保存reference加载 | PASS | 原构造4302数值一致，加载后一次供应protocoltick，无CNN/车体运动 |
| 内外配置路径检查 | nominal中心/条件包络在道路内 | 同源场景几何，evaluation_geometry_independent=false；非真实运动/完整车体净空 |
| 可移植入口复验 | 同三套共52 PASS | portable_test_receipt.json；重复原模块测试，不新增另一套52项验收 |
| installer temporary fixture | 24 collected，23 PASS / 1 SKIP | WinError1314禁止真实directory symlink；hardlink/reparse检查通过，非live Simulator |
| installer verify-payload | 14 files / 19 payload sides / writes0 | 精确源payload校验；fixture另行校验 |
| editor/source恢复 | 453保护文件，4项已声明采集改动，其余一致；core73files不变 | MCP原scene干净Edit，preset/bodies/请求逐字段一致，scope workers0，console无error |
| C#编译、默认Unity回归 | 未执行 | candidate未安装 |
| 标定内外圈L–R–L闭环 | 未执行 | 用户地图选择待确认，内圈缺snapshot |

相关原件见evidence/topology_verification.json、continuous_detour_tracker_unit_receipt.json、calibrated_transport_receipt.json、actual_warm_start_transport_receipt.json和configured_*_path_check.json。C#最终hash/patch同步见csharp_receipts_sync.json；原生成后有人工修订，不能用旧generator一键再现最终字节。

归档入口与安装器的复验分别见 [portable_test_receipt.json](evidence/portable_test_receipt.json)、[installer_verification.json](evidence/installer_verification.json)；完整恢复对照见 [editor_source_restoration.json](evidence/editor_source_restoration.json)。原始worker清理由各场result记录，本次额外通过MCP和指定process-command路径查无存活workers/encoder。

实际失败实验的configuration/result在evidence/runs，视频完整解码2441/2441、无decoder错误，接触表10帧已视觉检查；这不是逐帧视觉QA。该视频明确标失败，不是修复后左转成功视频。

## 后续实际验收

- 用户允许标定，或选择并实际修复自动matching；保留具体问题分析和来源。
- 精确candidate安装/编译/console无错误；默认关闭时正常红绿灯链行为回归。
- 内圈取得自己的真实闭合candidate snapshot；严格原构造一致。外圈warm使用已保存真实snapshot，fresh_learning_proof=false。
- 两起点分别实跑：真实红灯票、地图入口≤2 m、身体朝向和首次ABC/LYA请求，物理左入、内部右转、左出、明确退出和未跳跃进度。
- 逐段核对command/odom/RGB age、worker/epoch与控制owner，失败时安全停止。currentCorridor=false首次组合，不弱化TTL。
- 审查真实车体轨迹及道路越界、实际车体姿态/净空限制；配置静态路径不能替代。相机、灯位、physics/core/upstream哈希保持。
- 单独保留每场成功/失败视频：真实overlay、LLP/LYA、semantic输出，主画面无遮挡占75%；视频解码/帧数、关键状态视觉与轨迹QA。
- 清理workers0/terminal速度与errors，恢复原editor/preset/bodies/Play状态并明确归还Unity。

仍不构成反向/任意路口绕行、自动新路口学习、native DDS、Orin吞吐或实车验收。
