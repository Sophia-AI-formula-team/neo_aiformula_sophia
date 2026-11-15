# Calibrated RGB panel-only：最小替换设计，尚未编码

日期：2026-11-16。当前只读检查已冻结的 identity-guard v2 C#/Python。等待独立 panel-only 实际帧评估；本文件不代表批准部署，不改变 sealed v2、live、semantic、LYA、mask、相机或场景。

## 单一运行语义

新增唯一非持久 C# `UseCalibratedSignalPanelOnly`，默认 false；request 字段 `signal_panel_only`。启用时最终交通颜色仅来自**当前图像中满足现有标定身份约束、原 photometric/shape 条件和两帧确认**的 RGB panel。模型检测的 class10/color 保留用于原始证据，但不再单独驱动最终红/绿状态；没有当前确认 panel 就 unknown，不能回退模型色、重放旧色或依赖灯位/真值。

该模式替换 v2 identity guard，不叠加。最终活动代码删除 `EnableModelLightIdentityGuard`、`signal_panel_model_identity_guard` 及 `signal_model_identity` import；v1/v2 全部原件保留在 history。默认关闭时保持最初“模型色 + RGB supplement”的完整结果与 schema，不新增 effective/模式诊断字段。旧 Neo/bbox 和其他模式不被 Joint 的临时配置永久改变。

## 最少代码接点（待评估通过后实施）

| 文件 | 替换内容 | 必须保留 |
| --- | --- | --- |
| `AIFormulaRoadDetector.cs` | 将 v2 property/request 改成 `UseCalibratedSignalPanelOnly` / `signal_panel_only`；仍非持久、默认 false。 | 原 RGB capture、mask、detections、camera、lane_control、LLP 参数；不接四点短侧候选。 |
| `AIFormulaJointTopoTrial.cs` | 保存旧 panel-only 值、启用 true、cleanup 恢复；configuration/started event 明确模式。要求 `panelRgb=true`。 | 原内外起点、两圈、真实实体灯调度、TTL、重启/清理、轨迹计数。 |
| `RoadDetector/worker.py` | 用单一 `signal_panel_only` 参数调用 tracker；去除 v2 guard 参数。 | `traffic_light_original`、detections、原模型 overlay、原 RGB overlay 边框及颜色证据，原 range 诊断。 |
| `RoadDetector/signal_panel_rgb.py` | tracker 新默认 false 参数；only 时令 `effective_model_state="unknown"` 后继续原 RGB 候选/两帧确认/红优先流程；删除 v2 filter 调用。 | 全部原候选几何/HSV/边框/背景门槛、同色两帧关联、序号/时间门槛、fresh/stale 行为。 |

不需要改 `AIFormulaSemanticPlanning.cs`、follower、semantic worker、semantic source、LYA 或 lane-publisher；其他两个 C# 继续以原 frozen bytes 存档部署。

### 防止配置意外回退

Joint 在创建输出/进入 Play 之前拒绝 `panelRgb=false`。worker 也应在 `signal_panel_only=true` 但 `signal_panel_rgb` 非 true（或缺少所需 pipeline state）的情况下显式报告配置错误；不能因现有 `if signal_panel_rgb` 分支未进入，就把未经过滤的模型色送入控制。该新增配置拒绝只针对显式新模式，不改变默认请求。

tracker 原 `panel_candidates(..., calibration=None)` 是给旧 offline caller 的未标定兼容入口。**only 模式不能落入此入口**：运行端已经传 `camera or {}`，tracker 也应将 only + None 明确视为缺少校准（例如传 `{}`）；缺失/无效标定时保持 unknown，不能用未标定候选补回颜色。不要放宽 `project_panel_geometry`。

### 输出与视频语义

- `model_state` 永远保留真实模型原色；`traffic_light_original` 同样保留。
- 新模式 fresh 帧添加 `effective_model_state="unknown"` 和清晰 `signal_policy="calibrated_rgb_panel_only"`。最终控制色仍读取实际 `signal_panel.state` / `traffic_light_state`，不误把 effective_model_state 当控制结果。
- `source` 沿用 `rgb_panel` 或 `none`；确认/候选/原因等原字段保留。只在启用时更新 definition，避免默认 schema 或文本变化。
- `detections` 与模型 overlay 原样；已有 RGB panel label 可以继续加在 overlay 上。视频同时分开显示 raw model / final signal / policy；不要通过改标签掩盖模型误检。
- 原 `signal_range` 可保留为 RGB panel 诊断，但 Joint 的 semantic 请求继续只发颜色，**不增加距离输入、不混用 bbox 距离试验**。
- 重复/乱序 frame 仍 fresh=false/unknown，不复用旧确认或模型色；关闭模式的原返回字典必须逐项一致。

## 归档、回滚和最终六文件部署

先将当前 v2 **五个变化文件**（RoadDetector、JointTopoTrial、worker、signal_panel_rgb、signal_model_identity）和验证 receipt/manifest 对应 pin 保存在 `history/perception_identity_v2/`；现有 `history/perception_identity_v1/` 不动，原始 `before/` 不动。

最终 clean-baseline 安装集合为 6 个：原 SemanticPlanning、原 Joint follower、本次 RoadDetector、本次 JointTopoTrial、worker、signal_panel_rgb。原始 baseline 没有 identity helper，最终 payload 不再安装它，活动 Python 不保留 dead import。v2→最终 live 的 identity helper 删除由主管按精确 v2 SHA 单文件处理并记录迁移 receipt；不能让 clean-baseline installer 接受未知中间文件或递归清理。

manifest 应撤下 activity overlay 中的 identity 文件，但 history 的原件继续校验。当前 `.gitattributes` 由主管添加，必须原样保留并在最终 archive_files 纳入 SHA，避免 autocrlf 改写 sealed bytes。对已测试 v2 的旧 receipts 不改写成 final-mode 结果。

## 编码前/后最小验证要求

1. 等独立实际帧评估报告：真 red/green 的确认时刻、持续性和模型假灯抑制；分清保存 JPEG 与原 PNG 推理、实车与该 Unity 灯牌几何。不能仅统计 false red 消失而忽略真 red 也消失。
2. 默认 false 全字典与**原 pre-v1 helper** 相同；raw model_state/detections/模型 overlay 内容不改。
3. 新模式必须无 model fallback：模型 red + 无合格 panel → unknown；合格 panel 单帧未确认 → unknown；第二独立帧同色确认 → 相应色；缺校准 → unknown；RGB disabled → 配置拒绝。
4. 原 duplicate/out-of-order、两帧时间窗、红绿冲突优先规则不变；不以重复帧累积确认。
5. 在明确最终 hashes 上通过 MCP 重跑内外圈，分别保留失败/成功；红停、绿恢复、odom closure、official fusion、实际内转五项独立判定。假红减少不能代替 topo 建图成功。

本设计结束状态：没有代码变更、没有新增执行模式运行、没有 live 部署；最终结果与最低必要修改待评估决定。

## 后续实施回执

主管批准后已按本设计完成 staging 四文件替换，并交独立审查；`PANEL_ONLY_INDEPENDENT_REVIEW.md` 未发现阻塞问题。`staged_panel_only_verification.json` 记录 13 项边界与五场 3268 帧，39.70 s：默认关闭全字典等于 pre-v1，only 核心等于原 helper 输入 unknown，原 RGB/detections 不变；缺标定、禁用 RGB、旧 guard 冲突均已检查。初始两帧测试夹具包含原本无 panel 的 seq444，修正为实际 seq441/442 后通过，未改运行门槛；原失败写入 receipt。

v2 五 payload、旧 runner 与旧 manifest 已保存 `history/perception_identity_v2/`；final archive 为六 payload，正式 staging/overlay 已按精确 SHA 移出 identity helper，无 dead import，`.gitattributes` 纳入 manifest。runner 仅移除已删除 helper 的强制 hash 清单项，其他输入清单不动。没有 live 写入；live 迁移删除、C# 编译及真实 MCP 两圈由主管执行，结果仍 pending。四点短侧候选始终不部署；semantic 仍原 0.4.0。
