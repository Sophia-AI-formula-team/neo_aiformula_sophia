# Panel-only 接线独立审查

2026-11-16。本审查只读 SIM agent 的 staging 4 文件、pre-v1 原件与测试脚本/receipt，没有修改其文件，没有运行第二套测试，也没有操作 Unity。结论：**未发现阻止进入明确标记的 Unity 仿真实试的接线缺陷。** 这不是 C# 编译、闭环或实车验收。

## 已核对的代码边界

- `AIFormulaRoadDetector.cs:58,131,200`：唯一新选项 `UseCalibratedSignalPanelOnly` 为非序列化 auto property，默认 false；RGB disabled 时显式拒绝启动；每次真实请求传布尔值。没有修改原图像、相机、mask 或 lane 请求。
- `AIFormulaJointTopoTrial.cs:66,152,160,325`：试验拒绝 RGB disabled 配置，保存原值后临时启用，cleanup 恢复原值；配置与事件明确记录模式。未改变起点、两圈判据、实体灯调度、TTL、原 LYA 或四点侧线选择。
- `worker.py:187–206`：仅显式 `signal_panel_only=true` 使 tracker 接受新模式；在 model.predict 前拒绝非布尔配置、RGB disabled、pipeline_state=None。旧 identity guard=true 请求明确报错，避免旧新模式叠加。该项是迁移保护，不是模型 fallback。
- `signal_panel_rgb.py:160,179–196`：only + None calibration 变成 `{}`，不允许进入原 offline 未标定通道；只把参与颜色决策的模型 state 置 unknown。原 HSV、形状、背景、暗边、校准几何、时间关联与两帧确认代码不变；原 model_state/detections 保留；fresh only 帧增加 policy/effective 诊断。重复或乱序仍 fresh=false、unknown，不复用旧颜色。
- 默认关闭分支与 pre-v1 的模型/RGB 优先级、返回 definition 和完整字段保持；新模式仍接受原 RGB red 优先于 green。detections 仅按原规则辅助已满足身份约束的 RGB 候选排序，不能单独生成颜色。
- 原 RGB overlay 函数、模型 overlay 生成、range 诊断代码不变；本次 diff 没有新增距离到 semantic 的接线。semantic 0.4.0、官方图、投票、unknown/red latch、LYA 与 TTL 均不在改动范围。

## 测试证据独立阅读

检查了 `verify_staged_panel_only.py` 的实际导入路径及 `staged_panel_only_verification.json`。测试明确载入 staging helper/worker 与 pre-v1 原 helper，OpenCV 单线程，未导入或运行模型。13 项窄边界通过；5 场 3,268 张已保存真实 JPEG，39.70 秒内完成：

1. 默认关闭逐帧**完整字典**等于 pre-v1 原 helper，不只比较最终颜色。
2. 开启时除 raw model_state/模式 definition 外，全部原核心字段逐帧等于 pre-v1 `model_state='unknown'` 的结果；enabled policy/effective 字段也被检查。
3. 输入图像 SHA 与 detections 深比较保证没有修改原观测。
4. 无 panel 的模型 red 变 unknown；真实单帧 panel 不确认，第二张不同真实红灯图确认 red，即使模型是 green；重复帧不重放颜色。
5. None/空/无效 calibration 均 unknown；RGB disabled、缺 frame state、旧 guard 冲突、非 bool 请求均在推理前被拒绝。

原测试夹具 seq443+444 的第二帧本来没有 panel，初次失败后改用真实 seq441+442；receipt 已保留此事实，没有通过放宽 runtime 门槛修测试。测试没有单独新增“两已确认 RGB 相反色并存”案例，但选择分支和确认循环对 pre-v1 原件无行为改动，属于已读源码保留性质；无需为本次简单替换扩大测试或改原算法。

## 审查固定版本

| staging 文件 | SHA-256 |
| --- | --- |
| AIFormula/Runtime/Perception/AIFormulaRoadDetector.cs | `3cdcb5b1345df4db864204deb97948d67f76c6b86d03f92fcbe28e5fad7e8bc8` |
| AIFormula/Editor/Validation/AIFormulaJointTopoTrial.cs | `800e531be28c94a6afed0c61d5b94dbd8718d8703ec55231c0cc24f9a5525ff6` |
| StreamingAssets/RoadDetector/worker.py | `9ea581515738c8a8b900185405e2d7c01e5ff3fafd8d183e16207e7910a32a22` |
| StreamingAssets/RoadDetector/signal_panel_rgb.py | `a473e9eb81ee009f7b5eaaed5fbefc743b06f375b8cdd695f86137c0a60431cf` |

部署归档与 active identity helper 精确移除由 SIM/主管处理；本审查没有以“代码已无 import”冒充安装文件已经清理。源/LYA/bbox 模式、历史 v1/v2 原件应继续保留既定版本边界。

## 保留的限制

只对该 Unity 已知发光面板与现有标定支持新模式；不适用于一般交通灯模型替代。真实五场反事实信号证据详见 `SIM_PANEL_ONLY_SIGNAL_ANALYSIS.md`：inner05/06 静止时真绿可形成原 2/3 票；inner04 没有足够静止绿证据；outer01/02 没到第二圈红灯，不存在该停车位的验证。保存 JPEG 与推理 PNG 有差异，换信号时刻会改变后续轨迹，因此必须真实 MCP 重跑。

本接线没有解决 official topo 歧义或 inner06 的侧线选点不足；这些失败仍须独立报告，不能声称“建图正确”或“红灯左转成功”。
