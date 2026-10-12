# Diagnostic deployment

本目录交付默认关闭的 Unity 诊断接线，不交付已通过的红灯左转功能。先读 [README](README.md) 和 [交接](AGENT_HANDOFF.md)。原生 ROS 接线仍按 `workspace/src/aiformula/launchers/SEMANTIC_DEPLOYMENT.md` 使用 shadow-only 输出；本目录有 `COLCON_IGNORE`，不要加入 colcon 源根。

要求现有兼容 Simulator、Unity 2022.3.22f1、原 A3 权重/LP/LYA 和 Python ≥3.11。实际测试环境是 CPython 3.12.10；本目录不分发模型或安装环境。NumPy/OpenCV 仅供独立 offline observer 使用，不自动接入运行时。

`install.py` 继承已验证 final63 的全 payload/prerequisite hash 预检与逐文件写入保护；本 overlay 的 before 是已经安装 frozen final63 的项目。原生 semantic 0.2.0 不在修改范围。任何 drift/已有 added-file collision/mixed state 都拒绝 apply/rollback，禁止 force；保留完整 receipt。Windows clone 需要支持 long paths。

在本目录执行只读检查，替换为本机项目绝对路径：

```powershell
python -B install.py --action verify-payload
python -B install.py --action inspect --simulator 'C:\absolute\Simulator'
python -B install.py --action verify-before --simulator 'C:\absolute\Simulator'
```

预检全通过、取得共享 Editor/CPU 所有权并关闭 Unity 后，才能执行写入。保存命令 JSON 回执：

```powershell
python -B install.py --action apply --write --unity-closed --simulator 'C:\absolute\Simulator'
python -B install.py --action verify-after --simulator 'C:\absolute\Simulator'
```

随后使用 Unity MCP 重新打开/刷新兼容项目，检查编译错误并拍新完整基线。不要使用 Computer Use，不保存试验变更到场景。现有用户项目若已经是 after 状态，仅 verify-after，不重复 apply；中间 acquisition 状态不能自动转换。当前实装来源是严格两次 hash-guarded backup，见 evidence。

`EnableJunctionCaptureMetadata` 是 runtime bool，默认 false。只有独立诊断 harness 显式开启才添加 `aiformula_junction_capture/v2`；保存源绝对时钟、render pose、Body audit、RGB input SHA 和同响应原始 lane PNG。它不调用 observer/association，也不改变目标或命令。UI RGB 是重编码，不能提供原 PNG byte identity。停止后恢复开关；旧 epoch 响应不得进入新上下文。

Python 候选实验分别在自己的 import 根运行；不要把两个同名 `observer` 模块混在同一进程。transport、曲线、association 的已有测试回执分开记录，不能相加当作功能通过。安装器 tests 和真实 payload 临时项目 roundtrip 的回执发布在 evidence，绝不以 live Simulator 做 apply/rollback 测试夹具。

关闭 Unity、再次预检 after 全通过后回滚：

```powershell
python -B install.py --action rollback --write --unity-closed --simulator 'C:\absolute\Simulator'
python -B install.py --action verify-before --simulator 'C:\absolute\Simulator'
```

回滚仅恢复列出的精确 before bytes，移除 hash 完全一致的新增文件；不删除目录、不修改另一对话文件、场景、相机、车辆、参数或官方图。中途失败保存 completed-path 回执，inspect 后逐项处理，不能清空整个项目。

任何下一次真实试验都须重新取得独占所有权、拍 fresh source/Edit baseline、使用新 case、设 wall/sim/no-progress 上限并检查 cleanup/PID。0.1.0 是诊断版本；要声称左转可部署，须另有真实内外圈左入/通过/退出与红/绿回归视频、新的 source identity 和版本。

`experiments/official_observer_quantized_curve_review3` 已在独立进程通过33项合成方法，但不在 guarded installer 的 payload 中，不能据此启用导航。原RGB/plane45项与真实703-mask批次仍待执行；只有取得新的CPU/Unity独占窗口后才能继续。Video03只展示两场绿灯元数据采集，完整解码与采样展示检查不替代红灯左转功能验收。
