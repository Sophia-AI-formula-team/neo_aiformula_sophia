# 候选部署说明

当前 **未安装/未编译/标定左转未验收**。默认校准关闭，地图输入选择待用户确认。部署不等于启用，不等于功能验证。

## 准备与字节核验

先按 [final63部署说明](../unity_semantic_integration/DEPLOYMENT.md) 准备匹配的现有 Simulator。Unity2022.3.22f1、模型 checkpoint、依赖、相机与物理 pins保持；Unity worker Python≥3.11，测试用3.12.10。原生 ROS Python≥3.8语法支持不是本 worker 的安装要求。这里不下载模型，也不创建Unity项目。

本目录 installer 只管理 manifest 的14个文件，检查416个不变前置文件，完整基线是 final63已安装状态。其他功能/原生包不变。原始任务机器已有 left-trial采集改动，是中间状态：应在 `inspect` 看到差异，不能套 `apply` 或 `rollback` 强制覆盖。保留本机 receipts，明确比对中间路径后另做受控迁移。

从 neo repo root执行，替换示例路径：

```powershell
$candidateRoot = Join-Path (Get-Location).Path 'codexws\unity_semantic_left_turn_20261012'
$simulatorRoot = 'D:\Demo_LaneLineTrainer\Simulator'
$pythonExe = 'D:\python312\python.exe'
& $pythonExe -B "$candidateRoot\install.py" --action verify-payload
& $pythonExe -B "$candidateRoot\install.py" --simulator $simulatorRoot --action inspect
& $pythonExe -B "$candidateRoot\install.py" --simulator $simulatorRoot --action verify-before
```

写入前取得独占、停止 workers并关闭Unity，保留其他工作。明确执行：

```powershell
& $pythonExe -B "$candidateRoot\install.py" --simulator $simulatorRoot --action apply --write --unity-closed
& $pythonExe -B "$candidateRoot\install.py" --simulator $simulatorRoot --action verify-after
```

installer全量预检payload和dest哈希，不写 core/upstream/weights/scene/preset，不提供force；每文件原子替换，多文件不是事务。中断须保留completed-path receipt，停止并核对混合状态，不能重跑覆盖。`--unity-closed` 是调用者确认，不是自动进程检查。

重新打开后只通过 MCP 刷新和读 console；此候选尚无编译证据。先做默认关闭回归。新runtime属性不序列化，默认null/false；CLI未传 `--calibrated-map` 时不加载新helper，继续原自动链。

## 经用户选择后启用实验

只在允许使用仿真标定后，显式设置 `CalibratedMapPath`，需要地图ABC所有权时设置 `EnableCalibratedDetourOwnership=true`。外圈 `RecordedReferencePath` 可指向本归档的 `fixtures/real_outer_fusion_capture.json`，这时该真实历史快照会成为控制reference；快照旧采集字段control_input=false只描述采集当时。它不证明本轮新学习，状态必须显示warm_start。

内圈当前**没有严格真实快照**，不能写成已就绪。先用inner配置与真实绿色首圈采集；完整snapshots通过严格检查后，才可用于后续有界红灯实验。新harness使用相同RGB链和idealUnity里程计，设置色板真值仅用于真实灯调度/评测，RGB供实际观测。标定prior坐标明确进入控制，不可归为evaluator-only。

fixture不是Simulator安装payload，`verify-payload` 不检查它。启用前独立核对原始180,111bytes及SHA256（也记录在fixtures/*.sha256）：

```powershell
$referenceFile = Join-Path $candidateRoot 'fixtures\real_outer_fusion_capture.json'
if ((Get-Item -LiteralPath $referenceFile).Length -ne 180111 -or
    (Get-FileHash -LiteralPath $referenceFile -Algorithm SHA256).Hash.ToLowerInvariant() -ne
    'c0639a94bdc1dac1bbd52947fedd1acb839215dbbe913a9417feb6495ee01338') {
    throw 'Recorded reference fixture identity differs'
}
```

以下外圈调用仅是未执行示例，须先完成编译、用户选择、独占和新的输出目录：

```csharp
return AIFormula.Editor.AIFormulaSemanticLeftTrial.Start(
    @"D:\trial-output\new_outer_calibrated_red", "new_outer_calibrated_red",
    72.53f, -10f, true, false, 10f, true, true,
    true, false, true,
    @"D:\Demo_LaneLineTrainer\Simulator\Assets\StreamingAssets\SemanticPlanning\unity_east_detour_outer_v1.json",
    @"D:\neo\codexws\unity_semantic_left_turn_20261012\fixtures\real_outer_fusion_capture.json");
```

前三个追加bool依次projectedRecovery/currentCorridor/continueAfterLearning。保留RGB freshness、支持/command gate、E-STOP和worker所有权；不要放宽时效。当前prior只配置正向02→06→03，其余缺边拒绝。

只读 `AIFormulaSemanticLeftTrial.Status()`；必要时 `Stop("Operator stopped")`。420s墙钟/240s模拟/600m/15s无进度界限保持。等清理、workers0、真实车体停止且回到Edit Mode后下一场。保存入口距离/朝向/ABC、detour全过程/exit、失败尾段、实际轨迹道路审查和视频，按VALIDATION验收。

## 离线测试与回滚

无模型/Unity/encoder活动且CPU可用时，从repo root执行：

```powershell
& $pythonExe -B "$candidateRoot\run_tests.py"
& $pythonExe -B "$candidateRoot\test_install.py"
```

模块tests不执行实际车辆运动。历史实际快照重算receipt单列，不把供应的协议tick称Unity实跑。

关闭Unity和workers后，只有精确candidate-after状态可回滚：

```powershell
& $pythonExe -B "$candidateRoot\install.py" --simulator $simulatorRoot --action verify-after
& $pythonExe -B "$candidateRoot\install.py" --simulator $simulatorRoot --action rollback --write --unity-closed
& $pythonExe -B "$candidateRoot\install.py" --simulator $simulatorRoot --action verify-before
```

回滚恢复final63受管字节、逐个移除精确匹配的新增任务文件，保留无关目录/工作；Unity未来自动生成的未受管helper `.meta` 不由installer删除。它不会重新生成scene、改变Git状态或关闭其他进程。
