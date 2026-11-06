# 横移桶前的两种选择

[同场原速录像](paired-crossing.mp4) · [车头进入前的实际画面](before-entry.jpg) · [配对审计](primary.json)

原来七段录像中，从首次让行到车越过最后一只桶的站位或发生接触，共 201 次决策。185 次处于让行，目标速度全部低于同期车速；16 次已进入恢复。近距通过上限为 1.40 m/s，候选比例为 1.00、0.68 和 0.40。旧训练评分先看完圈和净距，再比较无障碍区速度。见[逐片结果](frozen_choices.txt)。

## 同一场景的开关对照

保持种子、起步姿态、车线观测器、A4、车辆物理和障碍运动完全一致。为留出提速空间，两车共同采用 2.4 m/s 的进场目标。时间分支只接收局部车线、观测到的桶及自车运动，对连续观测拟合运动，在单个目标接近直道时比较不同速度下车头进入到车尾离开的可用空间。估计距离和预测时长都计入不确定度。选择形式与门槛由人设计；这次未训练新的网络或改动车端控制。

所选开发示例中，两车到 7.80 s 的物理轨迹和此刻输入均一致。计时分支估计，加速时车尾 1.775 s 后可以清空，较慢方案需要 2.775 s，两者按不确定度扣减后的预计横向余量为 +0.780 m / −0.076 m。实际有先行选项时，从 2.72 m/s 加至 4.45 m/s 入区，车尾在 9.508 s 清空，桶于 10.638 s 进入车穿过的通行带。关闭先行时，以 1.83 m/s 入区，两种策略都通过。裁判区间用时 3.45 / 5.42 s，净距 0.99 / 0.33 m。

## 冻结后四个横移初相位

同一直道与起点，只改横移桶的初相位。只有两组均通过且满足锥距至少 0.15 m、路界至少 0.08 m，才比较 12 m 障碍区用时。以下净距按开放 / 关闭列示。

| 初相位（rad） | 主动先行 | 两组结果 | 净距（m） | 用时（s） |
|---:|:---:|---|---:|---:|
| -4.38 | 否 | 两者通过 | 1.17 / 1.17 | 4.43 / 4.43 |
| -3.54 | 是 | 两者通过 | 0.22 / 0.93 | 2.89 / 7.35 |
| -5.26 | 否 | 两者通过 | 2.52 / 2.52 | 4.45 / 4.45 |
| -1.1 | 否 | 两者碰桶 | 接触 | — |

只有一个留出相位触发先行。该相位用时少 4.46 s，净距也从 0.93 m 缩至 0.22 m。另一个相位两种策略都碰桶。保持原正常巡航上限的另一次对照没有触发先行，两组均通过；这次正例来自有余量的进场条件。额外的两个占道反例在两种模式下也都接触。

[注册协议](phase_protocol.json) · [四个相位审计](held_out.json) · [全部试次](../timing_revision/runs/) · [成对对照与负例](../timing_revision/pairs.json)

用两个实录 JSON 运行 `python -B crossing_audit.py timing_revision/runs/explanatory_go.json timing_revision/runs/explanatory_yield.json --out crossing_evidence/primary.json` 可复算主案例；使用 `--phases timing_revision/runs crossing_evidence/phase_protocol.json --out crossing_evidence/held_out.json` 可复算留出测试。审计使用车身有向矩形与锥桶半径，独立重建车尾通过时刻，并检查三秒内桶是否进入车辆实际走过的横向范围。见 [视频来源](paired-crossing.json)。

从本机归档的原始帧重出这支影片，在实验目录运行：

```powershell
python -B timing_comparison.py --baseline timing_revision/runs/explanatory_yield.json --candidate timing_revision/runs/explanatory_go.json --baseline-frames timing_revision/.work/frames/explanatory_yield --candidate-frames timing_revision/.work/frames/explanatory_go --baseline-a4 timing_revision/.work/camera/explanatory_yield/a4_samples --candidate-a4 timing_revision/.work/camera/explanatory_go/a4_samples --audit crossing_evidence/primary.json --prop 0 --start 5.5 --end 12.5 --out crossing_evidence/paired-crossing.mp4 --poster crossing_evidence/before-entry.jpg
```
