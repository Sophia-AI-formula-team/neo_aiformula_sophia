# 四点短单侧接线独立审查

审查者只读 `INNER06_LANE_FAILURE_ANALYSIS.md`、live原adapter、staging候选及原生 `lane_0215.py` / `withoutkalman_0312.py`。没有修改SIM agent文件，没有执行重复模型、回放或大测试。当前结论限接线审查，不能代替两圈Unity实跑。

## 已核对的边界

- 失败现场确为当前left投影4点、x=3.0/3.5/4.0/4.5m，原选择器不支持；traffic已normal/green_resume，未出现另一次红灯锁停。
- 新fallback放在全部原normal、双侧center、5+单侧选法之后；默认`allow_short_observed_side=False`，没有降低原`observed_triplet`全局门限。
- 候选严格4×3、finite、forward、有序、span≥1.5m，保留±4m横向范围、left正/right负和相邻heading差≤15°。选(0,1,3)来自本帧已有cloud；不补第5点、不延长line、不借历史ABC。
- 已建议并在staging看到相邻x间距必须为现有projector的0.5m（容差1e-6）。单纯有序+总span1.5m会放过局部接近零的dx，现补充契约避免该病态输入。
- 当前`use_current_corridor`分支不消费新fallback，原LYA/point classes、宽度补偿、epoch/frame/TTL与传输identity未改。新模式的准入长度变化仍属于实质行为扩展。

## 必须用原生输出验证，不能仅看supported/finite

原`lane_0215.py`左侧对A/B/C分别减`2.6/1.5`、`2.6/1.4`、`2.6/1.3`。新(0,1,3)点的相邻x距离为0.5m与1.0m，所以即使原单侧cloud为直线，补偿后的两段heading也会不同。原`withoutkalman_0312.py`还把两heading差除以0.1生成omega_t。

因此“没有修改原LYA源码”不等于选点响应平滑。需要真实seq667→原lane-point callbacks→原target→原LYA的target/ABC、theta1/theta2、omega_t和v/w，与最后有效seq666及正常单侧5+点行为对比。该变化可能是预期的弯道恢复，也可能显著改变瞬间转向；应记录实际量再进行闭环验证。没有建议改原补偿或LYA。

## 潜在双侧矛盾旁路（已反馈给实施者）

初版新fallback逐侧尝试、固定left优先。如果左右都各有合格4点，但paired center因宽度不稳定/两侧heading矛盾被拒，fallback会清空right并按left继续，绕过刚才的paired质量拒绝。实际失败[4,0,0]不需要这种更宽授权。

建议只在**恰好一个侧线通过新四点契约**时接纳；如果两侧都通过但正常center未成立，保持unsupported并明确矛盾原因。至少需要这一负例的窄测试/设计说明；不改变旧5+单侧行为。审查时该点尚待SIM agent回复。

## 待核实的交付证据

默认off真实失败帧/正常帧输出保持，正常选法优先，坏点数/NaN/乱序/小span/极小dx/错误侧/矛盾双侧拒绝；真实seq667原生LYA量及C#默认False、Joint启用/退出恢复的receipt。收到后补充本审查。没有这些或实际新场结果前，不能宣称第二圈已修复。

## 收到回放后的结论：不放行该候选

已只读核对SIM agent的 `short_observed_side_verification.json` 和执行脚本，未重复执行。0.91s的检查中，827帧默认选择与原版相等，真实seq667默认仍unsupported；包括坏点/错侧/小段/顺序/正常center优先等14项通过。这些是有用的接线证据，**不足以证明新选点的控制含义保持**。

实际原callback→原target→原LYA证实了前述静态风险：

| 项目 | seq666 最后原有有效center | seq667 新四点left-only |
| --- | ---: | ---: |
| theta1 | 0.222809091 | 0.118751023 |
| theta2 | 0.222809148 | 0.220433623 |
| omega_t | 0.000000567 | **1.016826003** |
| v | 2.243534965 | 2.247368837 |
| w | 0.831470145 | 0.999040952 |

raw seq667 left近乎直线，曲率差来自不同A/B/C侧偏补偿与非等距选点组合；新native w相对seq666增加0.167570807。不能把“代码没动、输出有限”表述为沿用相同控制行为。即使某次车体能走过，也不能用一次成功掩盖这个未解释的人工曲率。

按主管对候选的约束，SIM agent已明确不安装live、不加入部署manifest。本审查同意：**该四点候选暂不放行**，保持原LYA和semantic不改；也不继续叠加旁路guard令它看似通过。原真实失败、此候选与回放receipt应保留，用于后续在当前mask中恢复原≥2m支撑、或单独论证新选点几何契约。上述双侧矛盾旁路仍是额外未解决项，尚未实施修正。

这不是已经修好的第二圈，也不构成允许改原固定补偿/LYA/降低速度的授权。
