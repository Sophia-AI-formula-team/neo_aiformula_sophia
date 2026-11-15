# inner04 感知输入问题分析（修复前、只读回放）

2026-11-16。结论：**实际运行的是 A4 的四类 FCOS；同一位置可输出 cone 和 light 两个类别，类内 NMS 不会互相消除。不是 cone→light 的 label remap 错误。** 当前接线把任何模型 red 直接送入 semantic，未处理这一身份冲突。原 semantic 已按输入正确执行红灯锁停，不能用修改 semantic 状态机掩盖感知问题。

本次未改 live、A4 模型/weights、semantic、LYA 或 Unity 源码。`replay_perception_identity.py` 只在 audit 里实现两种候选过滤用于离线分析；没有重跑模型。

## 1. 实际运行路径与确定根因

实际 `runs/inner_joint_04/perception_settings.json` 指明 model=`a4`，Python 为 `<existing detector venv>/Scripts/python.exe`。`signal_panel.source=a3_model` 是旧接口沿用的名字，不能据此认定本场运行 A3。

以下相对路径均以实际 Simulator 的 `<Simulator>/Assets/StreamingAssets/RoadDetector` 为根：

| 文件行 | 事实与影响 |
| --- | --- |
| `a4/yolop_a3_runtime/shared_fcos.py:15,63,67` | `LABELS=(1,3,10,91)`，FCOS 有四个类；映射一对一，没有把91重映射成10。 |
| `a4/yolop_a3_runtime/shared_fcos.py:105-109` | 使用 TorchVision FCOS 的 postprocess，之后才做一对一 label_map。 |
| `.venv/Lib/site-packages/torchvision/models/detection/fcos.py:515-545` | 每个 anchor 的各类 sigmoid 分数分别参与 top-k；`anchor_idxs=floor(index/num_classes)`、`labels=index%num_classes`；同 anchor 的不同类使用同一回归框。最后 `batched_nms(..., image_labels, ...)` 是按类 NMS，所以不同类相同框可同时保留。 |
| `a4/yolop_a3_runtime/shared_runtime.py:53-70` | score≥0.5 的模型 detection 分别保留；class91算 cone颜色，class10算 light颜色；并集只要有red便输出red。没有创建或复制 class10 框的接线逻辑。 |
| `a4/yolop_a3_runtime/color_semantics.py:16-27` | 在给定 light crop 内按HSV计色，至少3个颜色像素即可能定色；这个函数不验证物体是灯，因此 cone 上少量红像素可给模型误分类的light框染成red。 |
| `signal_panel_rgb.py:102-108` | 现有RGB发光面候选已有相机/高度/发光面几何身份验证。 |
| `signal_panel_rgb.py:174-188` | 已确认RGB red优先，但模型red直接通过，即使 `panel=None,candidates=[]`；source字符串仍写 `a3_model`。 |
| `worker.py:187-205` | `output.traffic_light_state` 传给上述helper，再将helper state送给C#；原始detections保留。 |

因此 exact same box 与 A4 设计相符，证据支持“类概率冲突 + 当前接线缺少冲突身份判据”。未保存 logits/anchor index，不能声称从日志恢复了具体同一anchor编号；但已经排除当前接线 remap/复制类标签的解释。

运行前记录与当前一致的接线SHA：`signal_panel_rgb.py=aeb7b9c23d019b67fcf795a3a5067d9de55eacb33af9e0220f7095cb7dd61326`，`worker.py=e78fa6d52c64f5ab375c25c31c5bef23b384c9c4ea6bdab43cd2ea650736220b`。本次只读源码快照：A4 `shared_fcos.py=4eb4ad6aa193014f3300f0f2c013b2d5a274c9d73ffea947febe8f098b47ca48`，`shared_runtime.py=b6e9398ea8f0e84ce916d66519001e5ae87b6ec3f393ee25e6d23db4a06d8073`；TorchVision FCOS文件SHA=`d2e3ad5c5e9c39985a49e4c8075aeaade002dd775c0f8597ce4e104cc685f3d0`。运行hash manifest不含这两个A4 runtime文件，不能把当前文件hash说成当时保存的hash。

## 2. 实跑三帧证据

`inner04_behavior.json` 已保存完整检测列表、保存RGB/overlay文件名、时刻、位姿以及只用于评价的面板真值。

| seq | 模型 cone score | 模型 light score | 两类共用 bbox | 模型灯色 |
| ---: | ---: | ---: | --- | --- |
| 674 | 0.81033 | 0.58308 | [256.0917,179.8272,260.7865,188.4041] | red |
| 675 | 0.81694 | 0.50432 | [261.3895,180.9270,266.4345,188.8449] | red |
| 676 | 0.83356 | 0.51278 | [270.0741,180.8081,275.4131,189.2332] | red |

本场 K 中 `cy=181.149643`，R 为保存的相机光学轴→base_link轴，三个框的中心视线 up 分量分别为 -0.0116593、-0.0146871、-0.0152168，均在标定地平线下。现有RGB panel helper未在这些框内找到满足当前几何的发光面。seq675实际overlay在低矮目标处把 cone/light 标签叠在一起；当时被试灯牌已是green。t=298.5900s semantic收到2red/3samples，重新停止；以后unknown不解除已确认red是原包的既有设计。

图片：`runs/inner_joint_04/rgb_00674.jpg` 至 `rgb_00676.jpg`、同号 `overlay_*.png`。这些是保存的真实第一视角，不是生成图。

## 3. 最小候选与适用边界

推荐先用更窄的必要条件候选，作为**默认关闭、仅Joint模式显式启用的感知接线选项**：

1. 仅检查 class10 与更高分的 class91 发生同框/IoU≥0.95 的身份冲突；不要改FCOS或全局跨类NMS。
2. 使用当前帧、当前有效 K/R 和已存在的 RGB panel 候选检查。若有同色、位于冲突框内的当前真实发光面证据，保留此 light。
3. 只有没有上述身份证据、且模型框中心视线在当前标定地平线以下时，移除此候选对**聚合交通灯状态**的贡献。其他独立model red/green原样保留；现有RGB red优先级和确认过程原样保留。
4. 保存原始 `detections`、overlay、原model_state，并另记被拒候选及理由。若校准无效/缺失，不凭无效几何取消原模型红灯。
5. 不向 semantic 传距离，不启用bbox距离模式，不改相机、灯位、semantic投票/锁停、LYA或point TTL。未启用开关时行为必须逐项与旧路径相同。

**不能直接调用 `project_panel_geometry(model_bbox)` 并因任意 invalid 拒灯。** 该函数的高度一致性假定框住0.286m LED发光面；A4的整物体框可能包含支架。尤其 `panel_height_inconsistent` 不是可用于否决任意class10整框的理由。候选只用必要的光学视线方向，并由现有RGB发光面证据覆盖冲突；不采用LED尺寸对模型整框估距。

风险：身份冲突过滤不是识别率保证。若模型把真实灯和cone输出同框、而可见发光面不足且整框中心偏低，仍可能把真实灯暂判unknown。必须记录conflict evidence，保持非冲突模型灯不变，并实跑检查被试真红/真绿。仅按cone分数胜出或全局class-agnostic NMS会扩大抑制范围，本次不建议采用。

## 4. 已录制真图的只读回放结果

`replay_perception_identity.py` 使用全部839个已录制帧、原始detections和现有RGB helper，单进程/单线程 **11.39s**，无模型推理、无GPU、无Unity操作。分别回放旧行为、冲突必须有RGB身份的候选、更窄的“冲突+视线在地平线下+没有RGB身份”候选；后两者在本批输出相同。完整receipt=`perception_identity_replay.json`。

- seq674/675/676：旧行为red，两个候选均unknown；没有其他真实灯被一并移除。
- 物理red阶段，回放baseline的 **61个red全部保留**：57来自模型、4来自RGB panel。
- 物理green阶段，回放baseline的34个RGB panel green全部保留，37个非冲突模型green全部保留。
- 有4个cone共框的模型green变成unknown，明确作为行为变化保留，不能只报告去掉假红。另seq624从cone假red改为同帧仍存在的green。
- 物理green阶段的red从9帧降到1帧。剩余帧包含切灯与图像捕获/消费时差，不能只凭当前面板真值把所有该类帧认定为识别错。

回放限制必须保留：保存RGB是JPEG，非原始推理PNG；重放的HSV候选与现场可能不同，原录制有26个green在回放baseline变unknown，不能把这些差异归因于候选过滤。离线用录制 capture sim time代替helper单调到达时间；这不是semantic投票时序验收。当前候选只说明这些已保存帧中的错误模型red能被拒绝、基线中的实际red被保留，**尚未证明修改后的Unity已能完成2圈或拐入内部**。

## 5. 本场完整功能结果

`inner04_metrics.json` 与 `inner04_behavior.json` 记录：物理1圈、累计308.3987m；odom reference259.5646m闭合；official fusion拒绝 `excluded_gap_too_long`，最大reference不可信段11.7996m，闭合段另2.5942m，原门限2.4m。无mapped official节点、无detour、无左转。t280.0000s确实green_resume并继续行驶，t298.5900s才因上述假red再次锁停。包含t168.4208s渲染调整，属于mixed-render失败基线。修复感知冲突不能自动修复topo数据缺口，下一场应固定30fps重新采集首圈，再分别评价闭合、融合与内部转弯。

## 6. 报告后获准的 staging 接线实现与验证

主管在上述报告完成后批准更窄必要条件候选。仅写 `staging/Assets/StreamingAssets/RoadDetector/`：新增纯math `signal_model_identity.py`，修改 `worker.py` 显式传递布尔参数及 `signal_panel_rgb.py` 默认关闭接入口；live尚未安装，由主管和C#负责人执行SHA guard部署。

参数为 `signal_panel_reject_conflicting_below_horizon_lights`，只有严格 `True` 才启用。启用时的新鲜帧增加 `effective_model_state` 与 `model_identity_filter`；旧 `model_state`、detections和overlay保留。重复帧仍直接返回原stale结果，不重算身份。missing/invalid calibration、非法框不移除模型灯；没有尺寸估距，没有semantic距离字段。

`verify_staged_perception.py` 已通过17项检查，839帧完整回放7.67s：所有默认关闭结果**完整dict含schema逐帧相等**；61个baseline实际红灯结果全部保留；seq674–676错误红灯去除；detections深比较未变；另覆盖同分cone、不重叠cone、地平线上方、同色RGB覆盖、异色RGB、其他独立red/green、缺少校准、非法旋转/坐标与重复帧。证据 `staged_perception_verification.json`。重放JPEG/时钟限制仍适用，尚非部署后Unity验收。

冻结SHA：

- worker.py: `afdb387d74d1b8068cba7b3b0c0d0ecf14ede8a30ce40d978c4bc97bf0d10f7c`
- signal_panel_rgb.py: `38884c9c368691f7a842e61841a88f6e3878e01aa6441c54d237c6b68789e508`
- signal_model_identity.py: `ea9cbfaff2a8b9c8fa104644e77d413180fb4ec99b72e21f95944ace70e41eb4`

原worker/helper原样保存在 `audit/baseline_perception/`，原先没有identity文件。主管C#侧只在Joint trial临时启用并退出恢复，不持久化该选项；旧Neo、bbox试验和其他模式维持默认关闭。

## 7. 部署后 outer01：过滤范围未涵盖较弱cone与单独light误检

本节在进一步修复前记录。`outer_joint_01` 固定30fps，已安装上述冻结接线，仍在首圈行程 **95.72359 m**、t=38.3800s因2red/3samples进入 `red_wait_first_lap`，35s无进展后clean退出。0圈、未建reference、尚未调度测试red。原始第一视角和日志保留，不能把该场称为topo融合失败。

实际触发帧：

| seq | t | class10 red | 同框class91 | identity诊断 |
| ---: | ---: | --- | --- | --- |
| 161 | 37.9419s | score0.509735，bbox[151.9144,179.6427,156.1722,186.0931] | 无；旁边两个cone不与light重合 | enabled=true, conflicts=[], rejected=[] |
| 163 | 38.3756s | score0.601218，bbox[148.4140,179.4521,152.3549,185.6915] | 同框，但cone score0.545238低于light | enabled=true, conflicts=[], rejected=[] |

因此 opt-in 确实生效，局限是设计的“更高分cone”门槛未覆盖这一失败；不是worker/C#遗漏参数。两帧都没有RGB panel候选，模型框中心视线在标定地平线以下。实际overlay在远处低矮物体处出现cone/light重叠标签，没有该处灯牌。seq4、92、112还各有一个无同框cone的孤立red，但未单独满足semantic投票条件。完整数据=`outer01_behavior.json`，指标=`outer01_metrics.json`。

最小候选 A：仅去掉同框cone必须比light分数更高的要求，仍保持IoU≥0.95、校准视线在地平线下、无当前同色RGB panel身份三个条件。这会拒绝seq163、保留seq161，因而打断本次确证的2/3触发；真实灯如果被误分类成cone但RGB发光面在框内，仍应保留。**这不是解决全部单独light误检的保证**，seq161及其它无cone假red仍可通过，重跑必须继续记录。

候选 B（范围更大，尚未实施）：从class10框中的真实当前同色像素判断是否存在满足已知灯高的必要正向up光线，而不是用含支架整框的中心或LED尺寸。它可覆盖无cone的模型假灯，但会改变所有模型灯的接纳范围，必须先拿保存的真红/真绿与假红回放，并清楚报告JPEG与相机/路面姿态假设；不能直接把缺少RGB panel候选等同于“必定不是灯”。

不建议以仿真面板state/位置筛灯、强制green、取消red锁停或放宽投票。进一步实施选择由主管确认；本节写入时未更改已冻结接线。

## 8. 当前颜色像素必要条件：三场只读回放，不能直接替换原冲突判据

主管授权先回放更完整的像素候选。脚本 `replay_pixel_identity.py` 只读327+839+963个已保存帧，总耗时25.43s，单线程、无模型推理/Unity、无live/staging修改。各场receipt分别为 `outer_joint_01_pixel_replay.json`、`inner_joint_04_pixel_replay.json`、`inner_joint_05_pixel_replay.json`。

判据直接采用A4 `color_semantics.py` 的同色像素定义：S≥75、V≥65，red H≤9或≥170，green H38–90；box floor/ceil裁切，最少 `max(3,int(area*.002))` 像素。仅当**所有该色像素连同各自±0.5像素光栅单元的最上方视线也不高于标定地平线**，且无当前同色有效RGB panel身份，才将该模型灯候选排除。没有足够颜色像素、无效相机或跨越地平线均保留模型输入。这里不使用cone是否被检测或其分数，也不使用model整框LED尺寸。已知仿真LED下沿高于相机是适用前提；实车不同安装/坡度需重新验证，不能把此仿真身份必要条件泛化为所有交通灯的通用事实。

| 场次 | 帧数 | baseline实际red保留 | baseline实际green保留 | 关键帧 |
| --- | ---: | ---: | ---: | --- |
| outer01 | 327 | 无计划red段 | 34/34 | seq161的8个red像素全在地平线下，变unknown；seq163 JPEG中0个合格red像素，按保守规则仍red |
| inner04 | 839 | 61/61 | 101/101 | seq675拒绝；seq674/676有地平线边界上方红像素，保留 |
| inner05 | 963 | 58/58 | 85/85 | 实际红灯与绿灯都保留 |

这些分母是**同一JPEG回放的原helper baseline**，不是原PNG模型识别率；物理state只用于事后分组。原始录制与JPEG回放颜色有差异，尤其outer seq163原模型判red但JPEG中没有达到色阈值，不能伪称原模型输入没有红像素。

纯pixel候选能打断outer01实际seq161+163的2/3误触发，但不能替换现有冲突法：原inner04的seq674/676在纯pixel分支仍red，会失去已验证的冲突保护。建议**保留v1已验证的同框cone必要条件，再追加当前像素全部位于地平线下的独立必要条件**；按候选逐个排除，所有其他模型灯与RGB证据保持。此组合仍需单独核验，因为两个分别安全的聚合规则不能未经检查就声称组合永远保真。本节完成时没有实施组合，更没有将任何真值送入控制。

## 9. 报告后实施v2组合，staging验证完成

主管批准保留v1加独立pixel条件。v1三文件先原样保存到 `audit/baseline_perception_v1/`，原pre-v1基线仍在 `audit/baseline_perception/`。仅更新staging三Python文件，未自行安装live。

为了准确反映含义，request和tracker的单一开关改为 **`signal_panel_model_identity_guard`**，默认False，无旧名兼容别名；C#负责人的非持久属性同步为 `EnableModelLightIdentityGuard`。默认关闭时不加任何诊断字段；开启时保留raw `model_state`，加 `effective_model_state` 和v2 `model_identity_filter`，逐灯记录v1冲突、像素数量/门限、包含半像素不确定性的最高up ray、RGB身份及拒绝原因。原detections/overlay不改；不输出控制距离，不改semantic。

`verify_staged_perception_v2.py` 13项边界检查通过，涵盖三个像素/不足三个、半像素格跨horizon、真实颜色在horizon上方、含长支架整框但灯色在上方、当前同色RGB覆盖/异色不覆盖、无效相机/旋转、没有RGB、v1冲突保留、cone同分、独立真实red保留。

再逐场回放实际staging：

| 场次 | 帧数 | 耗时 | 默认off与原helper完整dict/schema | v1拒例保留 | v1 baseline真red / green保留 |
| --- | ---: | ---: | --- | ---: | --- |
| outer01 | 327 | 3.93s | 全部相等 | 5/5 | 无计划red段；35/35 green |
| inner04 | 839 | 10.06s | 全部相等 | 12/12 | 61/61 red；98/98 green |
| inner05 | 963 | 12.28s | 全部相等 | 14/14 | 58/58 red；79/79 green |

合计2129帧、31个v1候选拒例保留、119个red及212个green保留。outer seq161变unknown；seq163在保存JPEG中颜色证据不足，保守保留red，原两票误触发被打断。inner04 seq674–676继续由组合拒绝。没有把其余无法证明的模型灯强行改成green。上述真色仍是“物理状态分组中的同一JPEG回放v1 baseline”，不能替代新Unity验收或解释为保证所有环境真红不漏。

证据：总receipt `staged_perception_v2_verification.json`，每场 `<run>_staged_v2_verification.json` 包含全部变化帧与v2诊断。冻结SHA：

- worker.py: `b515fdb2e0abd7f21555acc09b83632d73a8ca71db3e9dc3c5298d3a8985f4d6`
- signal_panel_rgb.py: `939a59a6d4cb8d9efd19198700ea26afd460dbbf9e641551f14a245a52f40cee`
- signal_model_identity.py: `3e626adb605d55a6818c7473959692f606fd35e26f6d1b7c81d0fd4e62413841`
