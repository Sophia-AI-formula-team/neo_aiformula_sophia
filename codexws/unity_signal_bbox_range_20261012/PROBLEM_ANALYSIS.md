# 框尺寸距离接线试验 0.1.0

用户要求：当前识别后按框尺寸估距；框尺寸达到设定阈值则判断达到目标距离；太小的框不输出距离，其他功能保持现状。

## 改动前分析

当前 RoadDetector.worker 将 signal_panel_range.estimate_panel_range 的结果传给 signal_range；SemanticPlanning 原样使用 valid、distance_m、uncertainty_m。旧算法使用框中心射线与已知灯牌/相机离地高度差，框高仅作一致性检查。中心接近地平线时，一像素误差可导致明显距离变化。

最小改动：仅替换 worker 中距离函数的导入；新增独立估距模块和配置。旧函数及 RGB 身份识别不改，semantic、LLP/LYA、A3 权重、颜色/时序/避障/急停/相机/灯位/官方图均不改。默认禁用，可通过配置单独启用。

框口径：现有 signal_panel.panel.bbox 是发光面框，对应已知 0.286 m 高度；不可拿包含支架的 A3 大框套用该尺寸。保留原 RGB 身份检查，只有新鲜、连续至少两帧同色的有效框提供距离。小于独立 min_height_px 或 min_width_px 的框返回 valid=false、distance_m=null，不回放旧距离，不退回旧射线估距。

公式：光轴深度 Z = reference_depth_m * reference_height_px / 当前框高；车头纵向距离 = Z + camera_forward_offset - front_offset。达到判定用当前框高 >= target_height_px，目标距离由同一系数反算并输出。固定当前分辨率及 K/R/t，不跨分辨率猜测缩放。参考深度必须从相机光轴计。

标定来源按用户补充直接使用 Unity 和 neo 数据，无需用户另提供实测值。仓库 SN48442725/intrinsic_nHD.yaml 为 fy=254.391622px、640×360；Unity DisplayFace.asset 的 half extent=0.143m，prefab 根及显示面缩放均为1，即发光面高0.286m。10m光轴深度时理论高7.2756003892px。最小宽高4px；到达阈值选8px，对应车头纵向8.8785004865m。若将8.5m转换为8.3474px并对整数框直接比较，会到9px才触发（约7.868m），因此选择8px更符合已选的8–9m范围。这是投影预期，不是实跑停车验收。配置可修改；K允许2e-5的Unity float32序列化舍入差异，R/t仍严格检查。

## 不能隐瞒的接线限制

1. 现有 semantic 将红灯但 distance 缺失视作故障停车。本试验保留此行为；小框无距离不代表自动忽略小红框继续行驶。
2. target_reached 是距离输出内的观测标志，原 semantic 不消费它。制动仍使用原 distance/uncertainty/age/speed envelope，可能早于几何框阈值停下。不能以此宣称已实现精确8–9m停车，也不能把不确定性填0来强迫延后制动。
3. 框高正负1px区间仅描述两端各半像素量化，不包括识别边界误差、姿态、遮挡和尺寸标定误差。距离是近似纵向深度，不是斜距。禁止将该区间宣称为完整误差上界。
4. RGB识别保留旧射线身份检查，因此该试验只替换距离来源，不扩大识别集合，也不修复路口匹配/左转。

## 验证边界

初次准备时共享Unity/CPU由Lane对话持有，先静态实现。随后取得仅18项标准库单测窗口：2026-10-12 14:45 JST全部通过，保留完整回执，已归还CPU。安装器往返、实际安装及MCP尚未验证，不能宣称实际停车或左转成功。
