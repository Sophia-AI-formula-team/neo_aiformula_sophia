# R29 冻结清单纠正回执

状态：ACCEPTED_FOR_ANALYSIS，仅纠正记录，不是 runtime/两圈/上车放行。

唯一目的地：Sophia-AI-formula-team/neo_aiformula_sophia，ID1303517209。
输入代码8fcd4c0533a13d95400b9f377959733ed798f744，执行时文档HEAD
f8a2732a2763e97382f430fe2998808198e8b864，R29使用dirty且逐文件hash绑定的源码。

## 不修改原件的纠正

- 原r29_full_regression_01冻结PENDING/APPROVED的source_count=196错误；
  实际snapshot.files有197项，执行器按实际keys遍历，197项前后完全一致。
  原196项中193不变、3指定修改，加1新测试，没有删除或缺少pin。
- 原107测试文件全部保留，加1形成108显式文件，但四对相邻argv排序改变：
  corner_direction/integration、mask_localization/replay、native_current_union/dispatch、
  quantized_ribbon/margin。没有声称顺序字节不变；文化相关Sort只是可能原因。
- 110s是outer主wait超时，不是严格WCET；cleanup/taskkill及后续wait未独立限时。
  outer wrapper未纳入manifest snapshot pin，只有独立记录的hash。
- APPROVED中staged-scope diff措辞指预期修改范围，执行开始前并未stage；
  根已完整读候选、测试移植和tracked diff，发布前仍须另外读完整cached diff。
  driver_AST_valid=null是静态准备未执行Python的状态，不伪造准备AST验证。

原PENDING SHA f5c6b065bf00c43d91ab940c2cc249a4de4e845a953c3881a4dbc9e1e2e025f4，
APPROVED SHA2b201436ea79a97c6241a027eae4170d8b4971ea1deb6d165b4866c00a1f48e9。
原清单、driver、plan及run证据不覆写；独立静态REVIEW hash
dc04c18a3f5481fb2f0a80b378515856e4ef0db550000ee6109697c7d060ac9e。

## 实际执行

2026-10-11 20:43:30–20:45:02 UTC，单次执行，root92.0178302s/exit0。
4304通过、0失败/error、1原Linux限定skip；新增索引25/25通过，108文件。
native9身份/真实调用通过，installed_prefix_verified=false，不是colcon安装验证。
197项前后snapshot SHA6bc7afe4de4f0cf69996c4759b604237102feed548f258c9a5b3a0aabecc2782。
本批Python子进程0确认后明确归还CPU，未操作Unity。详见
[session025公开摘要](../sessions/2026-10-12-planner-indexed-faces-r29-025/evidence/PUBLIC_FULL_REGRESSION.md)。

本回执不把offline单位回归变成几何修复、111帧runtime、首圈/第二圈或视频证明。
此前的严格超时、历史依赖50/2、驱动SyntaxError、准备拒绝、R28混合CI全部保留。
生产门限、车、仿真、相机、LYA、运动源和因果顺序未改。
