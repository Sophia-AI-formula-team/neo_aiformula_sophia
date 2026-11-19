# 2026-11-19 车机活动代码快照与 Neo 对比

[下载压缩包](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/releases/download/field-archive-2026-11-19/2026-11-19-vehicle-active-code.tar.gz) · [SHA-256](2026-11-19-vehicle-active-code.tar.gz.sha256) · [归档核验](archive-verification.json)

车机副本在 `/media/control/OrinSSD2000/2026-11-19 Experiment Logs/active-code/`。本机内置 SSD 和 Extreme SSD 也各保存了一份相同内容的压缩包。包内保留了车道和目标识别、OA、轨迹跟踪、Allnodes、MapAssist 与 Concept 的源码、实际安装的节点、配置、桌面入口、依赖源码，以及 A3/A4/A4Light 启动所用的权重。ROS Foxy、ZED SDK、CUDA 和 Python 虚拟环境作为平台依赖记录在包内清单中。重复的较大文件按硬链接归档。

## 冻结车端运行包 ZIP

[下载完整冻结运行包 ZIP](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/releases/download/field-archive-2026-11-19/2026-11-19-vehicle-frozen-runtime.zip) · [ZIP 的 SHA-256](2026-11-19-vehicle-frozen-runtime.zip.sha256) · [逐项核验结果](frozen-runtime-zip.json) · [GitHub 回读核验](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/experiment/yolop-selection-20261119/docs/experiments/2026-11-19-yolop-selection/records/frozen-runtime-verification.json)

该 ZIP 直接展开为当天车机的 `home/` 文件树及两份原始 manifest。源文件为 17:34 校验的冻结 tar.gz，原包 SHA-256 为 `3ee71ae49b45ea78d676c5ada5b2bc66111051112e32025234d5360fca3a182e`。ZIP 大小为 259,393,710 字节，SHA-256 为 `93fb34a5f07eecbbd53a1f90d6ace9e399210ae591edb943f77c2c501c8bfc9e`。核验覆盖全部 8,486 个路径。其中有 5,321 个普通文件、1,964 个目录、1,194 个符号链接、7 个原 tar 用硬链接去重的文件。ZIP 在这 7 个路径直接保存相同的文件内容，原来的对应关系可在包内 `vehicle-active-code-manifest.json` 查到。Unix 权限、可执行位和符号链接目标均已逐项比对。

部署路径沿用车机绝对路径。复核文件树时可用 Linux 的 `unzip` 解到空目录。恢复运行依赖车机原配的 ROS 2 Foxy、ZED SDK、CUDA 与 Python 环境。A3 V14、A4/A4Light 权重和 Concept H12 选路模型随冻结包保存。Concept 的 B1.4 视觉权重按项目既有方式单独导入。本机已核对原件为 15,293,077 字节，SHA-256 为 `19d6eeed99fee90a46f09761746845afcf330974a8e1aff057055697a02706a6`。在已加载 Concept 工作区的环境中可用以下命令导入团队持有的原件。[部署时的权重校验](../concept-v6275/README.md)

```bash
ros2 run concept_yolop concept_yolop_weights /path/to/B1.4-road-aligned-research.pth
```

## 活动包与 diff

按车机 Allnodes、MapAssist、Concept 三个启动环境实际解析到的安装目录，核对了 60 个包版本及 152 个已安装可执行项。Allnodes 和 MapAssist 各解析到 45 个工作区包，Concept 解析到 57 个。同名包在不同安装层分别列出。

| Neo 基线 | 相同 | 有差异 | 车机独有 | 差异文件数 |
| --- | ---: | ---: | ---: | ---: |
| `main` `d8277bd` | 37 | 11 | 12 | 333 |
| `experiment/yolop-selection-20261119` `dbb95d9` | 38 | 10 | 12 | 344 |
| `feat/concept-v626-live` `a9d7d7a` 的 Concept 相关包 | 2 | 11 | 0 | 49 |
| `experiment/map-assist-auto-warm-detour-20261119` `8f58522` 的 MapAssist 相关包 | 2 | 7 | 0 | 83 |

- [活动包、安装目录和源码来源](neo-diff/active-packages.tsv) · [车机包解析原始清单](vehicle-package-resolution.tsv) · [已安装可执行项](vehicle-ros-executables.tsv)
- [与 main 的包级对比](neo-diff/main-packages.tsv) · [逐文件清单](neo-diff/main-changed-files.tsv) · [按包文本 diff](neo-diff/main/)
- [与今日实验分支的包级对比](neo-diff/experiment-packages.tsv) · [逐文件清单](neo-diff/experiment-changed-files.tsv) · [按包文本 diff](neo-diff/experiment/)
- [Concept 分支对比](neo-diff/concept-packages.tsv) · [MapAssist 专项分支对比](neo-diff/mapassist-experiment-packages.tsv) · [基线完整提交号](neo-diff/baselines.json)
- [MapAssist 与桌面接线 diff](neo-diff/main-runtime-wiring.patch) · [接线来源及路径](neo-diff/main-runtime-wiring.tsv) · [LYA 源码与安装文件的差异](neo-diff/lya-source-vs-installed.patch)

Allnodes 实际使用 `lane_0927oa`。MapAssist 正式入口读取原生 ABC 话题，通过单独的 OA 和 LYA 入口接管和跟踪。MapAssist 规划包按 semantic 安装层比较，Concept 的 drive 和 panel 按当天下午最后生效的覆盖层比较。LYA 的 `lya_follower_connected_omegat_global.py` 在安装目录与源码副本的增益不同，包级 diff 使用实际安装的实现。MapAssist 外部启动脚本在独立接线 diff 中列出。

| 时间（JST） | 操作 |
| --- | --- |
| 17:24–17:27 | 核对车机挂载和安装层，抓取 Neo 基线，生成带实际节点和识别器资产的压缩包。 |
| 17:28–17:32 | 续传到本机，核对 Ament 解析优先级，从整包重建包级与逐文件 diff。 |
| 17:33–17:34 | 补齐 63 份符号链接链条的目标并复核三份包的 SHA-256、源码和节点。 |
