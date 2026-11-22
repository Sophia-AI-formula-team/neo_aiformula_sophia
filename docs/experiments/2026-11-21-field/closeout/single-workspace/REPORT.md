# 2026-11-21 车端工作区整合

所有时间为 JST。正式 ROS 工作区为 `/home/workspace`，源码在 `/home/workspace/src/aiformula`，第三方依赖在 `/home/dependencies`。Concept 的隔离工作区按例外保留，现用入口为 `/home/control/concept_1121_ws`。

| 内容 | 位置 |
| --- | --- |
| OA 算法 | `/home/workspace/src/aiformula/perception/obsticle_avoidence/obsticle_avoidence/oa_2_3.py`，同目录有 OA 2.1 和 2.2 |
| OA 安装入口 | `/home/workspace/install/obsticle_avoidence/lib/obsticle_avoidence/oa_2_3` |
| MapAssist、Allnodes 和 YOLOP 启动文件 | `/home/workspace/src/aiformula/launchers/launch` |
| 终端入口 | `/home/workspace/src/aiformula/launchers/shellscript/vehicle-command` |
| LYA / LYA 2 | `/home/workspace/src/aiformula/control/trajectory_follower` 和 `/home/workspace/src/aiformula/control/lya_2` |
| YOLOP 代码与权重 | `/home/workspace/src/aiformula/perception/road_detector/road_detector` 下对应版本目录 |
| Jetson Python 与安装器 | `/home/dependencies/ai-formula-python` |
| 统一构建入口 | `/home/workspace/src/aiformula/launchers/shellscript/build.sh` |

## 时间线

- 12:58–13:00 SSH 盘点源码、快捷方式、旧工作区与包来源，正常退出一个残留的独立 OA 进程。
- 13:01–13:02 将实际加载的 OA、lane publisher、MapAssist、LYA 和控制接线迁回主工作区，11 个主包构建通过。OA 2.3 的算法和配置沿用冻结版，改动为包内导入和安装入口。
- 13:02–13:04 将 YOLOP 环境放入 dependencies，复核模型哈希。将 A3 V14 的实装 ROS 适配器与实装阈值源码写入正式源码树，替换 Allnodes、MapAssist、LYA、LYA 2、YOLOP 的终端链接，桌面入口沿用同一组命令。
- 13:05 在隔离 ROS 域验证红灯停等、手柄接管与松手重新等待新掩码、OA 清障或故障后归还选点、退出后重新发布车道输入。电机 Python 模块与迁移前实际加载版本逐文件比对一致。
- 13:06–13:07 将四个仿真包归入 `control`，修正标准构建入口与终端初始化。首次构建遇 Foxy 环境变量未设置和 colcon 扫描 Python venv，改正后仿真和启动包构建通过。YOLOP 和 Concept 的安装检查通过。
- 13:08–13:10 OA 路径变更写入本机 Neo Git 提交 `4f075e54ea3053b228e0ca13df76c46b6b2fb024`，基于冻结标签 `field-2026-11-21-oa23`。主工作区各包前缀及五个快捷方式通过校验。
- 13:13 校验旧目录源码及日志归档，删除车上 30 个退役路径。再次扫描只剩主工作区安装与 Concept 例外。测试范围为构建、安装检查和隔离 ROS，车保持静止。
- 13:15 车机网络断开。清理后的目录与 Concept 回读已完成，清理后重复执行的全量快捷方式检查停在包前缀阶段。全部五个快捷方式在清理前已由相同主工作区通过验证。

[删除回执](removed-workspaces.log) · [清理后目录](final-vehicle-layout.log) · [清理后入口](final-vehicle-verification.log) · [隔离 ROS 测试](isolated-verification.log) · [OA 迁移 Git 补丁](oa-standard-source-layout.patch)
